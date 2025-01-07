/*
  machine_limits.c - code pertaining to limit-switches and performing the homing cycle

  Part of grblHAL

  Copyright (c) 2017-2025 Terje Io
  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
*/

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "hal.h"
#include "nuts_bolts.h"
#include "protocol.h"
#include "motion_control.h"
#include "machine_limits.h"
#include "tool_change.h"
#include "state_machine.h"
#ifdef KINEMATICS_API
#include "kinematics.h"
#endif

#include "config.h"

static coord_data_t homing_pulloff = {0};

// Merge (bitwise or) all limit switch inputs.
ISR_CODE axes_signals_t ISR_FUNC(limit_signals_merge)(limit_signals_t signals)
{
    axes_signals_t state;

    state.mask = signals.min.mask | signals.min2.mask | signals.max.mask | signals.max2.mask;

    return state;
}

// Merge (bitwise or) home switch inputs (typically acquired from limits.min and limits.min2).
ISR_CODE static axes_signals_t ISR_FUNC(homing_signals_select)(home_signals_t signals, axes_signals_t auto_square, squaring_mode_t mode)
{
    axes_signals_t state;

    switch(mode) {

        case SquaringMode_A:
            signals.a.mask &= ~auto_square.mask;
            break;

        case SquaringMode_B:
            signals.b.mask &= ~auto_square.mask;
            break;

        default:
            break;
    }

    state.mask = signals.a.mask | signals.b.mask;

    return state;
}

// This is the Limit Pin Change Interrupt, which handles the hard limit feature. A bouncing
// limit switch can cause a lot of problems, like false readings and multiple interrupt calls.
// If a switch is triggered at all, something bad has happened and treat it as such, regardless
// if a limit switch is being disengaged. It's impossible to reliably tell the state of a
// bouncing pin because the microcontroller does not retain any state information when
// detecting a pin change. If we poll the pins in the ISR, you can miss the correct reading if the
// switch is bouncing.
ISR_CODE void ISR_FUNC(limit_interrupt_handler)(limit_signals_t state) // DEFAULT: Limit pin change interrupt process.
{
    // Ignore limit switches if already in an alarm state or in-process of executing an alarm.
    // When in the alarm state, grblHAL should have been reset or will force a reset, so any pending
    // moves in the planner and stream input buffers are all cleared and newly sent blocks will be
    // locked out until a homing cycle or a kill lock command. Allows the user to disable the hard
    // limit setting if their limits are constantly triggering after a reset and move their axes.

#if N_AXIS > 3
    if((limit_signals_merge(state).value & sys.hard_limits.mask) == 0)
        return;
#endif

    memcpy(&sys.last_event.limits, &state, sizeof(limit_signals_t));

    if (!(state_get() & (STATE_ALARM|STATE_ESTOP)) && !sys.rt_exec_alarm) {

      #if HARD_LIMIT_FORCE_STATE_CHECK
        // Check limit pin state.
        if (limit_signals_merge(state).value) {
            mc_reset(); // Initiate system kill.
            system_set_exec_alarm(Alarm_HardLimit); // Indicate hard limit critical event
        }
      #else
        mc_reset(); // Initiate system kill.
        system_set_exec_alarm(Alarm_HardLimit); // Indicate hard limit critical event
      #endif
    }
}

// Establish work envelope for homed axes, used by soft limits and jog limits handling.
// When hard limits are enabled pulloff distance is subtracted to avoid triggering limit switches.
void limits_set_work_envelope (void)
{
    uint_fast8_t idx = N_AXIS;

    do {
        if(sys.homed.mask & bit(--idx)) {

            float pulloff = settings.limits.flags.hard_enabled && bit_istrue(sys.homing.mask, bit(idx)) ? homing_pulloff.values[idx] : 0.0f;

            if(settings.homing.flags.force_set_origin) {
                if(bit_isfalse(settings.homing.dir_mask.value, bit(idx))) {
                    sys.work_envelope.min.values[idx] = settings.axis[idx].max_travel + pulloff;
                    sys.work_envelope.max.values[idx] = 0.0f;
                } else {
                    sys.work_envelope.min.values[idx] = 0.0f;
                    sys.work_envelope.max.values[idx] = - (settings.axis[idx].max_travel + pulloff);
                }
            } else {
                sys.work_envelope.min.values[idx] = settings.axis[idx].max_travel + pulloff;
                sys.work_envelope.max.values[idx] = - pulloff;
            }
        } else
            sys.work_envelope.min.values[idx] = sys.work_envelope.max.values[idx] = 0.0f;
    } while(idx);
}

#ifndef KINEMATICS_API

// Set machine positions for homed limit switches. Don't update non-homed axes.
// NOTE: settings.max_travel[] is stored as a negative value.
void limits_set_machine_positions (axes_signals_t cycle, bool add_pulloff)
{
    uint_fast8_t idx = N_AXIS;

    if(settings.homing.flags.force_set_origin) {
        do {
            if(cycle.mask & bit(--idx)) {
                sys.position[idx] = 0;
                sys.home_position[idx] = 0.0f;
            }
        } while(idx);
    } else do {
        if(cycle.mask & bit(--idx)) {
            sys.home_position[idx] = bit_istrue(settings.homing.dir_mask.value, bit(idx))
                                      ? settings.axis[idx].max_travel + (add_pulloff ? homing_pulloff.values[idx] : -0.0f)
                                      : -(add_pulloff ? homing_pulloff.values[idx] : -0.0f);
            sys.position[idx] = lroundf(sys.home_position[idx] * settings.axis[idx].steps_per_mm);
        }
    } while(idx);
}

#endif

// Set, get homing pulloff
coord_data_t *limits_homing_pulloff (coord_data_t *distance)
{
    if(distance)
        memcpy(&homing_pulloff, distance, sizeof(coord_data_t));

    return &homing_pulloff;
}

// Pulls off axes from asserted homing switches before homing starts.
// For now only for auto squared axes.
static bool limits_pull_off (axes_signals_t axis, coord_data_t *distance, float scaling)
{
    uint_fast8_t n_axis = 0, idx = N_AXIS;
    coord_data_t target = {0};
    plan_line_data_t plan_data;

    plan_data_init(&plan_data);
    plan_data.condition.system_motion = On;
    plan_data.condition.no_feed_override = On;
    plan_data.line_number = DEFAULT_HOMING_CYCLE_LINE_NUMBER;

    system_convert_array_steps_to_mpos(target.values, sys.position);

    do {
        idx--;
        if(bit_istrue(axis.mask, bit(idx))) {
            n_axis++;
            if(bit_istrue(settings.homing.dir_mask.value, bit(idx)))
                target.values[idx] += distance->values[idx] * scaling;
            else
                target.values[idx] -= distance->values[idx] * scaling;
        }
    } while(idx);

    plan_data.feed_rate = settings.axis[0].homing_seek_rate * sqrtf(n_axis); // Adjust so individual axes all move at pull-off rate.
    plan_data.condition.coolant = gc_state.modal.coolant;

#ifdef KINEMATICS_API
    coord_data_t k_target;
    plan_buffer_line(kinematics.transform_from_cartesian(k_target.values, target.values), &plan_data);    // Bypass mc_line(). Directly plan homing motion.;
#else
    plan_buffer_line(target.values, &plan_data);    // Bypass mc_line(). Directly plan homing motion.
#endif

    sys.step_control.flags = 0;                 // Clear existing flags and
    sys.step_control.execute_sys_motion = On;   // set to execute homing motion.
    sys.homing_axis_lock.mask = axis.mask;

    st_prep_buffer();   // Prep and fill segment buffer from newly planned block.
    st_wake_up();       // Initiate motion.

    while(true) {

        st_prep_buffer(); // Check and prep segment buffer.

        // Exit routines: No time to run protocol_execute_realtime() in this loop.
        if (sys.rt_exec_state & (EXEC_SAFETY_DOOR | EXEC_RESET | EXEC_CYCLE_COMPLETE)) {

            uint_fast16_t rt_exec = sys.rt_exec_state;

            // Homing failure condition: Reset issued during cycle.
            if (rt_exec & EXEC_RESET)
                system_set_exec_alarm(Alarm_HomingFailReset);

            // Homing failure condition: Safety door was opened.
            if (rt_exec & EXEC_SAFETY_DOOR)
                system_set_exec_alarm(Alarm_HomingFailDoor);

            // Homing failure condition: Homing switch(es) still engaged after pull-off motion
            if (homing_signals_select(hal.homing.get_state(), (axes_signals_t){0}, SquaringMode_Both).mask & axis.mask)
                system_set_exec_alarm(Alarm_FailPulloff);

            if (sys.rt_exec_alarm) {
                mc_reset(); // Stop motors, if they are running.
                protocol_execute_realtime();
                return false;
            } else {
                // Pull-off motion complete. Disable CYCLE_STOP from executing.
                system_clear_exec_state_flag(EXEC_CYCLE_COMPLETE);
                break;
            }
        }

        grbl.on_execute_realtime(STATE_HOMING);
    }

    st_reset(); // Immediately force kill steppers and reset step segment buffer.

    sys.step_control.flags = 0; // Return step control to normal operation.

    return true; // Note: failure is returned above if move fails.
}

// Homes the specified cycle axes, sets the machine position, and performs a pull-off motion after
// completing. Homing is a special motion case, which involves rapid uncontrolled stops to locate
// the trigger point of the limit switches. The rapid stops are handled by a system level axis lock
// mask, which prevents the stepper algorithm from executing step pulses. Homing motions typically
// circumvent the processes for executing motions in normal operation.
// NOTE: Only the abort realtime command can interrupt this process.
static bool homing_cycle (axes_signals_t cycle, axes_signals_t auto_square)
{
    if (ABORTED) // Block if system reset has been issued.
        return false;

    int32_t initial_trigger_position = 0, autosquare_fail_distance = 0;
    uint_fast8_t n_cycle = (2 * settings.homing.locate_cycles + 1);
    uint_fast8_t step_pin[N_AXIS], n_active_axis, dual_motor_axis = 0;
    bool autosquare_check = false;
    float max_travel = 0.0f, homing_rate;
    homing_mode_t mode = HomingMode_Seek;
    axes_signals_t axislock, homing_state;
    home_signals_t signals_state;
    squaring_mode_t squaring_mode = SquaringMode_Both;
    coord_data_t distance, target;
    plan_line_data_t plan_data;
    rt_exec_t rt_exec, rt_exec_states = EXEC_SAFETY_DOOR|EXEC_RESET|EXEC_CYCLE_COMPLETE;

    // Initialize plan data struct for homing motion.

    plan_data_init(&plan_data);
    plan_data.condition.system_motion = On;
    plan_data.condition.no_feed_override = On;
    plan_data.line_number = DEFAULT_HOMING_CYCLE_LINE_NUMBER;
    plan_data.condition.coolant = gc_state.modal.coolant;

    uint_fast8_t idx = N_AXIS;
    do {
        idx--;
        // Initialize step pin masks
#ifdef KINEMATICS_API
        step_pin[idx] = kinematics.limits_get_axis_mask(idx);
#else
        step_pin[idx] = bit(idx);
#endif
        // Set target based on max_travel setting. Ensure homing switches engaged with search scalar.
        // NOTE: settings.axis[].max_travel is stored as a negative value.
        if(bit_istrue(cycle.mask, bit(idx))) {
#if N_AXIS > 3
            if(bit_istrue(settings.steppers.is_rotary.mask, bit(idx)))
                distance.values[idx] = (settings.axis[idx].max_travel < -0.0f ? settings.axis[idx].max_travel : -360.0f) * (-HOMING_AXIS_SEARCH_SCALAR);
            else
#endif
            distance.values[idx] = settings.axis[idx].max_travel * (-HOMING_AXIS_SEARCH_SCALAR);

            max_travel = max(max_travel, distance.values[idx]);

            if(bit_istrue(auto_square.mask, bit(idx)))
                dual_motor_axis = idx;
        }
    } while(idx);

    if(max_travel == 0.0f)
        return true;

    if((homing_rate = hal.homing.get_feedrate(cycle, HomingMode_Seek)) == 0.0f)
        return false;

    if(auto_square.mask) {
        float fail_distance = (-settings.homing.dual_axis.fail_length_percent / 100.0f) * settings.axis[dual_motor_axis].max_travel;
        fail_distance = min(fail_distance, settings.homing.dual_axis.fail_distance_max);
        fail_distance = max(fail_distance, settings.homing.dual_axis.fail_distance_min);
        autosquare_fail_distance = truncf(fail_distance * settings.axis[dual_motor_axis].steps_per_mm);
    }

    if(settings.status_report.when_homing)
        rt_exec_states |= EXEC_STATUS_REPORT;

    // Set search mode with approach at seek rate to quickly engage the specified cycle.mask limit switches.
    do {

        // Initialize and declare variables needed for homing routine.
        system_convert_array_steps_to_mpos(target.values, sys.position);
        axislock = (axes_signals_t){0};
        n_active_axis = 0;

        idx = N_AXIS;
        do {
            // Set target location for active axes and setup computation for homing rate.
            if (bit_istrue(cycle.mask, bit(--idx))) {
                n_active_axis++;

#ifdef KINEMATICS_API
                kinematics.limits_set_target_pos(idx);
#else
                sys.position[idx] = 0;
#endif
                // Set target direction based on cycle mask and homing cycle approach state.
                if (bit_istrue(settings.homing.dir_mask.value, bit(idx)))
                    target.values[idx] = mode == HomingMode_Pulloff ? distance.values[idx] : - distance.values[idx];
                else
                    target.values[idx] = mode == HomingMode_Pulloff ? - distance.values[idx] : distance.values[idx];

                // Apply axislock to the step port pins active in this cycle.
                axislock.mask |= step_pin[idx];
            }
        } while(idx);

#ifdef KINEMATICS_API
        if(kinematics.homing_cycle_get_feedrate)
            homing_rate = kinematics.homing_cycle_get_feedrate(cycle, homing_rate, mode);
#endif

        if(grbl.on_homing_rate_set)
            grbl.on_homing_rate_set(cycle, homing_rate, mode);

        homing_rate *= sqrtf(n_active_axis); // [sqrt(N_AXIS)] Adjust so individual axes all move at homing rate.

        // Perform homing cycle. Planner buffer should be empty, as required to initiate the homing cycle.
        plan_data.feed_rate = homing_rate;      // Set current homing rate.
        sys.homing_axis_lock.mask = axislock.mask;

#ifdef KINEMATICS_API
        coord_data_t k_target;
        plan_buffer_line(kinematics.transform_from_cartesian(k_target.values, target.values), &plan_data);    // Bypass mc_line(). Directly plan homing motion.;
#else
        plan_buffer_line(target.values, &plan_data);    // Bypass mc_line(). Directly plan homing motion.
#endif

        sys.step_control.flags = 0;
        sys.step_control.execute_sys_motion = On; // Set to execute homing motion and clear existing flags.
        st_prep_buffer();   // Prep and fill segment buffer from newly planned block.
        st_wake_up();       // Initiate motion

        do {

            if (mode != HomingMode_Pulloff) {

                // Check homing switches state. Lock out cycle axes when they change.
                homing_state = homing_signals_select(signals_state = hal.homing.get_state(), auto_square, squaring_mode);

                // Auto squaring check
                if((homing_state.mask & auto_square.mask) && squaring_mode == SquaringMode_Both) {
                    if((autosquare_check = (signals_state.a.mask & auto_square.mask) != (signals_state.b.mask & auto_square.mask))) {
                        initial_trigger_position = sys.position[dual_motor_axis];
                        homing_state.mask &= ~auto_square.mask;
                        squaring_mode = (signals_state.a.mask & auto_square.mask) ? SquaringMode_A : SquaringMode_B;
                        hal.stepper.disable_motors(auto_square, squaring_mode);
                    }
                }

                idx = N_AXIS;
                do {
                    idx--;
                    if ((axislock.mask & step_pin[idx]) && (homing_state.mask & bit(idx))) {
#ifdef KINEMATICS_API
                        axislock.mask &= ~kinematics.limits_get_axis_mask(idx);
#else
                        axislock.mask &= ~bit(idx);
#endif
                        if(idx == dual_motor_axis)
                            autosquare_check = false;
                    }
                } while(idx);

                sys.homing_axis_lock.mask = axislock.mask;

                if (autosquare_check && abs(initial_trigger_position - sys.position[dual_motor_axis]) > autosquare_fail_distance) {
                    system_set_exec_alarm(Alarm_HomingFailAutoSquaringApproach);
                    mc_reset();
                    protocol_execute_realtime();
                    return false;
                }
            }

            st_prep_buffer(); // Check and prep segment buffer.

            // Exit routines: No time to run protocol_execute_realtime() in this loop.
            if((rt_exec = (sys.rt_exec_state & rt_exec_states))) {

                if(rt_exec == EXEC_STATUS_REPORT) {
                    system_clear_exec_state_flag(EXEC_STATUS_REPORT);
                    report_realtime_status();
                } else {

                    // Homing failure condition: Reset issued during cycle.
                    if (rt_exec & EXEC_RESET)
                        system_set_exec_alarm(Alarm_HomingFailReset);

                    // Homing failure condition: Safety door was opened.
                    if (rt_exec & EXEC_SAFETY_DOOR)
                        system_set_exec_alarm(Alarm_HomingFailDoor);

                    hal.delay_ms(2, NULL);

                    // Homing failure condition: Homing switch(es) still engaged after pull-off motion
                    if (mode == HomingMode_Pulloff && (homing_signals_select(hal.homing.get_state(), (axes_signals_t){0}, SquaringMode_Both).mask & cycle.mask))
                        system_set_exec_alarm(Alarm_FailPulloff);

                    // Homing failure condition: Limit switch not found during approach.
                    if (mode != HomingMode_Pulloff && (rt_exec & EXEC_CYCLE_COMPLETE))
                        system_set_exec_alarm(Alarm_HomingFailApproach);

                    if (sys.rt_exec_alarm) {
                        mc_reset(); // Stop motors, if they are running.
                        protocol_execute_realtime();
                        return false;
                    } else {
                        // Pull-off motion complete. Disable CYCLE_STOP from executing.
                        system_clear_exec_state_flag(EXEC_CYCLE_COMPLETE);
                        break;
                    }
                }
            }

            grbl.on_execute_realtime(STATE_HOMING);

        } while (axislock.mask & AXES_BITMASK);

        st_reset(); // Immediately force kill steppers and reset step segment buffer.
        hal.delay_ms(settings.homing.debounce_delay, NULL); // Delay to allow transient dynamics to dissipate.

        // Reverse direction and reset homing rate for cycle(s).
        mode = mode == HomingMode_Pulloff ? HomingMode_Locate : HomingMode_Pulloff;
        homing_rate = hal.homing.get_feedrate(cycle, mode);

        // After first cycle, homing enters locating phase. Shorten search to pull-off distance.
        idx = N_AXIS;
        do {
            // Only one initial pass for auto squared axis when both motors are active
            //if(mode == SquaringMode_Both && auto_square.mask)
            //    cycle.mask &= ~auto_square.mask;
            if(bit_istrue(cycle.mask, bit(--idx)))
                distance.values[idx] = homing_pulloff.values[idx] * (mode == HomingMode_Locate ? HOMING_AXIS_LOCATE_SCALAR : 1.0f);
        } while(idx);

        if(auto_square.mask) {
            autosquare_check = false;
            squaring_mode = SquaringMode_Both;
            hal.stepper.disable_motors((axes_signals_t){0}, SquaringMode_Both);
        }

    } while (homing_rate > 0.0f && cycle.mask && n_cycle-- > 0);

    // Pull off B motor to compensate for switch inaccuracy when configured.
    if(auto_square.mask && settings.axis[dual_motor_axis].dual_axis_offset != 0.0f) {
        hal.stepper.disable_motors(auto_square, settings.axis[dual_motor_axis].dual_axis_offset < 0.0f ? SquaringMode_B : SquaringMode_A);
        distance.values[dual_motor_axis] = fabs(settings.axis[dual_motor_axis].dual_axis_offset);
        if(!limits_pull_off(auto_square, &distance, 1.0f))
            return false;
        hal.stepper.disable_motors((axes_signals_t){0}, SquaringMode_Both);
    }

    // The active cycle axes should now be homed and machine limits have been located. By
    // default, grblHAL defines machine space as all negative, as do most CNCs. Since limit switches
    // can be on either side of an axes, check and set axes machine zero appropriately. Also,
    // set up pull-off maneuver from axes limit switches that have been homed. This provides
    // some initial clearance off the switches and should also help prevent them from falsely
    // triggering when hard limits are enabled or when more than one axes shares a limit pin.
#ifdef KINEMATICS_API
    kinematics.limits_set_machine_positions(cycle);
#else
    limits_set_machine_positions(cycle, true);
#endif

#if ENABLE_BACKLASH_COMPENSATION
    mc_backlash_init(cycle);
#endif
    sys.step_control.flags = 0; // Return step control to normal operation.
    sys.homed.mask |= cycle.mask;

    return true;
}

// Perform homing cycle(s) according to configuration.
// NOTE: only one auto squared axis can be homed at a time.
status_code_t limits_go_home (axes_signals_t cycle)
{
    axes_signals_t auto_square = {0}, auto_squared = {0};

    if(settings.homing.flags.per_axis_feedrates && bit_count(cycle.mask) > 1) {

        uint_fast8_t idx = 0, axis0 = 255;
        axes_signals_t _cycle = cycle;

        while(_cycle.mask) {
            if(_cycle.mask & 1) {
                if(axis0 == 255)
                    axis0 = idx;
                else if(settings.axis[axis0].homing_feed_rate != settings.axis[idx].homing_feed_rate ||
                         settings.axis[axis0].homing_seek_rate != settings.axis[idx].homing_seek_rate) {
                    axis0 = 254;
                    break;
                }
            }
            idx++;
            _cycle.mask >>= 1;
        }

        // If axes in cycle has different feed rates home them separately
        if(axis0 == 254) {

            status_code_t status = Status_OK;

            idx = 0;
            while(cycle.mask) {
                if(cycle.mask & 1) {
                    _cycle.mask = bit(idx);
                    if((status = limits_go_home(_cycle)) != Status_OK)
                        break;
                }
                idx++;
                cycle.mask >>= 1;
            }

            return status;
        }
    }

    hal.limits.enable(settings.limits.flags.hard_enabled, cycle); // Disable hard limits pin change register for cycle duration

    if(hal.stepper.get_ganged)
        auto_squared = hal.stepper.get_ganged(true);

    auto_squared.mask &= cycle.mask;

    if(auto_squared.mask) {

        if(!hal.stepper.disable_motors)
            return Status_IllegalHomingConfiguration; // Bad driver! - should not happen.

        auto_square.x = On;
        while(!(auto_squared.mask & auto_square.mask))
            auto_square.mask <<= 1;

        if(auto_squared.mask != auto_square.mask)
            return Status_IllegalHomingConfiguration; // Attempt at squaring more than one auto squared axis at the same time.

        if((auto_squared.mask & homing_signals_select(hal.homing.get_state(), (axes_signals_t){0}, SquaringMode_Both).mask) && !limits_pull_off(auto_square, &homing_pulloff, HOMING_AXIS_LOCATE_SCALAR))
            return Status_LimitsEngaged; // Auto squaring with limit switch asserted is not allowed.
    }

    return grbl.home_machine(cycle, auto_square) ? Status_OK : Status_Unhandled;
}

// Performs a soft limit check. Called from mc_line() only. Assumes the machine has been homed,
// the workspace volume is in all negative space, and the system is in normal operation.
// NOTE: Also used by jogging to block travel outside soft-limit volume.
void limits_soft_check (float *target, planner_cond_t condition)
{
#ifdef KINEMATICS_API
    if(condition.target_validated ? !condition.target_valid : !grbl.check_travel_limits(target, sys.soft_limits, false)) {
#else
    if(condition.target_validated ? !condition.target_valid : !grbl.check_travel_limits(target, sys.soft_limits, true)) {
#endif

        sys.flags.soft_limit = On;
        // Force feed hold if cycle is active. All buffered blocks are guaranteed to be within
        // workspace volume so just come to a controlled stop so position is not lost. When complete
        // enter alarm mode.
        if(state_get() == STATE_CYCLE) {
            system_set_exec_state_flag(EXEC_FEED_HOLD);
            do {
                if(!protocol_execute_realtime())
                    return; // aborted!
            } while(state_get() != STATE_IDLE);
        }
        mc_reset(); // Issue system reset and ensure spindle and coolant are shutdown.
        system_set_exec_alarm(Alarm_SoftLimit); // Indicate soft limit critical event
        protocol_execute_realtime(); // Execute to enter critical event loop and system abort
    }
}

// Set axes to be homed from settings.
void limits_set_homing_axes (void)
{
    uint_fast8_t idx = N_AXIS;

    sys.homing.mask = 0;

    do {
        sys.homing.mask |= settings.homing.cycle[--idx].mask;
    } while(idx);

    sys.homed.mask &= sys.homing.mask;
}

// Check if homing is required.
bool limits_homing_required (void)
{
    return settings.homing.flags.enabled && settings.homing.flags.init_lock &&
            (sys.cold_start || !settings.homing.flags.override_locks) &&
              sys.homing.mask && (sys.homing.mask & sys.homed.mask) != sys.homing.mask;
}

// Get homing rate from the first axis in the cycle.
static float get_homing_rate (axes_signals_t cycle, homing_mode_t mode)
{
    uint_fast8_t idx = 0;

    if(settings.homing.flags.per_axis_feedrates)
        idx = ffs(cycle.mask) - 1;

    return mode == HomingMode_Locate ? settings.axis[idx].homing_feed_rate : settings.axis[idx].homing_seek_rate;
}

// Checks and reports if target array exceeds machine travel limits. Returns false if check failed.
static bool check_travel_limits (float *target, axes_signals_t axes, bool is_cartesian)
{
    bool failed = false;
    uint_fast8_t idx = N_AXIS;

    if(is_cartesian && (sys.homed.mask & axes.mask)) do {
        idx--;
        if(bit_istrue(sys.homed.mask, bit(idx)) && bit_istrue(axes.mask, bit(idx)))
            failed = target[idx] < sys.work_envelope.min.values[idx] || target[idx] > sys.work_envelope.max.values[idx];
    } while(!failed && idx);

    return is_cartesian && !failed;
}

// Checks and reports if the arc exceeds machine travel limits. Returns false if check failed.
// NOTE: needs the work envelope to be a cuboid!
static bool check_arc_travel_limits (coord_data_t *target, coord_data_t *position, point_2d_t center, float radius, plane_t plane, int32_t turns)
{
    typedef union {
        uint_fast8_t value;
        struct {
            uint_fast8_t pos_y : 1,
                         neg_x : 1,
                         neg_y : 1,
                         pos_x : 1;
        };
    } arc_x_t;

    static const axes_signals_t xyz = { .x = On, .y = On, .z = On };

    if((sys.soft_limits.mask & xyz.mask) == 0)
        return grbl.check_travel_limits(target->values, sys.soft_limits, true);

    arc_x_t x = {0};
    point_2d_t start, end;

    // Set arc start and end points centered at 0,0 and convert CW arcs to CCW.
    if(turns > 0) { // CCW
        start.x = position->values[plane.axis_0] - center.x;
        start.y = position->values[plane.axis_1] - center.y;
        end.x = target->values[plane.axis_0] - center.x;
        end.y = target->values[plane.axis_1] - center.y;
    } else { // CW
        start.x = target->values[plane.axis_0] - center.x;
        start.y = target->values[plane.axis_1] - center.y;
        end.x = position->values[plane.axis_0] - center.x;
        end.y = position->values[plane.axis_1] - center.y;
    }

    if(labs(turns > 1))
        x.value = 0b1111;                   // Crosses all
    else if(start.y >= 0.0f) {
        if(start.x > 0.0f) {                // Starts in Q1
            if(end.y >= 0.0f) {
                if(end.x <= 0.0f)
                    x.value = 0b0001;       // Ends in Q2
                else if(end.x >= start.x)
                    x.value = 0b1111;       // Ends in Q1, crosses all
            } else if(end.x <= 0.0f)
                x.value = 0b0011;           // Ends in Q3
            else
                x.value = 0b0111;           // Ends in Q4
        } else {                            // Starts in Q2
            if(end.y >= 0.0f) {
                if(end.x > 0.0f)
                    x.value = 0b1110;       // Ends in Q1
                else if(end.x >= start.x)
                    x.value = 0b1111;       // Ends in Q2, crosses all
            } else if(end.x <= 0.0f)
                x.value = 0b0010;           // Ends in Q3
            else
                x.value = 0b0110;           // Ends in Q4
        }
    } else if(start.x < 0.0f) {             // Starts in Q3
        if(end.y < 0.0f) {
            if(end.x > 0.0f)
                x.value = 0b0100;           // Ends in Q4
            else if(end.x <= start.x)
                x.value = 0b1111;           // Ends in Q3, crosses all
        } else if(end.x > 0.0f)
            x.value = 0b1100;               // Ends in Q1
        else
            x.value = 0b1101;               // Ends in Q2
    } else {                                // Starts in Q4
        if(end.y < 0.0f) {
            if(end.x < 0.0f)
                x.value = 0b1011;           // Ends in Q3
            else if(end.x <= start.x)
                x.value = 0b1111;           // Ends in Q4, crosses all
        } else if(end.x > 0.0f)
            x.value = 0b1000;               // Ends in Q1
        else
            x.value = 0b1001;               // Ends in Q2
    }

    coord_data_t corner1, corner2;

    memcpy(&corner1, turns > 0 ? position : target, sizeof(coord_data_t));
    corner1.values[plane.axis_0] = x.neg_x ? center.x - radius : min(position->values[plane.axis_0], target->values[plane.axis_0]);
    corner1.values[plane.axis_1] = x.neg_y ? center.y - radius : max(position->values[plane.axis_1], target->values[plane.axis_1]);

    if(!grbl.check_travel_limits(corner1.values, sys.soft_limits, true))
        return false;

    memcpy(&corner2, turns > 0 ? target : position, sizeof(coord_data_t));
    corner2.values[plane.axis_0] = x.pos_x ? center.x + radius : max(position->values[plane.axis_0], target->values[plane.axis_0]);
    corner2.values[plane.axis_1] = x.pos_y ? center.y + radius : min(position->values[plane.axis_1], target->values[plane.axis_1]);

   return grbl.check_travel_limits(corner2.values, sys.soft_limits, true);
}

// Derived from code by Dimitrios Matthes & Vasileios Drakopoulos
// https://www.mdpi.com/1999-4893/16/4/201
static void clip_3d_target (coord_data_t *position, coord_data_t *target, work_envelope_t *envelope)
{
    float a = target->x - position->x;
    float b = target->y - position->y;
    float c = target->z - position->z;

    if(target->x < envelope->min.x) {
        target->y = b / a * (envelope->min.x - position->x) + position->y;
        target->z = c / a * (envelope->min.x - position->x) + position->z;
        target->x = envelope->min.x;
    } else if(target->x > envelope->max.x) {
        target->y = b / a * (envelope->max.x - position->x) + position->y;
        target->z = c / a * (envelope->max.x - position->x) + position->z;
        target->x = envelope->max.x;
    }

    if(target->y < envelope->min.y) {
        target->x = a / b * (envelope->min.y - position->y) + position->x;
        target->z = c / b * (envelope->min.y - position->y) + position->z;
        target->y = envelope->min.y;
    } else if(target->y > envelope->max.y) {
        target->x = a / b * (envelope->max.y - position->y) + position->x;
        target->z = c / b * (envelope->max.y - position->y) + position->z;
        target->y = envelope->max.y;
    }

    if(target->z < envelope->min.z) {
        target->x = a / c * (envelope->min.z - position->z) + position->x;
        target->y = b / c * (envelope->min.z - position->z) + position->y;
        target->z = envelope->min.z;
    } else if(target->z > envelope->max.z) {
        target->x = a / c * (envelope->max.z - position->z) + position->x;
        target->y = b / c * (envelope->max.z - position->z) + position->y;
        target->z = envelope->max.z;
    }
}

// Limits jog commands to be within machine limits, homed axes only.
static void apply_jog_limits (float *target, float *position)
{
    if(sys.homed.mask == 0)
        return;

    uint_fast8_t idx;

    if((sys.homed.mask & 0b111) == 0b111) {

        uint_fast8_t n_axes = 0;

        idx = Z_AXIS + 1;
        do {
            idx--;
            if(fabs(target[idx] - position[idx]) > 0.001f)
                n_axes++;
        } while(idx && n_axes < 2);

        if(n_axes > 1)
            clip_3d_target((coord_data_t *)position, (coord_data_t *)target, &sys.work_envelope);
    }

    idx = N_AXIS;
    do {
        idx--;
        if(bit_istrue(sys.homed.mask, bit(idx)) && settings.axis[idx].max_travel < -0.0f)
            target[idx] = max(min(target[idx], sys.work_envelope.max.values[idx]), sys.work_envelope.min.values[idx]);
    } while(idx);
}

void limits_init (void)
{
    hal.homing.get_feedrate = get_homing_rate;
    grbl.check_travel_limits = check_travel_limits;
    grbl.check_arc_travel_limits = check_arc_travel_limits;
    grbl.apply_jog_limits = apply_jog_limits;
    grbl.home_machine = homing_cycle;
}
