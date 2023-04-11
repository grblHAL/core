/*
  machine_limits.c - code pertaining to limit-switches and performing the homing cycle

  Part of grblHAL

  Copyright (c) 2017-2023 Terje Io
  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
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

// This is the Limit Pin Change Interrupt, which handles the hard limit feature. A bouncing
// limit switch can cause a lot of problems, like false readings and multiple interrupt calls.
// If a switch is triggered at all, something bad has happened and treat it as such, regardless
// if a limit switch is being disengaged. It's impossible to reliably tell the state of a
// bouncing pin because the microcontroller does not retain any state information when
// detecting a pin change. If we poll the pins in the ISR, you can miss the correct reading if the
// switch is bouncing.
// NOTE: Do not attach an e-stop to the limit pins, because this interrupt is disabled during
// homing cycles and will not respond correctly. Upon user request or need, there may be a
// special pinout for an e-stop, but it is generally recommended to just directly connect
// your e-stop switch to the microcontroller reset pin, since it is the most correct way to do this.

// Merge (bitwise or) all limit switch inputs.
ISR_CODE axes_signals_t ISR_FUNC(limit_signals_merge)(limit_signals_t signals)
{
    axes_signals_t state;

    state.mask = signals.min.mask | signals.min2.mask | signals.max.mask | signals.max2.mask;

    return state;
}

// Merge (bitwise or) home switch inputs (typically acquired from limits.min and limits.min2).
ISR_CODE static axes_signals_t ISR_FUNC(homing_signals_select)(limit_signals_t signals, axes_signals_t auto_square, squaring_mode_t mode)
{
    axes_signals_t state;

    switch(mode) {

        case SquaringMode_A:
            signals.min.mask &= ~auto_square.mask;
            break;

        case SquaringMode_B:
            signals.min2.mask &= ~auto_square.mask;
            break;

        default:
            break;
    }

    state.mask = signals.min.mask | signals.min2.mask;

    return state;
}

ISR_CODE void ISR_FUNC(limit_interrupt_handler)(limit_signals_t state) // DEFAULT: Limit pin change interrupt process.
{
    // Ignore limit switches if already in an alarm state or in-process of executing an alarm.
    // When in the alarm state, Grbl should have been reset or will force a reset, so any pending
    // moves in the planner and stream input buffers are all cleared and newly sent blocks will be
    // locked out until a homing cycle or a kill lock command. Allows the user to disable the hard
    // limit setting if their limits are constantly triggering after a reset and move their axes.

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

#ifndef KINEMATICS_API
// Set machine positions for homed limit switches. Don't update non-homed axes.
// NOTE: settings.max_travel[] is stored as a negative value.
void limits_set_machine_positions (axes_signals_t cycle, bool add_pulloff)
{
    uint_fast8_t idx = N_AXIS;
    float pulloff = add_pulloff ? settings.homing.pulloff : 0.0f;

    if(settings.homing.flags.force_set_origin) {
        do {
            if (cycle.mask & bit(--idx)) {
                sys.position[idx] = 0;
                sys.home_position[idx] = 0.0f;
            }
        } while(idx);
    } else do {
        if (cycle.mask & bit(--idx)) {
            sys.home_position[idx] = bit_istrue(settings.homing.dir_mask.value, bit(idx))
                                      ? settings.axis[idx].max_travel + pulloff
                                      : - pulloff;
            sys.position[idx] = sys.home_position[idx] * settings.axis[idx].steps_per_mm;
        }
    } while(idx);
}
#endif

// Pulls off axes from asserted homing switches before homing starts.
// For now only for auto squared axes.
static bool limits_pull_off (axes_signals_t axis, float distance)
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
            if (bit_istrue(settings.homing.dir_mask.value, bit(idx)))
                target.values[idx] += distance;
            else
                target.values[idx] -= distance;
        }
    } while(idx);

    plan_data.feed_rate = settings.homing.seek_rate * sqrtf(n_axis); // Adjust so individual axes all move at pull-off rate.
    plan_data.condition.coolant = gc_state.modal.coolant;
    memcpy(&plan_data.spindle, &gc_state.spindle, sizeof(spindle_t));

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

static float limits_get_homing_rate (axes_signals_t cycle, homing_mode_t mode)
{
    return mode == HomingMode_Locate ? settings.homing.feed_rate : settings.homing.seek_rate;
}

// Homes the specified cycle axes, sets the machine position, and performs a pull-off motion after
// completing. Homing is a special motion case, which involves rapid uncontrolled stops to locate
// the trigger point of the limit switches. The rapid stops are handled by a system level axis lock
// mask, which prevents the stepper algorithm from executing step pulses. Homing motions typically
// circumvent the processes for executing motions in normal operation.
// NOTE: Only the abort realtime command can interrupt this process.
static bool limits_homing_cycle (axes_signals_t cycle, axes_signals_t auto_square)
{
    if (ABORTED) // Block if system reset has been issued.
        return false;

    if(hal.homing.get_feedrate == NULL)
        hal.homing.get_feedrate = limits_get_homing_rate;

    int32_t initial_trigger_position = 0, autosquare_fail_distance = 0;
    uint_fast8_t n_cycle = (2 * settings.homing.locate_cycles + 1);
    uint_fast8_t step_pin[N_AXIS], n_active_axis, dual_motor_axis = 0;
    bool autosquare_check = false;
    float max_travel = 0.0f, homing_rate;
    homing_mode_t mode = HomingMode_Seek;
    axes_signals_t axislock, homing_state;
    limit_signals_t limits_state;
    squaring_mode_t squaring_mode = SquaringMode_Both;
    coord_data_t target;
    plan_line_data_t plan_data;

    plan_data_init(&plan_data);
    plan_data.condition.system_motion = On;
    plan_data.condition.no_feed_override = On;
    plan_data.line_number = DEFAULT_HOMING_CYCLE_LINE_NUMBER;

    // Initialize plan data struct for homing motion.
    memcpy(&plan_data.spindle, &gc_state.spindle, sizeof(spindle_t));
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
            if(bit_istrue(settings.steppers.is_rotational.mask, bit(idx)))
                max_travel = max(max_travel, (-HOMING_AXIS_SEARCH_SCALAR) * (settings.axis[idx].max_travel < -0.0f ? settings.axis[idx].max_travel : -360.0f));
            else
#endif
            max_travel = max(max_travel, (-HOMING_AXIS_SEARCH_SCALAR) * settings.axis[idx].max_travel);

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
                    target.values[idx] = mode == HomingMode_Pulloff ? max_travel : - max_travel;
                else
                    target.values[idx] = mode == HomingMode_Pulloff ? - max_travel : max_travel;

                // Apply axislock to the step port pins active in this cycle.
                axislock.mask |= step_pin[idx];
            }
        } while(idx);

#ifdef KINEMATICS_API
        if(kinematics.homing_cycle_get_feedrate)
            homing_rate = kinematics.homing_cycle_get_feedrate(homing_rate, cycle);
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
                homing_state = homing_signals_select(limits_state = hal.homing.get_state(), auto_square, squaring_mode);

                // Auto squaring check
                if((homing_state.mask & auto_square.mask) && squaring_mode == SquaringMode_Both) {
                    if((autosquare_check = (limits_state.min.mask & auto_square.mask) != (limits_state.min2.mask & auto_square.mask))) {
                        initial_trigger_position = sys.position[dual_motor_axis];
                        homing_state.mask &= ~auto_square.mask;
                        squaring_mode = (limits_state.min.mask & auto_square.mask) ? SquaringMode_A : SquaringMode_B;
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
            if (sys.rt_exec_state & (EXEC_SAFETY_DOOR | EXEC_RESET | EXEC_CYCLE_COMPLETE)) {

                uint_fast16_t rt_exec = sys.rt_exec_state;

                // Homing failure condition: Reset issued during cycle.
                if (rt_exec & EXEC_RESET)
                    system_set_exec_alarm(Alarm_HomingFailReset);

                // Homing failure condition: Safety door was opened.
                if (rt_exec & EXEC_SAFETY_DOOR)
                    system_set_exec_alarm(Alarm_HomingFailDoor);

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

            grbl.on_execute_realtime(STATE_HOMING);

        } while (axislock.mask & AXES_BITMASK);

        st_reset(); // Immediately force kill steppers and reset step segment buffer.
        hal.delay_ms(settings.homing.debounce_delay, NULL); // Delay to allow transient dynamics to dissipate.

        // Reverse direction and reset homing rate for cycle(s).
        mode = mode == HomingMode_Pulloff ? HomingMode_Locate : HomingMode_Pulloff;
        homing_rate = hal.homing.get_feedrate(cycle, mode);

        // After first cycle, homing enters locating phase. Shorten search to pull-off distance.
        if (mode == HomingMode_Locate) {
            // Only one initial pass for auto squared axis when both motors are active
            //if(mode == SquaringMode_Both && auto_square.mask)
            //    cycle.mask &= ~auto_square.mask;
            max_travel = settings.homing.pulloff * HOMING_AXIS_LOCATE_SCALAR;
        } else
            max_travel = settings.homing.pulloff;

        if(auto_square.mask) {
            autosquare_check = false;
            squaring_mode = SquaringMode_Both;
            hal.stepper.disable_motors((axes_signals_t){0}, SquaringMode_Both);
        }

    } while (homing_rate > 0.0f && cycle.mask && n_cycle-- > 0);

    // Pull off B motor to compensate for switch inaccuracy when configured.
    if(auto_square.mask && settings.axis[dual_motor_axis].dual_axis_offset != 0.0f) {
        hal.stepper.disable_motors(auto_square, settings.axis[dual_motor_axis].dual_axis_offset < 0.0f ? SquaringMode_B : SquaringMode_A);
        if(!limits_pull_off(auto_square, fabs(settings.axis[dual_motor_axis].dual_axis_offset)))
            return false;
        hal.stepper.disable_motors((axes_signals_t){0}, SquaringMode_Both);
    }

    // The active cycle axes should now be homed and machine limits have been located. By
    // default, Grbl defines machine space as all negative, as do most CNCs. Since limit switches
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

        if((auto_squared.mask & homing_signals_select(hal.homing.get_state(), (axes_signals_t){0}, SquaringMode_Both).mask) && !limits_pull_off(auto_square, settings.homing.pulloff * HOMING_AXIS_LOCATE_SCALAR))
            return Status_LimitsEngaged; // Auto squaring with limit switch asserted is not allowed.
    }

    tc_clear_tlo_reference(cycle);

    return limits_homing_cycle(cycle, auto_square) ? Status_OK : Status_Unhandled;
}

// Performs a soft limit check. Called from mc_line() only. Assumes the machine has been homed,
// the workspace volume is in all negative space, and the system is in normal operation.
// NOTE: Also used by jogging to block travel outside soft-limit volume.
void limits_soft_check  (float *target)
{
    if (!system_check_travel_limits(target)) {
        sys.flags.soft_limit = On;
        // Force feed hold if cycle is active. All buffered blocks are guaranteed to be within
        // workspace volume so just come to a controlled stop so position is not lost. When complete
        // enter alarm mode.
        if (state_get() == STATE_CYCLE) {
            system_set_exec_state_flag(EXEC_FEED_HOLD);
            do {
                if(!protocol_execute_realtime())
                    return; // aborted!
            } while (state_get() != STATE_IDLE);
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

