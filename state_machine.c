/*
  state_machine.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Main state machine

  Part of grblHAL

  Copyright (c) 2018-2022 Terje Io
  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
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

#include <string.h>

#include "hal.h"
#include "motion_control.h"
#include "state_machine.h"
#include "override.h"

static void state_idle (uint_fast16_t new_state);
static void state_cycle (uint_fast16_t rt_exec);
static void state_await_hold (uint_fast16_t rt_exec);
static void state_noop (uint_fast16_t rt_exec);
static void state_await_motion_cancel (uint_fast16_t rt_exec);
static void state_await_resume (uint_fast16_t rt_exec);
static void state_await_toolchanged (uint_fast16_t rt_exec);
static void state_await_waypoint_retract (uint_fast16_t rt_exec);
static void state_restore (uint_fast16_t rt_exec);
static void state_await_resumed (uint_fast16_t rt_exec);

static void (* volatile stateHandler)(uint_fast16_t rt_exec) = state_idle;

static float restore_spindle_rpm;
static planner_cond_t restore_condition;
static sys_state_t pending_state = STATE_IDLE, sys_state = STATE_IDLE;

typedef union {
    uint8_t value;
    struct {
        uint8_t active     :1,
                motion     :1,
                restart    :1,
                restoring  :1,
                unassigned :4;
    };
} parking_flags_t;

typedef struct {
    float target[N_AXIS];
    float restore_target[N_AXIS];
    float retract_waypoint;
    volatile parking_flags_t flags;
    plan_line_data_t plan_data;
} parking_data_t;

// Declare and initialize parking local variables
static parking_data_t park = {0};

static void state_restore_conditions (planner_cond_t *condition, float rpm)
{
    if (!settings.parking.flags.enabled || !park.flags.restart) {

        park.flags.restoring = On; //

        spindle_restore(condition->spindle, rpm);

        // Block if safety door re-opened during prior restore actions.
        if (gc_state.modal.coolant.value != hal.coolant.get_state().value) {
            // NOTE: Laser mode will honor this delay. An exhaust system is often controlled by this pin.
            coolant_set_state(condition->coolant);
            delay_sec(settings.safety_door.coolant_on_delay, DelayMode_SysSuspend);
        }

        park.flags.restoring = Off;

        sys.override.spindle_stop.value = 0; // Clear spindle stop override states
    }
}

bool initiate_hold (uint_fast16_t new_state)
{
    if (settings.parking.flags.enabled) {
        memset(&park.plan_data, 0, sizeof(plan_line_data_t));
        park.plan_data.condition.system_motion = On;
        park.plan_data.condition.no_feed_override = On;
        park.plan_data.line_number = PARKING_MOTION_LINE_NUMBER;
    }

    plan_block_t *block = plan_get_current_block();

    if (block == NULL) {
        restore_condition.spindle = gc_state.modal.spindle;
        restore_condition.coolant.mask = gc_state.modal.coolant.mask | hal.coolant.get_state().mask;
        restore_spindle_rpm = gc_state.spindle.rpm;
    } else {
        restore_condition = block->condition;
        restore_spindle_rpm = block->spindle.rpm;
    }

    if (sys.mode == Mode_Laser && settings.flags.disable_laser_during_hold)
        enqueue_accessory_override(CMD_OVERRIDE_SPINDLE_STOP);

    if (sys_state & (STATE_CYCLE|STATE_JOG)) {
        st_update_plan_block_parameters();  // Notify stepper module to recompute for hold deceleration.
        sys.step_control.execute_hold = On; // Initiate suspend state with active flag.
        stateHandler = state_await_hold;
    }

    if (new_state == STATE_HOLD)
        sys.holding_state = Hold_Pending;
    else {
        sys.parking_state = Parking_Retracting;
        park.flags.value = 0;
    }

    sys.suspend = true;
    pending_state = sys_state == STATE_JOG ? new_state : STATE_IDLE;

    return sys_state == STATE_CYCLE;
}

bool state_door_reopened (void)
{
    return settings.parking.flags.enabled && park.flags.restart;
}

void state_update (rt_exec_t rt_exec)
{
    if ((rt_exec & EXEC_SAFETY_DOOR) && sys_state != STATE_SAFETY_DOOR)
        state_set(STATE_SAFETY_DOOR);

    stateHandler(rt_exec);
}

ISR_CODE sys_state_t ISR_FUNC(state_get)(void)
{
    return sys_state;
}

void state_set (sys_state_t new_state)
{
    if (new_state != sys_state) {

        switch(new_state) {    // Set up new state and handler

            case STATE_IDLE:
                sys.suspend = false;        // Break suspend state.
                sys.step_control.flags = 0; // Restore step control to normal operation.
                sys.parking_state = Parking_DoorClosed;
                sys.holding_state = Hold_NotHolding;
                sys_state = pending_state = new_state;
                park.flags.value = 0;
                stateHandler = state_idle;
                break;

            case STATE_CYCLE:
                if (sys_state == STATE_IDLE) {
                    // Start cycle only if queued motions exist in planner buffer and the motion is not canceled.
                    plan_block_t *block;
                    if ((block = plan_get_current_block())) {
                        sys_state = new_state;
                        sys.steppers_deenergize = false;    // Cancel stepper deenergize if pending.
                        st_prep_buffer();                   // Initialize step segment buffer before beginning cycle.
                        if (block->condition.spindle.synchronized) {

                            if (hal.spindle.reset_data)
                                hal.spindle.reset_data();

                            uint32_t index = hal.spindle.get_data(SpindleData_Counters)->index_count + 2;

                            while(index != hal.spindle.get_data(SpindleData_Counters)->index_count); // check for abort in this loop?

                        }
                        st_wake_up();
                        stateHandler = state_cycle;
                    }
                }
                break;

            case STATE_JOG:
                if (sys_state == STATE_TOOL_CHANGE)
                    pending_state = STATE_TOOL_CHANGE;
                sys_state = new_state;
                stateHandler = state_cycle;
                break;

            case STATE_TOOL_CHANGE:
                sys_state = new_state;
                stateHandler = state_await_toolchanged;
                break;

            case STATE_HOLD:
                if (sys.override.control.sync && sys.override.control.feed_hold_disable)
                    sys.flags.feed_hold_pending = On;
                if (!((sys_state & STATE_JOG) || sys.override.control.feed_hold_disable)) {
                    if (!initiate_hold(new_state)) {
                        sys.holding_state = Hold_Complete;
                        stateHandler = state_await_resume;
                    }
                    sys_state = new_state;
                    sys.flags.feed_hold_pending = Off;
                }
                break;

            case STATE_SAFETY_DOOR:
                if ((sys_state & (STATE_ALARM|STATE_ESTOP|STATE_SLEEP|STATE_CHECK_MODE)))
                    return;
                grbl.report.feedback_message(Message_SafetyDoorAjar);
                // no break
            case STATE_SLEEP:
                sys.parking_state = Parking_Retracting;
                if (!initiate_hold(new_state)) {
                    if (pending_state != new_state) {
                        sys_state = new_state;
                        state_await_hold(EXEC_CYCLE_COMPLETE); // "Simulate" a cycle stop
                    }
                } else
                    sys_state = new_state;
                break;

            case STATE_ALARM:
            case STATE_ESTOP:
            case STATE_HOMING:
            case STATE_CHECK_MODE:
                sys_state = new_state;
                sys.suspend = false;
                stateHandler = state_noop;
                break;
        }

        if (!(sys_state & (STATE_ALARM|STATE_ESTOP)))
            sys.alarm = Alarm_None;

        if (grbl.on_state_change)
            grbl.on_state_change(new_state);
    }
}

// Suspend manager. Controls spindle overrides in hold states.
void state_suspend_manager (void)
{
    if (stateHandler != state_await_resume || !gc_state.modal.spindle.on)
        return;

    if (sys.override.spindle_stop.value) {

        // Handles beginning of spindle stop
        if (sys.override.spindle_stop.initiate) {
            sys.override.spindle_stop.value = 0; // Clear stop override state
            spindle_set_state((spindle_state_t){0}, 0.0f); // De-energize
            sys.override.spindle_stop.enabled = On; // Set stop override state to enabled, if de-energized.
        }

        // Handles restoring of spindle state
        if (sys.override.spindle_stop.restore) {
            grbl.report.feedback_message(Message_SpindleRestore);
            if (sys.mode == Mode_Laser) // When in laser mode, ignore spindle spin-up delay. Set to turn on laser when cycle starts.
                sys.step_control.update_spindle_rpm = On;
            else
                spindle_set_state(restore_condition.spindle, restore_spindle_rpm);
            sys.override.spindle_stop.value = 0; // Clear stop override state
        }

    } else if (sys.step_control.update_spindle_rpm && hal.spindle.get_state().on) {
        // Handles spindle state during hold. NOTE: Spindle speed overrides may be altered during hold state.
        spindle_set_state(restore_condition.spindle, restore_spindle_rpm);
        sys.step_control.update_spindle_rpm = Off;
    }
}

// **************
// State handlers
// **************

/*! /brief No operation handler.
 */
static void state_noop (uint_fast16_t rt_exec)
{
    // Do nothing - state change requests are handled elsewhere or ignored.
}

/*! /brief Waits for idle actions and executes them by switching to the appropriate sys_state.
 */
static void state_idle (uint_fast16_t rt_exec)
{
    if ((rt_exec & EXEC_CYCLE_START))
        state_set(STATE_CYCLE);

    if (rt_exec & EXEC_FEED_HOLD)
        state_set(STATE_HOLD);

    if ((rt_exec & EXEC_TOOL_CHANGE)) {
        hal.stream.suspend_read(true); // Block reading from input stream until tool change state is acknowledged
        state_set(STATE_TOOL_CHANGE);
    }

    if (rt_exec & EXEC_SLEEP)
        state_set(STATE_SLEEP);
}

/*! /brief Waits for cycle actions and executes them by switching to the appropriate sys_state.
 */
static void state_cycle (uint_fast16_t rt_exec)
{
    if (rt_exec == EXEC_CYCLE_START)
        return; // no need to perform other tests...

    if ((rt_exec & EXEC_TOOL_CHANGE))
        hal.stream.suspend_read(true); // Block reading from input stream until tool change state is acknowledged

    if (rt_exec & EXEC_CYCLE_COMPLETE)
        state_set(gc_state.tool_change ? STATE_TOOL_CHANGE : STATE_IDLE);

    if (rt_exec & EXEC_MOTION_CANCEL) {
        st_update_plan_block_parameters();  // Notify stepper module to recompute for hold deceleration.
        sys.suspend = true;
        sys.step_control.execute_hold = On; // Initiate suspend state with active flag.
        stateHandler = state_await_motion_cancel;
    }

    if ((rt_exec & EXEC_FEED_HOLD))
        state_set(STATE_HOLD);
}

/*! /brief Waits for tool change cycle to end then restarts the cycle.
 */
static void state_await_toolchanged (uint_fast16_t rt_exec)
{
    if (rt_exec & EXEC_CYCLE_START) {
        if (!gc_state.tool_change) {
            if (hal.stream.suspend_read)
                hal.stream.suspend_read(false); // Tool change complete, restore "normal" stream input.
            sys.report.tool = On;
        }
        pending_state = gc_state.tool_change ? STATE_TOOL_CHANGE : STATE_IDLE;
        state_set(STATE_IDLE);
        state_set(STATE_CYCLE);
        // Force a status report to let the sender know tool change is completed.
        system_set_exec_state_flag(EXEC_STATUS_REPORT);
    }
}

/*! /brief Waits for motion to end to complete then executes actions depending on the current sys_state.
 */
static void state_await_motion_cancel (uint_fast16_t rt_exec)
{
    if (rt_exec & EXEC_CYCLE_COMPLETE) {
        if (sys_state == STATE_JOG) {
            sys.step_control.flags = 0;
            plan_reset();
            st_reset();
            sync_position();
            sys.suspend = false;
        }
        state_set(pending_state);
        if (gc_state.tool_change)
            state_set(STATE_TOOL_CHANGE);
    }
}

/*! /brief Waits for feed hold to complete then executes actions depending on the current sys_state.
 */
static void state_await_hold (uint_fast16_t rt_exec)
{
    if (rt_exec & EXEC_CYCLE_COMPLETE) {

        bool handler_changed = false;

        plan_cycle_reinitialize();
        sys.step_control.flags = 0;

        if (sys.alarm_pending) {
            system_set_exec_alarm(sys.alarm_pending);
            sys.alarm_pending = Alarm_None;
        }

        switch (sys_state) {

            case STATE_TOOL_CHANGE:
                hal.spindle.set_state((spindle_state_t){0}, 0.0f); // De-energize
                hal.coolant.set_state((coolant_state_t){0}); // De-energize
                break;

            // Resume door state when parking motion has retracted and door has been closed.
            case STATE_SLEEP:
            case STATE_SAFETY_DOOR:
                // Parking manager. Handles de/re-energizing, switch state checks, and parking motions for
                // the safety door and sleep states.

                // Handles retraction motions and de-energizing.
                // Ensure any prior spindle stop override is disabled at start of safety door routine.
                sys.override.spindle_stop.value = 0;

                // Parking requires parking axis homed, the current location not exceeding the???
                // parking target location, and laser mode disabled.
                if (settings.parking.flags.enabled && !sys.override.control.parking_disable && sys.mode != Mode_Laser) {

                    // Get current position and store as restore location.
                    if (!park.flags.active) {
                        park.flags.active = On;
                        system_convert_array_steps_to_mpos(park.restore_target, sys.position);
                    }

                    // Execute slow pull-out parking retract motion if parking axis is homed and parking target is above restore target.
                    if (bit_istrue(sys.homed.mask, bit(settings.parking.axis)) && (park.restore_target[settings.parking.axis] < settings.parking.target)) {

                        bool await_motion;

                        handler_changed = true;
                        stateHandler = state_await_waypoint_retract;

                        // Copy current location to park target and calculate retract waypoint if not restarting.
                        if(park.flags.restart)
                            system_convert_array_steps_to_mpos(park.target, sys.position);
                        else {
                            memcpy(park.target, park.restore_target, sizeof(park.target));
                            park.retract_waypoint = settings.parking.pullout_increment + park.target[settings.parking.axis];
                            park.retract_waypoint = min(park.retract_waypoint, settings.parking.target);
                        }

                        // Retract by pullout distance. Ensure retraction motion moves away from
                        // the workpiece and waypoint motion doesn't exceed the parking target location.
                        if ((await_motion = park.target[settings.parking.axis] < park.retract_waypoint)) {
                            park.target[settings.parking.axis] = park.retract_waypoint;
                            park.plan_data.feed_rate = settings.parking.pullout_rate;
                            park.plan_data.condition.coolant = restore_condition.coolant; // Retain coolant state
                            park.plan_data.condition.spindle = restore_condition.spindle; // Retain spindle state
                            park.plan_data.spindle.rpm = restore_spindle_rpm;
                            await_motion = mc_parking_motion(park.target, &park.plan_data);
                        }

                        if(!park.flags.restart)
                            park.flags.motion = await_motion;

                        if (!await_motion)
                            stateHandler(EXEC_CYCLE_COMPLETE); // No motion, proceed to next step immediately.

                    } else {
                        // Parking motion not possible. Just disable the spindle and coolant.
                        // NOTE: Laser mode does not start a parking motion to ensure the laser stops immediately.
                        hal.spindle.set_state((spindle_state_t){0}, 0.0f); // De-energize
                        if (!settings.safety_door.flags.keep_coolant_on)
                            hal.coolant.set_state((coolant_state_t){0});     // De-energize
                        sys.parking_state = hal.control.get_state().safety_door_ajar ? Parking_DoorAjar : Parking_DoorClosed;
                    }
                } else {
                    hal.spindle.set_state((spindle_state_t){0}, 0.0f); // De-energize
                    if (!settings.safety_door.flags.keep_coolant_on)
                        hal.coolant.set_state((coolant_state_t){0}); // De-energize
                    sys.parking_state = hal.control.get_state().safety_door_ajar ? Parking_DoorAjar : Parking_DoorClosed;
                }
                break;

            default:
                break;
        }

        if (!handler_changed) {
            sys.holding_state = Hold_Complete;
            stateHandler = state_await_resume;
        }
    }
}

/*! /brief Waits for action to execute when in feed hold state.
 */
static void state_await_resume (uint_fast16_t rt_exec)
{
    if ((rt_exec & EXEC_CYCLE_COMPLETE) && settings.parking.flags.enabled) {
        if (sys.step_control.execute_sys_motion) {
            sys.step_control.execute_sys_motion = Off;
            st_parking_restore_buffer(); // Restore step segment buffer to normal run state.
        }
        sys.parking_state = hal.control.get_state().safety_door_ajar ? Parking_DoorAjar : Parking_DoorClosed;
    }

    if (rt_exec & EXEC_SLEEP)
        state_set(STATE_SLEEP);

    if (rt_exec & EXEC_SAFETY_DOOR)
        sys.parking_state = hal.control.get_state().safety_door_ajar ? Parking_DoorAjar : Parking_DoorClosed;

    else if (rt_exec & EXEC_CYCLE_START) {

        if (sys_state == STATE_HOLD && !sys.override.spindle_stop.value)
            sys.override.spindle_stop.restore_cycle = On;

        switch (sys_state) {

            case STATE_TOOL_CHANGE:
                break;

            case STATE_SLEEP:
                break;

            case STATE_SAFETY_DOOR:
                if (park.flags.restart || !hal.control.get_state().safety_door_ajar) {

                    bool await_motion = false;
                    stateHandler = state_restore;
                    sys.parking_state = Parking_Resuming;

                    // Resume door state when parking motion has retracted and door has been closed.
                    if (park.flags.motion) {

                        park.flags.restart = Off;

                        // Execute fast restore motion to the pull-out position.
                        // Check to ensure the motion doesn't move below pull-out position.
                        if (park.restore_target[settings.parking.axis] <= settings.parking.target) {
                            float target[N_AXIS];
                            memcpy(target, park.restore_target, sizeof(target));
                            target[settings.parking.axis] = park.retract_waypoint;
                            park.plan_data.feed_rate = settings.parking.rate;
                            await_motion = mc_parking_motion(target, &park.plan_data);
                        }
                    }

                    if (!await_motion) // No motion, proceed to next step immediately.
                        stateHandler(EXEC_CYCLE_COMPLETE);
                }
                break;

            default:
                if (!settings.flags.restore_after_feed_hold) {
                    if (!hal.spindle.get_state().on)
                        gc_spindle_off();
                    sys.override.spindle_stop.value = 0; // Clear spindle stop override states
                } else {

                    if (restore_condition.spindle.on != hal.spindle.get_state().on) {
                        grbl.report.feedback_message(Message_SpindleRestore);
                        spindle_restore(restore_condition.spindle, restore_spindle_rpm);
                    }

                    if (restore_condition.coolant.value != hal.coolant.get_state().value) {
                        // NOTE: Laser mode will honor this delay. An exhaust system is often controlled by coolant signals.
                        coolant_set_state(restore_condition.coolant);
                        delay_sec(settings.safety_door.coolant_on_delay, DelayMode_SysSuspend);
                    }

                    sys.override.spindle_stop.value = 0; // Clear spindle stop override states

                    grbl.report.feedback_message(Message_None);
                }
                break;
        }

        // Restart cycle
        if (!(sys_state & (STATE_SLEEP|STATE_SAFETY_DOOR))) {
            state_set(STATE_IDLE);
            state_set(STATE_CYCLE);
        }

    } else if ((rt_exec & EXEC_DOOR_CLOSED) && !hal.control.get_state().safety_door_ajar)
        sys.parking_state = Parking_DoorClosed;
}

// ********************
// Safety door handlers
// ********************

/*! /brief Waits until plunge motion abort is completed then calls state_await_hold() to restart retraction.
state_await_hold() is set to handle the cycle complete event.
 */
static void state_await_restart_retract (uint_fast16_t rt_exec)
{
    if (rt_exec & EXEC_CYCLE_COMPLETE) {

        if (sys.step_control.execute_sys_motion) {
            sys.step_control.execute_sys_motion = Off;
            st_parking_restore_buffer(); // Restore step segment buffer to normal run state.
        }

        stateHandler = state_await_hold;
        stateHandler(EXEC_CYCLE_COMPLETE);
    }
}

/*! /brief Sets up a feed hold to abort plunge motion.
state_await_restart_retract() is set to handle the cycle complete event.
 */
static void restart_retract (void)
{
    grbl.report.feedback_message(Message_SafetyDoorAjar);

    stateHandler = state_await_restart_retract;

    park.flags.restart = On;
    sys.parking_state = Parking_Retracting;

    if (sys.step_control.execute_sys_motion) {
        st_update_plan_block_parameters(); // Notify stepper module to recompute for hold deceleration.
        sys.step_control.execute_hold = On;
        sys.step_control.execute_sys_motion = On;
    } else // else NO_MOTION is active.
        stateHandler(EXEC_CYCLE_COMPLETE);
}

/*! /brief Waits until slow plunge motion is completed then deenergize spindle and coolant and execute fast retract motion.
state_await_resume() is set to handle the cycle complete event.
 */
static void state_await_waypoint_retract (uint_fast16_t rt_exec)
{
    if (rt_exec & EXEC_CYCLE_COMPLETE) {

        bool await_motion = false;

        if (sys.step_control.execute_sys_motion) {
            sys.step_control.execute_sys_motion = Off;
            st_parking_restore_buffer(); // Restore step segment buffer to normal run state.
        }

        // NOTE: Clear accessory state after retract and after an aborted restore motion.
        park.plan_data.condition.spindle.value = 0;
        park.plan_data.spindle.rpm = 0.0f;
        hal.spindle.set_state(park.plan_data.condition.spindle, 0.0f); // De-energize

        if (!settings.safety_door.flags.keep_coolant_on) {
            park.plan_data.condition.coolant.value = 0;
            hal.coolant.set_state(park.plan_data.condition.coolant); // De-energize
        }

        stateHandler = state_await_resume;

        // Execute fast parking retract motion to parking target location.
        if (park.flags.motion && park.target[settings.parking.axis] < settings.parking.target) {
            float target[N_AXIS];
            memcpy(target, park.target, sizeof(target));
            target[settings.parking.axis] = settings.parking.target;
            park.plan_data.feed_rate = settings.parking.rate;
            await_motion = mc_parking_motion(target, &park.plan_data);
        }

        if (!await_motion)
            stateHandler(EXEC_CYCLE_COMPLETE);
    }
}

/*! /brief Waits until fast plunge motion is completed then restore spindle and coolant and execute slow plunge motion.
state_await_resumed() is set to handle the cycle complete event.
Note: A safety door event during restoration or motion will halt it and restart the retract sequence.
 */
static void state_restore (uint_fast16_t rt_exec)
{
    if (rt_exec & EXEC_SAFETY_DOOR) {
        if(park.flags.restoring)
            park.flags.restart = On;
        else
            restart_retract();
    }

    else if (rt_exec & EXEC_CYCLE_COMPLETE) {

        bool await_motion = false;

        if (sys.step_control.execute_sys_motion) {
            sys.step_control.execute_sys_motion = Off;
            st_parking_restore_buffer(); // Restore step segment buffer to normal run state.
        }

        park.flags.restart = Off;
        stateHandler = state_await_resumed;

        // Restart spindle and coolant, delay to power-up.
        state_restore_conditions(&restore_condition, restore_spindle_rpm);

        if(park.flags.restart) {
            // Restart flag was set by a safety door event during
            // conditions restore so restart retract.
            restart_retract();
            return;
        }

        if (park.flags.motion) {

            sys.parking_state = Parking_Resuming;

            // Execute slow plunge motion from pull-out position to resume position.

            // Regardless if the retract parking motion was a valid/safe motion or not, the
            // restore parking motion should logically be valid, either by returning to the
            // original position through valid machine space or by not moving at all.
            park.plan_data.feed_rate = settings.parking.pullout_rate;
            park.plan_data.condition.coolant = restore_condition.coolant;
            park.plan_data.condition.spindle = restore_condition.spindle;
            park.plan_data.spindle.rpm = restore_spindle_rpm;
            await_motion = mc_parking_motion(park.restore_target, &park.plan_data);
        }

        if (!await_motion)
            stateHandler(EXEC_CYCLE_COMPLETE); // No motion, proceed to next step immediately.
    }
}

/*! /brief Waits until slow plunge motion is complete then restart the cycle.
Note: A safety door event during the motion will halt it and restart the retract sequence.
 */
static void state_await_resumed (uint_fast16_t rt_exec)
{
    if (rt_exec & EXEC_SAFETY_DOOR)
        restart_retract();

    else if (rt_exec & EXEC_CYCLE_COMPLETE) {
        sys.parking_state = Parking_DoorClosed;
        park.flags.value = 0;
        if (sys.step_control.execute_sys_motion) {
            sys.step_control.flags = 0;
            st_parking_restore_buffer(); // Restore step segment buffer to normal run state.
        }
        state_set(STATE_IDLE);
        state_set(STATE_CYCLE);
    }
}
