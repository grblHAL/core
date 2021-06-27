/*
  state_machine.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Main state machine

  Part of grblHAL

  Copyright (c) 2018-2021 Terje Io
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
//#include <stdlib.h>

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
static void state_await_restore (uint_fast16_t rt_exec);

static void (* volatile stateHandler)(uint_fast16_t rt_exec) = state_idle;

static float restore_spindle_rpm;
static planner_cond_t restore_condition;
static sys_state_t pending_state = STATE_IDLE, sys_state = STATE_IDLE;

typedef struct {
    float target[N_AXIS];
    float restore_target[N_AXIS];
    float retract_waypoint;
    bool retracting;
    bool restart_retract;
    bool active;
    plan_line_data_t plan_data;
} parking_data_t;

// Declare and initialize parking local variables
static parking_data_t park;

static void state_restore_conditions (planner_cond_t *condition, float rpm)
{
    if(!settings.parking.flags.enabled || !park.restart_retract) {

        spindle_restore(condition->spindle, rpm);

        // Block if safety door re-opened during prior restore actions.
        if (gc_state.modal.coolant.value != hal.coolant.get_state().value) {
            // NOTE: Laser mode will honor this delay. An exhaust system is often controlled by this pin.
            coolant_set_state(condition->coolant);
            delay_sec(SAFETY_DOOR_COOLANT_DELAY, DelayMode_SysSuspend);
        }

        sys.override.spindle_stop.value = 0; // Clear spindle stop override states
    }
}

bool initiate_hold (uint_fast16_t new_state)
{
    if(settings.parking.flags.enabled) {
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

    if(settings.mode == Mode_Laser && settings.flags.disable_laser_during_hold)
        enqueue_accessory_override(CMD_OVERRIDE_SPINDLE_STOP);

    if(sys_state & (STATE_CYCLE|STATE_JOG)) {
        st_update_plan_block_parameters();  // Notify stepper module to recompute for hold deceleration.
        sys.step_control.execute_hold = On; // Initiate suspend state with active flag.
        stateHandler = state_await_hold;
    }

    if(new_state == STATE_HOLD)
        sys.holding_state = Hold_Pending;
    else {
        sys.parking_state = Parking_Retracting;
        park.active = false;
    }

    sys.suspend = true;
    pending_state = sys_state == STATE_JOG ? new_state : STATE_IDLE;

    return sys_state == STATE_CYCLE;
}

bool state_door_reopened (void)
{
    return settings.parking.flags.enabled && park.restart_retract;
}

void state_update (rt_exec_t rt_exec)
{
    if((rt_exec & EXEC_SAFETY_DOOR) && sys_state != STATE_SAFETY_DOOR)
        state_set(STATE_SAFETY_DOOR);
    else
        stateHandler(rt_exec);
}

sys_state_t state_get (void)
{
    return sys_state;
}

void state_set (sys_state_t new_state)
{
    if(new_state != sys_state) {

        switch(new_state) {    // Set up new state and handler

            case STATE_IDLE:
                sys.suspend = false;        // Break suspend state.
                sys.step_control.flags = 0; // Restore step control to normal operation.
                sys.parking_state = Parking_DoorClosed;
                sys.holding_state = Hold_NotHolding;
                sys_state = pending_state = new_state;
                stateHandler = state_idle;
                break;

            case STATE_CYCLE:
                if(sys_state == STATE_IDLE) {
                    // Start cycle only if queued motions exist in planner buffer and the motion is not canceled.
                    plan_block_t *block;
                    if ((block = plan_get_current_block())) {
                        sys_state = new_state;
                        sys.steppers_deenergize = false;    // Cancel stepper deenergize if pending.
                        st_prep_buffer();                   // Initialize step segment buffer before beginning cycle.
                        if(block->condition.spindle.synchronized) {

                            if(hal.spindle.reset_data)
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
                if(sys_state == STATE_TOOL_CHANGE)
                    pending_state = STATE_TOOL_CHANGE;
                sys_state = new_state;
                stateHandler = state_cycle;
                break;

            case STATE_TOOL_CHANGE:
                sys_state = new_state;
                stateHandler = state_await_toolchanged;
                break;

            case STATE_HOLD:
                if(sys.override.control.sync && sys.override.control.feed_hold_disable)
                    sys.flags.feed_hold_pending = On;
                if(!((sys_state & STATE_JOG) || sys.override.control.feed_hold_disable)) {
                    if(!initiate_hold(new_state)) {
                        sys.holding_state = Hold_Complete;
                        stateHandler = state_await_resume;
                    }
                    sys_state = new_state;
                    sys.flags.feed_hold_pending = Off;
                }
                break;

            case STATE_SAFETY_DOOR:
                if((sys_state & (STATE_ALARM|STATE_ESTOP|STATE_SLEEP|STATE_CHECK_MODE)))
                    return;
                grbl.report.feedback_message(Message_SafetyDoorAjar);
                // no break
            case STATE_SLEEP:
                sys.parking_state = Parking_Retracting;
                if(!initiate_hold(new_state)) {
                    if(pending_state != new_state) {
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

        if(!(sys_state & (STATE_ALARM|STATE_ESTOP)))
            sys.alarm = Alarm_None;

        if(grbl.on_state_change)
            grbl.on_state_change(new_state);
    }
}

// Suspend manager. Controls spindle overrides in hold states.
void state_suspend_manager (void)
{
    if(stateHandler != state_await_resume || !gc_state.modal.spindle.on)
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
            if (settings.mode == Mode_Laser) // When in laser mode, ignore spindle spin-up delay. Set to turn on laser when cycle starts.
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

static void state_idle (uint_fast16_t rt_exec)
{
    if((rt_exec & EXEC_CYCLE_START))
        state_set(STATE_CYCLE);

    if(rt_exec & EXEC_FEED_HOLD)
        state_set(STATE_HOLD);

    if ((rt_exec & EXEC_TOOL_CHANGE)) {
        hal.stream.suspend_read(true); // Block reading from input stream until tool change state is acknowledged
        state_set(STATE_TOOL_CHANGE);
    }

    if (rt_exec & EXEC_SLEEP)
        state_set(STATE_SLEEP);
}

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

static void state_await_toolchanged (uint_fast16_t rt_exec)
{
    if (rt_exec & EXEC_CYCLE_START) {
        if(!gc_state.tool_change) {
            if(hal.stream.suspend_read)
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

static void state_await_motion_cancel (uint_fast16_t rt_exec)
{
    if (rt_exec & EXEC_CYCLE_COMPLETE) {
        if(sys_state == STATE_JOG) {
            sys.step_control.flags = 0;
            plan_reset();
            st_reset();
            sync_position();
#ifdef ENABLE_BACKLASH_COMPENSATION
            mc_sync_backlash_position();
#endif
            sys.suspend = false;
        }
        state_set(pending_state);
        if(gc_state.tool_change)
            state_set(STATE_TOOL_CHANGE);
    }
}

static void state_await_hold (uint_fast16_t rt_exec)
{
    if (rt_exec & EXEC_CYCLE_COMPLETE) {

        bool handler_changed = false;

        plan_cycle_reinitialize();
        sys.step_control.flags = 0;

        if(sys.alarm_pending) {
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
                if(settings.parking.flags.enabled && !sys.override.control.parking_disable && settings.mode != Mode_Laser) {

                    // Get current position and store as restore location.
                    if (!park.active) {
                        park.active = true;
                        system_convert_array_steps_to_mpos(park.restore_target, sys.position);
                    }

                    // Execute slow pull-out parking retract motion.
                    // NOTE: State is will remain DOOR, until the de-energizing and retract is complete.
                    if (bit_istrue(sys.homed.mask, bit(settings.parking.axis)) && (park.restore_target[settings.parking.axis] < settings.parking.target)) {

                        handler_changed = true;
                        stateHandler = state_await_waypoint_retract;

                        // Copy restore location to park target and calculate spindle retract waypoint.
                        memcpy(park.target, park.restore_target, sizeof(park.target));
                        park.retract_waypoint = settings.parking.pullout_increment + park.target[settings.parking.axis];
                        park.retract_waypoint = min(park.retract_waypoint, settings.parking.target);

                        // Retract spindle by pullout distance. Ensure retraction motion moves away from
                        // the workpiece and waypoint motion doesn't exceed the parking target location.
                        if (park.target[settings.parking.axis] < park.retract_waypoint) {
                            park.target[settings.parking.axis] = park.retract_waypoint;
                            park.plan_data.feed_rate = settings.parking.pullout_rate;
                            park.plan_data.condition.coolant = restore_condition.coolant; // Retain coolant state
                            park.plan_data.condition.spindle = restore_condition.spindle; // Retain spindle state
                            park.plan_data.spindle.rpm = restore_spindle_rpm;
                            if(!(park.retracting = mc_parking_motion(park.target, &park.plan_data)))
                                stateHandler(EXEC_CYCLE_COMPLETE);
                        } else
                            stateHandler(EXEC_CYCLE_COMPLETE);
                    } else {
                        // Parking motion not possible. Just disable the spindle and coolant.
                        // NOTE: Laser mode does not start a parking motion to ensure the laser stops immediately.
                        hal.spindle.set_state((spindle_state_t){0}, 0.0f); // De-energize
                        if(!settings.flags.keep_coolant_state_on_door_open)
                            hal.coolant.set_state((coolant_state_t){0});     // De-energize
                        sys.parking_state = Parking_DoorAjar;
                    }
                } else {
                    hal.spindle.set_state((spindle_state_t){0}, 0.0f); // De-energize
                    if(!settings.flags.keep_coolant_state_on_door_open)
                        hal.coolant.set_state((coolant_state_t){0}); // De-energize
                    sys.parking_state = Parking_DoorAjar;
                }
                break;

            default:
                break;
        }

        if(!handler_changed) {
            sys.holding_state = Hold_Complete;
            stateHandler = state_await_resume;
        }
    }
}

static void state_await_resume (uint_fast16_t rt_exec)
{
    if((rt_exec & EXEC_CYCLE_COMPLETE) && settings.parking.flags.enabled) {
        if(sys.step_control.execute_sys_motion)
            sys.step_control.execute_sys_motion = Off;
        sys.parking_state = Parking_DoorAjar;
    }

    if ((rt_exec & EXEC_CYCLE_START) && !(sys_state == STATE_SAFETY_DOOR && hal.control.get_state().safety_door_ajar)) {

        bool handler_changed = false;

        if(sys_state == STATE_HOLD && !sys.override.spindle_stop.value)
            sys.override.spindle_stop.restore_cycle = On;

        switch (sys_state) {

            case STATE_TOOL_CHANGE:
                break;

            case STATE_SLEEP:
                break;

            // Resume door state when parking motion has retracted and door has been closed.
            case STATE_SAFETY_DOOR:

                if(settings.parking.flags.enabled) {

                    park.restart_retract = false;
                    sys.parking_state = Parking_Resuming;

                    // Execute fast restore motion to the pull-out position. Parking requires homing enabled.
                    // NOTE: State is will remain DOOR, until the de-energizing and retract is complete.
                    if (park.retracting) {
                        handler_changed = true;
                        stateHandler = state_restore;
                        // Check to ensure the motion doesn't move below pull-out position.
                        if (park.target[settings.parking.axis] <= settings.parking.target) {
                            float target[N_AXIS];
                            memcpy(target, park.target, sizeof(target));
                            target[settings.parking.axis] = park.retract_waypoint;
                            park.plan_data.feed_rate = settings.parking.rate;
                            if(!mc_parking_motion(target, &park.plan_data))
                                stateHandler(EXEC_CYCLE_COMPLETE);
                            else
                                st_parking_setup_buffer();
                        } else // tell next handler to proceed with final step immediately
                            stateHandler(EXEC_CYCLE_COMPLETE);
                    }
                } else
                    // Delayed Tasks: Restart spindle and coolant, delay to power-up, then resume cycle.
                    // Block if safety door re-opened during prior restore actions.
                    state_restore_conditions(&restore_condition, restore_spindle_rpm);
                break;

            default:
                if (!settings.flags.restore_after_feed_hold) {
                    if(!hal.spindle.get_state().on) {
                        gc_state.spindle.rpm = 0.0f;
                        gc_state.modal.spindle.on = gc_state.modal.spindle.ccw = Off;
                    }
                    sys.override.spindle_stop.value = 0; // Clear spindle stop override states
                } else {
                    handler_changed = true;
                    stateHandler = state_await_restore;
                    stateHandler(0);
                }
                break;
        }

        // Restart cycle if there is no further processing to take place
        if(!(handler_changed || sys_state == STATE_SLEEP)) {
            state_set(STATE_IDLE);
            state_set(STATE_CYCLE);
        }
    }

    if (rt_exec & EXEC_SLEEP)
        state_set(STATE_SLEEP);
}

static void state_await_restore (uint_fast16_t rt_exec)
{
    static bool restart = false;

    if(rt_exec == 0) {

        restart = true;

        if (restore_condition.spindle.on != hal.spindle.get_state().on) {
            grbl.report.feedback_message(Message_SpindleRestore);
            spindle_restore(restore_condition.spindle, restore_spindle_rpm);
        }

        if (restore_condition.coolant.value != hal.coolant.get_state().value) {
            // NOTE: Laser mode will honor this delay. An exhaust system is often controlled by this pin.
            coolant_set_state(restore_condition.coolant);
            delay_sec(SAFETY_DOOR_COOLANT_DELAY, DelayMode_SysSuspend);
        }

        sys.override.spindle_stop.value = 0; // Clear spindle stop override states

        grbl.report.feedback_message(Message_None);

        if(restart) {
            state_set(STATE_IDLE);
            state_set(STATE_CYCLE);
        }
    }

    if(rt_exec & EXEC_FEED_HOLD) {
        restart = false;
        stateHandler = state_await_resume;
    }

}

static void restart_retract (void)
{
    grbl.report.feedback_message(Message_SafetyDoorAjar);

    stateHandler = state_await_hold;

    park.restart_retract = true;
    sys.parking_state = Parking_Retracting;

    if (sys.step_control.execute_sys_motion) {
        st_update_plan_block_parameters(); // Notify stepper module to recompute for hold deceleration.
        sys.step_control.execute_hold = On;
        sys.step_control.execute_sys_motion = On;
    } else // else NO_MOTION is active.
        stateHandler(EXEC_CYCLE_COMPLETE);
}

static void state_await_waypoint_cancel (uint_fast16_t rt_exec)
{
    if (rt_exec & EXEC_SAFETY_DOOR)
        restart_retract();

    else if (rt_exec & EXEC_CYCLE_COMPLETE) {
        sys.parking_state = Parking_Cancel;
        sys.step_control.execute_hold = Off;
        state_restore(rt_exec);
    }
}

static void state_await_waypoint_retract (uint_fast16_t rt_exec)
{
    if (rt_exec & EXEC_CYCLE_COMPLETE) {

        if(sys.step_control.execute_sys_motion)
            sys.step_control.execute_sys_motion = Off;

        // NOTE: Clear accessory state after retract and after an aborted restore motion.
        park.plan_data.condition.spindle.value = 0;
        park.plan_data.spindle.rpm = 0.0f;
        hal.spindle.set_state(park.plan_data.condition.spindle, 0.0f); // De-energize

        if(!settings.flags.keep_coolant_state_on_door_open) {
            park.plan_data.condition.coolant.value = 0;
            hal.coolant.set_state(park.plan_data.condition.coolant); // De-energize
        }

        stateHandler = state_await_resume;

        // Execute fast parking retract motion to parking target location.
        if (park.target[settings.parking.axis] < settings.parking.target) {
            float target[N_AXIS];
            memcpy(target, park.target, sizeof(target));
            target[settings.parking.axis] = settings.parking.target;
            park.plan_data.feed_rate = settings.parking.rate;
            if(mc_parking_motion(target, &park.plan_data))
                park.retracting = true;
            else
                stateHandler(EXEC_CYCLE_COMPLETE);
        } else
            stateHandler(EXEC_CYCLE_COMPLETE);

    } else if (rt_exec & EXEC_CYCLE_START) {

        stateHandler = state_await_waypoint_cancel;

        if (sys.step_control.execute_sys_motion) {
            st_update_plan_block_parameters(); // Notify stepper module to recompute for hold deceleration.
            sys.step_control.execute_hold = On;
            sys.step_control.execute_sys_motion = On;
        } else // else NO_MOTION is active.
            stateHandler(EXEC_CYCLE_COMPLETE);
    }
}

static void state_restore (uint_fast16_t rt_exec)
{
    if (rt_exec & EXEC_SAFETY_DOOR)
        restart_retract();

    else if (rt_exec & EXEC_CYCLE_COMPLETE) {

        if(sys.step_control.execute_sys_motion)
            sys.step_control.execute_sys_motion = Off;

        stateHandler = state_await_resumed;

        // Delayed Tasks: Restart spindle and coolant, delay to power-up, then resume cycle.
        // Block if safety door re-opened during prior restore actions.
        if(sys.parking_state != Parking_Cancel)
            state_restore_conditions(&restore_condition, restore_spindle_rpm);

        park.restart_retract = false;
        sys.parking_state = Parking_Resuming;

        // Execute slow plunge motion from pull-out position to resume position.

        // Regardless if the retract parking motion was a valid/safe motion or not, the
        // restore parking motion should logically be valid, either by returning to the
        // original position through valid machine space or by not moving at all.
        park.plan_data.feed_rate = settings.parking.pullout_rate;
        park.plan_data.condition.coolant = restore_condition.coolant;
        park.plan_data.condition.spindle = restore_condition.spindle;
        park.plan_data.spindle.rpm = restore_spindle_rpm;
        if(!mc_parking_motion(park.restore_target, &park.plan_data))
            stateHandler(EXEC_CYCLE_COMPLETE); // No motion, proceed to next step
    }
}

static void state_await_resumed (uint_fast16_t rt_exec)
{
    if (rt_exec & EXEC_SAFETY_DOOR)
        restart_retract();

    else if (rt_exec & EXEC_CYCLE_COMPLETE) {
        if(sys.step_control.execute_sys_motion) {
            sys.step_control.flags = 0;
            st_parking_restore_buffer(); // Restore step segment buffer to normal run state.
        }
        state_set(STATE_IDLE);
        state_set(STATE_CYCLE);
    }
}

static void state_noop (uint_fast16_t rt_exec)
{
    // Do nothing - state change requests are handled elsewhere or ignored.
}
