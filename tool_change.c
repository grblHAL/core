/*
  tool_change.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Manual tool change with option for automatic touch off

  Part of grblHAL

  Copyright (c) 2020-2025 Terje Io

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

#include <string.h>

#include "hal.h"
#include "motion_control.h"
#include "protocol.h"
#include "tool_change.h"

// NOTE: only used when settings.homing.flags.force_set_origin is true
#ifndef LINEAR_AXIS_HOME_OFFSET
#define LINEAR_AXIS_HOME_OFFSET -1.0f
#endif

#ifndef TOOL_CHANGE_PROBE_RETRACT_DISTANCE
#define TOOL_CHANGE_PROBE_RETRACT_DISTANCE 2.0f
#endif

static bool block_cycle_start, probe_toolsetter, change_at_g30;
static volatile bool execute_posted = false;
static volatile uint32_t spin_lock = 0;
static tool_data_t current_tool = {}, *next_tool = NULL;
static plane_t plane;
static coord_data_t target = {}, previous;

static tool_select_ptr tool_select;
static driver_reset_ptr driver_reset = NULL;
static enqueue_realtime_command_ptr enqueue_realtime_command = NULL;
static control_signals_callback_ptr control_interrupt_callback = NULL;
static on_homing_completed_ptr on_homing_completed = NULL;
static on_probe_completed_ptr on_probe_completed;
static on_wco_saved_ptr on_wco_saved;

// Clear tool length offset on homing
static void onHomingComplete (axes_signals_t homing_cycle, bool success)
{
    if(on_homing_completed)
        on_homing_completed(homing_cycle, success);

    if(settings.tool_change.mode != ToolChange_Disabled)
        system_clear_tlo_reference(homing_cycle);
}

static void onWcoSaved (coord_system_id_t id, coord_data_t *offset)
{
    if(on_wco_saved)
        on_wco_saved(id, offset);

    if(id == gc_state.modal.coord_system.id && block_cycle_start)
        block_cycle_start = change_at_g30;
}

// Set tool offset on successful $TPW probe, prompt for retry on failure.
// Called via probe completed event.
static void onProbeCompleted (void)
{
    if(!sys.flags.probe_succeeded)
        grbl.report.feedback_message(Message_ProbeFailedRetry);
    else if(sys.tlo_reference_set.mask & bit(plane.axis_linear))
        gc_set_tool_offset(ToolLengthOffset_EnableDynamic, plane.axis_linear, sys.probe_position[plane.axis_linear] - sys.tlo_reference[plane.axis_linear]);
//    else error?
}

// Restore HAL pointers on completion or reset.
static void change_completed (void)
{
    if(enqueue_realtime_command) {
        while(spin_lock);
        hal.irq_disable();
        hal.stream.set_enqueue_rt_handler(enqueue_realtime_command);
        enqueue_realtime_command = NULL;
        hal.irq_enable();
    }

    if(control_interrupt_callback) {
        while(spin_lock);
        hal.irq_disable();
        hal.control.interrupt_callback = control_interrupt_callback;
        control_interrupt_callback = NULL;
        hal.irq_enable();
    }

    if(probe_toolsetter)
        grbl.on_probe_toolsetter(&current_tool, NULL, true, false);

    if(grbl.on_probe_completed == onProbeCompleted)
        grbl.on_probe_completed = on_probe_completed;

    gc_state.tool_change = probe_toolsetter = false;

#ifndef NO_SAFETY_DOOR_SUPPORT
    if(hal.control.get_state().safety_door_ajar && hal.signals_cap.safety_door_ajar)
        system_set_exec_state_flag(EXEC_SAFETY_DOOR);
#endif
}

// Reset claimed HAL entry points and restore previous tool if needed on soft restart.
// Called from EXEC_RESET and EXEC_STOP handlers (via HAL).
static void reset (void)
{
    if(next_tool) { //TODO: move to gc_xxx() function?
        // Restore previous tool if reset is during change
        if(current_tool.tool_id != next_tool->tool_id) {
            if(grbl.tool_table.n_tools)
                memcpy(gc_state.tool, &current_tool, sizeof(tool_data_t));
            else
                memcpy(next_tool, &current_tool, sizeof(tool_data_t));
            system_add_rt_report(Report_Tool);
        }
        gc_state.tool_pending = gc_state.tool->tool_id;
        next_tool = NULL;
    }

    change_completed();
    driver_reset();
}

// Restore coolant and spindle status, return controlled point to original position.
static bool restore (void)
{
    bool ok;
    plan_line_data_t plan_data;

    plan_data_init(&plan_data);
    plan_data.condition.rapid_motion = On;

    if(!(ok = (target.values[plane.axis_0] == previous.values[plane.axis_0] &&
                target.values[plane.axis_1] == previous.values[plane.axis_1]))) {

        target.values[plane.axis_linear] = sys.home_position[plane.axis_linear];

        if((ok = mc_line(target.values, &plan_data)) && !settings.flags.no_restore_position_after_M6) {
            memcpy(&target, &previous, sizeof(coord_data_t));
            target.values[plane.axis_linear] = sys.home_position[plane.axis_linear];
            ok = mc_line(target.values, &plan_data);
        }
    }

    if(ok && protocol_buffer_synchronize()) {

        sync_position();

        coolant_restore(gc_state.modal.coolant, settings.coolant.on_delay);
        spindle_t *spindle = gc_spindle_get(-1);
        spindle_restore(spindle->hal, spindle->state, spindle->rpm, settings.spindle.on_delay);

        if(!settings.flags.no_restore_position_after_M6) {
            previous.values[plane.axis_linear] += gc_get_offset(plane.axis_linear, false);
            mc_line(previous.values, &plan_data);
        }
    }

    if(protocol_buffer_synchronize()) {
        sync_position();
        memcpy(&current_tool, next_tool, sizeof(tool_data_t));
    }

    return !ABORTED;
}

static bool go_linear_home (plan_line_data_t *pl_data)
{
    system_convert_array_steps_to_mpos(target.values, sys.position);

    if(target.values[plane.axis_linear] != sys.home_position[plane.axis_linear]) {

        target.values[plane.axis_linear] = sys.home_position[plane.axis_linear];
        if(!mc_line(target.values, pl_data))
            return false;
    }

    return true;
}
#if COMPATIBILITY_LEVEL <= 1

static bool go_toolsetter (plan_line_data_t *pl_data)
{
    // G59.3 contains offsets to toolsetter.
    settings_read_coord_data(CoordinateSystem_G59_3, &target.values);

    float tmp_pos = target.values[plane.axis_linear];

    target.values[plane.axis_linear] = sys.home_position[plane.axis_linear];

    if(probe_toolsetter)
        grbl.on_probe_toolsetter(next_tool, &target, false, true);

    if(!mc_line(target.values, pl_data))
        return false;

    target.values[plane.axis_linear] = tmp_pos;
    if(!mc_line(target.values, pl_data))
        return false;

    if(probe_toolsetter)
        grbl.on_probe_toolsetter(next_tool, NULL, true, true);

    return true;
}

#endif

// Issue warning on cycle start event if touch off by $TPW is pending.
// Used in Manual and Manual_G59_3 modes ($341=1 or $341=2). Called from the foreground process.
static void execute_warning (void *data)
{
    grbl.report.feedback_message(Message_ExecuteTPW);
}

// Execute restore position after tool change, either back to original or to toolsetter for touch off.
// Used when G30 position is used for changing the tool. Called from the foreground process.
static void execute_return_from_g30 (void *data)
{
    bool ok;
    plan_line_data_t plan_data;

    plan_data_init(&plan_data);
    plan_data.condition.rapid_motion = On;

    if((ok = go_linear_home(&plan_data))) {
#if COMPATIBILITY_LEVEL <= 1
        if(settings.tool_change.mode == ToolChange_Manual_G59_3)
            ok = go_toolsetter(&plan_data);
        else
#endif
        {
            // Rapid to original XY position.
            target.values[plane.axis_0] = previous.values[plane.axis_0];
            target.values[plane.axis_1] = previous.values[plane.axis_1];
            ok = mc_line(target.values, &plan_data);
        }
    }

    if(ok) {
        protocol_buffer_synchronize();
        sync_position();
    }

    change_at_g30 = execute_posted = false;
}

// Execute restore position after touch off (on cycle start event).
// Used in Manual and Manual_G59_3 modes ($341=1 or $341=2). Called from the foreground process.
static void execute_restore (void *data)
{
    // Get current position.
    system_convert_array_steps_to_mpos(target.values, sys.position);

    bool ok = restore();

    change_completed();

    grbl.report.feedback_message(Message_None);

    if(ok)
        system_set_exec_state_flag(EXEC_CYCLE_START);
}

// Set and limit probe travel to be within machine limits.
static void set_probe_target (coord_data_t *target, uint8_t axis)
{
    target->values[axis] -= settings.tool_change.probing_distance;

    if(bit_istrue(sys.homed.mask, bit(axis)) && settings.axis[axis].max_travel < -0.0f)
        target->values[axis] = max(min(target->values[axis], sys.work_envelope.max.values[axis]), sys.work_envelope.min.values[axis]);
}

// Execute touch off on cycle start event from @ G59.3 position.
// Used in SemiAutomatic mode ($341=3) only. Called from the foreground process.
static void execute_probe (void *data)
{
#if COMPATIBILITY_LEVEL <= 1
    bool ok;
    coord_data_t offset;
    plan_line_data_t plan_data;
    gc_parser_flags_t flags = {};

    // G59.3 contains offsets to position of TLS.
    settings_read_coord_data(CoordinateSystem_G59_3, &offset.values);

    plan_data_init(&plan_data);
    plan_data.condition.rapid_motion = On;

    ok = !change_at_g30 || go_linear_home(&plan_data);

    target.values[plane.axis_0] = offset.values[plane.axis_0];
    target.values[plane.axis_1] = offset.values[plane.axis_1];

    if(probe_toolsetter)
        grbl.on_probe_toolsetter(next_tool, &target, false, true);

    if((ok = ok && mc_line(target.values, &plan_data))) {

        target.values[plane.axis_linear] = offset.values[plane.axis_linear];
        ok = mc_line(target.values, &plan_data);

        plan_data.feed_rate = settings.tool_change.seek_rate;
        plan_data.condition.value = 0;
        plan_data.spindle.state.value = 0;

        if(ok && probe_toolsetter)
            plan_data.condition.probing_toolsetter = grbl.on_probe_toolsetter(next_tool, NULL, true, true);

        set_probe_target(&target, plane.axis_linear);

        if((ok = ok && mc_probe_cycle(target.values, &plan_data, flags) == GCProbe_Found)) {

            system_convert_array_steps_to_mpos(target.values, sys.probe_position);

            target.values[plane.axis_linear] += TOOL_CHANGE_PROBE_RETRACT_DISTANCE;

            if((flags.probe_is_away = settings.flags.tool_change_fast_pulloff))
                plan_data.feed_rate = settings.tool_change.feed_rate; // Retract slowly until contact lost.
            else {
                // Retract a bit and perform slow probe.
                plan_data.feed_rate = settings.tool_change.pulloff_rate;
                if((ok = mc_line(target.values, &plan_data))) {
                    plan_data.feed_rate = settings.tool_change.feed_rate;
                    target.values[plane.axis_linear] -= (TOOL_CHANGE_PROBE_RETRACT_DISTANCE + 2.0f);
                }
            }
            ok = ok && mc_probe_cycle(target.values, &plan_data, flags) == GCProbe_Found;
        }

        if(ok) {
            if(!(sys.tlo_reference_set.mask & bit(plane.axis_linear))) {
                sys.tlo_reference[plane.axis_linear] = sys.probe_position[plane.axis_linear];
                sys.tlo_reference_set.mask |= bit(plane.axis_linear);
                system_add_rt_report(Report_TLOReference);
                grbl.report.feedback_message(Message_ReferenceTLOEstablished);
            } else
                gc_set_tool_offset(ToolLengthOffset_EnableDynamic, plane.axis_linear,
                                    sys.probe_position[plane.axis_linear] - sys.tlo_reference[plane.axis_linear]);

            ok = restore();
        }
    }

    change_completed();

    if(ok)
        system_set_exec_state_flag(EXEC_CYCLE_START);
#endif
}

// Trap cycle start commands and redirect to foreground process
// by adding the function to be called to the realtime execution queue.
ISR_CODE static void ISR_FUNC(trap_control_cycle_start)(control_signals_t signals)
{
    spin_lock++;

    if(signals.cycle_start) {
        if(!execute_posted) {
            if(!block_cycle_start)
                execute_posted = task_add_immediate(settings.tool_change.mode == ToolChange_SemiAutomatic
                                                     ? execute_probe
                                                     : execute_restore, NULL);
            else if(change_at_g30)
                execute_posted = task_add_immediate(execute_return_from_g30, NULL);
            else
                task_add_immediate(execute_warning, NULL);
        }
        signals.cycle_start = Off;
    } else
        control_interrupt_callback(signals);

    spin_lock--;
}

ISR_CODE static bool ISR_FUNC(trap_stream_cycle_start)(uint8_t c)
{
    bool drop = false;

    spin_lock++;

    if((drop = (c == CMD_CYCLE_START || c == CMD_CYCLE_START_LEGACY))) {
        if(!execute_posted) {
            if(!block_cycle_start)
                execute_posted = task_add_immediate(settings.tool_change.mode == ToolChange_SemiAutomatic
                                                     ? execute_probe
                                                     : execute_restore, NULL);
            else if(change_at_g30)
                execute_posted = task_add_immediate(execute_return_from_g30, NULL);
            else
                task_add_immediate(execute_warning, NULL);
        }
    } else
        drop = enqueue_realtime_command(c);

    spin_lock--;

    return drop;
}

// Trap cycle start command and control signal when tool change is acknowledged by sender.
ISR_CODE static void ISR_FUNC(on_toolchange_ack)(void)
{
    control_interrupt_callback = hal.control.interrupt_callback;
    hal.control.interrupt_callback = trap_control_cycle_start;
    enqueue_realtime_command = hal.stream.set_enqueue_rt_handler(trap_stream_cycle_start);

}

// Set next and/or current tool. Called by gcode.c on on a Tn or M61 command (via HAL).
static void onToolSelect (tool_data_t *tool, bool next)
{
    next_tool = tool;

    if(!next)
        memcpy(&current_tool, tool, sizeof(tool_data_t));

    if(tool_select)
        tool_select(tool, next);
}

// Start a tool change sequence. Called by gcode.c on a M6 command (via HAL).
static status_code_t tool_change (parser_state_t *parser_state)
{
    if(next_tool == NULL)
        return Status_GCodeToolError;

    if(current_tool.tool_id == next_tool->tool_id)
        return Status_OK;

#if COMPATIBILITY_LEVEL > 1
    if(settings.tool_change.mode == ToolChange_Manual_G59_3 || settings.tool_change.mode == ToolChange_SemiAutomatic)
        return Status_GcodeUnsupportedCommand;
#endif

#if TOOL_LENGTH_OFFSET_AXIS >= 0
    plane.axis_linear = TOOL_LENGTH_OFFSET_AXIS;
  #if TOOL_LENGTH_OFFSET_AXIS == X_AXIS
    plane.axis_0 = Y_AXIS;
    plane.axis_1 = Z_AXIS;
  #elif TOOL_LENGTH_OFFSET_AXIS == Y_AXIS
    plane.axis_0 = Z_AXIS;
    plane.axis_1 = X_AXIS;
  #else
    plane.axis_0 = X_AXIS;
    plane.axis_1 = Y_AXIS;
  #endif
#else
    gc_get_plane_data(&plane, parser_state->modal.plane_select);
#endif

    uint8_t homed_req = settings.tool_change.mode == ToolChange_Manual ? bit(plane.axis_linear) : (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT);

    if((sys.homed.mask & homed_req) != homed_req)
        return Status_HomingRequired;

    plan_line_data_t plan_data;
    coord_data_t change_position;

    if(settings.tool_change.mode != ToolChange_SemiAutomatic && grbl.on_probe_completed != onProbeCompleted) {
        on_probe_completed = grbl.on_probe_completed;
        grbl.on_probe_completed = onProbeCompleted;
    }

    block_cycle_start = settings.tool_change.mode != ToolChange_SemiAutomatic;

    // Stop spindle and coolant.
    spindle_all_off();
    hal.coolant.set_state((coolant_state_t){0});

    execute_posted = false;
    probe_toolsetter = grbl.on_probe_toolsetter != NULL &&
                       (settings.tool_change.mode == ToolChange_Manual ||
                         settings.tool_change.mode == ToolChange_Manual_G59_3 ||
                          settings.tool_change.mode == ToolChange_SemiAutomatic);

    // Save current position.
    system_convert_array_steps_to_mpos(previous.values, sys.position);

    // Establish axis assignments.

    previous.values[plane.axis_linear] -= gc_get_offset(plane.axis_linear, false);

    memcpy(&change_position, &previous, sizeof(coord_data_t));

    plan_data_init(&plan_data);
    plan_data.condition.rapid_motion = On;

    // TODO: add?
    //if(!settings.homing.flags.force_set_origin && bit_istrue(settings.homing.dir_mask.value, bit(plane.axis_linear)))
    //    tool_change_position = ?
    //else

    if((change_at_g30 = (settings.flags.tool_change_at_g30) && (sys.homed.mask & (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT)) == (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT)))
        settings_read_coord_data(CoordinateSystem_G30, &change_position.values);
    else
        change_position.values[plane.axis_linear] = sys.home_position[plane.axis_linear]; // - settings.homing.flags.force_set_origin ? LINEAR_AXIS_HOME_OFFSET : 0.0f;

    // Rapid to home position of linear axis.
    if(!go_linear_home(&plan_data))
        return Status_Reset;

    if(change_at_g30) {

        // Rapid to G30 position.
        if(!(target.values[plane.axis_0] == change_position.values[plane.axis_0] &&
              target.values[plane.axis_1] == change_position.values[plane.axis_1])) {

            target.values[plane.axis_0] = change_position.values[plane.axis_0];
            target.values[plane.axis_1] = change_position.values[plane.axis_1];
            if(!mc_line(target.values, &plan_data))
                return Status_Reset;
        }

        if(target.values[plane.axis_linear] != change_position.values[plane.axis_linear]) {

            target.values[plane.axis_linear] = change_position.values[plane.axis_linear];
            if(!mc_line(target.values, &plan_data))
                return Status_Reset;
        }
    }

#if COMPATIBILITY_LEVEL <= 1
    else if(settings.tool_change.mode == ToolChange_Manual_G59_3) {
        if(!go_toolsetter(&plan_data))
            return Status_Reset;
    }
#endif

    protocol_buffer_synchronize();
    sync_position();

    // Enter tool change mode, waits for cycle start to continue.
    parser_state->tool_change = true;
    system_set_exec_state_flag(EXEC_TOOL_CHANGE);   // Set up program pause for manual tool change
    protocol_execute_realtime();                    // Execute...

    return Status_OK;
}

// Claim HAL tool change entry points and clear current tool offsets.
// TODO: change to survive a warm reset?
void tc_init (void)
{
    static bool on_homing_subscribed = false;

    if(hal.tool.atc_get_state() != ATC_None) // Do not override tool change implementation!
        return;

    if(!hal.stream.suspend_read) // Tool change requires support for suspending input stream.
        return;

    system_add_rt_report(Report_TLOReference);

    if(settings.tool_change.mode == ToolChange_Disabled || settings.tool_change.mode == ToolChange_Ignore) {
        hal.tool.change = NULL;
        grbl.on_toolchange_ack = NULL;
    } else {
        hal.tool.change = tool_change;
        grbl.on_toolchange_ack = on_toolchange_ack;
        if(!on_homing_subscribed) {

            on_homing_subscribed = true;

            tool_select = hal.tool.select;
            hal.tool.select = onToolSelect;

            on_homing_completed = grbl.on_homing_completed;
            grbl.on_homing_completed = onHomingComplete;

            on_wco_saved = grbl.on_wco_saved;
            grbl.on_wco_saved = onWcoSaved;
        }
        if(driver_reset == NULL) {
            driver_reset = hal.driver_reset;
            hal.driver_reset = reset;
        }
    }
}

// Perform a probe cycle: set tool length offset and restart job if successful.
// Note: tool length offset is set by the onProbeCompleted event handler.
// Called by the $TPW system command.
status_code_t tc_probe_workpiece (void)
{
    if(!(settings.tool_change.mode == ToolChange_Manual || settings.tool_change.mode == ToolChange_Manual_G59_3) || enqueue_realtime_command == NULL)
        return Status_InvalidStatement;

    if(change_at_g30) {
        grbl.report.feedback_message(Message_CycleStart2TouchOff);
        return Status_OK;
    }

    // TODO: add check for reference offset set?

    bool ok;
    gc_parser_flags_t flags = {};
    plan_line_data_t plan_data;

#if COMPATIBILITY_LEVEL <= 1
    if(probe_toolsetter)
        plan_data.condition.probing_toolsetter = grbl.on_probe_toolsetter(next_tool, NULL, system_xy_at_fixture(CoordinateSystem_G59_3, TOOLSETTER_RADIUS), true);
#endif

    // Get current position.
    system_convert_array_steps_to_mpos(target.values, sys.position);

    flags.probe_is_no_error = On;

    plan_data_init(&plan_data);
    plan_data.feed_rate = settings.tool_change.seek_rate;

    set_probe_target(&target, plane.axis_linear);

    if((ok = mc_probe_cycle(target.values, &plan_data, flags) == GCProbe_Found))
    {
        system_convert_array_steps_to_mpos(target.values, sys.probe_position);

        target.values[plane.axis_linear] += TOOL_CHANGE_PROBE_RETRACT_DISTANCE;

        if((flags.probe_is_away = settings.flags.tool_change_fast_pulloff))
            plan_data.feed_rate = settings.tool_change.feed_rate; // Retract slowly until contact lost.
        else {
            // Retract a bit before performing slow probe.
            plan_data.feed_rate = settings.tool_change.pulloff_rate;
            if((ok = mc_line(target.values, &plan_data))) {
                plan_data.feed_rate = settings.tool_change.feed_rate;
                target.values[plane.axis_linear] -= (TOOL_CHANGE_PROBE_RETRACT_DISTANCE + 2.0f);
            }
        }

        if((ok = ok && mc_probe_cycle(target.values, &plan_data, flags) == GCProbe_Found)) {
            // Retract a bit again so that any touch plate can be removed
            system_convert_array_steps_to_mpos(target.values, sys.probe_position);
            plan_data.feed_rate = settings.tool_change.seek_rate;
            target.values[plane.axis_linear] += TOOL_CHANGE_PROBE_RETRACT_DISTANCE * 2.0f;
            if(target.values[plane.axis_linear] > sys.home_position[plane.axis_linear])
                target.values[plane.axis_linear] = sys.home_position[plane.axis_linear];
            ok = mc_line(target.values, &plan_data);
        }
    }

    if(ok && protocol_buffer_synchronize()) {
        sync_position();
        block_cycle_start = false;
        grbl.report.feedback_message(settings.tool_change.mode == ToolChange_Manual_G59_3
                                      ? Message_CycleStart2Continue
                                      : Message_TPCycleStart2Continue);
    }

    return ok ? Status_OK : Status_GCodeToolError;
}
