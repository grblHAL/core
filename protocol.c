/*
  protocol.c - controls grblHAL execution protocol and procedures

  Part of grblHAL

  Copyright (c) 2017-2024 Terje Io
  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
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

#include <stdlib.h>
#include <string.h>

#include "hal.h"
#include "nuts_bolts.h"
#include "nvs_buffer.h"
#include "override.h"
#include "state_machine.h"
#include "motion_control.h"
#include "sleep.h"
#include "protocol.h"
#include "machine_limits.h"

#ifndef RT_QUEUE_SIZE
#define RT_QUEUE_SIZE 16 // must be a power of 2
#endif

// Define line flags. Includes comment type tracking and line overflow detection.
typedef union {
    uint8_t value;
    struct {
        uint8_t overflow            :1,
                comment_parentheses :1,
                comment_semicolon   :1,
                unassigned          :5;
    };
} line_flags_t;

typedef struct {
    void *data;
    fg_task_ptr task;
} delayed_task_t;

typedef struct {
    volatile uint_fast8_t head;
    volatile uint_fast8_t tail;
    delayed_task_t task[RT_QUEUE_SIZE];
} realtime_queue_t;

static uint_fast16_t char_counter = 0;
static char line[LINE_BUFFER_SIZE]; // Line to be executed. Zero-terminated.
static char xcommand[LINE_BUFFER_SIZE];
static bool keep_rt_commands = false;
static realtime_queue_t realtime_queue = {0};
static on_execute_realtime_ptr on_execute_delay;

static void protocol_execute_rt_commands (sys_state_t state);
static void protocol_exec_rt_suspend (sys_state_t state);

// add gcode to execute not originating from normal input stream
bool protocol_enqueue_gcode (char *gcode)
{
    bool ok = xcommand[0] == '\0' &&
               (state_get() == STATE_IDLE || (state_get() & (STATE_ALARM|STATE_JOG|STATE_TOOL_CHANGE))) &&
                 bit_isfalse(sys.rt_exec_state, EXEC_MOTION_CANCEL);

    if(ok && gc_state.file_run)
        ok = gc_state.modal.program_flow != ProgramFlow_Running || strncmp((char *)gcode, "$J=", 3);

    if(ok)
        strcpy(xcommand, gcode);

    return ok;
}

static void protocol_on_execute_delay (sys_state_t state)
{
    if(sys.rt_exec_state & EXEC_RT_COMMAND) {
        system_clear_exec_state_flag(EXEC_RT_COMMAND);
        protocol_execute_rt_commands(0);
    }

    on_execute_delay(state);
}

static bool recheck_line (char *line, line_flags_t *flags)
{
    bool keep_rt_commands = false, first_char = true;

    flags->value = 0;

    if(*line != '\0') do {

        switch(*line) {

            case '$':
            case '[':
                if(first_char)
                    keep_rt_commands = true;
                break;

            case '(':
                if(!keep_rt_commands && (flags->comment_parentheses = !flags->comment_semicolon))
                    keep_rt_commands = !hal.driver_cap.no_gcode_message_handling; // Suspend real-time processing of printable command characters.
                break;

            case ')':
                if(!flags->comment_semicolon)
                    flags->comment_parentheses = keep_rt_commands = false;
                break;

            case ';':
                if(!flags->comment_parentheses) {
                    keep_rt_commands = false;
                    flags->comment_semicolon = On;
                }
                break;
        }

        first_char = false;

    } while(*++line != '\0');

    return keep_rt_commands;
}

/*
  GRBL PRIMARY LOOP:
*/
bool protocol_main_loop (void)
{
    if(sys.alarm == Alarm_SelftestFailed) {
        sys.alarm = Alarm_None;
        system_raise_alarm(Alarm_SelftestFailed);
    } else if (hal.control.get_state().e_stop) {
        // Check for e-stop active. Blocks everything until cleared.
        system_raise_alarm(Alarm_EStop);
        grbl.report.feedback_message(Message_EStop);
    } else if(hal.control.get_state().motor_fault) {
        // Check for motor fault active. Blocks everything until cleared.
        system_raise_alarm(Alarm_MotorFault);
        grbl.report.feedback_message(Message_MotorFault);
    } else if(settings.probe.enable_protection && hal.control.get_state().probe_triggered) {
        system_raise_alarm(Alarm_ProbeProtect);
        grbl.report.feedback_message(Message_ProbeProtected);
    } else if (limits_homing_required()) {
        // Check for power-up and set system alarm if homing is enabled to force homing cycle
        // by setting grblHAL's alarm state. Alarm locks out all g-code commands, including the
        // startup scripts, but allows access to settings and internal commands.
        // Only a successful homing cycle '$H' will disable the alarm.
        // NOTE: The startup script will run after successful completion of the homing cycle. Prevents motion startup
        // blocks from crashing into things uncontrollably. Very bad.
        system_raise_alarm(Alarm_HomingRequired);
        grbl.report.feedback_message(Message_HomingCycleRequired);
    } else if (settings.limits.flags.hard_enabled &&
                settings.limits.flags.check_at_init &&
                 (limit_signals_merge(hal.limits.get_state()).value & sys.hard_limits.mask)) {
        if(sys.alarm == Alarm_LimitsEngaged && hal.control.get_state().limits_override)
            state_set(STATE_IDLE); // Clear alarm state to enable limit switch pulloff.
        else {
            // Check that no limit switches are engaged to make sure everything is good to go.
            system_raise_alarm(Alarm_LimitsEngaged);
            grbl.report.feedback_message(Message_CheckLimits);
        }
    } else if(sys.cold_start && (settings.flags.force_initialization_alarm || hal.control.get_state().reset)) {
        state_set(STATE_ALARM); // Ensure alarm state is set.
        grbl.report.feedback_message(Message_AlarmLock);
    } else if (state_get() & (STATE_ALARM|STATE_SLEEP)) {
        // Check for and report alarm state after a reset, error, or an initial power up.
        // NOTE: Sleep mode disables the stepper drivers and position can't be guaranteed.
        // Re-initialize the sleep state as an ALARM mode to ensure user homes or acknowledges.
        if(sys.alarm == Alarm_HomingRequired)
            sys.alarm = Alarm_None; // Clear Alarm_HomingRequired as the lock has been overridden by a soft reset.
        state_set(STATE_ALARM); // Ensure alarm state is set.
        grbl.report.feedback_message(Message_AlarmLock);
    } else {
        state_set(STATE_IDLE);
#ifndef NO_SAFETY_DOOR_SUPPORT
        // Check if the safety door is open.
        if (hal.signals_cap.safety_door_ajar && !settings.safety_door.flags.ignore_when_idle && hal.control.get_state().safety_door_ajar) {
            system_set_exec_state_flag(EXEC_SAFETY_DOOR);
            protocol_execute_realtime(); // Enter safety door mode. Should return as IDLE state.
        }
#endif
        // All systems go!
        system_execute_startup(); // Execute startup script.
    }

    // Ensure spindle and coolant is switched off on a cold start
    if(sys.cold_start) {
        spindle_all_off();
        hal.coolant.set_state((coolant_state_t){0});
        if(realtime_queue.head != realtime_queue.tail)
            system_set_exec_state_flag(EXEC_RT_COMMAND);  // execute any boot up commands
        on_execute_delay = grbl.on_execute_delay;
        grbl.on_execute_delay = protocol_on_execute_delay;
        sys.cold_start = false;
    } else // TODO: if flushing entries from the queue that has allocated data associated then these will be orphaned/leaked.
        memset(&realtime_queue, 0, sizeof(realtime_queue_t));

    // ---------------------------------------------------------------------------------
    // Primary loop! Upon a system abort, this exits back to main() to reset the system.
    // This is also where grblHAL idles while waiting for something to do.
    // ---------------------------------------------------------------------------------

    int16_t c;
    char eol = '\0';
    line_flags_t line_flags = {0};

    xcommand[0] = '\0';
    char_counter = 0;
    keep_rt_commands = false;

    while(true) {

        // Process one line of incoming stream data, as the data becomes available. Performs an
        // initial filtering by removing leading spaces and control characters.
        while((c = hal.stream.read()) != SERIAL_NO_DATA) {

            if(c == ASCII_CAN) {

                eol = xcommand[0] = '\0';
                keep_rt_commands = false;
                char_counter = line_flags.value = 0;
                gc_state.last_error = Status_OK;

                if (state_get() == STATE_JOG) // Block all other states from invoking motion cancel.
                    system_set_exec_state_flag(EXEC_MOTION_CANCEL);

            } else if(c == ASCII_EOF) {
                if(grbl.on_file_end)
                    grbl.on_file_end(hal.stream.file, gc_state.last_error);
            } else if ((c == '\n') || (c == '\r')) { // End of line reached

                // Check for possible secondary end of line character, do not process as empty line
                // if part of crlf (or lfcr pair) as this produces a possibly unwanted double response
                if(char_counter == 0 && eol && eol != c) {
                    eol = '\0';
                    continue;
                } else
                    eol = (char)c;

                if(!protocol_execute_realtime()) // Runtime command check point.
                    return !sys.flags.exit;      // Bail to calling function upon system abort

                line[char_counter] = '\0'; // Set string termination character.

              #if REPORT_ECHO_LINE_RECEIVED
                report_echo_line_received(line);
              #endif

                // Direct and execute one line of formatted input, and report status of execution.
                if (line_flags.overflow) // Report line overflow error.
                    gc_state.last_error = Status_Overflow;
                else if(*line == '\0') // Empty line. For syncing purposes.
                    gc_state.last_error = Status_OK;
                else if(*line == '$') {// grblHAL '$' system command
                    if((gc_state.last_error = system_execute_line(line)) == Status_LimitsEngaged) {
                        system_raise_alarm(Alarm_LimitsEngaged);
                        grbl.report.feedback_message(Message_CheckLimits);
                    }
                } else if(*line == '[' && grbl.on_user_command)
                    gc_state.last_error = grbl.on_user_command(line);
                else if (state_get() & (STATE_ALARM|STATE_ESTOP|STATE_JOG)) // Everything else is gcode. Block if in alarm, eStop or jog mode.
                    gc_state.last_error = Status_SystemGClock;
#if COMPATIBILITY_LEVEL == 0
                else if(gc_state.last_error == Status_OK || gc_state.last_error == Status_GcodeToolChangePending) { // Parse and execute g-code block.
#else
                else { // Parse and execute g-code block.

#endif
                    if((gc_state.last_error = gc_execute_block(line)) != Status_OK)
                        eol = '\0';
                }

                // Add a short delay for each block processed in Check Mode to
                // avoid overwhelming the sender with fast reply messages.
                // This is likely to happen when streaming is done via a protocol where
                // the speed is not limited to 115200 baud. An example is native USB streaming.
#if CHECK_MODE_DELAY
                if(state_get() == STATE_CHECK_MODE)
                    hal.delay_ms(CHECK_MODE_DELAY, NULL);
#endif
                if(ABORTED)
                    break;
                else
                    grbl.report.status_message(gc_state.last_error);

                // Reset tracking data for next line.
                keep_rt_commands = false;
                char_counter = line_flags.value = 0;

            } else if (c != ASCII_BS && c <= (char_counter > 0 ? ' ' - 1 : ' '))
                continue; // Strip control characters and leading whitespace.
            else {
                switch(c) {

                    case '$':
                    case '[':
                        if(char_counter == 0)
                            keep_rt_commands = true;
                        break;

                    case '(':
                        if(!keep_rt_commands && (line_flags.comment_parentheses = !line_flags.comment_semicolon))
                            keep_rt_commands = !hal.driver_cap.no_gcode_message_handling; // Suspend real-time processing of printable command characters.
                        break;

                    case ')':
                        if(!line_flags.comment_semicolon)
                            line_flags.comment_parentheses = keep_rt_commands = false;
                        break;

                    case ';':
                        if(!line_flags.comment_parentheses) {
                            keep_rt_commands = false;
                            line_flags.comment_semicolon = On;
                        }
                        break;

                    case ASCII_BS:
                    case ASCII_DEL:
                        if(char_counter) {
                            line[--char_counter] = '\0';
                            keep_rt_commands = recheck_line(line, &line_flags);
                        }
                        continue;
                }
                if(!(line_flags.overflow = char_counter >= (LINE_BUFFER_SIZE - 1)))
                    line[char_counter++] = c;
            }
        }

        // Handle extra command (internal stream)
        if(xcommand[0] != '\0') {

            if (xcommand[0] == '$') // grblHAL '$' system command
                system_execute_line(xcommand);
            else if (state_get() & (STATE_ALARM|STATE_ESTOP|STATE_JOG)) // Everything else is gcode. Block if in alarm, eStop or jog state.
                grbl.report.status_message(Status_SystemGClock);
            else // Parse and execute g-code block.
                gc_execute_block(xcommand);

            xcommand[0] = '\0';
        }

        // If there are no more characters in the input stream buffer to be processed and executed,
        // this indicates that g-code streaming has either filled the planner buffer or has
        // completed. In either case, auto-cycle start, if enabled, any queued moves.
        protocol_auto_cycle_start();

        if(sys.abort || !protocol_execute_realtime()) // Runtime command check point.
            return !sys.flags.exit;                   // Bail to main() program loop to reset system.

        sys.cancel = false;

        // Check for sleep conditions and execute auto-park, if timeout duration elapses.
        if(settings.flags.sleep_enable)
            sleep_check();
    }
}


// Block until all buffered steps are executed or in a cycle state. Works with feed hold
// during a synchronize call, if it should happen. Also, waits for clean cycle end.
bool protocol_buffer_synchronize (void)
{
    bool ok = true;

    // If system is queued, ensure cycle resumes if the auto start flag is present.
    protocol_auto_cycle_start();

    sys.flags.synchronizing = On;
    while ((ok = protocol_execute_realtime()) && (plan_get_current_block() || state_get() == STATE_CYCLE));
    sys.flags.synchronizing = Off;

    return ok;
}


// Auto-cycle start triggers when there is a motion ready to execute and if the main program is not
// actively parsing commands.
// NOTE: This function is called from the main loop, buffer sync, and mc_line() only and executes
// when one of these conditions exist respectively: There are no more blocks sent (i.e. streaming
// is finished, single commands), a command that needs to wait for the motions in the buffer to
// execute calls a buffer sync, or the planner buffer is full and ready to go.
void protocol_auto_cycle_start (void)
{
    if (plan_get_current_block() != NULL) // Check if there are any blocks in the buffer.
        system_set_exec_state_flag(EXEC_CYCLE_START); // If so, execute them!
}


// This function is the general interface to grblHAL's real-time command execution system. It is called
// from various check points in the main program, primarily where there may be a while loop waiting
// for a buffer to clear space or any point where the execution time from the last check point may
// be more than a fraction of a second. This is a way to execute realtime commands asynchronously
// (aka multitasking) with grbl's g-code parsing and planning functions. This function also serves
// as an interface for the interrupts to set the system realtime flags, where only the main program
// handles them, removing the need to define more computationally-expensive volatile variables. This
// also provides a controlled way to execute certain tasks without having two or more instances of
// the same task, such as the planner recalculating the buffer upon a feedhold or overrides.
// NOTE: The sys_rt_exec_state variable flags are set by any process, step or input stream events, pinouts,
// limit switches, or the main program.
// Returns false if aborted
bool protocol_execute_realtime (void)
{
    if(protocol_exec_rt_system()) {

        sys_state_t state = state_get();

        if(sys.suspend)
            protocol_exec_rt_suspend(state);

#if NVSDATA_BUFFER_ENABLE
        if((state == STATE_IDLE || (state & (STATE_ALARM|STATE_ESTOP))) && settings_dirty.is_dirty && !gc_state.file_run)
            nvs_buffer_sync_physical();
#endif
    }

    return !ABORTED;
}

static void protocol_poll_cmd (void)
{
    int16_t c;

    if((c = hal.stream.read()) != SERIAL_NO_DATA) {

        if ((c == '\n') || (c == '\r')) { // End of line reached
            line[char_counter] = '\0';
            gc_state.last_error = *line == '\0' ? Status_OK : (*line == '$' ? system_execute_line(line) : Status_SystemGClock);
            char_counter = 0;
            *line = '\0';
            grbl.report.status_message(gc_state.last_error);
        } else if(c == ASCII_DEL || c == ASCII_BS) {
            if(char_counter)
                line[--char_counter] = '\0';
        } else if(char_counter == 0 ? c != ' ' : char_counter < (LINE_BUFFER_SIZE - 1))
            line[char_counter++] = c;

        keep_rt_commands = char_counter > 0 && *line == '$';
    }
}

// Executes run-time commands, when required. This function primarily operates as grblHAL's state
// machine and controls the various real-time features grblHAL has to offer.
// NOTE: Do not alter this unless you know exactly what you are doing!
bool protocol_exec_rt_system (void)
{
    rt_exec_t rt_exec;
    bool killed = false;

    if (sys.rt_exec_alarm && (rt_exec = system_clear_exec_alarm())) { // Enter only if any bit flag is true

        if((sys.reset_pending = !!(sys.rt_exec_state & EXEC_RESET))) {
            // Kill spindle and coolant.
            killed = true;
            spindle_all_off();
            hal.coolant.set_state((coolant_state_t){0});
        }

        // System alarm. Everything has shutdown by something that has gone severely wrong. Report
        // the source of the error to the user. If critical, grblHAL disables by entering an infinite
        // loop until system reset/abort.
        system_raise_alarm((alarm_code_t)rt_exec);

        if(killed) // Tell driver/plugins about reset.
            hal.driver_reset();

        // Halt everything upon a critical event flag. Currently hard and soft limits flag this.
        if((sys.blocking_event = (alarm_code_t)rt_exec == Alarm_HardLimit ||
                                  (alarm_code_t)rt_exec == Alarm_SoftLimit ||
                                   (alarm_code_t)rt_exec == Alarm_EStop ||
                                    (alarm_code_t)rt_exec == Alarm_MotorFault)) {

            system_set_exec_alarm(rt_exec);

            switch((alarm_code_t)rt_exec) {

                case Alarm_EStop:
                    grbl.report.feedback_message(Message_EStop);
                    break;

                case Alarm_MotorFault:
                    grbl.report.feedback_message(Message_MotorFault);
                    break;

                default:
                    grbl.report.feedback_message(Message_CriticalEvent);
                    break;
            }

            system_clear_exec_state_flag(EXEC_RESET); // Disable any existing reset

            *line = '\0';
            char_counter = 0;
            hal.stream.reset_read_buffer();

            while (bit_isfalse(sys.rt_exec_state, EXEC_RESET)) {

                // Block everything, except reset and status reports, until user issues reset or power
                // cycles. Hard limits typically occur while unattended or not paying attention. Gives
                // the user and a GUI time to do what is needed before resetting, like killing the
                // incoming stream. The same could be said about soft limits. While the position is not
                // lost, continued streaming could cause a serious crash if by chance it gets executed.

                if(bit_istrue(sys.rt_exec_state, EXEC_STATUS_REPORT)) {
                    system_clear_exec_state_flag(EXEC_STATUS_REPORT);
                    report_realtime_status();
                }

                protocol_poll_cmd();
                grbl.on_execute_realtime(STATE_ESTOP);
            }

            system_clear_exec_alarm(); // Clear alarm
        }
    }

    if (sys.rt_exec_state && (rt_exec = system_clear_exec_states())) { // Get and clear volatile sys.rt_exec_state atomically.

        // Execute system abort.
        if((sys.reset_pending = !!(rt_exec & EXEC_RESET))) {

            if(!killed) {
                // Kill spindle and coolant.
                spindle_all_off();
                hal.coolant.set_state((coolant_state_t){0});
            }

            // Only place sys.abort is set true, when E-stop is not asserted.
            if(!(sys.abort = !hal.control.get_state().e_stop)) {
                hal.stream.reset_read_buffer();
                system_raise_alarm(Alarm_EStop);
                grbl.report.feedback_message(Message_EStop);
            } else if(hal.control.get_state().motor_fault) {
                sys.abort = false;
                hal.stream.reset_read_buffer();
                system_raise_alarm(Alarm_MotorFault);
                grbl.report.feedback_message(Message_MotorFault);
            }

            if(!killed) // Tell driver/plugins about reset.
                hal.driver_reset();

            return !sys.abort; // Nothing else to do but exit.
        }

        if(rt_exec & EXEC_STOP) { // Experimental for now, must be verified. Do NOT move to interrupt context!

            sys.cancel = true;
            sys.step_control.flags = 0;
            sys.flags.feed_hold_pending = Off;
            sys.override_delay.flags = 0;
            if(sys.override.control.sync)
                sys.override.control = gc_state.modal.override_ctrl;

            gc_state.tool_change = false;

            // Tell driver/plugins about reset.
            hal.driver_reset();

            if(!sys.flags.keep_input && hal.stream.suspend_read && hal.stream.suspend_read(false))
                hal.stream.cancel_read_buffer(); // flush pending blocks (after M6)

            sys.flags.keep_input = Off;

            gc_init(true);
            plan_reset();
            if(sys.alarm_pending == Alarm_ProbeProtect) {
                st_go_idle();
                system_set_exec_alarm(sys.alarm_pending);
                sys.alarm_pending = Alarm_None;
            } else
                st_reset();
            sync_position();

            // Kill spindle and coolant. TODO: Check Mach3 behaviour?
            gc_spindle_off();
            gc_coolant((coolant_state_t){0});

            flush_override_buffers();
            if(!((state_get() == STATE_ALARM) && (sys.alarm == Alarm_LimitsEngaged || sys.alarm == Alarm_HomingRequired))) {
                state_set(hal.control.get_state().safety_door_ajar ? STATE_SAFETY_DOOR : STATE_IDLE);
                grbl.report.feedback_message(Message_Stop);
            }
        }

        // Execute and print status to output stream
        if (rt_exec & EXEC_STATUS_REPORT)
            report_realtime_status();

        if(rt_exec & EXEC_GCODE_REPORT)
            report_gcode_modes();

        if(rt_exec & EXEC_TLO_REPORT)
            report_tool_offsets();

        // Execute and print PID log to output stream
        if (rt_exec & EXEC_PID_REPORT)
            report_pid_log();

        if(rt_exec & EXEC_RT_COMMAND)
            protocol_execute_rt_commands(0);

        rt_exec &= ~(EXEC_STOP|EXEC_STATUS_REPORT|EXEC_GCODE_REPORT|EXEC_PID_REPORT|EXEC_TLO_REPORT|EXEC_RT_COMMAND); // clear requests already processed

        if(sys.flags.feed_hold_pending) {
            if(rt_exec & EXEC_CYCLE_START)
                sys.flags.feed_hold_pending = Off;
            else if(!sys.override.control.feed_hold_disable)
                rt_exec |= EXEC_FEED_HOLD;
        }

        // Let state machine handle any remaining requests
        if(rt_exec)
            state_update(rt_exec);
    }

    grbl.on_execute_realtime(state_get());

    // Execute overrides.

    if(!sys.override_delay.feedrate && (rt_exec = get_feed_override())) {

        override_t new_f_override = sys.override.feed_rate;
        override_t new_r_override = sys.override.rapid_rate;

        do {

            switch(rt_exec) {

                case CMD_OVERRIDE_FEED_RESET:
                    new_f_override = DEFAULT_FEED_OVERRIDE;
                    break;

                case CMD_OVERRIDE_FEED_COARSE_PLUS:
                    new_f_override += FEED_OVERRIDE_COARSE_INCREMENT;
                    break;

                case CMD_OVERRIDE_FEED_COARSE_MINUS:
                    new_f_override -= FEED_OVERRIDE_COARSE_INCREMENT;
                    break;

                case CMD_OVERRIDE_FEED_FINE_PLUS:
                    new_f_override += FEED_OVERRIDE_FINE_INCREMENT;
                    break;

                case CMD_OVERRIDE_FEED_FINE_MINUS:
                    new_f_override -= FEED_OVERRIDE_FINE_INCREMENT;
                    break;

                case CMD_OVERRIDE_RAPID_RESET:
                    new_r_override = DEFAULT_RAPID_OVERRIDE;
                    break;

                case CMD_OVERRIDE_RAPID_MEDIUM:
                    new_r_override = RAPID_OVERRIDE_MEDIUM;
                    break;

                case CMD_OVERRIDE_RAPID_LOW:
                    new_r_override = RAPID_OVERRIDE_LOW;
                    break;
            }

            new_f_override = constrain(new_f_override, MIN_FEED_RATE_OVERRIDE, MAX_FEED_RATE_OVERRIDE);

        } while((rt_exec = get_feed_override()));

        plan_feed_override(new_f_override, new_r_override);
    }

    if(!sys.override_delay.spindle && (rt_exec = get_spindle_override())) {

        bool spindle_stop = false;
        spindle_ptrs_t *spindle = gc_spindle_get(-1)->hal;
        override_t last_s_override = spindle->param->override_pct;

        do {

            switch(rt_exec) {

                case CMD_OVERRIDE_SPINDLE_RESET:
                    last_s_override = DEFAULT_SPINDLE_RPM_OVERRIDE;
                    break;

                case CMD_OVERRIDE_SPINDLE_COARSE_PLUS:
                    last_s_override += SPINDLE_OVERRIDE_COARSE_INCREMENT;
                    break;

                case CMD_OVERRIDE_SPINDLE_COARSE_MINUS:
                    last_s_override -= SPINDLE_OVERRIDE_COARSE_INCREMENT;
                    break;

                case CMD_OVERRIDE_SPINDLE_FINE_PLUS:
                    last_s_override += SPINDLE_OVERRIDE_FINE_INCREMENT;
                    break;

                case CMD_OVERRIDE_SPINDLE_FINE_MINUS:
                    last_s_override -= SPINDLE_OVERRIDE_FINE_INCREMENT;
                    break;

                case CMD_OVERRIDE_SPINDLE_STOP:
                    spindle_stop = !spindle_stop;
                    break;

                default:
                    if(grbl.on_unknown_accessory_override)
                        grbl.on_unknown_accessory_override(rt_exec);
                    break;
            }

            last_s_override = constrain(last_s_override, MIN_SPINDLE_RPM_OVERRIDE, MAX_SPINDLE_RPM_OVERRIDE);

        } while((rt_exec = get_spindle_override()));

        spindle_set_override(spindle, last_s_override);

        if (spindle_stop && state_get() == STATE_HOLD && gc_spindle_get(-1)->state.on) {
            // Spindle stop override allowed only while in HOLD state.
            // NOTE: Report flag is set in spindle_set_state() when spindle stop is executed.
            if (!sys.override.spindle_stop.value)
                sys.override.spindle_stop.initiate = On;
            else if (sys.override.spindle_stop.enabled)
                sys.override.spindle_stop.restore = On;
        }
    }

    if(!sys.override_delay.coolant && (rt_exec = get_coolant_override())) {

        coolant_state_t coolant_state = gc_state.modal.coolant;

        do {

            switch(rt_exec) {

                case CMD_OVERRIDE_COOLANT_MIST_TOGGLE:
                    if(hal.coolant_cap.mist && ((state_get() == STATE_IDLE) || (state_get() & (STATE_CYCLE | STATE_HOLD))))
                        coolant_state.mist = !coolant_state.mist;
                    break;

                case CMD_OVERRIDE_COOLANT_FLOOD_TOGGLE:
                    if(hal.coolant_cap.flood && ((state_get() == STATE_IDLE) || (state_get() & (STATE_CYCLE | STATE_HOLD))))
                        coolant_state.flood = !coolant_state.flood;
                    break;

                default:
                    if(grbl.on_unknown_accessory_override)
                        grbl.on_unknown_accessory_override(rt_exec);
                    break;
            }

        } while((rt_exec = get_coolant_override()));

      // NOTE: Since coolant state always performs a planner sync whenever it changes, the current
      // run state can be determined by checking the parser state.
        if(coolant_state.value != gc_state.modal.coolant.value) {
            gc_coolant(coolant_state); // Report flag set in gc_coolant().
            if(grbl.on_override_changed)
                grbl.on_override_changed(OverrideChanged_CoolantState);
        }
    }

    // End execute overrides.

    // Reload step segment buffer
    if (state_get() & (STATE_CYCLE | STATE_HOLD | STATE_SAFETY_DOOR | STATE_HOMING | STATE_SLEEP| STATE_JOG))
        st_prep_buffer();

    return !ABORTED;
}

// Handles grblHAL system suspend procedures, such as feed hold, safety door, and parking motion.
// The system will enter this loop, create local variables for suspend tasks, and return to
// whatever function that invoked the suspend, such that grblHAL resumes normal operation.
// This function is written in a way to promote custom parking motions. Simply use this as a
// template.
static void protocol_exec_rt_suspend (sys_state_t state)
{
    if((sys.blocking_event = state == STATE_SLEEP)) {
        *line = '\0';
        char_counter = 0;
        hal.stream.reset_read_buffer();
    }

    while(sys.suspend) {

        if(sys.abort)
            return;

        if(sys.blocking_event)
            protocol_poll_cmd();

        // Handle spindle overrides during suspend
        state_suspend_manager();

        // If door closed keep issuing door closed requests until resumed
        if(state_get() == STATE_SAFETY_DOOR && !hal.control.get_state().safety_door_ajar)
            system_set_exec_state_flag(EXEC_DOOR_CLOSED);

        // Check for sleep conditions and execute auto-park, if timeout duration elapses.
        // Sleep is valid for both hold and door states, if the spindle or coolant are on or
        // set to be re-enabled.
        if(settings.flags.sleep_enable)
            sleep_check();

        protocol_exec_rt_system();
    }
}

// Pick off (drop) real-time command characters from input stream.
// These characters are not passed into the main buffer,
// but rather sets system state flag bits for later execution by protocol_exec_rt_system().
// Called from input stream interrupt handler.
ISR_CODE bool ISR_FUNC(protocol_enqueue_realtime_command)(char c)
{
    static bool esc = false;

    bool drop = false;

    // 1. Process characters in the ranges 0x - 1x and 8x-Ax
    // Characters with functions assigned are always acted upon even when the input stream
    // is redirected to a non-interactive stream such as from a SD card.

    switch ((unsigned char)c) {

        case '\n':
        case '\r':
            break;

        case '$':
            if(char_counter == 0)
                keep_rt_commands = !settings.flags.legacy_rt_commands;
            break;

        case CMD_STOP:
            system_set_exec_state_flag(EXEC_STOP);
            char_counter = 0;
            hal.stream.cancel_read_buffer();
            drop = true;
            break;

        case CMD_RESET: // Call motion control reset routine.
            if(!hal.control.get_state().e_stop)
                mc_reset();
            drop = true;
            break;

#if COMPATIBILITY_LEVEL == 0
        case CMD_EXIT: // Call motion control reset routine.
            mc_reset();
            sys.flags.exit = On;
            drop = true;
            break;
#endif

        case CMD_STATUS_REPORT_ALL: // Add all statuses to report
            {
                report_tracking_flags_t report;

                report.value = (uint32_t)Report_All;
                report.tool_offset = sys.report.tool_offset;
                report.m66result = sys.var5399 > -2;

                system_add_rt_report((report_tracking_t)report.value);
            }
            system_set_exec_state_flag(EXEC_STATUS_REPORT);
            drop = true;
            break;

        case CMD_STATUS_REPORT:
        case 0x05:
            if(!sys.flags.auto_reporting)
                system_set_exec_state_flag(EXEC_STATUS_REPORT);
            drop = true;
            break;

        case CMD_CYCLE_START:
            system_set_exec_state_flag(EXEC_CYCLE_START);
            // Cancel any pending tool change
            gc_state.tool_change = false;
            drop = true;
            break;

        case CMD_FEED_HOLD:
            system_set_exec_state_flag(EXEC_FEED_HOLD);
            drop = true;
            break;

        case CMD_SAFETY_DOOR:
            if(state_get() != STATE_SAFETY_DOOR) {
                system_set_exec_state_flag(EXEC_SAFETY_DOOR);
                drop = true;
            }
            break;

        case CMD_JOG_CANCEL:
            char_counter = 0;
            drop = true;
            hal.stream.cancel_read_buffer();
#ifdef KINEMATICS_API // needed when kinematics algorithm segments long jog distances (as it blocks reading from input stream)
            if (state_get() & STATE_JOG) // Block all other states from invoking motion cancel.
                system_set_exec_state_flag(EXEC_MOTION_CANCEL);
#endif
            if(grbl.on_jog_cancel)
                grbl.on_jog_cancel(state_get());
            break;

        case CMD_GCODE_REPORT:
            system_set_exec_state_flag(EXEC_GCODE_REPORT);
            drop = true;
            break;

        case CMD_PROBE_CONNECTED_TOGGLE:
            if(hal.probe.connected_toggle)
                hal.probe.connected_toggle();
            break;

        case CMD_OPTIONAL_STOP_TOGGLE:
            if(!hal.signals_cap.stop_disable) // Not available as realtime command if HAL supports physical switch
                sys.flags.optional_stop_disable = !sys.flags.optional_stop_disable;
            break;

        case CMD_SINGLE_BLOCK_TOGGLE:
            if(!hal.signals_cap.single_block) // Not available as realtime command if HAL supports physical switch
                sys.flags.single_block = !sys.flags.single_block;
            break;

        case CMD_PID_REPORT:
            system_set_exec_state_flag(EXEC_PID_REPORT);
            drop = true;
            break;

        case CMD_MPG_MODE_TOGGLE:           // Switch off MPG mode
            if((drop = hal.stream.type == StreamType_MPG))
                protocol_enqueue_foreground_task(stream_mpg_set_mode, NULL);
            break;

        case CMD_AUTO_REPORTING_TOGGLE:
            if((drop = settings.report_interval != 0))
                sys.flags.auto_reporting = !sys.flags.auto_reporting;
            break;

        case CMD_OVERRIDE_FEED_RESET:
        case CMD_OVERRIDE_FEED_COARSE_PLUS:
        case CMD_OVERRIDE_FEED_COARSE_MINUS:
        case CMD_OVERRIDE_FEED_FINE_PLUS:
        case CMD_OVERRIDE_FEED_FINE_MINUS:
        case CMD_OVERRIDE_RAPID_RESET:
        case CMD_OVERRIDE_RAPID_MEDIUM:
        case CMD_OVERRIDE_RAPID_LOW:
            drop = true;
            enqueue_feed_override(c);
            break;

        case CMD_OVERRIDE_SPINDLE_RESET:
        case CMD_OVERRIDE_SPINDLE_COARSE_PLUS:
        case CMD_OVERRIDE_SPINDLE_COARSE_MINUS:
        case CMD_OVERRIDE_SPINDLE_FINE_PLUS:
        case CMD_OVERRIDE_SPINDLE_FINE_MINUS:
        case CMD_OVERRIDE_SPINDLE_STOP:
            drop = true;
            enqueue_spindle_override((uint8_t)c);
            break;

        case CMD_OVERRIDE_COOLANT_FLOOD_TOGGLE:
        case CMD_OVERRIDE_COOLANT_MIST_TOGGLE:
        case CMD_OVERRIDE_FAN0_TOGGLE:
            drop = true;
            enqueue_coolant_override((uint8_t)c);
            break;

        case CMD_REBOOT:
            if(esc && hal.reboot)
                hal.reboot(); // Force MCU reboot. This call should never return.
            break;

        default:
            if((c < ' ' && c != ASCII_BS) || (c > ASCII_DEL && c <= 0xBF))
                drop = grbl.on_unknown_realtime_cmd == NULL || grbl.on_unknown_realtime_cmd(c);
            break;
    }

    // 2. Process printable ASCII characters and top-bit set characters
    //    If legacy realtime commands are disabled they are returned to the input stream
    //    when appearing in settings ($ commands) or comments

    if(!drop) switch ((unsigned char)c) {

        case CMD_STATUS_REPORT_LEGACY:
            if(!keep_rt_commands || settings.flags.legacy_rt_commands) {
                system_set_exec_state_flag(EXEC_STATUS_REPORT);
                drop = true;
            }
            break;

        case CMD_CYCLE_START_LEGACY:
            if(!keep_rt_commands || settings.flags.legacy_rt_commands) {
                system_set_exec_state_flag(EXEC_CYCLE_START);
                // Cancel any pending tool change
                gc_state.tool_change = false;
                drop = true;
            }
            break;

        case CMD_FEED_HOLD_LEGACY:
            if(!keep_rt_commands || settings.flags.legacy_rt_commands) {
                system_set_exec_state_flag(EXEC_FEED_HOLD);
                drop = true;
            }
            break;

        default: // Drop top bit set characters
            drop = !(keep_rt_commands || (unsigned char)c < 0x7F);
            break;
    }

    esc = c == ASCII_ESC;

    return drop;
}

static const uint32_t dummy_data = 0;


/*! \brief Enqueue a function to be called once by the foreground process.
\param fn pointer to a \a foreground_task_ptr type of function.
\param data pointer to data to be passed to the callee.
\returns true if successful, false otherwise.
*/
ISR_CODE bool ISR_FUNC(protocol_enqueue_foreground_task)(fg_task_ptr fn, void *data)
{
    bool ok;
    uint_fast8_t bptr = (realtime_queue.head + 1) & (RT_QUEUE_SIZE - 1);    // Get next head pointer

    if((ok = bptr != realtime_queue.tail)) {                    // If not buffer full
        realtime_queue.task[realtime_queue.head].data = data;
        realtime_queue.task[realtime_queue.head].task = fn;       // add function pointer to buffer,
        realtime_queue.head = bptr;                             // update pointer and
        system_set_exec_state_flag(EXEC_RT_COMMAND);            // flag it for execute
    }

    return ok;
}

/*! \brief Enqueue a function to be called once by the foreground process.
\param fn pointer to a \a on_execute_realtime_ptr type of function.
\returns true if successful, false otherwise.
__NOTE:__ Deprecated, use protocol_enqueue_foreground_task instead.
*/
ISR_CODE bool ISR_FUNC(protocol_enqueue_rt_command)(on_execute_realtime_ptr fn)
{
    return protocol_enqueue_foreground_task(fn, (void *)&dummy_data);
}

// Execute enqueued functions.
static void protocol_execute_rt_commands (sys_state_t state)
{
    while(realtime_queue.tail != realtime_queue.head) {
        uint_fast8_t bptr = realtime_queue.tail;
        if(realtime_queue.task[bptr].task.fn) {
            if(realtime_queue.task[bptr].data == (void *)&dummy_data) {
                on_execute_realtime_ptr call = realtime_queue.task[bptr].task.fn_deprecated;
                realtime_queue.task[bptr].task.fn_deprecated = NULL;
                call(state_get());
            } else {
                foreground_task_ptr call = realtime_queue.task[bptr].task.fn;
                realtime_queue.task[bptr].task.fn = NULL;
                call(realtime_queue.task[bptr].data);
            }
        }
        realtime_queue.tail = (bptr + 1) & (RT_QUEUE_SIZE - 1);
    }

    if(!sys.driver_started)
        while(true);
}

void protocol_execute_noop (sys_state_t state)
{
    (void)state;
}
