/*
  system.c - Handles system level commands and real-time processes

  Part of grblHAL

  Copyright (c) 2017-2022 Terje Io
  Copyright (c) 2014-2016 Sungeun K. Jeon for Gnea Research LLC

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
#include <string.h>

#include "hal.h"
#include "motion_control.h"
#include "protocol.h"
#include "tool_change.h"
#include "state_machine.h"
#include "machine_limits.h"
#ifdef KINEMATICS_API
#include "kinematics.h"
#endif

static status_code_t jog (sys_state_t state, char *args);
static status_code_t enumerate_alarms (sys_state_t state, char *args);
static status_code_t enumerate_alarms_grblformatted (sys_state_t state, char *args);
static status_code_t enumerate_errors (sys_state_t state, char *args);
static status_code_t enumerate_errors_grblformatted (sys_state_t state, char *args);
static status_code_t enumerate_groups (sys_state_t state, char *args);
static status_code_t enumerate_settings (sys_state_t state, char *args);
static status_code_t enumerate_all (sys_state_t state, char *args);
static status_code_t enumerate_settings_grblformatted (sys_state_t state, char *args);
static status_code_t enumerate_settings_halformatted (sys_state_t state, char *args);
static status_code_t enumerate_pins (sys_state_t state, char *args);
static status_code_t output_settings (sys_state_t state, char *args);
static status_code_t output_all_settings (sys_state_t state, char *args);
#ifndef NO_SETTINGS_DESCRIPTIONS
static status_code_t output_setting_description (sys_state_t state, char *args);
#endif
static status_code_t output_parser_state (sys_state_t state, char *args);
static status_code_t toggle_block_delete (sys_state_t state, char *args);
static status_code_t toggle_single_block (sys_state_t state, char *args);
static status_code_t toggle_optional_stop (sys_state_t state, char *args);
static status_code_t check_mode (sys_state_t state, char *args);
static status_code_t disable_lock (sys_state_t state, char *args);
static status_code_t output_help (sys_state_t state, char *args);
static status_code_t home (sys_state_t state, char *args);
static status_code_t home_x (sys_state_t state, char *args);
static status_code_t home_y (sys_state_t state, char *args);
static status_code_t home_z (sys_state_t state, char *args);
#ifdef A_AXIS
static status_code_t home_a (sys_state_t state, char *args);
#endif
#ifdef B_AXIS
static status_code_t home_b (sys_state_t state, char *args);
#endif
#ifdef C_AXIS
static status_code_t home_c (sys_state_t state, char *args);
#endif
static status_code_t enter_sleep (sys_state_t state, char *args);
static status_code_t set_tool_reference (sys_state_t state, char *args);
static status_code_t tool_probe_workpiece (sys_state_t state, char *args);
static status_code_t output_ngc_parameters (sys_state_t state, char *args);
static status_code_t build_info (sys_state_t state, char *args);
static status_code_t output_all_build_info (sys_state_t state, char *args);
static status_code_t settings_reset (sys_state_t state, char *args);
static status_code_t output_startup_lines (sys_state_t state, char *args);
static status_code_t set_startup_line0 (sys_state_t state, char *args);
static status_code_t set_startup_line1 (sys_state_t state, char *args);
static status_code_t rtc_action (sys_state_t state, char *args);
#ifdef DEBUGOUT
static status_code_t output_memmap (sys_state_t state, char *args);
#endif

// Simple hypotenuse computation function.
inline static float hypot_f (float x, float y)
{
    return sqrtf(x*x + y*y);
}

// Pin change interrupt for pin-out commands, i.e. cycle start, feed hold, and reset. Sets
// only the realtime command execute variable to have the main program execute these when
// its ready. This works exactly like the character-based realtime commands when picked off
// directly from the incoming data stream.
ISR_CODE void ISR_FUNC(control_interrupt_handler)(control_signals_t signals)
{
    if(signals.deasserted)
        return; // for now...

    if (signals.value) {

        sys.last_event.control.value = signals.value;

        if ((signals.reset || signals.e_stop || signals.motor_fault) && state_get() != STATE_ESTOP)
            mc_reset();
        else {
#ifndef NO_SAFETY_DOOR_SUPPORT
            if (signals.safety_door_ajar && hal.signals_cap.safety_door_ajar) {
                if(settings.safety_door.flags.ignore_when_idle) {
                    // Only stop the spindle (laser off) when idle or jogging,
                    // this to allow positioning the controlled point (spindle) when door is open.
                    // NOTE: at least for lasers there should be an external interlock blocking laser power.
                    if(state_get() != STATE_IDLE && state_get() != STATE_JOG)
                        system_set_exec_state_flag(EXEC_SAFETY_DOOR);
                    if(sys.mode == Mode_Laser) // Turn off spindle immediately (laser) when in laser mode
                        hal.spindle.set_state((spindle_state_t){0}, 0.0f);
                } else
                    system_set_exec_state_flag(EXEC_SAFETY_DOOR);
            }
#endif
            if (signals.probe_triggered) {
                if(sys.probing_state == Probing_Off && (state_get() & (STATE_CYCLE|STATE_JOG))) {
                    system_set_exec_state_flag(EXEC_STOP);
                    sys.alarm_pending = Alarm_ProbeProtect;
                } else
                    hal.probe.configure(false, false);
            } else if (signals.probe_disconnected) {
                if(sys.probing_state == Probing_Active && state_get() == STATE_CYCLE) {
                    system_set_exec_state_flag(EXEC_FEED_HOLD);
                    sys.alarm_pending = Alarm_ProbeProtect;
                }
            } else if (signals.feed_hold)
                system_set_exec_state_flag(EXEC_FEED_HOLD);
            else if (signals.cycle_start)
                system_set_exec_state_flag(EXEC_CYCLE_START);
        }
    }
}


// Executes user startup script, if stored.
void system_execute_startup (void)
{
    if(hal.nvs.type != NVS_None) {

        char line[sizeof(stored_line_t)];
        uint_fast8_t n;

        for (n = 0; n < N_STARTUP_LINE; n++) {
            if (!settings_read_startup_line(n, line))
                report_execute_startup_message(line, Status_SettingReadFail);
            else if (*line != '\0')
                report_execute_startup_message(line, gc_execute_block(line));
        }
    }
}

// Reset spindle encoder data
status_code_t spindle_reset_data (sys_state_t state, char *args)
{
    if(hal.spindle.reset_data)
        hal.spindle.reset_data();

    return hal.spindle.reset_data ? Status_OK : Status_InvalidStatement;
}

status_code_t read_int (char *s, int32_t *value)
{
    uint_fast8_t counter = 0;
    float parameter;
    if(!read_float(s, &counter, &parameter))
        return Status_BadNumberFormat;

    if(parameter - truncf(parameter) != 0.0f)
        return Status_InvalidStatement;

    *value = (int32_t)parameter;

    return Status_OK;
}

PROGMEM static const sys_command_t sys_commands[] = {
    { "G", true, output_parser_state },
    { "J", false, jog },
    { "#", false, output_ngc_parameters },
    { "$", false, output_settings },
    { "+", false, output_all_settings },
#ifndef NO_SETTINGS_DESCRIPTIONS
    { "SED", false, output_setting_description },
#endif
    { "B", true, toggle_block_delete },
    { "S", true, toggle_single_block },
    { "O", true, toggle_optional_stop },
    { "C", true, check_mode },
    { "X", false, disable_lock },
    { "H", false, home },
    { "HX", false, home_x },
    { "HY", false, home_y },
    { "HZ", false, home_z },
#ifdef A_AXIS
    { "HA", false, home_a },
#endif
#ifdef B_AXIS
    { "HB", false, home_b },
#endif
#ifdef C_AXIS
    { "HC", false, home_c },
#endif
    { "HELP", false, output_help },
    { "SLP", true, enter_sleep },
    { "TLR", true, set_tool_reference },
    { "TPW", true, tool_probe_workpiece },
    { "I", false, build_info },
    { "I+", true, output_all_build_info },
    { "RST", false, settings_reset },
    { "N", true, output_startup_lines },
    { "N0", false, set_startup_line0 },
    { "N1", false, set_startup_line1 },
    { "EA", true, enumerate_alarms },
    { "EAG", true, enumerate_alarms_grblformatted },
    { "EE", true, enumerate_errors },
    { "EEG", true, enumerate_errors_grblformatted },
    { "EG", true, enumerate_groups },
    { "ES", true, enumerate_settings },
    { "ESG", true, enumerate_settings_grblformatted },
    { "ESH", true, enumerate_settings_halformatted },
    { "E*", true, enumerate_all },
    { "PINS", true, enumerate_pins },
    { "RST", false, settings_reset },
    { "LEV", true, report_last_signals_event },
    { "LIM", true, report_current_limit_state },
    { "SD", false, report_spindle_data },
    { "SR", false, spindle_reset_data },
    { "RTC", false, rtc_action },
#ifdef DEBUGOUT
    { "Q", true, output_memmap },
#endif
};

void system_command_help (void)
{
    hal.stream.write("$I - output system information" ASCII_EOL);
    hal.stream.write("$I+ - output extended system information" ASCII_EOL);
    hal.stream.write("$<n> - output setting <n> value" ASCII_EOL);
    hal.stream.write("$<n>=<value> - assign <value> to settings <n>" ASCII_EOL);
    hal.stream.write("$$ - output all setting values" ASCII_EOL);
    hal.stream.write("$+ - output all setting values" ASCII_EOL);
    hal.stream.write("$$=<n> - output setting details for setting <n>" ASCII_EOL);
    hal.stream.write("$# - output offsets, tool table, probing and home position" ASCII_EOL);
    hal.stream.write("$#=<n> - output value for parameter <n>" ASCII_EOL);
    hal.stream.write("$G - output parser state" ASCII_EOL);
    hal.stream.write("$N - output startup lines" ASCII_EOL);
    if(settings.homing.flags.enabled)
        hal.stream.write("$H - home configured axes" ASCII_EOL);
    if(settings.homing.flags.single_axis_commands)
        hal.stream.write("$H<axisletter> - home single axis" ASCII_EOL);
    hal.stream.write("$X - unlock machine" ASCII_EOL);
    hal.stream.write("$SLP - enter sleep mode" ASCII_EOL);
    hal.stream.write("$HELP - output help topics" ASCII_EOL);
    hal.stream.write("$HELP <topic> - output help for <topic>" ASCII_EOL);
    hal.stream.write("$RST=* - restore/reset all settings" ASCII_EOL);
    hal.stream.write("$RST=$ - restore default settings" ASCII_EOL);
    if(settings_get_details()->next)
        hal.stream.write("$RST=& - restore driver and plugin default settings" ASCII_EOL);
#ifdef N_TOOLS
    hal.stream.write("$RST=# - reset offsets and tool data" ASCII_EOL);
#else
    hal.stream.write("$RST=# - reset offsets" ASCII_EOL);
#endif
    if(hal.spindle.reset_data)
        hal.stream.write("$SR - reset spindle encoder data" ASCII_EOL);
    if(hal.spindle.get_data)
        hal.stream.write("$SD - output spindle encoder data" ASCII_EOL);
    hal.stream.write("$TLR - set tool offset reference" ASCII_EOL);
    hal.stream.write("$TPW - probe tool plate" ASCII_EOL);
    hal.stream.write("$EA - enumerate alarms" ASCII_EOL);
    hal.stream.write("$EAG - enumerate alarms, Grbl formatted" ASCII_EOL);
    hal.stream.write("$EE - enumerate status codes" ASCII_EOL);
    hal.stream.write("$EEG - enumerate status codes, Grbl formatted" ASCII_EOL);
    hal.stream.write("$ES - enumerate settings" ASCII_EOL);
    hal.stream.write("$ESG - enumerate settings, Grbl formatted" ASCII_EOL);
    hal.stream.write("$ESH- enumerate settings, grblHAL formatted" ASCII_EOL);
    hal.stream.write("$ESG - enumerate alarms" ASCII_EOL);
    hal.stream.write("$E* - enumerate alarms, status codes and settings" ASCII_EOL);
    if(hal.enumerate_pins)
        hal.stream.write("$PINS - enumerate pin bindings" ASCII_EOL);
    hal.stream.write("$LEV - output last control signal events" ASCII_EOL);
    hal.stream.write("$LIM - output current limit pins state" ASCII_EOL);
    if(hal.rtc.get_datetime) {
        hal.stream.write("$RTC - output current time" ASCII_EOL);
        hal.stream.write("$RTC=<ISO8601 datetime> - set current time" ASCII_EOL);
    }
#ifndef NO_SETTINGS_DESCRIPTIONS
    hal.stream.write("$SED=<n> - output settings description for setting <n>" ASCII_EOL);
#endif
}

// Directs and executes one line of formatted input from protocol_process. While mostly
// incoming streaming g-code blocks, this also executes Grbl internal commands, such as
// settings, initiating the homing cycle, and toggling switch states. This differs from
// the realtime command module by being susceptible to when Grbl is ready to execute the
// next line during a cycle, so for switches like block delete, the switch only effects
// the lines that are processed afterward, not necessarily real-time during a cycle,
// since there are motions already stored in the buffer. However, this 'lag' should not
// be an issue, since these commands are not typically used during a cycle.

// NOTE: Code calling system_execute_line() needs to provide a line buffer of at least LINE_BUFFER_SIZE
status_code_t system_execute_line (char *line)
{
    if(line[1] == '\0') {
        report_grbl_help();
        return Status_OK;
    }

    sys_commands_t base = {
        .n_commands = sizeof(sys_commands) / sizeof(sys_command_t),
        .commands = sys_commands,
        .on_get_commands = grbl.on_get_commands
    };

    status_code_t retval = Status_Unhandled;

    char c, *s1, *s2;

    s1 = s2 = ++line;

    c = *s1;
    while(c && c != '=') {
        if(c != ' ')
            *s2++ = CAPS(c);
        c = *++s1;
    }

    while((c = *s1++))
        *s2++ = c;

    *s2 = '\0';

    if(!strncmp(line, "HELP", 4))
        return report_help(&line[4]);

    char *args = strchr(line, '=');

    if(args)
        *args++ = '\0';

    uint_fast8_t idx;
    sys_commands_t *cmd = &base;
    do {
        for(idx = 0; idx < cmd->n_commands; idx++) {
            if(!strcmp(line, cmd->commands[idx].command)) {
                if(!cmd->commands[idx].noargs || args == NULL) {
                    if((retval = cmd->commands[idx].execute(state_get(), args)) != Status_Unhandled)
                        break;
                }
            }
        }
        cmd = retval == Status_Unhandled && cmd->on_get_commands ? cmd->on_get_commands() : NULL;
    } while(cmd);

    // Let user code have a peek at system commands before check for global setting
    if(retval == Status_Unhandled && grbl.on_unknown_sys_command) {
        if(args)
            *(--args) = '=';

        retval = grbl.on_unknown_sys_command(state_get(), line);

        if(args)
            *args++ = '\0';
    }

    if (retval == Status_Unhandled) {
        // Check for global setting, store if so
        if(state_get() == STATE_IDLE || (state_get() & (STATE_ALARM|STATE_ESTOP|STATE_CHECK_MODE))) {
            uint_fast8_t counter = 0;
            float parameter;
            if(!read_float(line, &counter, &parameter))
                retval = Status_BadNumberFormat;
            else if(!isintf(parameter))
                retval = Status_InvalidStatement;
            else if(args)
                retval = settings_store_setting((setting_id_t)parameter, args);
            else
                retval = report_grbl_setting((setting_id_t)parameter, NULL);
        } else
            retval = Status_IdleError;
    }

    return retval;
}

// System commands

static status_code_t jog (sys_state_t state, char *args)
{
    if(!(state == STATE_IDLE || (state & (STATE_JOG|STATE_TOOL_CHANGE))))
         return Status_IdleError;

    if(args != NULL) {
        *(--args) = '=';
        args -= 2;
    }

    return args == NULL ? Status_InvalidStatement : gc_execute_block(args); // NOTE: $J= is ignored inside g-code parser and used to detect jog motions.
}

static status_code_t enumerate_alarms (sys_state_t state, char *args)
{
    return report_alarm_details(false);
}

static status_code_t enumerate_alarms_grblformatted (sys_state_t state, char *args)
{
    return report_alarm_details(true);
}

static status_code_t enumerate_errors (sys_state_t state, char *args)
{
    return report_error_details(false);
}

static status_code_t enumerate_errors_grblformatted (sys_state_t state, char *args)
{
    return report_error_details(true);
}

static status_code_t enumerate_groups (sys_state_t state, char *args)
{
    return report_setting_group_details(true, NULL);
}

static status_code_t enumerate_settings (sys_state_t state, char *args)
{
    return report_settings_details(SettingsFormat_MachineReadable, Setting_SettingsAll, Group_All);
}

static status_code_t enumerate_settings_grblformatted (sys_state_t state, char *args)
{
    return report_settings_details(SettingsFormat_Grbl, Setting_SettingsAll, Group_All);
}

static status_code_t enumerate_settings_halformatted (sys_state_t state, char *args)
{
    return report_settings_details(SettingsFormat_grblHAL, Setting_SettingsAll, Group_All);
}

static status_code_t enumerate_all (sys_state_t state, char *args)
{
    report_alarm_details(false);
    report_error_details(false);
    report_setting_group_details(true, NULL);
    return report_settings_details(SettingsFormat_MachineReadable, Setting_SettingsAll, Group_All);
}

static status_code_t enumerate_pins (sys_state_t state, char *args)
{
    return report_pins(state, args);
}

static status_code_t output_settings (sys_state_t state, char *args)
{
    status_code_t retval = Status_OK;

    if(args) {
        int32_t id;
        retval = read_int(args, &id);
        if(retval == Status_OK && id >= 0)
            retval = report_settings_details(SettingsFormat_HumanReadable, (setting_id_t)id, Group_All);
    } else if (state & (STATE_CYCLE|STATE_HOLD))
        retval = Status_IdleError; // Block during cycle. Takes too long to print.
    else
#if COMPATIBILITY_LEVEL <= 1
    report_grbl_settings(true, NULL);
#else
    report_grbl_settings(false, NULL);
#endif

    return retval;
}

#ifndef NO_SETTINGS_DESCRIPTIONS

static status_code_t output_setting_description (sys_state_t state, char *args)
{
    status_code_t retval = Status_BadNumberFormat;

    if(args) {
        int32_t id;
        retval = read_int(args, &id);
        if(retval == Status_OK && id >= 0)
            retval = report_setting_description(SettingsFormat_MachineReadable, (setting_id_t)id);
    }

    return retval;
}

#endif

static status_code_t output_all_settings (sys_state_t state, char *args)
{
    status_code_t retval = Status_OK;

    if(args) {
        int32_t id;
        retval = read_int(args, &id);
        if(retval == Status_OK && id >= 0)
            retval = report_settings_details(SettingsFormat_HumanReadable, (setting_id_t)id, Group_All);
    } else if (state & (STATE_CYCLE|STATE_HOLD))
        retval = Status_IdleError; // Block during cycle. Takes too long to print.
    else
        report_grbl_settings(true, NULL);

    return retval;
}

static status_code_t output_parser_state (sys_state_t state, char *args)
{
    report_gcode_modes();
    sys.report.homed = On; // Report homed state on next realtime report

    return Status_OK;
}

static status_code_t toggle_single_block (sys_state_t state, char *args)
{
    sys.flags.single_block = !sys.flags.single_block;
    grbl.report.feedback_message(sys.flags.single_block ? Message_Enabled : Message_Disabled);

    return Status_OK;
}

static status_code_t toggle_block_delete (sys_state_t state, char *args)
{
    sys.flags.block_delete_enabled = !sys.flags.block_delete_enabled;
    grbl.report.feedback_message(sys.flags.block_delete_enabled ? Message_Enabled : Message_Disabled);

    return Status_OK;
}

static status_code_t toggle_optional_stop (sys_state_t state, char *args)
{
    sys.flags.optional_stop_disable = !sys.flags.optional_stop_disable;
    grbl.report.feedback_message(sys.flags.block_delete_enabled ? Message_Enabled : Message_Disabled);

    return Status_OK;
}

static status_code_t check_mode (sys_state_t state, char *args)
{
    if (state == STATE_CHECK_MODE) {
        // Perform reset when toggling off. Check g-code mode should only work if Grbl
        // is idle and ready, regardless of alarm locks. This is mainly to keep things
        // simple and consistent.
        mc_reset();
        grbl.report.feedback_message(Message_Disabled);
    } else if (state == STATE_IDLE) { // Requires idle mode.
        state_set(STATE_CHECK_MODE);
        grbl.report.feedback_message(Message_Enabled);
    } else
        return Status_IdleError;

    return Status_OK;
}

static status_code_t disable_lock (sys_state_t state, char *args)
{
    status_code_t retval = Status_OK;

    if(state & (STATE_ALARM|STATE_ESTOP)) {

        control_signals_t control_signals = hal.control.get_state();

        // Block if self-test failed
        if(sys.alarm == Alarm_SelftestFailed)
            retval = Status_SelfTestFailed;
        // Block if e-stop is active.
        else if (control_signals.e_stop)
            retval = Status_EStop;
        // Block if safety door is ajar.
        else if (control_signals.safety_door_ajar)
            retval = Status_CheckDoor;
        // Block if safety reset is active.
        else if(control_signals.reset)
            retval = Status_Reset;
        else if(settings.limits.flags.hard_enabled && settings.limits.flags.check_at_init && limit_signals_merge(hal.limits.get_state()).value)
            retval = Status_LimitsEngaged;
        else if(limits_homing_required())
            retval = Status_HomingRequired;
        else {
            grbl.report.feedback_message(Message_AlarmUnlock);
            state_set(STATE_IDLE);
        }
        // Don't run startup script. Prevents stored moves in startup from causing accidents.
    } // Otherwise, no effect.

    return retval;
}

static status_code_t output_help (sys_state_t state, char *args)
{
    return report_help(args);
}

static status_code_t go_home (sys_state_t state, axes_signals_t axes)
{
    if(!(state_get() == STATE_IDLE || (state_get() & (STATE_ALARM|STATE_ESTOP))))
        return Status_IdleError;

    status_code_t retval = Status_OK;

    control_signals_t control_signals = hal.control.get_state();

    // Block if self-test failed
    if(sys.alarm == Alarm_SelftestFailed)
        retval = Status_SelfTestFailed;
    // Block if e-stop is active.
    else if (control_signals.e_stop)
        retval = Status_EStop;
    else if(control_signals.motor_fault)
        retval = Status_MotorFault;
    else if (!(settings.homing.flags.enabled && (sys.homing.mask || settings.homing.flags.single_axis_commands || settings.homing.flags.manual)))
        retval = Status_HomingDisabled;
    // Block if safety door is ajar.
    else if (control_signals.safety_door_ajar && !settings.safety_door.flags.ignore_when_idle)
        retval = Status_CheckDoor;
    // Block if safety reset is active.
    else if(control_signals.reset)
        retval = Status_Reset;

    if(retval == Status_OK)
        retval = mc_homing_cycle(axes); // Home axes according to configuration

    if (retval == Status_OK && !sys.abort) {
        state_set(STATE_IDLE);  // Set to IDLE when complete.
        st_go_idle();           // Set steppers to the settings idle state before returning.
        report_feedback_message(Message_None);
        // Execute startup scripts after successful homing.
        if (sys.homing.mask && (sys.homing.mask & sys.homed.mask) == sys.homing.mask)
            system_execute_startup();
        else if(limits_homing_required()) { // Keep alarm state active if homing is required and not all axes homed.
            sys.alarm = Alarm_HomingRequried;
            state_set(STATE_ALARM);
        }
    }

    return retval == Status_Unhandled ? Status_OK : retval;
}

static status_code_t home (sys_state_t state, char *args)
{
    return go_home(state, (axes_signals_t){0});
}

static status_code_t home_x (sys_state_t state, char *args)
{
    return go_home(state, (axes_signals_t){X_AXIS_BIT});
}

static status_code_t home_y (sys_state_t state, char *args)
{
    return go_home(state, (axes_signals_t){Y_AXIS_BIT});
}

static status_code_t home_z (sys_state_t state, char *args)
{
    return go_home(state, (axes_signals_t){Z_AXIS_BIT});
}

#ifdef A_AXIS
static status_code_t home_a (sys_state_t state, char *args)
{
    return go_home(state, (axes_signals_t){A_AXIS_BIT});
}
#endif

#ifdef B_AXIS
static status_code_t home_b (sys_state_t state, char *args)
{
    return go_home(state, (axes_signals_t){B_AXIS_BIT});
}
#endif

#ifdef C_AXIS
static status_code_t home_c (sys_state_t state, char *args)
{
    return go_home(state, (axes_signals_t){C_AXIS_BIT});
}
#endif

static status_code_t enter_sleep (sys_state_t state, char *args)
{
    if(!settings.flags.sleep_enable)
        return Status_InvalidStatement;
    else if(!(state == STATE_IDLE || state == STATE_ALARM))
        return Status_IdleError;
    else
        system_set_exec_state_flag(EXEC_SLEEP); // Set to execute enter_sleep mode immediately

    return Status_OK;
}

static status_code_t set_tool_reference (sys_state_t state, char *args)
{
#ifdef TOOL_LENGTH_OFFSET_AXIS
    if(sys.flags.probe_succeeded) {
        sys.tlo_reference_set.mask = bit(TOOL_LENGTH_OFFSET_AXIS);
        sys.tlo_reference[TOOL_LENGTH_OFFSET_AXIS] = sys.probe_position[TOOL_LENGTH_OFFSET_AXIS]; // - gc_state.tool_length_offset[Z_AXIS]));
    } else
        sys.tlo_reference_set.mask = 0;
#else
    plane_t plane;
    gc_get_plane_data(&plane, gc_state.modal.plane_select);
    if(sys.flags.probe_succeeded) {
        sys.tlo_reference_set.mask |= bit(plane.axis_linear);
        sys.tlo_reference[plane.axis_linear] = sys.probe_position[plane.axis_linear];
//                    - lroundf(gc_state.tool_length_offset[plane.axis_linear] * settings.axis[plane.axis_linear].steps_per_mm);
    } else
        sys.tlo_reference_set.mask = 0;
#endif
    sys.report.tlo_reference = On;

    return Status_OK;
}

static status_code_t tool_probe_workpiece (sys_state_t state, char *args)
{
    return tc_probe_workpiece();
}

static status_code_t output_ngc_parameters (sys_state_t state, char *args)
{
    status_code_t retval = Status_OK;

    if(args) {
        int32_t id;
        retval = read_int(args, &id);
        if(retval == Status_OK && id >= 0)
            retval = report_ngc_parameter((ngc_param_id_t)id);
        else
            retval = report_named_ngc_parameter(args);
    } else
        report_ngc_parameters();

    return retval;
}

static status_code_t build_info (sys_state_t state, char *args)
{
    if (!(state == STATE_IDLE || (state & (STATE_ALARM|STATE_ESTOP|STATE_CHECK_MODE))))
        return Status_IdleError;

    if (args == NULL) {
        char info[sizeof(stored_line_t)];
        settings_read_build_info(info);
        report_build_info(info, false);
    }
  #ifndef DISABLE_BUILD_INFO_WRITE_COMMAND
    else if (strlen(args) < (sizeof(stored_line_t) - 1))
        settings_write_build_info(args);
  #endif
    else
        return Status_InvalidStatement;

    return Status_OK;
}

static status_code_t output_all_build_info (sys_state_t state, char *args)
{
    char info[sizeof(stored_line_t)];

    settings_read_build_info(info);
    report_build_info(info, true);

    return Status_OK;
}

static status_code_t settings_reset (sys_state_t state, char *args)
{
    settings_restore_t restore = {0};
    status_code_t retval = Status_OK;

    if (!(state == STATE_IDLE || (state & (STATE_ALARM|STATE_ESTOP))))
        retval = Status_IdleError;

    else switch (*args) {

      #ifndef DISABLE_RESTORE_NVS_DEFAULT_SETTINGS
        case '$':
            restore.defaults = On;
            break;
      #endif

      #ifndef DISABLE_RESTORE_NVS_CLEAR_PARAMETERS
        case '#':
            restore.parameters = On;
            break;
      #endif

      #ifndef DISABLE_RESTORE_NVS_WIPE_ALL
        case '*':
            restore.mask = settings_all.mask;
            break;
      #endif

      #ifndef DISABLE_RESTORE_DRIVER_PARAMETERS
        case '&':
            restore.driver_parameters = On;
            break;
      #endif

        default:
            retval = Status_InvalidStatement;
            break;
    }

    if(retval == Status_OK && restore.mask) {
        settings_restore(restore);
        grbl.report.feedback_message(Message_RestoreDefaults);
        mc_reset(); // Force reset to ensure settings are initialized correctly.
    }

    return retval;
}

static status_code_t output_startup_lines (sys_state_t state, char *args)
{
    if (!(state == STATE_IDLE || (state & (STATE_ALARM|STATE_ESTOP|STATE_CHECK_MODE))))
        return Status_IdleError;

    // Print startup lines

    uint_fast8_t counter;
    char line[sizeof(stored_line_t)];

    for (counter = 0; counter < N_STARTUP_LINE; counter++) {
        if (!(settings_read_startup_line(counter, line)))
            grbl.report.status_message(Status_SettingReadFail);
        else
            report_startup_line(counter, line);
    }

    return Status_OK;
}

static status_code_t set_startup_line (sys_state_t state, char *args, uint_fast8_t lnr)
{
    // Store startup line [IDLE Only] Prevents motion during ALARM.
    if (!(state == STATE_IDLE || (state & (STATE_ALARM|STATE_ESTOP|STATE_CHECK_MODE))))
        return Status_IdleError;

    if(args == NULL)
        return Status_InvalidStatement;

    status_code_t retval = Status_OK;

    args = gc_normalize_block(args, NULL);

    if(strlen(args) >= (sizeof(stored_line_t) - 1))
        retval = Status_Overflow;
    else if ((retval = gc_execute_block(args)) == Status_OK) // Execute gcode block to ensure block is valid.
        settings_write_startup_line(lnr, args);

    return retval;
}

static status_code_t set_startup_line0 (sys_state_t state, char *args)
{
    return set_startup_line(state, args, 0);
}

static status_code_t set_startup_line1 (sys_state_t state, char *args)
{
    return set_startup_line(state, args, 1);
}

static status_code_t rtc_action (sys_state_t state, char *args)
{
    status_code_t retval = Status_OK;

    if(args) {

        struct tm *time = get_datetime(args);

        if(time)
            hal.rtc.set_datetime(time);
        else
            retval = Status_BadNumberFormat;
    } else
        retval = report_time();

    return retval;
}

#ifdef DEBUGOUT

#include "nvs_buffer.h"

static status_code_t output_memmap (sys_state_t state, char *args)
{
    nvs_memmap();

    return Status_OK;
}
#endif

// End system commands

void system_flag_wco_change (void)
{
    if(!settings.status_report.sync_on_wco_change)
        protocol_buffer_synchronize();

    sys.report.wco = On;
}

// Sets machine position. Must be sent a 'step' array.
// NOTE: If motor steps and machine position are not in the same coordinate frame, this function
//       serves as a central place to compute the transformation.
void system_convert_array_steps_to_mpos (float *position, int32_t *steps)
{
#ifdef KINEMATICS_API
    kinematics.transform_steps_to_cartesian(position, steps);
#else
    uint_fast8_t idx = N_AXIS;
    do {
        idx--;
        position[idx] = steps[idx] / settings.axis[idx].steps_per_mm;
    } while(idx);
#endif
}

// Checks if XY position is within coordinate system XY with given tolerance.
// If tolerance is 0 false is returned.
bool system_xy_at_fixture (coord_system_id_t id, float tolerance)
{
    bool ok = false;

    coord_data_t target, position;

    if(tolerance > 0.0f && settings_read_coord_data(id, &target.values)) {
        system_convert_array_steps_to_mpos(position.values, sys.position);
        ok = hypot_f(position.x - target.x, position.y - target.y) <= tolerance;
    }

    return ok;
}


// Checks and reports if target array exceeds machine travel limits. Returns false if check failed.
// NOTE: max_travel is stored as negative
// TODO: only check homed axes?
bool system_check_travel_limits (float *target)
{
    bool failed = false;
    uint_fast8_t idx = N_AXIS;

    if(settings.homing.flags.force_set_origin) {
        do {
            idx--;
        // When homing forced set origin is enabled, soft limits checks need to account for directionality.
            failed = settings.axis[idx].max_travel < -0.0f &&
                      (bit_istrue(settings.homing.dir_mask.value, bit(idx))
                        ? (target[idx] < 0.0f || target[idx] > -settings.axis[idx].max_travel)
                        : (target[idx] > 0.0f || target[idx] < settings.axis[idx].max_travel));
        } while(!failed && idx);
    } else do {
        idx--;
        failed = settings.axis[idx].max_travel < -0.0f && (target[idx] > 0.0f || target[idx] < settings.axis[idx].max_travel);
    } while(!failed && idx);

    return !failed;
}

// Limits jog commands to be within machine limits, homed axes only.
// When hard limits are enabled pulloff distance is subtracted to avoid triggering limit switches.
// NOTE: max_travel is stored as negative
void system_apply_jog_limits (float *target)
{
    uint_fast8_t idx = N_AXIS;

    if(sys.homed.mask) do {
        idx--;
        float pulloff = settings.limits.flags.hard_enabled && bit_istrue(sys.homing.mask, bit(idx)) ? settings.homing.pulloff : 0.0f;
        if(bit_istrue(sys.homed.mask, bit(idx)) && settings.axis[idx].max_travel < -0.0f) {
            if(settings.homing.flags.force_set_origin) {
                if(bit_isfalse(settings.homing.dir_mask.value, bit(idx))) {
                    if(target[idx] > 0.0f)
                        target[idx] = 0.0f;
                    else if(target[idx] < (settings.axis[idx].max_travel + pulloff))
                        target[idx] = (settings.axis[idx].max_travel + pulloff);
                } else {
                    if(target[idx] < 0.0f)
                        target[idx] = 0.0f;
                    else if(target[idx] > -(settings.axis[idx].max_travel + pulloff))
                        target[idx] = -(settings.axis[idx].max_travel + pulloff);
                }
            } else {
                if(target[idx] > -pulloff)
                    target[idx] = -pulloff;
                else if(target[idx] < (settings.axis[idx].max_travel + pulloff))
                    target[idx] = (settings.axis[idx].max_travel + pulloff);
            }
        }
    } while(idx);
}

void system_raise_alarm (alarm_code_t alarm)
{
    if(state_get() == STATE_HOMING && !(sys.rt_exec_state & EXEC_RESET))
        system_set_exec_alarm(alarm);
    else if(sys.alarm != alarm) {
        sys.alarm = alarm;
        state_set(alarm == Alarm_EStop ? STATE_ESTOP : STATE_ALARM);
        if(sys.driver_started || sys.alarm == Alarm_SelftestFailed)
            report_alarm_message(alarm);
    }
}
