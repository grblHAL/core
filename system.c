/*
  system.c - Handles system level commands and real-time processes

  Part of grblHAL

  Copyright (c) 2017-2024 Terje Io
  Copyright (c) 2014-2016 Sungeun K. Jeon for Gnea mResearch LLC

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

/*! \internal \brief Simple hypotenuse computation function.
\param x length
\param y height
\returns length of hypotenuse
 */
inline static float hypot_f (float x, float y)
{
    return sqrtf(x * x + y * y);
}

void system_init_switches (void)
{
    control_signals_t signals = hal.control.get_state();

    sys.flags.block_delete_enabled = signals.block_delete;
    sys.flags.single_block = signals.single_block;
    sys.flags.optional_stop_disable = signals.stop_disable;
}

/*! \brief Pin change interrupt handler for pin-out commands, i.e. cycle start, feed hold, reset etc.
Mainly sets the realtime command execute variable to have the main program execute these when
its ready. This works exactly like the character-based realtime commands when picked off
directly from the incoming data stream.
\param signals a \a control_signals_t union holding status of the signals.
*/
ISR_CODE void ISR_FUNC(control_interrupt_handler)(control_signals_t signals)
{
    static control_signals_t onoff_signals = {
       .block_delete = On,
       .single_block = On,
       .stop_disable = On,
       .deasserted = On
    };

    if(signals.deasserted)
        signals.value &= onoff_signals.mask;

    if(signals.value) {

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
                    if(settings.mode == Mode_Laser) // Turn off spindle immediately (laser) when in laser mode
                        spindle_all_off();
                } else
                    system_set_exec_state_flag(EXEC_SAFETY_DOOR);
            }
#endif

            if(signals.probe_overtravel) {
                limit_signals_t overtravel = { .min.z = On};
                hal.limits.interrupt_callback(overtravel);
                // TODO: add message?
            } else if (signals.probe_triggered) {
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
            else if (signals.cycle_start) {
                system_set_exec_state_flag(EXEC_CYCLE_START);
                sys.report.cycle_start = settings.status_report.pin_state;
            }

            if(signals.block_delete)
                sys.flags.block_delete_enabled = !signals.deasserted;

            if(signals.single_block)
                sys.flags.single_block = !signals.deasserted;

            if(signals.stop_disable)
                sys.flags.optional_stop_disable = !signals.deasserted;
        }
    }
}


/*! \brief Executes user startup scripts, if stored.
*/
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

static status_code_t read_int (char *s, int32_t *value)
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

//
// System commands
//

// Reset spindle encoder data
static status_code_t spindle_reset_data (sys_state_t state, char *args)
{
    spindle_ptrs_t *spindle = gc_spindle_get();

    if(spindle->reset_data)
        spindle->reset_data();

    return spindle->reset_data ? Status_OK : Status_InvalidStatement;
}

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

#ifndef NO_SETTINGS_DESCRIPTIONS

static status_code_t pin_state (sys_state_t state, char *args)
{
    return report_pin_states(state, args);
}

#endif

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
    system_add_rt_report(Report_Homed); // Report homed state on next realtime report

    return Status_OK;
}

static status_code_t toggle_single_block (sys_state_t state, char *args)
{
    if(!hal.signals_cap.single_block) {
        sys.flags.single_block = !sys.flags.single_block;
        grbl.report.feedback_message(sys.flags.single_block ? Message_Enabled : Message_Disabled);
    }

    return hal.signals_cap.single_block ? Status_InvalidStatement : Status_OK;
}

static status_code_t toggle_block_delete (sys_state_t state, char *args)
{
    if(!hal.signals_cap.block_delete) {
        sys.flags.block_delete_enabled = !sys.flags.block_delete_enabled;
        grbl.report.feedback_message(sys.flags.block_delete_enabled ? Message_Enabled : Message_Disabled);
    }

    return hal.signals_cap.block_delete ? Status_InvalidStatement : Status_OK;
}

static status_code_t toggle_optional_stop (sys_state_t state, char *args)
{
    if(!hal.signals_cap.stop_disable) {
        sys.flags.optional_stop_disable = !sys.flags.optional_stop_disable;
        grbl.report.feedback_message(sys.flags.block_delete_enabled ? Message_Enabled : Message_Disabled);
    }

    return hal.signals_cap.stop_disable ? Status_InvalidStatement : Status_OK;
}

static status_code_t check_mode (sys_state_t state, char *args)
{
    if (state == STATE_CHECK_MODE) {
        // Perform reset when toggling off. Check g-code mode should only work if grblHAL
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

static status_code_t enumerate_spindles (sys_state_t state, char *args)
{
    return report_spindles(false);
}

static status_code_t enumerate_spindles_mr (sys_state_t state, char *args)
{
    return report_spindles(true);
}

static status_code_t go_home (sys_state_t state, axes_signals_t axes)
{
    if(axes.mask && !settings.homing.flags.single_axis_commands)
        return Status_HomingDisabled;

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
        grbl.report.feedback_message(Message_None);
        // Execute startup scripts after successful homing.
        if (sys.homing.mask && (sys.homing.mask & sys.homed.mask) == sys.homing.mask)
            system_execute_startup();
        else if(limits_homing_required()) { // Keep alarm state active if homing is required and not all axes homed.
            sys.alarm = Alarm_HomingRequired;
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

#ifdef U_AXIS
static status_code_t home_u (sys_state_t state, char *args)
{
    return go_home(state, (axes_signals_t){U_AXIS_BIT});
}
#endif

#ifdef V_AXIS
static status_code_t home_v (sys_state_t state, char *args)
{
    return go_home(state, (axes_signals_t){V_AXIS_BIT});
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
#if TOOL_LENGTH_OFFSET_AXIS >= 0
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
    system_add_rt_report(Report_TLOReference);

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
#if NGC_PARAMETERS_ENABLE
        int32_t id;
        retval = read_int(args, &id);
        if(retval == Status_OK && id >= 0)
            retval = report_ngc_parameter((ngc_param_id_t)id);
        else
            retval = report_named_ngc_parameter(args);
#else
        retval = Status_InvalidStatement;
#endif
    } else
        report_ngc_parameters();

    return retval;
}

static status_code_t build_info (sys_state_t state, char *args)
{
    if (!(state == STATE_IDLE || (state & (STATE_ALARM|STATE_ESTOP|STATE_SLEEP|STATE_CHECK_MODE))))
        return Status_IdleError;

    if (args == NULL) {
        char info[sizeof(stored_line_t)];
        settings_read_build_info(info);
        report_build_info(info, false);
    }
  #if !DISABLE_BUILD_INFO_WRITE_COMMAND
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

      #if ENABLE_RESTORE_NVS_DEFAULT_SETTINGS
        case '$':
            restore.defaults = On;
            break;
      #endif

      #if ENABLE_RESTORE_NVS_CLEAR_PARAMETERS
        case '#':
            restore.parameters = On;
            break;
      #endif

      #if ENABLE_RESTORE_NVS_WIPE_ALL
        case '*':
            restore.mask = settings_all.mask;
            break;
      #endif

      #if ENABLE_RESTORE_NVS_DRIVER_PARAMETERS
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

const char *help_rst (const char *cmd)
{
#if ENABLE_RESTORE_NVS_WIPE_ALL
    hal.stream.write("$RST=* - restore/reset all settings." ASCII_EOL);
#endif
#if ENABLE_RESTORE_NVS_DEFAULT_SETTINGS
    hal.stream.write("$RST=$ - restore default settings." ASCII_EOL);
#endif
#if ENABLE_RESTORE_NVS_DRIVER_PARAMETERS
    if(settings_get_details()->next)
        hal.stream.write("$RST=& - restore driver and plugin default settings." ASCII_EOL);
#endif
#if ENABLE_RESTORE_NVS_CLEAR_PARAMETERS
    if(grbl.tool_table.n_tools)
        hal.stream.write("$RST=# - reset offsets and tool data." ASCII_EOL);
    else
        hal.stream.write("$RST=# - reset offsets." ASCII_EOL);
#endif

    return NULL;
}

const char *help_rtc (const char *cmd)
{
    if(hal.rtc.get_datetime) {
        hal.stream.write("$RTC - output current time." ASCII_EOL);
        hal.stream.write("$RTC=<ISO8601 datetime> - set current time." ASCII_EOL);
    }

    return NULL;
}

const char *help_spindle (const char *cmd)
{
    spindle_ptrs_t *spindle = gc_spindle_get();

    if(cmd[1] == 'R' && spindle->reset_data)
        hal.stream.write("$SR - reset spindle encoder data." ASCII_EOL);

    if(cmd[1] == 'D' && spindle->get_data)
        hal.stream.write("$SD - output spindle encoder data." ASCII_EOL);

    return NULL;
}

const char *help_pins (const char *cmd)
{
    return hal.enumerate_pins ? "enumerate pin bindings" : NULL;
}

#ifndef NO_SETTINGS_DESCRIPTIONS

const char *help_pin_state (const char *cmd)
{
    return hal.port.get_pin_info ? "output auxillary pin states" : NULL;
}

#endif

const char *help_switches (const char *cmd)
{
    const char *help = NULL;

    switch(*cmd) {

        case 'B':
            if(!hal.signals_cap.block_delete)
                help = "toggle block delete switch";
            break;

        case 'O':
            if(!hal.signals_cap.stop_disable)
                help = "toggle optional stop switch (M1)";
            break;

        case 'S':
            if(!hal.signals_cap.single_block)
                help = "toggle single stepping switch";
            break;
    }

    return help;
}

const char *help_homing (const char *cmd)
{
    if(settings.homing.flags.enabled)
        hal.stream.write("$H - home configured axes." ASCII_EOL);

    if(settings.homing.flags.single_axis_commands)
        hal.stream.write("$H<axisletter> - home single axis." ASCII_EOL);

    return NULL;
}

/*! \brief Command dispatch table
 */
PROGMEM static const sys_command_t sys_commands[] = {
    { "G", output_parser_state, { .noargs = On, .allow_blocking = On }, { .str = "output parser state" } },
    { "J", jog, {}, { .str = "$J=<gcode> - jog machine" } },
    { "#", output_ngc_parameters, { .allow_blocking = On }, {
        .str = "output offsets, tool table, probing and home position"
#if NGC_PARAMETERS_ENABLE
     ASCII_EOL "$#=<n> - output value for parameter <n>"
#endif
    } },
    { "$", output_settings, { .allow_blocking = On }, {
        .str = "$<n> - output setting <n> value"
     ASCII_EOL "$<n>=<value> - assign <value> to settings <n>"
     ASCII_EOL "$$ - output all setting values"
     ASCII_EOL "$$=<n> - output setting details for setting <n>"
    } },
    { "+", output_all_settings, { .allow_blocking = On }, { .str = "output all setting values" } },
#ifndef NO_SETTINGS_DESCRIPTIONS
    { "SED", output_setting_description, { .allow_blocking = On }, { .str = "$SED=<n> - output settings description for setting <n>" } },
#endif
    { "B", toggle_block_delete, { .noargs = On, .help_fn = On }, { .fn = help_switches } },
    { "S", toggle_single_block, { .noargs = On, .help_fn = On }, { .fn = help_switches } },
    { "O", toggle_optional_stop, { .noargs = On, .help_fn = On }, { .fn = help_switches } },
    { "C", check_mode, { .noargs = On }, { .str = "enable check mode, <Reset> to exit" } },
    { "X", disable_lock, {}, { .str = "unlock machine" } },
    { "H", home, { .help_fn = On }, { .fn = help_homing } },
    { "HX", home_x },
    { "HY", home_y },
    { "HZ", home_z },
#if AXIS_REMAP_ABC2UVW
  #ifdef A_AXIS
    { "HU", home_a },
  #endif
  #ifdef B_AXIS
    { "HV", home_b },
  #endif
  #ifdef C_AXIS
    { "HW", home_c },
  #endif
#else
  #ifdef A_AXIS
    { "HA", home_a },
  #endif
  #ifdef B_AXIS
    { "HB", home_b },
  #endif
  #ifdef C_AXIS
    { "HC", home_c },
  #endif
#endif
#ifdef U_AXIS
    { "HU", home_u },
#endif
#ifdef V_AXIS
    { "HV", home_v },
#endif
    { "HSS", report_current_home_signal_state, { .noargs = On, .allow_blocking = On }, { .str = "report homing switches status" } },
    { "HELP", output_help, { .allow_blocking = On }, {
        .str = "$HELP - output help topics"
     ASCII_EOL "$HELP <topic> - output help for <topic>"
    } },
    { "SPINDLES", enumerate_spindles, { .noargs = On, .allow_blocking = On }, { .str = "enumerate spindles, human readable" } },
    { "SPINDLESH", enumerate_spindles_mr, { .noargs = On, .allow_blocking = On }, { .str = "enumerate spindles, machine readable" } },
    { "SLP", enter_sleep, { .noargs = On }, { .str = "enter sleep mode" } },
    { "TLR", set_tool_reference, { .noargs = On }, { .str = "set tool offset reference" } },
    { "TPW", tool_probe_workpiece, { .noargs = On }, { .str = "probe tool plate" } },
    { "I", build_info, { .allow_blocking = On }, {
        .str = "output system information"
#if !DISABLE_BUILD_INFO_WRITE_COMMAND
     ASCII_EOL "$I=<string> - set build info string"
#endif
    } },
    { "I+", output_all_build_info, { .noargs = On, .allow_blocking = On }, { .str = "output extended system information" } },
    { "RST", settings_reset, { .allow_blocking = On, .help_fn = On }, { .fn = help_rst } },
    { "N", output_startup_lines, { .noargs = On, .allow_blocking = On }, { .str = "output startup lines" } },
    { "N0", set_startup_line0, {}, { .str = "N0=<gcode> - set startup line 0" } },
    { "N1", set_startup_line1, {}, { .str = "N1=<gcode> - set startup line 1" } },
    { "EA", enumerate_alarms, { .noargs = On, .allow_blocking = On }, { .str = "enumerate alarms" } },
    { "EAG", enumerate_alarms_grblformatted, { .noargs = On, .allow_blocking = On }, { .str = "enumerate alarms, Grbl formatted" } },
    { "EE", enumerate_errors, { .noargs = On, .allow_blocking = On }, { .str = "enumerate status codes" } },
    { "EEG", enumerate_errors_grblformatted, { .noargs = On, .allow_blocking = On }, { .str = "enumerate status codes, Grbl formatted" } },
    { "EG", enumerate_groups, { .noargs = On, .allow_blocking = On }, { .str = "enumerate setting groups" } },
    { "ES", enumerate_settings, { .noargs = On, .allow_blocking = On }, { .str = "enumerate settings" } },
    { "ESG", enumerate_settings_grblformatted, { .noargs = On, .allow_blocking = On }, { .str = "enumerate settings, Grbl formatted" } },
    { "ESH", enumerate_settings_halformatted, { .noargs = On, .allow_blocking = On }, { .str = "enumerate settings, grblHAL formatted" } },
    { "E*", enumerate_all, { .noargs = On, .allow_blocking = On }, { .str = "enumerate alarms, status codes and settings" } },
    { "PINS", enumerate_pins, { .noargs = On, .allow_blocking = On, .help_fn = On }, { .fn = help_pins } },
#ifndef NO_SETTINGS_DESCRIPTIONS
    { "PINSTATE", pin_state, { .noargs = On, .allow_blocking = On, .help_fn = On }, { .fn = help_pin_state } },
#endif
    { "LEV", report_last_signals_event, { .noargs = On, .allow_blocking = On }, { .str = "output last control signal events" } },
    { "LIM", report_current_limit_state, { .noargs = On, .allow_blocking = On }, { .str = "output current limit pins" } },
    { "SD", report_spindle_data, { .help_fn = On }, { .fn = help_spindle } },
    { "SR", spindle_reset_data, { .help_fn = On }, { .fn = help_spindle } },
    { "RTC", rtc_action, { .allow_blocking = On, .help_fn = On }, { .fn = help_rtc } },
#ifdef DEBUGOUT
    { "Q", output_memmap, { .noargs = On }, { .str = "output NVS memory allocation" } },
#endif
};

static sys_commands_t core_commands = {
    .n_commands = sizeof(sys_commands) / sizeof(sys_command_t),
    .commands = sys_commands,
};

static sys_commands_t *commands_root = &core_commands;

void system_register_commands (sys_commands_t *commands)
{
    commands->next = commands_root;
    commands_root = commands;
}

void _system_output_help (sys_commands_t *commands, bool traverse)
{
    const char *help;
    uint_fast8_t idx;

    while(commands) {
        for(idx = 0; idx < commands->n_commands; idx++) {

            if(commands->commands[idx].help.str) {

                if(commands->commands[idx].flags.help_fn)
                    help = commands->commands[idx].help.fn(commands->commands[idx].command);
                else
                    help = commands->commands[idx].help.str;

                if(help) {
                    if(*help != '$') {
                        hal.stream.write_char('$');
                        hal.stream.write(commands->commands[idx].command);
                        hal.stream.write(" - ");
                    }
                    hal.stream.write(help);
                    hal.stream.write("." ASCII_EOL);
                }
            }
        }
        commands = traverse && commands->next != &core_commands ? commands->next : NULL;
    }
}

// Deprecated, to be removed.
void system_output_help (const sys_command_t *commands, uint32_t num_commands)
{
    sys_commands_t cmd = {
         .commands = commands,
         .n_commands = num_commands
    };

    _system_output_help(&cmd, false);
}

void system_command_help (void)
{
    _system_output_help(&core_commands, false);
    if(commands_root != &core_commands)
        _system_output_help(commands_root, true);
}

/*! \brief Directs and executes one line of input from protocol_process.

While mostly incoming streaming g-code blocks, this also executes grblHAL internal commands, such as
settings, initiating the homing cycle, and toggling switch states. This differs from
the realtime command module by being susceptible to when grblHAL is ready to execute the
next line during a cycle, so for switches like block delete, the switch only effects
the lines that are processed afterward, not necessarily real-time during a cycle,
since there are motions already stored in the buffer. However, this 'lag' should not
be an issue, since these commands are not typically used during a cycle.
If the command is not known to the core a grbl.on_unknown_sys_command event is raised
so that plugin code can check if it is a command it supports.

__NOTE:__ Code calling this function needs to provide the command in a writable buffer since
 the first part of the command (up to the first = character) is changed to uppercase and having
 spaces removed.

\param line pointer to the command string.
\returns \a status_code_t enum value; #Status_OK if successfully handled, another relevant status code if not.
*/
status_code_t system_execute_line (char *line)
{
    if(line[1] == '\0') {
        grbl.report.help_message();
        return Status_OK;
    }

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
    sys_commands_t *cmd = commands_root;
    do {
        for(idx = 0; idx < cmd->n_commands; idx++) {
            if(!strcmp(line, cmd->commands[idx].command)) {
                if(sys.blocking_event && !cmd->commands[idx].flags.allow_blocking) {
                    retval = Status_NotAllowedCriticalEvent;
                    break;
                } else if(!cmd->commands[idx].flags.noargs || args == NULL) {
                    if((retval = cmd->commands[idx].execute(state_get(), args)) != Status_Unhandled)
                        break;
                }
            }
        }
        cmd = retval == Status_Unhandled ? cmd->next : NULL;
    } while(cmd);

    // deprecated, to be removed
    if(retval == Status_Unhandled && (cmd = grbl.on_get_commands ? grbl.on_get_commands() : NULL)) {

        do {
            for(idx = 0; idx < cmd->n_commands; idx++) {
                if(!strcmp(line, cmd->commands[idx].command)) {
                    if(sys.blocking_event && !cmd->commands[idx].flags.allow_blocking) {
                        retval = Status_NotAllowedCriticalEvent;
                        break;
                    } else if(!cmd->commands[idx].flags.noargs || args == NULL) {
                        if((retval = cmd->commands[idx].execute(state_get(), args)) != Status_Unhandled)
                            break;
                    }
                }
            }
            cmd = retval == Status_Unhandled && cmd->on_get_commands ? cmd->on_get_commands() : NULL;
        } while(cmd);
    }
    // end of to be removed

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

// End system $-commands

/*! \brief Called on homing state changes.

Clears the tool length offset (TLO) when linear axis or X- or Z-axis is homed in lathe mode.
Typically called on the grbl.on_homing complete event.
\param id a \a axes_signals_t holding the axis flags that homed status was changed for.
*/
void system_clear_tlo_reference (axes_signals_t homing_cycle)
{
    plane_t plane;

#if TOOL_LENGTH_OFFSET_AXIS >= 0
    plane.axis_linear = TOOL_LENGTH_OFFSET_AXIS;
#else
    gc_get_plane_data(&plane, gc_state.modal.plane_select);
#endif
    if(homing_cycle.mask & (settings.mode == Mode_Lathe ? (X_AXIS_BIT|Z_AXIS_BIT) : bit(plane.axis_linear))) {
        if(sys.tlo_reference_set.mask != 0) {
            sys.tlo_reference_set.mask = 0;  // Invalidate tool length offset reference
            system_add_rt_report(Report_TLOReference);
        }
    }
}

/*! \brief Called on a work coordinate (WCO) changes.

If configured waits for the planner buffer to empty then fires the
grbl.on_wco_changed event and sets the #Report_WCO flag to add
the WCO report element to the next status report.
*/
void system_flag_wco_change (void)
{
    if(!settings.status_report.sync_on_wco_change)
        protocol_buffer_synchronize();

    if(grbl.on_wco_changed)
        grbl.on_wco_changed();

    system_add_rt_report(Report_WCO);
}

/*! \brief Sets machine position. Must be sent a 'step' array.

__NOTE:__ If motor steps and machine position are not in the same coordinate frame,
          this function serves as a central place to compute the transformation.
\param position pointer to the target float array for the machine position.
\param steps pointer to the source step count array to transform.
 */
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

/*! \brief Checks if XY position is within coordinate system XY with given tolerance.
\param id a \a coord_system_id_t, typically #CoordinateSystem_G59_3.
\param tolerance as the allowed radius the current position has to be within.
\returns \a false if tolerance is 0 or position is outside the allowed radius, otherwise \a true.
*/
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

/*! \brief Raise and report a system alarm.
\param a #alarm_code_t enum representing the alarm code.
 */
void system_raise_alarm (alarm_code_t alarm)
{
    if(state_get() == STATE_HOMING && !(sys.rt_exec_state & EXEC_RESET))
        system_set_exec_alarm(alarm);
    else if(sys.alarm != alarm) {
        sys.alarm = alarm;
        sys.blocking_event = sys.alarm == Alarm_HardLimit ||
                              sys.alarm == Alarm_SoftLimit ||
                               sys.alarm == Alarm_EStop ||
                                sys.alarm == Alarm_MotorFault;
        state_set(alarm == Alarm_EStop ? STATE_ESTOP : STATE_ALARM);
        if(sys.driver_started || sys.alarm == Alarm_SelftestFailed)
            grbl.report.alarm_message(alarm);
    }
}

// TODO: encapsulate sys.report

/*! \brief Get the active realtime report addon flags for the next report.
\return a #report_tracking_flags_t union containing the flags.
 */
report_tracking_flags_t system_get_rt_report_flags (void)
{
    return sys.report;
}

/*! \brief Set(s) or clear all active realtime report addon flag(s) for the next report.

Fires the \ref grbl.on_rt_reports_added event.
\param report a #report_tracking_t enum containing the flag(s) to set or clear.
 */
void system_add_rt_report (report_tracking_t report)
{
    switch(report) {

        case Report_ClearAll:
            sys.report.value = 0;
            return;

        case Report_MPGMode:
            if(!hal.driver_cap.mpg_mode)
                return;
            break;

        case Report_LatheXMode:
            sys.report.wco = settings.status_report.work_coord_offset;
            break;

        default:
            break;
    }

    sys.report.value |= (uint32_t)report;

    if(sys.report.value && grbl.on_rt_reports_added)
        grbl.on_rt_reports_added((report_tracking_flags_t)((uint32_t)report));
}
