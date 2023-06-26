/*
  report.c - reporting and messaging methods

  Part of grblHAL

  Copyright (c) 2017-2023 Terje Io
  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC

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

/*
  This file functions as the primary feedback interface for Grbl. Any outgoing data, such
  as the protocol status messages, feedback messages, and status reports, are stored here.
  For the most part, these functions primarily are called from protocol.c methods. If a
  different style feedback is desired (i.e. JSON), then a user can change these following
  methods to accommodate their needs.
*/

#include <math.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#include "hal.h"
#include "report.h"
#include "nvs_buffer.h"
#include "machine_limits.h"
#include "state_machine.h"
#include "regex.h"

#if ENABLE_SPINDLE_LINEARIZATION
#include <stdio.h>
#endif

static char buf[(STRLEN_COORDVALUE + 1) * N_AXIS];
static char *(*get_axis_values)(float *axis_values);
static char *(*get_axis_value)(float value);
static char *(*get_rate_value)(float value);
static uint8_t override_counter = 0; // Tracks when to add override data to status reports.
static uint8_t wco_counter = 0;      // Tracks when to add work coordinate offset data to status reports.
static const char vbar[2] = { '|', '\0' };

// Append a number of strings to the static buffer
// NOTE: do NOT use for several int/float conversions as these share the same underlying buffer!
static char *appendbuf (int argc, ...)
{
    char c, *s = buf, *arg;

    va_list list;
    va_start(list, argc);

    while(argc--) {
        arg = va_arg(list, char *);
        do {
            c = *s++ = *arg++;
        } while(c);
        s--;
    }

    va_end(list);

    return buf;
}

static char *map_coord_system (coord_system_id_t id)
{
    uint8_t g5x = id + 54;

    strcpy(buf, uitoa((uint32_t)(g5x > 59 ? 59 : g5x)));
    if(g5x > 59) {
        strcat(buf, ".");
        strcat(buf, uitoa((uint32_t)(g5x - 59)));
    }

    return buf;
}

// Convert axis position values to null terminated string (mm).
static char *get_axis_values_mm (float *axis_values)
{
    uint_fast32_t idx;

    buf[0] = '\0';

    for (idx = 0; idx < N_AXIS; idx++) {
        if(idx == X_AXIS && gc_state.modal.diameter_mode)
            strcat(buf, ftoa(axis_values[idx] * 2.0f, N_DECIMAL_COORDVALUE_MM));
        else
            strcat(buf, ftoa(axis_values[idx], N_DECIMAL_COORDVALUE_MM));
        if (idx < (N_AXIS - 1))
            strcat(buf, ",");
    }

    return buf;
}

// Convert axis position values to null terminated string (inch).
static char *get_axis_values_inches (float *axis_values)
{
    uint_fast32_t idx;

    buf[0] = '\0';

    for (idx = 0; idx < N_AXIS; idx++) {
        if(idx == X_AXIS && gc_state.modal.diameter_mode)
            strcat(buf, ftoa(axis_values[idx] * INCH_PER_MM * 2.0f, N_DECIMAL_COORDVALUE_INCH));
#if N_AXIS > 3
        else if(idx > Z_AXIS && bit_istrue(settings.steppers.is_rotational.mask, bit(idx)))
            strcat(buf, ftoa(axis_values[idx], N_DECIMAL_COORDVALUE_MM));
#endif
        else
             strcat(buf, ftoa(axis_values[idx] * INCH_PER_MM, N_DECIMAL_COORDVALUE_INCH));
        if (idx < (N_AXIS - 1))
            strcat(buf, ",");
    }

    return buf;
}

// Convert rate value to null terminated string (mm).
static char *get_axis_value_mm (float value)
{
    return strcpy(buf, ftoa(value, N_DECIMAL_COORDVALUE_MM));
}

// Convert rate value to null terminated string (mm).
static char *get_axis_value_inches (float value)
{
    return strcpy(buf, ftoa(value * INCH_PER_MM, N_DECIMAL_COORDVALUE_INCH));
}

// Convert rate value to null terminated string (mm).
static char *get_rate_value_mm (float value)
{
    return uitoa((uint32_t)value);
}

// Convert rate value to null terminated string (mm).
static char *get_rate_value_inch (float value)
{
    return uitoa((uint32_t)(value * INCH_PER_MM));
}

// Convert axes signals bits to string representation.
// NOTE: returns pointer to null terminator!
inline static char *axis_signals_tostring (char *buf, axes_signals_t signals)
{
    uint_fast16_t idx = 0;

    signals.mask &= AXES_BITMASK;

    while(signals.mask) {
        if(signals.mask & 0x01)
            *buf++ = *axis_letter[idx];
        idx++;
        signals.mask >>= 1;
    };

    *buf = '\0';

    return buf;
}

// Convert control signals bits to string representation.
// NOTE: returns pointer to null terminator!
inline static char *control_signals_tostring (char *buf, control_signals_t signals)
{
    static const char signals_map[] = "RHSDLTE FM    P ";

    char *map = (char *)signals_map;

    if(!hal.signals_cap.stop_disable)
        signals.stop_disable = sys.flags.optional_stop_disable;

    if(!signals.deasserted)
      while(signals.mask) {

        if(signals.mask & 0x01) {

            switch(*map) {

                case ' ':
                    break;

                case 'D':
                    if(hal.signals_cap.safety_door_ajar)
                        *buf++ = *map;
                    break;

                case 'L':
                    if(sys.flags.block_delete_enabled)
                        *buf++ = *map;
                    break;

                default:
                    *buf++ = *map;
                    break;
            }
        }

        map++;
        signals.mask >>= 1;
    }

    *buf = '\0';

    return buf;
}

void report_init (void)
{
    get_axis_value = settings.flags.report_inches ? get_axis_value_inches : get_axis_value_mm;
    get_axis_values = settings.flags.report_inches ? get_axis_values_inches : get_axis_values_mm;
    get_rate_value = settings.flags.report_inches ? get_rate_value_inch : get_rate_value_mm;
}

// Handles the primary confirmation protocol response for streaming interfaces and human-feedback.
// For every incoming line, this method responds with an 'ok' for a successful command or an
// 'error:'  to indicate some error event with the line or some critical system error during
// operation. Errors events can originate from the g-code parser, settings module, or asynchronously
// from a critical error, such as a triggered hard limit. Interface should always monitor for these
// responses.
static status_code_t report_status_message (status_code_t status_code)
{
    switch(status_code) {

        case Status_OK: // STATUS_OK
            hal.stream.write("ok" ASCII_EOL);
            break;

        default:
            hal.stream.write(appendbuf(3, "error:", uitoa((uint32_t)status_code), ASCII_EOL));
            break;
    }

    return status_code;
}

// Prints alarm messages.
static alarm_code_t report_alarm_message (alarm_code_t alarm_code)
{
    hal.stream.write_all(appendbuf(3, "ALARM:", uitoa((uint32_t)alarm_code), ASCII_EOL));
    hal.delay_ms(100, NULL); // Force delay to ensure message clears output stream buffer.

    return alarm_code;
}

// Prints feedback message, typically from gcode.
void report_message (const char *msg, message_type_t type)
{
    hal.stream.write("[MSG:");

    switch(type) {

        case Message_Info:
            hal.stream.write("Info: ");
            break;

        case Message_Warning:
            hal.stream.write("Warning: ");
            break;

        default:
            break;
    }

    hal.stream.write(msg);
    hal.stream.write("]" ASCII_EOL);
}

// Prints feedback messages. This serves as a centralized method to provide additional
// user feedback for things that are not of the status/alarm message protocol. These are
// messages such as setup warnings, switch toggling, and how to exit alarms.
// NOTE: For interfaces, messages are always placed within brackets. And if silent mode
// is installed, the message number codes are less than zero.
static message_code_t report_feedback_message (message_code_t id)
{
    const message_t *msg = message_get(id);

    report_message(msg ? msg->text : "", msg ? msg->type : Message_Plain);

    if(id == Message_None && grbl.on_gcode_message)
        grbl.on_gcode_message("");

    return id;
}

// Welcome message
static void report_init_message (void)
{
    override_counter = wco_counter = 0;
#if COMPATIBILITY_LEVEL == 0
    hal.stream.write_all(ASCII_EOL "GrblHAL " GRBL_VERSION " ['$' or '$HELP' for help]" ASCII_EOL);
#else
    hal.stream.write_all(ASCII_EOL "Grbl " GRBL_VERSION " ['$' for help]" ASCII_EOL);
#endif
}

// grblHAL help message
static void report_help_message (void)
{
    hal.stream.write("[HLP:$$ $# $G $I $N $x=val $Nx=line $J=line $SLP $C $X $H $B ~ ! ? ctrl-x]" ASCII_EOL);
}

static bool report_group_settings (const setting_group_detail_t *groups, const uint_fast8_t n_groups, char *args)
{
    bool found = false;
    uint_fast8_t idx;
    char c, *s, group[26];

    for(idx = 0; idx < n_groups; idx++) {

        s = group;
        strncpy(group, groups[idx].name, sizeof(group) - 1);

        // Uppercase group name
        while((c = *s))
            *s++ = CAPS(c);

        if((found = matchhere(args, group))) {
            hal.stream.write(ASCII_EOL "---- ");
            hal.stream.write(groups[idx].name);
            hal.stream.write(":" ASCII_EOL);
            report_settings_details(SettingsFormat_HumanReadable, Setting_SettingsAll, groups[idx].id);
            break;
        }
    }

    return found;
}

status_code_t report_help (char *args)
{
    // Strip leading spaces
    while(*args == ' ')
        args++;

    if(*args == '\0') {

        hal.stream.write("Help topics:" ASCII_EOL);
        hal.stream.write(" Commands" ASCII_EOL);
        hal.stream.write(" Settings" ASCII_EOL);
        report_setting_group_details(false, " ");

    } else {

        char c, *s = args;

        // Upper case argument
        while((c = *s))
            *s++ = CAPS(c);

        if(matchhere(args, "COMMANDS")) {
            if(grbl.on_report_command_help)
                grbl.on_report_command_help();

        } else if(matchhere(args, "SETTINGS"))
            report_settings_details(SettingsFormat_HumanReadable, Setting_SettingsAll, Group_All);

        else {

            bool found = false;
            setting_details_t *settings_info = settings_get_details();

            found = report_group_settings(settings_info->groups, settings_info->n_groups, args);

            if(!found && (settings_info = settings_info->next)) do {
                if(settings_info->groups && (found = report_group_settings(settings_info->groups, settings_info->n_groups, args)))
                    break;
            } while((settings_info = settings_info->next));

            if(!found)
                hal.stream.write( ASCII_EOL "N/A" ASCII_EOL);
        }
    }

    return Status_OK;
}


// Grbl settings print out.

static int cmp_settings (const void *a, const void *b)
{
  return (*(setting_detail_t **)(a))->id - (*(setting_detail_t **)(b))->id;
}

static bool report_setting (const setting_detail_t *setting, uint_fast16_t offset, void *data)
{
    appendbuf(3, "$", uitoa(setting->id + offset), "=");

    char *value = setting_get_value(setting, offset);

    if(value) {
        hal.stream.write(buf);
        hal.stream.write(value);
        hal.stream.write(ASCII_EOL);
    }

    return true;
}

status_code_t report_grbl_setting (setting_id_t id, void *data)
{
    status_code_t status = Status_OK;

    const setting_detail_t *setting = setting_get_details(id, NULL);

    if(setting)
        grbl.report.setting(setting, id - setting->id, data);
    else
        status = Status_SettingDisabled;

    return status;
}

static bool print_setting (const setting_detail_t *setting, uint_fast16_t offset, void *data)
{
    if(setting->value != NULL)
        grbl.report.setting(setting, offset, data);
    else {
        hal.stream.write("$");
        hal.stream.write(uitoa(setting->id));
        hal.stream.write("=N/A" ASCII_EOL);
    }

    return true;
}

void report_grbl_settings (bool all, void *data)
{
    uint_fast16_t idx, n_settings = 0;
    const setting_detail_t *setting;
    setting_detail_t **all_settings, **psetting;
    setting_details_t *details = settings_get_details();

    do {
        n_settings += details->n_settings;
    } while((details = details->next));

    details = settings_get_details();

    if((all_settings = psetting = calloc(n_settings, sizeof(setting_detail_t *)))) {

        n_settings = 0;

        // Report core settings
        for(idx = 0; idx < details->n_settings; idx++) {
            setting = &details->settings[idx];
            if((all || setting->type == Setting_IsLegacy || setting->type == Setting_IsLegacyFn) &&
                  (setting->is_available == NULL ||setting->is_available(setting))) {
                *psetting++ = (setting_detail_t *)setting;
                n_settings++;
            }
        }

        // Report driver and plugin settings
        if(all && (details = details->next)) do {
            for(idx = 0; idx < details->n_settings; idx++) {
                setting = &details->settings[idx];
                if(setting->is_available == NULL ||setting->is_available(setting)) {
                    *psetting++ = (setting_detail_t *)setting;
                    n_settings++;
                }
            }
        } while((details = details->next));

        qsort(all_settings, n_settings, sizeof(setting_detail_t *), cmp_settings);

        for(idx = 0; idx < n_settings; idx++)
            settings_iterator(all_settings[idx], print_setting, data);

        free(all_settings);

    } else do {
        for(idx = 0; idx < n_settings; idx++)
            settings_iterator(&details->settings[idx], print_setting, data);
    } while((details = details->next));
}


// Prints current probe parameters. Upon a probe command, these parameters are updated upon a
// successful probe or upon a failed probe with the G38.3 without errors command (if supported).
// These values are retained until Grbl is power-cycled, whereby they will be re-zeroed.
void report_probe_parameters (void)
{
    // Report in terms of machine position.
    float print_position[N_AXIS];
    system_convert_array_steps_to_mpos(print_position, sys.probe_position);
    hal.stream.write("[PRB:");
    hal.stream.write(get_axis_values(print_position));
    hal.stream.write(sys.flags.probe_succeeded ? ":1" : ":0");
    hal.stream.write("]" ASCII_EOL);
}

// Prints current home position in terms of machine position.
// Bitmask for homed axes attached.
void report_home_position (void)
{
    hal.stream.write("[HOME:");
    hal.stream.write(get_axis_values(sys.home_position));
    hal.stream.write(":");
    hal.stream.write(uitoa(sys.homed.mask));
    hal.stream.write("]" ASCII_EOL);
}

// Prints current tool offsets.
void report_tool_offsets (void)
{
    hal.stream.write("[TLO:");
#if TOOL_LENGTH_OFFSET_AXIS >= 0
    hal.stream.write(get_axis_value(gc_state.tool_length_offset[TOOL_LENGTH_OFFSET_AXIS]));
#else
    hal.stream.write(get_axis_values(gc_state.tool_length_offset));
#endif
    hal.stream.write("]" ASCII_EOL);
}

// Prints NIST/LinuxCNC NGC parameter value
status_code_t report_ngc_parameter (ngc_param_id_t id)
{
    float value;

    hal.stream.write("[PARAM:");
    hal.stream.write(uitoa(id));
    if(ngc_param_get(id, &value)) {
        hal.stream.write("=");
        hal.stream.write(ftoa(value, 3));
    } else
        hal.stream.write("=N/A");
    hal.stream.write("]" ASCII_EOL);

    return Status_OK;
}

// Prints named LinuxCNC NGC parameter value
status_code_t report_named_ngc_parameter (char *arg)
{
    float value;

    hal.stream.write("[PARAM:");
    hal.stream.write(arg);
    if(ngc_named_param_get(arg, &value)) {
        hal.stream.write("=");
        hal.stream.write(ftoa(value, 3));
    } else
        hal.stream.write("=N/A");
    hal.stream.write("]" ASCII_EOL);

    return Status_OK;
}


// Prints Grbl NGC parameters (coordinate offsets, probing, tool table)
void report_ngc_parameters (void)
{
    uint_fast8_t idx;
    float coord_data[N_AXIS];

    if(gc_state.modal.scaling_active) {
        hal.stream.write("[G51:");
        hal.stream.write(get_axis_values(gc_get_scaling()));
        hal.stream.write("]" ASCII_EOL);
    }

    for (idx = 0; idx < N_CoordinateSystems; idx++) {

        if (!(settings_read_coord_data((coord_system_id_t)idx, &coord_data))) {
            grbl.report.status_message(Status_SettingReadFail);
            return;
        }

        hal.stream.write("[G");

        switch (idx) {

            case CoordinateSystem_G28:
                hal.stream.write("28");
                break;

            case CoordinateSystem_G30:
                hal.stream.write("30");
                break;

            case CoordinateSystem_G92:
                break;

            default: // G54-G59
                hal.stream.write(map_coord_system((coord_system_id_t)idx));
                break;
        }

        if(idx != CoordinateSystem_G92) {
            hal.stream.write(":");
            hal.stream.write(get_axis_values(coord_data));
            hal.stream.write("]" ASCII_EOL);
        }
    }

    // Print G92, G92.1 which are not persistent in memory
    hal.stream.write("92:");
    hal.stream.write(get_axis_values(gc_state.g92_coord_offset));
    hal.stream.write("]" ASCII_EOL);

#if N_TOOLS
    for (idx = 1; idx <= N_TOOLS; idx++) {
        hal.stream.write("[T:");
        hal.stream.write(uitoa((uint32_t)idx));
        hal.stream.write("|");
        hal.stream.write(get_axis_values(tool_table[idx].offset));
        hal.stream.write("|");
        hal.stream.write(get_axis_value(tool_table[idx].radius));
        hal.stream.write("]" ASCII_EOL);
    }
#endif

#if COMPATIBILITY_LEVEL < 10
    if(settings.homing.flags.enabled)
        report_home_position();
#endif

    report_tool_offsets();      // Print tool length offset value.
    report_probe_parameters();  // Print probe parameters. Not persistent in memory.
    if(sys.tlo_reference_set.mask) { // Print tool length reference offset. Not persistent in memory.
        plane_t plane;
        gc_get_plane_data(&plane, gc_state.modal.plane_select);
        hal.stream.write("[TLR:");
        hal.stream.write(get_axis_value(sys.tlo_reference[plane.axis_linear] / settings.axis[plane.axis_linear].steps_per_mm));
        hal.stream.write("]" ASCII_EOL);
    }
}

static inline bool is_g92_active (void)
{
    bool active = false;
    uint_fast32_t idx = N_AXIS;

    do {
        idx--;
        active = !(gc_state.g92_coord_offset[idx] == 0.0f || gc_state.g92_coord_offset[idx] == -0.0f);
    } while(idx && !active);

    return active;
}

// Print current gcode parser mode state
void report_gcode_modes (void)
{
    hal.stream.write("[GC:G");
    if (gc_state.modal.motion >= MotionMode_ProbeToward) {
        hal.stream.write("38.");
        hal.stream.write(uitoa((uint32_t)(gc_state.modal.motion - (MotionMode_ProbeToward - 2))));
    } else
        hal.stream.write(uitoa((uint32_t)gc_state.modal.motion));

    hal.stream.write(" G");
    hal.stream.write(map_coord_system(gc_state.modal.coord_system.id));

#if COMPATIBILITY_LEVEL < 10

    if(is_g92_active())
        hal.stream.write(" G92");

#endif

    if(settings.mode == Mode_Lathe)
        hal.stream.write(gc_state.modal.diameter_mode ? " G7" : " G8");

    hal.stream.write(" G");
    hal.stream.write(uitoa((uint32_t)(gc_state.modal.plane_select + 17)));

    hal.stream.write(gc_state.modal.units_imperial ? " G20" : " G21");

    hal.stream.write(gc_state.modal.distance_incremental ? " G91" : " G90");

    hal.stream.write(" G");
    hal.stream.write(uitoa((uint32_t)(94 - gc_state.modal.feed_mode)));

    if(settings.mode == Mode_Lathe && gc_spindle_get()->cap.variable)
        hal.stream.write(gc_state.modal.spindle.rpm_mode == SpindleSpeedMode_RPM ? " G97" : " G96");

#if COMPATIBILITY_LEVEL < 10

    if(gc_state.modal.tool_offset_mode == ToolLengthOffset_Cancel)
        hal.stream.write(" G49");
    else {
        hal.stream.write(" G43");
        if(gc_state.modal.tool_offset_mode != ToolLengthOffset_Enable)
            hal.stream.write(gc_state.modal.tool_offset_mode == ToolLengthOffset_EnableDynamic ? ".1" : ".2");
    }

    hal.stream.write(gc_state.canned.retract_mode == CCRetractMode_RPos ? " G99" : " G98");

    if(gc_state.modal.scaling_active) {
        hal.stream.write(" G51:");
        axis_signals_tostring(buf, gc_get_g51_state());
        hal.stream.write(buf);
    } else
        hal.stream.write(" G50");

#endif

    if (gc_state.modal.program_flow) {

        switch (gc_state.modal.program_flow) {

            case ProgramFlow_Paused:
                hal.stream.write(" M0");
                break;

            case ProgramFlow_OptionalStop:
                hal.stream.write(" M1");
                break;

            case ProgramFlow_CompletedM2:
                hal.stream.write(" M2");
                break;

            case ProgramFlow_CompletedM30:
                hal.stream.write(" M30");
                break;

            case ProgramFlow_CompletedM60:
                hal.stream.write(" M60");
                break;

            default:
                break;
        }
    }

    hal.stream.write(gc_state.modal.spindle.state.on ? (gc_state.modal.spindle.state.ccw ? " M4" : " M3") : " M5");

    if(gc_state.tool_change)
        hal.stream.write(" M6");

    if (gc_state.modal.coolant.value) {

        if (gc_state.modal.coolant.mist)
             hal.stream.write(" M7");

        if (gc_state.modal.coolant.flood)
            hal.stream.write(" M8");

    } else
        hal.stream.write(" M9");

    if (sys.override.control.feed_rate_disable)
        hal.stream.write(" M50");

    if (sys.override.control.spindle_rpm_disable)
        hal.stream.write(" M51");

    if (sys.override.control.feed_hold_disable)
        hal.stream.write(" M53");

    if (settings.parking.flags.enable_override_control && sys.override.control.parking_disable)
        hal.stream.write(" M56");

    hal.stream.write(appendbuf(2, " T", uitoa((uint32_t)gc_state.tool->tool)));

    hal.stream.write(appendbuf(2, " F", get_rate_value(gc_state.feed_rate)));

    if(gc_spindle_get()->cap.variable)
        hal.stream.write(appendbuf(2, " S", ftoa(gc_state.spindle.rpm, N_DECIMAL_RPMVALUE)));

    hal.stream.write("]" ASCII_EOL);
}

// Prints specified startup line
void report_startup_line (uint8_t n, char *line)
{
    hal.stream.write(appendbuf(3, "$N", uitoa((uint32_t)n), "="));
    hal.stream.write(line);
    hal.stream.write(ASCII_EOL);
}

void report_execute_startup_message (char *line, status_code_t status_code)
{
    hal.stream.write(">");
    hal.stream.write(line);
    hal.stream.write(":");
    grbl.report.status_message(status_code);
}

// Prints build info line
void report_build_info (char *line, bool extended)
{
    char buf[100];

    hal.stream.write("[VER:" GRBL_VERSION ".");
    hal.stream.write(uitoa(GRBL_BUILD));
    hal.stream.write(":");
    hal.stream.write(line);
    hal.stream.write("]" ASCII_EOL);

#if COMPATIBILITY_LEVEL == 0
    extended = true;
#endif

    // Generate compile-time build option list

    char *append = &buf[5];

    strcpy(buf, "[OPT:");

    if(spindle_get_caps(false).variable)
        *append++ = 'V';

    *append++ = 'N';

    if(hal.driver_cap.mist_control)
        *append++ = 'M';

#if COREXY
    *append++ = 'C';
#endif

    if(settings.parking.flags.enabled)
        *append++ = 'P';

    if(settings.homing.flags.force_set_origin)
        *append++ = 'Z';

    if(settings.homing.flags.single_axis_commands)
        *append++ = 'H';

    if(settings.limits.flags.two_switches)
        *append++ = 'T';

    if(settings.probe.allow_feed_override)
        *append++ = 'A';

    if(settings.spindle.flags.enable_rpm_controlled)
        *append++ = '0';

    if(hal.driver_cap.software_debounce)
        *append++ = 'S';

    if(settings.parking.flags.enable_override_control)
        *append++ = 'R';

    if(!settings.homing.flags.init_lock)
        *append++ = 'L';

    if(hal.signals_cap.safety_door_ajar)
        *append++ = '+';

  #if !ENABLE_RESTORE_NVS_WIPE_ALL // NOTE: Shown when disabled.
    *append++ = '*';
  #endif

  #if !ENABLE_RESTORE_NVS_DEFAULT_SETTINGS // NOTE: Shown when disabled.
    *append++ = '$';
  #endif

  #if !ENABLE_RESTORE_NVS_CLEAR_PARAMETERS // NOTE: Shown when disabled.
    *append++ = '#';
  #endif

  #if DISABLE_BUILD_INFO_WRITE_COMMAND // NOTE: Shown when disabled.
    *append++ = 'I';
  #endif

    if(!settings.status_report.sync_on_wco_change) // NOTE: Shown when disabled.
        *append++ = 'W';

    if(hal.stepper.get_ganged)
        *append++ = '2';

    *append++ = ',';
    *append = '\0';
    hal.stream.write(buf);

    // NOTE: Compiled values, like override increments/max/min values, may be added at some point later.
    hal.stream.write(uitoa((uint32_t)plan_get_buffer_size()));
    hal.stream.write(",");
    hal.stream.write(uitoa(hal.rx_buffer_size));
    if(extended) {
        hal.stream.write(",");
        hal.stream.write(uitoa((uint32_t)N_AXIS));
        hal.stream.write(",");
  #if N_TOOLS
        hal.stream.write(uitoa((uint32_t)N_TOOLS));
  #else
        hal.stream.write("0");
  #endif
    }
    hal.stream.write("]" ASCII_EOL);

    if(extended) {

        uint_fast8_t idx;
        nvs_io_t *nvs = nvs_buffer_get_physical();

        strcat(strcpy(buf, "[AXS:"), uitoa(N_AXIS));

        append = &buf[6];
        *append++ = ':';

        for(idx = 0; idx < N_AXIS; idx++)
            *append++ = *axis_letter[idx];

        *append = '\0';

        hal.stream.write(strcat(buf, "]" ASCII_EOL));

        strcpy(buf, "[NEWOPT:ENUMS,RT");
        strcat(buf, settings.flags.legacy_rt_commands ? "+," : "-,");

        if(settings.homing.flags.enabled)
            strcat(buf, "HOME,");

        if(!hal.probe.get_state)
            strcat(buf, "NOPROBE,");
        else if(hal.signals_cap.probe_disconnected)
            strcat(buf, "PC,");

        if(hal.signals_cap.stop_disable)
            strcat(buf, "OS,");

        if(hal.signals_cap.block_delete)
            strcat(buf, "BD,");

        if(hal.signals_cap.e_stop)
            strcat(buf, "ES,");

        if(hal.driver_cap.mpg_mode)
            strcat(buf, "MPG,");

        if(settings.mode == Mode_Lathe)
            strcat(buf, "LATHE,");

        if(hal.driver_cap.laser_ppi_mode)
            strcat(buf, "PPI,");

        if(hal.reboot)
            strcat(buf, "REBOOT,");

    #if NGC_EXPRESSIONS_ENABLE
        strcat(buf, "EXPR,");
    #endif

        if(hal.tool.change)
            strcat(buf, hal.driver_cap.atc ? "ATC," : "TC,"); // Tool change supported (M6)

        if(hal.driver_cap.spindle_sync)
            strcat(buf, "SS,");

    #ifndef NO_SETTINGS_DESCRIPTIONS
        strcat(buf, "SED,");
    #endif

        if(hal.rtc.get_datetime)
            strcat(buf, "RTC,");

    #ifdef PID_LOG
        strcat(buf, "PID,");
    #endif

        append = &buf[strlen(buf) - 1];
        if(*append == ',')
            *append = '\0';

        hal.stream.write(buf);
        grbl.on_report_options(true);
        hal.stream.write("]" ASCII_EOL);

        hal.stream.write("[FIRMWARE:grblHAL]" ASCII_EOL);

        if(!(nvs->type == NVS_None || nvs->type == NVS_Emulated)) {
            hal.stream.write("[NVS STORAGE:");
            *buf = '\0';
            if(hal.nvs.type == NVS_Emulated)
                strcat(buf, "*");
            strcat(buf, nvs->type == NVS_Flash ? "FLASH" : (nvs->type == NVS_FRAM ? "FRAM" : "EEPROM"));
            hal.stream.write(buf);
            hal.stream.write("]" ASCII_EOL);
        }

        if(hal.info) {
            hal.stream.write("[DRIVER:");
            hal.stream.write(hal.info);
            hal.stream.write("]" ASCII_EOL);
        }

        if(hal.driver_version) {
            hal.stream.write("[DRIVER VERSION:");
            hal.stream.write(hal.driver_version);
            hal.stream.write("]" ASCII_EOL);
        }

        if(hal.driver_options) {
            hal.stream.write("[DRIVER OPTIONS:");
            hal.stream.write(hal.driver_options);
            hal.stream.write("]" ASCII_EOL);
        }

        if(hal.board) {
            hal.stream.write("[BOARD:");
            hal.stream.write(hal.board);
            hal.stream.write("]" ASCII_EOL);
        }

        if(hal.max_step_rate) {
            hal.stream.write("[MAX STEP RATE:");
            hal.stream.write(uitoa(hal.max_step_rate));
            hal.stream.write(" Hz]" ASCII_EOL);
        }

#if COMPATIBILITY_LEVEL > 0
        hal.stream.write("[COMPATIBILITY LEVEL:");
        hal.stream.write(uitoa(COMPATIBILITY_LEVEL));
        hal.stream.write("]" ASCII_EOL);
#endif

        if(hal.port.num_digital_in + hal.port.num_digital_out + hal.port.num_analog_in + hal.port.num_analog_out > 0) {
            hal.stream.write("[AUX IO:");
            hal.stream.write(uitoa(hal.port.num_digital_in));
            hal.stream.write(",");
            hal.stream.write(uitoa(hal.port.num_digital_out));
            hal.stream.write(",");
            hal.stream.write(uitoa(hal.port.num_analog_in));
            hal.stream.write(",");
            hal.stream.write(uitoa(hal.port.num_analog_out));
            hal.stream.write("]" ASCII_EOL);
        }

        grbl.on_report_options(false);
    }
}


// Prints the character string line Grbl has received from the user, which has been pre-parsed,
// and has been sent into protocol_execute_line() routine to be executed by Grbl.
void report_echo_line_received (char *line)
{
    hal.stream.write("[echo: ");
    hal.stream.write(line);
    hal.stream.write("]" ASCII_EOL);
}


 // Prints real-time data. This function grabs a real-time snapshot of the stepper subprogram
 // and the actual location of the CNC machine. Users may change the following function to their
 // specific needs, but the desired real-time data report must be as short as possible. This is
 // requires as it minimizes the computational overhead and allows grbl to keep running smoothly,
 // especially during g-code programs with fast, short line segments and high frequency reports (5-20Hz).
void report_realtime_status (void)
{
    static bool probing = false;

    float print_position[N_AXIS];
    report_tracking_flags_t report = system_get_rt_report_flags();
    probe_state_t probe_state = {
        .connected = On,
        .triggered = Off
    };

    system_convert_array_steps_to_mpos(print_position, sys.position);

    if(hal.probe.get_state)
        probe_state = hal.probe.get_state();

    // Report current machine state and sub-states
    hal.stream.write_all("<");

    sys_state_t state = state_get();

    switch (gc_state.tool_change && state == STATE_CYCLE ? STATE_TOOL_CHANGE : state) {

        case STATE_IDLE:
            hal.stream.write_all("Idle");
            break;

        case STATE_CYCLE:
            hal.stream.write_all("Run");
            if(sys.probing_state == Probing_Active && settings.status_report.run_substate)
                probing = true;
            else if (probing)
                probing = probe_state.triggered;
            if(sys.flags.feed_hold_pending)
                hal.stream.write_all(":1");
            else if(probing)
                hal.stream.write_all(":2");
            break;

        case STATE_HOLD:
            hal.stream.write_all(appendbuf(2, "Hold:", uitoa((uint32_t)(sys.holding_state - 1))));
            break;

        case STATE_JOG:
            hal.stream.write_all("Jog");
            break;

        case STATE_HOMING:
            hal.stream.write_all("Home");
            break;

        case STATE_ESTOP:
        case STATE_ALARM:
            if((report.all || settings.status_report.alarm_substate) && sys.alarm)
                hal.stream.write_all(appendbuf(2, "Alarm:", uitoa((uint32_t)sys.alarm)));
            else
                hal.stream.write_all("Alarm");
            break;

        case STATE_CHECK_MODE:
            hal.stream.write_all("Check");
            break;

        case STATE_SAFETY_DOOR:
            hal.stream.write_all(appendbuf(2, "Door:", uitoa((uint32_t)sys.parking_state)));
            break;

        case STATE_SLEEP:
            hal.stream.write_all("Sleep");
            break;

        case STATE_TOOL_CHANGE:
            hal.stream.write_all("Tool");
            break;
    }

    uint_fast8_t idx;
    float wco[N_AXIS];
    if (!settings.status_report.machine_position || report.wco) {
        for (idx = 0; idx < N_AXIS; idx++) {
            // Apply work coordinate offsets and tool length offset to current position.
            wco[idx] = gc_get_offset(idx);
            if (!settings.status_report.machine_position)
                print_position[idx] -= wco[idx];
        }
    }

    // Report position
    hal.stream.write_all(settings.status_report.machine_position ? "|MPos:" : "|WPos:");
    hal.stream.write_all(get_axis_values(print_position));

    // Returns planner and output stream buffer states.

    if (settings.status_report.buffer_state) {
        hal.stream.write_all("|Bf:");
        hal.stream.write_all(uitoa((uint32_t)plan_get_block_buffer_available()));
        hal.stream.write_all(",");
        hal.stream.write_all(uitoa(hal.stream.get_rx_buffer_free()));
    }

    if(settings.status_report.line_numbers) {
        // Report current line number
        plan_block_t *cur_block = plan_get_current_block();
        if (cur_block != NULL && cur_block->line_number > 0)
            hal.stream.write_all(appendbuf(2, "|Ln:", uitoa((uint32_t)cur_block->line_number)));
    }

    spindle_ptrs_t *spindle_0;
    spindle_state_t spindle_0_state;

    spindle_0 = spindle_get(0);
    spindle_0_state = spindle_0->get_state();

    // Report realtime feed speed
    if(settings.status_report.feed_speed) {
        if(spindle_0->cap.variable) {
            hal.stream.write_all(appendbuf(2, "|FS:", get_rate_value(st_get_realtime_rate())));
            hal.stream.write_all(appendbuf(2, ",", uitoa(spindle_0_state.on ? lroundf(spindle_0->param->rpm_overridden) : 0)));
            if(spindle_0->get_data /* && sys.mpg_mode */)
                hal.stream.write_all(appendbuf(2, ",", uitoa(lroundf(spindle_0->get_data(SpindleData_RPM)->rpm))));
        } else
            hal.stream.write_all(appendbuf(2, "|F:", get_rate_value(st_get_realtime_rate())));
    }

#if N_SYS_SPINDLE > 1

    for(idx = 1; idx < N_SYS_SPINDLE; idx++) {

        spindle_ptrs_t *spindle_n;
        spindle_state_t spindle_n_state;

        if((spindle_n = spindle_get(idx))) {
            spindle_n_state = spindle_n->get_state();
            hal.stream.write_all(appendbuf(3, "|SP", uitoa(idx), ":"));
            hal.stream.write_all(appendbuf(3, uitoa(spindle_n_state.on ? lroundf(spindle_n->param->rpm_overridden) : 0), ",,", spindle_n_state.on ? (spindle_n_state.ccw ? "C" : "S") : ""));
            if(settings.status_report.overrides)
                hal.stream.write_all(appendbuf(2, ",", uitoa(spindle_n->param->override_pct)));
        }
    }

#endif

    if(settings.status_report.pin_state) {

        axes_signals_t lim_pin_state = limit_signals_merge(hal.limits.get_state());
        control_signals_t ctrl_pin_state = hal.control.get_state();

        if (lim_pin_state.value | ctrl_pin_state.value | probe_state.triggered | !probe_state.connected | sys.flags.block_delete_enabled) {

            char *append = &buf[4];

            strcpy(buf, "|Pn:");

            if (probe_state.triggered)
                *append++ = 'P';

            if(!probe_state.connected)
                *append++ = 'O';

            if (lim_pin_state.value && !hal.control.get_state().limits_override)
                append = axis_signals_tostring(append, lim_pin_state);

            if (ctrl_pin_state.value)
                append = control_signals_tostring(append, ctrl_pin_state);

            *append = '\0';
            hal.stream.write_all(buf);
        }
    }

    if(settings.status_report.work_coord_offset) {

        if(wco_counter > 0 && !report.wco) {
            if(wco_counter > (REPORT_WCO_REFRESH_IDLE_COUNT - 1) && state_get() == STATE_IDLE)
                wco_counter = REPORT_WCO_REFRESH_IDLE_COUNT - 1;
            wco_counter--;
        } else
            wco_counter = state_get() & (STATE_HOMING|STATE_CYCLE|STATE_HOLD|STATE_JOG|STATE_SAFETY_DOOR)
                            ? (REPORT_WCO_REFRESH_BUSY_COUNT - 1) // Reset counter for slow refresh
                            : (REPORT_WCO_REFRESH_IDLE_COUNT - 1);
    } else
        report.wco = Off;

    if(settings.status_report.overrides) {

        if (override_counter > 0 && !report.overrides)
            override_counter--;
        else if((report.overrides = !report.wco)) {
            report.spindle = report.spindle || spindle_0_state.on;
            report.coolant = report.coolant || hal.coolant.get_state().value != 0;
            override_counter = state_get() & (STATE_HOMING|STATE_CYCLE|STATE_HOLD|STATE_JOG|STATE_SAFETY_DOOR)
                                 ? (REPORT_OVERRIDE_REFRESH_BUSY_COUNT - 1) // Reset counter for slow refresh
                                 : (REPORT_OVERRIDE_REFRESH_IDLE_COUNT - 1);
        }
    } else
        report.overrides = Off;

    if(report.value || gc_state.tool_change) {

        if(report.wco) {
            hal.stream.write_all("|WCO:");
            hal.stream.write_all(get_axis_values(wco));
        }

        if(report.gwco) {
            hal.stream.write_all("|WCS:G");
            hal.stream.write_all(map_coord_system(gc_state.modal.coord_system.id));
        }

        if(report.overrides) {
            hal.stream.write_all(appendbuf(2, "|Ov:", uitoa((uint32_t)sys.override.feed_rate)));
            hal.stream.write_all(appendbuf(2, ",", uitoa((uint32_t)sys.override.rapid_rate)));
            hal.stream.write_all(appendbuf(2, ",", uitoa((uint32_t)spindle_0->param->override_pct)));
        }

        if(report.spindle || report.coolant || report.tool || gc_state.tool_change) {

            coolant_state_t cl_state = hal.coolant.get_state();

            char *append = &buf[3];

            strcpy(buf, "|A:");

            if (spindle_0_state.on)

                *append++ = spindle_0_state.ccw ? 'C' : 'S';

#if COMPATIBILITY_LEVEL == 0
            if(spindle_0_state.encoder_error && hal.driver_cap.spindle_sync)
                *append++ = 'E';
#endif

            if (cl_state.flood)
                *append++ = 'F';

            if (cl_state.mist)
                *append++ = 'M';

            if(gc_state.tool_change && !report.tool)
                *append++ = 'T';

            *append = '\0';
            hal.stream.write_all(buf);
        }

        if(report.scaling) {
            axis_signals_tostring(buf, gc_get_g51_state());
            hal.stream.write_all("|Sc:");
            hal.stream.write_all(buf);
        }

        if(report.mpg_mode && hal.driver_cap.mpg_mode)
            hal.stream.write_all(sys.mpg_mode ? "|MPG:1" : "|MPG:0");

        if(report.homed && (sys.homing.mask || settings.homing.flags.single_axis_commands || settings.homing.flags.manual)) {
            axes_signals_t homing = {sys.homing.mask ? sys.homing.mask : AXES_BITMASK};
            hal.stream.write_all(appendbuf(2, "|H:", (homing.mask & sys.homed.mask) == homing.mask ? "1" : "0"));
            if(settings.homing.flags.single_axis_commands)
                hal.stream.write_all(appendbuf(2, ",", uitoa(sys.homed.mask)));
        }

        if(report.xmode && settings.mode == Mode_Lathe)
            hal.stream.write_all(gc_state.modal.diameter_mode ? "|D:1" : "|D:0");

        if(report.tool)
            hal.stream.write_all(appendbuf(2, "|T:", uitoa(gc_state.tool->tool)));

        if(report.tlo_reference)
            hal.stream.write_all(appendbuf(2, "|TLR:", uitoa(sys.tlo_reference_set.mask != 0)));

        if(report.m66result && sys.var5399 > -2) { // M66 result
            if(sys.var5399 >= 0)
                hal.stream.write_all(appendbuf(2, "|In:", uitoa(sys.var5399)));
            else
                hal.stream.write_all("|In:-1");
        }
    }

    if(grbl.on_realtime_report)
        grbl.on_realtime_report(hal.stream.write_all, sys.report);

#if COMPATIBILITY_LEVEL <= 1
    if(report.all) {
        hal.stream.write_all("|FW:grblHAL");
        if(settings.report_interval) {
            hal.stream.write_all(sys.flags.auto_reporting ? "|AR:" : "|AR");
            if(sys.flags.auto_reporting)
                hal.stream.write_all(uitoa(settings.report_interval));
        }
        if(sys.blocking_event)
            hal.stream.write_all("|$C:1");
    } else
#endif

    if(settings.status_report.parser_state) {

        static uint32_t tool;
        static float feed_rate, spindle_rpm;
        static gc_modal_t last_state;
        static bool g92_active;

        bool is_changed = feed_rate != gc_state.feed_rate || spindle_rpm != gc_state.spindle.rpm || tool != gc_state.tool->tool;

        if(is_changed) {
            feed_rate = gc_state.feed_rate;
            tool = gc_state.tool->tool;
            spindle_rpm = gc_state.spindle.rpm;
        } else if ((is_changed = g92_active != is_g92_active()))
            g92_active = !g92_active;
        else if(memcmp(&last_state, &gc_state.modal, sizeof(gc_modal_t))) {
            last_state = gc_state.modal;
            is_changed = true;
        }

        if (is_changed)
            system_set_exec_state_flag(EXEC_GCODE_REPORT);

        if(report.tool_offset)
            system_set_exec_state_flag(EXEC_TLO_REPORT);
    }

    hal.stream.write_all(">" ASCII_EOL);

    system_add_rt_report(Report_ClearAll);
    if(settings.status_report.work_coord_offset && wco_counter == 0)
        system_add_rt_report(Report_WCO); // Set to report on next request
}

static void report_bitfield (const char *format, bool bitmap)
{
    char *s;
    uint_fast8_t bit = 0;
    uint_fast16_t val = 1;

    // Copy string from Flash to RAM, strtok cannot be used unless doing so.
    if((s = (char *)malloc(strlen(format) + 1))) {

        strcpy(s, format);
        char *element = strtok(s, ",");

        while(element) {
            if(strcmp(element, "N/A")) {
                hal.stream.write(ASCII_EOL);
                hal.stream.write("    ");
                hal.stream.write(uitoa(bit));
                hal.stream.write(" - ");
                if(*element)
                hal.stream.write(element);
                if(bitmap) {
                    hal.stream.write(" (");
                    hal.stream.write(uitoa(val));
                    hal.stream.write(")");
                }
            }
            bit++;
            val <<= 1;
            element = strtok(NULL, ",");
        }

        free(s);
    }
}

static void write_quoted (const char *s, const char *sep)
{
    hal.stream.write("\"");
    hal.stream.write(s); // TODO: escape double quoutes
    hal.stream.write("\"");
    if(sep)
        hal.stream.write(sep);
}

static void write_name (const char *s, uint_fast8_t offset)
{
    char *q = hal.stream.write_n ? strchr(s, '?') : NULL;

    if(q) {
        if(q != s)
            hal.stream.write_n(s, q - s);
        hal.stream.write(uitoa(offset + 1));
        hal.stream.write(q + 1);
    } else
        hal.stream.write(s);
}

static void report_settings_detail (settings_format_t format, const setting_detail_t *setting, uint_fast8_t offset)
{
    uint_fast8_t suboffset = setting->flags.subgroups ? offset / setting->flags.increment : offset;

    switch(format)
    {
        case SettingsFormat_HumanReadable:
            hal.stream.write(ASCII_EOL "$");
            hal.stream.write(uitoa(setting->id + offset));
            hal.stream.write(": ");
            if(setting->group == Group_Axis0)
                hal.stream.write(axis_letter[offset]);
            write_name(setting->name, suboffset);

            switch(setting_datatype_to_external(setting->datatype)) {

                case Format_AxisMask:
                    hal.stream.write(" as axismask");
                    break;

                case Format_Bool:
                    hal.stream.write(" as boolean");
                    break;

                case Format_Bitfield:
                    hal.stream.write(" as bitfield:");
                    report_bitfield(setting->format, true);
                    break;

                case Format_XBitfield:
                    hal.stream.write(" as bitfield where setting bit 0 enables the rest:");
                    report_bitfield(setting->format, true);
                    break;

                case Format_RadioButtons:
                    hal.stream.write(":");
                    report_bitfield(setting->format, false);
                    break;

                case Format_IPv4:
                    hal.stream.write(" as IP address");
                    break;

                default:
                    if(setting->unit) {
                        hal.stream.write(" in ");
                        hal.stream.write(setting->unit);
                    }
                    break;
            }

            if(setting->min_value && setting->max_value) {
                hal.stream.write(", range: ");
                hal.stream.write(setting->min_value);
                hal.stream.write(" - ");
                hal.stream.write(setting->max_value);
            } else if(!setting_is_list(setting)) {
                if(setting->min_value) {
                    hal.stream.write(", min: ");
                    hal.stream.write(setting->min_value);
                }
                if(setting->max_value) {
                    hal.stream.write(", max: ");
                    hal.stream.write(setting->max_value);
                }
            }

            if(setting->flags.reboot_required)
                hal.stream.write(", reboot required");

#ifndef NO_SETTINGS_DESCRIPTIONS
            // Add description if driver is capable of outputting it...
            if(hal.stream.write_n) {
                const char *description = setting_get_description((setting_id_t)(setting->id + offset));
                if(description && *description != '\0') {
                    char *lf;
                    hal.stream.write(ASCII_EOL);
                    if((lf = strstr(description, "\\n"))) while(lf) {
                        hal.stream.write(ASCII_EOL);
                        hal.stream.write_n(description, lf - description);
                        description = lf + 2;
                        lf = strstr(description, "\\n");
                    }
                    if(*description != '\0') {
                        hal.stream.write(ASCII_EOL);
                        hal.stream.write(description);
                    }
                }
                if(setting->flags.reboot_required) {
                    if(description && *description != '\0')
                        hal.stream.write(ASCII_EOL ASCII_EOL);
                    hal.stream.write(SETTINGS_HARD_RESET_REQUIRED + 4);
                }
            }
#endif
            break;

        case SettingsFormat_MachineReadable:
            hal.stream.write("[SETTING:");
            hal.stream.write(uitoa(setting->id + offset));
            hal.stream.write(vbar);
            hal.stream.write(uitoa(setting->group + (setting->flags.subgroups ? suboffset : 0)));
            hal.stream.write(vbar);
            if(setting->group == Group_Axis0)
                hal.stream.write(axis_letter[offset]);
            write_name(setting->name, suboffset);
            hal.stream.write(vbar);
            if(setting->unit)
                hal.stream.write(setting->unit);
            hal.stream.write(vbar);
            hal.stream.write(uitoa(setting_datatype_to_external(setting->datatype)));
            hal.stream.write(vbar);
            if(setting->format)
                hal.stream.write(setting->format);
            hal.stream.write(vbar);
            if(setting->min_value && !setting_is_list(setting))
                hal.stream.write(setting->min_value);
            hal.stream.write(vbar);
            if(setting->max_value)
                hal.stream.write(setting->max_value);
            hal.stream.write(vbar);
            hal.stream.write(uitoa(setting->flags.reboot_required));
            hal.stream.write(vbar);
            hal.stream.write(uitoa(setting->flags.allow_null));
            hal.stream.write("]");
            break;

        case SettingsFormat_Grbl:
            {
                write_quoted(uitoa(setting->id + offset), ",");
                hal.stream.write("\"");
                if(setting->group == Group_Axis0)
                    hal.stream.write(axis_letter[offset]);
                write_name(setting->name, suboffset);
                hal.stream.write("\",");
                if(setting->unit) {
                    write_quoted(setting->unit, ",");
                } else // TODO: output sensible unit from datatype
                    write_quoted("", ",");

    #ifndef NO_SETTINGS_DESCRIPTIONS
                const char *description = setting_get_description((setting_id_t)(setting->id + offset));
                write_quoted(description ? description : "", ",");
    #else
                write_quoted("", NULL);
    #endif
            }
            break;

        case SettingsFormat_grblHAL:
            {
                hal.stream.write(uitoa(setting->id + offset));

                hal.stream.write("\t");

                if(setting->group == Group_Axis0)
                    hal.stream.write(axis_letter[offset]);
                write_name(setting->name, suboffset);

                hal.stream.write("\t");

                if(setting->unit)
                    hal.stream.write(setting->unit);
                else if(setting->datatype == Format_AxisMask || setting->datatype == Format_Bitfield || setting->datatype == Format_XBitfield)
                    hal.stream.write("mask");
                else if(setting->datatype == Format_Bool)
                    hal.stream.write("boolean");
                else if(setting->datatype == Format_RadioButtons)
                    hal.stream.write("integer");

                hal.stream.write("\t");
    /*
                Format_Bool = 0,
                Format_Bitfield,
                Format_XBitfield,
                Format_RadioButtons,
                Format_AxisMask,
                Format_Integer, // 32 bit
                ,
                Format_String,
                Format_Password,
                Format_IPv4,
                // For internal use only
                Format_Int8,
                Format_Int16,
    */
                switch(setting_datatype_to_external(setting->datatype)) {

                    case Format_Integer:
                        hal.stream.write("integer");
                        break;

                    case Format_Decimal:
                        hal.stream.write("float");
                        break;

                    case Format_Bool:
                        hal.stream.write("bool");
                        break;

                    case Format_AxisMask:
                    case Format_Bitfield:
                        hal.stream.write("bitfield");
                        break;

                    case Format_XBitfield:
                        hal.stream.write("xbitfield");
                        break;

                    case Format_RadioButtons:
                        hal.stream.write("radiobuttons");
                        break;

                    case Format_IPv4:
                        hal.stream.write("ipv4");
                        break;

                    case Format_String:
                        hal.stream.write("string");
                        break;

                    case Format_Password:
                        hal.stream.write("password");
                        break;

                    default:
                        break;
                }

                hal.stream.write("\t");

                if(setting->format)
                    hal.stream.write(setting->format);
                else if (setting->datatype == Format_AxisMask)
                    hal.stream.write("axes");

                hal.stream.write("\t");

    #ifndef NO_SETTINGS_DESCRIPTIONS
                const char *description = setting_get_description((setting_id_t)(setting->id + offset));
                hal.stream.write(description ? description : "");
                if(setting->flags.reboot_required)
                    hal.stream.write(SETTINGS_HARD_RESET_REQUIRED + (description && *description != '\0' ? 0 : 4));
    #endif
                hal.stream.write("\t");

                if(setting->min_value)
                    hal.stream.write(setting->min_value);

                hal.stream.write("\t");

                if(setting->max_value)
                    hal.stream.write(setting->max_value);

                hal.stream.write("\t");
                hal.stream.write(uitoa(setting->flags.reboot_required));
                hal.stream.write("\t");
                hal.stream.write(uitoa(setting->flags.allow_null));
            }
            break;
    }

    hal.stream.write(ASCII_EOL);
}

typedef struct {
    settings_format_t format;
    setting_group_t group;
    uint_fast16_t offset;
} report_args_t;

static bool print_sorted (const setting_detail_t *setting, uint_fast16_t offset, void *args)
{
    if(!(((report_args_t *)args)->group == setting->group && ((report_args_t *)args)->offset != offset))
        report_settings_detail (((report_args_t *)args)->format, setting, offset);

    return true;
}

static bool print_unsorted (const setting_detail_t *setting, uint_fast16_t offset, void *args)
{
    if(!(((report_args_t *)args)->group == setting->group && ((report_args_t *)args)->offset != offset) &&
       (setting->is_available == NULL ||setting->is_available(setting)))
        report_settings_detail(((report_args_t *)args)->format, setting, offset);

    return true;
}

static status_code_t print_settings_details (settings_format_t format, setting_group_t group)
{
    uint_fast16_t idx, n_settings = 0;
    bool reported = group == Group_All;
    const setting_detail_t *setting;
    report_args_t args;
    setting_detail_t **all_settings, **psetting;
    setting_details_t *details = settings_get_details();

    args.group = settings_normalize_group(group);
    args.offset = group - args.group;
    args.format = format;

    do {
        n_settings += details->n_settings;
    } while((details = details->next));

    if(format == SettingsFormat_Grbl)
        hal.stream.write("\"$-Code\",\" Setting\",\" Units\",\" Setting Description\"" ASCII_EOL);
    else if(format == SettingsFormat_grblHAL)
        hal.stream.write("$-Code\tSetting\tUnits\tDatatype\tData format\tSetting Description\tMin\tMax" ASCII_EOL);

    details = settings_get_details();

    if((all_settings = psetting = calloc(n_settings, sizeof(setting_detail_t *)))) {

        n_settings = 0;

        do {
            for(idx = 0; idx < details->n_settings; idx++) {
                setting = &details->settings[idx];
                if((group == Group_All || setting->group == args.group) && (setting->is_available == NULL || setting->is_available(setting))) {
                    *psetting++ = (setting_detail_t *)setting;
                    n_settings++;
                }
            }
        } while((details = details->next));

        qsort(all_settings, n_settings, sizeof(setting_detail_t *), cmp_settings);

        for(idx = 0; idx < n_settings; idx++) {
            if(settings_iterator(all_settings[idx], print_sorted, &args))
                reported = true;
        }

        free(all_settings);

    } else do {
        for(idx = 0; idx < details->n_settings; idx++) {

            setting = &details->settings[idx];

            if(group == Group_All || setting->group == args.group) {
                if(settings_iterator(setting, print_unsorted, &args))
                    reported = true;
            }
        }
    } while((details = details->next));

    return reported ? Status_OK : Status_SettingDisabled;
}

status_code_t report_settings_details (settings_format_t format, setting_id_t id, setting_group_t group)
{
    if(id != Setting_SettingsAll) {
        status_code_t status = Status_OK;

        const setting_detail_t *setting = setting_get_details(id, NULL);

        if(setting)
            report_settings_detail(format, setting, id - setting->id);
        else
            status = Status_SettingDisabled;

        return status;
    }

    return print_settings_details(format, group);
}

#ifndef NO_SETTINGS_DESCRIPTIONS

status_code_t report_setting_description (settings_format_t format, setting_id_t id)
{
    const setting_detail_t *setting = setting_get_details(id, NULL);
    const char *description = setting_get_description(id);

    if(format == SettingsFormat_MachineReadable) {
        hal.stream.write("[SETTINGDESCR:");
        hal.stream.write(uitoa(id));
        hal.stream.write(vbar);
    }
//    hal.stream.write(description == NULL ? (is_setting_available(setting_get_details(id, NULL)) ? "" : "N/A") : description); // TODO?
    hal.stream.write(description ? description : (setting ? "" : "N/A"));
    if(setting && setting->flags.reboot_required)
        hal.stream.write(SETTINGS_HARD_RESET_REQUIRED + (description && *description != '\0' ? 0 : 4));

    if(format == SettingsFormat_MachineReadable)
        hal.stream.write("]" ASCII_EOL);

    return Status_OK;
}

#endif

static int cmp_alarms (const void *a, const void *b)
{
  return (*(alarm_detail_t **)(a))->id - (*(alarm_detail_t **)(b))->id;
}

static void print_alarm (const alarm_detail_t *alarm, bool grbl_format)
{
    if(grbl_format) {
        write_quoted(uitoa(alarm->id), ",");
        write_quoted("N/A", ",");
        write_quoted(alarm->description ? alarm->description : "", NULL);
        hal.stream.write(ASCII_EOL);
    } else {
        hal.stream.write("[ALARMCODE:");
        hal.stream.write(uitoa(alarm->id));
        hal.stream.write(vbar);
        hal.stream.write(vbar);
        if(alarm->description)
            hal.stream.write(alarm->description);
        hal.stream.write("]" ASCII_EOL);
    }
}

status_code_t report_alarm_details (bool grbl_format)
{
    uint_fast16_t idx, n_alarms = 0;
    alarm_details_t *details = grbl.on_get_alarms();
    alarm_detail_t **all_alarms, **palarm;

    if(grbl_format)
        hal.stream.write("\"Alarm Code in v1.1+\",\" Alarm Message in v1.0-\",\" Alarm Description\"" ASCII_EOL);

    do {
        n_alarms += details->n_alarms;
    } while((details = details->next));

    details = grbl.on_get_alarms();

    if((all_alarms = palarm = calloc(n_alarms, sizeof(alarm_detail_t *)))) {

        do {
            for(idx = 0; idx < details->n_alarms; idx++)
                *palarm++ = (alarm_detail_t *)&(details->alarms[idx]);
        } while((details = details->next));

        qsort(all_alarms, n_alarms, sizeof(alarm_detail_t *), cmp_alarms);

        for(idx = 0; idx < n_alarms; idx++)
            print_alarm(all_alarms[idx], grbl_format);

        free(all_alarms);

    } else do {
        for(idx = 0; idx < details->n_alarms; idx++)
            print_alarm(&details->alarms[idx], grbl_format);
    } while((details = details->next));

    return Status_OK;
}

static int cmp_errors (const void *a, const void *b)
{
  return (*(status_detail_t **)(a))->id - (*(status_detail_t **)(b))->id;
}

static void print_error (const status_detail_t *error, bool grbl_format)
{
    if(grbl_format) {
        write_quoted(uitoa(error->id), ",");
        write_quoted("N/A", ",");
        write_quoted(error->description ? error->description : "", NULL);
        hal.stream.write(ASCII_EOL);
    } else {
        hal.stream.write("[ERRORCODE:");
        hal.stream.write(uitoa(error->id));
        hal.stream.write(vbar);
        hal.stream.write(vbar);
        if(error->description)
            hal.stream.write(error->description);
        hal.stream.write("]" ASCII_EOL);
    }
}

status_code_t report_error_details (bool grbl_format)
{
    uint_fast16_t idx, n_errors = 0;
    error_details_t *details = grbl.on_get_errors();
    status_detail_t **all_errors, **perror;

    if(grbl_format)
        hal.stream.write("\"Error Code in v1.1+\",\"Error Message in v1.0-\",\"Error Description\"" ASCII_EOL);

    do {
        n_errors += details->n_errors;
    } while((details = details->next));

    details = grbl.on_get_errors();

    if((all_errors = perror = calloc(n_errors, sizeof(status_detail_t *)))) {

        do {
            for(idx = 0; idx < details->n_errors; idx++)
                *perror++ = (status_detail_t *)&(details->errors[idx]);
        } while((details = details->next));

        qsort(all_errors, n_errors, sizeof(status_detail_t *), cmp_errors);

        for(idx = 0; idx < n_errors; idx++)
            print_error(all_errors[idx], grbl_format);

        free(all_errors);

    } else do {
        for(idx = 0; idx < details->n_errors; idx++)
            print_error(&details->errors[idx], grbl_format);
    } while((details = details->next));

    return Status_OK;
}

static void print_setting_group (const setting_group_detail_t *group, char *prefix)
{
    if(settings_is_group_available(group->id)) {
        if(!prefix) {
            hal.stream.write("[SETTINGGROUP:");
            hal.stream.write(uitoa(group->id));
            hal.stream.write(vbar);
            hal.stream.write(uitoa(group->parent));
            hal.stream.write(vbar);
            hal.stream.write(group->name);
            hal.stream.write("]" ASCII_EOL);
        } else if(group->id != Group_Root) {
            hal.stream.write(prefix);
            hal.stream.write(group->name);
            hal.stream.write(ASCII_EOL);
        }
    }
}

static int cmp_setting_group_id (const void *a, const void *b)
{
    return (*(setting_group_detail_t **)(a))->id - (*(setting_group_detail_t **)(b))->id;
}

static int cmp_setting_group_name (const void *a, const void *b)
{
    return strcmp((*(setting_group_detail_t **)(a))->name, (*(setting_group_detail_t **)(b))->name);
}

static bool group_is_dup (setting_group_detail_t **groups, setting_group_t group)
{
    while(*groups) {
        if((*groups)->id == group)
            return true;
        groups++;
    }

    return false;
}

status_code_t report_setting_group_details (bool by_id, char *prefix)
{
    uint_fast16_t idx, n_groups = 0;
    setting_details_t *details = settings_get_details();
    setting_group_detail_t **all_groups, **group;

    do {
        n_groups += details->n_groups;
    } while((details = details->next));

    details = settings_get_details();

    if((all_groups = group = calloc(n_groups, sizeof(setting_group_detail_t *)))) {

        uint_fast16_t idx;

        do {
            for(idx = 0; idx < details->n_groups; idx++) {
                if(!group_is_dup(all_groups, details->groups[idx].id))
                    *group++ = (setting_group_detail_t *)&details->groups[idx];
            }
        } while((details = details->next));

        qsort(all_groups, n_groups, sizeof(setting_group_detail_t *), by_id ? cmp_setting_group_id : cmp_setting_group_name);

        for(idx = 0; idx < n_groups; idx++)
            print_setting_group(all_groups[idx], prefix);

        free(all_groups);

    } else do {
        for(idx = 0; idx < details->n_groups; idx++)
            print_setting_group(&details->groups[idx], prefix);
    } while((details = details->next));

    return Status_OK;
}

static char *add_limits (char *buf, limit_signals_t limits)
{
    buf = axis_signals_tostring(buf, limits.min);
    *buf++ = ',';
    buf = axis_signals_tostring(buf, limits.max);
    *buf++ = ',';
    buf = axis_signals_tostring(buf, limits.min2);
    *buf++ = ',';
    buf = axis_signals_tostring(buf, limits.max2);

    return buf;
}

status_code_t report_last_signals_event (sys_state_t state, char *args)
{
    char *append = &buf[12];

    strcpy(buf, "[LASTEVENTS:");

    append = control_signals_tostring(append, sys.last_event.control);
    *append++ = ',';
    append = add_limits(append, sys.last_event.limits);

    hal.stream.write(buf);
    hal.stream.write("]" ASCII_EOL);

    return Status_OK;
}

status_code_t report_current_limit_state (sys_state_t state, char *args)
{
    char *append = &buf[8];

    strcpy(buf, "[LIMITS:");

    append = add_limits(append, hal.limits.get_state());

    hal.stream.write(buf);
    hal.stream.write("]" ASCII_EOL);

    return Status_OK;
}

// Prints spindle data (encoder pulse and index count, angular position).
status_code_t report_spindle_data (sys_state_t state, char *args)
{
    spindle_ptrs_t *spindle = gc_spindle_get();

    if(spindle->get_data) {

        float apos = spindle->get_data(SpindleData_AngularPosition)->angular_position;
        spindle_data_t *data = spindle->get_data(SpindleData_Counters);

        hal.stream.write("[SPINDLE:");
        hal.stream.write(uitoa(data->index_count));
        hal.stream.write(",");
        hal.stream.write(uitoa(data->pulse_count));
        hal.stream.write(",");
        hal.stream.write(uitoa(data->error_count));
        hal.stream.write(",");
        hal.stream.write(ftoa(apos, 3));
        hal.stream.write("]" ASCII_EOL);
    }

    return spindle->get_data ? Status_OK : Status_InvalidStatement;
}

static const char *get_pinname (pin_function_t function)
{
    const char *name = NULL;
    uint_fast8_t idx = sizeof(pin_names) / sizeof(pin_name_t);

    do {
        if(pin_names[--idx].function == function)
            name = pin_names[idx].name;
    } while(idx && !name);

    return name ? name : "N/A";
}

static void report_pin (xbar_t *pin, void *data)
{
    hal.stream.write("[PIN:");
    if(pin->port)
        hal.stream.write((char *)pin->port);
    hal.stream.write(uitoa(pin->pin));
    hal.stream.write(",");
    hal.stream.write(get_pinname(pin->function));
    if(pin->description) {
        hal.stream.write(",");
        hal.stream.write(pin->description);
    }
    hal.stream.write("]" ASCII_EOL);
}

status_code_t report_pins (sys_state_t state, char *args)
{
    if(hal.enumerate_pins)
        hal.enumerate_pins(false, report_pin, NULL);

    return Status_OK;
}

static void print_uito2a (char *prefix, uint32_t v)
{
    hal.stream.write(prefix);
    if(v < 10)
        hal.stream.write("0");
    hal.stream.write(uitoa(v));
}

status_code_t report_time (void)
{
    bool ok = false;

    if(hal.rtc.get_datetime) {
        struct tm time;
        if((ok = !!hal.rtc.get_datetime(&time))) {
            hal.stream.write("[RTC:");
            hal.stream.write(uitoa(time.tm_year + 1900));
            print_uito2a("-", time.tm_mon + 1);
            print_uito2a("-", time.tm_mday);
            print_uito2a("T", time.tm_hour);
            print_uito2a(":", time.tm_min);
            print_uito2a(":", time.tm_sec);
            hal.stream.write("]" ASCII_EOL);
        }
    }

    return ok ? Status_OK : Status_InvalidStatement;
}

static void report_spindle (spindle_info_t *spindle)
{
    hal.stream.write(uitoa(spindle->id));
    hal.stream.write(" - ");
    hal.stream.write(spindle->name);
    if(spindle->enabled) {
#if N_SPINDLE > 1
        hal.stream.write(", enabled as spindle ");
        hal.stream.write(uitoa(spindle->num));
#else
        hal.stream.write(", active");
#endif
    }
    hal.stream.write(ASCII_EOL);
}

status_code_t report_spindles (void)
{
    if(!spindle_enumerate_spindles(report_spindle))
        hal.stream.write("No spindles registered." ASCII_EOL);

    return Status_OK;
}

void report_pid_log (void)
{
#ifdef PID_LOG
    uint_fast16_t idx = 0;

    hal.stream.write("[PID:");
    hal.stream.write(ftoa(sys.pid_log.setpoint, N_DECIMAL_PIDVALUE));
    hal.stream.write(",");
    hal.stream.write(ftoa(sys.pid_log.t_sample, N_DECIMAL_PIDVALUE));
    hal.stream.write(",2|"); // 2 is number of values per sample!

    if(sys.pid_log.idx) do {
        hal.stream.write(ftoa(sys.pid_log.target[idx], N_DECIMAL_PIDVALUE));
        hal.stream.write(",");
        hal.stream.write(ftoa(sys.pid_log.actual[idx], N_DECIMAL_PIDVALUE));
        idx++;
        if(idx != sys.pid_log.idx)
            hal.stream.write(",");
    } while(idx != sys.pid_log.idx);

    hal.stream.write("]" ASCII_EOL);
    grbl.report.status_message(Status_OK);
#else
    grbl.report.status_message(Status_GcodeUnsupportedCommand);
#endif
}

static const report_t report_fns = {
    .init_message = report_init_message,
    .help_message = report_help_message,
    .status_message = report_status_message,
    .feedback_message = report_feedback_message,
    .alarm_message = report_alarm_message,
    .setting = report_setting
};

void report_init_fns (void)
{
    memcpy(&grbl.report, &report_fns, sizeof(report_t));

    if(grbl.on_report_handlers_init)
        grbl.on_report_handlers_init();
}
