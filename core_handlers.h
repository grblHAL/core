/*
  core_handlers.h - core event handler entry points and some reporting entry points

  Part of grblHAL

  Copyright (c) 2020-2023 Terje Io

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

/*! \file
    \brief core function pointers and data structures definitions.
*/

#pragma once

#include "system.h"
#include "stream.h"
#include "alarms.h"
#include "errors.h"
#include "settings.h"
#include "report.h"
#include "planner.h"
#include "machine_limits.h"

typedef enum {
    OverrideChanged_FeedRate = 0,
    OverrideChanged_RapidRate = 0,
    OverrideChanged_SpindleRPM = 0,
    OverrideChanged_SpindleState = 0,
    OverrideChanged_CoolantState = 0,
    OverrideChanged_FanState = 0
} override_changed_t;

/* TODO: add to grbl pointers so that a different formatting (xml, json etc) of reports may be implemented by driver?
typedef struct {
    status_code_t (*report_status_message)(status_code_t status_code);
    alarm_code_t (*report_alarm_message)(alarm_code_t alarm_code);
    message_code_t (*report_feedback_message)(message_code_t message_code);
    void (*report_init_message)(void);
    void (*report_grbl_help)(void);
    void (*report_echo_line_received)(char *line);
    void (*report_realtime_status)(void);
    void (*report_probe_parameters)(void);
    void (*report_ngc_parameters)(void);
    void (*report_gcode_modes)(void);
    void (*report_startup_line)(uint8_t n, char *line);
    void (*report_execute_startup_message)(char *line, status_code_t status_code);
} grbl_report_t;
*/

// Report entry points set by core at reset.

typedef status_code_t (*status_message_ptr)(status_code_t status_code);
typedef message_code_t (*feedback_message_ptr)(message_code_t message_code);

typedef struct {
    setting_output_ptr setting;
    status_message_ptr status_message;
    feedback_message_ptr feedback_message;
} report_t;

// Core event handler and other entry points.
// Most of the event handlers defaults to NULL, a few is set up to call a dummy handler for simpler code.

typedef bool (*enqueue_gcode_ptr)(char *data);
typedef bool (*protocol_enqueue_realtime_command_ptr)(char c);

typedef void (*on_state_change_ptr)(sys_state_t state);
typedef void (*on_override_changed_ptr)(override_changed_t override);
typedef void (*on_spindle_programmed_ptr)(spindle_state_t spindle, float rpm, spindle_rpm_mode_t mode);
typedef void (*on_program_completed_ptr)(program_flow_t program_flow, bool check_mode);
typedef void (*on_execute_realtime_ptr)(sys_state_t state);
typedef void (*on_unknown_accessory_override_ptr)(uint8_t cmd);
typedef bool (*on_unknown_realtime_cmd_ptr)(char c);
typedef void (*on_report_handlers_init_ptr)(void);
typedef void (*on_report_options_ptr)(bool newopt);
typedef void (*on_report_command_help_ptr)(void);
typedef void (*on_global_settings_restore_ptr)(void);
typedef void (*on_realtime_report_ptr)(stream_write_ptr stream_write, report_tracking_flags_t report);
typedef void (*on_unknown_feedback_message_ptr)(stream_write_ptr stream_write);
typedef void (*on_stream_changed_ptr)(stream_type_t type);
typedef bool (*on_laser_ppi_enable_ptr)(uint_fast16_t ppi, uint_fast16_t pulse_length);
typedef void (*on_homing_rate_set_ptr)(axes_signals_t axes, float rate, homing_mode_t mode);
typedef void (*on_homing_completed_ptr)(void);
typedef bool (*on_probe_fixture_ptr)(tool_data_t *tool, bool at_g59_3, bool on);
typedef bool (*on_probe_start_ptr)(axes_signals_t axes, float *target, plan_line_data_t *pl_data);
typedef void (*on_probe_completed_ptr)(void);
typedef void (*on_toolchange_ack_ptr)(void);
typedef void (*on_reset_ptr)(void);
typedef void (*on_jog_cancel_ptr)(sys_state_t state);
typedef bool (*on_spindle_select_ptr)(spindle_id_t spindle_id);
typedef status_code_t (*on_unknown_sys_command_ptr)(sys_state_t state, char *line); // return Status_Unhandled.
typedef status_code_t (*on_user_command_ptr)(char *line);
typedef sys_commands_t *(*on_get_commands_ptr)(void);

typedef struct {
    // report entry points set by core at reset.
    report_t report;
    // grbl core events - may be subscribed to by drivers or by the core.
    on_state_change_ptr on_state_change;
    on_override_changed_ptr on_override_changed;
    on_report_handlers_init_ptr on_report_handlers_init;
    on_spindle_programmed_ptr on_spindle_programmed;
    on_program_completed_ptr on_program_completed;
    on_execute_realtime_ptr on_execute_realtime;
    on_execute_realtime_ptr on_execute_delay;
    on_unknown_accessory_override_ptr on_unknown_accessory_override;
    on_report_options_ptr on_report_options;
    on_report_command_help_ptr on_report_command_help;
    on_global_settings_restore_ptr on_global_settings_restore;
    on_get_alarms_ptr on_get_alarms;
    on_get_errors_ptr on_get_errors;
    on_get_settings_ptr on_get_settings;
    on_realtime_report_ptr on_realtime_report;
    on_unknown_feedback_message_ptr on_unknown_feedback_message;
    on_unknown_realtime_cmd_ptr on_unknown_realtime_cmd;
    on_unknown_sys_command_ptr on_unknown_sys_command; // return Status_Unhandled if not handled.
    on_get_commands_ptr on_get_commands;
    on_user_command_ptr on_user_command;
    on_stream_changed_ptr on_stream_changed;
    on_homing_rate_set_ptr on_homing_rate_set;
    on_homing_completed_ptr on_homing_completed;
    on_probe_fixture_ptr on_probe_fixture;
    on_probe_start_ptr on_probe_start;
    on_probe_completed_ptr on_probe_completed;
    on_toolchange_ack_ptr on_toolchange_ack; // Called from interrupt context.
    on_jog_cancel_ptr on_jog_cancel; // Called from interrupt context.
    on_laser_ppi_enable_ptr on_laser_ppi_enable;
    on_spindle_select_ptr on_spindle_select;
    on_reset_ptr on_reset; // Called from interrupt context.
    // core entry points - set up by core before driver_init() is called.
    enqueue_gcode_ptr enqueue_gcode;
    enqueue_realtime_command_ptr enqueue_realtime_command;
} grbl_t;

extern grbl_t grbl;

/*EOF*/
