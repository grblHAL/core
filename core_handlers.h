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
#include "vfs.h"

typedef enum {
    OverrideChanged_FeedRate = 0,
    OverrideChanged_RapidRate = 0,
    OverrideChanged_SpindleRPM = 0,
    OverrideChanged_SpindleState = 0,
    OverrideChanged_CoolantState = 0,
    OverrideChanged_FanState = 0
} override_changed_t;

/* TODO: add to grbl pointers so that a different formatting (xml, json etc) of reports may be implemented by a plugin?
typedef struct {
    void (*report_echo_line_received)(char *line);
    void (*report_realtime_status)(void);
    void (*report_probe_parameters)(void);
    void (*report_ngc_parameters)(void);
    void (*report_gcode_modes)(void);
    void (*report_startup_line)(uint8_t n, char *line);
    void (*report_execute_startup_message)(char *line, status_code_t status_code);
} grbl_report_t;
*/

// Report entry points set by core at startup and reset.

typedef void (*init_message_ptr)(void);
typedef void (*help_message_ptr)(void);
typedef status_code_t (*status_message_ptr)(status_code_t status_code);
typedef message_code_t (*feedback_message_ptr)(message_code_t message_code);
typedef alarm_code_t (*alarm_message_ptr)(alarm_code_t alarm_code);

typedef struct {
    init_message_ptr init_message;          //<! Prints system welcome message.
    help_message_ptr help_message;          //<! Prints system help message.
    status_message_ptr status_message;      //<! Prints system status messages.
    feedback_message_ptr feedback_message;  //<! Prints miscellaneous feedback messages.
    alarm_message_ptr alarm_message;        //<! Prints alarm message.
    setting_output_ptr setting;
} report_t;

// Core event handler and other entry points.
// Most of the event handlers defaults to NULL, a few is set up to call a dummy handler for simpler code.

typedef bool (*enqueue_gcode_ptr)(char *data);
typedef bool (*protocol_enqueue_realtime_command_ptr)(char c);

typedef void (*on_parser_init_ptr)(parser_state_t *gc_state);
typedef void (*on_state_change_ptr)(sys_state_t state);
typedef void (*on_override_changed_ptr)(override_changed_t override);
typedef void (*on_spindle_programmed_ptr)(spindle_ptrs_t *spindle, spindle_state_t state, float rpm, spindle_rpm_mode_t mode);
typedef void (*on_wco_changed_ptr)(void);
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
typedef void (*on_tool_selected_ptr)(tool_data_t *tool);
typedef void (*on_toolchange_ack_ptr)(void);
typedef void (*on_reset_ptr)(void);
typedef void (*on_jog_cancel_ptr)(sys_state_t state);
typedef bool (*on_spindle_select_ptr)(spindle_ptrs_t *spindle);
typedef void (*on_spindle_selected_ptr)(spindle_ptrs_t *spindle);
typedef void (*on_gcode_message_ptr)(char *msg);
typedef void (*on_rt_reports_added_ptr)(report_tracking_flags_t report);
typedef void (*on_vfs_mount_ptr)(const char *path, const vfs_t *fs);
typedef void (*on_vfs_unmount_ptr)(const char *path);
typedef status_code_t (*on_file_open_ptr)(const char *fname, vfs_file_t *handle, bool stream);
typedef status_code_t (*on_unknown_sys_command_ptr)(sys_state_t state, char *line); // return Status_Unhandled.
typedef status_code_t (*on_user_command_ptr)(char *line);
typedef sys_commands_t *(*on_get_commands_ptr)(void);
typedef status_code_t (*on_macro_execute_ptr)(macro_id_t macro); // macro implementations _must_ claim hal.stream.read to stream macros!
typedef void (*on_macro_return_ptr)(void);

typedef struct {
    // report entry points set by core at reset.
    report_t report;
    // grbl core events - may be subscribed to by drivers or by the core.
    on_parser_init_ptr on_parser_init;
    on_state_change_ptr on_state_change;
    on_override_changed_ptr on_override_changed;
    on_report_handlers_init_ptr on_report_handlers_init;
    on_spindle_programmed_ptr on_spindle_programmed;
    on_wco_changed_ptr on_wco_changed;
    on_program_completed_ptr on_program_completed;
    on_execute_realtime_ptr on_execute_realtime;
    on_execute_realtime_ptr on_execute_delay;
    on_unknown_accessory_override_ptr on_unknown_accessory_override;
    on_report_options_ptr on_report_options;
    on_report_command_help_ptr on_report_command_help;
    on_rt_reports_added_ptr on_rt_reports_added;
    on_global_settings_restore_ptr on_global_settings_restore;
    on_get_alarms_ptr on_get_alarms;
    on_get_errors_ptr on_get_errors;
    on_get_settings_ptr on_get_settings;
    on_realtime_report_ptr on_realtime_report;
    on_unknown_feedback_message_ptr on_unknown_feedback_message;
    on_unknown_realtime_cmd_ptr on_unknown_realtime_cmd;
    on_unknown_sys_command_ptr on_unknown_sys_command;  //!< return Status_Unhandled if not handled.
    on_get_commands_ptr on_get_commands;
    on_user_command_ptr on_user_command;
    on_stream_changed_ptr on_stream_changed;
    on_homing_rate_set_ptr on_homing_rate_set;
    on_homing_completed_ptr on_homing_completed;
    on_probe_fixture_ptr on_probe_fixture;
    on_probe_start_ptr on_probe_start;
    on_probe_completed_ptr on_probe_completed;
    on_gcode_message_ptr on_gcode_message;              //!< Called on output of message parsed from gcode. NOTE: string pointed to is freed after this call.
    on_gcode_message_ptr on_gcode_comment;              //!< Called when a plain gcode comment has been parsed.
    on_tool_selected_ptr on_tool_selected;              //!< Called prior to executing M6 or after executing M61.
    on_toolchange_ack_ptr on_toolchange_ack;            //!< Called from interrupt context.
    on_jog_cancel_ptr on_jog_cancel;                    //!< Called from interrupt context.
    on_laser_ppi_enable_ptr on_laser_ppi_enable;
    on_spindle_select_ptr on_spindle_select;            //!< Called before spindle is selected, hook in HAL overrides here
    on_spindle_selected_ptr on_spindle_selected;        //!< Called when spindle is selected, do not change HAL pointers here!
    on_reset_ptr on_reset;                              //!< Called from interrupt context.
    on_vfs_mount_ptr on_vfs_mount;                      //!< Called when a file system is mounted.
    on_vfs_unmount_ptr on_vfs_unmount;                  //!< Called when a file system is unmounted.
    on_file_open_ptr on_file_open;                      //!< Called when a file is opened for streaming.
    // core entry points - set up by core before driver_init() is called.
    enqueue_gcode_ptr enqueue_gcode;
    enqueue_realtime_command_ptr enqueue_realtime_command;
    on_macro_execute_ptr on_macro_execute;
    on_macro_return_ptr on_macro_return;                //!< NOTE: will be cleared on a hal.driver_reset call.
} grbl_t;

extern grbl_t grbl;

/*EOF*/
