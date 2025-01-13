/*
  core_handlers.h - core event handler entry points and some reporting entry points

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

typedef void (*init_message_ptr)(stream_write_ptr write);
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
typedef bool (*travel_limits_ptr)(float *target, axes_signals_t axes, bool is_cartesian);
typedef bool (*arc_limits_ptr)(coord_data_t *target, coord_data_t *position, point_2d_t center, float radius, plane_t plane, int32_t turns);

typedef void (*jog_limits_ptr)(float *target, float *position);
typedef bool (*home_machine_ptr)(axes_signals_t cycle, axes_signals_t auto_square);

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
typedef void (*on_report_ngc_parameters_ptr)(void);
typedef void (*on_report_command_help_ptr)(void);
typedef const char *(*on_setting_get_description_ptr)(setting_id_t id);
typedef void (*on_global_settings_restore_ptr)(void);
typedef void (*on_realtime_report_ptr)(stream_write_ptr stream_write, report_tracking_flags_t report);
typedef void (*on_unknown_feedback_message_ptr)(stream_write_ptr stream_write);
typedef void (*on_stream_changed_ptr)(stream_type_t type);
typedef bool (*on_laser_ppi_enable_ptr)(uint_fast16_t ppi, uint_fast16_t pulse_length);
typedef void (*on_homing_rate_set_ptr)(axes_signals_t axes, float rate, homing_mode_t mode);

// NOTE: cycle contains the axis flags of the executed homing cycle, success will be true when all the configured cycles are completed.
typedef void (*on_homing_completed_ptr)(axes_signals_t cycle, bool success);

typedef bool (*on_probe_toolsetter_ptr)(tool_data_t *tool, coord_data_t *position, bool at_g59_3, bool on);
typedef bool (*on_probe_start_ptr)(axes_signals_t axes, float *target, plan_line_data_t *pl_data);
typedef void (*on_probe_completed_ptr)(void);
typedef void (*on_tool_selected_ptr)(tool_data_t *tool);
typedef void (*on_tool_changed_ptr)(tool_data_t *tool);
typedef void (*on_toolchange_ack_ptr)(void);
typedef void (*on_reset_ptr)(void);
typedef void (*on_jog_cancel_ptr)(sys_state_t state);
typedef bool (*on_spindle_select_ptr)(spindle_ptrs_t *spindle);
typedef void (*on_spindle_selected_ptr)(spindle_ptrs_t *spindle);
typedef char *(*on_process_gcode_comment_ptr)(char *msg);
typedef status_code_t (*on_gcode_message_ptr)(char *msg);
typedef void (*on_rt_reports_added_ptr)(report_tracking_flags_t report);
typedef const char *(*on_set_axis_setting_unit_ptr)(setting_id_t setting_id, uint_fast8_t axis_idx);
typedef status_code_t (*on_file_open_ptr)(const char *fname, vfs_file_t *handle, bool stream);
typedef status_code_t (*on_file_end_ptr)(vfs_file_t *handle, status_code_t status);
typedef status_code_t (*on_unknown_sys_command_ptr)(sys_state_t state, char *line); // return Status_Unhandled.
typedef status_code_t (*on_user_command_ptr)(char *line);
typedef sys_commands_t *(*on_get_commands_ptr)(void);
typedef status_code_t (*on_macro_execute_ptr)(macro_id_t macro); // macro implementations _must_ claim hal.stream.read to stream macros!
typedef void (*on_macro_return_ptr)(void);
typedef void (*on_file_demarcate_ptr)(bool start);

typedef bool (*write_tool_data_ptr)(tool_data_t *tool_data);
typedef bool (*read_tool_data_ptr)(tool_id_t tool_id, tool_data_t *tool_data);
typedef bool (*clear_tool_data_ptr)(void);

typedef struct {
    uint32_t n_tools;
    tool_data_t *tool;          //!< Array of tool data, size _must_ be n_tools + 1
    read_tool_data_ptr read;
    write_tool_data_ptr write;
    clear_tool_data_ptr clear;
} tool_table_t;

/*****************
 *  User M-code  *
 *****************/

typedef enum {
    UserMCode_Unsupported = 0,  //!< 0 - M-code is not supported.
    UserMCode_Normal,           //!< 1 - M-code is supported.
    UserMCode_NoValueWords      //!< 2 - M-code supports valueless parameter words.
} user_mcode_type_t;

/*! \brief Pointer to function for checking if user defined M-code is supported.
\param mcode as a #user_mcode_t enum.
\returns #UserMCode_Normal or #UserMCode_NoValueWords if M-code is handled, #UserMCode_Unsupported if not.
*/
typedef user_mcode_type_t (*user_mcode_check_ptr)(user_mcode_t mcode);

/*! \brief Pointer to function for validating parameters for a user defined M-code.

The M-code to validate is found in \a gc_block->user_mcode, parameter values in \a gc_block->values
 and corresponding parameter letters in the \a gc_block->words bitfield union.

Parameter values claimed by the M-code must be flagged in the \a gc_block->words bitfield union by setting the
 respective parameter letters to \a false or the parser will raise the #Status_GcodeUnusedWords error.

The validation function may set \a gc_block->user_mcode_sync to \a true if it is to be executed
after all buffered motions are completed, otherwise it will be executed immediately.

__NOTE:__ Valueless parameter letters are allowed for floats if the check function returns
#UserMCode_NoValueWords for the M-code. The corresponding values are set to NAN (not a number)
if no value is given.
The validation function should always test all parameter values by using the isnan() function in addition
to any range checks when the check function returns #UserMCode_NoValueWords for the M-code.

\param gc_block pointer to a parser_block_t structure.
\returns #Status_OK enum value if validated ok, appropriate \ref status_code_t enum value if not or #Status_Unhandled if the M-code is not recognized.
*/
typedef status_code_t (*user_mcode_validate_ptr)(parser_block_t *gc_block);

/*! \brief Pointer to function for executing a user defined M-code.

The M-code to execute is found in \a gc_block->user_mcode, parameter values in \a gc_block->values
 and claimed/validated parameter letters in the \a gc_block->words bitfield union.

\param state as a #sys_state_t variable.
\param gc_block pointer to a parser_block_t structure.
\returns #Status_OK enum value if validated ok, appropriate \ref status_code_t enum value if not or #Status_Unhandled if M-code is not recognized.
*/
typedef void (*user_mcode_execute_ptr)(sys_state_t state, parser_block_t *gc_block);

/*! \brief Optional handlers for user defined M-codes.

Handlers may be chained so that several plugins can add M-codes.
Chaining is achieved by saving a copy of the current user_mcode_ptrs_t struct
 when the plugin is initialized and calling the same handler via the copy when a
 M-code is not recognized.
 */
typedef struct {
    user_mcode_check_ptr check;         //!< Handler for checking if a user defined M-code is supported.
    user_mcode_validate_ptr validate;   //!< Handler for validating parameters for a user defined M-code.
    user_mcode_execute_ptr execute;     //!< Handler for executing a user defined M-code.
} user_mcode_ptrs_t;


typedef struct {
    // report entry points set by core at reset.
    report_t report;
    //
    tool_table_t tool_table;
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
    on_report_ngc_parameters_ptr on_report_ngc_parameters;
    on_report_command_help_ptr on_report_command_help; //!< Deprecated, use system_register_commands() to register new commands.
    on_rt_reports_added_ptr on_rt_reports_added;
    on_global_settings_restore_ptr on_global_settings_restore;
    on_setting_get_description_ptr on_setting_get_description;
    on_get_alarms_ptr on_get_alarms;
    on_get_errors_ptr on_get_errors;
    on_get_settings_ptr on_get_settings;
    on_realtime_report_ptr on_realtime_report;
    on_unknown_feedback_message_ptr on_unknown_feedback_message;
    on_unknown_realtime_cmd_ptr on_unknown_realtime_cmd;
    on_unknown_sys_command_ptr on_unknown_sys_command;  //!< return Status_Unhandled if not handled.
    on_get_commands_ptr on_get_commands;                //!< Deprecated, use system_register_commands() to register new commands.
    on_user_command_ptr on_user_command;
    on_stream_changed_ptr on_stream_changed;
    on_homing_rate_set_ptr on_homing_rate_set;
    on_homing_completed_ptr on_homing_completed;
    on_probe_toolsetter_ptr on_probe_toolsetter;
    on_probe_start_ptr on_probe_start;
    on_probe_completed_ptr on_probe_completed;
    on_set_axis_setting_unit_ptr on_set_axis_setting_unit;
    on_process_gcode_comment_ptr on_process_gcode_comment;
    on_gcode_message_ptr on_gcode_message;                  //!< Called on output of message parsed from gcode. NOTE: string pointed to is freed after this call.
    on_gcode_message_ptr on_gcode_comment;                  //!< Called when a plain gcode comment has been parsed.
    on_tool_selected_ptr on_tool_selected;                  //!< Called prior to executing M6 or after executing M61.
    on_tool_changed_ptr on_tool_changed;                    //!< Called after executing M6 or M61.
    on_toolchange_ack_ptr on_toolchange_ack;                //!< Called from interrupt context.
    on_jog_cancel_ptr on_jog_cancel;                        //!< Called from interrupt context.
    on_laser_ppi_enable_ptr on_laser_ppi_enable;
    on_spindle_select_ptr on_spindle_select;                //!< Called before spindle is selected, hook in HAL overrides here
    on_spindle_selected_ptr on_spindle_selected;            //!< Called when spindle is selected, do not change HAL pointers here!
    on_reset_ptr on_reset;                                  //!< Called from interrupt context.
    on_file_demarcate_ptr on_file_demarcate;                //!< Called when percent sign is parsed in the gcode stream.
    on_file_open_ptr on_file_open;                          //!< Called when a file is opened for streaming.
    on_file_end_ptr on_file_end;                            //!< Called when a file opened for streaming reaches the end.
    user_mcode_ptrs_t user_mcode;                           //!< Optional handlers for user defined M-codes.
    // core entry points - set up by core before driver_init() is called.
    home_machine_ptr home_machine;
    travel_limits_ptr check_travel_limits;
    arc_limits_ptr check_arc_travel_limits;
    jog_limits_ptr apply_jog_limits;
    enqueue_gcode_ptr enqueue_gcode;
    enqueue_realtime_command_ptr enqueue_realtime_command;
    on_macro_execute_ptr on_macro_execute;
    on_macro_return_ptr on_macro_return;                //!< NOTE: will be cleared on a hal.driver_reset call.
} grbl_t;

extern grbl_t grbl;

/*EOF*/
