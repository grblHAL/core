/*
  report.h - reporting and messaging methods

  Part of grblHAL

  Copyright (c) 2018-2021 Terje Io
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

#ifndef _REPORT_H_
#define _REPORT_H_

#include "system.h"

// Message types for uncoded messages
typedef enum {
    Message_Plain = 0,
    Message_Info,
    Message_Warning
} message_type_t;

typedef enum {
    SettingsFormat_MachineReadable = 0,
    SettingsFormat_HumanReadable,
    SettingsFormat_Grbl,
    SettingsFormat_grblHAL
} settings_format_t;

// Initialize reporting subsystem
void report_init (void);
void report_init_fns (void);

// Prints system status messages.
status_code_t report_status_message (status_code_t status_code);

// Prints system alarm messages.
alarm_code_t report_alarm_message (alarm_code_t alarm_code);

// Prints feedback message, typically from gcode.
void report_message (const char *msg, message_type_t type);

// Prints miscellaneous feedback messages.
message_code_t report_feedback_message (message_code_t message_code);

// Prints welcome message.
void report_init_message (void);

// Prints Grbl help.
status_code_t report_help (char *args, char *lcargs);
void report_grbl_help();

// Prints Grbl settings
void report_grbl_settings (bool all, void *data);

// Prints Grbl setting
status_code_t report_grbl_setting (setting_id_t id, void *data);

// Prints an echo of the pre-parsed line received right before execution.
void report_echo_line_received (char *line);

// Prints realtime status report.
void report_realtime_status (void);

// Prints recorded probe position.
void report_probe_parameters (void);

// Prints current tool offsets.
void report_tool_offsets (void);

// Prints Grbl NGC parameters (coordinate offsets, probe).
void report_ngc_parameters (void);

// Prints current g-code parser mode state.
void report_gcode_modes (void);

// Prints startup line when requested and executed.
void report_startup_line (uint8_t n, char *line);
void report_execute_startup_message (char *line, status_code_t status_code);

// Prints build info and user info.
void report_build_info (char *line, bool extended);

status_code_t report_alarm_details (void);
status_code_t report_error_details (void);
status_code_t report_setting_group_details (bool by_id, char *prefix);
status_code_t report_settings_details (settings_format_t format, setting_id_t setting, setting_group_t group);
#ifndef NO_SETTINGS_DESCRIPTIONS
status_code_t report_setting_description (settings_format_t format, setting_id_t id);
#endif

status_code_t report_last_signals_event (sys_state_t state, char *args);
status_code_t report_current_limit_state (sys_state_t state, char *args);

// Prints spindle data (encoder pulse and index count, angular position).
status_code_t report_spindle_data (sys_state_t state, char *args);

// Prints pin assignments
status_code_t report_pins (sys_state_t state, char *args);

// Prints current PID log.
void report_pid_log (void);

#endif
