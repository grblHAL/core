/*
  protocol.h - controls Grbl execution protocol and procedures

  Part of grblHAL

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

#ifndef _PROTOCOL_H_
#define _PROTOCOL_H_

// Line buffer size from the input stream to be executed.
// NOTE: Not a problem except for extreme cases, but the line buffer size can be too small
// and g-code blocks can get truncated. Officially, the g-code standards support up to 256
// characters. In future versions, this will be increased, when we know how much extra
// memory space we can invest into here or we re-write the g-code parser not to have this
// buffer.
#ifndef LINE_BUFFER_SIZE
  #define LINE_BUFFER_SIZE 257 // 256 characters plus terminator
#endif

typedef void (*foreground_task_ptr)(void *data);

// Starts Grbl main loop. It handles all incoming characters from the input stream and executes
// them as they complete. It is also responsible for finishing the initialization procedures.
bool protocol_main_loop (void);

// Checks and executes a realtime command at various stop points in main program
bool protocol_execute_realtime (void);
bool protocol_exec_rt_system (void);
void protocol_execute_noop (uint_fast16_t state);
bool protocol_enqueue_rt_command (on_execute_realtime_ptr fn);
bool protocol_enqueue_foreground_task (foreground_task_ptr fn, void *data);

// Executes the auto cycle feature, if enabled.
void protocol_auto_cycle_start (void);

// Block until all buffered steps are executed
bool protocol_buffer_synchronize (void);

bool protocol_enqueue_realtime_command (char c);
bool protocol_enqueue_gcode (char *data);
void protocol_message (char *message);

#endif
