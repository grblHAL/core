/*
  override.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Buffer handlers for real-time override commands

  Part of grblHAL

  Copyright (c) 2016-2023 Terje Io

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

#ifndef _OVERRIDE_H_
#define _OVERRIDE_H_

#ifndef OVERRIDE_BUFSIZE
#define OVERRIDE_BUFSIZE 16 // must be a power of 2
#endif

void flush_override_buffers ();
void enqueue_feed_override (uint8_t cmd);
uint8_t get_feed_override (void);
void enqueue_spindle_override (uint8_t cmd);
uint8_t get_spindle_override (void);
void enqueue_coolant_override (uint8_t cmd);
uint8_t get_coolant_override (void);

#endif
