/*
  tool_change.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Manual tool change with automatic touch off

  Part of grblHAL

  Copyright (c) 2020-2024 Terje Io

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

#ifndef _TOOL_CHANGE_H_
#define _TOOL_CHANGE_H_

void tc_init (void);
status_code_t tc_probe_workpiece (void);

#endif
