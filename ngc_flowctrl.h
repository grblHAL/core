/*
  ngc_flowctrl.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Program flow control, for filesystem macros

  Part of grblHAL

  Copyright (c) 2023 Terje Io

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

#ifndef _NGC_FLOWCTRL_H_
#define _NGC_FLOWCTRL_H_

void ngc_flowctrl_init (void);
status_code_t ngc_flowctrl (uint32_t o_label, char *line, uint_fast8_t *pos, bool *skip);

#endif
