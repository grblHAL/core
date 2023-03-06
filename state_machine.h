/*
  state_machine.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Main state machine

  Part of grblHAL

  Copyright (c) 2018-2023 Terje Io

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

#ifndef _STATE_MACHINE_H_
#define _STATE_MACHINE_H_

sys_state_t state_get (void);
uint8_t state_get_substate (void);
void state_set (sys_state_t state);
void state_update (rt_exec_t rt_exec);
bool state_door_reopened (void);
void state_suspend_manager (void);

#endif
