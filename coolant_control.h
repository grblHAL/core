/*
  coolant_control.h - spindle control methods

  Part of grblHAL

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

#ifndef _COOLANT_CONTROL_H_
#define _COOLANT_CONTROL_H_

typedef union {
    uint8_t value;
    uint8_t mask;
    struct {
        uint8_t flood          :1,
                mist           :1,
                shower         :1,
                trough_spindle :1,
                reserved4 :1,
                reserved5 :1,
                reserved6 :1,
                reserved7 :1;
    };
} coolant_state_t;

// Sets the coolant pins according to state specified.
void coolant_set_state(coolant_state_t mode);

// G-code parser entry-point for setting coolant states. Checks for and executes additional conditions.
bool coolant_sync(coolant_state_t mode);

#endif
