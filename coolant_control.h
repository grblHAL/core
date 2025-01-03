/*
  coolant_control.h - coolant control methods

  Part of grblHAL

  Copyright (c) 2017-2025 Terje Io
  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC

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

#ifndef _COOLANT_CONTROL_H_
#define _COOLANT_CONTROL_H_

// if changed to > 8 bits planner_cond_t needs to be changed too
typedef union {
    uint8_t bits;                  //!< Bitmask bits
    uint8_t mask;                  //!< Bitmask
    uint8_t value;                 //!< Bitmask value
    struct {
        uint8_t flood          :1, //!< Flood coolant.
                mist           :1, //!< Mist coolant, optional.
                shower         :1, //!< Shower coolant, currently unused.
                trough_spindle :1, //!< Through spindle coolant, currently unused.
                unused         :4;
    };
} coolant_state_t;

typedef struct {
    coolant_state_t invert;
    uint16_t on_delay;
} coolant_settings_t;

// Sets the coolant pins according to state specified.
void coolant_set_state (coolant_state_t mode);

// G-code parser entry-point for setting coolant states. Checks for and executes additional conditions.
bool coolant_set_state_synced (coolant_state_t mode);

void coolant_restore (coolant_state_t mode, uint16_t on_delay_ms);

#endif
