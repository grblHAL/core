/*
  machine_limits.h - code pertaining to limit-switches and performing the homing cycle

  Part of grblHAL

  Copyright (c) 2017-2023 Terje Io
  Copyright (c) 2012-2015 Sungeun K. Jeon
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

#ifndef _MACHINE_LIMITS_H_
#define _MACHINE_LIMITS_H_

#include "nuts_bolts.h"

typedef enum
{
    HomingMode_Seek = 0,
    HomingMode_Locate,
    HomingMode_Pulloff
} homing_mode_t;

void limits_init (void);

// Perform one portion of the homing cycle based on the input settings.
status_code_t limits_go_home (axes_signals_t cycle);

// Check for soft limit violations
void limits_soft_check (float *target, planner_cond_t condition);

// Check if homing is required.
bool limits_homing_required (void);

// Set axes to be homed from settings.
void limits_set_homing_axes (void);
void limits_set_machine_positions (axes_signals_t cycle, bool add_pulloff);
void limits_set_work_envelope (void);
coord_data_t *limits_homing_pulloff (coord_data_t *distance);
void limit_interrupt_handler (limit_signals_t state);

axes_signals_t limit_signals_merge (limit_signals_t signals);

#endif
