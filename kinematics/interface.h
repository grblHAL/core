/*
  kinematics.h - kinematics interface (API)

  Part of grblHAL

  Copyright (c) 2019-2026 Terje Io

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

#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_

#include "../nuts_bolts.h"

typedef coord_data_t *(*transform_steps_to_cartesian_ptr)(coord_data_t *position, mpos_t *steps);
typedef coord_data_t *(*transform_from_cartesian_ptr) (coord_data_t *target, coord_data_t *position);
typedef coord_data_t *(*segment_line_ptr) (coord_data_t *target, coord_data_t *position, plan_line_data_t *pl_data, bool init); // target is cartesian, position transformed
typedef uint_fast8_t (*limits_get_axis_mask_ptr)(uint_fast8_t idx);
typedef void (*limits_set_target_pos_ptr)(uint_fast8_t idx);
typedef void (*limits_set_machine_positions_ptr)(axes_signals_t cycle);
typedef bool (*homing_cycle_validate_ptr)(axes_signals_t cycle);
typedef float (*homing_cycle_get_feedrate_ptr)(axes_signals_t axes, float rate, homing_mode_t mode);

typedef struct {
    transform_steps_to_cartesian_ptr transform_steps_to_cartesian;
    transform_from_cartesian_ptr transform_from_cartesian;
    segment_line_ptr segment_line; // target is cartesian, position transformed
    limits_get_axis_mask_ptr limits_get_axis_mask;
    limits_set_target_pos_ptr limits_set_target_pos;
    limits_set_machine_positions_ptr limits_set_machine_positions;
    homing_cycle_validate_ptr homing_cycle_validate;
    homing_cycle_get_feedrate_ptr homing_cycle_get_feedrate;
} kinematics_t;

extern kinematics_t kinematics;

#endif
