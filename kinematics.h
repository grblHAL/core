/*
  kinematics.h - kinematics interface (API)

  Part of grblHAL

  Copyright (c) 2019-2023 Terje Io

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

#ifndef _KINEMATICS_H_
#define _KINEMATICS_H_

typedef struct {
    float *(*transform_steps_to_cartesian)(float *position, int32_t *steps);
    float *(*transform_from_cartesian) (float *target, float *position);
    float *(*segment_line) (float *target, float *position, plan_line_data_t *pl_data, bool init); // target is cartesian, position transformed
    uint_fast8_t (*limits_get_axis_mask)(uint_fast8_t idx);
    void (*limits_set_target_pos)(uint_fast8_t idx);
    void (*limits_set_machine_positions)(axes_signals_t cycle);
    bool (*homing_cycle_validate)(axes_signals_t cycle);
    float (*homing_cycle_get_feedrate)(axes_signals_t axes, float rate, homing_mode_t mode);
} kinematics_t;

extern kinematics_t kinematics;

#endif
