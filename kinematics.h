/*
  kinematics.h - kinematics interface (API)

  Part of grblHAL

  Copyright (c) 2019 Terje Io

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
    void (*convert_array_steps_to_mpos)(float *position, int32_t *steps);
    void (*plan_target_to_steps) (int32_t *target_steps, float *target);
    bool (*segment_line) (float *target, plan_line_data_t *pl_data, bool init);
    uint_fast8_t (*limits_get_axis_mask)(uint_fast8_t idx);
    void (*limits_set_target_pos)(uint_fast8_t idx);
    void (*limits_set_machine_positions)(axes_signals_t cycle);
} kinematics_t;

extern kinematics_t kinematics;

#endif
