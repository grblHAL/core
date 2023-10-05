/*
  stepper2.h - secondary stepper motor driver

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

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    Stepper2_Steps = 0,
    Stepper2_InfiniteSteps,
    Stepper2_mm
} position_t;

struct st2_motor; // members defined in stepper2.c
typedef struct st2_motor st2_motor_t;

st2_motor_t *st2_motor_init (uint_fast8_t axis_idx);
float st2_motor_set_speed (st2_motor_t *motor, float speed);
bool st2_motor_move (st2_motor_t *motor, const float move, const float speed, position_t type);
bool st2_motor_run (st2_motor_t *motor);
bool st2_motor_running (st2_motor_t *motor);
bool st2_motor_cruising (st2_motor_t *motor);
bool st2_motor_stop (st2_motor_t *motor);
int64_t st2_get_position (st2_motor_t *motor);
bool st2_set_position (st2_motor_t *motor, int64_t position);
