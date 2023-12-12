/*
  pid.h - An embedded CNC Controller with rs274/ngc (g-code) support

  PID algorithm for closed loop control

  NOTE: not referenced in the core grbl code

  Part of grblHAL

  Copyright (c) 2020-2023 Terje Io

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

#ifndef _PID_H_
#define _PID_H_

#include <stdbool.h>

typedef struct {
    float p_gain;
    float i_gain;
    float d_gain;
    float p_max_error;
    float i_max_error;
    float d_max_error;
    float deadband;
    float max_error;
} pid_values_t;

typedef struct {
    pid_values_t cfg;
    float deadband;
    float i_error;
    float d_error;
    float sample_rate_prev;
    float error;
    float max_error;
} pidf_t;

void pidf_reset (pidf_t *pid);
void pidf_init(pidf_t *pid, pid_values_t *config);
bool pidf_config_changed (pidf_t *pid, pid_values_t *config);
float pidf (pidf_t *pid, float command, float actual, float sample_rate);

#endif
