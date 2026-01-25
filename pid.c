/*
  pid.c - An embedded CNC Controller with rs274/ngc (g-code) support

  PID algorithm for closed loop control

  NOTE: not referenced in the core grblHAL code

  Part of grblHAL

  Copyright (c) 2020-2021 Terje Io

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

#include <string.h>

#include "pid.h"

// Fixed point version: TODO

// Float version

void pidf_init (pidf_t *pid, pid_values_t *config)
{
    pidf_reset(pid);
    memcpy(&pid->cfg, config, sizeof(pid_values_t));
}

bool pidf_config_changed (pidf_t *pid, pid_values_t *config)
{
    return memcmp(&pid->cfg, config, sizeof(pid_values_t));
}

void pidf_reset (pidf_t *pid)
{
    pid->error = 0.0f;
    pid->i_error = 0.0f;
    pid->d_error = 0.0f;
    pid->sample_rate_prev = 1.0f;
}

float pidf (pidf_t *pid, float command, float actual, float sample_rate)
{
    float error = command - actual;
/*
    if(error > pid->deadband)
        error -= pid->deadband;
    else if (error < pid->deadband)
        error += pid->deadband;
    else
        error = 0.0f;
*/
    // calculate the proportional term
    float pidres = pid->cfg.p_gain * error;

    // calculate and add the integral term
    pid->i_error += error * (pid->sample_rate_prev / sample_rate);

    if(pid->cfg.i_max_error != 0.0f) {
        if (pid->i_error > pid->cfg.i_max_error)
            pid->i_error = pid->cfg.i_max_error;
        else if (pid->i_error < -pid->cfg.i_max_error)
            pid->i_error = -pid->cfg.i_max_error;
    }

    pidres += pid->cfg.i_gain * pid->i_error;

    // calculate and add the derivative term
    if(pid->cfg.d_gain != 0.0f) {
        float p_error = (error - pid->d_error) * (sample_rate / pid->sample_rate_prev);
        if(pid->cfg.d_max_error != 0.0f) {
            if (p_error > pid->cfg.d_max_error)
                p_error = pid->cfg.d_max_error;
            else if (p_error < -pid->cfg.d_max_error)
                p_error = -pid->cfg.d_max_error;
        }
        pidres += pid->cfg.d_gain * p_error;
        pid->d_error = error;
    }

    pid->sample_rate_prev = sample_rate;

    // limit error output
    if(pid->cfg.max_error != 0.0f) {
        if(pidres > pid->cfg.max_error)
            pidres = pid->cfg.max_error;
        else if(pidres < -pid->cfg.max_error)
            pidres = -pid->cfg.max_error;
    }

    pid->error = pidres;

    return pidres;
}
