/*
  stepper2.c - secondary stepper motor driver

  Part of grblHAL

  Copyright (c) 2023 Terje Io

  Algorithm based on article/code by David Austin:
  https://www.embedded.com/generate-stepper-motor-speed-profiles-in-real-time/

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

#include "hal.h"

#include <math.h>
#include <stdlib.h>

#include "stepper2.h"

#define SETDELAY(delay) ((delay) > 65535 ? 65535 : (delay))

typedef enum {
    State_Idle = 0,
    State_Accel,
    State_Run,
    State_Decel
} st2_state_t;

struct st2_motor {
    uint_fast8_t idx;
    axes_signals_t axis;
    volatile int32_t position;  // absolute step number
    uint32_t move;              // total steps to move
    uint32_t step_no;           // progress of move
    uint32_t step_run;          //
    uint32_t step_down;         // start of down-ramp
    uint32_t c32;               // 24.8 fixed point delay count
    uint32_t delay;             // integer delay count
    uint32_t first_delay;       // integer delay count
    uint16_t min_delay;         // integer delay count
    int16_t denom;              // 4.n+1 in ramp algo
    uint16_t n;                 // accel/decel steps
    uint16_t speed;             // speed mm/s
    uint16_t accel;             // acceleration m/s/s
    st2_state_t state;          // state machine state
    axes_signals_t dir;         // current direction
    uint32_t next_step;
};

st2_motor_t *st2_motor_init (uint_fast8_t axis_idx)
{
    st2_motor_t *motor;

    if((motor = malloc(sizeof(st2_motor_t)))) {
        memset(motor, 0, sizeof(st2_motor_t));
        motor->idx = axis_idx;
        motor->axis.mask = 1 << axis_idx;
        st2_motor_set_speed(motor, settings.axis[axis_idx].max_rate);
    }

    return motor;
}

uint32_t st2_motor_set_speed (st2_motor_t *motor, uint32_t speed)
{
    uint32_t prev_speed = (uint32_t)motor->speed;
    float acceleration = settings.axis[motor->idx].acceleration * settings.axis[motor->idx].steps_per_mm / 3600.0f;

    motor->speed       = speed > settings.axis[motor->idx].max_rate ? settings.axis[motor->idx].max_rate : speed;
    motor->speed       *= settings.axis[motor->idx].steps_per_mm;
    motor->min_delay   = 1000000.0f / speed;
    motor->first_delay = (uint32_t)(0.676f * sqrtf(2.0f / acceleration) * 1000000.0f);
    motor->n           = (uint32_t)(speed * speed) / (2.0f * acceleration);

    if(motor->n == 0)
        motor->n = 1;

    if(motor->first_delay < motor->min_delay)
        motor->first_delay = motor->min_delay;

    return prev_speed;
}

bool st2_motor_move (st2_motor_t *motor, const float move, const float speed, position_t type)
{
    bool dir = move < 0.0f;

    st2_motor_set_speed(motor, speed);

    if((motor->dir.mask == 0) != dir)
        motor->dir.mask = dir ? 0 : motor->axis.mask;

    switch(type) {

        case Stepper2_Steps:
            motor->move = (uint32_t)fabs((int32_t)move);
            break;

        case Stepper2_mm:
            motor->move = (uint32_t)fabs(move * settings.axis[motor->idx].steps_per_mm);
            break;
    }

    motor->step_no = 0; // step counter

    if(motor->move == 1) {

        motor->step_run  = 1;
        motor->step_down = 1;
        motor->delay     = motor->first_delay;
        motor->c32       = ((uint32_t)motor->delay) << 8;   // keep delay in 24.8 fixed-point format for ramp calcs
        motor->denom     = 1;                               // 4.n + 1, n = 0

    } else if(motor->move != 0) {

        motor->state = State_Accel;
        motor->step_run  = (motor->move - ((motor->move & 0x0001) ? 1 : 0)) >> 1;
        if(motor->step_run > motor->n)
            motor->step_run = motor->n;
        motor->step_down = motor->move - motor->step_run;
        motor->delay     = motor->first_delay;
        motor->c32       = ((uint32_t)motor->delay) << 8;   // keep delay in 24.8 fixed-point format for ramp calcs
        motor->denom     = 1;                               // 4.n + 1, n = 0
        motor->next_step = hal.get_micros();
    }

    hal.stepper.output_step(motor->axis, motor->dir);

    return true;
}

bool st2_motor_run (st2_motor_t *motor)
{
    if(motor->state == State_Idle || hal.get_micros() - motor->next_step < motor->delay)
        return motor->state != State_Idle;

    // output step;

    hal.stepper.output_step(motor->axis, motor->dir);

    if(motor->dir.mask)
        motor->position--;
    else
        motor->position++;

    motor->step_no++;
    motor->next_step += motor->delay;

    switch(motor->state) {

        case State_Accel:
            if (motor->step_no == motor->step_run) {

                motor->state = motor->step_run == motor->step_down ? State_Decel : State_Run;
                motor->denom -= 2;
                if(motor->state == State_Run)
                    motor->delay = motor->min_delay;

            } else {

                motor->denom += 4;
                motor->c32 -= (motor->c32 << 1) / motor->denom; // ramp algorithm
                motor->delay = (motor->c32 + 128) >> 8;         // round 24.8 format -> int16

                if (motor->delay <= motor->min_delay) {         // go to constant speed?
                    motor->denom -= 6;
                    motor->state = State_Run;
                    motor->step_down = motor->move - motor->step_no;
                    motor->delay = motor->min_delay;
                }
            }

            break;

        case State_Run:
            if (motor->step_no == motor->step_down)
                motor->state = State_Decel;
            break;

        case State_Decel:
            if (motor->denom < 2)  // done?
                motor->state = State_Idle;

            else {

                motor->c32 += (motor->c32 << 1) / motor->denom; // ramp algorithm
                motor->delay = (motor->c32 - 128) >> 8;         // round 24.8 format -> int16
                motor->denom -= 4;
            }
            break;

        default:
            break;
    }

    return motor->state != State_Idle;
}

bool st2_motor_stop (st2_motor_t *motor)
{
    if(motor->state == State_Accel) {
        motor->step_no = motor->step_down - 1;
        motor->step_run = motor->step_down;
    } else if(motor->state == State_Run)
        motor->step_no = motor->step_down - 1;

    return true;
}

bool st2_motor_running (st2_motor_t *motor)
{
    return motor->state != State_Idle;
}
