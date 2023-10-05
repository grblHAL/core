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

typedef enum {
    State_Idle = 0,
    State_Accel,
    State_Run,
    State_RunInfinite,
    State_DecelTo,
    State_Decel
} st2_state_t;

struct st2_motor {
    uint_fast8_t idx;
    axes_signals_t axis;
    volatile int64_t position;  // absolute step number
    position_t ptype;           //
    st2_state_t state;          // state machine state
    uint32_t move;              // total steps to move
    uint32_t step_no;           // progress of move
    uint32_t step_run;          //
    uint32_t step_down;         // start of down-ramp
    uint64_t c64;               // 24.16 fixed point delay count
    uint32_t delay;             // integer delay count
    uint32_t first_delay;       // integer delay count
    uint16_t min_delay;         // integer delay count
    int32_t denom;              // 4.n+1 in ramp algo
    uint32_t n;                 // accel/decel steps
    float speed;                // speed steps/s
    float prev_speed;           // speed steps/s
    float acceleration;         // acceleration steps/s^2
    axes_signals_t dir;         // current direction
    uint32_t next_step;
    st2_motor_t *next;
};

static st2_motor_t *motors = NULL;
static settings_changed_ptr settings_changed;
static on_reset_ptr on_reset;

static void st_motor_config (st2_motor_t *motor)
{
    motor->acceleration = settings.axis[motor->idx].acceleration * settings.axis[motor->idx].steps_per_mm / 3600.0f;
    motor->first_delay = (uint32_t)(0.676f * sqrtf(2.0f / motor->acceleration) * 1000000.0f);
}

static void st2_reset (void)
{
    st2_motor_t *motor = motors;

    while(motor) {
        motor->state = State_Idle;
        motor = motor->next;
    }
}

static void st2_settings_changed (settings_t *settings, settings_changed_flags_t changed)
{
    st2_motor_t *motor = motors;

    settings_changed(settings, changed);

    while(motor) {
        st_motor_config(motor);
        motor = motor->next;
    }
}

st2_motor_t *st2_motor_init (uint_fast8_t axis_idx)
{
    st2_motor_t *motor, *new = motors;

    if((motor = malloc(sizeof(st2_motor_t)))) {

        memset(motor, 0, sizeof(st2_motor_t));
        motor->idx = axis_idx;
        motor->axis.mask = 1 << axis_idx;

        st_motor_config(motor);

        if(new == NULL) {
            motors = motor;

            settings_changed = hal.settings_changed;
            hal.settings_changed = st2_settings_changed;

            on_reset = grbl.on_reset;
            grbl.on_reset = st2_reset;

        } else {
            while(new->next)
                new = new->next;
            new->next = motor;
        }
    }

    return motor;
}

float st2_motor_set_speed (st2_motor_t *motor, float speed)
{
    motor->speed = speed > settings.axis[motor->idx].max_rate ? settings.axis[motor->idx].max_rate : speed;
    motor->speed *= settings.axis[motor->idx].steps_per_mm / 60.0f;

    if(motor->speed == motor->prev_speed)
       return motor->speed;

    motor->min_delay = (uint32_t)(1000000.0f / motor->speed);
    motor->n         = (uint32_t)(motor->speed * motor->speed) / (2.0f * motor->acceleration);

    if(motor->n == 0)
        motor->n = 1;

    if(motor->state != State_Idle) {

        int32_t pn = motor->n - ((motor->denom - 1) >> 2);

        if(pn == 0)
            return motor->speed;

#ifdef DEBUGOUT
        debug_writeln("!!");
        debug_writeln(uitoa(motor->state));
        debug_writeln(ftoa(motor->prev_speed, 2));
        debug_writeln(ftoa(motor->speed, 2));
        debug_writeln(uitoa((motor->denom - 1) >> 2));
        debug_writeln(uitoa(motor->n));
        debug_write(pn < 0 ? "-" : "+");
        debug_writeln(uitoa(pn < 0 ? -pn : pn));
        debug_writeln(uitoa(motor->denom));
#endif

        if(motor->speed > motor->prev_speed) {
            if(motor->state == State_Accel)
                motor->step_run += pn;
            else {
                motor->step_run = motor->step_no + pn;
                motor->state = State_Accel;
            }
        } else {
            if(motor->speed == 0.0f)
                motor->state = State_Decel;
            if(motor->state != State_Decel) {
                motor->step_run = motor->step_no - pn;
                motor->state = State_DecelTo;
            }
        }
    }

    motor->prev_speed = motor->speed;

    if(motor->first_delay < motor->min_delay)
        motor->first_delay = motor->min_delay;

    return motor->prev_speed;
}

bool st2_motor_move (st2_motor_t *motor, const float move, const float speed, position_t type)
{
    bool dir = move < 0.0f;

    if(speed == 0.0f)
        return false;

    if((motor->dir.mask == 0) != dir)
        motor->dir.mask = dir ? 0 : motor->axis.mask;

    motor->ptype = type;

    switch(type) {

        case Stepper2_Steps:
            motor->move = (uint32_t)fabs((int32_t)move);
            break;

        case Stepper2_InfiniteSteps:
            motor->move = (uint32_t)fabs((int32_t)move);
            break;

        case Stepper2_mm:
            motor->move = (uint32_t)fabs(move * settings.axis[motor->idx].steps_per_mm);

            break;
    }

    st2_motor_set_speed(motor, speed);

    motor->step_no = 0; // step counter

    if(type == Stepper2_InfiniteSteps) {

        motor->state     = State_Accel;
        motor->step_run  = motor->n;
        motor->step_down = motor->n + 1;
        motor->delay     = motor->first_delay;
        motor->c64       = ((uint32_t)motor->delay) << 16;   // keep delay in 24.16 fixed-point format for ramp calcs
        motor->denom     = 1;                               // 4.n + 1, n = 0
        motor->next_step = hal.get_micros();
    } else if(motor->move == 1) {

        motor->step_run  = 1;
        motor->step_down = 1;
        motor->delay     = motor->first_delay;
        motor->c64       = ((uint32_t)motor->delay) << 8;   // keep delay in 24.16 fixed-point format for ramp calcs
        motor->denom     = 1;                               // 4.n + 1, n = 0

        hal.stepper.output_step(motor->axis, motor->dir);

    } else if(motor->move != 0) {

        motor->state = State_Accel;
        motor->step_run  = (motor->move - ((motor->move & 0x0001) ? 1 : 0)) >> 1;
        if(motor->step_run > motor->n)
            motor->step_run = motor->n;
        motor->step_down = motor->move - motor->step_run;
        motor->delay     = motor->first_delay;
        motor->c64       = ((uint32_t)motor->delay) << 8;   // keep delay in 24.16 fixed-point format for ramp calcs
        motor->denom     = 1;                               // 4.n + 1, n = 0
        motor->next_step = hal.get_micros();
    }


#ifdef DEBUGOUT
    uint32_t nn = motor->n;
    float cn = motor->first_delay;
    do {
        cn -= (2.0f * cn) / (4.0f * nn + 1);
    } while(--nn);

    debug_writeln("move");
    debug_writeln(ftoa(speed, 2));
    debug_writeln(ftoa(settings.axis[motor->idx].steps_per_mm, 3));
    debug_writeln(uitoa(motor->n));
    debug_writeln(uitoa(motor->delay));
    debug_writeln(uitoa(motor->min_delay));
    debug_writeln(ftoa(cn, 2));
    debug_writeln(ftoa(motor->speed, 2));
#endif

    return true;
}

int64_t st2_get_position (st2_motor_t *motor)
{
    return motor->position;
}

bool st2_set_position (st2_motor_t *motor, int64_t position)
{
    if(motor->state == State_Idle)
        motor->position = position;

    return motor->state == State_Idle;
}

bool st2_motor_run (st2_motor_t *motor)
{
    uint32_t t = hal.get_micros();

    if(motor->state == State_Idle || t - motor->next_step < motor->delay)
        return motor->state != State_Idle;

    // output step;

    hal.stepper.output_step(motor->axis, motor->dir);

    if(motor->dir.mask)
        motor->position--;
    else
        motor->position++;

    motor->step_no++;

    switch(motor->state) {

        case State_Accel:
            if(motor->step_no == motor->step_run) {
                motor->state = motor->step_run == motor->step_down ? State_Decel : (motor->ptype == Stepper2_InfiniteSteps ? State_RunInfinite : State_Run);
                motor->denom -= 2;
                if(motor->state == State_Run || motor->state == State_RunInfinite)
                    motor->delay = motor->min_delay;
            } else {
                motor->denom += 4;
                motor->c64 -= (motor->c64 << 1) / motor->denom; // ramp algorithm
                motor->delay = (motor->c64 + 32768) >> 16;      // round 24.16 format -> int16
                if (motor->delay < motor->min_delay) {         // go to constant speed?
                    motor->denom -= 6; // causes issues with speed override for infinite moves
                    motor->state = motor->ptype == Stepper2_InfiniteSteps ? State_RunInfinite : State_Run;
                    motor->step_down = motor->move - motor->step_no;
                    motor->delay = motor->min_delay;
                }
            }
            break;

        case State_Run:
            if(motor->step_no == motor->step_down)
                motor->state = State_Decel;
            break;

        case State_Decel:
            if(motor->denom < 2) { // done?
                motor->state = State_Idle;
                motor->prev_speed = 0.0f;
                motor->n = 0;
#ifdef DEBUGOUT
                debug_writeln(uitoa(motor->position));
#endif
            } else {
                motor->c64 += (motor->c64 << 1) / motor->denom; // ramp algorithm
                motor->delay = (motor->c64 - 32768) >> 16;      // round 24.16 format -> int16
                motor->denom -= 4;
            }
            break;

        case State_DecelTo:
            if(motor->step_no != motor->step_run) {
                motor->c64 += (motor->c64 << 1) / motor->denom; // ramp algorithm
                motor->delay = (motor->c64 - 32768) >> 16;      // round 24.16 format -> int16
                motor->denom -= 4;
            } else
                motor->state = motor->ptype == Stepper2_InfiniteSteps ? State_RunInfinite : State_Run;
            break;

        default:
            break;
    }
    motor->next_step = t;

    return motor->state != State_Idle;
}

bool st2_motor_stop (st2_motor_t *motor)
{
    switch(motor->state) {

        case State_Accel:
            motor->step_no = motor->step_down - 1;
            motor->step_run = motor->step_down;
            break;

        case State_Run:
            motor->step_no = motor->step_down - 1;
            break;

        case State_RunInfinite:
        case State_DecelTo:
            motor->state = State_Decel;
            break;

        default:
            break;
    }

    return motor->state != State_Idle;
}

bool st2_motor_running (st2_motor_t *motor)
{
    return motor->state != State_Idle;
}

bool st2_motor_cruising (st2_motor_t *motor)
{
    return motor->state == State_Run || motor->state == State_RunInfinite;
}
