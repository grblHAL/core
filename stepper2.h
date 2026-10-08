/*
  stepper2.h - secondary stepper motor driver

  Part of grblHAL

  Copyright (c) 2023 - 2026 Terje Io

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

#include <stdint.h>
#include <stdbool.h>

#include "task.h"

typedef enum {
    Stepper2_Steps = 0,     //!< 0
    Stepper2_InfiniteSteps, //!< 1
    Stepper2_mm             //!< 2
} position_t;

struct st2_motor; // members defined in stepper2.c
typedef struct st2_motor st2_motor_t;

st2_motor_t *st2_motor_init (uint_fast8_t axis_idx, bool is_spindle);
bool st2_motor_poll (st2_motor_t *motor);
bool st2_motor_bind_spindle (uint_fast8_t axis_idx, axis_settings_t *cfg);
float st2_get_speed (st2_motor_t *motor);
float st2_motor_set_speed (st2_motor_t *motor, float speed);
bool st2_motor_move (st2_motor_t *motor, const float move, const float speed, position_t type);
bool st2_motor_run (st2_motor_t *motor);
bool st2_motor_running (st2_motor_t *motor);
bool st2_motor_cruising (st2_motor_t *motor);
bool st2_motor_stop (st2_motor_t *motor);
int64_t st2_get_position (st2_motor_t *motor);
bool st2_set_position (st2_motor_t *motor, int64_t position);
void st2_motor_register_stopped_callback (st2_motor_t *motor, foreground_task_ptr callback);

// Lowlevel API

typedef enum {
    State_Idle = 0,     //!< 0
    State_Accel,        //!< 1
    State_Run,          //!< 2
    State_RunInfinite,  //!< 3
    State_DecelTo,      //!< 4
    State_Decel         //!< 5
} st2_state_t;

typedef struct {
    position_t ptype;           // finite steps, distance or infinite motion
    st2_state_t state;          // state machine state
    uint32_t move;              // total steps to move
    uint32_t step_no;           // ramp cursor, may be changed by controlled stop
    uint32_t step_run;          // end of acceleration or speed transition
    uint32_t step_down;         // start of down-ramp
    uint64_t c64;               // 24.16 fixed point delay count
    uint64_t delay;             // microseconds until the next service
    uint32_t step_length;       // step pulse length, includes min. off period
    uint32_t first_delay;       // integer delay count
    uint32_t min_delay;         // integer delay count
    int32_t denom;              // 4.n+1 in ramp algo
    uint32_t n;                 // accel/decel steps
    float speed;                // speed steps/s
    float prev_speed;           // speed steps/s
    float steps_per_mm;         // unit conversion, steps/mm
    float acceleration;         // acceleration steps/s^2
} st2_profile_t;

__attribute__((always_inline)) static inline void _st2_config (st2_profile_t *profile, axis_settings_t *cfg)
{
    profile->steps_per_mm = cfg->steps_per_mm;
    profile->acceleration = cfg->acceleration * cfg->steps_per_mm / 3600.0f;
    profile->step_length = (uint32_t)ceilf(settings.steppers.pulse_microseconds + 2.0f);
    if((profile->first_delay = (uint32_t)(0.676f * sqrtf(2.0f / profile->acceleration) * 1000000.0f)) < profile->step_length)
        profile->first_delay = profile->step_length;
}

__attribute__((always_inline)) static inline float _st2_get_speed (st2_profile_t *profile)
{
    return profile->state == State_Idle ? 0.0f : 60.0f / ((float)profile->delay * profile->steps_per_mm / 1000000.0f);
}

__attribute__((always_inline)) static inline float _st2_set_speed (st2_profile_t *profile, float steps_per_second)
{
    if((profile->speed = steps_per_second) == profile->prev_speed)
       return profile->speed;

    profile->n = (uint32_t)((profile->speed * profile->speed) / (2.0f * profile->acceleration));
    if((profile->min_delay = (uint32_t)(1000000.0f / profile->speed)) < profile->step_length)
        profile->min_delay = profile->step_length;

    if(profile->n == 0)
        profile->n = 1;

    if(profile->state != State_Idle) {

        int32_t pn = profile->n - ((profile->denom - 1) >> 2);

        if(pn == 0)
            return profile->speed;

#if ST2_DEBUG
        debug_printf("!!: %d %.2f %.3f %d %d %d %d", profile->state, profile->prev_speed, profile->speed, profile->denom - 1, profile->n, pn, profile->denom);
#endif

        if(profile->speed > profile->prev_speed) {
            if(profile->state == State_Accel)
                profile->step_run += pn;
            else {
                profile->step_run = profile->step_no + pn;
                profile->state = State_Accel;
            }
        } else {
            if(profile->speed == 0.0f)
                profile->state = State_Decel;
            if(profile->state != State_Decel) {
                profile->step_run = profile->step_no - pn;
                profile->state = State_DecelTo;
            }
        }
    }

    profile->prev_speed = profile->speed;

    if(profile->first_delay < profile->min_delay)
        profile->first_delay = profile->min_delay;

    return profile->prev_speed;
}

__attribute__((always_inline)) static inline uint32_t _st2_calc_steps (st2_profile_t *profile, float move, position_t ptype)
{
// TODO: keep track of fractional steps?

    switch((profile->ptype = ptype)) {

        case Stepper2_Steps:
        case Stepper2_InfiniteSteps:
            profile->move = (uint32_t)fabsf(move);
            break;

        case Stepper2_mm:
            profile->move = (uint32_t)lroundf(fabsf(move * profile->steps_per_mm));
            break;
    }

    return profile->move;
}

__attribute__((always_inline)) static inline bool _st2_start (st2_profile_t *profile)
{
    if(profile->ptype == Stepper2_InfiniteSteps) {
        profile->step_run  = profile->n;
        profile->step_down = profile->n + 1;
    } else if(profile->move != 0) {
        profile->step_run  = (profile->move - ((profile->move & 0x0001) ? 1 : 0)) >> 1;
        if(profile->step_run > profile->n)
            profile->step_run = profile->n;
        profile->step_down = profile->move - profile->step_run;
    } else
        return false;

    profile->state   = State_Accel;
    profile->delay   = profile->first_delay;
    profile->c64     = profile->delay << 16;  // keep delay in 24.16 fixed-point format for ramp calcs
    profile->denom   = 1;                     // 4.n + 1, n = 0
    profile->step_no = 0;                     // ramp cursor

    return true;
}

__attribute__((always_inline)) static inline bool _st2_run (st2_profile_t *profile)
{
    switch(profile->state) {

        case State_Accel:
            if(profile->step_no != profile->step_run) {
                profile->denom += 4;
                profile->c64 -= (profile->c64 << 1) / profile->denom; // ramp algorithm
                profile->delay = (profile->c64 + 32768) >> 16;        // round 24.16 format -> int16
                if(profile->delay < profile->min_delay) {             // go to constant speed?
              //      profile->denom -= 6; // causes issues with speed override for infinite moves
                    profile->state = profile->ptype == Stepper2_InfiniteSteps ? State_RunInfinite : State_Run;
                    profile->step_down = profile->move - (profile->step_no + 1);
                    profile->delay = profile->min_delay;
                }
                break;
            } else if((profile->state = profile->step_run == profile->step_down ? State_Decel : (profile->ptype == Stepper2_InfiniteSteps ? State_RunInfinite : State_Run)) != State_Decel) {
                profile->delay = profile->min_delay;
                break;
            } // else
            // no break

        case State_Run:
            if(profile->step_no != profile->step_down)
                break;
            else
                profile->state = State_Decel;
            // no break

        case State_Decel:
            if(profile->denom < 2) { // done?
                profile->state = State_Idle;
                profile->prev_speed = 0.0f;
                profile->n = 0;
#if ST2_DEBUG
                debug_writeln(uitoa(profile->position));
#endif
            } else {
                profile->c64 += (profile->c64 << 1) / profile->denom; // ramp algorithm
                profile->delay = (profile->c64 - 32768) >> 16;        // round 24.16 format -> int16
                profile->denom -= 4;
            }
            break;

        case State_DecelTo:
            if(profile->step_no != profile->step_run) {
                profile->denom -= 4;
                profile->c64 += (profile->c64 << 1) / profile->denom; // ramp algorithm
                profile->delay = (profile->c64 + 32768) >> 16;        // round 24.16 format -> int16
            } else {
                profile->delay = profile->min_delay;
                profile->state = profile->ptype == Stepper2_InfiniteSteps ? State_RunInfinite : State_Run;
            }
            break;

        default:
            break;
    }

    if(profile->state != State_Idle)
        profile->step_no++;

    return profile->state != State_Idle;
}

/*! \brief Stop a move.
This will initiate deceleration to stop the motor if it is running.
\param motor pointer to a \a st2_motor structure.
\returns \a true if motor was running, \a false if not.
*/
__attribute__((always_inline)) static inline bool _st2_stop (st2_profile_t *profile)
{
    switch(profile->state) {

        case State_Accel:
            profile->step_no = profile->step_down - 1;
            profile->step_run = profile->step_down;
            break;

        case State_Run:
            profile->step_no = profile->step_down - 1;
            break;

        case State_RunInfinite:
        case State_DecelTo:
            profile->state = State_Decel;
            break;

        default:
            break;
    }

    return profile->state != State_Idle;
}

/*! \brief Check if motor is running.
\param motor pointer to a \a st2_motor structure.
\returns \a true if motor is running, \a false if not.
*/
__attribute__((always_inline)) static inline bool _st2_running (st2_profile_t *profile)
{
    return profile->state != State_Idle;
}

/*! \brief Check if motor is running in cruising phase.
\param motor pointer to a \a st2_motor structure.
\returns \a true if motor is cruising (not acceleration or decelerating), \a false if not.
*/
__attribute__((always_inline)) static inline bool _st2_cruising (st2_profile_t *profile)
{
    return profile->state == State_Run || profile->state == State_RunInfinite;
}
