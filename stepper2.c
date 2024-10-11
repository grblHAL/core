/*
  stepper2.c - secondary stepper motor driver

  Part of grblHAL

  Copyright (c) 2023-2024 Terje Io

  Algorithm based on article/code by David Austin:
  https://www.embedded.com/generate-stepper-motor-speed-profiles-in-real-time/

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

#include "hal.h"

#include <math.h>
#include <stdlib.h>

#include "stepper2.h"

typedef enum {
    State_Idle = 0,     //!< 0
    State_Accel,        //!< 1
    State_Run,          //!< 2
    State_RunInfinite,  //!< 3
    State_DecelTo,      //!< 4
    State_Decel         //!< 5
} st2_state_t;

/*! \brief Internal structure for holding motor configuration and keeping track of its status.

__NOTE:__ The contents of this structure should _not_ be accessed directly by user code.
*/
struct st2_motor {
    uint_fast8_t idx;
    axes_signals_t axis;
    bool is_spindle;
    bool is_bound;
    bool polling;
    bool position_lost;
    volatile int64_t position;  // absolute step number
    position_t ptype;           //
    st2_state_t state;          // state machine state
    uint32_t move;              // total steps to move
    uint32_t step_no;           // progress of move
    uint32_t step_run;          //
    uint32_t step_down;         // start of down-ramp
    uint64_t c64;               // 24.16 fixed point delay count
    uint64_t delay;             // integer delay count
    uint32_t first_delay;       // integer delay count
    uint16_t min_delay;         // integer delay count
    int32_t denom;              // 4.n+1 in ramp algo
    uint32_t n;                 // accel/decel steps
    float speed;                // speed steps/s
    float prev_speed;           // speed steps/s
    float acceleration;         // acceleration steps/s^2
    axes_signals_t dir;         // current direction
    uint64_t next_step;
    hal_timer_t step_inject_timer;
    foreground_task_ptr on_stopped;
    st2_motor_t *next;
};

static st2_motor_t *motors = NULL;
static uint8_t spindle_motors = 0;
static settings_changed_ptr settings_changed;
static on_set_axis_setting_unit_ptr on_set_axis_setting_unit;
static on_setting_get_description_ptr on_setting_get_description;
static on_reset_ptr on_reset;

static void motor_irq (void *context);

/*! \brief Calculate basic motor configuration.

\param motor pointer to a \a st2_motor structure.
*/
static void st_motor_config (st2_motor_t *motor)
{
    motor->acceleration = settings.axis[motor->idx].acceleration * settings.axis[motor->idx].steps_per_mm / 3600.0f;
    motor->first_delay = (uint32_t)(0.676f * sqrtf(2.0f / motor->acceleration) * 1000000.0f);
}

/*! \brief Stop all motors.
 *
This will be called on a soft reset and stops all running motors abruptly.

__NOTE:__ position will likely be lost for running motors.
*/
static void st2_reset (void)
{
    st2_motor_t *motor = motors;

    while(motor) {
        motor->position_lost = motor->state != State_Idle;
        motor->state = State_Idle;
        motor = motor->next;
    }
}

/*! \brief Update basic motor configuration on settings changes.

\param settings pointer to a \a settings_t structure.
\param changed a \a settings_changed_flags_t structure.
*/
static void st2_settings_changed (settings_t *settings, settings_changed_flags_t changed)
{
    st2_motor_t *motor = motors;

    settings_changed(settings, changed);

    while(motor) {
        if(motor->is_bound)
            st_motor_config(motor);
        motor = motor->next;
    }
}

/*! \brief Override default axis settings units for stepper spindle motors.

\param setting_id id of setting.
\param axis_idx axis index, X = 0, Y = 1, Z = 2, ...
\returns pointer to new unit string or NULL if no change.
*/
static const char *st2_set_axis_setting_unit (setting_id_t setting_id, uint_fast8_t axis_idx)
{
    const char *unit = NULL;

    if(bit_istrue(spindle_motors, bit(axis_idx))) switch(setting_id) {

        case Setting_AxisStepsPerMM:
            unit = "step/rev";
            break;

        case Setting_AxisMaxRate:
            unit = "rev/min";
            break;

        case Setting_AxisAcceleration:
            unit = "rev/sec^2";
            break;

        case Setting_AxisMaxTravel:
        case Setting_AxisLimitPos:
        case Setting_AxisBacklash:
            unit = "--";
            break;

        default:
            break;
    }

    return unit == NULL && on_set_axis_setting_unit != NULL
            ? on_set_axis_setting_unit(setting_id, axis_idx)
            : unit;
}

/*! \brief Override default axis settings descriptions for stepper spindle motors.

\param setting_id id of setting.
\returns pointer to new description string or original string if no change.
*/
static const char *st2_setting_get_description (setting_id_t id)
{
    uint_fast8_t axis_idx;
    const char *descr = NULL;

    switch(settings_get_axis_base(id, &axis_idx)) {

        case Setting_AxisStepsPerMM:
            if(bit_istrue(spindle_motors, bit(axis_idx)))
                descr = "Stepper resolution in steps per revolution.";
            break;

        case Setting_AxisMaxRate:
            if(bit_istrue(spindle_motors, bit(axis_idx)))
                descr = "Max RPM for stepper spindle.";
            break;

        case Setting_AxisAcceleration:
            if(bit_istrue(spindle_motors, bit(axis_idx)))
                descr = "Acceleration in revolutions/sec^2.";
            break;

        case Setting_AxisBacklash:
        case Setting_AxisLimitPos:
        case Setting_AxisMaxTravel:
            if(bit_istrue(spindle_motors, bit(axis_idx)))
                descr = "This setting is ignored for stepper spindles.";
            break;

        default:
            break;
    }

    return descr ? descr
                 : (on_setting_get_description ? on_setting_get_description(id) : NULL);
}

void st2_motor_register_stopped_callback (st2_motor_t *motor, foreground_task_ptr callback)
{
    motor->on_stopped = callback;
}

/*! \brief Bind and initialize a motor.

Binds motor 0 as a spindle.
\param axis_idx axis index of motor to bind to. 3 = A, 4 = B, ...
\returns \a true if successful, \a false if not.
*/
bool st2_motor_bind_spindle (uint_fast8_t axis_idx)
{
    if(motors && axis_idx > Z_AXIS) {

        motors->idx = axis_idx;
        motors->axis.mask = 1 << axis_idx;
        motors->is_bound = motors->is_spindle = true;

        spindle_motors |= motors->axis.mask;

        on_set_axis_setting_unit = grbl.on_set_axis_setting_unit;
        grbl.on_set_axis_setting_unit = st2_set_axis_setting_unit;

        on_setting_get_description = grbl.on_setting_get_description;
        grbl.on_setting_get_description = st2_setting_get_description;

        st_motor_config(motors);
    }

    return motors && axis_idx > Z_AXIS;
}

/*! \brief Bind and initialize a motor.

Allocates and initializes motor configuration/data structure.
If \a is_spindle is set \a true then axis settings will be changed to step/rev etc. when bound.
<br>__NOTE:__ X, Y or Z motors cannot be bound as a spindle.
<br>__NOTE:__ currently any axis bound as a spindle should not be instructed to move via gcode commands.
\param axis_idx axis index of motor to bind to. 0 = X, 1 = Y, 2 = Z, ...
\param is_spindle set to \a true if axis is to be used as a spindle (infinite motion).
\returns pointer to a \a st2_motor structure if successful, \a NULL if not.
*/
st2_motor_t *st2_motor_init (uint_fast8_t axis_idx, bool is_spindle)
{
    st2_motor_t *motor = NULL, *new = motors;

    if(hal.stepper.output_step && (motor = calloc(sizeof(st2_motor_t), 1))) {

        if(hal.timer.claim && (motor->step_inject_timer = hal.timer.claim((timer_cap_t){ .periodic = Off }, 1000))) {
            timer_cfg_t step_inject_cfg = {
                .single_shot = true,
                .timeout_callback = motor_irq
            };
            step_inject_cfg.context = motor;
            hal.timer.configure(motor->step_inject_timer, &step_inject_cfg);
        } else if(hal.get_micros)
            motor->polling = true;
        else {
            free(motor);
            return NULL;
        }

        if(!is_spindle) {

            motor->idx = axis_idx;
            motor->axis.mask = 1 << axis_idx;
            motor->is_bound = true;

            st_motor_config(motor);
        }

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

/*! \brief Get current speed (RPM).
\param motor pointer to a \a st2_motor structure.
\returns current speed in RPM.
*/
float st2_get_speed (st2_motor_t *motor)
{
    return motor->state == State_Idle ? 0.0f : 60.0f / ((float)motor->delay * settings.axis[motor->idx].steps_per_mm / 1000000.0f);
}

/*! \brief Set speed.

Change speed of a running motor. Typically used for motors bound as a spindle.
Motor will be accelerated or decelerated to the new speed.
\param motor pointer to a \a st2_motor structure.
\param speed new speed.
\returns new speed in steps/s.
*/
float st2_motor_set_speed (st2_motor_t *motor, float speed)
{
    motor->speed = speed > settings.axis[motor->idx].max_rate ? settings.axis[motor->idx].max_rate : speed;
    motor->speed *= settings.axis[motor->idx].steps_per_mm / 60.0f;

    if(motor->speed == motor->prev_speed)
       return motor->speed;

    motor->min_delay = (uint32_t)(1000000.0f / motor->speed);
    motor->n         = (uint32_t)((motor->speed * motor->speed) / (2.0f * motor->acceleration));

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

/*! \brief Command a motor to move.

__NOTE:__ For all motions except single steps st2_motor_run() has to be called from
the foreground process at a high frequency in order for steps to be generated.
Typically this is done by registering a function with the hal.on_execute_realtime event
that calls st2_motor_run().
\param motor pointer to a \a st2_motor structure.
\param move relative distance to move.
\param speed speed
\param type a #position_t enum.
\returns \a true if command is accepted, \a false if not.
*/
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
        case Stepper2_InfiniteSteps:
            motor->move = (uint32_t)fabsf(move);
            break;

        case Stepper2_mm:
            motor->move = (uint32_t)lroundf(fabsf(move * settings.axis[motor->idx].steps_per_mm));
            break;
    }

    st2_motor_set_speed(motor, speed);

    if(motor->move == 1 && type == Stepper2_Steps) {
        if(motor->state == State_Idle) {

            if(motor->dir.mask)
                motor->position--;
            else
                motor->position++;

            hal.stepper.output_step(motor->axis, motor->dir);
        }

        return motor->state == State_Idle;
    }

    if(type == Stepper2_InfiniteSteps) {
        motor->step_run  = motor->n;
        motor->step_down = motor->n + 1;
    } else if(motor->move != 0) {
        motor->step_run  = (motor->move - ((motor->move & 0x0001) ? 1 : 0)) >> 1;
        if(motor->step_run > motor->n)
            motor->step_run = motor->n;
        motor->step_down = motor->move - motor->step_run;
    } else
        return false;

    motor->state     = State_Accel;
    motor->delay     = motor->first_delay;
    motor->c64       = motor->delay << 16;  // keep delay in 24.16 fixed-point format for ramp calcs
    motor->denom     = 1;                   // 4.n + 1, n = 0
    motor->step_no   = 0;                   // step counter
    motor->next_step = hal.get_micros();

    if(motor->step_inject_timer)
        hal.timer.start(motor->step_inject_timer, motor->delay);

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

/*! \brief Get current position in steps.
\param motor pointer to a \a st2_motor structure.
\returns current position as number of steps.
*/
int64_t st2_get_position (st2_motor_t *motor)
{
    return motor->position;
}

/*! \brief Set current position in steps.

__NOTE:__ position will _not_ be set if motor is moving.
\param motor pointer to a \a st2_motor structure.
\param position position to set.
\returns \a true if new position was accepted, \a false if not.
*/
bool st2_set_position (st2_motor_t *motor, int64_t position)
{
    if(motor->state == State_Idle) {
        motor->position = position;
        motor->position_lost = false;
    }

    return motor->state == State_Idle;
}

/*! \brief Execute a move commanded by st2_motor_move().
\param motor pointer to a \a st2_motor structure.
\returns \a true if motor is moving (steps are output), \a false if not (motion is completed).
*/
__attribute__((always_inline)) static inline bool _motor_run (st2_motor_t *motor)
{
    st2_state_t prev_state = motor->state;

    switch(motor->state) {

        case State_Accel:
            if(motor->step_no != motor->step_run) {
                motor->denom += 4;
                motor->c64 -= (motor->c64 << 1) / motor->denom; // ramp algorithm
                motor->delay = (motor->c64 + 32768) >> 16;      // round 24.16 format -> int16
                if (motor->delay < motor->min_delay) {          // go to constant speed?
              //      motor->denom -= 6; // causes issues with speed override for infinite moves
                    motor->state = motor->ptype == Stepper2_InfiniteSteps ? State_RunInfinite : State_Run;
                    motor->step_down = motor->move - motor->step_no;
                    motor->delay = motor->min_delay;
                }
            } else {
                motor->state = motor->step_run == motor->step_down ? State_Decel : (motor->ptype == Stepper2_InfiniteSteps ? State_RunInfinite : State_Run);
                if(motor->state != State_Decel)
                    motor->delay = motor->min_delay;
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
                motor->denom -= 4;
                motor->c64 += (motor->c64 << 1) / motor->denom; // ramp algorithm
                motor->delay = (motor->c64 + 32768) >> 16;      // round 24.16 format -> int16
            } else {
                motor->delay = motor->min_delay;
                motor->state = motor->ptype == Stepper2_InfiniteSteps ? State_RunInfinite : State_Run;
            }
            break;

        default:
            break;
    }

    // output step;
    hal.stepper.output_step(motor->axis, motor->dir);

    if(motor->dir.mask)
        motor->position--;
    else
        motor->position++;

    motor->step_no++;

    if(motor->state == State_Idle && prev_state != State_Idle && motor->on_stopped)
        task_add_delayed(motor->on_stopped, motor, 2);

    return motor->state != State_Idle;
}

ISR_CODE static void ISR_FUNC(motor_irq)(void *context)
{
    if(_motor_run((st2_motor_t *)context))
        hal.timer.start(((st2_motor_t *)context)->step_inject_timer, ((st2_motor_t *)context)->delay);
    else
        hal.timer.stop(((st2_motor_t *)context)->step_inject_timer);
}

/*! \brief Execute a move commanded by st2_motor_move().

This should be called from the foreground process as often as possible
when step output is not driven by interrupts (polling mode).
\param motor pointer to a \a st2_motor structure.
\returns \a true if motor is moving (steps are output), \a false if not (motion is completed).
*/
bool st2_motor_run (st2_motor_t *motor)
{
    if(motor->polling && motor->state != State_Idle) {

        uint64_t t = hal.get_micros();

        if(t - motor->next_step >= motor->delay) {

            _motor_run(motor);

            motor->next_step = t;
        }
    }

    return motor->state != State_Idle;
}

/*! \brief Stop a move.
This will initiate deceleration to stop the motor if it is running.
\param motor pointer to a \a st2_motor structure.
\returns \a true if motor was running, \a false if not.
*/
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

/*! \brief Check if motor is run by polling.
\param motor pointer to a \a st2_motor structure.
\returns \a true if motor is run by polling, \a false if not.
*/
bool st2_motor_poll (st2_motor_t *motor)
{
    return motor->polling;
}

/*! \brief Check if motor is running.
\param motor pointer to a \a st2_motor structure.
\returns \a true if motor is running, \a false if not.
*/
bool st2_motor_running (st2_motor_t *motor)
{
    return motor->state != State_Idle;
}

/*! \brief Check if motor is running in cruising phase.
\param motor pointer to a \a st2_motor structure.
\returns \a true if motor is cruising (not acceleration or decelerating), \a false if not.
*/
bool st2_motor_cruising (st2_motor_t *motor)
{
    return motor->state == State_Run || motor->state == State_RunInfinite;
}
