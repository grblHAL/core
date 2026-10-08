/*
  stepper2.c - secondary stepper motor driver

  Part of grblHAL

  Copyright (c) 2023-2026 Terje Io

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

#ifdef DEBUGOUT
#define ST2_DEBUG 0
#else
#define ST2_DEBUG 0
#endif

typedef struct {
    uint8_t is_spindle    :1,
            is_bound      :1,
            polling       :1,
            position_lost :1,
            timer_32_bit  :1,
            unused        :3;
} st2_motor_flags_t;

/*! \brief Internal structure for holding motor configuration and keeping track of its status.

__NOTE:__ The contents of this structure should _not_ be accessed directly by user code.
*/
struct st2_motor {
    uint_fast8_t idx;
    axes_signals_t axis;
    st2_motor_flags_t flags;
    volatile int64_t position;  // absolute step number
    st2_profile_t profile;
    axes_signals_t dir;         // current direction
    uint64_t next_step;
    hal_timer_t step_inject_timer;
    foreground_task_ptr on_stopped;
    st2_motor_t *next;
};

static st2_motor_t *motors = NULL;
static uint8_t spindle_motors = 0;
static settings_changed_ptr on_settings_changed;
static on_set_axis_setting_unit_ptr on_set_axis_setting_unit = NULL;
static on_setting_get_description_ptr on_setting_get_description;
static on_reset_ptr on_reset;

static void motor_irq (void *context);

/*! \brief Calculate basic motor configuration.

\param motor pointer to a \a st2_motor structure.
*/
FLASHMEM static void st2_motor_config (st2_motor_t *motor, axis_settings_t *cfg)
{
    _st2_config(&motor->profile, cfg);
}

/*! \brief Stop all motors.
 *
This will be called on a soft reset and stops all running motors abruptly.

__NOTE:__ position will likely be lost for running motors.
*/
FLASHMEM static void st2_reset (void)
{
    st2_motor_t *motor = motors;

    while(motor) {
        motor->flags.position_lost = motor->profile.state != State_Idle;
        motor->profile.state = State_Idle;
        motor = motor->next;
    }

    if(on_reset)
        on_reset();
}

/*! \brief Update basic motor configuration on settings changes.

\param settings pointer to a \a settings_t structure.
\param changed a \a settings_changed_flags_t structure.
*/
FLASHMEM static void onSettingsChanged (settings_t *settings, settings_changed_flags_t changed)
{
    st2_motor_t *motor = motors;

    on_settings_changed(settings, changed);

    while(motor) {
        if(motor->flags.is_bound)
            st2_motor_config(motor, &settings->axis[motor->idx]);
        motor = motor->next;
    }
}

/*! \brief Override default axis settings units for stepper spindle motors.

\param setting_id id of setting.
\param axis_idx axis index, X = 0, Y = 1, Z = 2, ...
\returns pointer to new unit string or NULL if no change.
*/
FLASHMEM static const char *st2_set_axis_setting_unit (setting_id_t setting_id, uint_fast8_t axis_idx)
{
    const char *unit = NULL;

    if(!settings.stepper_spindle_flags.cfg_as_rotary && bit_istrue(spindle_motors, bit(axis_idx))) switch(setting_id) {

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
FLASHMEM static const char *st2_setting_get_description (setting_id_t id)
{
    uint_fast8_t axis_idx;
    const char *descr = NULL;

    if(!settings.stepper_spindle_flags.cfg_as_rotary) switch(settings_get_axis_base(id, &axis_idx)) {

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

FLASHMEM void st2_motor_register_stopped_callback (st2_motor_t *motor, foreground_task_ptr callback)
{
    motor->on_stopped = callback;
}

/*! \brief Bind and initialize a motor.

Binds motor 0 as a spindle.
\param axis_idx axis index of motor to bind to. 3 = A, 4 = B, ...
\returns \a true if successful, \a false if not.
*/
FLASHMEM bool st2_motor_bind_spindle (uint_fast8_t axis_idx, axis_settings_t *cfg)
{
    if(motors && N_AXIS > 3 && axis_idx > Z_AXIS) {

        motors->idx = axis_idx;
        motors->axis.mask = 1 << axis_idx;
        motors->flags.is_bound = motors->flags.is_spindle = true;

        spindle_motors |= motors->axis.mask;

        if(!settings.stepper_spindle_flags.cfg_as_rotary && on_set_axis_setting_unit == NULL) {

            on_set_axis_setting_unit = grbl.on_set_axis_setting_unit;
            grbl.on_set_axis_setting_unit = st2_set_axis_setting_unit;

            on_setting_get_description = grbl.on_setting_get_description;
            grbl.on_setting_get_description = st2_setting_get_description;
        }

        st2_motor_config(motors, cfg);
    }

    return motors && axis_idx > Z_AXIS;
}

static hal_timer_t *claim_timer (st2_motor_t *motor)
{
    hal_timer_t t;

    if(!(motor->flags.timer_32_bit = !!(t = hal.timer.claim((timer_cap_t){ .periodic = Off, .resolution = Timer_32bit }, 1000))))
        t = hal.timer.claim((timer_cap_t){ .periodic = Off, .resolution = Timer_16bit }, 1000);

    return t;
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
FLASHMEM st2_motor_t *st2_motor_init (uint_fast8_t axis_idx, bool is_spindle)
{
    st2_motor_t *motor = NULL, *new = motors;

    if(hal.stepper.output_step && (motor = calloc(1, sizeof(st2_motor_t)))) {

        if(hal.timer.claim && (motor->step_inject_timer = claim_timer(motor))) {
            timer_cfg_t step_inject_cfg = {
                .single_shot = false,
                .timeout_callback = motor_irq
            };
            step_inject_cfg.context = motor;
            hal.timer.configure(motor->step_inject_timer, &step_inject_cfg);
        } else if(hal.get_micros)
            motor->flags.polling = true;
        else {
            free(motor);
            return NULL;
        }

        if(!is_spindle) {

            motor->idx = axis_idx;
            motor->axis.mask = 1 << axis_idx;
            motor->flags.is_bound = true;

            st2_motor_config(motor, &settings.axis[motor->idx]);
        }

        if(new == NULL) {
            motors = motor;

            on_settings_changed = grbl.on_settings_changed;
            grbl.on_settings_changed = onSettingsChanged;

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
FLASHMEM float st2_get_speed (st2_motor_t *motor)
{
    return _st2_get_speed(&motor->profile);
}

/*! \brief Set speed.

Change speed of a running motor. Typically used for motors bound as a spindle.
Motor will be accelerated or decelerated to the new speed.
\param motor pointer to a \a st2_motor structure.
\param speed new speed.
\returns new speed in steps/s.
*/
FLASHMEM float st2_motor_set_speed (st2_motor_t *motor, float speed)
{
    if(speed == 0.0f) {
        st2_motor_stop(motor);
        return speed;
    }

    if(speed > settings.axis[motor->idx].max_rate)
        speed = settings.axis[motor->idx].max_rate;

    return _st2_set_speed(&motor->profile, speed * (motor->profile.steps_per_mm / 60.0f));
}

/*! \brief Command a motor to move.

__NOTE:__ When not driven by timer interrupt st2_motor_run() has to be called from
the foreground process at a high frequency in order for steps to be generated.
Typically this is done by registering a function with the hal.on_execute_realtime event
that calls st2_motor_run().
\param motor pointer to a \a st2_motor structure.
\param move relative distance to move.
\param speed speed
\param type a #position_t enum.
\returns \a true if command is accepted, \a false if not.
*/
FLASHMEM bool st2_motor_move (st2_motor_t *motor, const float move, const float speed, position_t type)
{
    if(speed == 0.0f)
        return false;

    motor->dir.bits = (move < 0.0f ? motor->axis.bits : 0);

    st2_motor_set_speed(motor, speed);

    if(_st2_calc_steps(&motor->profile, move, type) == 1 && type == Stepper2_Steps) {
        if(motor->profile.state == State_Idle) {

            if(motor->dir.bits)
                motor->position--;
            else
                motor->position++;

            hal.stepper.output_step(motor->axis, motor->dir);
        }

        return motor->profile.state == State_Idle;
    }

    if(!_st2_start(&motor->profile))
        return false;

    if(motor->step_inject_timer)
        hal.timer.start(motor->step_inject_timer, motor->profile.delay);

#if ST2_DEBUG
    uint32_t nn = motor->profile.n;
    float cn = motor->profile.first_delay;
    do {
        cn -= (2.0f * cn) / (4.0f * nn + 1);
    } while(--nn);

    debug_printf("mv: %.2f %.3f %d %d %d %.2f %.2f", speed, motor->profile.steps_per_mm, motor->profile.n, move, motor->profile.delay, cn, motor->profile.speed);
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
FLASHMEM bool st2_set_position (st2_motor_t *motor, int64_t position)
{
    if(motor->profile.state == State_Idle) {
        motor->position = position;
        motor->flags.position_lost = false;
    }

    return motor->profile.state == State_Idle;
}

/*! \brief Execute a move commanded by st2_motor_move().
\param motor pointer to a \a st2_profile_t structure.
\returns \a true if motor is moving (steps are output), \a false if not (motion is completed).
*/
__attribute__((always_inline)) static inline bool _motor_run (st2_motor_t *motor)
{
    st2_state_t prev_state = motor->profile.state;

    if(_st2_run(&motor->profile)) {

        hal.stepper.output_step(motor->axis, motor->dir);

        if(motor->dir.bits)
            motor->position--;
        else
            motor->position++;
    } else if(prev_state != State_Idle && motor->on_stopped)
        task_add_delayed(motor->on_stopped, motor, 2);

    return motor->profile.state != State_Idle;
}

ISR_CODE static void ISR_FUNC(motor_irq)(void *context)
{
    if(_motor_run((st2_motor_t *)context))
        hal.timer.start(((st2_motor_t *)context)->step_inject_timer, ((st2_motor_t *)context)->profile.delay);
    else
        hal.timer.stop(((st2_motor_t *)context)->step_inject_timer);
}

/*! \brief Execute a move commanded by st2_motor_move().

This should be called from the foreground process as often as possible
when step output is not driven by interrupts (polling mode).
\param motor pointer to a \a st2_motor structure.
\returns \a true if motor is moving (steps are output), \a false if not (motion is completed).
*/
FLASHMEM bool st2_motor_run (st2_motor_t *motor)
{
    if(motor->flags.polling && motor->profile.state != State_Idle) {

        uint64_t t = hal.get_micros();

        if(t - motor->next_step >= motor->profile.delay) {

            _motor_run(motor);

            motor->next_step = t;
        }
    }

    return motor->profile.state != State_Idle;
}

/*! \brief Stop a move.
This will initiate deceleration to stop the motor if it is running.
\param motor pointer to a \a st2_motor structure.
\returns \a true if motor was running, \a false if not.
*/
FLASHMEM bool st2_motor_stop (st2_motor_t *motor)
{
    return _st2_stop(&motor->profile);
}

/*! \brief Check if motor is run by polling.
\param motor pointer to a \a st2_motor structure.
\returns \a true if motor is run by polling, \a false if not.
*/
bool st2_motor_poll (st2_motor_t *motor)
{
    return motor->flags.polling;
}

/*! \brief Check if motor is running.
\param motor pointer to a \a st2_motor structure.
\returns \a true if motor is running, \a false if not.
*/
bool st2_motor_running (st2_motor_t *motor)
{
    return _st2_running(&motor->profile);
}

/*! \brief Check if motor is running in cruising phase.
\param motor pointer to a \a st2_motor structure.
\returns \a true if motor is cruising (not acceleration or decelerating), \a false if not.
*/
bool st2_motor_cruising (st2_motor_t *motor)
{
    return _st2_cruising(&motor->profile);
}
