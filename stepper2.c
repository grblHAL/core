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

typedef enum {
    State_Idle = 0,     //!< 0
    State_Accel,        //!< 1
    State_Run,          //!< 2
    State_RunInfinite,  //!< 3
    State_DecelTo,      //!< 4
    State_Decel         //!< 5
} st2_state_t;

/* Ramp state is private to the generator, independent of HAL and DMA.
 * step_no is a recurrence cursor; controlled stop may rewrite it. It must
 * never be used as an EOF-confirmed physical step counter. */
typedef struct {
    position_t ptype;           // finite steps, distance or infinite motion
    st2_state_t state;          // state machine state
    uint32_t move;              // total steps to move
    uint32_t step_no;           // ramp cursor, may be changed by controlled stop
    uint32_t step_run;          // end of acceleration or speed transition
    uint32_t step_down;         // start of down-ramp
    uint64_t c64;               // 24.16 fixed point delay count
    uint64_t delay;             // microseconds until the next service
    uint32_t first_delay;       // integer delay count
    uint16_t min_delay;         // integer delay count
    int32_t denom;              // 4.n+1 in ramp algo
    uint32_t n;                 // accel/decel steps
    float speed;                // speed steps/s
    float prev_speed;           // speed steps/s
    float steps_per_mm;         // unit conversion, steps/mm
    float acceleration;         // acceleration steps/s^2
} st2_profile_t;

/* One direct executor: prefer a claimed timer, otherwise poll the clock.
 * Timer availability is runtime-dependent; both branches share the profile. */
typedef struct {
    bool polling;
    uint64_t next_step;
    hal_timer_t step_inject_timer;
#if STEP_INJECT_STREAM
    bool stream;
    bool active;
    bool stop_requested;
    bool single_step;
    bool completed_pending;
    uint32_t id;
    uint32_t generated;
    uint32_t confirmed;
#endif
} st2_executor_t;

/* The optional stream executor tracks one finite motion and its confirmed
 * position. The transport owns descriptor/generation checkpoints and reports
 * cumulative progress outside its ISR and locks. Profile generation may finish
 * before physical output; the executor remains active until final confirmation.
 * Application completion callbacks are deferred to the core foreground. */

/*! \brief Internal structure for holding motor configuration and keeping track of its status.

__NOTE:__ The contents of this structure should _not_ be accessed directly by user code.
*/
struct st2_motor {
    uint_fast8_t idx;
    axes_signals_t axis;
    bool is_spindle;
    bool is_bound;
    bool position_lost;
    volatile int64_t position;  // absolute step number
    st2_profile_t profile;
    st2_executor_t executor;
    axes_signals_t dir;         // current direction
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
#if STEP_INJECT_STREAM
static bool st2_stream_move (st2_motor_t *motor, float move, float speed, position_t type);
static void st2_stream_service (st2_motor_t *motor);
#endif

/*! \brief Calculate basic motor configuration.

\param motor pointer to a \a st2_motor structure.
*/
FLASHMEM static void st2_motor_config (st2_motor_t *motor, axis_settings_t *cfg)
{
    motor->profile.steps_per_mm = cfg->steps_per_mm;
    motor->profile.acceleration = cfg->acceleration * cfg->steps_per_mm / 3600.0f;
    motor->profile.first_delay = (uint32_t)(0.676f * sqrtf(2.0f / motor->profile.acceleration) * 1000000.0f);
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
#if STEP_INJECT_STREAM
        if(motor->executor.stream) {
            hal.stepper.injection->lock();
            motor->position_lost = motor->position_lost || motor->executor.active || sys.position_lost;
            if(motor->executor.active)
                hal.stepper.injection->cancel(motor->executor.id);
            motor->executor.active = false;
            motor->executor.completed_pending = false;
            motor->executor.id++; // invalidate any callback already copied by the worker
            motor->profile.state = State_Idle;
            hal.stepper.injection->unlock();
            motor = motor->next;
            continue;
        }
#endif
        motor->position_lost = motor->profile.state != State_Idle;
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
        if(motor->is_bound)
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
        motors->is_bound = motors->is_spindle = true;

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

#if STEP_INJECT_STREAM
    if(hal.stepper.injection && (is_spindle || axis_idx >= N_AXIS ||
       !hal.stepper.injection->supports(1u << axis_idx)))
        return NULL; // no unsafe timer fallback to a stream-only output_step
#endif

    if(hal.stepper.output_step && (motor = calloc(1, sizeof(st2_motor_t)))) {

#if STEP_INJECT_STREAM
        if(!is_spindle && hal.stepper.injection && hal.stepper.injection->supports(1u << axis_idx))
            motor->executor.stream = true;
        else
#endif
        if(hal.timer.claim && (motor->executor.step_inject_timer = hal.timer.claim((timer_cap_t){ .periodic = Off }, 1000))) {
            timer_cfg_t step_inject_cfg = {
                .single_shot = false,
                .timeout_callback = motor_irq
            };
            step_inject_cfg.context = motor;
            hal.timer.configure(motor->executor.step_inject_timer, &step_inject_cfg);
        } else if(hal.get_micros)
            motor->executor.polling = true;
        else {
            free(motor);
            return NULL;
        }

        if(!is_spindle) {

            motor->idx = axis_idx;
            motor->axis.mask = 1 << axis_idx;
            motor->is_bound = true;

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
#if STEP_INJECT_STREAM
    if(motor->executor.stream) {
        hal.stepper.injection->lock();
        float speed = motor->executor.active ? motor->profile.speed * 60.0f / motor->profile.steps_per_mm : 0.0f;
        hal.stepper.injection->unlock();
        return speed;
    }
#endif
    return motor->profile.state == State_Idle ? 0.0f : 60.0f / ((float)motor->profile.delay * motor->profile.steps_per_mm / 1000000.0f);
}

static float st2_profile_set_speed (st2_profile_t *profile, float steps_per_second)
{
    profile->speed = steps_per_second;

    if(profile->speed == profile->prev_speed)
       return profile->speed;

    profile->min_delay = (uint32_t)(1000000.0f / profile->speed);
    profile->n         = (uint32_t)((profile->speed * profile->speed) / (2.0f * profile->acceleration));

    if(profile->n == 0)
        profile->n = 1;

    if(profile->state != State_Idle) {

        int32_t pn = profile->n - ((profile->denom - 1) >> 2);

        if(pn == 0)
            return profile->speed;

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

/*! \brief Set speed.

Change speed of a running motor. Typically used for motors bound as a spindle.
Motor will be accelerated or decelerated to the new speed.
\param motor pointer to a \a st2_motor structure.
\param speed new speed.
\returns new speed in steps/s.
*/
FLASHMEM float st2_motor_set_speed (st2_motor_t *motor, float speed)
{
#if STEP_INJECT_STREAM
    if(motor->executor.stream) {
        hal.stepper.injection->lock();
        if(speed == 0.0f)
            motor->executor.stop_requested = motor->executor.active;
        else if(!motor->executor.active && isfinite(speed) && speed > 0.0f) {
            float limited = speed > settings.axis[motor->idx].max_rate ? settings.axis[motor->idx].max_rate : speed;
            st2_profile_set_speed(&motor->profile, limited * (motor->profile.steps_per_mm / 60.0f));
        }
        // Finite stream speed override is deliberately not supported in v1.
        float actual = motor->profile.speed;
        hal.stepper.injection->unlock();
        return actual;
    }
#endif
    if(speed == 0.0f) {
        st2_motor_stop(motor);
        return speed;
    }

    float limited_speed = speed > settings.axis[motor->idx].max_rate
                        ? settings.axis[motor->idx].max_rate : speed;

    return st2_profile_set_speed(&motor->profile, limited_speed * (motor->profile.steps_per_mm / 60.0f));
}

/* Initialize a complete finite/infinite ramp after unit conversion. */
static bool st2_profile_start (st2_profile_t *profile)
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

    profile->state     = State_Accel;
    profile->delay     = profile->first_delay;
    profile->c64       = profile->delay << 16;  // keep delay in 24.16 fixed-point format for ramp calcs
    profile->denom     = 1;                   // 4.n + 1, n = 0
    profile->step_no   = 0;                   // ramp cursor

    return true;
}

static void st2_executor_start (st2_motor_t *motor)
{
    motor->executor.next_step = hal.get_micros();

    if(motor->executor.step_inject_timer)
        hal.timer.start(motor->executor.step_inject_timer, motor->profile.delay);
}

/*! \brief Command a motor to move.

__NOTE:__ When no timer was claimed, st2_motor_run() has to be called from
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
#if STEP_INJECT_STREAM
    if(motor->executor.stream)
        return st2_stream_move(motor, move, speed, type);
#endif
    if(speed == 0.0f)
        return false;

    motor->profile.ptype = type;
    motor->dir.bits = (move < 0.0f ? motor->axis.bits : 0);

    switch(type) {

        case Stepper2_Steps:
        case Stepper2_InfiniteSteps:
            motor->profile.move = (uint32_t)fabsf(move);
            break;

        case Stepper2_mm:
            motor->profile.move = (uint32_t)lroundf(fabsf(move * motor->profile.steps_per_mm));
            break;
    }

    st2_motor_set_speed(motor, speed);

    if(motor->profile.move == 1 && type == Stepper2_Steps) {
        if(motor->profile.state == State_Idle) {

            if(motor->dir.bits)
                motor->position--;
            else
                motor->position++;

            hal.stepper.output_step(motor->axis, motor->dir);
        }

        return motor->profile.state == State_Idle;
    }

    if(!st2_profile_start(&motor->profile))
        return false;

    st2_executor_start(motor);

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
#if STEP_INJECT_STREAM
    if(motor->executor.stream) {
        hal.stepper.injection->lock();
        int64_t position = motor->position;
        hal.stepper.injection->unlock();
        return position;
    }
#endif
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
#if STEP_INJECT_STREAM
    if(motor->executor.stream) {
        hal.stepper.injection->lock();
        bool idle = !motor->executor.active;
        if(idle) {
            motor->position = position;
            motor->position_lost = false;
        }
        hal.stepper.injection->unlock();
        return idle;
    }
#endif
    if(motor->profile.state == State_Idle) {
        motor->position = position;
        motor->position_lost = false;
    }

    return motor->profile.state == State_Idle;
}

/* Service one due profile event. Entry delay belongs to this event;
 * the updated delay belongs to the next event. The terminal service changes
 * state to Idle without producing an output. No HAL or callback side effects. */
__attribute__((always_inline)) static inline bool st2_profile_advance (st2_profile_t *profile)
{
    switch(profile->state) {

        case State_Accel:
            if(profile->step_no != profile->step_run) {
                profile->denom += 4;
                profile->c64 -= (profile->c64 << 1) / profile->denom; // ramp algorithm
                profile->delay = (profile->c64 + 32768) >> 16;      // round 24.16 format -> int16
                if(profile->delay < profile->min_delay) { // go to constant speed?
                    // Retain denom: adjusting it breaks infinite-move speed overrides.
                    profile->state = profile->ptype == Stepper2_InfiniteSteps ? State_RunInfinite : State_Run;
                    // step_no excludes the step being prepared by this call.
                    profile->step_down = profile->move - (profile->step_no + 1);
                    profile->delay = profile->min_delay;
                }
            } else {
                profile->state = profile->step_run == profile->step_down ? State_Decel : (profile->ptype == Stepper2_InfiniteSteps ? State_RunInfinite : State_Run);
                if(profile->state != State_Decel)
                    profile->delay = profile->min_delay;
            }
            if(profile->state != State_Decel)
                break;
            // Continue into deceleration without emitting a transition-only step.
            // fall through

        case State_Run:
            if(profile->step_no != profile->step_down)
                break;
            profile->state = State_Decel;
            // fall through

        case State_Decel:
            if(profile->denom < 2) { // done?
                profile->state = State_Idle;
                profile->prev_speed = 0.0f;
                profile->n = 0;
            } else {
                profile->c64 += (profile->c64 << 1) / profile->denom; // ramp algorithm
                profile->delay = (profile->c64 - 32768) >> 16;      // round 24.16 format -> int16
                profile->denom -= 4;
            }
            break;

        case State_DecelTo:
            if(profile->step_no != profile->step_run) {
                profile->denom -= 4;
                profile->c64 += (profile->c64 << 1) / profile->denom; // ramp algorithm
                profile->delay = (profile->c64 + 32768) >> 16;      // round 24.16 format -> int16
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

/* Direct executor accounting follows output acceptance (output_step is void).
 * Stream execution will account only from confirmed DMA checkpoints instead. */
__attribute__((always_inline)) static inline bool st2_executor_run (st2_motor_t *motor)
{
    st2_state_t prev_state = motor->profile.state;

    st2_profile_advance(&motor->profile);

    if(motor->profile.state != State_Idle) {
        hal.stepper.output_step(motor->axis, motor->dir);

        if(motor->dir.bits)
            motor->position--;
        else
            motor->position++;
    }

    if(motor->profile.state == State_Idle && prev_state != State_Idle && motor->on_stopped)
        task_add_delayed(motor->on_stopped, motor, 2);

    return motor->profile.state != State_Idle;
}

ISR_CODE static void ISR_FUNC(motor_irq)(void *context)
{
    st2_motor_t *motor = (st2_motor_t *)context;

    if(st2_executor_run(motor))
        hal.timer.start(motor->executor.step_inject_timer, motor->profile.delay);
    else
        hal.timer.stop(motor->executor.step_inject_timer);
}

/*! \brief Execute a move commanded by st2_motor_move().

This should be called from the foreground process as often as possible
when step output is not driven by interrupts (polling mode).
\param motor pointer to a \a st2_motor structure.
\returns \a true if motor is moving (steps are output), \a false if not (motion is completed).
*/
FLASHMEM bool st2_motor_run (st2_motor_t *motor)
{
#if STEP_INJECT_STREAM
    if(motor->executor.stream) {
        st2_stream_service(motor);
        return st2_motor_running(motor);
    }
#endif
    if(motor->executor.polling && motor->profile.state != State_Idle) {

        uint64_t t = hal.get_micros();

        if(t - motor->executor.next_step >= motor->profile.delay) {

            st2_executor_run(motor);

            motor->executor.next_step = t;
        }
    }

    return motor->profile.state != State_Idle;
}

/* Request braking without assigning a new finite endpoint. */
static bool st2_profile_request_stop (st2_profile_t *profile)
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

/*! \brief Stop a move.
This will initiate deceleration to stop the motor if it is running.
\param motor pointer to a \a st2_motor structure.
\returns \a true if motor was running, \a false if not.
*/
FLASHMEM bool st2_motor_stop (st2_motor_t *motor)
{
#if STEP_INJECT_STREAM
    if(motor->executor.stream) {
        hal.stepper.injection->lock();
        bool active = motor->executor.active;
        motor->executor.stop_requested = active;
        hal.stepper.injection->unlock();
        return active;
    }
#endif
    return st2_profile_request_stop(&motor->profile);
}

/*! \brief Check if motor is run by polling.
\param motor pointer to a \a st2_motor structure.
\returns \a true if motor is run by polling, \a false if not.
*/
bool st2_motor_poll (st2_motor_t *motor)
{
#if STEP_INJECT_STREAM
    if(motor->executor.stream)
        return true; // foreground completion service, not wall-clock step generation
#endif
    return motor->executor.polling;
}

/*! \brief Check if motor is running.
\param motor pointer to a \a st2_motor structure.
\returns \a true if motor is running, \a false if not.
*/
bool st2_motor_running (st2_motor_t *motor)
{
#if STEP_INJECT_STREAM
    if(motor->executor.stream) {
        hal.stepper.injection->lock();
        bool active = motor->executor.active;
        hal.stepper.injection->unlock();
        return active;
    }
#endif
    return motor->profile.state != State_Idle;
}

/*! \brief Check if motor is running in cruising phase.
\param motor pointer to a \a st2_motor structure.
\returns \a true if motor is cruising (not acceleration or decelerating), \a false if not.
*/
bool st2_motor_cruising (st2_motor_t *motor)
{
#if STEP_INJECT_STREAM
    if(motor->executor.stream) {
        hal.stepper.injection->lock();
        bool cruising = motor->executor.active && motor->profile.state == State_Run;
        hal.stepper.injection->unlock();
        return cruising;
    }
#endif
    return motor->profile.state == State_Run || motor->profile.state == State_RunInfinite;
}

#if STEP_INJECT_STREAM
/* The I2S worker owns the profile while active. Foreground stop requests are
 * consumed here under the transport lock, at the next unreserved event. */
static injection_event_t st2_stream_next (void *context)
{
    st2_motor_t *motor = context;
    if(motor->executor.single_step) {
        motor->executor.generated = 1;
        return (injection_event_t){0, true, true};
    }
    if(motor->executor.stop_requested) {
        st2_profile_request_stop(&motor->profile);
        motor->executor.stop_requested = false;
    }
    uint32_t delay_us = (uint32_t)motor->profile.delay;
    bool step = st2_profile_advance(&motor->profile);
    if(step)
        motor->executor.generated++;
    return (injection_event_t){delay_us, step, !step};
}

static void st2_stream_progress (void *context, const injection_progress_t *progress)
{
    st2_motor_t *motor = context;
    hal.stepper.injection->lock();
    if(motor->executor.active && motor->executor.id == progress->id) {
        if(progress->completed_steps >= motor->executor.confirmed &&
           progress->completed_steps <= motor->executor.generated) {
            int64_t delta = progress->completed_steps - motor->executor.confirmed;
            motor->position += motor->dir.bits ? -delta : delta;
            motor->executor.confirmed = progress->completed_steps;
        } else
            motor->position_lost = true;
        if(progress->result != Injection_Progress) {
            motor->executor.active = false;
            motor->profile.state = State_Idle;
            motor->profile.prev_speed = 0.0f;
            if(progress->result == Injection_Completed)
                motor->executor.completed_pending = true;
            else
                motor->position_lost = true;
        }
    }
    hal.stepper.injection->unlock();
}

static void st2_stream_service (st2_motor_t *motor)
{
    hal.stepper.injection->lock();
    bool completed = motor->executor.completed_pending;
    motor->executor.completed_pending = false;
    hal.stepper.injection->unlock();
    // Keep client callbacks in the core foreground, never the I2S worker.
    if(completed && motor->on_stopped)
        task_add_delayed(motor->on_stopped, motor, 2);
}

static bool st2_stream_move (st2_motor_t *motor, float move, float speed, position_t type)
{
    if(!isfinite(move) || !isfinite(speed) || speed <= 0.0f || type == Stepper2_InfiniteSteps)
        return false;
    st2_stream_service(motor);
    hal.stepper.injection->lock();
    if(motor->executor.active || motor->position_lost) {
        hal.stepper.injection->unlock();
        return false;
    }
    float count = fabsf(type == Stepper2_mm ? move * motor->profile.steps_per_mm : move);
    if(count < 0.5f || count >= 2147483647.0f) {
        hal.stepper.injection->unlock();
        return false;
    }
    motor->profile.move = type == Stepper2_mm ? (uint32_t)lroundf(count) : (uint32_t)count;
    motor->profile.ptype = type;
    motor->dir.bits = move < 0.0f ? motor->axis.bits : 0;
    float limited = speed > settings.axis[motor->idx].max_rate ? settings.axis[motor->idx].max_rate : speed;
    st2_profile_set_speed(&motor->profile, limited * (motor->profile.steps_per_mm / 60.0f));
    st2_profile_start(&motor->profile);
    motor->executor.single_step = type == Stepper2_Steps && motor->profile.move == 1;
    motor->executor.generated = motor->executor.confirmed = 0;
    motor->executor.stop_requested = false;
    if(++motor->executor.id == 0)
        motor->executor.id = 1;
    injection_motion_t motion = {
        .id = motor->executor.id, .axis_mask = motor->axis.bits,
        .direction_mask = motor->dir.bits, .requested_steps = motor->profile.move,
        .context = motor, .next = st2_stream_next, .notify = st2_stream_progress
    };
    motor->executor.active = hal.stepper.injection->submit(&motion);
    bool accepted = motor->executor.active;
    if(!accepted)
        motor->profile.state = State_Idle;
    hal.stepper.injection->unlock();
    return accepted;
}
#endif
