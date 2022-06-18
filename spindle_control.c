/*
  spindle_control.c - spindle control methods

  Part of grblHAL

  Copyright (c) 2017-2022 Terje Io
  Copyright (c) 2012-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud

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

#include <math.h>
#include <string.h>

#include "hal.h"
#include "protocol.h"
#include "state_machine.h"

#ifndef UNUSED
#define UNUSED(x) (void)(x)
#endif

static uint8_t n_spindle = 0;
static const spindle_ptrs_t *spindles[N_SPINDLE];
static spindle_id_t current_spindle = 0;

spindle_id_t spindle_register (const spindle_ptrs_t *spindle, const char *name)
{
    if(n_spindle == 0)
        memcpy(&hal.spindle, spindle, sizeof(spindle_ptrs_t));

    if(n_spindle < N_SPINDLE && settings_add_spindle_type(name)) {
        spindles[n_spindle++] = spindle;
        return n_spindle - 1;
    }

    return -1;
}

bool spindle_select (spindle_id_t spindle_id)
{
    bool ok;

    if(n_spindle == 0) {

        if(hal.spindle.set_state)
            spindles[n_spindle++] = &hal.spindle;
        else
            spindle_add_null();
    }

    if((ok = spindle_id >= 0 && spindle_id < n_spindle)) {

        if(hal.spindle.set_state && hal.spindle.set_state != spindles[spindle_id]->set_state)
            gc_spindle_off();

        if(ok) {

            spindle_ptrs_t spindle_org;

            memcpy(&spindle_org, &hal.spindle, offsetof(spindle_ptrs_t, get_data));
            memcpy(&hal.spindle, spindles[spindle_id], offsetof(spindle_ptrs_t, get_data));

            if(!hal.spindle.cap.rpm_range_locked) {
                hal.spindle.rpm_min = settings.spindle.rpm_min;
                hal.spindle.rpm_max = settings.spindle.rpm_max;
            }

            if(spindles[spindle_id]->config)
                ok = spindles[spindle_id]->config();

            if(ok) {
                current_spindle = spindle_id;
                sys.mode = settings.mode == Mode_Laser && !hal.spindle.cap.laser ? Mode_Standard : settings.mode;
                if(grbl.on_spindle_select)
                    grbl.on_spindle_select(spindle_id);
            } else
                memcpy(&spindle_org, &hal.spindle, offsetof(spindle_ptrs_t, get_data));
        }
    }

    return ok;
}

const spindle_ptrs_t *spindle_get (spindle_id_t spindle_id)
{
    if(spindle_id >= 0 && spindle_id < n_spindle)
        return spindles[spindle_id];

    return NULL;
}

spindle_id_t spindle_get_current (void)
{
    return current_spindle;
}

spindle_cap_t spindle_get_caps (void)
{
    spindle_cap_t caps = {0};
    uint_fast8_t idx = n_spindle;

    if(!idx)
        caps.value = hal.spindle.cap.value;
    else do {
        caps.value |= spindles[--idx]->cap.value;
    } while(idx);

    return caps;
}

uint8_t spindle_get_count (void)
{
    if(n_spindle == 0)
        spindle_select(0);

    return n_spindle;
}

//
// Null (dummy) spindle, automatically installed if no spindles are registered.
//

static void null_set_state (spindle_state_t state, float rpm)
{
    UNUSED(state);
    UNUSED(rpm);
}

static spindle_state_t null_get_state (void)
{
    return (spindle_state_t){0};
}

// Sets spindle speed
static void null_update_pwm (uint_fast16_t pwm_value)
{
    UNUSED(pwm_value);
}

static uint_fast16_t null_get_pwm (float rpm)
{
    UNUSED(rpm);

    return 0;
}

static void null_update_rpm (float rpm)
{
    UNUSED(rpm);
}

void spindle_add_null (void)
{
    static const spindle_ptrs_t spindle = {
        .cap.variable = Off,
        .cap.at_speed = Off,
        .cap.direction = Off,
        .set_state = null_set_state,
        .get_state = null_get_state,
        .get_pwm = null_get_pwm,
        .update_pwm = null_update_pwm,
        .update_rpm = null_update_rpm
    };

    spindle_register(&spindle, "NULL");
}

// End null (dummy) spindle.

// Set spindle speed override
// NOTE: Unlike motion overrides, spindle overrides do not require a planner reinitialization.
void spindle_set_override (uint_fast8_t speed_override)
{
    if(sys.override.control.spindle_rpm_disable)
        return;

    speed_override = constrain(speed_override, MIN_SPINDLE_RPM_OVERRIDE, MAX_SPINDLE_RPM_OVERRIDE);

    if ((uint8_t)speed_override != sys.override.spindle_rpm) {
        sys.override.spindle_rpm = (uint8_t)speed_override;
        if(state_get() == STATE_IDLE)
            spindle_set_state(0, gc_state.modal.spindle, gc_state.spindle.rpm);
        else
            sys.step_control.update_spindle_rpm = On;
       sys.report.overrides = On; // Set to report change immediately
    }
}

// Immediately sets spindle running state with direction and spindle rpm, if enabled.
// Called by g-code parser spindle_sync(), parking retract and restore, g-code program end,
// sleep, and spindle stop override.
static bool set_state (const spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    if (!ABORTED) { // Block during abort.

        if (!state.on) { // Halt or set spindle direction and rpm.
            sys.spindle_rpm = rpm = 0.0f;
            spindle->set_state((spindle_state_t){0}, 0.0f);
        } else {
            // NOTE: Assumes all calls to this function is when Grbl is not moving or must remain off.
            // TODO: alarm/interlock if going from CW to CCW directly in non-laser mode?
            if (sys.mode == Mode_Laser && state.ccw)
                rpm = 0.0f; // TODO: May need to be rpm_min*(100/MAX_SPINDLE_RPM_OVERRIDE);

            spindle->set_state(state, spindle_set_rpm(rpm, sys.override.spindle_rpm));
        }
        sys.report.spindle = On; // Set to report change immediately

        st_rpm_changed(rpm);
    }

    return !ABORTED;
}


// Immediately sets spindle running state with direction and spindle rpm, if enabled.
// Called by g-code parser spindle_sync(), parking retract and restore, g-code program end,
// sleep, and spindle stop override.
bool spindle_set_state (spindle_id_t spindle_id, spindle_state_t state, float rpm)
{
    return set_state(spindle_id == 0 ? &hal.spindle : spindles[spindle_id], state, rpm);
}

// G-code parser entry-point for setting spindle state. Forces a planner buffer sync and bails
// if an abort or check-mode is active.
bool spindle_sync (spindle_id_t spindle_id, spindle_state_t state, float rpm)
{
    bool ok;

    if (!(ok = state_get() == STATE_CHECK_MODE)) {

        const spindle_ptrs_t *spindle = spindle_id == 0 ? &hal.spindle : spindles[spindle_id];
        bool at_speed = !state.on || !spindle->cap.at_speed || settings.spindle.at_speed_tolerance <= 0.0f;

        // Empty planner buffer to ensure spindle is set when programmed.
        if((ok = protocol_buffer_synchronize()) && set_state(spindle, state, rpm) && !at_speed) {
            float on_delay = 0.0f;
            while(!(at_speed = hal.spindle.get_state().at_speed)) {
                delay_sec(0.2f, DelayMode_Dwell);
                on_delay += 0.2f;
                if(ABORTED)
                    break;
                if(on_delay >= settings.safety_door.spindle_on_delay) {
                    gc_spindle_off();
                    system_raise_alarm(Alarm_Spindle);
                    break;
                }
            }
        }

        ok &= at_speed;
    }

    return ok;
}

// Restore spindle running state with direction, enable, spindle RPM and appropriate delay.
bool spindle_restore (spindle_state_t state, float rpm)
{
    bool ok = true;

    if(sys.mode == Mode_Laser) // When in laser mode, ignore spindle spin-up delay. Set to turn on laser when cycle starts.
        sys.step_control.update_spindle_rpm = On;
    else { // TODO: add check for current spindle state matches restore state?
        spindle_set_state(0, state, rpm);
        if(state.on) {
            if((ok = !hal.spindle.cap.at_speed))
                delay_sec(settings.safety_door.spindle_on_delay, DelayMode_SysSuspend);
            else if((ok == (settings.spindle.at_speed_tolerance <= 0.0f))) {
                float delay = 0.0f;
                while(!(ok = hal.spindle.get_state().at_speed)) {
                    delay_sec(0.1f, DelayMode_SysSuspend);
                    delay += 0.1f;
                    if(ABORTED)
                        break;
                    if(delay >= settings.safety_door.spindle_on_delay) {
                        system_raise_alarm(Alarm_Spindle);
                        break;
                    }
                }
            }
        }
    }

    return ok;
}

// Calculate and set programmed RPM according to override and max/min limits
float spindle_set_rpm (float rpm, uint8_t override_pct)
{
    if(override_pct != 100)
        rpm *= 0.01f * (float)override_pct; // Scale RPM by override value.

    // Apply RPM limits
    if (rpm <= 0.0f)
        rpm = 0.0f;
    else if (rpm > hal.spindle.rpm_max)
        rpm = hal.spindle.rpm_max;
    else if (rpm < hal.spindle.rpm_min)
        rpm = hal.spindle.rpm_min;

    sys.spindle_rpm = rpm;

    return rpm;
}

//
// The following functions are not called by the core, may be called by driver code.
//

// calculate inverted pwm value if configured
static inline uint_fast16_t invert_pwm (spindle_pwm_t *pwm_data, uint_fast16_t pwm_value)
{
    return pwm_data->invert_pwm ? pwm_data->period - pwm_value - 1 : pwm_value;
}

// Precompute PWM values for faster conversion.
// Returns false if no PWM range possible, driver should revert to simple on/off spindle control if so.
bool spindle_precompute_pwm_values (spindle_pwm_t *pwm_data, uint32_t clock_hz)
{
    if(hal.spindle.rpm_max > hal.spindle.rpm_min) {
        pwm_data->period = (uint_fast16_t)((float)clock_hz / settings.spindle.pwm_freq);
        if(settings.spindle.pwm_off_value == 0.0f)
            pwm_data->off_value = pwm_data->invert_pwm ? pwm_data->period : 0;
        else
            pwm_data->off_value = invert_pwm(pwm_data, (uint_fast16_t)(pwm_data->period * settings.spindle.pwm_off_value / 100.0f));
        pwm_data->min_value = (uint_fast16_t)(pwm_data->period * settings.spindle.pwm_min_value / 100.0f);
        pwm_data->max_value = (uint_fast16_t)(pwm_data->period * settings.spindle.pwm_max_value / 100.0f) + pwm_data->offset;
        pwm_data->pwm_gradient = (float)(pwm_data->max_value - pwm_data->min_value) / (hal.spindle.rpm_max - hal.spindle.rpm_min);
        pwm_data->always_on = settings.spindle.pwm_off_value != 0.0f;
    }

#ifdef ENABLE_SPINDLE_LINEARIZATION
    uint_fast8_t idx;

    pwm_data->n_pieces = 0;

    for(idx = 0; idx < SPINDLE_NPWM_PIECES; idx++) {
        if(!isnan(settings.spindle.pwm_piece[idx].rpm) && settings.spindle.pwm_piece[idx].start != 0.0f)
            memcpy(&pwm_data->piece[pwm_data->n_pieces++], &settings.spindle.pwm_piece[idx], sizeof(pwm_piece_t));
    }
#endif

    return hal.spindle.rpm_max > hal.spindle.rpm_min;
}

// Spindle RPM to PWM conversion.
uint_fast16_t spindle_compute_pwm_value (spindle_pwm_t *pwm_data, float rpm, bool pid_limit)
{
    uint_fast16_t pwm_value;

    if(rpm > hal.spindle.rpm_min) {
      #ifdef ENABLE_SPINDLE_LINEARIZATION
        // Compute intermediate PWM value with linear spindle speed model via piecewise linear fit model.
        uint_fast8_t idx = pwm_data->n_pieces;

        if(idx) {
            do {
                idx--;
                if(idx == 0 || rpm > pwm_data->piece[idx].rpm) {
                    pwm_value = floorf(pwm_data->piece[idx].start * rpm - pwm_data->piece[idx].end);
                    break;
                }
            } while(idx);
        } else
      #endif
        // Compute intermediate PWM value with linear spindle speed model.
        pwm_value = (uint_fast16_t)floorf((rpm - hal.spindle.rpm_min) * pwm_data->pwm_gradient) + pwm_data->min_value;

        if(pwm_value >= (pid_limit ? pwm_data->period : pwm_data->max_value))
            pwm_value = pid_limit ? pwm_data->period - 1 : pwm_data->max_value;
        else if(pwm_value < pwm_data->min_value)
            pwm_value = pwm_data->min_value;

        pwm_value = invert_pwm(pwm_data, pwm_value);
    } else
        pwm_value = rpm == 0.0f ? pwm_data->off_value : invert_pwm(pwm_data, pwm_data->min_value);

    return pwm_value;
}
