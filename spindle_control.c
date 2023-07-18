/*
  spindle_control.c - spindle control methods

  Part of grblHAL

  Copyright (c) 2017-2023 Terje Io
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
#include "settings.h"

#ifndef UNUSED
#define UNUSED(x) (void)(x)
#endif

/*! \brief Structure for holding spindle registration data */
typedef struct {
    const spindle_ptrs_t *cfg;
    spindle_ptrs_t hal;
    const char *name;
    bool init_ok;
} spindle_reg_t;

/*! \brief Structure for holding data about an enabled spindle */
typedef struct {
    spindle_param_t param;
    spindle_ptrs_t hal;
    bool enabled;
} spindle_sys_t;

static uint8_t n_spindle = 0;
static spindle_sys_t sys_spindle[N_SYS_SPINDLE] = {0};
static spindle_reg_t spindles[N_SPINDLE] = {0}, *pwm_spindle = NULL;

/*! \internal \brief Activates and registers a spindle as enabled with a specific spindle number.
\param spindle_id spindle id of spindle to activate as a \ref spindle_id_t.
\param spindle_num spindle number to set as enabled as a \ref spindle_num_t.
\returns \a true if succsesful, \a false if not.
*/
static bool spindle_activate (spindle_id_t spindle_id, spindle_num_t spindle_num)
{
    bool ok;
    spindle_reg_t *spindle;

    // Always configure PWM spindle on startup to ensure outputs are set correctly.
    if(pwm_spindle && pwm_spindle->cfg->config && pwm_spindle != &spindles[spindle_id]) {

        if(!pwm_spindle->hal.cap.rpm_range_locked) {
            pwm_spindle->hal.rpm_min = settings.spindle.rpm_min;
            pwm_spindle->hal.rpm_max = settings.spindle.rpm_max;
        }

        if((pwm_spindle->init_ok = pwm_spindle->hal.config == NULL || pwm_spindle->hal.config(&pwm_spindle->hal)))
            pwm_spindle->hal.set_state((spindle_state_t){0}, 0.0f);
    }
    pwm_spindle = NULL;

    if((ok = spindle_id >= 0 && spindle_id < n_spindle && !!spindles[spindle_id].cfg)) {

        spindle = &spindles[spindle_id];

        if(sys_spindle[spindle_num].enabled && sys_spindle[spindle_num].hal.id != spindle_id && sys_spindle[spindle_num].hal.set_state)
            gc_spindle_off(); // TODO: switch off only the default spindle?

        if(!spindle->hal.cap.rpm_range_locked) {
            spindle->hal.rpm_min = settings.spindle.rpm_min;
            spindle->hal.rpm_max = settings.spindle.rpm_max;
        }

        if(!spindle->init_ok)
            ok = spindle->init_ok = spindle->hal.config == NULL || spindle->hal.config(&spindle->hal);

        if(ok) {

            spindle_ptrs_t spindle_hal;

            memcpy(&spindle_hal, &spindle->hal, sizeof(spindle_ptrs_t));

            if(spindle->cfg->get_data == NULL) {
                spindle_hal.get_data = hal.spindle_data.get;
                spindle_hal.reset_data = hal.spindle_data.reset;
                if(!spindle->cfg->cap.at_speed)
                    spindle_hal.cap.at_speed = !!spindle_hal.get_data;
            }

            spindle_hal.cap.laser &= settings.mode == Mode_Laser;

            if(grbl.on_spindle_select)
                ok = grbl.on_spindle_select(&spindle_hal);

            if(ok) {
                sys_spindle[spindle_num].enabled = true;
                sys_spindle[spindle_num].param.hal = &sys_spindle[spindle_num].hal;
                if(sys_spindle[spindle_num].param.override_pct == 0)
                    sys_spindle[spindle_num].param.override_pct = DEFAULT_SPINDLE_RPM_OVERRIDE;
                spindle_hal.param = &sys_spindle[spindle_num].param;
                memcpy(&sys_spindle[spindle_num].hal, &spindle_hal, sizeof(spindle_ptrs_t));
                if(grbl.on_spindle_selected)
                    grbl.on_spindle_selected(&sys_spindle[spindle_num].hal);
            }
        }
    }

    return ok;
}

/*! \brief Register a spindle with the core.
\param spindle pointer to a \a spindle_ptrs_t structure.
\param name pointer to a null terminated string.
\returns assigned \a spindle id as a \ref spindle_id_t if successful, -1 if not.

__NOTE:__ The first spindle registered will become the default active spindle.
__NOTE:__ up to \ref N_SPINDLE spindles can be registered at a time.
*/
spindle_id_t spindle_register (const spindle_ptrs_t *spindle, const char *name)
{
    if(n_spindle < N_SPINDLE && settings_add_spindle_type(name)) {

        spindles[n_spindle].cfg = spindle;
        spindles[n_spindle].name = name;
        memcpy(&spindles[n_spindle].hal, spindles[n_spindle].cfg, sizeof(spindle_ptrs_t));
        spindles[n_spindle].hal.id = n_spindle;

        if(spindle->type == SpindleType_PWM && pwm_spindle == NULL) {
            pwm_spindle = &spindles[n_spindle];
            hal.driver_cap.pwm_spindle = On;
        }

        if(n_spindle == 0)
            memcpy(&sys_spindle[0].hal, spindle, sizeof(spindle_ptrs_t));

        return n_spindle++;
    }

    return -1;
}

/*! \brief Enable a spindle and make it available for use by gcode.
\param spindle_id spindle id as a \ref spindle_id_t.
\returns assigned spindle number as a \a spindle_num_t if successful \a -1 if not.

__NOTE:__ up to \ref N_SYS_SPINDLE spindles can be enabled at a time.
*/
spindle_num_t spindle_enable (spindle_id_t spindle_id)
{
    uint_fast8_t idx = 0;
    spindle_num_t spindle_num = -1;

    if(spindle_id >= 0 && spindle_id < n_spindle) do {
        if(!sys_spindle[idx].enabled && spindle_activate(spindle_id, idx))
            spindle_num = idx;
    } while(++idx < N_SYS_SPINDLE && spindle_num == -1);

    return spindle_num;
}

/*! \brief Enables a spindle and sets it as default spindle (spindle number 0).
\param spindle_id spindle id as a \ref spindle_id_t.
\returns \a true if succsesful, \a false if not.
*/
bool spindle_select (spindle_id_t spindle_id)
{
    if(n_spindle == 0 && spindle_id >= 0) {
        spindle_id = 0;
        spindle_add_null();
    }

    return (sys_spindle[0].enabled && sys_spindle[0].hal.id == spindle_id) || spindle_activate(spindle_id, 0);
}


/*! \brief Get the handlers (function pointers) etc. associated with the spindle.
\param spindle_id spindle id as a \ref spindle_id_t.
\param hal a \ref spindle_hal_t enum value:
<br>\ref SpindleHAL_Raw - get the read only version as supplied at registration
<br>\ref SpindleHAL_Configured - get the version with run-time modifications applied by the spindle driver.
<br>\ref SpindleHAL_Active - get the enabled version available from gcode. Can be overriden by event handlers prior to activation.
\returns pointer to a \ref spindle_ptrs_t structure if successful, \a NULL if not.

__NOTE:__ do not modify the returned structure!
*/
spindle_ptrs_t *spindle_get_hal (spindle_id_t spindle_id, spindle_hal_t hal)
{
    spindle_ptrs_t *spindle = NULL;

    if(hal == SpindleHAL_Active) {

        uint_fast8_t idx = N_SYS_SPINDLE;

        do {
            idx--;
            if(sys_spindle[idx].hal.id == spindle_id && sys_spindle[idx].enabled)
                spindle = &sys_spindle[idx].hal;
        } while(idx && spindle == NULL);

    } else if(spindle_id >= 0 && spindle_id < n_spindle && spindles[spindle_id].cfg)
        spindle = hal == SpindleHAL_Raw ? (spindle_ptrs_t *)spindles[spindle_id].cfg : &spindles[spindle_id].hal;

    return spindle;
}

/*! \brief Get the spindle id of the default spindle (spindle number 0).
\returns spindle id as a \ref spindle_id_t if successful, \a -2 if not (no spindle available).
*/
spindle_id_t spindle_get_default (void)
{
    return sys_spindle[0].enabled ? sys_spindle[0].hal.id : -2;
}

/*! \brief Get the merged spindle capabilities of all registered spindles.
\param active true to return active capabilities, false to return default capabilities.
\returns capabilities in a \ref spindle_cap_t structure.
*/
spindle_cap_t spindle_get_caps (bool active)
{
    spindle_cap_t caps = {0};
    uint_fast8_t idx = n_spindle;

    do {
        --idx;
        caps.value |= (active ? spindles[idx].hal.cap.value : spindles[idx].cfg->cap.value);
    } while(idx);

    return caps;
}

/*! \brief Get the registered name of a spindle.
\param spindle_id spindle id as a \ref spindle_id_t.
\returns pointer to a null terminated string if succesful, \a NULL if not.
*/
const char *spindle_get_name (spindle_id_t spindle_id)
{
    return spindle_id >= 0 && spindle_id < n_spindle && spindles[spindle_id].cfg ? spindles[spindle_id].name : NULL;
}

/*! \brief Update the capabilities of a registered PWM spindle.
May be used by the driver on spindle initialization or when spindle settings has been changed.
\param spindle pointer to a \ref spindle_ptrs_t structure.
\param pwm_caps pointer to a \ref spindle_pwm_t structure.
*/
void spindle_update_caps (spindle_ptrs_t *spindle, spindle_pwm_t *pwm_caps)
{
    uint_fast8_t idx = N_SYS_SPINDLE;

    spindle->type = pwm_caps ? SpindleType_PWM : SpindleType_Basic;
    spindle->cap.laser = !!pwm_caps && !!spindle->update_pwm && settings.mode == Mode_Laser;
    spindle->pwm_off_value = pwm_caps ? pwm_caps->off_value : 0;

    do {
        idx--;
        if(sys_spindle[idx].enabled && spindle->id == sys_spindle[idx].hal.id) {
            sys_spindle[idx].hal.type = spindle->type;
            sys_spindle[idx].hal.cap.laser = spindle->cap.laser;
            sys_spindle[idx].hal.pwm_off_value =  spindle->pwm_off_value;
            break;
        }
    } while(idx);
}

/*! \brief Get number of registered spindles.
\returns number of registered spindles.
*/
uint8_t spindle_get_count (void)
{
    if(n_spindle == 0)
        spindle_select(0);

    return n_spindle;
}

static spindle_num_t spindle_get_num (spindle_id_t spindle_id)
{
    uint_fast8_t idx = N_SPINDLE_SELECTABLE;
    spindle_num_t spindle_num = -1;

    const setting_detail_t *setting;

    do {
        idx--;
        if((setting = setting_get_details(idx == 0 ? Setting_SpindleType : (setting_id_t)(Setting_SpindleEnable0 + idx), NULL))) {
            if(setting_get_int_value(setting, 0) == spindle_id)
                spindle_num = idx;
        }
    } while(idx && spindle_num == -1);

    return spindle_num;
}

/*! \brief Enumerate registered spindles by calling a callback function for each of them.
\param callback pointer to a \ref spindle_enumerate_callback_ptr type function.
\param data pointer to optional data to pass to the callback function.
\returns \a true if spindles are registered and a callback function was provided, \a false otherwise.
*/
bool spindle_enumerate_spindles (spindle_enumerate_callback_ptr callback, void *data)
{
    if(callback == NULL || n_spindle == 0)
        return false;

    uint_fast8_t idx;
    spindle_info_t spindle;

    for(idx = 0; idx < n_spindle; idx++) {

        spindle.id = idx;
        spindle.name = spindles[idx].name;
        spindle.hal = &spindles[idx].hal;
        spindle.num = spindle_get_num(idx);
        spindle.enabled = spindle.num != -1;
        spindle.is_current = spindle.enabled && sys_spindle[0].hal.id == idx;

        callback(&spindle, data);
    }

    return true;
}

// The following calls uses logical spindle numbers pointing into the sys_spindle array
// containing enabled spindles (spindle number as used by the $ gcode word)

/*! \brief Check if a spindle is enabled and available or not.
\param spindle_num spindle number as a \ref spindle_num_t.
\returns \a true if the spindle is enabled, \a false otherwise.
*/
bool spindle_is_enabled (spindle_num_t spindle_num)
{
    if(spindle_num == -1)
        spindle_num = 0;

    return spindle_num >= 0 && spindle_num < N_SYS_SPINDLE && sys_spindle[spindle_num].enabled;
}

/*! \brief Get the handlers (function pointers) etc. associated with an enabled spindle.
\param spindle_num spindle number as a \ref spindle_num_t.
\returns pointer to a \ref spindle_ptrs_t structure if successful, \a NULL if not.

__NOTE:__ do not modify the returned structure!
*/
spindle_ptrs_t *spindle_get (spindle_num_t spindle_num)
{
    return spindle_num >= 0 && spindle_num < N_SYS_SPINDLE && sys_spindle[spindle_num].enabled ? &sys_spindle[spindle_num].hal : NULL;
}

//

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

/*! \brief Register a null spindle that has no connection to the outside world.
This is done automatically on startup if no spindle can be succesfully enabled.
\returns assigned spindle id as a \ref spindle_id_t if successful, \a -1 if not.
*/
spindle_id_t spindle_add_null (void)
{
    static const spindle_ptrs_t spindle = {
        .type = SpindleType_Null,
        .cap.variable = Off,
        .cap.at_speed = Off,
        .cap.direction = Off,
        .set_state = null_set_state,
        .get_state = null_get_state,
        .get_pwm = null_get_pwm,
        .update_pwm = null_update_pwm,
        .update_rpm = null_update_rpm
    };

    bool registered = false;
    uint_fast8_t idx = n_spindle;

    if(idx) do {
        if((registered = spindles[--idx].hal.type == SpindleType_Null))
            break;
    } while(idx);

    if(!registered)
        return spindle_register(&spindle, "NULL");

    return idx;
}

// End null (dummy) spindle.

/*! \brief Set spindle speed override.
\param spindle pointer to a \ref spindle_ptrs_t structure.
\param speed_override override as a percentage of the programmed RPM.

__NOTE:__ Unlike motion overrides, spindle overrides do not require a planner reinitialization.
*/
void spindle_set_override (spindle_ptrs_t *spindle, override_t speed_override)
{
//    if(speed_override != 100 && sys.override.control.spindle_rpm_disable)
//        return;

    if(speed_override != 100 && spindle->param->state.override_disable)
        return;

    speed_override = constrain(speed_override, MIN_SPINDLE_RPM_OVERRIDE, MAX_SPINDLE_RPM_OVERRIDE);

    if ((uint8_t)speed_override != spindle->param->override_pct) {

        spindle->param->override_pct = speed_override;

        if(state_get() == STATE_IDLE)
            spindle_set_state(spindle, gc_state.modal.spindle.state, gc_state.spindle.rpm);
        else
            sys.step_control.update_spindle_rpm = On;

        system_add_rt_report(Report_Overrides); // Set to report change immediately

       if(grbl.on_spindle_programmed)
           grbl.on_spindle_programmed(spindle, gc_state.modal.spindle.state, spindle_set_rpm(spindle, gc_state.spindle.rpm, speed_override), gc_state.modal.spindle.rpm_mode);

       if(grbl.on_override_changed)
           grbl.on_override_changed(OverrideChanged_SpindleRPM);
    }
}

/*! \internal \brief Immediately sets spindle running state with direction and spindle rpm, if enabled.
Called by g-code parser spindle_sync(), parking retract and restore, g-code program end,
sleep, and spindle stop override.
\param spindle pointer to a \ref spindle_ptrs_t structure.
\param state a \ref spindle_state_t structure.
\param rpm the spindle RPM to set.
\returns \a true if successful, \a false if the current controller state is \ref ABORTED.
*/
static bool set_state (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    if (!ABORTED) { // Block during abort.

        if (!state.on) { // Halt or set spindle direction and rpm.
            spindle->param->rpm = rpm = 0.0f;
            spindle->set_state((spindle_state_t){0}, 0.0f);
        } else {
            // NOTE: Assumes all calls to this function is when Grbl is not moving or must remain off.
            // TODO: alarm/interlock if going from CW to CCW directly in non-laser mode?
            if (spindle->cap.laser && state.ccw)
                rpm = 0.0f; // TODO: May need to be rpm_min*(100/MAX_SPINDLE_RPM_OVERRIDE);

            spindle->set_state(state, spindle_set_rpm(spindle, rpm, spindle->param->override_pct));
        }

        system_add_rt_report(Report_Spindle); // Set to report change immediately

        st_rpm_changed(rpm);
    }

    return !ABORTED;
}


/*! \brief Immediately sets spindle running state with direction and spindle rpm, if enabled.
Called by g-code parser spindle_sync(), parking retract and restore, g-code program end,
sleep, and spindle stop override.
\param spindle pointer to a \ref spindle_ptrs_t structure.
\param state a \ref spindle_state_t structure.
\param rpm the spindle RPM to set.
\returns \a true if successful, \a false if the current controller state is \ref ABORTED.
*/
bool spindle_set_state (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    return set_state(spindle, state, rpm);
}

/*! \brief G-code parser entry-point for setting spindle state. Forces a planner buffer sync and bails
if an abort or check-mode is active. If the spindle supports at speed functionality it will wait
for it to reach the speed and raise an alarm if the speed is not reached within the timeout period.
\param spindle pointer to a \ref spindle_ptrs_t structure.
\param state a \ref spindle_state_t structure.
\param rpm the spindle RPM to set.
\returns \a true if successful, \a false if the current controller state is \ref ABORTED.
*/
bool spindle_sync (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    bool ok;

    if (!(ok = state_get() == STATE_CHECK_MODE)) {

        bool at_speed = !state.on || !spindle->cap.at_speed || settings.spindle.at_speed_tolerance <= 0.0f;

        // Empty planner buffer to ensure spindle is set when programmed.
        if((ok = protocol_buffer_synchronize()) && set_state(spindle, state, rpm) && !at_speed) {
            float on_delay = 0.0f;
            while(!(at_speed = spindle->get_state().at_speed)) {
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

/*! \brief Restore spindle running state with direction, enable, spindle RPM and appropriate delay.
\param spindle pointer to a \ref spindle_ptrs_t structure.
\param state a \ref spindle_state_t structure.
\param rpm the spindle RPM to set.
\returns \a true if successful, \a false if the current controller state is \ref ABORTED.
*/
bool spindle_restore (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    bool ok = true;

    if(spindle->cap.laser) // When in laser mode, ignore spindle spin-up delay. Set to turn on laser when cycle starts.
        sys.step_control.update_spindle_rpm = On;
    else { // TODO: add check for current spindle state matches restore state?
        spindle_set_state(spindle, state, rpm);
        if(state.on) {
            if((ok = !spindle->cap.at_speed))
                delay_sec(settings.safety_door.spindle_on_delay, DelayMode_SysSuspend);
            else if((ok == (settings.spindle.at_speed_tolerance <= 0.0f))) {
                float delay = 0.0f;
                while(!(ok = spindle->get_state().at_speed)) {
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

/*! \brief Calculate and set programmed RPM according to override and max/min limits
\param spindle pointer to a \ref spindle_ptrs_t structure.
\param rpm the programmed RPM.
\param override_pct override value in percent.
\returns the calulated RPM.
*/
float spindle_set_rpm (spindle_ptrs_t *spindle, float rpm, override_t override_pct)
{
    if(override_pct != 100)
        rpm *= 0.01f * (float)override_pct; // Scale RPM by override value.

    // Apply RPM limits
    if (rpm <= 0.0f) // TODO: remove this test?
        rpm = 0.0f;
    else if (rpm > spindle->rpm_max)
        rpm = spindle->rpm_max;
    else if (rpm < spindle->rpm_min)
        rpm = spindle->rpm_min;

    spindle->param->rpm_overridden = rpm;
    spindle->param->override_pct = override_pct;

    return rpm;
}

/*! \brief Turn off all enabled spindles.
*/
void spindle_all_off (void)
{
    spindle_ptrs_t *spindle;
    uint_fast8_t spindle_num = N_SYS_SPINDLE;
    do {
        if((spindle = spindle_get(--spindle_num))) {
            spindle->param->rpm = spindle->param->rpm_overridden = 0.0f;
            spindle->param->state.value = 0;
#ifdef GRBL_ESP32
            spindle->esp32_off();
#else
            spindle->set_state((spindle_state_t){0}, 0.0f);
#endif
        }
    } while(spindle_num);
}

/*! \brief Check if any of the enabled spindles is running.
\returns \a true if a spindle is running, \a false otherwise.
*/
bool spindle_is_on (void)
{
    bool on = false;

    spindle_ptrs_t *spindle;
    uint_fast8_t spindle_num = N_SYS_SPINDLE;
    do {
        if((spindle = spindle_get(--spindle_num)))
            on = spindle->get_state().on;
    } while(spindle_num && !on);

    return on;
}

//
// The following functions are not called by the core, may be called by driver code.
//

/*! \brief calculate inverted pwm value if configured
\param pwm_data pointer t a \a spindle_pwm_t structure.
\param pwm_value non inverted PWM value.
\returns the inverted PWM value to use.
*/
static inline uint_fast16_t invert_pwm (spindle_pwm_t *pwm_data, uint_fast16_t pwm_value)
{
    return pwm_data->invert_pwm ? pwm_data->period - pwm_value - 1 : pwm_value;
}

/*! \brief Precompute PWM values for faster conversion.
\param spindle pointer to a \ref spindle_ptrs_t structure.
\param pwm_data pointer to a \a spindle_pwm_t structure, to hold the precomputed values.
\param clock_hz timer clock frequency used for PWM generation.
\returns \a true if successful, \a false if no PWM range possible - driver should then revert to simple on/off spindle control.
*/
bool spindle_precompute_pwm_values (spindle_ptrs_t *spindle, spindle_pwm_t *pwm_data, uint32_t clock_hz)
{
    if(spindle->rpm_max > spindle->rpm_min) {
        pwm_data->rpm_min = spindle->rpm_min;
        pwm_data->period = (uint_fast16_t)((float)clock_hz / settings.spindle.pwm_freq);
        if(settings.spindle.pwm_off_value == 0.0f)
            pwm_data->off_value = pwm_data->invert_pwm ? pwm_data->period : 0;
        else
            pwm_data->off_value = invert_pwm(pwm_data, (uint_fast16_t)(pwm_data->period * settings.spindle.pwm_off_value / 100.0f));
        pwm_data->min_value = (uint_fast16_t)(pwm_data->period * settings.spindle.pwm_min_value / 100.0f);
        pwm_data->max_value = (uint_fast16_t)(pwm_data->period * settings.spindle.pwm_max_value / 100.0f) + pwm_data->offset;
        pwm_data->pwm_gradient = (float)(pwm_data->max_value - pwm_data->min_value) / (spindle->rpm_max - spindle->rpm_min);
        pwm_data->always_on = settings.spindle.pwm_off_value != 0.0f;
    }

#if ENABLE_SPINDLE_LINEARIZATION
    uint_fast8_t idx;

    pwm_data->n_pieces = 0;

    for(idx = 0; idx < SPINDLE_NPWM_PIECES; idx++) {
        if(!isnan(settings.spindle.pwm_piece[idx].rpm) && settings.spindle.pwm_piece[idx].start != 0.0f)
            memcpy(&pwm_data->piece[pwm_data->n_pieces++], &settings.spindle.pwm_piece[idx], sizeof(pwm_piece_t));
    }

    spindle->cap.pwm_linearization = pwm_data->n_pieces > 0;
#endif

    return spindle->rpm_max > spindle->rpm_min;
}

/*! \brief Spindle RPM to PWM conversion.
\param pwm_data pointer t a \a spindle_pwm_t structure.
\param rpm spindle RPM.
\param pid_limit boolean, \a true if PID based spindle sync is used, \a false otherwise.
\returns the PWM value to use.

__NOTE:__ \a spindle_precompute_pwm_values() must be called to precompute values before this function is called.
Typically this is done by the spindle initialization code.
*/
uint_fast16_t spindle_compute_pwm_value (spindle_pwm_t *pwm_data, float rpm, bool pid_limit)
{
    uint_fast16_t pwm_value;

    if(rpm > pwm_data->rpm_min) {
      #if ENABLE_SPINDLE_LINEARIZATION
        // Compute intermediate PWM value with linear spindle speed model via piecewise linear fit model.
        uint_fast8_t idx = pwm_data->n_pieces;

        if(idx) {
            do {
                idx--;
                if(idx == 0 || rpm > pwm_data->piece[idx].rpm) {
                    pwm_value = floorf((pwm_data->piece[idx].start * rpm - pwm_data->piece[idx].end) * pwm_data->pwm_gradient);
                    break;
                }
            } while(idx);
        } else
      #endif
        // Compute intermediate PWM value with linear spindle speed model.
        pwm_value = (uint_fast16_t)floorf((rpm - pwm_data->rpm_min) * pwm_data->pwm_gradient) + pwm_data->min_value;

        if(pwm_value >= (pid_limit ? pwm_data->period : pwm_data->max_value))
            pwm_value = pid_limit ? pwm_data->period - 1 : pwm_data->max_value;
        else if(pwm_value < pwm_data->min_value)
            pwm_value = pwm_data->min_value;

        pwm_value = invert_pwm(pwm_data, pwm_value);
    } else
        pwm_value = rpm == 0.0f ? pwm_data->off_value : invert_pwm(pwm_data, pwm_data->min_value);

    return pwm_value;
}
