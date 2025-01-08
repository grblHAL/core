/*
  spindle_control.c - spindle control methods

  Part of grblHAL

  Copyright (c) 2017-2025 Terje Io
  Copyright (c) 2012-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud

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
static const spindle_data_ptrs_t *encoder;

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
            pwm_spindle->hal.rpm_min = settings.pwm_spindle.rpm_min;
            pwm_spindle->hal.rpm_max = settings.pwm_spindle.rpm_max;
        }

        if((pwm_spindle->init_ok = pwm_spindle->hal.config == NULL || pwm_spindle->hal.config(&pwm_spindle->hal)))
            pwm_spindle->hal.set_state(&pwm_spindle->hal, (spindle_state_t){0}, 0.0f);
    }
    pwm_spindle = NULL;

    if((ok = spindle_id >= 0 && spindle_id < n_spindle && !!spindles[spindle_id].cfg)) {

        spindle = &spindles[spindle_id];

        if(sys_spindle[spindle_num].enabled && sys_spindle[spindle_num].hal.id != spindle_id && sys_spindle[spindle_num].hal.set_state)
            gc_spindle_off(); // TODO: switch off only the default spindle?

        if(!spindle->hal.cap.rpm_range_locked) {
            spindle->hal.rpm_min = settings.pwm_spindle.rpm_min;
            spindle->hal.rpm_max = settings.pwm_spindle.rpm_max;
        }

        if(!spindle->init_ok)
            ok = spindle->init_ok = spindle->hal.config == NULL || spindle->hal.config(&spindle->hal);

        if(ok) {

            spindle_ptrs_t spindle_hal;

            memcpy(&spindle_hal, &spindle->hal, sizeof(spindle_ptrs_t));

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
#if N_SPINDLE > 1
                system_add_rt_report(Report_SpindleId);
#endif
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
    if(n_spindle == 1 && spindles[0].cfg->type == SpindleType_Null)
        n_spindle = 0;

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

    if(n_spindle) do {
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
    spindle->cap.laser = !!pwm_caps && !pwm_caps->flags.laser_mode_disable && !!spindle->update_pwm && settings.mode == Mode_Laser;
    spindle->pwm_off_value = pwm_caps ? pwm_caps->off_value : 0;

    do {
        idx--;
        if(sys_spindle[idx].enabled && spindle->id == sys_spindle[idx].hal.id) {
            sys_spindle[idx].hal.type = spindle->type;
            sys_spindle[idx].hal.cap.laser = spindle->cap.laser;
            sys_spindle[idx].hal.rpm_min =  spindle->rpm_min;
            sys_spindle[idx].hal.rpm_max =  spindle->rpm_max;
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

    return n_spindle == 1 && spindles[0].cfg->type == SpindleType_Null ? 0 : n_spindle;
}

bool spindle_get_id (uint8_t ref_id, spindle_id_t *spindle_id)
{
    bool ok = false;
    uint_fast8_t idx;

    *spindle_id = 0;

    for(idx = 0; idx < n_spindle; idx++) {
        if((ok = spindles[idx].cfg->ref_id == ref_id)) {
            *spindle_id = idx;
            break;
        }
    }

    return ok;
}

static spindle_num_t spindle_get_num (spindle_id_t spindle_id)
{
    spindle_num_t spindle_num;

    if((spindle_num = spindle_get_count() == 1 ? 0 : -1) == -1) {

        const setting_detail_t *setting;
        uint_fast8_t idx = N_SPINDLE_SELECTABLE;

        do {
            idx--;
            if((setting = setting_get_details(idx == 0 ? Setting_SpindleType : (setting_id_t)(Setting_SpindleEnable0 + idx), NULL))) {
                if(setting_get_int_value(setting, 0) - (idx == 0 ? 0 : 1) == spindle_id)
                    spindle_num = idx;
            }
        } while(idx && spindle_num == -1);
    }

    return spindle_num;
}

void spindle_bind_encoder (const spindle_data_ptrs_t *encoder_data)
{
    uint_fast8_t idx;
    spindle_ptrs_t *spindle;
    spindle_num_t spindle_num;

    encoder = encoder_data;

    for(idx = 0; idx < n_spindle; idx++) {

        spindle = spindle_get((spindle_num = spindle_get_num(idx)));

        if(encoder_data && spindle_num == settings.spindle.encoder_spindle) {
            spindles[idx].hal.get_data = encoder_data->get;
            spindles[idx].hal.reset_data = encoder_data->reset;
            spindles[idx].hal.cap.at_speed = spindles[idx].hal.cap.variable;
        } else {
            spindles[idx].hal.get_data = spindles[idx].cfg->get_data;
            spindles[idx].hal.reset_data = spindles[idx].cfg->reset_data;
            spindles[idx].hal.cap.at_speed = spindles[idx].cfg->cap.at_speed;
        }

        if(spindle) {
            spindle->get_data = spindles[idx].hal.get_data;
            spindle->reset_data = spindles[idx].hal.reset_data;
            spindle->cap.at_speed = spindles[idx].hal.cap.at_speed;
        }
    }
}

bool spindle_set_at_speed_range (spindle_ptrs_t *spindle, spindle_data_t *spindle_data, float rpm)
{
    spindle_data->rpm_programmed = rpm;
    spindle_data->state_programmed.at_speed = false;

    if((spindle_data->at_speed_enabled = spindle->at_speed_tolerance > 0.0f)) {
        spindle_data->rpm_low_limit = rpm * (1.0f - (spindle->at_speed_tolerance / 100.0f));
        spindle_data->rpm_high_limit = rpm * (1.0f + (spindle->at_speed_tolerance / 100.0f));
    }

    return spindle_data->at_speed_enabled;
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
        spindle.ref_id = spindles[idx].cfg->ref_id;
        spindle.name = spindles[idx].name;
        spindle.num = spindle_get_num(idx);
        spindle.enabled = spindle.num != -1;
        spindle.hal = spindle.enabled && sys_spindle[spindle.num].hal.id == spindle.id ? &sys_spindle[spindle.num].hal : &spindles[idx].hal;
        spindle.is_current = spindle.enabled && sys_spindle[0].hal.id == idx;

        if(callback(&spindle, data))
            return true;
    }

    return false;
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

static void null_set_state (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    UNUSED(spindle);
    UNUSED(state);
    UNUSED(rpm);
}

static spindle_state_t null_get_state (spindle_ptrs_t *spindle)
{
    UNUSED(spindle);

    return (spindle_state_t){0};
}

// Sets spindle speed
static void null_update_pwm (spindle_ptrs_t *spindle, uint_fast16_t pwm_value)
{
    UNUSED(spindle);
    UNUSED(pwm_value);
}

static uint_fast16_t null_get_pwm (spindle_ptrs_t *spindle, float rpm)
{
    UNUSED(spindle);
    UNUSED(rpm);

    return 0;
}

static void null_update_rpm (spindle_ptrs_t *spindle, float rpm)
{
    UNUSED(spindle);
    UNUSED(rpm);
}

#ifdef GRBL_ESP32
static void null_esp32_off (spindle_ptrs_t *spindle)
{
    UNUSED(spindle);
}
#endif

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
#ifdef GRBL_ESP32
        .esp32_off = null_esp32_off,
#endif
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

    if((uint8_t)speed_override != spindle->param->override_pct) {

        spindle_set_rpm(spindle, spindle->param->rpm, speed_override);

        if(state_get() == STATE_IDLE) {
            if(spindle->get_pwm && spindle->update_pwm)
                spindle->update_pwm(spindle, spindle->get_pwm(spindle, spindle->param->rpm_overridden));
            else if(spindle->update_rpm)
                spindle->update_rpm(spindle, spindle->param->rpm_overridden);
        } else
            sys.step_control.update_spindle_rpm = On;

        system_add_rt_report(Report_Overrides); // Set to report change immediately

//       if(grbl.on_spindle_programmed)
//           grbl.on_spindle_programmed(spindle, spindle->param->state, spindle->param->rpm, spindle->param->rpm_mode);

       if(grbl.on_override_changed)
           grbl.on_override_changed(OverrideChanged_SpindleRPM);
    }
}

/*! \internal \brief Immediately sets spindle running state with direction and spindle rpm, if enabled.
Called by g-code parser spindle_set_state_synced(), parking retract and restore, g-code program end,
sleep, and spindle stop override.
\param spindle pointer to a \ref spindle_ptrs_t structure.
\param state a \ref spindle_state_t structure.
\param rpm the spindle RPM to set.
\returns \a true if successful, \a false if the current controller state is \ref ABORTED.
*/
bool spindle_set_state (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    if (!ABORTED) { // Block during abort.

        if (!state.on) { // Halt or set spindle direction and rpm.
            rpm = 0.0f;
            spindle->set_state(spindle, (spindle_state_t){0}, 0.0f);
        } else {
            // NOTE: Assumes all calls to this function is when grblHAL is not moving or must remain off.
            // TODO: alarm/interlock if going from CW to CCW directly in non-laser mode?
            if (spindle->cap.laser && state.ccw)
                rpm = 0.0f; // TODO: May need to be rpm_min*(100/MAX_SPINDLE_RPM_OVERRIDE);

            spindle->set_state(spindle, state, spindle_set_rpm(spindle, rpm, spindle->param->override_pct));
        }

        spindle->param->rpm = rpm;
        spindle->param->state = state;

        system_add_rt_report(Report_Spindle); // Set to report change immediately

        st_rpm_changed(rpm);
    }

    return !ABORTED;
}

/*! \brief If the spindle supports at speed functionality it will wait
for it to reach the speed and raise an alarm if the speed is not reached within the timeout period.
\param spindle pointer to a \ref spindle_ptrs_t structure.
\param state a \ref spindle_state_t structure.
\param rpm the spindle RPM to set.
\returns \a true if successful, \a false if the current controller state is \ref ABORTED.
*/
static bool spindle_set_state_wait (spindle_ptrs_t *spindle, spindle_state_t state, float rpm, uint16_t on_delay_ms, delaymode_t delay_mode)
{
    bool ok;

    if(!(ok = state_get() == STATE_CHECK_MODE)) {

        if((ok = spindle_set_state(spindle, state, rpm))) {

            bool at_speed = !state.on || !spindle->cap.at_speed || spindle->at_speed_tolerance <= 0.0f;

            if(at_speed)
                ok = on_delay_ms == 0 || delay_sec((float)on_delay_ms / 1000.0f, delay_mode);
            else {
                uint16_t delay = 0;
                if(on_delay_ms == 0)
                    on_delay_ms = 60000; // one minute...
                while(!(at_speed = spindle->get_state(spindle).at_speed)) {
                    if(!delay_sec(0.2f, delay_mode))
                        break;
                    delay += 200;
                    if(delay > on_delay_ms) {
                        gc_spindle_off();
                        system_raise_alarm(Alarm_Spindle);
                        break;
                    }
                }
                ok &= at_speed;
            }
        }
    }

    return ok;
}

/*! \brief G-code parser entry-point for setting spindle state. Forces a planner buffer sync and bails
if an abort or check-mode is active. If the spindle supports at speed functionality it will wait
for it to reach the speed and raise an alarm if the speed is not reached within the timeout period.
\param spindle pointer to a \ref spindle_ptrs_t structure.
\param state a \ref spindle_state_t structure.
\param rpm the spindle RPM to set.
\returns \a true if successful, \a false if the current controller state is \ref ABORTED.
*/
bool spindle_set_state_synced (spindle_ptrs_t *spindle, spindle_state_t state, float rpm)
{
    // Empty planner buffer to ensure spindle is set when programmed.
    return protocol_buffer_synchronize() && spindle_set_state_wait(spindle, state, rpm, state.on ? settings.spindle.on_delay : settings.spindle.off_delay, DelayMode_Dwell);
}

/*! \brief Restore spindle running state with direction, enable, spindle RPM and appropriate delay.
\param spindle pointer to a \ref spindle_ptrs_t structure.
\param state a \ref spindle_state_t structure.
\param rpm the spindle RPM to set.
\returns \a true if successful, \a false if the current controller state is \ref ABORTED.
*/
bool spindle_restore (spindle_ptrs_t *spindle, spindle_state_t state, float rpm, uint16_t on_delay_ms)
{
    bool ok;

    if((ok = spindle->cap.laser)) // When in laser mode, ignore spindle spin-up delay. Set to turn on laser when cycle starts.
        sys.step_control.update_spindle_rpm = On;
    else if(!(ok = state.value == spindle->get_state(spindle).value))
        ok = spindle_set_state_wait(spindle, state, rpm, on_delay_ms, DelayMode_SysSuspend);

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

    rpm = rpm <= 0.0f ? 0.0f : constrain(rpm, spindle->rpm_min, spindle->rpm_max);

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
            spindle->esp32_off(spindle);
#else
            spindle->set_state(spindle, (spindle_state_t){0}, 0.0f);
#endif
        }
    } while(spindle_num);

    system_add_rt_report(Report_Spindle);
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
            on = spindle->get_state(spindle).on;
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
    return pwm_data->flags.invert_pwm ? pwm_data->period - pwm_value - 1 : pwm_value;
}

/*! \brief Spindle RPM to PWM conversion.
\param pwm_data pointer t a \a spindle_pwm_t structure.
\param rpm spindle RPM.
\param pid_limit boolean, \a true if PID based spindle sync is used, \a false otherwise.
\returns the PWM value to use.

__NOTE:__ \a spindle_precompute_pwm_values() must be called to precompute values before this function is called.
Typically this is done by the spindle initialization code.
*/
static uint_fast16_t spindle_compute_pwm_value (spindle_pwm_t *pwm_data, float rpm, bool pid_limit)
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

/*! \internal \brief Dummy spindle RPM to PWM conversion, used if precompute fails.
\param pwm_data pointer t a \a spindle_pwm_t structure.
\param rpm spindle RPM.
\param pid_limit boolean, \a true if PID based spindle sync is used, \a false otherwise.
\returns the PWM value to use.
*/
static uint_fast16_t compute_dummy_pwm_value (spindle_pwm_t *pwm_data, float rpm, bool pid_limit)
{
    return pwm_data->off_value;
}

/*! \brief Precompute PWM values for faster conversion.
\param spindle pointer to a \ref spindle_ptrs_t structure.
\param pwm_data pointer to a \a spindle_pwm_t structure, to hold the precomputed values.
\param clock_hz timer clock frequency used for PWM generation.
\returns \a true if successful, \a false if no PWM range possible - driver should then revert to simple on/off spindle control.
*/
bool spindle_precompute_pwm_values (spindle_ptrs_t *spindle, spindle_pwm_t *pwm_data, spindle_pwm_settings_t *settings, uint32_t clock_hz)
{
    pwm_data->settings = settings;
    spindle->rpm_min = pwm_data->rpm_min = settings->rpm_min;
    spindle->rpm_max = settings->rpm_max;
    spindle->at_speed_tolerance = settings->at_speed_tolerance;
    spindle->cap.rpm_range_locked = On;

    if((spindle->cap.variable = !settings->flags.pwm_disable && spindle->rpm_max > spindle->rpm_min)) {
        pwm_data->f_clock = clock_hz;
        pwm_data->period = (uint_fast16_t)((float)clock_hz / settings->pwm_freq);
        if(settings->pwm_off_value == 0.0f)
            pwm_data->off_value = pwm_data->flags.invert_pwm ? pwm_data->period : 0;
        else
            pwm_data->off_value = invert_pwm(pwm_data, (uint_fast16_t)(pwm_data->period * settings->pwm_off_value / 100.0f));
        pwm_data->max_value = (uint_fast16_t)(pwm_data->period * settings->pwm_max_value / 100.0f) + pwm_data->offset;
        if((pwm_data->min_value = (uint_fast16_t)(pwm_data->period * settings->pwm_min_value / 100.0f)) == 0 && spindle->rpm_min > 0.0f)
            pwm_data->min_value = (uint_fast16_t)((float)pwm_data->max_value * 0.004f);

        pwm_data->pwm_gradient = (float)(pwm_data->max_value - pwm_data->min_value) / (spindle->rpm_max - spindle->rpm_min);
        pwm_data->flags.always_on = settings->pwm_off_value != 0.0f;
        pwm_data->compute_value = spindle_compute_pwm_value;
    } else {
        pwm_data->off_value = 0;
        pwm_data->flags.always_on = false;
        pwm_data->compute_value = compute_dummy_pwm_value;
    }

    spindle->context.pwm = pwm_data;

#if ENABLE_SPINDLE_LINEARIZATION
    uint_fast8_t idx;

    pwm_data->n_pieces = 0;

    for(idx = 0; idx < SPINDLE_NPWM_PIECES; idx++) {
        if(!isnan(settings->pwm_piece[idx].rpm) && settings->pwm_piece[idx].start != 0.0f)
            memcpy(&pwm_data->piece[pwm_data->n_pieces++], &settings->pwm_piece[idx], sizeof(pwm_piece_t));
    }

    spindle->cap.pwm_linearization = pwm_data->n_pieces > 0;
#endif

    return spindle->cap.variable;
}

#if N_SPINDLE > 1

#include "grbl/nvs_buffer.h"

static spindle1_pwm_settings_t sp1_settings;
static uint32_t nvs_address;
static char spindle_signals[] = "Spindle enable,Spindle direction,PWM";
static bool ports_ok = false;
static char max_aport[4], max_dport[4];
static spindle_cap_t spindle_cap;
static spindle1_settings_changed_ptr on_settings_changed;

#if ENABLE_SPINDLE_LINEARIZATION

static status_code_t set_linear_piece (setting_id_t id, char *svalue)
{
    uint32_t idx = id - Setting_LinearSpindle1Piece1;
    float rpm, start, end;

    if(*svalue == '\0' || (svalue[0] == '0' && svalue[1] == '\0')) {
        sp1_settings.cfg.pwm_piece[idx].rpm = NAN;
        sp1_settings.cfg.pwm_piece[idx].start =
        sp1_settings.cfg.pwm_piece[idx].end = 0.0f;
    } else if(sscanf(svalue, "%f,%f,%f", &rpm, &start, &end) == 3) {
        sp1_settings.cfg.pwm_piece[idx].rpm = rpm;
        sp1_settings.cfg.pwm_piece[idx].start = start;
        sp1_settings.cfg.pwm_piece[idx].end = end;
//??       if(idx == 0)
//            sp1_settings.cfg.rpm_min = rpm;
    } else
        return Status_SettingValueOutOfRange;

    return Status_OK;
}

static char *get_linear_piece (setting_id_t id)
{
    static char buf[40];

    uint32_t idx = id - Setting_LinearSpindle1Piece1;

    if(isnan(sp1_settings.cfg.pwm_piece[idx].rpm))
        *buf = '\0';
    else
        snprintf(buf, sizeof(buf), "%g,%g,%g", sp1_settings.cfg.pwm_piece[idx].rpm, sp1_settings.cfg.pwm_piece[idx].start, sp1_settings.cfg.pwm_piece[idx].end);

    return buf;
}

#endif

static status_code_t set_spindle_invert (setting_id_t id, uint_fast16_t int_value)
{
    sp1_settings.cfg.invert.mask = int_value;
    if(sp1_settings.cfg.invert.pwm && !spindle_cap.pwm_invert) {
        sp1_settings.cfg.invert.pwm = Off;
        return Status_SettingDisabled;
    }

    return Status_OK;
}

static uint32_t get_int (setting_id_t id)
{
    uint32_t value = 0;

    switch(id) {

        case Setting_SpindleInvertMask1:
            value = sp1_settings.cfg.invert.value;
            break;

        default:
            break;
    }

    return value;
}

static status_code_t set_dir_port (setting_id_t id, float value)
{
    sp1_settings.port_dir = value < 0.0f ? 255 : (int8_t)value;

    return Status_OK;
}

static float get_dir_port (setting_id_t id)
{
    return sp1_settings.port_dir == 255 ? -1.0f : (float)sp1_settings.port_dir;
}

static uint32_t get_pwm_port (setting_id_t id)
{
    return (uint32_t)sp1_settings.port_pwm;
}

bool pwm_port_validate (xbar_t *properties, uint8_t port, void *data)
{
    return port == (uint8_t)((uint32_t)data);
}

static status_code_t set_pwm_port (setting_id_t id, uint_fast16_t int_value)
{
    bool ok;

    if((ok = (uint8_t)int_value == sp1_settings.port_pwm ||
              ioports_enumerate(Port_Analog, Port_Output, (pin_cap_t){ .pwm = On, .claimable = On }, pwm_port_validate, (void *)((uint32_t)int_value))))
        sp1_settings.port_pwm = (uint8_t)int_value;

    return ok ? Status_OK : Status_SettingValueOutOfRange;
}

static bool has_pwm (const setting_detail_t *setting)
{
    return spindle_cap.variable;
}

static bool has_freq (const setting_detail_t *setting)
{
    return spindle_cap.variable && !spindle_cap.cloned;
}

static bool has_ports (const setting_detail_t *setting)
{
    return ports_ok;
}

static const setting_detail_t spindle1_settings[] = {
    { Setting_Spindle_OnPort, Group_AuxPorts, "PWM2 spindle on port", NULL, Format_Int8, "##0", "0", max_dport, Setting_NonCore, &sp1_settings.port_on, NULL, has_ports, { .reboot_required = On } },
    { Setting_Spindle_DirPort, Group_AuxPorts, "PWM2 spindle direction port", NULL, Format_Decimal, "-#0", "-1", max_dport, Setting_NonCoreFn, set_dir_port, get_dir_port, has_ports, { .reboot_required = On } },
    { Setting_SpindleInvertMask1, Group_Spindle, "Invert PWM2 spindle signals", NULL, Format_Bitfield, spindle_signals, NULL, NULL, Setting_IsExtendedFn, set_spindle_invert, get_int, NULL, { .reboot_required = On } },
    { Setting_Spindle_PWMPort, Group_AuxPorts, "PWM2 spindle PWM port", NULL, Format_Int8, "#0", "0", max_aport, Setting_NonCoreFn, set_pwm_port, get_pwm_port, has_ports, { .reboot_required = On } },
    { Setting_RpmMax1, Group_Spindle, "Maximum PWM2 spindle speed", "RPM", Format_Decimal, "#####0.000", NULL, NULL, Setting_IsLegacy, &sp1_settings.cfg.rpm_max, NULL, has_pwm },
    { Setting_RpmMin1, Group_Spindle, "Minimum PWM2 spindle speed", "RPM", Format_Decimal, "#####0.000", NULL, NULL, Setting_IsLegacy, &sp1_settings.cfg.rpm_min, NULL, has_pwm },
    { Setting_PWMFreq1, Group_Spindle, "PWM2 spindle PWM frequency", "Hz", Format_Decimal, "#####0", NULL, NULL, Setting_IsExtended, &sp1_settings.cfg.pwm_freq, NULL, has_freq },
    { Setting_PWMOffValue1, Group_Spindle, "PWM2 spindle PWM off value", "percent", Format_Decimal, "##0.0", NULL, "100", Setting_IsExtended, &sp1_settings.cfg.pwm_off_value, NULL, has_pwm },
    { Setting_PWMMinValue1, Group_Spindle, "PWM2 spindle PWM min value", "percent", Format_Decimal, "##0.0", NULL, "100", Setting_IsExtended, &sp1_settings.cfg.pwm_min_value, NULL, has_pwm },
    { Setting_PWMMaxValue1, Group_Spindle, "PWM2 spindle PWM max value", "percent", Format_Decimal, "##0.0", NULL, "100", Setting_IsExtended, &sp1_settings.cfg.pwm_max_value, NULL, has_pwm }
#if xENABLE_SPINDLE_LINEARIZATION
     { Setting_LinearSpindle1Piece1, Group_Spindle, "PWM2 spindle linearisation, 1st point", NULL, Format_String, "x(39)", NULL, "39", Setting_IsExtendedFn, set_linear_piece, get_linear_piece, NULL },
  #if SPINDLE_NPWM_PIECES > 1
     { Setting_LinearSpindle1Piece2, Group_Spindle, "PWM2 spindle linearisation, 2nd point", NULL, Format_String, "x(39)", NULL, "39", Setting_IsExtendedFn, set_linear_piece, get_linear_piece, NULL },
  #endif
  #if SPINDLE_NPWM_PIECES > 2
     { Setting_LinearSpindle1Piece3, Group_Spindle, "PWM2 spindle linearisation, 3rd point", NULL, Format_String, "x(39)", NULL, "39", Setting_IsExtendedFn, set_linear_piece, get_linear_piece, NULL },
  #endif
  #if SPINDLE_NPWM_PIECES > 3
     { Setting_LinearSpindle1Piece4, Group_Spindle, "PWM2 spindle linearisation, 4th point", NULL, Format_String, "x(39)", NULL, "39", Setting_IsExtendedFn, set_linear_piece, get_linear_piece, NULL },
  #endif
#endif
};

#ifndef NO_SETTINGS_DESCRIPTIONS
static const setting_descr_t spindle1_settings_descr[] = {
    { Setting_Spindle_OnPort, "On/off aux port." },
    { Setting_Spindle_DirPort, "Direction aux port, set to -1 if not required." },
    { Setting_SpindleInvertMask1, "Inverts the spindle on, counterclockwise and PWM signals (active low)." },
    { Setting_Spindle_PWMPort, "Spindle analog aux port. Must be PWM capable!" },
    { Setting_RpmMax1, "Maximum spindle speed." },
    { Setting_RpmMin1, "Minimum spindle speed." },
    { Setting_PWMFreq1, "PWM frequency." },
    { Setting_PWMOffValue1, "PWM off value in percent (duty cycle)." },
    { Setting_PWMMinValue1, "PWM min value in percent (duty cycle)." },
    { Setting_PWMMaxValue1, "PWM max value in percent (duty cycle)." }
#if xENABLE_SPINDLE_LINEARIZATION
     { Setting_LinearSpindle1Piece1, "Comma separated list of values: RPM_MIN, RPM_LINE_A1, RPM_LINE_B1, set to blank to disable." },
  #if SPINDLE_NPWM_PIECES > 1
     { Setting_LinearSpindle1Piece2, "Comma separated list of values: RPM_POINT12, RPM_LINE_A2, RPM_LINE_B2, set to blank to disable." },
  #endif
  #if SPINDLE_NPWM_PIECES > 2
     { Setting_LinearSpindle1Piece3, "Comma separated list of values: RPM_POINT23, RPM_LINE_A3, RPM_LINE_B3, set to blank to disable." },
  #endif
  #if SPINDLE_NPWM_PIECES > 3
     { Setting_LinearSpindle1Piece4, "Comma separated list of values: RPM_POINT34, RPM_LINE_A4, RPM_LINE_B4, set to blank to disable." },
  #endif
#endif
};
#endif

static void spindle1_settings_changed (settings_t *settings, settings_changed_flags_t changed)
{
    UNUSED(changed);

    if(on_settings_changed) {
        changed.spindle = On;
        on_settings_changed(&sp1_settings);
    }
}

static void spindle1_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&sp1_settings, sizeof(spindle1_pwm_settings_t), true);
}

static void spindle1_settings_restore (void)
{
    static const spindle_pwm_settings_t defaults = {
        .rpm_max = DEFAULT_SPINDLE1_RPM_MAX,
        .rpm_min = DEFAULT_SPINDLE1_RPM_MIN,
        .flags.pwm_disable = false,
        .flags.enable_rpm_controlled = 0, //DEFAULT_SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED, TODO: add setting?
        .flags.laser_mode_disable = 0, // TODO: add setting? Not possible?
        .invert.on = DEFAULT_INVERT_SPINDLE1_ENABLE_PIN,
        .invert.ccw = DEFAULT_INVERT_SPINDLE1_CCW_PIN,
        .invert.pwm = DEFAULT_INVERT_SPINDLE1_PWM_PIN,
        .pwm_freq = DEFAULT_SPINDLE1_PWM_FREQ,
        .pwm_off_value = DEFAULT_SPINDLE1_PWM_OFF_VALUE,
        .pwm_min_value = DEFAULT_SPINDLE1_PWM_MIN_VALUE,
        .pwm_max_value = DEFAULT_SPINDLE1_PWM_MAX_VALUE,
        .at_speed_tolerance = DEFAULT_SPINDLE_AT_SPEED_TOLERANCE,
#if ENABLE_SPINDLE_LINEARIZATION
  #if SPINDLE_NPWM_PIECES > 0
        .pwm_piece[0] = { .rpm = DEFAULT_RPM_POINT01, .start = DEFAULT_RPM_LINE_A1, .end = DEFAULT_RPM_LINE_B1 },
  #endif
  #if SPINDLE_NPWM_PIECES > 1
        .pwm_piece[1] = { .rpm = DEFAULT_RPM_POINT12, .start = DEFAULT_RPM_LINE_A2, .end = DEFAULT_RPM_LINE_B2 },
  #endif
  #if SPINDLE_NPWM_PIECES > 2
        .pwm_piece[2] = { .rpm = DEFAULT_RPM_POINT23, .start = DEFAULT_RPM_LINE_A3, .end = DEFAULT_RPM_LINE_B3 },
  #endif
  #if SPINDLE_NPWM_PIECES > 3
        .pwm_piece[3] = { .rpm = DEFAULT_RPM_POINT34, .start = DEFAULT_RPM_LINE_A4, .end = DEFAULT_RPM_LINE_B4 },
  #endif
#else
  #if SPINDLE_NPWM_PIECES > 0
        .pwm_piece[0] = { .rpm = NAN, .start = 0.0f, .end = 0.0f },
  #endif
  #if SPINDLE_NPWM_PIECES > 1
        .pwm_piece[1] = { .rpm = NAN, .start = 0.0f, .end = 0.0f },
  #endif
  #if SPINDLE_NPWM_PIECES > 2
        .pwm_piece[2] = { .rpm = NAN, .start = 0.0f, .end = 0.0f },
  #endif
  #if SPINDLE_NPWM_PIECES > 3
        .pwm_piece[3] = { .rpm = NAN, .start = 0.0f, .end = 0.0f },
  #endif
#endif
    };

    memcpy(&sp1_settings.cfg, &defaults, sizeof(spindle_pwm_settings_t));

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&sp1_settings, sizeof(spindle1_pwm_settings_t), true);
}

static void spindle1_settings_load (void)
{
    if((hal.nvs.memcpy_from_nvs((uint8_t *)&sp1_settings, nvs_address, sizeof(spindle1_pwm_settings_t), true) != NVS_TransferResult_OK))
        spindle1_settings_restore();
}

static bool pwm_count (xbar_t *properties, uint8_t port, void *data)
{
    *((uint32_t *)data) += 1;

    sp1_settings.port_pwm = max(sp1_settings.port_pwm, port);

    return false;
}

static bool check_pwm_ports (void)
{
    uint32_t n_pwm_out = 0;

    ioports_enumerate(Port_Analog, Port_Output, (pin_cap_t){ .pwm = On, .claimable = On }, pwm_count, &n_pwm_out);

    return n_pwm_out != 0;
}

spindle1_pwm_settings_t *spindle1_settings_add (bool claim_ports)
{
    if((ports_ok = claim_ports && hal.port.num_digital_out > 0 && check_pwm_ports())) {

        sp1_settings.port_on = hal.port.num_digital_out - 1;
        sp1_settings.port_dir = hal.port.num_digital_out > 1 ? hal.port.num_digital_out - 2 : 255;

        strcpy(max_aport, uitoa(sp1_settings.port_pwm));
        strcpy(max_dport, uitoa(hal.port.num_digital_out - 1));
    }

    return nvs_address == 0 && (!claim_ports || ports_ok) && (nvs_address = nvs_alloc(sizeof(spindle1_pwm_settings_t))) ? &sp1_settings : NULL;
}

void spindle1_settings_register (spindle_cap_t cap, spindle1_settings_changed_ptr on_changed)
{
    static setting_details_t spindle1_setting_details = {
        .settings = spindle1_settings,
        .n_settings = sizeof(spindle1_settings) / sizeof(setting_detail_t),
    #ifndef NO_SETTINGS_DESCRIPTIONS
        .descriptions = spindle1_settings_descr,
        .n_descriptions = sizeof(spindle1_settings_descr) / sizeof(setting_descr_t),
    #endif
        .load = spindle1_settings_load,
        .restore = spindle1_settings_restore,
        .save = spindle1_settings_save,
        .on_changed = spindle1_settings_changed
    };

    on_settings_changed = on_changed;

    settings_register(&spindle1_setting_details);

    spindle_cap = cap;

    spindle_state_t spindle_state = { .on = On };
    spindle_state.ccw = cap.direction;
    spindle_state.pwm = cap.pwm_invert;

    setting_remove_elements(Setting_SpindleInvertMask1, spindle_state.mask);
}

#endif
