/*
  spindle_control.h - spindle control methods

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

#ifndef _SPINDLE_CONTROL_H_
#define _SPINDLE_CONTROL_H_

#include "pid.h"

#define SPINDLE_NONE        0
#define SPINDLE_HUANYANG1   1
#define SPINDLE_HUANYANG2   2
#define SPINDLE_GS20        3
#define SPINDLE_YL620A      4
#define SPINDLE_MODVFD      5
#define SPINDLE_H100        6
#define SPINDLE_ONOFF0      7 // typically implemented by driver.c
#define SPINDLE_ONOFF0_DIR  8 // typically implemented by driver.c
#define SPINDLE_ONOFF1      9
#define SPINDLE_ONOFF1_DIR 10
#define SPINDLE_PWM0       11 // typically implemented by driver.c
#define SPINDLE_PWM0_NODIR 12 // typically implemented by driver.c
#define SPINDLE_PWM1       13 // typically implemented by driver.c
#define SPINDLE_PWM1_NODIR 14
#define SPINDLE_PWM2       15
#define SPINDLE_PWM2_NODIR 16
#define SPINDLE_PWM0_CLONE 17
#define SPINDLE_SOLENOID   18
#define SPINDLE_STEPPER    19
#define SPINDLE_NOWFOREVER 20
#define SPINDLE_MY_SPINDLE 30

#define SPINDLE_ALL_VFD ((1<<SPINDLE_HUANYANG1)|(1<<SPINDLE_HUANYANG2)|(1<<SPINDLE_GS20)|(1<<SPINDLE_YL620A)|(1<<SPINDLE_MODVFD)|(1<<SPINDLE_H100)|(1<<SPINDLE_NOWFOREVER))
#define SPINDLE_ALL (SPINDLE_ALL_VFD|(1<<SPINDLE_PWM0))

typedef int8_t spindle_id_t;
typedef int8_t spindle_num_t;

// if changed to > 8 bits planner_cond_t needs to be changed too
typedef union {
    uint8_t value;
    uint8_t mask;
    struct {
        uint8_t on               :1,
                ccw              :1,
                pwm              :1, //!< NOTE: only used for PWM inversion setting
                reserved         :1,
                override_disable :1,
                encoder_error    :1,
                at_speed         :1, //!< Spindle is at speed.
                synchronized     :1;
    };
} spindle_state_t;

/*! \brief  Bitmap flags for spindle capabilities. */
typedef union {
    uint16_t value; //!< All bitmap flags.
    struct {
        uint16_t variable          :1, //!< Variable spindle speed is supported.
                 direction         :1, //!< Spindle direction (M4) is supported.
                 at_speed          :1, //!< Spindle at speed feedback is supported.
                 laser             :1, //!< Spindle can control a laser.
                 pwm_invert        :1, //!< Spindle PWM output can be inverted.
                 pid               :1,
                 pwm_linearization :1,
                 rpm_range_locked  :1, //!< Spindle RPM range (min, max) not inherited from settings.
                 gpio_controlled   :1, //!< On/off and direction is controlled by GPIO.
                 cmd_controlled    :1, //!< Command controlled, e.g. over ModBus.
                 cloned            :1, //!< Spindle is cloned.
                 unassigned        :5;
    };
} spindle_cap_t;

/*! \brief Used when HAL driver supports spindle synchronization. */
typedef struct {
    float rpm;
    float rpm_low_limit;
    float rpm_high_limit;
    float angular_position; //!< Number of revolutions since last reset
    float rpm_programmed;
    uint32_t index_count;
    uint32_t pulse_count;
    uint32_t error_count;
    bool at_speed_enabled;
    spindle_state_t state_programmed;
} spindle_data_t;

typedef enum {
    SpindleData_Counters,           //!< 0
    SpindleData_RPM,                //!< 1
    SpindleData_AngularPosition,    //!< 2
    SpindleData_AtSpeed             //!< 3
} spindle_data_request_t;

typedef enum {
    SpindleType_PWM,        //!< 0
    SpindleType_Basic,      //!< 1 - on/off + optional direction
    SpindleType_VFD,        //!< 2
    SpindleType_Solenoid,   //!< 3
    SpindleType_Stepper,    //!< 4
    SpindleType_Null,       //!< 5
} spindle_type_t;

typedef enum {
    SpindleHAL_Raw,         //!< 0 - NOTE: read-only
    SpindleHAL_Configured,  //!< 1
    SpindleHAL_Active,      //!< 2
} spindle_hal_t;

struct spindle_ptrs;    // members defined below
struct spindle_pwm;     // members defined below
struct spindle_param;   // members defined below

/*! \brief Pointer to function for configuring a spindle.
\param spindle a pointer to a \ref spindle_struct.
\returns \a true if successful, \false if not.
*/
typedef bool (*spindle_config_ptr)(struct spindle_ptrs *spindle);

/*! \brief Pointer to function for setting the spindle state.
\param state a \a spindle_state_t union variable.
\param rpm spindle RPM.
*/
typedef void (*spindle_set_state_ptr)(struct spindle_ptrs *spindle, spindle_state_t state, float rpm);

#ifdef GRBL_ESP32
typedef void (*esp32_spindle_off_ptr)(struct spindle_ptrs *spindle);
#endif

/*! \brief Pointer to function for getting the spindle state.
\returns state in a \a spindle_state_t union variable.
*/
typedef spindle_state_t (*spindle_get_state_ptr)(struct spindle_ptrs *spindle);

/*! \brief Pointer to function for converting a RPM value to a PWM value.

Typically this is a wrapper for the spindle_compute_pwm_value() function provided by the core.

\param rpm spindle RPM.
\returns the corresponding PWM value.
*/
typedef uint_fast16_t (*spindle_get_pwm_ptr)(struct spindle_ptrs *spindle, float rpm);

/*! \brief Pointer to function for updating spindle speed on the fly.
\param pwm new spindle PWM value.
\returns the actual PWM value used.

__NOTE:__ this function will be called from an interrupt context.
*/
typedef void (*spindle_update_pwm_ptr)(struct spindle_ptrs *spindle, uint_fast16_t pwm);

/*! \brief Pointer to function for updating spindle RPM.
\param rpm spindle RPM.
*/
typedef void (*spindle_update_rpm_ptr)(struct spindle_ptrs *spindle, float rpm);

/*! \brief Pointer to function for getting spindle data.
\param request request type as a \a #spindle_data_request_t enum.
\returns pointer to the requested information in a spindle_data_t structure.

__NOTE:__ this function requires input from a spindle encoder.
*/
typedef spindle_data_t *(*spindle_get_data_ptr)(spindle_data_request_t request);

/*! \brief Pointer to function for resetting spindle data. */
typedef void (*spindle_reset_data_ptr)(void);

/*! \brief Pointer to function for outputting a spindle on pulse.
Used for Pulses Per Inch (PPI) laser mode.
\param pulse_length spindle on length in microseconds.
*/
typedef void (*spindle_pulse_on_ptr)(uint_fast16_t pulse_length);

typedef struct {
    float rpm;
    float start;
    float end;
} pwm_piece_t;

typedef union {
    uint8_t value;
    uint8_t mask;
    struct {
        uint8_t enable_rpm_controlled :1, // PWM spindle only
                laser_mode_disable    :1, // PWM spindle only
                pwm_disable           :1, // PWM spindle only
                unassigned            :5;
    };
} spindle_settings_flags_t;

typedef struct {
    spindle_state_t invert;
    spindle_settings_flags_t flags;
    float rpm_max;
    float rpm_min;
    float pwm_freq;
    float pwm_off_value;
    float pwm_min_value;
    float pwm_max_value;
    float at_speed_tolerance;                   //!< Tolerance in percent of programmed speed.
    pwm_piece_t pwm_piece[SPINDLE_NPWM_PIECES];
} spindle_pwm_settings_t;

typedef struct {
    uint8_t ref_id;
    uint8_t encoder_spindle;
    uint16_t ppr;                               //!< Spindle encoder pulses per revolution (PPR).
    uint16_t on_delay;
    uint16_t off_delay;
    float at_speed_tolerance;                   //!< Tolerance in percent of programmed speed.
} spindle_settings_t;

typedef struct {
    uint8_t port_on;
    uint8_t port_dir;
    uint8_t port_pwm;
    spindle_pwm_settings_t cfg;
} spindle1_pwm_settings_t;

typedef union {
    uint8_t value;
    struct {
        uint8_t invert_pwm         :1, //!< NOTE: set (by driver) when inversion is done in code
                always_on          :1,
                cloned             :1,
                laser_mode_disable :1,
                unused             :4;
    };
} spindle_pwm_flags_t;

//!* \brief Precalculated values that may be set/used by HAL driver to speed up RPM to PWM conversions if variable spindle is supported. */
typedef struct spindle_pwm {
    uint32_t f_clock;
    spindle_pwm_settings_t *settings;
    uint_fast16_t period;
    uint_fast16_t off_value;    //!< NOTE: this value holds the inverted version if software PWM inversion is enabled by the driver.
    uint_fast16_t min_value;
    uint_fast16_t max_value;
    float rpm_min;              //!< Minimum spindle RPM.
    float pwm_gradient;
    spindle_pwm_flags_t flags;
    int_fast16_t offset;
    uint_fast16_t n_pieces;
    pwm_piece_t piece[SPINDLE_NPWM_PIECES];
    uint_fast16_t (*compute_value)(struct spindle_pwm *pwm_data, float rpm, bool pid_limit);
} spindle_pwm_t;

typedef union
{
    spindle_pwm_t *pwm;
} spindle_context_ptr_t __attribute__ ((__transparent_union__));

/*! \brief Handlers and data for spindle support. */
struct spindle_ptrs {
    spindle_id_t id;                    //!< Spindle id, assigned on spindle registration.
    uint8_t ref_id;                     //!< Spindle id, assigned on spindle registration.
    struct spindle_param *param;        //!< Pointer to current spindle parameters, assigned when spindle is enabled.
    spindle_type_t type;                //!< Spindle type.
    spindle_cap_t cap;                  //!< Spindle capabilities.
    spindle_context_ptr_t context;      //!< Optional pointer to spindle specific context.
    uint_fast16_t pwm_off_value;        //!< Value for switching PWM signal off.
    float rpm_min;                      //!< Minimum spindle RPM.
    float rpm_max;                      //!< Maximum spindle RPM.
    float at_speed_tolerance;           //!< Tolerance in percent of programmed speed.
    spindle_config_ptr config;          //!< Optional handler for configuring the spindle.
    spindle_set_state_ptr set_state;    //!< Handler for setting spindle state.
    spindle_get_state_ptr get_state;    //!< Handler for getting spindle state.
    spindle_get_pwm_ptr get_pwm;        //!< Handler for calculating spindle PWM value from RPM.
    spindle_update_pwm_ptr update_pwm;  //!< Handler for updating spindle PWM output.
    spindle_update_rpm_ptr update_rpm;  //!< Handler for updating spindle RPM.
#ifdef GRBL_ESP32
    esp32_spindle_off_ptr esp32_off;    //!< Workaround handler for snowflake ESP32 Guru awaken by floating point data in ISR context.
#endif
    // Optional entry points.
    spindle_pulse_on_ptr pulse_on;      //!< Optional handler for Pulses Per Inch (PPI) mode. Required for the laser PPI plugin.
    spindle_get_data_ptr get_data;      //!< Optional handler for getting spindle data. Required for spindle sync, copied from hal.spindle_data.get on selection.
    spindle_reset_data_ptr reset_data;  //!< Optional handler for resetting spindle data. Required for spindle sync, copied from hal.spindle_data.reset on selection.
};

typedef struct spindle_ptrs spindle_ptrs_t;

//! \brief  Data used for Constant Surface Speed (CSS) mode calculations.
typedef struct {
    float surface_speed;    //!< Surface speed in millimeters/min
    float target_rpm;       //!< Target RPM at end of movement
    float delta_rpm;        //!< Delta between start and target RPM
    float max_rpm;          //!< Maximum spindle RPM
    float tool_offset;      //!< Tool offset
    uint_fast8_t axis;      //!< Linear (tool) axis
} spindle_css_data_t;

/*! \brief Structure used for holding the current state of an enabled spindle. */
typedef struct spindle_param {
    float rpm;
    float rpm_overridden;
    spindle_state_t state;
    override_t override_pct;    //!< Spindle RPM override value in percent
    spindle_css_data_t css;     //!< Data used for Constant Surface Speed Mode (CSS) calculations, NULL if not in CSS mode.
    spindle_ptrs_t *hal;
} spindle_param_t;

typedef struct {
    spindle_get_data_ptr get;      //!< Optional handler for getting spindle data. Required for spindle sync.
    spindle_reset_data_ptr reset;  //!< Optional handler for resetting spindle data. Required for spindle sync.
} spindle_data_ptrs_t;

/*! \brief Structure holding data passed to the callback function called by spindle_enumerate_spindles(). */
typedef struct  {
    spindle_id_t id;
    uint8_t ref_id;
    spindle_num_t num;
    const char *name;
    bool enabled;
    bool is_current;
    const spindle_ptrs_t *hal;
} spindle_info_t;

/*! \brief Pointer to callback function called by spindle_enumerate_spindles().
\param spindle prointer to a \a spindle_info_t struct.
*/
typedef bool (*spindle_enumerate_callback_ptr)(spindle_info_t *spindle, void *data);

void spindle_set_override (spindle_ptrs_t *spindle, override_t speed_override);

// Sets spindle running state with direction, enable, and spindle RPM.
bool spindle_set_state (spindle_ptrs_t *spindle, spindle_state_t state, float rpm);

// Called by g-code parser when setting spindle state and requires a buffer sync.
bool spindle_set_state_synced (spindle_ptrs_t *spindle, spindle_state_t state, float rpm);

// Spindle speed calculation and limit handling
float spindle_set_rpm (spindle_ptrs_t *spindle, float rpm, override_t speed_override);

// Restore spindle running state with direction, enable, spindle RPM and appropriate delay.
bool spindle_restore (spindle_ptrs_t *spindle, spindle_state_t state, float rpm, uint16_t on_delay_ms);

void spindle_all_off (void);

//
// The following functions are not called by the core, may be called by driver code.
//

#define spindle_validate_at_speed(d, r) { d.rpm = r; d.state_programmed.at_speed = !d.at_speed_enabled || (d.rpm >= d.rpm_low_limit && d.rpm <= d.rpm_high_limit); }
/*
__attribute__((always_inline)) static inline void spindle_validate_at_speed (spindle_data_t *spindle_data, float rpm)
{
    spindle_data->rpm = rpm;
    spindle_data->state_programmed.at_speed = !spindle_data->at_speed_enabled || (spindle_data->rpm >= spindle_data->rpm_low_limit && spindle_data->rpm <= spindle_data->rpm_high_limit);
}
*/

bool spindle_precompute_pwm_values (spindle_ptrs_t *spindle, spindle_pwm_t *pwm_data, spindle_pwm_settings_t *settings, uint32_t clock_hz);

spindle_id_t spindle_register (const spindle_ptrs_t *spindle, const char *name);

spindle_id_t spindle_add_null (void);

uint8_t spindle_get_count (void);

bool spindle_get_id (uint8_t ref_id, spindle_id_t *spindle_id);

bool spindle_select (spindle_id_t spindle_id);

spindle_cap_t spindle_get_caps (bool active);

void spindle_update_caps (spindle_ptrs_t *spindle, spindle_pwm_t *pwm_caps);

void spindle_bind_encoder (const spindle_data_ptrs_t *encoder_data);

bool spindle_set_at_speed_range (spindle_ptrs_t *spindle, spindle_data_t *spindle_data, float rpm);

spindle_ptrs_t *spindle_get_hal (spindle_id_t spindle_id, spindle_hal_t hal);

const char *spindle_get_name (spindle_id_t spindle_id);

spindle_id_t spindle_get_default (void);

spindle_num_t spindle_enable (spindle_id_t spindle_id);

bool spindle_enumerate_spindles (spindle_enumerate_callback_ptr callback, void *data);

//

bool spindle_is_enabled (spindle_num_t spindle_num);

bool spindle_is_on (void);

spindle_ptrs_t *spindle_get (spindle_num_t spindle_num);

#if N_SPINDLE > 1

typedef void (*spindle1_settings_changed_ptr)(spindle1_pwm_settings_t *settings);

spindle1_pwm_settings_t *spindle1_settings_add (bool claim_ports);
void spindle1_settings_register (spindle_cap_t cap, spindle1_settings_changed_ptr on_changed);

#endif

#endif // _SPINDLE_CONTROL_H_
