/*
  asymmetric_ganging.c - kinematics implementation for asymmetric ganging of two axis motors

  Part of grblHAL

  Copyright (c) 2026 Terje Io

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

#include "../grbl.h"

#if defined(ASYMMETRIC_GANGING) || defined(ASYMMETRIC_AUTO_SQUARE)

#if N_AXIS <= 3
#error "Kinematics for asymmetric ganging must have N_AXIS > 3"
#endif

#include <math.h>

#include "../hal.h"
#include "../settings.h"
#include "../planner.h"
#include "../kinematics.h"

#ifdef ASYMMETRIC_AUTO_SQUARE
#define PRIMARY_AXIS ASYMMETRIC_AUTO_SQUARE
#else
#define PRIMARY_AXIS ASYMMETRIC_GANGING
#endif
#define PRIMARY_AXIS_BIT (1 << PRIMARY_AXIS)
#define GANGED_AXIS (N_AXIS - 1)
#define GANGED_AXIS_BIT (1 << GANGED_AXIS)

static on_report_options_ptr on_report_options;
static on_settings_changed_ptr on_settings_changed;
static stepper_get_ganged_ptr get_ganged_axes;

#ifdef ASYMMETRIC_AUTO_SQUARE

static axes_signals_t motor_disable = {0};

static stepper_disable_motors_ptr disable_motors;
static limits_get_state_ptr get_limits_state;
static home_get_state_ptr get_home_state;
static stepper_pulse_start_ptr pulse_start;

static void onDisableMotors (axes_signals_t axes, squaring_mode_t mode)
{
    if(disable_motors)
        disable_motors(axes, mode);

    if(!axes.bits)
        motor_disable.bits = 0;
    else if(axes.bits & (1 << PRIMARY_AXIS)) {
/*
        if(mode == SquaringMode_A)
            sys.homing_axis_lock.bits &= ~(1 << PRIMARY_AXIS);
        else
            sys.homing_axis_lock.bits &= ~GANGED_AXIS_BIT;
*/
        motor_disable.y = mode == SquaringMode_A;
        if(mode == SquaringMode_B)
            motor_disable.bits |= GANGED_AXIS_BIT;
        else
            motor_disable.bits &= ~GANGED_AXIS_BIT;
    }
}

static limit_signals_t onGetLimitsState (void)
{
    limit_signals_t limits = get_limits_state();

    limits.min2.y = !!(limits.min.bits & GANGED_AXIS_BIT);

    return limits;
}

/*
ISR_CODE static home_signals_t ISR_FUNC(onGetHomingState)(void)
{
    home_signals_t home = get_home_state();

    if(home.a.bits && GANGED_AXIS_BIT) {
        home.b.y = On;
        home.a.bits &= ~GANGED_AXIS_BIT;
    }

    return home;
}
*/

void ISR_CODE ISR_FUNC(onStepperPulse) (stepper_t *stepper)
{
    if(stepper->step_out.bits & motor_disable.bits) {

        if(motor_disable.bits & PRIMARY_AXIS_BIT)
            stepper->step_out.bits &= ~PRIMARY_AXIS_BIT;

        if(motor_disable.bits & GANGED_AXIS_BIT)
            stepper->step_out.bits &= ~GANGED_AXIS_BIT;
    }

    pulse_start(stepper);
}

static bool homing_cycle_validate (axes_signals_t cycle)
{
    return !(cycle.bits & PRIMARY_AXIS_BIT) || ((cycle.bits & PRIMARY_AXIS_BIT) && (cycle.bits & GANGED_AXIS_BIT));
}

static axes_signals_t onGetGangedAxes (bool auto_squared)
{
    axes_signals_t axes = {0};

    if(get_ganged_axes)
        axes = get_ganged_axes(auto_squared);

    axes.bits |= PRIMARY_AXIS_BIT;

    return axes;
}

#else

static bool homing_cycle_validate (axes_signals_t cycle)
{
    return true;
}

static axes_signals_t onGetGangedAxes (bool auto_squared)
{
    axes_signals_t axes = {0};

    if(get_ganged_axes)
        axes = get_ganged_axes(auto_squared);

    if(!auto_squared)
        axes.bits |= PRIMARY_AXIS_BIT;

    return axes;
}


#endif // ASYMMETRIC_AUTO_SQUARE

static float *convert_array_steps_to_mpos (float *position, int32_t *steps)
{
    uint_fast8_t idx = N_AXIS;
    do {
        idx--;
        position[idx] = steps[idx] / settings.axis[idx].steps_per_mm;
    } while(idx);

    return position;
}

// Transform position from cartesian coordinate system to corexy coordinate system
static inline float *transform_from_cartesian (float *target, float *position)
{
    memcpy(target, position, sizeof(coord_data_t));

    target[GANGED_AXIS] = position[PRIMARY_AXIS];

    return target;
}

static uint_fast8_t get_axis_mask (uint_fast8_t idx)
{
    return bit(idx);
}

static void set_target_pos (uint_fast8_t idx) // fn name?
{
    sys.position[idx] = 0;
}

// Set machine positions for homed limit switches. Don't update non-homed axes.
// NOTE: settings.max_travel[] is stored as a negative value.
static void set_machine_positions (axes_signals_t cycle)
{
    limits_set_machine_positions(cycle, true);

    if(!settings.homing.flags.force_set_origin)
        sys.position[GANGED_AXIS] = lroundf(sys.home_position[PRIMARY_AXIS] * settings.axis[GANGED_AXIS].steps_per_mm);
}

// called from mc_line() to segment lines if not overridden, default implementation for pass-through
static float *kinematics_segment_line (float *target, float *position, plan_line_data_t *pl_data, bool init)
{
    static uint_fast8_t iterations;
    static coord_data_t trsf;

    if(init) {
        iterations = 2;
        transform_from_cartesian(trsf.values, target);
    }

    return iterations-- == 0 ? NULL : trsf.values;
}

static float homing_cycle_get_feedrate (axes_signals_t cycle, float feedrate, homing_mode_t mode)
{
    return feedrate;
}

static void onSettingsChanged (settings_t *settings, settings_changed_flags_t changed)
{
    uint_fast8_t idx = sizeof(settings->homing.cycle) / sizeof(axes_signals_t);

    on_settings_changed(settings, changed);

    float steps_per_mm = settings->axis[GANGED_AXIS].steps_per_mm;

    memcpy(&settings->axis[GANGED_AXIS], &settings->axis[PRIMARY_AXIS], sizeof(axis_settings_t));

    settings->axis[GANGED_AXIS].steps_per_mm = steps_per_mm;

    do {
        if(settings->homing.cycle[--idx].bits & PRIMARY_AXIS_BIT)
            settings->homing.cycle[idx].bits |= GANGED_AXIS_BIT;
        else if(settings->homing.cycle[idx].bits & GANGED_AXIS_BIT)
            settings->homing.cycle[idx].bits &= GANGED_AXIS_BIT;
    } while(idx);

    if(settings->steppers.enable_invert.bits & PRIMARY_AXIS_BIT)
        settings->steppers.enable_invert.bits |= GANGED_AXIS_BIT;
    else
        settings->steppers.enable_invert.bits &= ~GANGED_AXIS_BIT;

    if(settings->steppers.dir_invert.bits & PRIMARY_AXIS_BIT)
        settings->steppers.dir_invert.bits |= GANGED_AXIS_BIT;
    else
        settings->steppers.dir_invert.bits &= ~GANGED_AXIS_BIT;

    if(settings->steppers.step_invert.bits & PRIMARY_AXIS_BIT)
        settings->steppers.step_invert.bits |= GANGED_AXIS_BIT;
    else
        settings->steppers.step_invert.bits &= ~GANGED_AXIS_BIT;

    if(settings->steppers.energize.bits & PRIMARY_AXIS_BIT)
        settings->steppers.energize.bits |= GANGED_AXIS_BIT;
    else
        settings->steppers.energize.bits &= ~GANGED_AXIS_BIT;

    if(settings->homing.dir_mask.bits & PRIMARY_AXIS_BIT)
        settings->homing.dir_mask.bits |= GANGED_AXIS_BIT;
    else
        settings->homing.dir_mask.bits &= ~GANGED_AXIS_BIT;

    settings->steppers.is_rotary.bits &= ~GANGED_AXIS_BIT;

#ifdef ASYMMETRIC_AUTO_SQUARE

    if(settings->limits.invert.bits & PRIMARY_AXIS_BIT)
        settings->limits.invert.bits |= GANGED_AXIS_BIT;
    else
        settings->limits.invert.bits &= ~GANGED_AXIS_BIT;

    if(hal.stepper.pulse_start != onStepperPulse) {
        pulse_start = hal.stepper.pulse_start;
        hal.stepper.pulse_start = onStepperPulse;
    }

#endif
}

PROGMEM static const char label[] = {
#if PRIMARY_AXIS == X_AXIS
    "Ganged X-motor travel resolution"
#elif PRIMARY_AXIS == Y_AXIS
    "Ganged Y-motor travel resolution"
#elif PRIMARY_AXIS == Z_AXIS
    "Ganged Z-motor travel resolution"
#endif
};

PROGMEM static const setting_detail_t axis_settings[] = {
    { Setting_AxisStepsPerMM + GANGED_AXIS, Group_Axis0 + PRIMARY_AXIS, label, "step/mm", Format_Decimal, "#####0.000##", NULL, NULL, Setting_IsLegacy, &settings.axis[GANGED_AXIS].steps_per_mm, NULL, NULL },
};

static void report_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[KINEMATICS:Asymmetric ganging v0.01]" ASCII_EOL);
}

// Initialize API pointers for xxx kinematics
void asymmetric_ganging_init (void)
{
    static setting_details_t axis_setting_details = {
        .is_core = true,
        .settings = axis_settings,
        .n_settings = sizeof(axis_settings) / sizeof(setting_detail_t),
        .save = settings_write_global
    };

    system_claim_axis();
    kinematics.limits_set_target_pos = set_target_pos;
    kinematics.limits_get_axis_mask = get_axis_mask;
    kinematics.limits_set_machine_positions = set_machine_positions;
    kinematics.transform_from_cartesian = transform_from_cartesian;
    kinematics.transform_steps_to_cartesian = convert_array_steps_to_mpos;
    kinematics.segment_line = kinematics_segment_line;
    kinematics.homing_cycle_validate = homing_cycle_validate;
    kinematics.homing_cycle_get_feedrate = homing_cycle_get_feedrate;

    settings_register(&axis_setting_details);

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = report_options;

    on_settings_changed = grbl.on_settings_changed;
    grbl.on_settings_changed = onSettingsChanged;

    get_ganged_axes = hal.stepper.get_ganged;
    hal.stepper.get_ganged = onGetGangedAxes;

#ifdef ASYMMETRIC_AUTO_SQUARE

    get_limits_state = hal.limits.get_state;
    hal.limits.get_state = onGetLimitsState;

    get_home_state = hal.homing.get_state;
//    hal.homing.get_state = onGetHomingState;

    disable_motors = hal.stepper.disable_motors;
    hal.stepper.disable_motors = onDisableMotors;

#endif
}

#endif // ASYMMETRIC_GANGING || ASYMMETRIC_AUTO_SQUARE
