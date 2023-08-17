/*
  settings.c - non-volatile storage configuration handling

  Part of grblHAL

  Copyright (c) 2017-2023 Terje Io
  Copyright (c) 2011-2015 Sungeun K. Jeon
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
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#include "hal.h"
#include "config.h"
#include "machine_limits.h"
#include "nvs_buffer.h"
#include "tool_change.h"
#include "state_machine.h"
#if ENABLE_BACKLASH_COMPENSATION
#include "motion_control.h"
#endif
#if ENABLE_SPINDLE_LINEARIZATION
#include <stdio.h>
#endif

settings_t settings;

const settings_restore_t settings_all = {
    .defaults          = SETTINGS_RESTORE_DEFAULTS,
    .parameters        = SETTINGS_RESTORE_PARAMETERS,
    .startup_lines     = SETTINGS_RESTORE_STARTUP_LINES,
    .build_info        = SETTINGS_RESTORE_BUILD_INFO,
    .driver_parameters = SETTINGS_RESTORE_DRIVER_PARAMETERS
};

PROGMEM const settings_t defaults = {

    .version = SETTINGS_VERSION,

#if DEFAULT_LASER_MODE
    .mode = Mode_Laser,
#elif DEFAULT_LATHE_MODE
    .mode = Mode_Lathe,
#else
    .mode = Mode_Standard,
#endif
    .junction_deviation = DEFAULT_JUNCTION_DEVIATION,
    .arc_tolerance = DEFAULT_ARC_TOLERANCE,
    .g73_retract = DEFAULT_G73_RETRACT,
    .report_interval = DEFAULT_AUTOREPORT_INTERVAL,
    .timezone = DEFAULT_TIMEZONE_OFFSET,
    .planner_buffer_blocks = DEFAULT_PLANNER_BUFFER_BLOCKS,
    .flags.legacy_rt_commands = DEFAULT_LEGACY_RTCOMMANDS,
    .flags.report_inches = DEFAULT_REPORT_INCHES,
    .flags.sleep_enable = DEFAULT_SLEEP_ENABLE,
    .flags.compatibility_level = COMPATIBILITY_LEVEL,
#if DEFAULT_DISABLE_G92_PERSISTENCE
    .flags.g92_is_volatile = 1,
#else
    .flags.g92_is_volatile = 0,
#endif
    .flags.disable_laser_during_hold = DEFAULT_DISABLE_LASER_DURING_HOLD,
    .flags.restore_after_feed_hold = DEFAULT_RESTORE_AFTER_FEED_HOLD,
    .flags.force_initialization_alarm = DEFAULT_FORCE_INITIALIZATION_ALARM,
    .flags.restore_overrides = DEFAULT_RESET_OVERRIDES,
    .flags.no_restore_position_after_M6 = DEFAULT_TOOLCHANGE_NO_RESTORE_POSITION,

    .probe.disable_probe_pullup = DEFAULT_PROBE_SIGNAL_DISABLE_PULLUP,
    .probe.allow_feed_override = DEFAULT_ALLOW_FEED_OVERRIDE_DURING_PROBE_CYCLES,
    .probe.invert_probe_pin = DEFAULT_PROBE_SIGNAL_INVERT,

    .steppers.pulse_microseconds = DEFAULT_STEP_PULSE_MICROSECONDS,
    .steppers.pulse_delay_microseconds = DEFAULT_STEP_PULSE_DELAY,
    .steppers.idle_lock_time = DEFAULT_STEPPER_IDLE_LOCK_TIME,
    .steppers.step_invert.mask = DEFAULT_STEP_SIGNALS_INVERT_MASK,
    .steppers.dir_invert.mask = DEFAULT_DIR_SIGNALS_INVERT_MASK,
    .steppers.ganged_dir_invert.mask = DEFAULT_GANGED_DIRECTION_INVERT_MASK,
#if COMPATIBILITY_LEVEL <= 2
    .steppers.enable_invert.mask = DEFAULT_ENABLE_SIGNALS_INVERT_MASK,
#elif DEFAULT_ENABLE_SIGNALS_INVERT_MASK
    .steppers.enable_invert.mask = 0,
#else
    .steppers.enable_invert.mask = AXES_BITMASK,
#endif
    .steppers.deenergize.mask = DEFAULT_STEPPER_DEENERGIZE_MASK,
#if N_AXIS > 3
    .steppers.is_rotational.mask = (DEFAULT_AXIS_ROTATIONAL_MASK & AXES_BITMASK) & 0b11111000,
#endif
#if DEFAULT_HOMING_ENABLE
    .homing.flags.enabled = DEFAULT_HOMING_ENABLE,
    .homing.flags.init_lock = DEFAULT_HOMING_INIT_LOCK,
    .homing.flags.single_axis_commands = DEFAULT_HOMING_SINGLE_AXIS_COMMANDS,
    .homing.flags.force_set_origin = DEFAULT_HOMING_FORCE_SET_ORIGIN,
    .homing.flags.manual = DEFAULT_HOMING_ALLOW_MANUAL,
    .homing.flags.override_locks = DEFAULT_HOMING_OVERRIDE_LOCKS,
    .homing.flags.keep_on_reset = DEFAULT_HOMING_KEEP_STATUS_ON_RESET,
#else
    .homing.flags.value = 0,
#endif
    .homing.dir_mask.value = DEFAULT_HOMING_DIR_MASK,
    .homing.feed_rate = DEFAULT_HOMING_FEED_RATE,
    .homing.seek_rate = DEFAULT_HOMING_SEEK_RATE,
    .homing.debounce_delay = DEFAULT_HOMING_DEBOUNCE_DELAY,
    .homing.pulloff = DEFAULT_HOMING_PULLOFF,
    .homing.locate_cycles = DEFAULT_N_HOMING_LOCATE_CYCLE,
    .homing.cycle[0].mask = DEFAULT_HOMING_CYCLE_0,
    .homing.cycle[1].mask = DEFAULT_HOMING_CYCLE_1,
    .homing.cycle[2].mask = DEFAULT_HOMING_CYCLE_2,
    .homing.dual_axis.fail_length_percent = DEFAULT_DUAL_AXIS_HOMING_FAIL_AXIS_LENGTH_PERCENT,
    .homing.dual_axis.fail_distance_min = DEFAULT_DUAL_AXIS_HOMING_FAIL_DISTANCE_MIN,
    .homing.dual_axis.fail_distance_max = DEFAULT_DUAL_AXIS_HOMING_FAIL_DISTANCE_MAX,

    .status_report.machine_position = DEFAULT_REPORT_MACHINE_POSITION,
    .status_report.buffer_state = DEFAULT_REPORT_BUFFER_STATE,
    .status_report.line_numbers = DEFAULT_REPORT_LINE_NUMBERS,
    .status_report.feed_speed = DEFAULT_REPORT_CURRENT_FEED_SPEED,
    .status_report.pin_state = DEFAULT_REPORT_PIN_STATE,
    .status_report.work_coord_offset = DEFAULT_REPORT_WORK_COORD_OFFSET,
    .status_report.overrides = DEFAULT_REPORT_OVERRIDES,
    .status_report.probe_coordinates = DEFAULT_REPORT_PROBE_COORDINATES,
    .status_report.sync_on_wco_change = DEFAULT_REPORT_SYNC_ON_WCO_CHANGE,
    .status_report.parser_state = DEFAULT_REPORT_PARSER_STATE,
    .status_report.alarm_substate = DEFAULT_REPORT_ALARM_SUBSTATE,
    .status_report.run_substate = DEFAULT_REPORT_RUN_SUBSTATE,
    .limits.flags.hard_enabled = DEFAULT_HARD_LIMIT_ENABLE,
    .limits.flags.soft_enabled = DEFAULT_SOFT_LIMIT_ENABLE,
    .limits.flags.jog_soft_limited = DEFAULT_JOG_LIMIT_ENABLE,
    .limits.flags.check_at_init = DEFAULT_CHECK_LIMITS_AT_INIT,
    .limits.flags.two_switches = DEFAULT_LIMITS_TWO_SWITCHES_ON_AXES,
    .limits.invert.mask = DEFAULT_LIMIT_SIGNALS_INVERT_MASK,
    .limits.disable_pullup.mask = DEFAULT_LIMIT_SIGNALS_PULLUP_DISABLE_MASK,

    .control_invert.mask = DEFAULT_CONTROL_SIGNALS_INVERT_MASK,
    .control_disable_pullup.mask = DEFAULT_DISABLE_CONTROL_PINS_PULL_UP_MASK,

    .spindle.rpm_max = DEFAULT_SPINDLE_RPM_MAX,
    .spindle.rpm_min = DEFAULT_SPINDLE_RPM_MIN,
    .spindle.flags.pwm_disable = false,
    .spindle.flags.enable_rpm_controlled = DEFAULT_SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED,
    .spindle.invert.on = DEFAULT_INVERT_SPINDLE_ENABLE_PIN,
    .spindle.invert.ccw = DEFAULT_INVERT_SPINDLE_CCW_PIN,
    .spindle.invert.pwm = DEFAULT_INVERT_SPINDLE_PWM_PIN,
    .spindle.pwm_freq = DEFAULT_SPINDLE_PWM_FREQ,
    .spindle.pwm_off_value = DEFAULT_SPINDLE_PWM_OFF_VALUE,
    .spindle.pwm_min_value = DEFAULT_SPINDLE_PWM_MIN_VALUE,
    .spindle.pwm_max_value = DEFAULT_SPINDLE_PWM_MAX_VALUE,
    .spindle.at_speed_tolerance = DEFAULT_SPINDLE_AT_SPEED_TOLERANCE,
    .spindle.ppr = DEFAULT_SPINDLE_PPR,
    .spindle.pid.p_gain = DEFAULT_SPINDLE_P_GAIN,
    .spindle.pid.i_gain = DEFAULT_SPINDLE_I_GAIN,
    .spindle.pid.d_gain = DEFAULT_SPINDLE_D_GAIN,
    .spindle.pid.i_max_error = DEFAULT_SPINDLE_I_MAX,
#if ENABLE_SPINDLE_LINEARIZATION
  #if SPINDLE_NPWM_PIECES > 0
    .spindle.pwm_piece[0] = { .rpm = DEFAULT_RPM_POINT01, .start = DEFAULT_RPM_LINE_A1, .end = DEFAULT_RPM_LINE_B1 },
  #endif
  #if SPINDLE_NPWM_PIECES > 1
    .spindle.pwm_piece[1] = { .rpm = DEFAULT_RPM_POINT12, .start = DEFAULT_RPM_LINE_A2, .end = DEFAULT_RPM_LINE_B2 },
  #endif
  #if SPINDLE_NPWM_PIECES > 2
    .spindle.pwm_piece[2] = { .rpm = DEFAULT_RPM_POINT23, .start = DEFAULT_RPM_LINE_A3, .end = DEFAULT_RPM_LINE_B3 },
  #endif
  #if SPINDLE_NPWM_PIECES > 3
    .spindle.pwm_piece[3] = { .rpm = DEFAULT_RPM_POINT34, .start = DEFAULT_RPM_LINE_A4, .end = DEFAULT_RPM_LINE_B4 },
  #endif
#else
  #if SPINDLE_NPWM_PIECES > 0
    .spindle.pwm_piece[0] = { .rpm = NAN, .start = 0.0f, .end = 0.0f },
  #endif
  #if SPINDLE_NPWM_PIECES > 1
    .spindle.pwm_piece[1] = { .rpm = NAN, .start = 0.0f, .end = 0.0f },
  #endif
  #if SPINDLE_NPWM_PIECES > 2
    .spindle.pwm_piece[2] = { .rpm = NAN, .start = 0.0f, .end = 0.0f },
  #endif
  #if SPINDLE_NPWM_PIECES > 3
    .spindle.pwm_piece[3] = { .rpm = NAN, .start = 0.0f, .end = 0.0f },
  #endif
#endif

    .coolant_invert.flood = DEFAULT_INVERT_COOLANT_FLOOD_PIN,
    .coolant_invert.mist = DEFAULT_INVERT_COOLANT_MIST_PIN,

    .axis[X_AXIS].steps_per_mm = DEFAULT_X_STEPS_PER_MM,
    .axis[X_AXIS].max_rate = DEFAULT_X_MAX_RATE,
    .axis[X_AXIS].acceleration = (DEFAULT_X_ACCELERATION * 60.0f * 60.0f),
    .axis[X_AXIS].max_travel = (-DEFAULT_X_MAX_TRAVEL),
    .axis[X_AXIS].dual_axis_offset = 0.0f,
#if ENABLE_BACKLASH_COMPENSATION
    .axis[X_AXIS].backlash = 0.0f,
#endif

    .axis[Y_AXIS].steps_per_mm = DEFAULT_Y_STEPS_PER_MM,
    .axis[Y_AXIS].max_rate = DEFAULT_Y_MAX_RATE,
    .axis[Y_AXIS].max_travel = (-DEFAULT_Y_MAX_TRAVEL),
    .axis[Y_AXIS].acceleration = (DEFAULT_Y_ACCELERATION * 60.0f * 60.0f),
    .axis[Y_AXIS].dual_axis_offset = 0.0f,
#if ENABLE_BACKLASH_COMPENSATION
    .axis[Y_AXIS].backlash = 0.0f,
#endif

    .axis[Z_AXIS].steps_per_mm = DEFAULT_Z_STEPS_PER_MM,
    .axis[Z_AXIS].max_rate = DEFAULT_Z_MAX_RATE,
    .axis[Z_AXIS].acceleration = (DEFAULT_Z_ACCELERATION * 60.0f * 60.0f),
    .axis[Z_AXIS].max_travel = (-DEFAULT_Z_MAX_TRAVEL),
    .axis[Z_AXIS].dual_axis_offset = 0.0f,
#if ENABLE_BACKLASH_COMPENSATION
    .axis[Z_AXIS].backlash = 0.0f,
#endif

#ifdef A_AXIS
    .axis[A_AXIS].steps_per_mm = DEFAULT_A_STEPS_PER_MM,
    .axis[A_AXIS].max_rate = DEFAULT_A_MAX_RATE,
    .axis[A_AXIS].acceleration =(DEFAULT_A_ACCELERATION * 60.0f * 60.0f),
    .axis[A_AXIS].max_travel = (-DEFAULT_A_MAX_TRAVEL),
    .axis[A_AXIS].dual_axis_offset = 0.0f,
#if ENABLE_BACKLASH_COMPENSATION
    .axis[A_AXIS].backlash = 0.0f,
#endif
    .homing.cycle[3].mask = DEFAULT_HOMING_CYCLE_3,
#endif

#ifdef B_AXIS
    .axis[B_AXIS].steps_per_mm = DEFAULT_B_STEPS_PER_MM,
    .axis[B_AXIS].max_rate = DEFAULT_B_MAX_RATE,
    .axis[B_AXIS].acceleration = (DEFAULT_B_ACCELERATION * 60.0f * 60.0f),
    .axis[B_AXIS].max_travel = (-DEFAULT_B_MAX_TRAVEL),
    .axis[B_AXIS].dual_axis_offset = 0.0f,
#if ENABLE_BACKLASH_COMPENSATION
    .axis[B_AXIS].backlash = 0.0f,
#endif
    .homing.cycle[4].mask = DEFAULT_HOMING_CYCLE_4,
#endif

#ifdef C_AXIS
    .axis[C_AXIS].steps_per_mm = DEFAULT_C_STEPS_PER_MM,
    .axis[C_AXIS].acceleration = (DEFAULT_C_ACCELERATION * 60.0f * 60.0f),
    .axis[C_AXIS].max_rate = DEFAULT_C_MAX_RATE,
    .axis[C_AXIS].max_travel = (-DEFAULT_C_MAX_TRAVEL),
    .axis[C_AXIS].dual_axis_offset = 0.0f,
#if ENABLE_BACKLASH_COMPENSATION
    .axis[C_AXIS].backlash = 0.0f,
#endif
    .homing.cycle[5].mask = DEFAULT_HOMING_CYCLE_5,
#endif

#ifdef U_AXIS
    .axis[U_AXIS].steps_per_mm = DEFAULT_U_STEPS_PER_MM,
    .axis[U_AXIS].acceleration = (DEFAULT_U_ACCELERATION * 60.0f * 60.0f),
    .axis[U_AXIS].max_rate = DEFAULT_U_MAX_RATE,
    .axis[U_AXIS].max_travel = (-DEFAULT_U_MAX_TRAVEL),
    .axis[U_AXIS].dual_axis_offset = 0.0f,
#if ENABLE_BACKLASH_COMPENSATION
    .axis[U_AXIS].backlash = 0.0f,
#endif
#endif

#ifdef V_AXIS
    .axis[V_AXIS].steps_per_mm = DEFAULT_V_STEPS_PER_MM,
    .axis[V_AXIS].acceleration = (DEFAULT_V_ACCELERATION * 60.0f * 60.0f),
    .axis[V_AXIS].max_rate = DEFAULT_V_MAX_RATE,
    .axis[V_AXIS].max_travel = (-DEFAULT_V_MAX_TRAVEL),
    .axis[V_AXIS].dual_axis_offset = 0.0f,
#if ENABLE_BACKLASH_COMPENSATION
    .axis[V_AXIS].backlash = 0.0f,
#endif
#endif

    .tool_change.mode = (toolchange_mode_t)DEFAULT_TOOLCHANGE_MODE,
    .tool_change.probing_distance = DEFAULT_TOOLCHANGE_PROBING_DISTANCE,
    .tool_change.feed_rate = DEFAULT_TOOLCHANGE_FEED_RATE,
    .tool_change.seek_rate = DEFAULT_TOOLCHANGE_SEEK_RATE,
    .tool_change.pulloff_rate = DEFAULT_TOOLCHANGE_PULLOFF_RATE,

    .parking.flags.enabled = DEFAULT_PARKING_ENABLE,
    .parking.flags.deactivate_upon_init = DEFAULT_DEACTIVATE_PARKING_UPON_INIT,
    .parking.flags.enable_override_control= DEFAULT_ENABLE_PARKING_OVERRIDE_CONTROL,
    .parking.axis = DEFAULT_PARKING_AXIS,
    .parking.target = DEFAULT_PARKING_TARGET,
    .parking.rate = DEFAULT_PARKING_RATE,
    .parking.pullout_rate = DEFAULT_PARKING_PULLOUT_RATE,
    .parking.pullout_increment = DEFAULT_PARKING_PULLOUT_INCREMENT,

    .safety_door.flags.ignore_when_idle = DEFAULT_DOOR_IGNORE_WHEN_IDLE,
    .safety_door.flags.keep_coolant_on = DEFAULT_DOOR_KEEP_COOLANT_ON,
    .safety_door.spindle_on_delay = DEFAULT_SAFETY_DOOR_SPINDLE_DELAY,
    .safety_door.coolant_on_delay = DEFAULT_SAFETY_DOOR_COOLANT_DELAY
};

static bool group_is_available (const setting_group_detail_t *group)
{
    return true;
}

PROGMEM static const setting_group_detail_t setting_group_detail [] = {
     { Group_Root, Group_Root, "Root", group_is_available },
     { Group_Root, Group_General, "General", group_is_available },
     { Group_Root, Group_ControlSignals, "Control signals" },
     { Group_Root, Group_Limits, "Limits" },
     { Group_Limits, Group_Limits_DualAxis, "Dual axis" },
     { Group_Root, Group_Coolant, "Coolant" },
     { Group_Root, Group_Spindle, "Spindle" },
     { Group_Spindle, Group_Spindle_Sync, "Spindle sync" },
     { Group_Root, Group_Toolchange, "Tool change" },
     { Group_Root, Group_Homing, "Homing" },
     { Group_Root, Group_Probing, "Probing" },
     { Group_Root, Group_SafetyDoor, "Safety door" },
     { Group_Root, Group_Jogging, "Jogging"},
     { Group_Root, Group_Stepper, "Stepper" },
     { Group_Root, Group_MotorDriver, "Stepper driver" },
     { Group_Root, Group_Axis, "Axis", group_is_available },
     { Group_Axis, Group_XAxis, "X-axis", group_is_available },
     { Group_Axis, Group_YAxis, "Y-axis", group_is_available },
     { Group_Axis, Group_ZAxis, "Z-axis", group_is_available },
#if !AXIS_REMAP_ABC2UVW
  #ifdef A_AXIS
     { Group_Axis, Group_AAxis, "A-axis", group_is_available },
  #endif
  #ifdef B_AXIS
     { Group_Axis, Group_BAxis, "B-axis", group_is_available },
  #endif
  #ifdef C_AXIS
     { Group_Axis, Group_CAxis, "C-axis", group_is_available },
  #endif
  #ifdef U_AXIS
     { Group_Axis, Group_UAxis, "U-axis", group_is_available },
  #endif
  #ifdef V_AXIS
     { Group_Axis, Group_VAxis, "V-axis", group_is_available }
  #endif
#else
  #ifdef A_AXIS
     { Group_Axis, Group_AAxis, "U-axis", group_is_available },
  #endif
  #ifdef B_AXIS
     { Group_Axis, Group_BAxis, "V-axis", group_is_available },
  #endif
  #ifdef C_AXIS
     { Group_Axis, Group_CAxis, "W-axis", group_is_available },
  #endif
#endif
};

static status_code_t set_probe_invert (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_report_mask (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_report_inches (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_control_invert (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_spindle_invert (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_pwm_mode (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_pwm_options (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_spindle_type (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_control_disable_pullup (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_probe_disable_pullup (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_soft_limits_enable (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_hard_limits_enable (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_jog_soft_limited (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_homing_enable (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_enable_legacy_rt_commands (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_homing_cycle (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_mode (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_sleep_enable (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_hold_actions (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_force_initialization_alarm (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_probe_allow_feed_override (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_tool_change_mode (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_tool_change_probing_distance (setting_id_t id, float value);
static status_code_t set_tool_restore_pos (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_ganged_dir_invert (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_stepper_deenergize_mask (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_report_interval (setting_id_t setting, uint_fast16_t int_value);
static status_code_t set_estop_unlock (setting_id_t id, uint_fast16_t int_value);
#ifndef NO_SAFETY_DOOR_SUPPORT
static status_code_t set_parking_enable (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_restore_overrides (setting_id_t id, uint_fast16_t int_value);
#endif
#if ENABLE_SPINDLE_LINEARIZATION
static status_code_t set_linear_piece (setting_id_t id, char *svalue);
static char *get_linear_piece (setting_id_t id);
#endif
#if N_AXIS > 3
static status_code_t set_rotational_axes (setting_id_t id, uint_fast16_t int_value);
#endif
#if COMPATIBILITY_LEVEL > 2
static status_code_t set_enable_invert_mask (setting_id_t id, uint_fast16_t int_value);
#endif
#if COMPATIBILITY_LEVEL > 1
static status_code_t set_limits_invert_mask (setting_id_t id, uint_fast16_t int_value);
#endif
static status_code_t set_axis_setting (setting_id_t setting, float value);
#if COMPATIBILITY_LEVEL <= 1
static status_code_t set_g92_disable_persistence (setting_id_t id, uint_fast16_t int_value);
#endif
static float get_float (setting_id_t setting);
static uint32_t get_int (setting_id_t id);
static bool is_setting_available (const setting_detail_t *setting);
static bool is_group_available (const setting_detail_t *setting);
#if NGC_EXPRESSIONS_ENABLE
static status_code_t set_ngc_debug_out (setting_id_t id, uint_fast16_t int_value);
#endif

static bool machine_mode_changed = false;
static char control_signals[] = "Reset,Feed hold,Cycle start,Safety door,Block delete,Optional stop,EStop,Probe connected,Motor fault";
static char spindle_signals[] = "Spindle enable,Spindle direction,PWM";
static char coolant_signals[] = "Flood,Mist";
static char ganged_axes[] = "X-Axis,Y-Axis,Z-Axis";
static char spindle_types[100] = "";
static char axis_dist[4] = "mm";
static char axis_rate[8] = "mm/min";
static char axis_accel[10] = "mm/sec^2";
static char axis_steps[9] = "step/mm";

#define AXIS_OPTS { .subgroups = On, .increment = 1 }

PROGMEM static const setting_detail_t setting_detail[] = {
     { Setting_PulseMicroseconds, Group_Stepper, "Step pulse time", "microseconds", Format_Decimal, "#0.0", "2.0", NULL, Setting_IsLegacy, &settings.steppers.pulse_microseconds, NULL, NULL },
     { Setting_StepperIdleLockTime, Group_Stepper, "Step idle delay", "milliseconds", Format_Int16, "####0", NULL, "65535", Setting_IsLegacy, &settings.steppers.idle_lock_time, NULL, NULL },
     { Setting_StepInvertMask, Group_Stepper, "Step pulse invert", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_IsLegacy, &settings.steppers.step_invert.mask, NULL, NULL },
     { Setting_DirInvertMask, Group_Stepper, "Step direction invert", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_IsLegacy, &settings.steppers.dir_invert.mask, NULL, NULL },
#if COMPATIBILITY_LEVEL <= 2
     { Setting_InvertStepperEnable, Group_Stepper, "Invert stepper enable pin(s)", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_IsLegacy, &settings.steppers.enable_invert.mask, NULL, NULL },
#else
     { Setting_InvertStepperEnable, Group_Stepper, "Invert stepper enable pins", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsLegacyFn, set_enable_invert_mask, get_int, NULL },
#endif
#if COMPATIBILITY_LEVEL <= 1
     { Setting_LimitPinsInvertMask, Group_Limits, "Invert limit pins", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_IsLegacy, &settings.limits.invert.mask, NULL, NULL },
#else
     { Setting_LimitPinsInvertMask, Group_Limits, "Invert limit pins", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsLegacyFn, set_limits_invert_mask, get_int, NULL },
#endif
     { Setting_InvertProbePin, Group_Probing, "Invert probe pin", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsLegacyFn, set_probe_invert, get_int, is_setting_available },
     { Setting_SpindlePWMBehaviour, Group_Spindle, "Deprecated", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsLegacyFn, set_pwm_mode, get_int, is_setting_available },
     { Setting_GangedDirInvertMask, Group_Stepper, "Ganged axes direction invert", NULL, Format_Bitfield, ganged_axes, NULL, NULL, Setting_IsExtendedFn, set_ganged_dir_invert, get_int, is_setting_available },
     { Setting_SpindlePWMOptions, Group_Spindle, "PWM Spindle", NULL, Format_XBitfield, "Enable,RPM controls spindle enable signal", NULL, NULL, Setting_IsExtendedFn, set_pwm_options, get_int, is_setting_available },
#if COMPATIBILITY_LEVEL <= 1
     { Setting_StatusReportMask, Group_General, "Status report options", NULL, Format_Bitfield, "Position in machine coordinate,Buffer state,Line numbers,Feed & speed,Pin state,Work coordinate offset,Overrides,Probe coordinates,Buffer sync on WCO change,Parser state,Alarm substatus,Run substatus", NULL, NULL, Setting_IsExtendedFn, set_report_mask, get_int, NULL },
#else
     { Setting_StatusReportMask, Group_General, "Status report options", NULL, Format_Bitfield, "Position in machine coordinate,Buffer state", NULL, NULL, Setting_IsLegacyFn, set_report_mask, get_int, NULL },
#endif
     { Setting_JunctionDeviation, Group_General, "Junction deviation", "mm", Format_Decimal, "#####0.000", NULL, NULL, Setting_IsLegacy, &settings.junction_deviation, NULL, NULL },
     { Setting_ArcTolerance, Group_General, "Arc tolerance", "mm", Format_Decimal, "#####0.000", NULL, NULL, Setting_IsLegacy, &settings.arc_tolerance, NULL, NULL },
     { Setting_ReportInches, Group_General, "Report in inches", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsLegacyFn, set_report_inches, get_int, NULL },
     { Setting_ControlInvertMask, Group_ControlSignals, "Invert control pins", NULL, Format_Bitfield, control_signals, NULL, NULL, Setting_IsExpandedFn, set_control_invert, get_int, NULL },
     { Setting_CoolantInvertMask, Group_Coolant, "Invert coolant pins", NULL, Format_Bitfield, coolant_signals, NULL, NULL, Setting_IsExtended, &settings.coolant_invert.mask, NULL, NULL },
     { Setting_SpindleInvertMask, Group_Spindle, "Invert spindle signals", NULL, Format_Bitfield, spindle_signals, NULL, NULL, Setting_IsExtendedFn, set_spindle_invert, get_int, NULL, { .reboot_required = On } },
     { Setting_ControlPullUpDisableMask, Group_ControlSignals, "Pullup disable control pins", NULL, Format_Bitfield, control_signals, NULL, NULL, Setting_IsExtendedFn, set_control_disable_pullup, get_int, NULL },
     { Setting_LimitPullUpDisableMask, Group_Limits, "Pullup disable limit pins", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_IsExtended, &settings.limits.disable_pullup.mask, NULL, NULL },
     { Setting_ProbePullUpDisable, Group_Probing, "Pullup disable probe pin", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsExtendedFn, set_probe_disable_pullup, get_int, is_setting_available },
     { Setting_SoftLimitsEnable, Group_Limits, "Soft limits enable", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsLegacyFn, set_soft_limits_enable, get_int, NULL },
#if COMPATIBILITY_LEVEL <= 1
     { Setting_HardLimitsEnable, Group_Limits, "Hard limits enable", NULL, Format_XBitfield, "Enable,Strict mode", NULL, NULL, Setting_IsExpandedFn, set_hard_limits_enable, get_int, NULL },
#else
     { Setting_HardLimitsEnable, Group_Limits, "Hard limits enable", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsLegacyFn, set_hard_limits_enable, get_int, NULL },
#endif
#if COMPATIBILITY_LEVEL <= 1
     { Setting_HomingEnable, Group_Homing, "Homing cycle", NULL, Format_XBitfield, "Enable,Enable single axis commands,Homing on startup required,Set machine origin to 0,Two switches shares one input pin,Allow manual,Override locks,Keep homed status on reset", NULL, NULL, Setting_IsExpandedFn, set_homing_enable, get_int, NULL },
#else
     { Setting_HomingEnable, Group_Homing, "Homing cycle enable", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsLegacyFn, set_homing_enable, get_int, NULL },
#endif
     { Setting_HomingDirMask, Group_Homing, "Homing direction invert", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_IsLegacy, &settings.homing.dir_mask.value, NULL, NULL },
     { Setting_HomingFeedRate, Group_Homing, "Homing locate feed rate", "mm/min", Format_Decimal, "#####0.0", NULL, NULL, Setting_IsLegacy, &settings.homing.feed_rate, NULL, NULL },
     { Setting_HomingSeekRate, Group_Homing, "Homing search seek rate", "mm/min", Format_Decimal, "#####0.0", NULL, NULL, Setting_IsLegacy, &settings.homing.seek_rate, NULL, NULL },
     { Setting_HomingDebounceDelay, Group_Homing, "Homing switch debounce delay", "milliseconds", Format_Int16, "##0", NULL, NULL, Setting_IsLegacy, &settings.homing.debounce_delay, NULL, NULL },
     { Setting_HomingPulloff, Group_Homing, "Homing switch pull-off distance", "mm", Format_Decimal, "#####0.000", NULL, NULL, Setting_IsLegacy, &settings.homing.pulloff, NULL, NULL },
     { Setting_G73Retract, Group_General, "G73 Retract distance", "mm", Format_Decimal, "#####0.000", NULL, NULL, Setting_IsExtended, &settings.g73_retract, NULL, NULL },
     { Setting_PulseDelayMicroseconds, Group_Stepper, "Pulse delay", "microseconds", Format_Decimal, "#0.0", NULL, "10", Setting_IsExtended, &settings.steppers.pulse_delay_microseconds, NULL, NULL },
     { Setting_RpmMax, Group_Spindle, "Maximum spindle speed", "RPM", Format_Decimal, "#####0.000", NULL, NULL, Setting_IsLegacy, &settings.spindle.rpm_max, NULL, is_setting_available },
     { Setting_RpmMin, Group_Spindle, "Minimum spindle speed", "RPM", Format_Decimal, "#####0.000", NULL, NULL, Setting_IsLegacy, &settings.spindle.rpm_min, NULL, is_setting_available },
     { Setting_Mode, Group_General, "Mode of operation", NULL, Format_RadioButtons, "Normal,Laser mode,Lathe mode", NULL, NULL, Setting_IsLegacyFn, set_mode, get_int, NULL },
     { Setting_PWMFreq, Group_Spindle, "Spindle PWM frequency", "Hz", Format_Decimal, "#####0", NULL, NULL, Setting_IsExtended, &settings.spindle.pwm_freq, NULL, is_setting_available },
     { Setting_PWMOffValue, Group_Spindle, "Spindle PWM off value", "percent", Format_Decimal, "##0.0", NULL, "100", Setting_IsExtended, &settings.spindle.pwm_off_value, NULL, is_setting_available },
     { Setting_PWMMinValue, Group_Spindle, "Spindle PWM min value", "percent", Format_Decimal, "##0.0", NULL, "100", Setting_IsExtended, &settings.spindle.pwm_min_value, NULL, is_setting_available },
     { Setting_PWMMaxValue, Group_Spindle, "Spindle PWM max value", "percent", Format_Decimal, "##0.0", NULL, "100", Setting_IsExtended, &settings.spindle.pwm_max_value, NULL, is_setting_available },
     { Setting_StepperDeenergizeMask, Group_Stepper, "Steppers deenergize", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_IsExtendedFn, set_stepper_deenergize_mask, get_int, NULL },
     { Setting_SpindlePPR, Group_Spindle, "Spindle pulses per revolution (PPR)", NULL, Format_Int16, "###0", NULL, NULL, Setting_IsExtended, &settings.spindle.ppr, NULL, is_setting_available, { .reboot_required = On } },
     { Setting_EnableLegacyRTCommands, Group_General, "Enable legacy RT commands", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsExtendedFn, set_enable_legacy_rt_commands, get_int, NULL },
     { Setting_JogSoftLimited, Group_Jogging, "Limit jog commands", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsExtendedFn, set_jog_soft_limited, get_int, NULL },
#ifndef NO_SAFETY_DOOR_SUPPORT
     { Setting_ParkingEnable, Group_SafetyDoor, "Parking cycle", NULL, Format_XBitfield, "Enable,Enable parking override control,Deactivate upon init", NULL, NULL, Setting_IsExtendedFn, set_parking_enable, get_int, is_setting_available },
     { Setting_ParkingAxis, Group_SafetyDoor, "Parking axis", NULL, Format_RadioButtons, "X,Y,Z", NULL, NULL, Setting_IsExtended, &settings.parking.axis, NULL, is_setting_available },
#endif
     { Setting_HomingLocateCycles, Group_Homing, "Homing passes", NULL, Format_Int8, "##0", "1", "128", Setting_IsExtended, &settings.homing.locate_cycles, NULL, NULL },
     { Setting_HomingCycle_1, Group_Homing, "Axes homing, first pass", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_IsExtendedFn, set_homing_cycle, get_int, NULL },
     { Setting_HomingCycle_2, Group_Homing, "Axes homing, second pass", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_IsExtendedFn, set_homing_cycle, get_int, NULL },
     { Setting_HomingCycle_3, Group_Homing, "Axes homing, third pass", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_IsExtendedFn, set_homing_cycle, get_int, NULL },
#ifdef A_AXIS
     { Setting_HomingCycle_4, Group_Homing, "Axes homing, fourth pass", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_IsExtendedFn, set_homing_cycle, get_int, NULL },
#endif
#ifdef B_AXIS
     { Setting_HomingCycle_5, Group_Homing, "Axes homing, fifth pass", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_IsExtendedFn, set_homing_cycle, get_int, NULL },
#endif
#ifdef C_AXIS
     { Setting_HomingCycle_6, Group_Homing, "Axes homing, sixth pass", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_IsExtendedFn, set_homing_cycle, get_int, NULL },
#endif
#ifndef NO_SAFETY_DOOR_SUPPORT
     { Setting_ParkingPulloutIncrement, Group_SafetyDoor, "Parking pull-out distance", "mm", Format_Decimal, "###0.0", NULL, NULL, Setting_IsExtended, &settings.parking.pullout_increment, NULL, is_setting_available },
     { Setting_ParkingPulloutRate, Group_SafetyDoor, "Parking pull-out rate", "mm/min", Format_Decimal, "###0.0", NULL, NULL, Setting_IsExtended, &settings.parking.pullout_rate, NULL, is_setting_available },
     { Setting_ParkingTarget, Group_SafetyDoor, "Parking target", "mm", Format_Decimal, "-###0.0", "-100000", NULL, Setting_IsExtended, &settings.parking.target, NULL, is_setting_available },
     { Setting_ParkingFastRate, Group_SafetyDoor, "Parking fast rate", "mm/min", Format_Decimal, "###0.0", NULL, NULL, Setting_IsExtended, &settings.parking.rate, NULL, is_setting_available },
     { Setting_RestoreOverrides, Group_General, "Restore overrides", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsExtendedFn, set_restore_overrides, get_int, is_setting_available },
     { Setting_DoorOptions, Group_SafetyDoor, "Safety door options", NULL, Format_Bitfield, "Ignore when idle,Keep coolant state on open", NULL, NULL, Setting_IsExtended, &settings.safety_door.flags.value, NULL, is_setting_available },
#endif
     { Setting_SleepEnable, Group_General, "Sleep enable", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsExtendedFn, set_sleep_enable, get_int, NULL },
     { Setting_HoldActions, Group_General, "Feed hold actions", NULL, Format_Bitfield, "Disable laser during hold,Restore spindle and coolant state on resume", NULL, NULL, Setting_IsExtendedFn, set_hold_actions, get_int, NULL },
     { Setting_ForceInitAlarm, Group_General, "Force init alarm", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsExtendedFn, set_force_initialization_alarm, get_int, NULL },
     { Setting_ProbingFeedOverride, Group_Probing, "Probing feed override", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsExtendedFn, set_probe_allow_feed_override, get_int, is_setting_available },
#if ENABLE_SPINDLE_LINEARIZATION
     { Setting_LinearSpindlePiece1, Group_Spindle, "Spindle linearisation, 1st point", NULL, Format_String, "x(39)", NULL, "39", Setting_IsExtendedFn, set_linear_piece, get_linear_piece, NULL },
  #if SPINDLE_NPWM_PIECES > 1
     { Setting_LinearSpindlePiece2, Group_Spindle, "Spindle linearisation, 2nd point", NULL, Format_String, "x(39)", NULL, "39", Setting_IsExtendedFn, set_linear_piece, get_linear_piece, NULL },
  #endif
  #if SPINDLE_NPWM_PIECES > 2
     { Setting_LinearSpindlePiece3, Group_Spindle, "Spindle linearisation, 3rd point", NULL, Format_String, "x(39)", NULL, "39", Setting_IsExtendedFn, set_linear_piece, get_linear_piece, NULL },
  #endif
  #if SPINDLE_NPWM_PIECES > 3
     { Setting_LinearSpindlePiece4, Group_Spindle, "Spindle linearisation, 4th point", NULL, Format_String, "x(39)", NULL, "39", Setting_IsExtendedFn, set_linear_piece, get_linear_piece, NULL },
  #endif
#endif
     { Setting_SpindlePGain, Group_Spindle_ClosedLoop, "Spindle P-gain", NULL, Format_Decimal, "###0.000", NULL, NULL, Setting_IsExtended, &settings.spindle.pid.p_gain, NULL, is_group_available },
     { Setting_SpindleIGain, Group_Spindle_ClosedLoop, "Spindle I-gain", NULL, Format_Decimal, "###0.000", NULL, NULL, Setting_IsExtended, &settings.spindle.pid.i_gain, NULL, is_group_available },
     { Setting_SpindleDGain, Group_Spindle_ClosedLoop, "Spindle D-gain", NULL, Format_Decimal, "###0.000", NULL, NULL, Setting_IsExtended, &settings.spindle.pid.d_gain, NULL, is_group_available },
     { Setting_SpindleMaxError, Group_Spindle_ClosedLoop, "Spindle PID max error", NULL, Format_Decimal, "###0.000", NULL, NULL, Setting_IsExtended, &settings.spindle.pid.max_error, NULL, is_group_available },
     { Setting_SpindleIMaxError, Group_Spindle_ClosedLoop, "Spindle PID max I error", NULL, Format_Decimal, "###0.000", NULL, NULL, Setting_IsExtended, &settings.spindle.pid.i_max_error, NULL, is_group_available },
     { Setting_PositionPGain, Group_Spindle_Sync, "Spindle sync P-gain", NULL, Format_Decimal, "###0.000", NULL, NULL, Setting_IsExtended, &settings.position.pid.p_gain, NULL, is_group_available },
     { Setting_PositionIGain, Group_Spindle_Sync, "Spindle sync I-gain", NULL, Format_Decimal, "###0.000", NULL, NULL, Setting_IsExtended, &settings.position.pid.i_gain, NULL, is_group_available },
     { Setting_PositionDGain, Group_Spindle_Sync, "Spindle sync D-gain", NULL, Format_Decimal, "###0.000", NULL, NULL, Setting_IsExtended, &settings.position.pid.d_gain, NULL, is_group_available },
     { Setting_PositionIMaxError, Group_Spindle_Sync, "Spindle sync PID max I error", NULL, Format_Decimal, "###0.000", NULL, NULL, Setting_IsExtended, &settings.position.pid.i_max_error, NULL, is_group_available },
     { Setting_AxisStepsPerMM, Group_Axis0, "-axis travel resolution", axis_steps, Format_Decimal, "#####0.000", NULL, NULL, Setting_IsLegacyFn, set_axis_setting, get_float, NULL, AXIS_OPTS },
     { Setting_AxisMaxRate, Group_Axis0, "-axis maximum rate", axis_rate, Format_Decimal, "#####0.000", NULL, NULL, Setting_IsLegacyFn, set_axis_setting, get_float, NULL, AXIS_OPTS },
     { Setting_AxisAcceleration, Group_Axis0, "-axis acceleration", axis_accel, Format_Decimal, "#####0.000", NULL, NULL, Setting_IsLegacyFn, set_axis_setting, get_float, NULL, AXIS_OPTS },
     { Setting_AxisMaxTravel, Group_Axis0, "-axis maximum travel", axis_dist, Format_Decimal, "#####0.000", NULL, NULL, Setting_IsLegacyFn, set_axis_setting, get_float, NULL, AXIS_OPTS },
#if ENABLE_BACKLASH_COMPENSATION
     { Setting_AxisBacklash, Group_Axis0, "-axis backlash compensation", axis_dist, Format_Decimal, "#####0.000", NULL, NULL, Setting_IsExtendedFn, set_axis_setting, get_float, NULL, AXIS_OPTS },
#endif
     { Setting_AxisAutoSquareOffset, Group_Axis0, "-axis dual axis offset", "mm", Format_Decimal, "-0.000", "-10", "10", Setting_IsExtendedFn, set_axis_setting, get_float, is_setting_available, AXIS_OPTS },
     { Setting_SpindleAtSpeedTolerance, Group_Spindle, "Spindle at speed tolerance", "percent", Format_Decimal, "##0.0", NULL, NULL, Setting_IsExtended, &settings.spindle.at_speed_tolerance, NULL, is_setting_available },
     { Setting_ToolChangeMode, Group_Toolchange, "Tool change mode", NULL, Format_RadioButtons, "Normal,Manual touch off,Manual touch off @ G59.3,Automatic touch off @ G59.3,Ignore M6", NULL, NULL, Setting_IsExtendedFn, set_tool_change_mode, get_int, NULL },
     { Setting_ToolChangeProbingDistance, Group_Toolchange, "Tool change probing distance", "mm", Format_Decimal, "#####0.0", NULL, NULL, Setting_IsExtendedFn, set_tool_change_probing_distance, get_float, NULL },
     { Setting_ToolChangeFeedRate, Group_Toolchange, "Tool change locate feed rate", "mm/min", Format_Decimal, "#####0.0", NULL, NULL, Setting_IsExtended, &settings.tool_change.feed_rate, NULL, NULL },
     { Setting_ToolChangeSeekRate, Group_Toolchange, "Tool change search seek rate", "mm/min", Format_Decimal, "#####0.0", NULL, NULL, Setting_IsExtended, &settings.tool_change.seek_rate, NULL, NULL },
     { Setting_ToolChangePulloffRate, Group_Toolchange, "Tool change probe pull-off rate", "mm/min", Format_Decimal, "#####0.0", NULL, NULL, Setting_IsExtended, &settings.tool_change.pulloff_rate, NULL, NULL },
     { Setting_ToolChangeRestorePosition, Group_Toolchange, "Restore position after M6", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsExtendedFn, set_tool_restore_pos, get_int, NULL },
     { Setting_DualAxisLengthFailPercent, Group_Limits_DualAxis, "Dual axis length fail", "percent", Format_Decimal, "##0.0", "0", "100", Setting_IsExtended, &settings.homing.dual_axis.fail_length_percent, NULL, is_setting_available },
     { Setting_DualAxisLengthFailMin, Group_Limits_DualAxis, "Dual axis length fail min", "mm", Format_Decimal, "#####0.000", NULL, NULL, Setting_IsExtended, &settings.homing.dual_axis.fail_distance_min, NULL, is_setting_available },
     { Setting_DualAxisLengthFailMax, Group_Limits_DualAxis, "Dual axis length fail max", "mm", Format_Decimal, "#####0.000", NULL, NULL, Setting_IsExtended, &settings.homing.dual_axis.fail_distance_max, NULL, is_setting_available },
#if COMPATIBILITY_LEVEL <= 1
     { Setting_DisableG92Persistence, Group_General, "Disable G92 persistence", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsExtendedFn, set_g92_disable_persistence, get_int, NULL },
#endif
#if !AXIS_REMAP_ABC2UVW
  #if N_AXIS == 4
   { Settings_Axis_Rotational, Group_Stepper, "Rotational axes", NULL, Format_Bitfield, "A-Axis", NULL, NULL, Setting_IsExtendedFn, set_rotational_axes, get_int, NULL },
  #elif N_AXIS == 5
   { Settings_Axis_Rotational, Group_Stepper, "Rotational axes", NULL, Format_Bitfield, "A-Axis,B-Axis", NULL, NULL, Setting_IsExtendedFn, set_rotational_axes, get_int, NULL },
  #elif N_AXIS == 6
   { Settings_Axis_Rotational, Group_Stepper, "Rotational axes", NULL, Format_Bitfield, "A-Axis,B-Axis,C-Axis", NULL, NULL, Setting_IsExtendedFn, set_rotational_axes, get_int, NULL },
  #elif N_AXIS == 7
   { Settings_Axis_Rotational, Group_Stepper, "Rotational axes", NULL, Format_Bitfield, "A-Axis,B-Axis,C-Axis,U-Axis", NULL, NULL, Setting_IsExtendedFn, set_rotational_axes, get_int, NULL },
  #elif N_AXIS == 8
   { Settings_Axis_Rotational, Group_Stepper, "Rotational axes", NULL, Format_Bitfield, "A-Axis,B-Axis,C-Axis,U-Axis,V-Axis", NULL, NULL, Setting_IsExtendedFn, set_rotational_axes, get_int, NULL },
  #endif
#else
  #if N_AXIS == 4
     { Settings_Axis_Rotational, Group_Stepper, "Rotational axes", NULL, Format_Bitfield, "U-Axis", NULL, NULL, Setting_IsExtendedFn, set_rotational_axes, get_int, NULL },
  #elif N_AXIS == 5
     { Settings_Axis_Rotational, Group_Stepper, "Rotational axes", NULL, Format_Bitfield, "U-Axis,V-Axis", NULL, NULL, Setting_IsExtendedFn, set_rotational_axes, get_int, NULL },
  #elif N_AXIS == 6
     { Settings_Axis_Rotational, Group_Stepper, "Rotational axes", NULL, Format_Bitfield, "U-Axis,V-Axis,W-Axis", NULL, NULL, Setting_IsExtendedFn, set_rotational_axes, get_int, NULL },
  #endif
#endif
#ifndef NO_SAFETY_DOOR_SUPPORT
     { Setting_DoorSpindleOnDelay, Group_SafetyDoor, "Spindle on delay", "s", Format_Decimal, "#0.0", "0.5", "20", Setting_IsExtended, &settings.safety_door.spindle_on_delay, NULL, is_setting_available },
     { Setting_DoorCoolantOnDelay, Group_SafetyDoor, "Coolant on delay", "s", Format_Decimal, "#0.0", "0.5", "20", Setting_IsExtended, &settings.safety_door.coolant_on_delay, NULL, is_setting_available },
#endif
     { Setting_SpindleOnDelay, Group_Spindle, "Spindle on delay", "s", Format_Decimal, "#0.0", "0.5", "20", Setting_IsExtended, &settings.safety_door.spindle_on_delay, NULL, is_setting_available },
     { Setting_SpindleType, Group_Spindle, "Default spindle", NULL, Format_RadioButtons, spindle_types, NULL, NULL, Setting_IsExtendedFn, set_spindle_type, get_int, is_setting_available },
     { Setting_PlannerBlocks, Group_General, "Planner buffer blocks", NULL, Format_Int16, "####0", "30", "1000", Setting_IsExtended, &settings.planner_buffer_blocks, NULL, NULL, { .reboot_required = On } },
     { Setting_AutoReportInterval, Group_General, "Autoreport interval", "ms", Format_Int16, "###0", "100", "1000", Setting_IsExtendedFn, set_report_interval, get_int, NULL, { .reboot_required = On, .allow_null = On } },
//     { Setting_TimeZoneOffset, Group_General, "Timezone offset", NULL, Format_Decimal, "-#0.00", "0", "12", Setting_IsExtended, &settings.timezone, NULL, NULL },
     { Setting_UnlockAfterEStop, Group_General, "Unlock required after E-Stop", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsExtendedFn, set_estop_unlock, get_int, is_setting_available },
#if NGC_EXPRESSIONS_ENABLE
     { Setting_NGCDebugOut, Group_General, "Output NGC debug messages", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsExtendedFn, set_ngc_debug_out, get_int, NULL },
#endif
};

#ifndef NO_SETTINGS_DESCRIPTIONS

PROGMEM static const setting_descr_t setting_descr[] = {
    { Setting_PulseMicroseconds, "Sets time length per step. Minimum 2 microseconds.\\n\\n"
                                 "This needs to be reduced from the default value of 10 when max. step rates exceed approximately 80 kHz."
    },
    { Setting_StepperIdleLockTime, "Sets a short hold delay when stopping to let dynamics settle before disabling steppers. Value 255 keeps motors enabled." },
    { Setting_StepInvertMask, "Inverts the step signals (active low)." },
    { Setting_DirInvertMask, "Inverts the direction signals (active low)." },
#if COMPATIBILITY_LEVEL <= 2
    { Setting_InvertStepperEnable, "Inverts the stepper driver enable signals. Most drivers uses active low enable requiring inversion.\\n\\n"
                                   "NOTE: If the stepper drivers shares the same enable signal only X is used."
    },
#else
    { Setting_InvertStepperEnable, "Inverts the stepper driver enable signals. Drivers using active high enable require inversion.\\n\\n" },
#endif
    { Setting_LimitPinsInvertMask, "Inverts the axis limit input signals." },
    { Setting_InvertProbePin, "Inverts the probe input pin signal." },
    { Setting_SpindlePWMOptions, "Enable controls PWM output availability.\\n"
                                 "When `RPM controls spindle enable signal` is checked and M3 or M4 is active S0 switches it off and S > 0 switches it on."
    },
    { Setting_GangedDirInvertMask, "Inverts the direction signals for the second motor used for ganged axes.\\n\\n"
                                   "NOTE: This inversion will be applied in addition to the inversion from setting $3."
    },
    { Setting_StatusReportMask, "Specifies optional data included in status reports.\\n"
                                "If Run substatus is enabled it may be used for simple probe protection.\\n\\n"
                                "NOTE: Parser state will be sent separately after the status report and only on changes."
    },
    { Setting_JunctionDeviation, "Sets how fast Grbl travels through consecutive motions. Lower value slows it down." },
    { Setting_ArcTolerance, "Sets the G2 and G3 arc tracing accuracy based on radial error. Beware: A very small value may effect performance." },
    { Setting_ReportInches, "Enables inch units when returning any position and rate value that is not a settings value." },
    { Setting_ControlInvertMask, "Inverts the control signals (active low).\\n"
                                 "NOTE: Block delete, Optional stop, EStop and Probe connected are optional signals, availability is driver dependent."
    },
    { Setting_CoolantInvertMask, "Inverts the coolant and mist signals (active low)." },
    { Setting_SpindleInvertMask, "Inverts the spindle on, counterclockwise and PWM signals (active low)." },
    { Setting_ControlPullUpDisableMask, "Disable the control signals pullup resistors. Potentially enables pulldown resistor if available.\\n"
                                        "NOTE: Block delete, Optional stop and EStop are optional signals, availability is driver dependent."
    },
    { Setting_LimitPullUpDisableMask, "Disable the limit signals pullup resistors. Potentially enables pulldown resistor if available."},
    { Setting_ProbePullUpDisable, "Disable the probe signal pullup resistor. Potentially enables pulldown resistor if available." },
    { Setting_SoftLimitsEnable, "Enables soft limits checks within machine travel and sets alarm when exceeded. Requires homing." },
    { Setting_HardLimitsEnable, "When enabled immediately halts motion and throws an alarm when a limit switch is triggered. In strict mode only homing is possible when a switch is engaged." },
    { Setting_HomingEnable, "Enables homing cycle. Requires limit switches on axes to be automatically homed.\\n\\n"
                            "When `Enable single axis commands` is checked, single axis homing can be performed by $H<axis letter> commands.\\n\\n"
                            "When `Allow manual` is checked, axes not homed automatically may be homed manually by $H or $H<axis letter> commands.\\n\\n"
                            "`Override locks` is for allowing a soft reset to disable `Homing on startup required`."
    },
    { Setting_HomingDirMask, "Homing searches for a switch in the positive direction. Set axis bit to search in negative direction." },
    { Setting_HomingFeedRate, "Feed rate to slowly engage limit switch to determine its location accurately." },
    { Setting_HomingSeekRate, "Seek rate to quickly find the limit switch before the slower locating phase." },
    { Setting_HomingDebounceDelay, "Sets a short delay between phases of homing cycle to let a switch debounce." },
    { Setting_HomingPulloff, "Retract distance after triggering switch to disengage it. Homing will fail if switch isn't cleared." },
    { Setting_G73Retract, "G73 retract distance (for chip breaking drilling)." },
    { Setting_PulseDelayMicroseconds, "Step pulse delay.\\n\\n"
                                      "Normally leave this at 0 as there is an implicit delay on direction changes when AMASS is active."
    },
    { Setting_RpmMax, "Maximum spindle speed, can be overridden by spindle plugins." },
    { Setting_RpmMin, "Minimum spindle speed, can be overridden by spindle plugins." },
    { Setting_Mode, "Laser mode: consecutive G1/2/3 commands will not halt when spindle speed is changed.\\n"
                    "Lathe mode: allows use of G7, G8, G96 and G97."
    },
    { Setting_PWMFreq, "Spindle PWM frequency." },
    { Setting_PWMOffValue, "Spindle PWM off value in percent (duty cycle)." },
    { Setting_PWMMinValue, "Spindle PWM min value in percent (duty cycle)." },
    { Setting_PWMMaxValue, "Spindle PWM max value in percent (duty cycle)." },
    { Setting_StepperDeenergizeMask, "Specifies which steppers not to disable when stopped." },
    { Setting_SpindlePPR, "Spindle encoder pulses per revolution." },
    { Setting_EnableLegacyRTCommands, "Enables \"normal\" processing of ?, ! and ~ characters when part of $-setting or comment. If disabled then they are added to the input string instead." },
    { Setting_JogSoftLimited, "Limit jog commands to machine limits for homed axes." },
    { Setting_ParkingEnable, "Enables parking cycle, requires parking axis homed." },
    { Setting_ParkingAxis, "Define which axis that performs the parking motion." },
    { Setting_HomingLocateCycles, "Number of homing passes. Minimum 1, maximum 128." },
    { Setting_HomingCycle_1, "Axes to home in first pass." },
    { Setting_HomingCycle_2, "Axes to home in second pass." },
    { Setting_HomingCycle_3, "Axes to home in third pass." },
#ifdef A_AXIS
    { Setting_HomingCycle_4, "Axes to home in fourth pass." },
#endif
#ifdef B_AXIS
    { Setting_HomingCycle_5, "Axes to home in fifth pass." },
#endif
#ifdef C_AXIS
    { Setting_HomingCycle_6, "Axes to home in sixth pass." },
#endif
    { Setting_JogStepSpeed, "Step jogging speed in millimeters per minute." },
    { Setting_JogSlowSpeed, "Slow jogging speed in millimeters per minute." },
    { Setting_JogFastSpeed, "Fast jogging speed in millimeters per minute." },
    { Setting_JogStepDistance, "Jog distance for single step jogging." },
    { Setting_JogSlowDistance, "Jog distance before automatic stop." },
    { Setting_JogFastDistance, "Jog distance before automatic stop." },
#ifndef NO_SAFETY_DOOR_SUPPORT
    { Setting_ParkingPulloutIncrement, "Spindle pull-out and plunge distance in mm.Incremental distance." },
    { Setting_ParkingPulloutRate, "Spindle pull-out/plunge slow feed rate in mm/min." },
    { Setting_ParkingTarget, "Parking axis target. In mm, as machine coordinate [-max_travel, 0]." },
    { Setting_ParkingFastRate, "Parking fast rate to target after pull-out in mm/min." },
    { Setting_RestoreOverrides, "Restore overrides to default values at program end." },
    { Setting_DoorOptions, "Enable this if it is desirable to open the safety door when in IDLE mode (eg. for jogging)." },
#endif
    { Setting_SleepEnable, "Enable sleep mode." },
    { Setting_HoldActions, "Actions taken during feed hold and on resume from feed hold." },
    { Setting_ForceInitAlarm, "Starts Grbl in alarm mode after a cold reset." },
    { Setting_ProbingFeedOverride, "Allow feed override during probing." },
#if ENABLE_SPINDLE_LINEARIZATION
     { Setting_LinearSpindlePiece1, "Comma separated list of values: RPM_MIN, RPM_LINE_A1, RPM_LINE_B1, set to blank to disable." },
  #if SPINDLE_NPWM_PIECES > 1
     { Setting_LinearSpindlePiece2, "Comma separated list of values: RPM_POINT12, RPM_LINE_A2, RPM_LINE_B2, set to blank to disable." },
  #endif
  #if SPINDLE_NPWM_PIECES > 2
     { Setting_LinearSpindlePiece3, "Comma separated list of values: RPM_POINT23, RPM_LINE_A3, RPM_LINE_B3, set to blank to disable." },
  #endif
  #if SPINDLE_NPWM_PIECES > 3
     { Setting_LinearSpindlePiece4, "Comma separated list of values: RPM_POINT34, RPM_LINE_A4, RPM_LINE_B4, set to blank to disable." },
  #endif
#endif
    { Setting_SpindlePGain, "" },
    { Setting_SpindleIGain, "" },
    { Setting_SpindleDGain, "" },
    { Setting_SpindleMaxError, "" },
    { Setting_SpindleIMaxError, "Spindle PID max integrator error." },
    { Setting_PositionPGain, "" },
    { Setting_PositionIGain, "" },
    { Setting_PositionDGain, "" },
    { Setting_PositionIMaxError, "Spindle sync PID max integrator error." },
    { Setting_AxisStepsPerMM, "Travel resolution in steps per millimeter." },
    { (setting_id_t)(Setting_AxisStepsPerMM + 1), "Travel resolution in steps per degree." }, // "Hack" to get correct description for rotary axes
    { Setting_AxisMaxRate, "Maximum rate. Used as G0 rapid rate." },
    { Setting_AxisAcceleration, "Acceleration. Used for motion planning to not exceed motor torque and lose steps." },
    { Setting_AxisMaxTravel, "Maximum axis travel distance from homing switch. Determines valid machine space for soft-limits and homing search distances." },
#if ENABLE_BACKLASH_COMPENSATION
    { Setting_AxisBacklash, "Backlash distance to compensate for." },
#endif
    { Setting_AxisAutoSquareOffset, "Offset between sides to compensate for homing switches inaccuracies." },
    { Setting_SpindleAtSpeedTolerance, "Spindle at speed tolerance as percentage deviation from programmed speed, set to 0 to disable.\\n"
                                       "If not within tolerance when checked after spindle on delay ($392) alarm 14 is raised."
    },
    { Setting_ToolChangeMode, "Normal: allows jogging for manual touch off. Set new position manually.\\n\\n"
                              "Manual touch off: retracts tool axis to home position for tool change, use jogging or $TPW for touch off.\\n\\n"
                              "Manual touch off @ G59.3: retracts tool axis to home position then to G59.3 position for tool change, use jogging or $TPW for touch off.\\n\\n"
                              "Automatic touch off @ G59.3: retracts tool axis to home position for tool change, then to G59.3 position for automatic touch off.\\n\\n"
                              "All modes except \"Normal\" and \"Ignore M6\" returns the tool (controlled point) to original position after touch off."
    },
    { Setting_ToolChangeProbingDistance, "Maximum probing distance for automatic or $TPW touch off." },
    { Setting_ToolChangeFeedRate, "Feed rate to slowly engage tool change sensor to determine the tool offset accurately." },
    { Setting_ToolChangeSeekRate, "Seek rate to quickly find the tool change sensor before the slower locating phase." },
    { Setting_ToolChangePulloffRate, "Pull-off rate for the retract move before the slower locating phase." },
    { Setting_ToolChangeRestorePosition, "When set the spindle is moved so that the controlled point (tool tip) is the same as before the M6 command, if not the spindle is only moved to the Z home position." },
    { Setting_DualAxisLengthFailPercent, "Dual axis length fail in percent of axis max travel." },
    { Setting_DualAxisLengthFailMin, "Dual axis length fail minimum distance." },
    { Setting_DualAxisLengthFailMax, "Dual axis length fail minimum distance." },
#if COMPATIBILITY_LEVEL <= 1
    { Setting_DisableG92Persistence, "Disables save/restore of G92 offset to non-volatile storage (NVS)." },
#endif
#ifndef NO_SAFETY_DOOR_SUPPORT
    { Setting_DoorSpindleOnDelay, "Delay to allow spindle to spin up after safety door is opened." },
    { Setting_DoorCoolantOnDelay, "Delay to allow coolant to restart after safety door is opened." },
#else
    { Setting_DoorSpindleOnDelay, "Delay to allow spindle to spin up when spindle at speed tolerance is > 0." },
#endif
    { Setting_SpindleType, "Spindle selected on startup." },
    { Setting_PlannerBlocks, "Number of blocks in the planner buffer." },
    { Setting_AutoReportInterval, "Interval the real time report will be sent, set to 0 to disable." },
    { Setting_TimeZoneOffset, "Offset in hours from UTC." },
    { Setting_UnlockAfterEStop, "If set unlock (by sending $X) is required after resetting a cleared E-Stop condition." },
#if NGC_EXPRESSIONS_ENABLE
    { Setting_NGCDebugOut, "Example: (debug, metric mode: #<_metric>, coord system: #5220)" },
#endif
};

#endif

static setting_details_t setting_details = {
    .groups = setting_group_detail,
    .n_groups = sizeof(setting_group_detail) / sizeof(setting_group_detail_t),
    .settings = setting_detail,
    .n_settings = sizeof(setting_detail) / sizeof(setting_detail_t),
#ifndef NO_SETTINGS_DESCRIPTIONS
    .descriptions = setting_descr,
    .n_descriptions = sizeof(setting_descr) / sizeof(setting_descr_t),
#endif
    .save = settings_write_global
};

// Acceleration override

static struct {
    bool valid;
    float acceleration[N_AXIS];
} override_backup = { .valid = false };

static void save_override_backup (void)
{
    uint_fast8_t idx = N_AXIS;

    do {
        idx--;
        override_backup.acceleration[idx] = settings.axis[idx].acceleration;
    } while(idx);

    override_backup.valid = true;
}

static void restore_override_backup (void)
{
    uint_fast8_t idx = N_AXIS;

    if(override_backup.valid) do {
        idx--;
        settings.axis[idx].acceleration = override_backup.acceleration[idx];
    } while(idx);
}

// Temporarily override acceleration, if 0 restore to setting value.
// Note: only allowed when current state is idle.
bool settings_override_acceleration (uint8_t axis, float acceleration)
{
    sys_state_t state = state_get();

    if(!(state == STATE_IDLE || (state & (STATE_HOMING|STATE_ALARM))))
        return false;

    if(acceleration <= 0.0f) {
        if(override_backup.valid)
            settings.axis[axis].acceleration = override_backup.acceleration[axis];
    } else {
        if(!override_backup.valid)
            save_override_backup();
        settings.axis[axis].acceleration = acceleration * 60.0f * 60.0f; // Limit max to setting value?
    }

    return true;
}

// ---

static setting_details_t *settingsd = &setting_details;

void settings_register (setting_details_t *details)
{
    settingsd->next = details;
    settingsd = details;
}

setting_details_t *settings_get_details (void)
{
    return &setting_details;
}

#if COMPATIBILITY_LEVEL > 2

static status_code_t set_enable_invert_mask (setting_id_t id, uint_fast16_t int_value)
{
    settings.steppers.enable_invert.mask = int_value ? 0 : AXES_BITMASK;

    return Status_OK;
}

#endif

#if COMPATIBILITY_LEVEL > 1

static status_code_t set_limits_invert_mask (setting_id_t id, uint_fast16_t int_value)
{
    settings.limits.invert.mask = (int_value ? ~(DEFAULT_LIMIT_SIGNALS_INVERT_MASK) : DEFAULT_LIMIT_SIGNALS_INVERT_MASK) & AXES_BITMASK;

    return Status_OK;
}

#endif

static status_code_t set_probe_invert (setting_id_t id, uint_fast16_t int_value)
{
    if(!hal.probe.configure)
        return Status_SettingDisabled;

    settings.probe.invert_probe_pin = int_value != 0;
    hal.probe.configure(false, false);

    return Status_OK;
}

static status_code_t set_ganged_dir_invert (setting_id_t id, uint_fast16_t int_value)
{
    if(!hal.stepper.get_ganged)
        return Status_SettingDisabled;

    settings.steppers.ganged_dir_invert.mask = int_value & hal.stepper.get_ganged(false).mask;

    return Status_OK;
}

static status_code_t set_stepper_deenergize_mask (setting_id_t id, uint_fast16_t int_value)
{
    settings.steppers.deenergize.mask = int_value;

    hal.stepper.enable(settings.steppers.deenergize);

    return Status_OK;
}

static status_code_t set_report_interval (setting_id_t setting, uint_fast16_t int_value)
{
    if((settings.report_interval = int_value) == 0)
        sys.flags.auto_reporting = Off;

    return Status_OK;
}

static status_code_t set_report_mask (setting_id_t id, uint_fast16_t int_value)
{
#if COMPATIBILITY_LEVEL <= 1
    settings.status_report.mask = int_value;
#else
    int_value &= 0b11;
    settings.status_report.mask = (settings.status_report.mask & ~0b11) | int_value;
#endif

    return Status_OK;
}

static status_code_t set_report_inches (setting_id_t id, uint_fast16_t int_value)
{
    settings.flags.report_inches = int_value != 0;
    report_init();
    system_flag_wco_change(); // Make sure WCO is immediately updated.

    return Status_OK;
}

#if NGC_EXPRESSIONS_ENABLE

static status_code_t set_ngc_debug_out (setting_id_t id, uint_fast16_t int_value)
{
    settings.flags.ngc_debug_out = int_value != 0;
    report_init();
    system_flag_wco_change(); // Make sure WCO is immediately updated.

    return Status_OK;
}

#endif

static status_code_t set_control_invert (setting_id_t id, uint_fast16_t int_value)
{
    settings.control_invert.mask = int_value & hal.signals_cap.mask;

    return Status_OK;
}

static status_code_t set_pwm_mode (setting_id_t id, uint_fast16_t int_value)
{
    settings.spindle.flags.enable_rpm_controlled = int_value != 0;

    return Status_OK;
}

static status_code_t set_pwm_options (setting_id_t id, uint_fast16_t int_value)
{
    if(int_value & 0x01) {
        if(int_value > 0b11)
            return Status_SettingValueOutOfRange;
        settings.spindle.flags.pwm_disable = Off;
        settings.spindle.flags.enable_rpm_controlled = (int_value & 0b10) >> 1;
    } else {
        settings.spindle.flags.pwm_disable = On;
        settings.spindle.flags.enable_rpm_controlled = Off;
    }

    return Status_OK;
}

static status_code_t set_spindle_type (setting_id_t id, uint_fast16_t int_value)
{
    if(spindle_get_count() < 2)
        return Status_SettingDisabled;
    else if(int_value >= spindle_get_count())
        return Status_SettingValueOutOfRange;

    settings.spindle.flags.type = int_value;

    spindle_select(settings.spindle.flags.type);

    return Status_OK;
}

static status_code_t set_spindle_invert (setting_id_t id, uint_fast16_t int_value)
{
    settings.spindle.invert.mask = int_value;
    if(settings.spindle.invert.pwm && !spindle_get_caps(false).pwm_invert) {
        settings.spindle.invert.pwm = Off;
        return Status_SettingDisabled;
    }

    return Status_OK;
}

static status_code_t set_control_disable_pullup (setting_id_t id, uint_fast16_t int_value)
{
    settings.control_disable_pullup.mask = int_value & hal.signals_cap.mask;

    return Status_OK;
}

static status_code_t set_probe_disable_pullup (setting_id_t id, uint_fast16_t int_value)
{
    if(!hal.probe.configure)
        return Status_SettingDisabled;

    settings.probe.disable_probe_pullup = int_value != 0;

    return Status_OK;
}

static status_code_t set_soft_limits_enable (setting_id_t id, uint_fast16_t int_value)
{
    if(int_value && !settings.homing.flags.enabled)
        return Status_SoftLimitError;

    settings.limits.flags.soft_enabled = int_value != 0;

    return Status_OK;
}

static status_code_t set_estop_unlock (setting_id_t id, uint_fast16_t int_value)
{
    if(!hal.signals_cap.e_stop)
        return Status_SettingDisabled;

    settings.flags.no_unlock_after_estop = int_value != 0;

    return Status_OK;
}

static status_code_t set_hard_limits_enable (setting_id_t id, uint_fast16_t int_value)
{
    settings.limits.flags.hard_enabled = bit_istrue(int_value, bit(0));
#if COMPATIBILITY_LEVEL <= 1
    settings.limits.flags.check_at_init = bit_istrue(int_value, bit(1));
#endif
    hal.limits.enable(settings.limits.flags.hard_enabled, false); // Change immediately. NOTE: Nice to have but could be problematic later.

    return Status_OK;
}

static status_code_t set_jog_soft_limited (setting_id_t id, uint_fast16_t int_value)
{
    if (int_value && !settings.homing.flags.enabled)
        return Status_SoftLimitError;

    settings.limits.flags.jog_soft_limited = int_value != 0;

    return Status_OK;
}

static status_code_t set_homing_enable (setting_id_t id, uint_fast16_t int_value)
{
    if (bit_istrue(int_value, bit(0))) {
#if COMPATIBILITY_LEVEL > 1
        settings.homing.flags.enabled = On;
        settings.homing.flags.init_lock = DEFAULT_HOMING_INIT_LOCK;
        settings.homing.flags.single_axis_commands = DEFAULT_HOMING_SINGLE_AXIS_COMMANDS;
        settings.homing.flags.force_set_origin = DEFAULT_HOMING_FORCE_SET_ORIGIN;
        settings.homing.flags.manual = DEFAULT_HOMING_ALLOW_MANUAL;
        settings.homing.flags.override_locks = DEFAULT_HOMING_OVERRIDE_LOCKS;
        settings.homing.flags.keep_on_reset = DEFAULT_HOMING_KEEP_STATUS_ON_RESET;
        settings.limits.flags.two_switches = DEFAULT_LIMITS_TWO_SWITCHES_ON_AXES;
#else
        settings.homing.flags.value = int_value & 0x0F;
        settings.limits.flags.two_switches = bit_istrue(int_value, bit(4));
        settings.homing.flags.manual = bit_istrue(int_value, bit(5));
        settings.homing.flags.override_locks = bit_istrue(int_value, bit(6));
        settings.homing.flags.keep_on_reset = bit_istrue(int_value, bit(7));
#endif
    } else {
        settings.homing.flags.value = 0;
        settings.limits.flags.soft_enabled = Off; // Force disable soft-limits.
        settings.limits.flags.jog_soft_limited = Off;
    }

    return Status_OK;
}

static status_code_t set_enable_legacy_rt_commands (setting_id_t id, uint_fast16_t int_value)
{
    settings.flags.legacy_rt_commands = int_value != 0;

    return Status_OK;
}

static status_code_t set_homing_cycle (setting_id_t id, uint_fast16_t int_value)
{
    settings.homing.cycle[id - Setting_HomingCycle_1].mask = int_value;
    limits_set_homing_axes();

    return Status_OK;
}

static status_code_t set_mode (setting_id_t id, uint_fast16_t int_value)
{
    switch((machine_mode_t)int_value) {

        case Mode_Standard:
           gc_state.modal.diameter_mode = false;
           break;

        case Mode_Laser:
            if(!spindle_get_caps(false).laser)
                return Status_SettingDisabledLaser;
            gc_state.modal.diameter_mode = false;
            break;

         case Mode_Lathe:
            break;

         default: // Mode_Standard
            return Status_InvalidStatement;
    }

    machine_mode_changed = true;
    settings.mode = (machine_mode_t)int_value;

    return Status_OK;
}

#ifndef NO_SAFETY_DOOR_SUPPORT

static status_code_t set_parking_enable (setting_id_t id, uint_fast16_t int_value)
{
    settings.parking.flags.value = bit_istrue(int_value, bit(0)) ? (int_value & 0x07) : 0;

    return Status_OK;
}

static status_code_t set_restore_overrides (setting_id_t id, uint_fast16_t int_value)
{
    settings.flags.restore_overrides = int_value != 0;

    return Status_OK;
}

#endif

static status_code_t set_sleep_enable (setting_id_t id, uint_fast16_t int_value)
{
    settings.flags.sleep_enable = int_value != 0;

    return Status_OK;
}

static status_code_t set_hold_actions (setting_id_t id, uint_fast16_t int_value)
{
    settings.flags.disable_laser_during_hold = bit_istrue(int_value, bit(0));
    settings.flags.restore_after_feed_hold = bit_istrue(int_value, bit(1));

    return Status_OK;
}

#if COMPATIBILITY_LEVEL <= 1
static status_code_t set_g92_disable_persistence (setting_id_t id, uint_fast16_t int_value)
{
    settings.flags.g92_is_volatile = int_value != 0;

    return Status_OK;
}
#endif

static status_code_t set_force_initialization_alarm (setting_id_t id, uint_fast16_t int_value)
{
    settings.flags.force_initialization_alarm = int_value != 0;

    return Status_OK;
}

static status_code_t set_probe_allow_feed_override (setting_id_t id, uint_fast16_t int_value)
{
    settings.probe.allow_feed_override = int_value != 0;

    return Status_OK;
}

static status_code_t set_tool_change_mode (setting_id_t id, uint_fast16_t int_value)
{
    if(!hal.driver_cap.atc && hal.stream.suspend_read && int_value <= ToolChange_Ignore) {
#if COMPATIBILITY_LEVEL > 1
        if((toolchange_mode_t)int_value == ToolChange_Manual_G59_3 || (toolchange_mode_t)int_value == ToolChange_SemiAutomatic)
            return Status_InvalidStatement;
#endif
        settings.tool_change.mode = (toolchange_mode_t)int_value;
        tc_init();
    } else
        return Status_InvalidStatement;

    return Status_OK;
}

static status_code_t set_tool_change_probing_distance (setting_id_t id, float value)
{
    if(hal.driver_cap.atc)
        return Status_InvalidStatement;

    settings.tool_change.probing_distance = value;

    return Status_OK;
}

static status_code_t set_tool_restore_pos (setting_id_t id, uint_fast16_t int_value)
{
    if(hal.driver_cap.atc)
        return Status_InvalidStatement;

    settings.flags.no_restore_position_after_M6 = int_value == 0;

    return Status_OK;
}

#if N_AXIS > 3

static status_code_t set_rotational_axes (setting_id_t id, uint_fast16_t int_value)
{
    settings.steppers.is_rotational.mask = (int_value << 3) & AXES_BITMASK;

    return Status_OK;
}

static inline bool axis_is_rotary (uint_fast8_t axis_idx)
{
    return bit_istrue(settings.steppers.is_rotational.mask, bit(axis_idx));
}

static void set_axis_setting_unit (const setting_detail_t *setting, uint_fast8_t axis_idx)
{
    bool is_rotary = axis_is_rotary(axis_idx);

    switch(setting->id) {

        case Setting_AxisStepsPerMM:
            strcpy((char *)setting->unit, is_rotary ? "step/deg" : "step/mm");
            break;

        case Setting_AxisMaxRate:
            strcpy((char *)setting->unit, is_rotary ? "deg/min" : "mm/min");
            break;

        case Setting_AxisAcceleration:
            strcpy((char *)setting->unit, is_rotary ? "deg/sec^2" : "mm/sec^2");
            break;

        case Setting_AxisMaxTravel:
        case Setting_AxisBacklash:
            strcpy((char *)setting->unit, is_rotary ? "deg" : "mm");
            break;

        default:
            break;
    }
}

#endif

#if ENABLE_SPINDLE_LINEARIZATION

static status_code_t set_linear_piece (setting_id_t id, char *svalue)
{
    uint32_t idx = id - Setting_LinearSpindlePiece1;
    float rpm, start, end;

    if(*svalue == '\0' || (svalue[0] == '0' && svalue[1] == '\0')) {
        settings.spindle.pwm_piece[idx].rpm = NAN;
        settings.spindle.pwm_piece[idx].start =
        settings.spindle.pwm_piece[idx].end = 0.0f;
    } else if(sscanf(svalue, "%f,%f,%f", &rpm, &start, &end) == 3) {
        settings.spindle.pwm_piece[idx].rpm = rpm;
        settings.spindle.pwm_piece[idx].start = start;
        settings.spindle.pwm_piece[idx].end = end;
//??       if(idx == 0)
//            settings.spindle.rpm_min = rpm;
    } else
        return Status_SettingValueOutOfRange;

    return Status_OK;
}

static char *get_linear_piece (setting_id_t id)
{
    static char buf[40];

    uint32_t idx = id - Setting_LinearSpindlePiece1;

    if(isnan(settings.spindle.pwm_piece[idx].rpm))
        *buf = '\0';
    else
        snprintf(buf, sizeof(buf), "%g,%g,%g", settings.spindle.pwm_piece[idx].rpm, settings.spindle.pwm_piece[idx].start, settings.spindle.pwm_piece[idx].end);

    return buf;
}

#endif

inline static setting_id_t normalize_id (setting_id_t id)
{
    if((id > Setting_AxisSettingsBase && id <= Setting_AxisSettingsMax) ||
       (id > Setting_AxisSettingsBase2 && id <= Setting_AxisSettingsMax2))
        id -= id % AXIS_SETTINGS_INCREMENT;
    else if(id > Setting_EncoderSettingsBase && id <= Setting_EncoderSettingsMax)
        id = (setting_id_t)(Setting_EncoderSettingsBase + (id % ENCODER_SETTINGS_INCREMENT));
    else if(id > Setting_ModbusTCPBase && id <= Setting_ModbusTCPMax)
        id = (setting_id_t)(Setting_ModbusTCPBase + (id % MODBUS_TCP_SETTINGS_INCREMENT));

    return id;
}

setting_id_t settings_get_axis_base (setting_id_t id, uint_fast8_t *idx)
{
    setting_id_t base = normalize_id(id);
    *idx = id - base;

    return *idx < N_AXIS ? base : Setting_SettingsMax;
}

static status_code_t set_axis_setting (setting_id_t setting, float value)
{
    uint_fast8_t idx;
    status_code_t status = Status_OK;

    switch(settings_get_axis_base(setting, &idx)) {

        case Setting_AxisStepsPerMM:
            if (hal.max_step_rate && value * settings.axis[idx].max_rate > (float)hal.max_step_rate * 60.0f)
                status = Status_MaxStepRateExceeded;
            else {
                if(settings.axis[idx].steps_per_mm > 0.0f && settings.axis[idx].steps_per_mm != value) {
                    float comp = value / settings.axis[idx].steps_per_mm;
                    sys.position[idx] *= comp;
                    sys.home_position[idx] *= comp;
                    sys.probe_position[idx] *= comp;
                    sys.tlo_reference[idx] *= comp;
                    sync_position();
                }
                settings.axis[idx].steps_per_mm = value;
            }
            break;

        case Setting_AxisMaxRate:
            if (hal.max_step_rate && value * settings.axis[idx].steps_per_mm > (float)hal.max_step_rate * 60.0f)
                status = Status_MaxStepRateExceeded;
            else
                settings.axis[idx].max_rate = value;
            break;

        case Setting_AxisAcceleration:
            settings.axis[idx].acceleration = override_backup.acceleration[idx] = value * 60.0f * 60.0f; // Convert to mm/sec^2 for grbl internal use.
            break;

        case Setting_AxisMaxTravel:
            if(settings.axis[idx].max_travel != -value) {
                bit_false(sys.homed.mask, bit(idx));
                system_add_rt_report(Report_Homed);
            }
            settings.axis[idx].max_travel = -value; // Store as negative for grbl internal use.
            if(settings.homing.flags.init_lock && (sys.homing.mask & sys.homed.mask) != sys.homing.mask) {
                system_raise_alarm(Alarm_HomingRequired);
                grbl.report.feedback_message(Message_HomingCycleRequired);
            }
            break;

        case Setting_AxisBacklash:
#if ENABLE_BACKLASH_COMPENSATION
            if(settings.axis[idx].backlash != value) {
                axes_signals_t axes;
                axes.mask = bit(idx);
                settings.axis[idx].backlash = value;
                mc_backlash_init(axes);
            }
#else
            status = Status_SettingDisabled;
#endif
            break;

        case Setting_AxisAutoSquareOffset:
            if(hal.stepper.get_ganged && bit_istrue(hal.stepper.get_ganged(true).mask, bit(idx)))
                settings.axis[idx].dual_axis_offset = value;
            else
                status = Status_SettingDisabled;
            break;

        default:
            status = Status_SettingDisabled;
            break;
    }

    return status;
}

static float get_float (setting_id_t setting)
{
    float value = 0.0f;

    if (setting >= Setting_AxisSettingsBase && setting <= Setting_AxisSettingsMax) {

        uint_fast8_t idx;

        switch(settings_get_axis_base(setting, &idx)) {

            case Setting_AxisStepsPerMM:
                value = settings.axis[idx].steps_per_mm;
                break;

            case Setting_AxisMaxRate:
                value = settings.axis[idx].max_rate;
                break;

            case Setting_AxisAcceleration:
                value = settings.axis[idx].acceleration  / (60.0f * 60.0f); // Convert to mm/min^2 for grbl internal use.
                break;

            case Setting_AxisMaxTravel:
                value = -settings.axis[idx].max_travel; // Store as negative for grbl internal use.
                break;

#if ENABLE_BACKLASH_COMPENSATION
            case Setting_AxisBacklash:
                value = settings.axis[idx].backlash;
                break;
#endif

            case Setting_AxisAutoSquareOffset:
                value = settings.axis[idx].dual_axis_offset;
                break;

            default: // for stopping compiler warning
                break;
        }
    } else switch(setting) {

        case Setting_ToolChangeProbingDistance:
            value = settings.tool_change.probing_distance;
            break;

        default:
            break;
    }

    return value;
}

static uint32_t get_int (setting_id_t id)
{
    uint32_t value = 0;

    switch(id) {

#if COMPATIBILITY_LEVEL > 2
        case Setting_InvertStepperEnable:
            value = settings.steppers.enable_invert.mask ? 0 : 1;
            break;
#endif

#if COMPATIBILITY_LEVEL > 1
        case Setting_LimitPinsInvertMask:
            value = settings.limits.invert.mask == DEFAULT_LIMIT_SIGNALS_INVERT_MASK ? 0 : 1;
            break;
#endif

        case Setting_SpindlePWMOptions:
            value = settings.spindle.flags.pwm_disable ? 0 : (settings.spindle.flags.enable_rpm_controlled ? 0b11 : 0b01);
            break;

        case Setting_Mode:
            value = settings.mode;
            break;

        case Setting_InvertProbePin:
            value = settings.probe.invert_probe_pin;
            break;

        case Setting_GangedDirInvertMask:
            value = settings.steppers.ganged_dir_invert.mask;
            break;

        case Setting_StatusReportMask:
#if COMPATIBILITY_LEVEL <= 1
            value = settings.status_report.mask;
#else
            value = settings.status_report.mask & 0b11;
#endif
            break;

        case Setting_ReportInches:
            value = settings.flags.report_inches;
            break;

        case Setting_ControlInvertMask:
            value = settings.control_invert.mask & hal.signals_cap.mask;
            break;

        case Setting_SpindleInvertMask:
            value = settings.spindle.invert.mask;
            break;

        case Setting_ControlPullUpDisableMask:
            value = settings.control_disable_pullup.mask & hal.signals_cap.mask;
            break;

        case Setting_ProbePullUpDisable:
            value = settings.probe.disable_probe_pullup;
            break;

        case Setting_SoftLimitsEnable:
            value = settings.limits.flags.soft_enabled;
            break;

        case Setting_HardLimitsEnable:
            value = ((settings.limits.flags.hard_enabled & bit(0)) ? bit(0) | (settings.limits.flags.check_at_init ? bit(1) : 0) : 0);
            break;

        case Setting_JogSoftLimited:
            value = settings.limits.flags.jog_soft_limited;
            break;

        case Setting_HomingEnable:
            value = (settings.homing.flags.value & 0x0F) |
                     (settings.limits.flags.two_switches ? bit(4) : 0) |
                      (settings.homing.flags.manual ? bit(5) : 0) |
                       (settings.homing.flags.override_locks ? bit(6) : 0) |
                        (settings.homing.flags.keep_on_reset ? bit(7) : 0);
            break;

        case Setting_StepperDeenergizeMask:
            value = settings.steppers.deenergize.mask;
            break;

        case Setting_EnableLegacyRTCommands:
            value = settings.flags.legacy_rt_commands;
            break;

        case Setting_ParkingEnable:
            value = settings.parking.flags.value;
            break;

        case Setting_HomingCycle_1:
        case Setting_HomingCycle_2:
        case Setting_HomingCycle_3:
        case Setting_HomingCycle_4:
        case Setting_HomingCycle_5:
        case Setting_HomingCycle_6:
            value = settings.homing.cycle[id - Setting_HomingCycle_1].mask;
            break;

        case Setting_RestoreOverrides:
            value = settings.flags.restore_overrides;
            break;

        case Setting_SleepEnable:
            value = settings.flags.sleep_enable;
            break;

        case Setting_HoldActions:
            value = (settings.flags.disable_laser_during_hold ? bit(0) : 0) | (settings.flags.restore_after_feed_hold ? bit(1) : 0);
            break;

        case Setting_ForceInitAlarm:
            value = settings.flags.force_initialization_alarm;
            break;

        case Setting_ProbingFeedOverride:
            value = settings.probe.allow_feed_override;
            break;

        case Setting_ToolChangeMode:
            value = settings.tool_change.mode;
            break;

        case Setting_ToolChangeRestorePosition:
            value = settings.flags.no_restore_position_after_M6 ? 0 : 1;
            break;

        case Setting_DisableG92Persistence:
            value = settings.flags.g92_is_volatile;
            break;

        case Setting_SpindleType:
            value = settings.spindle.flags.type;
            break;

        case Setting_AutoReportInterval:
            value = settings.report_interval;
            break;

#if N_AXIS > 3
        case Settings_Axis_Rotational:
            value = (settings.steppers.is_rotational.mask & AXES_BITMASK) >> 3;
            break;
#endif

        case Setting_UnlockAfterEStop:
            value = settings.flags.no_unlock_after_estop ? 0 : 1;
            break;

#if NGC_EXPRESSIONS_ENABLE
        case Setting_NGCDebugOut:
            value = settings.flags.ngc_debug_out;
            break;
#endif

        default:
            break;
    }

    return value;
}

inline static uint8_t get_decimal_places (const char *format)
{
    char *dp = format == NULL ? NULL : strchr(format, '.');

    return dp ? strchr(format, '\0') - dp - 1 : 1;
}

char *setting_get_value (const setting_detail_t *setting, uint_fast16_t offset)
{
    char *value = NULL;

    if(setting == NULL)
        return NULL;

    switch(setting->type) {

        case Setting_NonCore:
        case Setting_IsExtended:
        case Setting_IsLegacy:
        case Setting_IsExpanded:
            switch(setting->datatype) {

                case Format_Decimal:
                    value = ftoa(*((float *)(setting->value)), get_decimal_places(setting->format));
                    break;

                case Format_Int8:
                case Format_Bool:
                case Format_Bitfield:
                case Format_XBitfield:
                case Format_AxisMask:
                case Format_RadioButtons:
                    value = uitoa(*((uint8_t *)(setting->value)));
                    break;

                case Format_Int16:
                    value = uitoa(*((uint16_t *)(setting->value)));
                    break;

                case Format_Integer:
                    value = uitoa(*((uint32_t *)(setting->value)));
                    break;

                case Format_Password:
                    value = hal.stream.state.webui_connected ? PASSWORD_MASK : ((char *)(setting->value));
                    break;

                case Format_String:
                case Format_IPv4:
                    value = ((char *)(setting->value));
                    break;

                default:
                    break;
            }
            break;

        case Setting_NonCoreFn:
        case Setting_IsExtendedFn:
        case Setting_IsLegacyFn:
        case Setting_IsExpandedFn:;

            setting_id_t id = (setting_id_t)(setting->id + offset);

            switch(setting->datatype) {

                case Format_Decimal:
                    value = ftoa(((setting_get_float_ptr)(setting->get_value))(id), get_decimal_places(setting->format));
                    break;

                case Format_Password:
                    value = hal.stream.state.webui_connected ? "********" : ((setting_get_string_ptr)(setting->get_value))(id);
                    break;

                case Format_String:
                case Format_IPv4:
                    value = ((setting_get_string_ptr)(setting->get_value))(id);
                    break;

                default:
                    value = uitoa(((setting_get_int_ptr)(setting->get_value))(id));
                    break;
            }
            break;
    }

    return value;
}

uint32_t setting_get_int_value (const setting_detail_t *setting, uint_fast16_t offset)
{
    uint32_t value = 0;

    if(setting) switch(setting->type) {

        case Setting_NonCore:
        case Setting_IsExtended:
        case Setting_IsLegacy:
        case Setting_IsExpanded:
            switch(setting->datatype) {

                case Format_Int8:
                case Format_Bool:
                case Format_Bitfield:
                case Format_XBitfield:
                case Format_AxisMask:
                case Format_RadioButtons:
                    value = *((uint8_t *)(setting->value));
                    break;

                case Format_Int16:
                    value = *((uint16_t *)(setting->value));
                    break;

                case Format_Integer:
                    value = *((uint32_t *)(setting->value));
                    break;

                default:
                    break;
            }
            break;

        case Setting_NonCoreFn:
        case Setting_IsExtendedFn:
        case Setting_IsLegacyFn:
        case Setting_IsExpandedFn:
            switch(setting->datatype) {

                case Format_Decimal:
                case Format_String:
                case Format_Password:
                case Format_IPv4:
                    break;

                default:
                    value = ((setting_get_int_ptr)(setting->get_value))((setting_id_t)(setting->id + offset));
                    break;
            }
            break;
    }

    return value;
}

float setting_get_float_value (const setting_detail_t *setting, uint_fast16_t offset)
{
    float value = NAN;

    if(setting && setting->datatype == Format_Decimal) switch(setting->type) {

        case Setting_NonCore:
        case Setting_IsExtended:
        case Setting_IsLegacy:
        case Setting_IsExpanded:
            value = *((float *)(setting->value));
            break;

        case Setting_NonCoreFn:
        case Setting_IsExtendedFn:
        case Setting_IsLegacyFn:
        case Setting_IsExpandedFn:
            value = ((setting_get_float_ptr)(setting->get_value))((setting_id_t)(setting->id + offset));
            break;

        default:
            break;
    }

    return value;
}

static bool is_setting_available (const setting_detail_t *setting)
{
    bool available = false;

    if(setting) switch(normalize_id(setting->id)) {

        case Setting_GangedDirInvertMask:
            available = hal.stepper.get_ganged && hal.stepper.get_ganged(false).mask != 0;
            break;

        case Setting_InvertProbePin:
        case Setting_ProbePullUpDisable:
        case Setting_ProbingFeedOverride:
//        case Setting_ToolChangeProbingDistance:
//        case Setting_ToolChangeFeedRate:
//        case Setting_ToolChangeSeekRate:
            available = hal.probe.get_state != NULL;
            break;

        case Setting_SpindlePWMBehaviour:
            available = false;
            break;

        case Setting_SpindlePWMOptions:
            available = hal.driver_cap.pwm_spindle && spindle_get_caps(false).laser;
            break;

        case Setting_PWMFreq:
        case Setting_PWMOffValue:
        case Setting_PWMMinValue:
        case Setting_PWMMaxValue:
            available = hal.driver_cap.pwm_spindle;
            break;

        case Setting_SpindleType:
            available = spindle_get_count() > 1;
            break;

        case Setting_SpindlePPR:
            available = hal.driver_cap.spindle_sync || hal.driver_cap.spindle_pid;
            break;

        case Setting_RpmMax:
        case Setting_RpmMin:
            available = spindle_get_caps(false).variable;
            break;

        case Setting_DualAxisLengthFailPercent:
        case Setting_DualAxisLengthFailMin:
        case Setting_DualAxisLengthFailMax:
        case Setting_AxisAutoSquareOffset:
            available = hal.stepper.get_ganged && hal.stepper.get_ganged(true).mask != 0;
//            available = hal.stepper.get_ganged && bit_istrue(hal.stepper.get_ganged(true).mask, setting->id - Setting_AxisAutoSquareOffset);
            break;

#ifndef NO_SAFETY_DOOR_SUPPORT
        case Setting_ParkingEnable:
        case Setting_ParkingAxis:
        case Setting_ParkingPulloutIncrement:
        case Setting_ParkingPulloutRate:
        case Setting_ParkingTarget:
        case Setting_ParkingFastRate:
        case Setting_RestoreOverrides:
        case Setting_DoorOptions:
        case Setting_DoorSpindleOnDelay:
        case Setting_DoorCoolantOnDelay:
            available = hal.signals_cap.safety_door_ajar;
            break;
#endif

        case Setting_SpindleAtSpeedTolerance:
            available = spindle_get_caps(true).at_speed || hal.driver_cap.spindle_sync;
            break;

        case Setting_SpindleOnDelay:
            available = !hal.signals_cap.safety_door_ajar && spindle_get_caps(true).at_speed;
            break;

        case Setting_AutoReportInterval:
            available = hal.get_elapsed_ticks != NULL;
            break;

        case Setting_TimeZoneOffset:
            available = hal.rtc.set_datetime != NULL;
            break;

        case Setting_UnlockAfterEStop:
            available = hal.signals_cap.e_stop;
            break;

        default:
            break;
    }

    return available;
}

// Write build info to persistent storage
void settings_write_build_info (char *line)
{
    if(hal.nvs.type != NVS_None)
        hal.nvs.memcpy_to_nvs(NVS_ADDR_BUILD_INFO, (uint8_t *)line, sizeof(stored_line_t), true);
}

// Read build info from persistent storage.
bool settings_read_build_info(char *line)
{
    if (!(hal.nvs.type != NVS_None && hal.nvs.memcpy_from_nvs((uint8_t *)line, NVS_ADDR_BUILD_INFO, sizeof(stored_line_t), true) == NVS_TransferResult_OK)) {
        // Reset line with default value
        line[0] = 0; // Empty line
        settings_write_build_info(line);
        return false;
    }
    return true;
}

// Write startup line to persistent storage
void settings_write_startup_line (uint8_t idx, char *line)
{
    assert(idx < N_STARTUP_LINE);

#ifdef FORCE_BUFFER_SYNC_DURING_NVS_WRITE
    protocol_buffer_synchronize(); // A startup line may contain a motion and be executing.
#endif

    if(hal.nvs.type != NVS_None)
        hal.nvs.memcpy_to_nvs(NVS_ADDR_STARTUP_BLOCK + idx * (sizeof(stored_line_t) + NVS_CRC_BYTES), (uint8_t *)line, sizeof(stored_line_t), true);
}

// Read startup line to persistent storage.
bool settings_read_startup_line (uint8_t idx, char *line)
{
    assert(idx < N_STARTUP_LINE);

    if (!(hal.nvs.type != NVS_None && hal.nvs.memcpy_from_nvs((uint8_t *)line, NVS_ADDR_STARTUP_BLOCK + idx * (sizeof(stored_line_t) + NVS_CRC_BYTES), sizeof(stored_line_t), true) == NVS_TransferResult_OK)) {
        // Reset line with default value
        *line = '\0'; // Empty line
        settings_write_startup_line(idx, line);
        return false;
    }
    return true;
}

// Write selected coordinate data to persistent storage.
void settings_write_coord_data (coord_system_id_t id, float (*coord_data)[N_AXIS])
{
    assert(id <= N_CoordinateSystems);

#ifdef FORCE_BUFFER_SYNC_DURING_NVS_WRITE
    protocol_buffer_synchronize();
#endif

    if(hal.nvs.type != NVS_None)
        hal.nvs.memcpy_to_nvs(NVS_ADDR_PARAMETERS + id * (sizeof(coord_data_t) + NVS_CRC_BYTES), (uint8_t *)coord_data, sizeof(coord_data_t), true);
}

// Read selected coordinate data from persistent storage.
bool settings_read_coord_data (coord_system_id_t id, float (*coord_data)[N_AXIS])
{
    assert(id <= N_CoordinateSystems);

    if (!(hal.nvs.type != NVS_None && hal.nvs.memcpy_from_nvs((uint8_t *)coord_data, NVS_ADDR_PARAMETERS + id * (sizeof(coord_data_t) + NVS_CRC_BYTES), sizeof(coord_data_t), true) == NVS_TransferResult_OK)) {
        // Reset with default zero vector
        memset(coord_data, 0, sizeof(coord_data_t));
        settings_write_coord_data(id, coord_data);
        return false;
    }
    return true;
}

// Write selected tool data to persistent storage.
bool settings_write_tool_data (tool_data_t *tool_data)
{
#if N_TOOLS
    assert(tool_data->tool > 0 && tool_data->tool <= N_TOOLS); // NOTE: idx 0 is a non-persistent entry for tools not in tool table

    if(hal.nvs.type != NVS_None)
        hal.nvs.memcpy_to_nvs(NVS_ADDR_TOOL_TABLE + (tool_data->tool - 1) * (sizeof(tool_data_t) + NVS_CRC_BYTES), (uint8_t *)tool_data, sizeof(tool_data_t), true);

    return true;
#else
    return false;
#endif
}

// Read selected tool data from persistent storage.
bool settings_read_tool_data (uint32_t tool, tool_data_t *tool_data)
{
#if N_TOOLS
    assert(tool > 0 && tool <= N_TOOLS); // NOTE: idx 0 is a non-persistent entry for tools not in tool table

    if (!(hal.nvs.type != NVS_None && hal.nvs.memcpy_from_nvs((uint8_t *)tool_data, NVS_ADDR_TOOL_TABLE + (tool - 1) * (sizeof(tool_data_t) + NVS_CRC_BYTES), sizeof(tool_data_t), true) == NVS_TransferResult_OK && tool_data->tool == tool)) {
        memset(tool_data, 0, sizeof(tool_data_t));
        tool_data->tool = tool;
    }

    return tool_data->tool == tool;
#else
    return false;
#endif
}

// Read global settings from persistent storage.
// Checks version-byte of non-volatile storage and global settings copy.
bool read_global_settings ()
{
    bool ok = hal.nvs.type != NVS_None && SETTINGS_VERSION == hal.nvs.get_byte(0) && hal.nvs.memcpy_from_nvs((uint8_t *)&settings, NVS_ADDR_GLOBAL, sizeof(settings_t), true) == NVS_TransferResult_OK;

    // Sanity check of settings, board map could have been changed...
    if(settings.mode == Mode_Laser && !spindle_get_caps(false).laser)
        settings.mode = Mode_Standard;

    if(settings.spindle.flags.type >= spindle_get_count())
        settings.spindle.flags.type = 0;

    if(settings.planner_buffer_blocks < 30 || settings.planner_buffer_blocks > 1000)
        settings.planner_buffer_blocks = 35;

    if(!(hal.driver_cap.spindle_sync || hal.driver_cap.spindle_pid))
        settings.spindle.ppr = 0;

#if COMPATIBILITY_LEVEL > 1 && DEFAULT_DISABLE_G92_PERSISTENCE
    settings.flags.g92_is_volatile = On;
#endif

#if COMPATIBILITY_LEVEL > 2
    if(settings.steppers.enable_invert.mask)
        settings.steppers.enable_invert.mask = AXES_BITMASK;
#endif

    return ok && settings.version == SETTINGS_VERSION;
}


// Write global settings to persistent storage
void settings_write_global (void)
{
    if(override_backup.valid)
        restore_override_backup();

    settings.flags.compatibility_level = COMPATIBILITY_LEVEL;

    if(hal.nvs.type != NVS_None)
        hal.nvs.memcpy_to_nvs(NVS_ADDR_GLOBAL, (uint8_t *)&settings, sizeof(settings_t), true);
}


// Restore global settings to defaults and write to persistent storage
void settings_restore (settings_restore_t restore)
{
    uint_fast8_t idx;
    stored_line_t empty_line;

    memset(empty_line, 0xFF, sizeof(stored_line_t));
    *empty_line = '\0';

    hal.nvs.put_byte(0, SETTINGS_VERSION); // Forces write to physical storage

    if (restore.defaults) {

        memcpy(&settings, &defaults, sizeof(settings_t));

        settings.control_invert.mask &= hal.signals_cap.mask;
        settings.spindle.invert.ccw &= spindle_get_caps(false).direction;
        settings.spindle.invert.pwm &= spindle_get_caps(false).pwm_invert;
#if ENABLE_BACKLASH_COMPENSATION
        if(sys.driver_started)
            mc_backlash_init((axes_signals_t){AXES_BITMASK});
#endif
        settings_write_global();
    }

    if (restore.parameters) {
        float coord_data[N_AXIS];

        memset(coord_data, 0, sizeof(coord_data));
        for (idx = 0; idx <= N_WorkCoordinateSystems; idx++)
            settings_write_coord_data((coord_system_id_t)idx, &coord_data);

        settings_write_coord_data(CoordinateSystem_G92, &coord_data); // Clear G92 offsets

#if N_TOOLS
        tool_data_t tool_data;
        memset(&tool_data, 0, sizeof(tool_data_t));
        for (idx = 1; idx <= N_TOOLS; idx++) {
            tool_data.tool = idx;
            settings_write_tool_data(&tool_data);
        }
#endif
    }

    if (restore.startup_lines) {
        for (idx = 0; idx < N_STARTUP_LINE; idx++)
            settings_write_startup_line(idx, empty_line);
    }

    if (restore.build_info) {
        settings_write_build_info(empty_line);
        settings_write_build_info(BUILD_INFO);
    }

    setting_details_t *details = setting_details.next;

    if(details) do {
        if(details->restore)
            details->restore();
    } while((details = details->next));

    nvs_buffer_sync_physical();
}

inline static bool is_available (const setting_detail_t *setting)
{
    return setting->is_available == NULL || setting->is_available(setting);
}

static bool is_group_available (const setting_detail_t *setting)
{
    return settings_is_group_available(setting->group);
}

bool settings_is_group_available (setting_group_t id)
{
    const setting_group_detail_t *group = setting_get_group_details(id);

    if(!group)
        return false;

    bool available = group->is_available ? group->is_available(group) : false;

    if(!available) switch(group->id) {

        case Group_Probing:
            available = hal.probe.get_state != NULL;
            break;

        case Group_Spindle_Sync:
            available = hal.driver_cap.spindle_sync;
            break;

        case Group_Spindle_ClosedLoop:
            available = hal.driver_cap.spindle_pid;
            break;

        case Group_Limits_DualAxis:
            available = hal.stepper.get_ganged && hal.stepper.get_ganged(true).mask != 0;
            break;

        case Group_Homing:
        case Group_Jogging:
        case Group_Limits:
        case Group_ControlSignals:
        case Group_Spindle:
            available = true;
            break;

        default:
            {
                uint_fast16_t idx;
                setting_details_t *details = &setting_details;

                do {
                    if(details->settings) {
                        for(idx = 0; idx < details->n_settings; idx++) {
                            if(details->settings[idx].group == id && (available = is_available(&details->settings[idx])))
                                break;
                        }
                    }
                } while(!available && (details = details->next));
            }
            break;
    }

    return available;
}

setting_group_t settings_normalize_group (setting_group_t group)
{
    return (group > Group_Axis0 && group < Group_Axis0 + N_AXIS) ? Group_Axis0 : group;
}

bool settings_iterator (const setting_detail_t *setting, setting_output_ptr callback, void *data)
{
    bool ok = false;

    if(setting->group == Group_Axis0) {

        uint_fast8_t axis_idx = 0;

        for(axis_idx = 0; axis_idx < N_AXIS; axis_idx++) {
#if N_AXIS > 3
            set_axis_setting_unit(setting, axis_idx);
#endif
            if(callback(setting, axis_idx, data))
                ok = true;
        }
    } else if(setting->flags.increment) {
        setting_details_t *set;
        setting = setting_get_details(setting->id, &set);
        if(set->iterator)
            ok = set->iterator(setting, callback, data);
    } else
        ok = callback(setting, 0, data);

    return ok;
}

const setting_detail_t *setting_get_details (setting_id_t id, setting_details_t **set)
{
    uint_fast16_t idx, offset = id - normalize_id(id);
    setting_details_t *details = settings_get_details();

    id -= offset;

    do {
        for(idx = 0; idx < details->n_settings; idx++) {
            if(details->settings[idx].id == id && is_available(&details->settings[idx])) {
#if N_AXIS > 3
                if(details->settings[idx].group == Group_Axis0)
                    set_axis_setting_unit(&details->settings[idx], offset);
#endif
                if(offset && details->iterator == NULL && offset >= (details->settings[idx].group == Group_Encoder0 ? hal.encoder.get_n_encoders() : N_AXIS))
                    return NULL;
                if(set)
                    *set = details;
                return &details->settings[idx];
            }
        }
    } while((details = details->next));

    return NULL;
}

const char *setting_get_description (setting_id_t id)
{
    const char *description = NULL;

#ifndef NO_SETTINGS_DESCRIPTIONS

    uint_fast16_t idx;
    setting_details_t *settings = settings_get_details();
    const setting_detail_t *setting = setting_get_details(id, NULL);

    if(setting) do {
        if(settings->descriptions) {
            idx = settings->n_descriptions;
            do {
                if(settings->descriptions[--idx].id == setting->id) {
#if N_AXIS > 3
                    if(setting->id == Setting_AxisStepsPerMM && axis_is_rotary(id - setting->id))
                        idx++;
#endif
                    description = settings->descriptions[idx].description;
                }
            } while(idx && description == NULL);
        }
    } while(description == NULL && (settings = settings->next));

#endif

    return description;
}

const setting_group_detail_t *setting_get_group_details (setting_group_t id)
{
    uint_fast16_t idx;
    setting_details_t *details = settings_get_details();
    const setting_group_detail_t *detail = NULL;

    do {
        for(idx = 0; idx < details->n_groups; idx++) {
            if(details->groups[idx].id == id)
                detail = &details->groups[idx];
        }
    } while(detail == NULL && (details = details->next));

    return detail;
}

/*
setting_group_t setting_get_parent_group (setting_group_t id)
{
    const setting_group_detail_t *group = setting_get_group_details(id);

    return group ? group->parent : Group_Unknown;
}
*/

static status_code_t validate_value (const setting_detail_t *setting, float value)
{
    float val;
    uint_fast8_t set_idx = 0;

    if(setting->min_value) {
        if(!read_float((char *)setting->min_value, &set_idx, &val))
            return Status_BadNumberFormat;

        if(!(value >= val || (setting->flags.allow_null && value == 0.0f)))
            return Status_SettingValueOutOfRange;

    } else if(value < 0.0f)
        return Status_NegativeValue;

    if(setting->max_value) {
        set_idx = 0;

        if(!read_float((char *)setting->max_value, &set_idx, &val))
            return Status_BadNumberFormat;

        if(value > val)
            return Status_SettingValueOutOfRange;
    }

    return Status_OK;
}

static status_code_t validate_uint_value (const setting_detail_t *setting, uint32_t value)
{
    uint32_t val;
    uint_fast8_t set_idx = 0;
    status_code_t status;

    if(setting->min_value) {
        if((status = read_uint((char *)setting->min_value, &set_idx, &val)) != Status_OK)
            return status;

        if(!(value >= val || (setting->flags.allow_null && value == 0)))
            return Status_SettingValueOutOfRange;

    } else if(value < 0.0f)
        return Status_NegativeValue;

    if(setting->max_value) {
        set_idx = 0;

        if((status = read_uint((char *)setting->max_value, &set_idx, &val)) != Status_OK)
            return Status_BadNumberFormat;

        if(value > val)
            return Status_SettingValueOutOfRange;
    }

    return Status_OK;
}

static uint32_t strnumentries (const char *s, const char delimiter)
{
    if(s == NULL || *s == '\0')
        return 0;

    char *p = (char *)s;
    uint32_t entries = 1;

    while((p = strchr(p, delimiter))) {
        p++;
        entries++;
    }

    return entries;
}

setting_datatype_t setting_datatype_to_external (setting_datatype_t datatype)
{
    switch(datatype) {

        case Format_Int8:
        case Format_Int16:
            datatype = Format_Integer;
            break;

        default:
            break;
    }

    return datatype;
}

bool setting_is_list (const setting_detail_t *setting)
{
    return setting->datatype == Format_Bitfield || setting->datatype == Format_XBitfield || setting->datatype == Format_RadioButtons;
}

bool setting_is_integer (const setting_detail_t *setting)
{
    return setting->datatype == Format_Integer || setting->datatype == Format_Int8 || setting->datatype == Format_Int16;
}

static char *remove_element (char *s, uint_fast8_t entry)
{
    while(entry && *s) {
        if(*s == ',')
            entry--;
        s++;
    }

    if(entry == 0) {
        *s++ = 'N';
        *s++ = '/';
        *s++ = 'A';
        char *s2 = s;
        while(*s2 && *s2 != ',')
            s2++;
        while(*s2)
            *s++ = *s2++;
        *s = '\0';
    }

    return s;
}

static void setting_remove_element (setting_id_t id, uint_fast8_t pos)
{
    const setting_detail_t *setting = setting_get_details(id, NULL);

    if(setting && setting_is_list(setting))
        remove_element((char *)setting->format, pos);
}

// Flag setting elements for bitfields as N/A according to a mask
// Note: setting format string has to reside in RAM.
void setting_remove_elements (setting_id_t id, uint32_t mask)
{
    char *format = (char *)setting_get_details(id, NULL)->format, *s;
    uint_fast8_t idx, entries = strnumentries(format, ',');

    for(idx = 0; idx < entries; idx++ ) {
        if(!(mask & 0x1))
            setting_remove_element(id, idx);
        mask >>= 1;
    }

    // Strip trailing N/A's
    while((s = strrchr(format, ','))) {
        if(strncmp(s, ",N/A", 4))
            break;
        *s = '\0';
    }
}

inline static bool setting_is_string (setting_datatype_t  datatype)
{
    return datatype == Format_String || datatype == Format_Password || datatype == Format_IPv4;
}

inline static bool setting_is_core (setting_type_t type)
{
    return !(type == Setting_NonCore || type == Setting_NonCoreFn);
}

static status_code_t setting_validate_me_uint (const setting_detail_t *setting, char *svalue)
{
    uint_fast8_t idx = 0;
    uint32_t value;
    status_code_t status;

    if((status = read_uint(svalue, &idx, &value)) != Status_OK)
        return status;

    switch(setting->datatype) {

        case Format_Bool:
            if(!(value == 0 || value == 1))
                status = Status_SettingValueOutOfRange;
            break;

        case Format_Bitfield:
        case Format_XBitfield:;
            if(value >= (1UL << strnumentries(setting->format, ',')))
                status = Status_SettingValueOutOfRange;
            break;

        case Format_RadioButtons:
            if(value >= strnumentries(setting->format, ','))
                status = Status_SettingValueOutOfRange;
            break;

        case Format_AxisMask:
            if(value >= (1 << N_AXIS))
                status = Status_SettingValueOutOfRange;
            break;

        case Format_Int8:
        case Format_Int16:
        case Format_Integer:
            status = validate_uint_value(setting, value);
            break;

        default:
            break;
    }

    return status;
}

status_code_t setting_validate_me (const setting_detail_t *setting, float value, char *svalue)
{
    status_code_t status = Status_OK;

    switch(setting->datatype) {

        case Format_Bool:
        case Format_Bitfield:
        case Format_XBitfield:;
        case Format_RadioButtons:
        case Format_AxisMask:
        case Format_Int8:
        case Format_Int16:
        case Format_Integer:
            status = setting_validate_me_uint(setting, svalue);
            break;

        case Format_Decimal:
            status = validate_value(setting, value);
            break;

        case Format_Password:
            {
                uint_fast16_t len = strlen(svalue);
                if(hal.stream.state.webui_connected && len == strlen(PASSWORD_MASK) && !strcmp(PASSWORD_MASK, svalue))
                    status = Status_InvalidStatement;
                else
                    status = validate_value(setting, (float)len);
            }
            break;

        case Format_String:
            {
                uint_fast16_t len = strlen(svalue);
                status = validate_value(setting, (float)len);
            }
            break;

        case Format_IPv4:
            // handled by driver or plugin, dependent on network library
            break;
    }

    return status;
}

status_code_t setting_validate (setting_id_t id, float value, char *svalue)
{
    const setting_detail_t *setting = setting_get_details(id, NULL);

    // If no details available setting could nevertheless be a valid setting id.
    return setting == NULL ? Status_OK : setting_validate_me(setting, value, svalue);
}

static bool settings_changed_spindle (void)
{
    static spindle_settings_t spindle_settings = {0};

    bool changed;

    if((changed = memcmp(&spindle_settings, &settings.spindle, sizeof(spindle_settings_t))) != 0)
        memcpy(&spindle_settings, &settings.spindle, sizeof(spindle_settings_t));

    return changed;
}

// A helper method to set settings from command line
status_code_t settings_store_setting (setting_id_t id, char *svalue)
{
    uint_fast8_t set_idx = 0;
    uint32_t int_value = 0;
    float value = NAN;
    status_code_t status = Status_OK;
    setting_details_t *set;
    const setting_detail_t *setting = setting_get_details(id, &set);

    if(setting == NULL) {
        if(id == Setting_SpindlePWMBehaviour) {
            set = &setting_details;
            setting = &setting_detail[Setting_SpindlePWMBehaviour];
        } else
            return Status_SettingDisabled;
    }

    // Trim leading spaces
    while(*svalue == ' ')
        svalue++;

    if(setting->datatype == Format_Decimal)  {
        if(!read_float(svalue, &set_idx, &value) && setting_is_core(setting->type))
            return Status_BadNumberFormat;
    } else if(!setting_is_string(setting->datatype) && read_uint(svalue, &set_idx, &int_value) != Status_OK && setting_is_core(setting->type))
        return Status_BadNumberFormat;

    if((status = setting_validate_me(setting, value, svalue)) != Status_OK) {
        if(setting == Setting_PulseMicroseconds && status == Status_SettingValueOutOfRange)
            status =  Status_SettingStepPulseMin;

        return status;
    }

    switch(setting->type) {

        case Setting_NonCore:
        case Setting_IsExtended:
        case Setting_IsLegacy:
        case Setting_IsExpanded:
            switch(setting->datatype) {

                case Format_Decimal:
                    *((float *)(setting->value)) = value;
                    break;

                case Format_String:
                case Format_Password:
                    strcpy(((char *)(setting->value)), svalue);
                    break;

                case Format_AxisMask:
                    *((uint8_t *)(setting->value)) = (uint8_t)int_value & AXES_BITMASK;
                    break;

                case Format_Bool:
                case Format_Bitfield:
                case Format_XBitfield:
                case Format_RadioButtons:
                case Format_Int8:
                    *((uint8_t *)(setting->value)) = (uint8_t)int_value;
                    break;

                case Format_Int16:
                    *((uint16_t *)(setting->value)) = (uint16_t)int_value;
                    break;

                case Format_Integer:
                    *((uint32_t *)(setting->value)) = (uint32_t)int_value;
                    break;

                default:
                    status = Status_BadNumberFormat;
                    break;
            }
            break;

        case Setting_NonCoreFn:
        case Setting_IsExtendedFn:
        case Setting_IsLegacyFn:
        case Setting_IsExpandedFn:
            switch(setting->datatype) {

                case Format_Decimal:
                    status = ((setting_set_float_ptr)(setting->value))(id, value);
                    break;

                case Format_String:
                case Format_Password:
                case Format_IPv4:
                    status = ((setting_set_string_ptr)(setting->value))(id, svalue);
                    break;

                default:
                    status = ((setting_set_int_ptr)(setting->value))(id, (uint_fast16_t)int_value);
                    break;
            }
            break;
    }

    if(status == Status_OK) {

        if(set->save)
            set->save();

        if(set->on_changed) {

            settings_changed_flags_t changed = {0};

            changed.spindle = settings_changed_spindle() || machine_mode_changed;
            machine_mode_changed = false;

            set->on_changed(&settings, changed);
        }
    }

    return status;
}

bool settings_add_spindle_type (const char *type)
{
    bool ok;

    if((ok = strlen(spindle_types) + strlen(type) + 1 < sizeof(spindle_types))) {
        if(*spindle_types != '\0')
            strcat(spindle_types, ",");
        strcat(spindle_types, type);
    }

    return ok;
}

// Clear settings chain
void settings_clear (void)
{
    setting_details.next = NULL;
    settingsd = &setting_details;
}

// Initialize the config subsystem
void settings_init (void)
{
    settings_changed_flags_t changed = {0};

    if(!read_global_settings()) {

        settings_restore_t settings = settings_all;
        settings.defaults = 1; // Ensure global settings get restored

        if(hal.nvs.type != NVS_None)
            grbl.report.status_message(Status_SettingReadFail);

        settings_restore(settings); // Force restore all non-volatile storage data.
        report_init();
#if COMPATIBILITY_LEVEL <= 1
        report_grbl_settings(true, NULL);
#else
        report_grbl_settings(false, NULL);
#endif
        changed.spindle = settings_changed_spindle();
    } else {

        memset(&tool_table, 0, sizeof(tool_data_t)); // First entry is for tools not in tool table
#if N_TOOLS
        uint_fast8_t idx;
        for (idx = 1; idx <= N_TOOLS; idx++)
            settings_read_tool_data(idx, &tool_table[idx]);
#endif
        report_init();

        changed.spindle = settings_changed_spindle();

        hal.settings_changed(&settings, changed);

        if(hal.probe.configure) // Initialize probe invert mask.
            hal.probe.configure(false, false);
    }

    if(spindle_get_count() == 0)
        spindle_add_null();

    spindle_state_t spindle_cap = {
        .on = On,
    };

    spindle_cap.ccw = spindle_get_caps(false).direction;
    spindle_cap.pwm = spindle_get_caps(false).pwm_invert;

    setting_remove_elements(Setting_SpindleInvertMask, spindle_cap.mask);
    setting_remove_elements(Setting_ControlInvertMask, hal.signals_cap.mask);

    if(hal.stepper.get_ganged)
        setting_remove_elements(Setting_GangedDirInvertMask, hal.stepper.get_ganged(false).mask);

    if(!hal.driver_cap.mist_control)
        setting_remove_element(Setting_CoolantInvertMask, 1);

    setting_details_t *details = setting_details.next;

    if(details) do {
        if(details->load)
            details->load();
        if(details->on_changed)
            details->on_changed(&settings, changed);
    } while((details = details->next));

    setting_details.on_changed = hal.settings_changed;
}
