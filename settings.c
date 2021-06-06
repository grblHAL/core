/*
  settings.c - non-volatile storage configuration handling

  Part of grblHAL

  Copyright (c) 2017-2021 Terje Io
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
#include "defaults.h"
#include "limits.h"
#include "nvs_buffer.h"
#include "tool_change.h"
#include "state_machine.h"
#ifdef ENABLE_BACKLASH_COMPENSATION
#include "motion_control.h"
#endif
#ifdef ENABLE_SPINDLE_LINEARIZATION
#include <stdio.h>
#endif

#ifndef SETTINGS_RESTORE_DEFAULTS
#define SETTINGS_RESTORE_DEFAULTS          1
#endif
#ifndef SETTINGS_RESTORE_PARAMETERS
#define SETTINGS_RESTORE_PARAMETERS        1
#endif
#ifndef SETTINGS_RESTORE_STARTUP_LINES
#define SETTINGS_RESTORE_STARTUP_LINES     1
#endif
#ifndef SETTINGS_RESTORE_BUILD_INFO
#define SETTINGS_RESTORE_BUILD_INFO        1
#endif
#ifndef SETTINGS_RESTORE_DRIVER_PARAMETERS
#define SETTINGS_RESTORE_DRIVER_PARAMETERS 1
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

    .junction_deviation = DEFAULT_JUNCTION_DEVIATION,
    .arc_tolerance = DEFAULT_ARC_TOLERANCE,
    .g73_retract = DEFAULT_G73_RETRACT,

    .flags.legacy_rt_commands = DEFAULT_LEGACY_RTCOMMANDS,
    .flags.report_inches = DEFAULT_REPORT_INCHES,
    .flags.sleep_enable = DEFAULT_SLEEP_ENABLE,
#if DEFAULT_LASER_MODE
    .mode = Mode_Laser,
    .flags.disable_laser_during_hold = DEFAULT_DISABLE_LASER_DURING_HOLD,
#else
    .flags.disable_laser_during_hold = 0,
  #if DEFAULT_LATHE_MODE
    .mode = Mode_Lathe,
  #endif
#endif
    .flags.restore_after_feed_hold = DEFAULT_RESTORE_AFTER_FEED_HOLD,
    .flags.force_initialization_alarm = DEFAULT_FORCE_INITIALIZATION_ALARM,

    .probe.disable_probe_pullup = DISABLE_PROBE_PIN_PULL_UP,
    .probe.allow_feed_override = ALLOW_FEED_OVERRIDE_DURING_PROBE_CYCLES,
    .probe.invert_probe_pin = DEFAULT_INVERT_PROBE_PIN,

    .steppers.pulse_microseconds = DEFAULT_STEP_PULSE_MICROSECONDS,
    .steppers.pulse_delay_microseconds = DEFAULT_STEP_PULSE_DELAY,
    .steppers.idle_lock_time = DEFAULT_STEPPER_IDLE_LOCK_TIME,
    .steppers.step_invert.mask = DEFAULT_STEPPING_INVERT_MASK,
    .steppers.dir_invert.mask = DEFAULT_DIRECTION_INVERT_MASK,
    .steppers.enable_invert.mask = INVERT_ST_ENABLE_MASK,
    .steppers.deenergize.mask = ST_DEENERGIZE_MASK,
//    .steppers.is_rotational.mask = 0,
#if DEFAULT_HOMING_ENABLE
    .homing.flags.enabled = DEFAULT_HOMING_ENABLE,
    .homing.flags.init_lock = DEFAULT_HOMING_INIT_LOCK,
    .homing.flags.single_axis_commands = HOMING_SINGLE_AXIS_COMMANDS,
    .homing.flags.force_set_origin = HOMING_FORCE_SET_ORIGIN,
    .homing.flags.manual = DEFAULT_HOMING_ALLOW_MANUAL,
    .homing.flags.override_locks = DEFAULT_HOMING_OVERRIDE_LOCKS,
#else
    .homing.flags.value = 0,
#endif
    .homing.dir_mask.value = DEFAULT_HOMING_DIR_MASK,
    .homing.feed_rate = DEFAULT_HOMING_FEED_RATE,
    .homing.seek_rate = DEFAULT_HOMING_SEEK_RATE,
    .homing.debounce_delay = DEFAULT_HOMING_DEBOUNCE_DELAY,
    .homing.pulloff = DEFAULT_HOMING_PULLOFF,
    .homing.locate_cycles = DEFAULT_N_HOMING_LOCATE_CYCLE,
    .homing.cycle[0].mask = HOMING_CYCLE_0,
    .homing.cycle[1].mask = HOMING_CYCLE_1,
    .homing.cycle[2].mask = HOMING_CYCLE_2,
    .homing.dual_axis.fail_length_percent = DUAL_AXIS_HOMING_FAIL_AXIS_LENGTH_PERCENT,
    .homing.dual_axis.fail_distance_min = DUAL_AXIS_HOMING_FAIL_DISTANCE_MIN,
    .homing.dual_axis.fail_distance_max = DUAL_AXIS_HOMING_FAIL_DISTANCE_MAX,

    .status_report.machine_position = DEFAULT_REPORT_BUFFER_STATE,
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

    .limits.flags.hard_enabled = DEFAULT_HARD_LIMIT_ENABLE,
    .limits.flags.soft_enabled = DEFAULT_SOFT_LIMIT_ENABLE,
    .limits.flags.jog_soft_limited = DEFAULT_JOG_LIMIT_ENABLE,
    .limits.flags.check_at_init = DEFAULT_CHECK_LIMITS_AT_INIT,
    .limits.flags.two_switches = DEFAULT_LIMITS_TWO_SWITCHES_ON_AXES,
    .limits.invert.mask = INVERT_LIMIT_PIN_MASK,
    .limits.disable_pullup.mask = DISABLE_LIMIT_PINS_PULL_UP_MASK,

    .control_invert.mask = INVERT_CONTROL_PIN_MASK,
    .control_disable_pullup.mask = DISABLE_CONTROL_PINS_PULL_UP_MASK,

    .spindle.rpm_max = DEFAULT_SPINDLE_RPM_MAX,
    .spindle.rpm_min = DEFAULT_SPINDLE_RPM_MIN,
    .spindle.flags.pwm_action = DEFAULT_SPINDLE_PWM_ACTION,
    .spindle.invert.on = INVERT_SPINDLE_ENABLE_PIN,
    .spindle.invert.ccw = INVERT_SPINDLE_CCW_PIN,
    .spindle.invert.pwm = INVERT_SPINDLE_PWM_PIN,
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

    .coolant_invert.flood = INVERT_COOLANT_FLOOD_PIN,
    .coolant_invert.mist = INVERT_COOLANT_MIST_PIN,

    .axis[X_AXIS].steps_per_mm = DEFAULT_X_STEPS_PER_MM,
    .axis[X_AXIS].max_rate = DEFAULT_X_MAX_RATE,
    .axis[X_AXIS].acceleration = DEFAULT_X_ACCELERATION,
    .axis[X_AXIS].max_travel = (-DEFAULT_X_MAX_TRAVEL),
    .axis[X_AXIS].dual_axis_offset = 0.0f,
#ifdef ENABLE_BACKLASH_COMPENSATION
    .axis[X_AXIS].backlash = 0.0f,
#endif

    .axis[Y_AXIS].steps_per_mm = DEFAULT_Y_STEPS_PER_MM,
    .axis[Y_AXIS].max_rate = DEFAULT_Y_MAX_RATE,
    .axis[Y_AXIS].max_travel = (-DEFAULT_Y_MAX_TRAVEL),
    .axis[Y_AXIS].acceleration = DEFAULT_Y_ACCELERATION,
    .axis[Y_AXIS].dual_axis_offset = 0.0f,
#ifdef ENABLE_BACKLASH_COMPENSATION
    .axis[Y_AXIS].backlash = 0.0f,
#endif

    .axis[Z_AXIS].steps_per_mm = DEFAULT_Z_STEPS_PER_MM,
    .axis[Z_AXIS].max_rate = DEFAULT_Z_MAX_RATE,
    .axis[Z_AXIS].acceleration = DEFAULT_Z_ACCELERATION,
    .axis[Z_AXIS].max_travel = (-DEFAULT_Z_MAX_TRAVEL),
    .axis[Z_AXIS].dual_axis_offset = 0.0f,
#ifdef ENABLE_BACKLASH_COMPENSATION
    .axis[Z_AXIS].backlash = 0.0f,
#endif

#ifdef A_AXIS
    .axis[A_AXIS].steps_per_mm = DEFAULT_A_STEPS_PER_MM,
    .axis[A_AXIS].max_rate = DEFAULT_A_MAX_RATE,
    .axis[A_AXIS].acceleration = DEFAULT_A_ACCELERATION,
    .axis[A_AXIS].max_travel = (-DEFAULT_A_MAX_TRAVEL),
    .axis[A_AXIS].dual_axis_offset = 0.0f,
#ifdef ENABLE_BACKLASH_COMPENSATION
    .axis[A_AXIS].backlash = 0.0f,
#endif
    .homing.cycle[3].mask = HOMING_CYCLE_3,
#endif

#ifdef B_AXIS
    .axis[B_AXIS].steps_per_mm = DEFAULT_B_STEPS_PER_MM,
    .axis[B_AXIS].max_rate = DEFAULT_B_MAX_RATE,
    .axis[B_AXIS].acceleration = DEFAULT_B_ACCELERATION,
    .axis[B_AXIS].max_travel = (-DEFAULT_B_MAX_TRAVEL),
    .axis[B_AXIS].dual_axis_offset = 0.0f,
#ifdef ENABLE_BACKLASH_COMPENSATION
    .axis[B_AXIS].backlash = 0.0f,
#endif
    .homing.cycle[4].mask = HOMING_CYCLE_4,
#endif

#ifdef C_AXIS
    .axis[C_AXIS].steps_per_mm = DEFAULT_C_STEPS_PER_MM,
    .axis[C_AXIS].acceleration = DEFAULT_C_ACCELERATION,
    .axis[C_AXIS].max_rate = DEFAULT_C_MAX_RATE,
    .axis[C_AXIS].max_travel = (-DEFAULT_C_MAX_TRAVEL),
    .axis[C_AXIS].dual_axis_offset = 0.0f,
#ifdef ENABLE_BACKLASH_COMPENSATION
    .axis[C_AXIS].backlash = 0.0f,
#endif
    .homing.cycle[5].mask = HOMING_CYCLE_5,
#endif

#ifdef U_AXIS
    .axis[U_AXIS].steps_per_mm = DEFAULT_U_STEPS_PER_MM,
    .axis[U_AXIS].acceleration = DEFAULT_U_ACCELERATION,
    .axis[U_AXIS].max_rate = DEFAULT_U_MAX_RATE,
    .axis[U_AXIS].max_travel = (-DEFAULT_U_MAX_TRAVEL),
    .axis[U_AXIS].dual_axis_offset = 0.0f,
#ifdef ENABLE_BACKLASH_COMPENSATION
    .axis[U_AXIS].backlash = 0.0f,
#endif
#endif

#ifdef V_AXIS
    .axis[V_AXIS].steps_per_mm = DEFAULT_V_STEPS_PER_MM,
    .axis[V_AXIS].acceleration = DEFAULT_V_ACCELERATION,
    .axis[V_AXIS].max_rate = DEFAULT_V_MAX_RATE,
    .axis[V_AXIS].max_travel = (-DEFAULT_V_MAX_TRAVEL),
    .axis[V_AXIS].dual_axis_offset = 0.0f,
#ifdef ENABLE_BACKLASH_COMPENSATION
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
    .parking.pullout_increment = DEFAULT_PARKING_PULLOUT_INCREMENT
};

PROGMEM static const setting_group_detail_t setting_group_detail [] = {
    { Group_Root, Group_Root, "Root"},
    { Group_Root, Group_General, "General"},
    { Group_Root, Group_ControlSignals, "Control signals"},
    { Group_Root, Group_Limits, "Limits"},
    { Group_Limits, Group_Limits_DualAxis, "Dual axis"},
    { Group_Root, Group_Coolant, "Coolant"},
    { Group_Root, Group_Spindle, "Spindle"},
    { Group_Spindle, Group_Spindle_Sync, "Spindle sync"},
    { Group_Root, Group_Toolchange, "Tool change"},
    { Group_Root, Group_Homing, "Homing"},
    { Group_Root, Group_Probing, "Probing"},
    { Group_Root, Group_SafetyDoor, "Safety door"},
    { Group_Root, Group_Jogging, "Jogging"},
    { Group_Root, Group_Stepper, "Stepper"},
    { Group_Root, Group_MotorDriver, "Stepper driver"},
    { Group_Root, Group_Axis, "Axis"},
    { Group_Axis, Group_XAxis, "X-axis"},
    { Group_Axis, Group_YAxis, "Y-axis"},
    { Group_Axis, Group_ZAxis, "Z-axis"},
#ifdef A_AXIS
    { Group_Axis, Group_AAxis, "A-axis"},
#endif
#ifdef B_AXIS
    { Group_Axis, Group_BAxis, "B-axis"},
#endif
#ifdef C_AXIS
    { Group_Axis, Group_CAxis, "C-axis"},
#endif
#ifdef U_AXIS
    { Group_Axis, Group_UAxis, "U-axis"},
#endif
#ifdef V_AXIS
    { Group_Axis, Group_VAxis, "V-axis"}
#endif
};

static status_code_t set_probe_invert (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_report_mask (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_report_inches (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_control_invert (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_spindle_invert (setting_id_t id, uint_fast16_t int_value);
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
#ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
static status_code_t set_parking_enable (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_restore_overrides (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_door_options (setting_id_t id, uint_fast16_t int_value);
#endif
#ifdef ENABLE_SPINDLE_LINEARIZATION
static status_code_t set_linear_piece (setting_id_t id, char *svalue);
static char *get_linear_piece (setting_id_t id);
#endif
#if COMPATIBILITY_LEVEL > 1
static status_code_t set_limits_invert_mask (setting_id_t id, uint_fast16_t int_value);
#endif
static status_code_t set_axis_setting (setting_id_t setting, float value);
static float get_float (setting_id_t setting);
static uint32_t get_int (setting_id_t id);
static bool is_setting_available (const setting_detail_t *setting);

static char control_signals[] = "Reset,Feed hold,Cycle start,Safety door,Block delete,Optional stop,EStop,Probe connected,Motor fault";
static char control_signals_map[] = "0,1,2,3,4,5,6,7,8";
static char spindle_signals[] = "Spindle enable,Spindle direction,PWM";

PROGMEM static const setting_detail_t setting_detail[] = {
    { Setting_PulseMicroseconds, Group_Stepper, "Step pulse time", "microseconds", Format_Decimal, "#0.0", "2.0", NULL, Setting_IsLegacy, &settings.steppers.pulse_microseconds, NULL, NULL },
    { Setting_StepperIdleLockTime, Group_Stepper, "Step idle delay", "milliseconds", Format_Int16, "####0", NULL, "65535", Setting_IsLegacy, &settings.steppers.idle_lock_time, NULL, NULL },
    { Setting_StepInvertMask, Group_Stepper, "Step pulse invert", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_IsLegacy, &settings.steppers.step_invert.mask, NULL, NULL },
    { Setting_DirInvertMask, Group_Stepper, "Step direction invert", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_IsLegacy, &settings.steppers.dir_invert.mask, NULL, NULL },
    { Setting_InvertStepperEnable, Group_Stepper, "Invert step enable pin(s)", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_IsLegacy, &settings.steppers.enable_invert.mask, NULL, NULL },
#if COMPATIBILITY_LEVEL <= 1
    { Setting_LimitPinsInvertMask, Group_Limits, "Invert limit pins", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_IsLegacy, &settings.limits.invert.mask, NULL, NULL },
#else
    { Setting_LimitPinsInvertMask, Group_Limits, "Invert limit pins", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsLegacyFn, set_limits_invert_mask, get_int, NULL },
#endif
    { Setting_InvertProbePin, Group_Probing, "Invert probe pin", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsLegacyFn, set_probe_invert, get_int, NULL },
    { Setting_SpindlePWMBehaviour, Group_Spindle, "Disable spindle with zero speed", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsExtended, &settings.spindle.flags.mask, NULL, is_setting_available },
//    { Setting_SpindlePWMBehaviour, Group_Spindle, "Spindle enable vs. speed behaviour", NULL, Format_RadioButtons, "No action,Disable spindle with zero speed,Enable spindle with all speeds", NULL, NULL, Setting_IsExtended, &settings.spindle.flags.mask, NULL, NULL },
#if COMPATIBILITY_LEVEL <= 1
    { Setting_StatusReportMask, Group_General, "Status report options", NULL, Format_Bitfield, "Position in machine coordinate,Buffer state,Line numbers,Feed & speed,Pin state,Work coordinate offset,Overrides,Probe coordinates,Buffer sync on WCO change,Parser state,Alarm substatus,Run substatus", NULL, NULL, Setting_IsExtendedFn, set_report_mask, get_int, NULL },
#else
    { Setting_StatusReportMask, Group_General, "Status report options", NULL, Format_Bitfield, "Position in machine coordinate,Buffer state", NULL, NULL, Setting_IsLegacyFn, set_report_mask, get_int, NULL },
#endif
    { Setting_JunctionDeviation, Group_General, "Junction deviation", "mm", Format_Decimal, "#####0.000", NULL, NULL, Setting_IsLegacy, &settings.junction_deviation, NULL, NULL },
    { Setting_ArcTolerance, Group_General, "Arc tolerance", "mm", Format_Decimal, "#####0.000", NULL, NULL, Setting_IsLegacy, &settings.arc_tolerance, NULL, NULL },
    { Setting_ReportInches, Group_General, "Report in inches", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsLegacyFn, set_report_inches, get_int, NULL },
    { Setting_ControlInvertMask, Group_ControlSignals, "Invert control pins", NULL, Format_Bitfield, control_signals, control_signals_map, NULL, Setting_IsExpandedFn, set_control_invert, get_int, NULL },
    { Setting_CoolantInvertMask, Group_Coolant, "Invert coolant pins", NULL, Format_Bitfield, "Flood,Mist", NULL, NULL, Setting_IsExtended, &settings.coolant_invert.mask, NULL, NULL },
    { Setting_SpindleInvertMask, Group_Spindle, "Invert spindle signals", NULL, Format_Bitfield, spindle_signals, NULL, NULL, Setting_IsExtendedFn, set_spindle_invert, get_int, NULL },
    { Setting_ControlPullUpDisableMask, Group_ControlSignals, "Pullup disable control pins", NULL, Format_Bitfield, control_signals, control_signals_map, NULL, Setting_IsExtendedFn, set_control_disable_pullup, get_int, NULL },
    { Setting_LimitPullUpDisableMask, Group_Limits, "Pullup disable limit pins", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_IsExtended, &settings.limits.disable_pullup.mask, NULL, NULL },
    { Setting_ProbePullUpDisable, Group_Probing, "Pullup disable probe pin", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsExtendedFn, set_probe_disable_pullup, get_int, NULL },
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
    { Setting_StepperDeenergizeMask, Group_Stepper, "Steppers deenergize", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_IsExtended, &settings.steppers.deenergize.mask, NULL, NULL },
    { Setting_SpindlePPR, Group_Spindle, "Spindle pulses per revolution (PPR)", NULL, Format_Int16, "###0", NULL, NULL, Setting_IsExtended, &settings.spindle.ppr, NULL, is_setting_available },
    { Setting_EnableLegacyRTCommands, Group_General, "Enable legacy RT commands", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsExtendedFn, set_enable_legacy_rt_commands, get_int, NULL },
    { Setting_JogSoftLimited, Group_Jogging, "Limit jog commands", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsExtendedFn, set_jog_soft_limited, get_int, NULL },
#ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
    { Setting_ParkingEnable, Group_SafetyDoor, "Parking cycle", NULL, Format_XBitfield, "Enable,Enable parking override control,Deactivate upon init", NULL, NULL, Setting_IsExtendedFn, set_parking_enable, get_int, NULL },
    { Setting_ParkingAxis, Group_SafetyDoor, "Parking axis", NULL, Format_RadioButtons, "X,Y,Z", NULL, NULL, Setting_IsExtended, &settings.parking.axis, NULL, NULL },
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
#ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
    { Setting_ParkingPulloutIncrement, Group_SafetyDoor, "Parking pull-out distance", "mm", Format_Decimal, "###0.0", NULL, NULL, Setting_IsExtended, &settings.parking.pullout_increment, NULL, NULL },
    { Setting_ParkingPulloutRate, Group_SafetyDoor, "Parking pull-out rate", "mm/min", Format_Decimal, "###0.0", NULL, NULL, Setting_IsExtended, &settings.parking.pullout_rate, NULL, NULL },
    { Setting_ParkingTarget, Group_SafetyDoor, "Parking target", "mm", Format_Decimal, "-###0.0", "-100000", NULL, Setting_IsExtended, &settings.parking.target, NULL, NULL },
    { Setting_ParkingFastRate, Group_SafetyDoor, "Parking fast rate", "mm/min", Format_Decimal, "###0.0", NULL, NULL, Setting_IsExtended, &settings.parking.rate, NULL, NULL },
    { Setting_RestoreOverrides, Group_General, "Restore overrides", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsExtendedFn, set_restore_overrides, get_int, NULL },
    { Setting_DoorOptions, Group_SafetyDoor, "Safety door options", NULL, Format_Bitfield, "Ignore when idle,Keep coolant state on open", NULL, NULL, Setting_IsExtendedFn, set_door_options, get_int, NULL },
#endif
    { Setting_SleepEnable, Group_General, "Sleep enable", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsExtendedFn, set_sleep_enable, get_int, NULL },
    { Setting_HoldActions, Group_General, "Feed hold actions", NULL, Format_Bitfield, "Disable laser during hold,Restore spindle and coolant state on resume", NULL, NULL, Setting_IsExtendedFn, set_hold_actions, get_int, NULL },
    { Setting_ForceInitAlarm, Group_General, "Force init alarm", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsExtendedFn, set_force_initialization_alarm, get_int, NULL },
    { Setting_ProbingFeedOverride, Group_Probing, "Probing feed override", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsExtendedFn, set_probe_allow_feed_override, get_int, NULL },
#ifdef ENABLE_SPINDLE_LINEARIZATION
    { Setting_LinearSpindlePiece1, Group_Spindle, "Spindle linearisation, first point", NULL, Format_String, "x30", NULL, "30", Setting_IsExtendedFn, set_linear_piece, get_linear_piece, NULL },
    { Setting_LinearSpindlePiece2, Group_Spindle, "Spindle linearisation, second point", NULL, Format_String, "x30", NULL, "30", Setting_IsExtendedFn, set_linear_piece, get_linear_piece, NULL },
    { Setting_LinearSpindlePiece3, Group_Spindle, "Spindle linearisation, third point", NULL, Format_String, "x30", NULL, "30", Setting_IsExtendedFn, set_linear_piece, get_linear_piece, NULL },
    { Setting_LinearSpindlePiece4, Group_Spindle, "Spindle linearisation, fourth point", NULL, Format_String, "x30", NULL, "30", Setting_IsExtendedFn, set_linear_piece, get_linear_piece, NULL },
#endif
    { Setting_SpindlePGain, Group_Spindle_ClosedLoop, "Spindle P-gain", NULL, Format_Decimal, "###0.000", NULL, NULL, Setting_IsExtended, &settings.spindle.pid.p_gain, NULL, NULL },
    { Setting_SpindleIGain, Group_Spindle_ClosedLoop, "Spindle I-gain", NULL, Format_Decimal, "###0.000", NULL, NULL, Setting_IsExtended, &settings.spindle.pid.i_gain, NULL, NULL },
    { Setting_SpindleDGain, Group_Spindle_ClosedLoop, "Spindle D-gain", NULL, Format_Decimal, "###0.000", NULL, NULL, Setting_IsExtended, &settings.spindle.pid.d_gain, NULL, NULL },
    { Setting_SpindleMaxError, Group_Spindle_ClosedLoop, "Spindle PID max error", NULL, Format_Decimal, "###0.000", NULL, NULL, Setting_IsExtended, &settings.spindle.pid.max_error, NULL, NULL },
    { Setting_SpindleIMaxError, Group_Spindle_ClosedLoop, "Spindle PID max I error", NULL, Format_Decimal, "###0.000", NULL, NULL, Setting_IsExtended, &settings.spindle.pid.i_max_error, NULL, NULL },
    { Setting_PositionPGain, Group_Spindle_Sync, "Spindle sync P-gain", NULL, Format_Decimal, "###0.000", NULL, NULL, Setting_IsExtended, &settings.position.pid.p_gain, NULL, NULL },
    { Setting_PositionIGain, Group_Spindle_Sync, "Spindle sync I-gain", NULL, Format_Decimal, "###0.000", NULL, NULL, Setting_IsExtended, &settings.position.pid.i_gain, NULL, NULL },
    { Setting_PositionDGain, Group_Spindle_Sync, "Spindle sync D-gain", NULL, Format_Decimal, "###0.000", NULL, NULL, Setting_IsExtended, &settings.position.pid.d_gain, NULL, NULL },
    { Setting_PositionIMaxError, Group_Spindle_Sync, "Spindle sync PID max I error", NULL, Format_Decimal, "###0.000", NULL, NULL, Setting_IsExtended, &settings.position.pid.i_max_error, NULL, NULL },
    { Setting_AxisStepsPerMM, Group_Axis0, "?-axis travel resolution", "step/mm", Format_Decimal, "#####0.000", NULL, NULL, Setting_IsLegacyFn, set_axis_setting, get_float, NULL },
    { Setting_AxisMaxRate, Group_Axis0, "?-axis maximum rate", "mm/min", Format_Decimal, "#####0.000", NULL, NULL, Setting_IsLegacyFn, set_axis_setting, get_float, NULL },
    { Setting_AxisAcceleration, Group_Axis0, "?-axis acceleration", "mm/sec^2", Format_Decimal, "#####0.000", NULL, NULL, Setting_IsLegacyFn, set_axis_setting, get_float, NULL },
    { Setting_AxisMaxTravel, Group_Axis0, "?-axis maximum travel", "mm", Format_Decimal, "#####0.000", NULL, NULL, Setting_IsLegacyFn, set_axis_setting, get_float, NULL },
#ifdef ENABLE_BACKLASH_COMPENSATION
    { Setting_AxisBacklash, Group_Axis0, "?-axis backlash compensation", "mm", Format_Decimal, "#####0.000", NULL, NULL, Setting_IsExtendedFn, set_axis_setting, get_float, NULL, NULL },
#endif
    { Setting_AxisAutoSquareOffset, Group_Axis0, "?-axis dual axis offset", "mm", Format_Decimal, "-0.000", "-2", "2", Setting_IsExtendedFn, set_axis_setting, get_float, is_setting_available },
    { Setting_SpindleAtSpeedTolerance, Group_Spindle, "Spindle at speed tolerance", "percent", Format_Decimal, "##0.0", NULL, NULL, Setting_IsExtended, &settings.spindle.at_speed_tolerance, NULL, is_setting_available },
    { Setting_ToolChangeMode, Group_Toolchange, "Tool change mode", NULL, Format_RadioButtons, "Normal,Manual touch off,Manual touch off @ G59.3,Automatic touch off @ G59.3,Ignore M6", NULL, NULL, Setting_IsExtendedFn, set_tool_change_mode, get_int, NULL },
    { Setting_ToolChangeProbingDistance, Group_Toolchange, "Tool change probing distance", "mm", Format_Decimal, "#####0.0", NULL, NULL, Setting_IsExtendedFn, set_tool_change_probing_distance, get_float, NULL },
    { Setting_ToolChangeFeedRate, Group_Toolchange, "Tool change locate feed rate", "mm/min", Format_Decimal, "#####0.0", NULL, NULL, Setting_IsExtended, &settings.tool_change.feed_rate, NULL, NULL },
    { Setting_ToolChangeSeekRate, Group_Toolchange, "Tool change search seek rate", "mm/min", Format_Decimal, "#####0.0", NULL, NULL, Setting_IsExtended, &settings.tool_change.seek_rate, NULL, NULL },
    { Setting_ToolChangePulloffRate, Group_Toolchange, "Tool change probe pull-off rate", "mm/min", Format_Decimal, "#####0.0", NULL, NULL, Setting_IsExtended, &settings.tool_change.pulloff_rate, NULL, NULL },
    { Setting_DualAxisLengthFailPercent, Group_Limits_DualAxis, "Dual axis length fail", "percent", Format_Decimal, "##0.0", "0", "100", Setting_IsExtended, &settings.homing.dual_axis.fail_length_percent, NULL, NULL },
    { Setting_DualAxisLengthFailMin, Group_Limits_DualAxis, "Dual axis length fail min", "mm/min", Format_Decimal, "#####0.000", NULL, NULL, Setting_IsExtended, &settings.homing.dual_axis.fail_distance_min, NULL, NULL },
    { Setting_DualAxisLengthFailMax, Group_Limits_DualAxis, "Dual axis length fail max", "mm/min", Format_Decimal, "#####0.000", NULL, NULL, Setting_IsExtended, &settings.homing.dual_axis.fail_distance_max, NULL, NULL }
//    { Settings_Axis_Rotational, Group_Stepper, "Rotational axes", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_IsExtended, &settings.steppers.is_rotational.mask, NULL, NULL }
};

static setting_details_t details = {
    .groups = setting_group_detail,
    .n_groups = sizeof(setting_group_detail) / sizeof(setting_group_detail_t),
    .settings = setting_detail,
    .n_settings = sizeof(setting_detail) / sizeof(setting_detail_t),
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
    if(state_get() != STATE_IDLE)
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

setting_details_t *settings_get_details (void)
{
    details.on_get_settings = grbl.on_get_settings;

    return &details;
}

#if COMPATIBILITY_LEVEL > 1
static status_code_t set_limits_invert_mask (setting_id_t id, uint_fast16_t int_value)
{
    settings.limits.invert.mask = (int_value ? ~(INVERT_LIMIT_PIN_MASK) : INVERT_LIMIT_PIN_MASK) & AXES_BITMASK;

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

static status_code_t set_control_invert (setting_id_t id, uint_fast16_t int_value)
{
    settings.control_invert.mask = int_value & hal.signals_cap.mask;

    return Status_OK;
}

static status_code_t set_spindle_invert (setting_id_t id, uint_fast16_t int_value)
{
    settings.spindle.invert.mask = int_value;
    if(settings.spindle.invert.pwm && !hal.driver_cap.spindle_pwm_invert) {
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
    if (int_value && !settings.homing.flags.enabled)
        return Status_SoftLimitError;

    settings.limits.flags.soft_enabled = int_value != 0;

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
           settings.flags.disable_laser_during_hold = 0;
           gc_state.modal.diameter_mode = false;
           break;

        case Mode_Laser:
            if(!hal.driver_cap.variable_spindle)
                return Status_SettingDisabledLaser;
            if(settings.mode != Mode_Laser)
                settings.flags.disable_laser_during_hold = DEFAULT_DISABLE_LASER_DURING_HOLD;
            gc_state.modal.diameter_mode = false;
            break;

         case Mode_Lathe:
            settings.flags.disable_laser_during_hold = 0;
            break;

         default: // Mode_Standard
            return Status_InvalidStatement;
    }

    settings.mode = (machine_mode_t)int_value;

    return Status_OK;
}

#ifdef ENABLE_SAFETY_DOOR_INPUT_PIN

static status_code_t set_parking_enable (setting_id_t id, uint_fast16_t int_value)
{
    settings.parking.flags.value = bit_istrue(int_value, bit(0)) ? (int_value & 0x07) : 0;

    return Status_OK;
}

static status_code_t set_restore_overrides (setting_id_t id, uint_fast16_t int_value)
{
    settings.flags.restore_overrides = int_value != 0;;

    return Status_OK;
}

static status_code_t set_door_options (setting_id_t id, uint_fast16_t int_value)
{
    settings.flags.safety_door_ignore_when_idle = bit_istrue(int_value, bit(0));
    settings.flags.keep_coolant_state_on_door_open = bit_istrue(int_value, bit(1));

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

#ifdef ENABLE_SPINDLE_LINEARIZATION

static status_code_t set_linear_piece (setting_id_t id, char *svalue)
{
    uint32_t idx = id - Setting_LinearSpindlePiece1;
    float rpm, start, end;

    if(svalue[0] == '0' && svalue[1] == '\0') {
        settings.spindle.pwm_piece[idx].rpm = NAN;
        settings.spindle.pwm_piece[idx].start =
        settings.spindle.pwm_piece[idx].end = 0.0f;
    } else if(sscanf(svalue, "%f,%f,%f", &rpm, &start, &end) == 3) {
        settings.spindle.pwm_piece[idx].rpm = rpm;
        settings.spindle.pwm_piece[idx].start = start;
        settings.spindle.pwm_piece[idx].end = end;
    } else
        return Status_InvalidStatement;

    return Status_OK;
}

static char *get_linear_piece (setting_id_t id)
{
    static char buf[20];

    uint32_t idx = id - Setting_LinearSpindlePiece1;

    if(isnan(settings.spindle.pwm_piece[idx].rpm))
        strcpy(buf, ftoa(settings.spindle.pwm_piece[idx].rpm, N_DECIMAL_RPMVALUE));
    else {
        sprintf(buf, "$%d=%f,%f,%f" ASCII_EOL, (setting_id_t)(Setting_LinearSpindlePiece1 + idx), settings.spindle.pwm_piece[idx].rpm, settings.spindle.pwm_piece[idx].start, settings.spindle.pwm_piece[idx].end);
        hal.stream.write(buf);
    }

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
            else
                settings.axis[idx].steps_per_mm = value;
            break;

        case Setting_AxisMaxRate:
            if (hal.max_step_rate && value * settings.axis[idx].steps_per_mm > (float)hal.max_step_rate * 60.0f)
                status = Status_MaxStepRateExceeded;
            else
                settings.axis[idx].max_rate = value;
            break;

        case Setting_AxisAcceleration:
            settings.axis[idx].acceleration = override_backup.acceleration[idx] = value * 60.0f * 60.0f; // Convert to mm/min^2 for grbl internal use.
            break;

        case Setting_AxisMaxTravel:
            settings.axis[idx].max_travel = -value; // Store as negative for grbl internal use.
            break;

        case Setting_AxisBacklash:
#ifdef ENABLE_BACKLASH_COMPENSATION
            settings.axis[idx].backlash = value;
#else
            status = Status_SettingDisabled;
#endif
            break;

        case Setting_AxisAutoSquareOffset:
            if(hal.stepper.get_auto_squared && bit_istrue(hal.stepper.get_auto_squared().mask, bit(idx)))
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

#ifdef ENABLE_BACKLASH_COMPENSATION
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

#if COMPATIBILITY_LEVEL > 1
        case Setting_LimitPinsInvertMask:
            value = settings.limits.invert.mask == INVERT_LIMIT_PIN_MASK ? 0 : 1;
            break;
#endif

        case Setting_Mode:
            value = settings.mode;
            break;

        case Setting_InvertProbePin:
            value = settings.probe.invert_probe_pin;
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
            value = settings.control_invert.mask;
            break;

        case Setting_SpindleInvertMask:
            value = settings.spindle.invert.mask;
            break;

        case Setting_ControlPullUpDisableMask:
            value = settings.control_disable_pullup.mask;
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

        case Setting_DoorOptions:
            value = settings.flags.safety_door_ignore_when_idle | (settings.flags.keep_coolant_state_on_door_open << 1) ;
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
    setting_id_t id = (setting_id_t)(setting->id + offset);

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

                case Format_String:
                case Format_Password:
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
        case Setting_IsExpandedFn:
            switch(setting->datatype) {

                case Format_Decimal:
                    value = ftoa(((setting_get_float_ptr)(setting->get_value))(id), get_decimal_places(setting->format));
                    break;

                case Format_String:
                case Format_Password:
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

static bool is_setting_available (const setting_detail_t *setting)
{
    bool available = false; // settings_is_group_available(setting->group);

    switch(normalize_id(setting->id)) {

        case Setting_SpindlePWMBehaviour:
            available = hal.driver_cap.variable_spindle;
            break;

        case Setting_SpindlePPR:
            available = hal.driver_cap.spindle_sync || hal.driver_cap.spindle_pid;
            break;

        case Setting_SpindleAtSpeedTolerance:
            available = hal.driver_cap.spindle_at_speed;
            break;

        case Setting_RpmMax:
        case Setting_RpmMin:
        case Setting_PWMFreq:
        case Setting_PWMOffValue:
        case Setting_PWMMinValue:
        case Setting_PWMMaxValue:
            available = hal.driver_cap.variable_spindle;
            break;

        case Setting_AxisAutoSquareOffset:
            available = hal.stepper.get_auto_squared != NULL;
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
#ifdef N_TOOLS
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
#ifdef N_TOOLS
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

// Read Grbl global settings from persistent storage.
// Checks version-byte of non-volatile storage and global settings copy.
bool read_global_settings ()
{
    bool ok = hal.nvs.type != NVS_None && SETTINGS_VERSION == hal.nvs.get_byte(0) && hal.nvs.memcpy_from_nvs((uint8_t *)&settings, NVS_ADDR_GLOBAL, sizeof(settings_t), true) == NVS_TransferResult_OK;

    return ok && settings.version == SETTINGS_VERSION;
}


// Write Grbl global settings to persistent storage
void settings_write_global (void)
{
    if(override_backup.valid)
        restore_override_backup();

    if(hal.nvs.type != NVS_None)
        hal.nvs.memcpy_to_nvs(NVS_ADDR_GLOBAL, (uint8_t *)&settings, sizeof(settings_t), true);
}


// Restore Grbl global settings to defaults and write to persistent storage
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
        settings.spindle.invert.ccw &= hal.driver_cap.spindle_dir;
        settings.spindle.invert.pwm &= hal.driver_cap.spindle_pwm_invert;

        settings_write_global();
    }

    if (restore.parameters) {
        float coord_data[N_AXIS];

        memset(coord_data, 0, sizeof(coord_data));
        for (idx = 0; idx <= N_WorkCoordinateSystems; idx++)
            settings_write_coord_data((coord_system_id_t)idx, &coord_data);

        settings_write_coord_data(CoordinateSystem_G92, &coord_data); // Clear G92 offsets

#ifdef N_TOOLS
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

    setting_details_t *details = settings_get_details();

    while(details->on_get_settings) {
        details = details->on_get_settings();
        if(details->restore)
            details->restore();
    };

    nvs_buffer_sync_physical();
}

inline static bool is_available (const setting_detail_t *setting)
{
    return setting->is_available == NULL || setting->is_available(setting);
}

bool settings_is_group_available (setting_group_t group)
{
    bool available = false;

    switch(group) {

        case Group_Probing:
            available = hal.probe.get_state != NULL;
            break;

        case Group_Encoders:
        case Group_Encoder0:
            available = hal.encoder.get_n_encoders && hal.encoder.get_n_encoders() > 0;
            break;

        case Group_Spindle_Sync:
            available = hal.driver_cap.spindle_sync;
            break;

        case Group_Spindle_ClosedLoop:
            available = hal.driver_cap.spindle_pid;
            break;

        case Group_Limits_DualAxis:
            available = hal.stepper.get_auto_squared != NULL;
            break;

        case Group_General:
        case Group_Homing:
        case Group_Jogging:
        case Group_Limits:
        case Group_ControlSignals:
        case Group_Spindle:
        case Group_Axis:
        case Group_XAxis:
        case Group_YAxis:
        case Group_ZAxis:
        #ifdef A_AXIS
        case Group_AAxis:
        #endif
        #ifdef B_AXIS
        case Group_BAxis:
        #endif
        #ifdef C_AXIS
        case Group_CAxis:
        #endif
        #ifdef U_AXIS
        case Group_UAxis:
        #endif
        #ifdef V_AXIS
        case Group_VAxis:
        #endif
            available = true;
            break;

        default:
            {
                uint_fast16_t idx;
                setting_details_t *details = settings_get_details();
                do {
                    if(details->settings) {
                        for(idx = 0; idx < details->n_settings; idx++) {
                            if(details->settings[idx].group == group && is_available(&details->settings[idx])) {
                                available = true;
                                break;
                            }
                        }
                    }
                    details = !available && details->on_get_settings ? details->on_get_settings() : NULL;
                } while(details);
            }
            break;
    }

    return available;
}

setting_group_t settings_normalize_group (setting_group_t group)
{
    return (group > Group_Axis0 && group < Group_Axis0 + N_AXIS) ? Group_Axis0 : group;
}

/*
setting_group_t settings_get_parent_group (setting_group_t group)
{
    uint_fast16_t idx;
    setting_details_t *settings = settings_get_details();

    for(idx = 0; idx < settings->n_groups; idx++) {
        if(settings->groups[idx].id == group) {
            group = settings->groups[idx].parent;
            break;
        }
    }

    return group;
}
*/

bool settings_iterator (const setting_detail_t *setting, setting_output_ptr callback, void *data)
{
    bool ok = false;

    switch(setting->id) {

        case Setting_AxisStepsPerMM:
        case Setting_AxisMaxRate:
        case Setting_AxisAcceleration:
        case Setting_AxisMaxTravel:
        case Setting_AxisStepperCurrent:
        case Setting_AxisMicroSteps:
        case Setting_AxisBacklash:
        case Setting_AxisAutoSquareOffset:
        case Setting_AxisExtended0:
        case Setting_AxisExtended1:
        case Setting_AxisExtended2:
        case Setting_AxisExtended3:
        case Setting_AxisExtended4:
        case Setting_AxisExtended5:
        case Setting_AxisExtended6:
        case Setting_AxisExtended7:
        case Setting_AxisExtended8:
        case Setting_AxisExtended9:
            {
                uint_fast8_t axis_idx = 0;
                for(axis_idx = 0; axis_idx < N_AXIS; axis_idx++) {
                    if(callback(setting, axis_idx, data))
                        ok = true;
                }
            }
            break;

        case Setting_EncoderModeBase:
        case Setting_EncoderCPRBase:
        case Setting_EncoderCPDBase:
        case Setting_EncoderDblClickWindowBase:
            {
                uint_fast8_t encoder_idx = 0, n_encoders = hal.encoder.get_n_encoders();
                for(encoder_idx = 0; encoder_idx < n_encoders; encoder_idx++) {
                    if(callback(setting, encoder_idx * ENCODER_SETTINGS_INCREMENT, data))
                        ok = true;
                }
            }
            break;

        default:
            ok = callback(setting, 0, data);
            break;
    }

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
                if(offset && offset >= (details->settings[idx].group == Group_Encoder0 ? hal.encoder.get_n_encoders() : N_AXIS))
                    return NULL;
                if(set)
                    *set = details;
                return &details->settings[idx];
            }
        }
        details = details->on_get_settings ? details->on_get_settings() : NULL;
    } while(details);

    return NULL;
}

static status_code_t validate_value (const setting_detail_t *setting, float value)
{
    float val;
    uint_fast8_t set_idx = 0;

    if(setting->min_value) {
        if(!read_float((char *)setting->min_value, &set_idx, &val))
            return Status_BadNumberFormat;

        if(value < val)
            return Status_SettingValueOutOfRange;

    } else if(value < 0.0f)
        return Status_NegativeValue;

    if (setting->max_value) {
        set_idx = 0;

        if(!read_float((char *)setting->max_value, &set_idx, &val))
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

static char *remove_element (char *s, uint_fast8_t entry)
{
    if(entry) {
        while(*s && entry) {
            if(*s == ',' && --entry == 0)
                *s = '\0';
            else
                s++;
        }
    }

    if(entry == 0) {
        char *s2 = s + 1;
        if(*s2 == ',')
            s2++;
        else while(*s2 && *s2 != ',')
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

    if(setting && setting_is_list(setting)) {
        remove_element((char *)setting->format, pos);
        if(setting->min_value)
            remove_element((char *)setting->min_value, pos);
    }
}

inline static bool setting_is_string (setting_datatype_t  datatype)
{
    return datatype == Format_String || datatype == Format_Password || datatype == Format_IPv4;
}

inline static bool setting_is_core (setting_type_t  type)
{
    return !(type == Setting_NonCore || type == Setting_NonCoreFn);
}
/*
static uint32_t get_mask (const char *bits)
{
    uint32_t mask = 0;
    uint_fast8_t set_idx = 0;
    float value;

    while(read_float((char *)bits, &set_idx, &value)) {
        mask |= (1 << (uint8_t)value);
        if(bits[set_idx] == ',')
            set_idx++;
    }

    return mask;
}
*/
status_code_t setting_validate_me (const setting_detail_t *setting, float value, char *svalue)
{
    status_code_t status = Status_OK;

    switch(setting->datatype) {

        case Format_Bool:
            if(!(value == 0.0f || value == 1.0f))
                status = Status_SettingValueOutOfRange;
            break;

        case Format_Bitfield:
        case Format_XBitfield:;
            if(!(isintf(value) && /* (setting->min_value
                                   ? (((uint32_t)value & ~get_mask(setting->min_value)) == 0)
                                   : */ ((uint32_t)value < (1UL << strnumentries(setting->format, ','))))) //)
                status = Status_SettingValueOutOfRange;
            break;

        case Format_RadioButtons:
            if(!(isintf(value) && (uint32_t)value < strnumentries(setting->format, ',')))
                status = Status_SettingValueOutOfRange;
            break;

        case Format_AxisMask:
            if(!(isintf(value) && (uint32_t)value < (1 << N_AXIS)))
                status = Status_SettingValueOutOfRange;
            break;

        case Format_Int8:
        case Format_Int16:
        case Format_Integer:
        case Format_Decimal:
            status = validate_value(setting, value);
            if(setting->datatype == Format_Integer && status == Status_OK && !isintf(value))
                status = Status_BadNumberFormat;
            break;

        case Format_String:
        case Format_Password:
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

// A helper method to set settings from command line
status_code_t settings_store_setting (setting_id_t id, char *svalue)
{
    uint_fast8_t set_idx = 0;
    float value = NAN;
    status_code_t status = Status_OK;
    setting_details_t *set;
    const setting_detail_t *setting = setting_get_details(id, &set);

    if(setting == NULL)
        return Status_SettingDisabled;

    // Trim leading spaces
    while(*svalue == ' ')
        svalue++;

    if(!setting_is_string(setting->datatype) && !read_float(svalue, &set_idx, &value) && setting_is_core(setting->type))
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
                    *((uint8_t *)(setting->value)) = (uint8_t)truncf(value) & AXES_BITMASK;
                    break;

                case Format_Int8:
                case Format_Bool:
                case Format_Bitfield:
                case Format_XBitfield:
                case Format_RadioButtons:
                    *((uint8_t *)(setting->value)) = (uint8_t)truncf(value);
                    break;

                case Format_Int16:
                    *((uint16_t *)(setting->value)) = (uint16_t)truncf(value);
                    break;

                case Format_Integer:
                    *((uint32_t *)(setting->value)) = (uint32_t)truncf(value);
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
                    status = ((setting_set_int_ptr)(setting->value))(id, (uint_fast16_t)truncf(value));
                    break;
            }
            break;
    }

    if(status == Status_OK) {
        if(set->save)
            set->save();
#ifdef ENABLE_BACKLASH_COMPENSATION
  mc_backlash_init();
#endif
        if(set->on_changed)
            set->on_changed(&settings);
    }

    return status;
}

// Initialize the config subsystem
void settings_init (void)
{
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
    } else {
        memset(&tool_table, 0, sizeof(tool_data_t)); // First entry is for tools not in tool table
#ifdef N_TOOLS
        uint_fast8_t idx;
        for (idx = 1; idx <= N_TOOLS; idx++)
            settings_read_tool_data(idx, &tool_table[idx]);
#endif
        report_init();
#ifdef ENABLE_BACKLASH_COMPENSATION
        mc_backlash_init();
#endif
        hal.settings_changed(&settings);
        if(hal.probe.configure) // Initialize probe invert mask.
            hal.probe.configure(false, false);
    }

    if(!hal.driver_cap.spindle_pwm_invert)
        setting_remove_element(Setting_SpindleInvertMask, 2);

    if(!hal.signals_cap.motor_fault)
        setting_remove_element(Setting_ControlInvertMask, 8);
/* TODO
    if(!hal.signals_cap.probe_disconnected)
        setting_remove_element(Setting_ControlInvertMask, 7);

    if(!hal.signals_cap.e_stop)
        setting_remove_element(Setting_ControlInvertMask, 6);

    if(!hal.signals_cap.stop_disable)
        setting_remove_element(Setting_ControlInvertMask, 5);

    if(!hal.signals_cap.block_delete)
        setting_remove_element(Setting_ControlInvertMask, 4);

    if(!hal.signals_cap.safety_door_ajar)
        setting_remove_element(Setting_ControlInvertMask, 3);
*/

    setting_details_t *details = settings_get_details();

    while(details->on_get_settings) {
        details = details->on_get_settings();
        if(details->load)
            details->load();
        if(details->on_changed)
            details->on_changed(&settings);
    };

    settings_get_details()->on_changed = hal.settings_changed;
}
