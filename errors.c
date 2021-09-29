/*
  errors.c -

  Part of grblHAL

  Copyright (c) 2017-2021 Terje Io
  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
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

#include <stdint.h>

#include "grbl.h"
#include "core_handlers.h"

PROGMEM static const status_detail_t status_detail[] = {
    { Status_OK, "ok", NULL },
    { Status_ExpectedCommandLetter, "Expected command letter", "G-code words consist of a letter and a value. Letter was not found." },
    { Status_BadNumberFormat, "Bad number format", "Missing the expected G-code word value or numeric value format is not valid." },
    { Status_InvalidStatement, "Invalid statement", "'$' system command was not recognized or supported." },
    { Status_NegativeValue, "Value < 0", "Negative value received for an expected positive value." },
    { Status_HomingDisabled, "Homing disabled", "Homing cycle failure. Homing is not configured via settings." },
    { Status_SettingStepPulseMin, "Value < 2 microseconds", "Step pulse time must be greater or equal to 2 microseconds." },
    { Status_SettingReadFail, "Settings read failed. Using defaults", "A settings read failed. Auto-restoring affected settings to default values." },
    { Status_IdleError, "Not idle", "'$' command cannot be used unless controller state is IDLE. Ensures smooth operation during a job." },
    { Status_SystemGClock, "G-code lock", "G-code commands are locked out during alarm or jog state." },
    { Status_SoftLimitError, "Homing not enabled", "Soft limits cannot be enabled without homing also enabled." },
    { Status_Overflow, "Line overflow", "Max characters per line exceeded. Received command line was not executed." },
    { Status_MaxStepRateExceeded, "Max step rate too high", "'$' setting value cause the step rate to exceed the maximum supported." },
    { Status_CheckDoor, "Check Door", "Safety door detected as opened and door state initiated." },
    { Status_LineLengthExceeded, "Line length exceeded", "Build info or startup line exceeded line length limit. Line not stored." },
    { Status_TravelExceeded, "Travel exceeded", "Jog target exceeds machine travel. Jog command has been ignored." },
    { Status_InvalidJogCommand, "Invalid jog command", "Jog command has no '=' or contains prohibited g-code." },
    { Status_SettingDisabledLaser, "Setting disabled", "Laser mode requires PWM output." },
    { Status_Reset, "Reset asserted", "" },
    { Status_NonPositiveValue, "Non positive value", "" },
    { Status_GcodeUnsupportedCommand, "Unsupported command", "Unsupported or invalid g-code command found in block." },
    { Status_GcodeModalGroupViolation, "Modal group violation", "More than one g-code command from same modal group found in block." },
    { Status_GcodeUndefinedFeedRate, "Undefined feed rate", "Feed rate has not yet been set or is undefined." },
    { Status_GcodeCommandValueNotInteger, "Invalid gcode ID:23", "G-code command in block requires an integer value." },
    { Status_GcodeAxisCommandConflict, "Invalid gcode ID:24", "More than one g-code command that requires axis words found in block." },
    { Status_GcodeWordRepeated, "Invalid gcode ID:25", "Repeated g-code word found in block." },
    { Status_GcodeNoAxisWords, "Invalid gcode ID:26", "No axis words found in block for g-code command or current modal state which requires them." },
    { Status_GcodeInvalidLineNumber, "Invalid gcode ID:27", "Line number value is invalid." },
    { Status_GcodeValueWordMissing, "Invalid gcode ID:28", "G-code command is missing a required value word." },
    { Status_GcodeUnsupportedCoordSys, "Invalid gcode ID:29", "G59.x work coordinate systems are not supported." },
    { Status_GcodeG53InvalidMotionMode, "Invalid gcode ID:30", "G53 only allowed with G0 and G1 motion modes." },
    { Status_GcodeAxisWordsExist, "Invalid gcode ID:31", "Axis words found in block when no command or current modal state uses them." },
    { Status_GcodeNoAxisWordsInPlane, "Invalid gcode ID:32", "G2 and G3 arcs require at least one in-plane axis word." },
    { Status_GcodeInvalidTarget, "Invalid gcode ID:33", "Motion command target is invalid." },
    { Status_GcodeArcRadiusError, "Invalid gcode ID:34", "Arc radius value is invalid." },
    { Status_GcodeNoOffsetsInPlane, "Invalid gcode ID:35", "G2 and G3 arcs require at least one in-plane offset word." },
    { Status_GcodeUnusedWords, "Invalid gcode ID:36", "Unused value words found in block." },
    { Status_GcodeG43DynamicAxisError, "Invalid gcode ID:37", "G43.1 dynamic tool length offset is not assigned to configured tool length axis." },
    { Status_GcodeIllegalToolTableEntry, "Invalid gcode ID:38", "Tool number greater than max supported value or undefined tool selected." },
    { Status_GcodeValueOutOfRange, "Invalid gcode ID:39", "Value out of range." },
    { Status_GcodeToolChangePending, "Invalid gcode ID:40", "G-code command not allowed when tool change is pending." },
    { Status_GcodeSpindleNotRunning, "Invalid gcode ID:41", "Spindle not running when motion commanded in CSS or spindle sync mode." },
    { Status_GcodeIllegalPlane, "Invalid gcode ID:42", "Plane must be ZX for threading." },
    { Status_GcodeMaxFeedRateExceeded, "Invalid gcode ID:43", "Max. feed rate exceeded." },
    { Status_GcodeRPMOutOfRange, "Invalid gcode ID:44", "RPM out of range." },
    { Status_LimitsEngaged, "Limit switch engaged", "Only homing is allowed when a limit switch is engaged." },
    { Status_HomingRequired, "Homing required", "Home machine to continue." },
    { Status_GCodeToolError, "Invalid gcode ID:47", "ATC: current tool is not set. Set current tool with M61." },
    { Status_ValueWordConflict, "Invalid gcode ID:48", "Value word conflict." },
    { Status_SelfTestFailed, "Self test failed", "Power on self test failed. A hard reset is required." },
    { Status_EStop, "E-stop", "Emergency stop active." },
    { Status_MotorFault, "Motor fault", "Motor fault." },
    { Status_SettingValueOutOfRange, "Value out of range.", "Setting value is out of range." },
    { Status_SettingDisabled, "Setting disabled", "Setting is not available, possibly due to limited driver support." },
    { Status_GcodeInvalidRetractPosition, "Invalid gcode ID:54", "Retract position is less than drill depth." },
#if NGC_EXPRESSIONS_ENABLE
    { Status_ExpressionUknownOp, "Unknown operation found in expression", "Unknown operation found in expression." },
    { Status_ExpressionDivideByZero, "Divide by zero in expression", "Divide by zero in expression attempted." },
    { Status_ExpressionArgumentOutOfRange, "Expression argument out of range", "Too large or too small argrument provided." },
    { Status_ExpressionInvalidArgument, "Invalid expression argument", "Argument is not valid for the operation" },
    { Status_ExpressionSyntaxError, "Syntax error in expression", "Expression is not valid." },
    { Status_ExpressionInvalidResult, "Invalid result returned from expression", "Either NAN (not a number) or infinity was returned from expression." }
#endif
 };

static error_details_t details = {
    .errors = status_detail,
    .n_errors = sizeof(status_detail) / sizeof(status_detail_t)
};

error_details_t *errors_get_details (void)
{
    details.on_get_errors = grbl.on_get_errors;

    return &details;
}
