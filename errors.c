/*
  errors.c -

  Part of grblHAL

  Copyright (c) 2017-2024 Terje Io
  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
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

#include <stdint.h>

#include "grbl.h"
#include "core_handlers.h"

PROGMEM static const status_detail_t status_detail[] = {
#ifndef NO_SETTINGS_DESCRIPTIONS
    { Status_OK, NULL },
    { Status_ExpectedCommandLetter, "G-code words consist of a letter and a value. Letter was not found." },
    { Status_BadNumberFormat, "Missing the expected G-code word value or numeric value format is not valid." },
    { Status_InvalidStatement, "'$' system command was not recognized or supported." },
    { Status_NegativeValue, "Negative value received for an expected positive value." },
    { Status_HomingDisabled, "Homing cycle failure. Homing is not configured via settings." },
    { Status_SettingStepPulseMin, "Step pulse time must be greater or equal to 2 microseconds." },
    { Status_SettingReadFail, "A settings read failed. Auto-restoring affected settings to default values." },
    { Status_IdleError, "'$' command cannot be used unless controller state is IDLE. Ensures smooth operation during a job." },
    { Status_SystemGClock, "G-code commands are locked out during alarm or jog state." },
    { Status_SoftLimitError, "Soft limits cannot be enabled without homing also enabled." },
    { Status_Overflow, "Max characters per line exceeded. Received command line was not executed." },
    { Status_MaxStepRateExceeded, "'$' setting value cause the step rate to exceed the maximum supported." },
    { Status_CheckDoor, "Safety door detected as opened and door state initiated." },
    { Status_LineLengthExceeded, "Build info or startup line exceeded line length limit. Line not stored." },
    { Status_TravelExceeded, "Jog target exceeds machine travel. Jog command has been ignored." },
    { Status_InvalidJogCommand, "Jog command has no '=' or contains prohibited g-code." },
    { Status_SettingDisabledLaser, "Laser mode requires PWM output." },
    { Status_Reset, "Reset asserted" },
    { Status_NonPositiveValue, "Non positive value" },
    { Status_GcodeUnsupportedCommand, "Unsupported or invalid g-code command found in block." },
    { Status_GcodeModalGroupViolation, "More than one g-code command from same modal group found in block." },
    { Status_GcodeUndefinedFeedRate, "Feed rate has not yet been set or is undefined." },
    { Status_GcodeCommandValueNotInteger, "G-code command in block requires an integer value." },
    { Status_GcodeAxisCommandConflict, "More than one g-code command that requires axis words found in block." },
    { Status_GcodeWordRepeated, "Repeated g-code word found in block." },
    { Status_GcodeNoAxisWords, "No axis words found in block for g-code command or current modal state which requires them." },
    { Status_GcodeInvalidLineNumber, "Line number value is invalid." },
    { Status_GcodeValueWordMissing, "G-code command is missing a required value word." },
    { Status_GcodeUnsupportedCoordSys, "G59.x work coordinate systems are not supported." },
    { Status_GcodeG53InvalidMotionMode, "G53 only allowed with G0 and G1 motion modes." },
    { Status_GcodeAxisWordsExist, "Axis words found in block when no command or current modal state uses them." },
    { Status_GcodeNoAxisWordsInPlane, "G2 and G3 arcs require at least one in-plane axis word." },
    { Status_GcodeInvalidTarget, "Motion command target is invalid." },
    { Status_GcodeArcRadiusError, "Arc radius value is invalid." },
    { Status_GcodeNoOffsetsInPlane, "G2 and G3 arcs require at least one in-plane offset word." },
    { Status_GcodeUnusedWords, "Unused value words found in block." },
    { Status_GcodeG43DynamicAxisError, "G43.1 dynamic tool length offset is not assigned to configured tool length axis." },
    { Status_GcodeIllegalToolTableEntry, "Tool number greater than max supported value or undefined tool selected." },
    { Status_GcodeValueOutOfRange, "Value out of range." },
    { Status_GcodeToolChangePending, "G-code command not allowed when tool change is pending." },
    { Status_GcodeSpindleNotRunning, "Spindle not running when motion commanded in CSS or spindle sync mode." },
    { Status_GcodeIllegalPlane, "Plane must be ZX for threading." },
    { Status_GcodeMaxFeedRateExceeded, "Max. feed rate exceeded." },
    { Status_GcodeRPMOutOfRange, "RPM out of range." },
    { Status_LimitsEngaged, "Only homing is allowed when a limit switch is engaged." },
    { Status_HomingRequired, "Home machine to continue." },
    { Status_GCodeToolError, "ATC: current tool is not set. Set current tool with M61." },
    { Status_ValueWordConflict, "Value word conflict." },
    { Status_SelfTestFailed, "Power on self test failed. A hard reset is required." },
    { Status_EStop, "Emergency stop active." },
    { Status_MotorFault, "Motor fault." },
    { Status_SettingValueOutOfRange, "Setting value is out of range." },
    { Status_SettingDisabled, "Setting is not available, possibly due to limited driver support." },
    { Status_GcodeInvalidRetractPosition, "Retract position is less than drill depth." },
    { Status_IllegalHomingConfiguration, "Attempt to home two auto squared axes at the same time." },
#if COMPATIBILITY_LEVEL <= 1
    { Status_GCodeCoordSystemLocked, "Coordinate system is locked." },
#endif
#if NGC_EXPRESSIONS_ENABLE
    { Status_ExpressionUknownOp, "Unknown operation found in expression." },
    { Status_ExpressionDivideByZero, "Divide by zero in expression attempted." },
    { Status_ExpressionArgumentOutOfRange, "Too large or too small argrument provided." },
    { Status_ExpressionInvalidArgument, "Argument is not valid for the operation" },
    { Status_ExpressionSyntaxError, "Expression is not valid." },
    { Status_ExpressionInvalidResult, "Either NAN (not a number) or infinity was returned from expression." },
#endif
    { Status_AuthenticationRequired, "Authentication required." },
    { Status_AccessDenied, "Access denied." },
    { Status_NotAllowedCriticalEvent, "Not allowed while critical event is active." },
#if NGC_EXPRESSIONS_ENABLE
    { Status_FlowControlNotExecutingMacro, "Flow statement only allowed in filesystem macro." },
    { Status_FlowControlSyntaxError, "Unknown flow statement." },
    { Status_FlowControlStackOverflow, "Stack overflow while executing flow statement." },
    { Status_FlowControlOutOfMemory, "Out of memory while executing flow statement." },
#endif
    { Status_FileOpenFailed, "Could not open file." },
    { Status_UserException, "User defined error occured." }
#endif // NO_SETTINGS_DESCRIPTIONS
};

static error_details_t details = {
    .errors = status_detail,
    .n_errors = sizeof(status_detail) / sizeof(status_detail_t)
};

static error_details_t *errors = &details;

void errors_register (error_details_t *details)
{
    errors->next = details;
    errors = details;
}

error_details_t *errors_get_details (void)
{
    return &details;
}

const char *errors_get_description (status_code_t id)
{
    uint_fast16_t n_errors;
    const char *description = NULL;
    error_details_t *details = grbl.on_get_errors();

    do {
        if((n_errors = details->n_errors)) do {
            if(details->errors[--n_errors].id == id)
                description = details->errors[n_errors].description;
        } while(description == NULL && n_errors);
    } while(description == NULL && (details = details->next));

    return description;
}
