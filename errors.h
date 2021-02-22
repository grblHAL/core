/*
  errors.h -

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

#ifndef _ERRORS_H_
#define _ERRORS_H_

#include <stddef.h>

// Define Grbl status codes. Valid values (0-255)
typedef enum {
    Status_OK = 0,
    Status_ExpectedCommandLetter = 1,
    Status_BadNumberFormat = 2,
    Status_InvalidStatement = 3,
    Status_NegativeValue = 4,
    Status_HomingDisabled = 5,
    Status_SettingStepPulseMin = 6,
    Status_SettingReadFail = 7,
    Status_IdleError = 8,
    Status_SystemGClock = 9,
    Status_SoftLimitError = 10,
    Status_Overflow = 11,
    Status_MaxStepRateExceeded = 12,
    Status_CheckDoor = 13,
    Status_LineLengthExceeded = 14,
    Status_TravelExceeded = 15,
    Status_InvalidJogCommand = 16,
    Status_SettingDisabledLaser = 17,
    Status_Reset = 18,
    Status_NonPositiveValue = 19,

    Status_GcodeUnsupportedCommand = 20,
    Status_GcodeModalGroupViolation = 21,
    Status_GcodeUndefinedFeedRate = 22,
    Status_GcodeCommandValueNotInteger = 23,
    Status_GcodeAxisCommandConflict = 24,
    Status_GcodeWordRepeated = 25,
    Status_GcodeNoAxisWords = 26,
    Status_GcodeInvalidLineNumber = 27,
    Status_GcodeValueWordMissing = 28,
    Status_GcodeUnsupportedCoordSys = 29,
    Status_GcodeG53InvalidMotionMode = 30,
    Status_GcodeAxisWordsExist = 31,
    Status_GcodeNoAxisWordsInPlane = 32,
    Status_GcodeInvalidTarget = 33,
    Status_GcodeArcRadiusError = 34,
    Status_GcodeNoOffsetsInPlane = 35,
    Status_GcodeUnusedWords = 36,
    Status_GcodeG43DynamicAxisError = 37,
    Status_GcodeIllegalToolTableEntry = 38,
    Status_GcodeValueOutOfRange = 39,
    Status_GcodeToolChangePending = 40,
    Status_GcodeSpindleNotRunning = 41,
    Status_GcodeIllegalPlane = 42,
    Status_GcodeMaxFeedRateExceeded = 43,
    Status_GcodeRPMOutOfRange = 44,
    Status_LimitsEngaged = 45,
    Status_HomingRequired = 46,
    Status_GCodeToolError = 47,
    Status_ValueWordConflict = 48,
    Status_SelfTestFailed = 49,
    Status_EStop = 50,
    Status_MotorFault = 51,
    Status_SettingValueOutOfRange = 52,
    Status_SettingDisabled = 53,

// Some error codes as defined in bdring's ESP32 port
    Status_SDMountError = 60,
    Status_SDReadError = 61,
    Status_SDFailedOpenDir = 62,
    Status_SDDirNotFound = 63,
    Status_SDFileEmpty = 64,

    Status_BTInitError = 70,
    Status_Unhandled // For internal use only
} status_code_t;

typedef struct {
    status_code_t id;
    const char *name;
    const char *description;
} status_detail_t;

PROGMEM static const status_detail_t status_detail[] = {
    { Status_OK, "ok", NULL },
    { Status_ExpectedCommandLetter, "Expected command letter", "G-code words consist of a letter and a value. Letter was not found." },
    { Status_BadNumberFormat, "Bad number format", "Missing the expected G-code word value or numeric value format is not valid." },
    { Status_InvalidStatement, "Invalid statement", "Grbl '$' system command was not recognized or supported." },
    { Status_NegativeValue, "Value < 0", "Negative value received for an expected positive value." },
    { Status_HomingDisabled, "Homing disabled", "Homing cycle failure. Homing is not configured via settings." },
    { Status_SettingStepPulseMin, "Value < 2 microseconds", "Step pulse time must be greater or equal to 2 microseconds." },
    { Status_SettingReadFail, "EEPROM read fail. Using defaults", "An EEPROM read failed. Auto-restoring affected EEPROM to default values." },
    { Status_IdleError, "Not idle", "Grbl '$' command cannot be used unless Grbl is IDLE. Ensures smooth operation during a job." },
    { Status_SystemGClock, "G-code lock", "G-code commands are locked out during alarm or jog state." },
    { Status_SoftLimitError, "Homing not enabled", "Soft limits cannot be enabled without homing also enabled." },
    { Status_Overflow, "Line overflow", "Max characters per line exceeded. Received command line was not executed." },
    { Status_MaxStepRateExceeded, "Step rate > 30kHz", "Grbl '$' setting value cause the step rate to exceed the maximum supported." },
    { Status_CheckDoor, "Check Door", "Safety door detected as opened and door state initiated." },
    { Status_LineLengthExceeded, "Line length exceeded", "Build info or startup line exceeded EEPROM line length limit. Line not stored." },
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
    { Status_SDMountError, "SD Card", "SD Card mount failed." },
    { Status_SDReadError, "SD Card", "SD Card file open/read failed." },
    { Status_SDFailedOpenDir, "SD Card", "SD Card directory listing failed." },
    { Status_SDDirNotFound, "SD Card", "SD Card directory not found." },
    { Status_SDFileEmpty, "SD Card", "SD Card file empty." },
    { Status_BTInitError, "Bluetooth", "Bluetooth initalisation failed." }
};

#endif
