/*
  alarms.h -

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

#ifndef _ALARMS_H_
#define _ALARMS_H_

// Alarm executor codes. Valid values (1-255). Zero is reserved.
typedef enum {
    Alarm_None = 0,
    Alarm_HardLimit = 1,
    Alarm_SoftLimit = 2,
    Alarm_AbortCycle = 3,
    Alarm_ProbeFailInitial = 4,
    Alarm_ProbeFailContact = 5,
    Alarm_HomingFailReset = 6,
    Alarm_HomingFailDoor = 7,
    Alarm_FailPulloff = 8,
    Alarm_HomingFailApproach = 9,
    Alarm_EStop = 10,
    Alarm_HomingRequried = 11,
    Alarm_LimitsEngaged = 12,
    Alarm_ProbeProtect = 13,
    Alarm_Spindle = 14,
    Alarm_HomingFailAutoSquaringApproach = 15,
    Alarm_SelftestFailed = 16,
    Alarm_MotorFault = 17
} alarm_code_t;

typedef struct {
    alarm_code_t id;
    const char *name;
    const char *description;
} alarm_detail_t;

PROGMEM static const alarm_detail_t alarm_detail[] = {
    { Alarm_HardLimit, "Hard limit", "Hard limit has been triggered. Machine position is likely lost due to sudden halt. Re-homing is highly recommended." },
    { Alarm_SoftLimit, "Soft limit", "Soft limit alarm. G-code motion target exceeds machine travel. Machine position retained. Alarm may be safely unlocked." },
    { Alarm_AbortCycle, "Abort during cycle", "Reset while in motion. Machine position is likely lost due to sudden halt. Re-homing is highly recommended." },
    { Alarm_ProbeFailInitial, "Probe fail", "Probe fail. Probe is not in the expected initial state before starting probe cycle when G38.2 and G38.3 is not triggered and G38.4 and G38.5 is triggered." },
    { Alarm_ProbeFailContact, "Probe fail", "Probe fail. Probe did not contact the workpiece within the programmed travel for G38.2 and G38.4." },
    { Alarm_HomingFailReset, "Homing fail", "Homing fail. The active homing cycle was reset." },
    { Alarm_HomingFailDoor, "Homing fail", "Homing fail. Safety door was opened during homing cycle." },
    { Alarm_FailPulloff, "Homing fail", "Homing fail. Pull off travel failed to clear limit switch. Try increasing pull-off setting or check wiring." },
    { Alarm_HomingFailApproach, "Homing fail", "Homing fail. Could not find limit switch within search distances. Try increasing max travel, decreasing pull-off distance, or check wiring." },
    { Alarm_EStop, "EStop", "EStop asserted. Clear and reset" },
    { Alarm_HomingRequried, "Homing required", "Homing required. Execute homing command ($H) to continue." },
    { Alarm_LimitsEngaged, "Limit switch engaged", "Limit switch engaged. Clear before continuing." },
    { Alarm_ProbeProtect, "Probe protection triggered", "Probe protection triggered. Clear before continuing." },
    { Alarm_Spindle, "Spindle at speed timeout", "Spindle at speed timeout. Clear before continuing." },
    { Alarm_HomingFailAutoSquaringApproach, "Homing fail", "Homing fail. Could not find second limit switch for auto squared axis within search distances. Try increasing max travel, decreasing pull-off distance, or check wiring." },
    { Alarm_SelftestFailed, "Selftest failed", "Power on selftest (POS) failed." },
    { Alarm_MotorFault, "Motor fault", "Motor fault." }
};

#endif

