/*
  alarms.c -

  Part of grblHAL

  Copyright (c) 2017-2022 Terje Io
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

PROGMEM static const alarm_detail_t alarm_detail[] = {
    { Alarm_HardLimit, "Hard limit has been triggered. Machine position is likely lost due to sudden halt. Re-homing is highly recommended." },
    { Alarm_SoftLimit, "Soft limit alarm. G-code motion target exceeds machine travel. Machine position retained. Alarm may be safely unlocked." },
    { Alarm_AbortCycle, "Reset while in motion. Machine position is likely lost due to sudden halt. Re-homing is highly recommended." },
    { Alarm_ProbeFailInitial, "Probe fail. Probe is not in the expected initial state before starting probe cycle when G38.2 and G38.3 is not triggered and G38.4 and G38.5 is triggered." },
    { Alarm_ProbeFailContact, "Probe fail. Probe did not contact the workpiece within the programmed travel for G38.2 and G38.4." },
    { Alarm_HomingFailReset, "Homing fail. The active homing cycle was reset." },
    { Alarm_HomingFailDoor, "Homing fail. Safety door was opened during homing cycle." },
    { Alarm_FailPulloff, "Homing fail. Pull off travel failed to clear limit switch. Try increasing pull-off setting or check wiring." },
    { Alarm_HomingFailApproach, "Homing fail. Could not find limit switch within search distances. Try increasing max travel, decreasing pull-off distance, or check wiring." },
    { Alarm_EStop, "EStop asserted. Clear and reset" },
    { Alarm_HomingRequried, "Homing required. Execute homing command ($H) to continue." },
    { Alarm_LimitsEngaged, "Limit switch engaged. Clear before continuing." },
    { Alarm_ProbeProtect, "Probe protection triggered. Clear before continuing." },
    { Alarm_Spindle, "Spindle at speed timeout. Clear before continuing." },
    { Alarm_HomingFailAutoSquaringApproach, "Homing fail. Could not find second limit switch for auto squared axis within search distances. Try increasing max travel, decreasing pull-off distance, or check wiring." },
    { Alarm_SelftestFailed, "Power on selftest (POS) failed." },
    { Alarm_MotorFault, "Motor fault." },
    { Alarm_HomingFail, "Homing fail. Bad configuration." }
};

static alarm_details_t details = {
    .alarms = alarm_detail,
    .n_alarms = sizeof(alarm_detail) / sizeof(alarm_detail_t)
};

static alarm_details_t *alarms = &details;

void alarms_register (alarm_details_t *details)
{
    alarms->next = details;
    alarms = details;
}

alarm_details_t *alarms_get_details (void)
{
    return &details;
}
