/*
  messages.c - system messages

  Part of grblHAL

  Copyright (c) 2017-2024 Terje Io
  Copyright (c) 2014-2016 Sungeun K. Jeon for Gnea Research LLC

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

#include "grbl.h"
#include "messages.h"

PROGMEM static const message_t messages[] = {
    { .id = Message_None, .text = "" },
    { .id = Message_CriticalEvent, .text = "Reset to continue" },
    { .id = Message_AlarmLock, .text = "'$H'|'$X' to unlock" },
    { .id = Message_AlarmUnlock, .text = "Caution: Unlocked" },
    { .id = Message_Enabled, .text = "Enabled" },
    { .id = Message_Disabled, .text = "Disabled" },
    { .id = Message_SafetyDoorAjar, .text = "Check Door" },
    { .id = Message_CheckLimits, .text = "Check Limits" },
    { .id = Message_ProgramEnd, .text = "Pgm End" },
    { .id = Message_RestoreDefaults, .text = "Restoring defaults" },
    { .id = Message_SpindleRestore, .text = "Restoring spindle" },
    { .id = Message_SleepMode, .text = "Sleeping" },
    { .id = Message_EStop, .text = "Emergency stop - clear, then reset to continue" },
    { .id = Message_HomingCycleRequired, .text = "Homing cycle required" },
    { .id = Message_CycleStartToRerun, .text = "Press cycle start to rerun job" },
    { .id = Message_ReferenceTLOEstablished, .text = "Reference tool length offset established" },
    { .id = Message_MotorFault, .text = "Motor fault - clear, then reset to continue" },
    { .id = Message_CycleStart2Continue, .text = "Press cycle start to continue." },
    { .id = Message_TPCycleStart2Continue, .text = "Remove any touch plate and press cycle start to continue." },
    { .id = Message_ProbeFailedRetry, .text = "Probe failed, try again." },
    { .id = Message_ExecuteTPW, .text = "Perform a probe with $TPW first!", .type = Message_Warning},
    { .id = Message_ProbeProtected, .text = "Probe protection activated."},
    { .id = Message_Stop, .text = "Stop"}
};

const message_t *message_get (message_code_t id)
{
    uint_fast16_t idx = 0;
    const message_t *msg = NULL;

    do {
        if(messages[idx].id == id)
            msg = &messages[idx];
    } while(msg == NULL && ++idx < Message_NextMessage);

    return msg;
}
