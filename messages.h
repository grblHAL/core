/*
  messages.h - system messages

  Part of grblHAL

  Copyright (c) 2017-2024 Terje Io

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

#ifndef _MESSAGES_H_
#define _MESSAGES_H_

// Define feedback message codes. Valid values (0-255).
typedef enum {
    Message_None = 0,                       //!< 0 - reserved, do not change value.
    Message_CriticalEvent = 1,              //!< 1
    Message_AlarmLock = 2,                  //!< 2
    Message_AlarmUnlock = 3,                //!< 3
    Message_Enabled = 4,                    //!< 4
    Message_Disabled = 5,                   //!< 5
    Message_SafetyDoorAjar = 6,             //!< 6
    Message_CheckLimits = 7,                //!< 7
    Message_ProgramEnd = 8,                 //!< 8
    Message_RestoreDefaults = 9,            //!< 9
    Message_SpindleRestore = 10,            //!< 10
    Message_SleepMode = 11,                 //!< 11
    Message_EStop = 12,                     //!< 12
    Message_HomingCycleRequired = 13,       //!< 13
    Message_CycleStartToRerun = 14,         //!< 14
    Message_ReferenceTLOEstablished = 15,   //!< 15
    Message_MotorFault = 16,                //!< 16
    Message_CycleStart2Continue = 17,       //!< 17
    Message_TPCycleStart2Continue = 18,     //!< 18
    Message_ProbeFailedRetry = 19,          //!< 19
    Message_ExecuteTPW = 20,                //!< 20
    Message_ProbeProtected = 21,            //!< 21
    Message_Stop = 22,                      //!< 22
    Message_CycleStart2TouchOff = 23,       //!< 23
    Message_NextMessage                     //!< 24 - next unassigned message number.
} message_code_t;

typedef enum {
    Message_Plain = 0,
    Message_Info,
    Message_Warning,
    Message_Error,
    Message_Debug
} message_type_t;

typedef struct {
    message_code_t id;
    message_type_t type;
    const char *text;
} message_t;

const message_t *message_get (message_code_t id);

#endif
