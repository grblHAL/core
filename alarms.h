/*
  alarms.h -

  Part of grblHAL

  Copyright (c) 2017-2023 Terje Io
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
    Alarm_None = 0,                             //!< 0
    Alarm_HardLimit = 1,                        //!< 1
    Alarm_SoftLimit = 2,                        //!< 2
    Alarm_AbortCycle = 3,                       //!< 3
    Alarm_ProbeFailInitial = 4,                 //!< 4
    Alarm_ProbeFailContact = 5,                 //!< 5
    Alarm_HomingFailReset = 6,                  //!< 6
    Alarm_HomingFailDoor = 7,                   //!< 7
    Alarm_FailPulloff = 8,                      //!< 8
    Alarm_HomingFailApproach = 9,               //!< 9
    Alarm_EStop = 10,                           //!< 10
    Alarm_HomingRequried = 11,                  //!< 11
    Alarm_LimitsEngaged = 12,                   //!< 12
    Alarm_ProbeProtect = 13,                    //!< 13
    Alarm_Spindle = 14,                         //!< 14
    Alarm_HomingFailAutoSquaringApproach = 15,  //!< 15
    Alarm_SelftestFailed = 16,                  //!< 16
    Alarm_MotorFault = 17,                      //!< 17
    Alarm_HomingFail = 18,                      //!< 18
    Alarm_AlarmMax = Alarm_HomingFail
} alarm_code_t;

typedef struct {
    alarm_code_t id;
    const char *description;
} alarm_detail_t;

typedef struct alarm_details {
    const uint16_t n_alarms;
    const alarm_detail_t *alarms;
    struct alarm_details *next;
} alarm_details_t;

typedef alarm_details_t *(*on_get_alarms_ptr)(void);

alarm_details_t *alarms_get_details (void);
const char *alarms_get_description (alarm_code_t id);
void alarms_register (alarm_details_t *details);

#endif

