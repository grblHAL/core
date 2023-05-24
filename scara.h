/*
  scara.h - scara kinematics implementation

  Part of grblHAL

  Copyright (c) 2019 Terje Io

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

#include "grbl.h"

#ifndef _SCARA_H_
#define _SCARA_H_


#define A_MOTOR X_AXIS // Must be X_AXIS
#define B_MOTOR Y_AXIS // Must be Y_AXIS

#define MAX_SEG_LENGTH_MM 2.0f // segmenting long lines due to non-linear motions [mm]

#define SCARA_L1 500.0f // Length of first arm [mm]
#define SCARA_L2 450.0f // Length of second arm [mm]

// if defined, q2 is absolute joint angle, otherwise relative
#define SCARA_ABSOLUTE_JOINT_ANGLES On

// struct to hold the xy coordinates
typedef struct {
    float x;
    float y;
} xy_t;

// struct to hold the joint angles q
typedef struct {
    float q1;
    float q2;
} q_t;

// Initialize HAL pointers for scara kinematics
void scara_init(void);

#endif
