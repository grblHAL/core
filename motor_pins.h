/*
  motor_pins.h - pin mappings resolver for ganged/squared/ABC axes

  NOTE: This file is not used by the core, it may be used by drivers to simplify board map files

  Part of grblHAL

  Copyright (c) 2021 Terje Io

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

#pragma once

#if N_GANGED

#if N_GANGED > N_ABC_MOTORS
#error "Axis configuration is not supported!"
#endif

#if N_AUTO_SQUARED
#define SQUARING_ENABLED
#endif

#if Z_GANGED
#define Z_DOUBLED N_ABC_MOTORS
#elif Y_GANGED
#define Y_DOUBLED N_ABC_MOTORS
#elif X_GANGED
#define X_DOUBLED N_ABC_MOTORS
#endif

#if Y_GANGED && !defined(Y_DOUBLED)
#define Y_DOUBLED (N_ABC_MOTORS - 1)
#elif X_GANGED && !defined(X_DOUBLED)
#define X_DOUBLED (N_ABC_MOTORS - 1)
#endif

#if X_GANGED && !defined(X_DOUBLED)
#define X_DOUBLED (N_ABC_MOTORS - 2)
#endif

#if X_DOUBLED == 1

#ifdef A_AXIS
#error "A-axis motor is used for ganged X motor"
#endif
#define X2_STEP_PORT        M3_STEP_PORT
#define X2_STEP_PIN         M3_STEP_PIN
#define X2_STEP_BIT         (1<<M3_STEP_PIN)
#define X2_DIRECTION_PORT   M3_DIRECTION_PORT
#define X2_DIRECTION_PIN    M3_DIRECTION_PIN
#define X2_DIRECTION_BIT    (1<<M3_DIRECTION_PIN)
#ifdef M3_LIMIT_PIN
 #if X_AUTO_SQUARE
  #define X2_LIMIT_PORT     M3_LIMIT_PORT
  #define X2_LIMIT_PIN      M3_LIMIT_PIN
  #define X2_LIMIT_BIT      (1<<M3_LIMIT_PIN)
 #elif X_GANGED_LIM_MAX && !defined(M3_LIMIT_PIN_MAX)
  #define X_LIMIT_PORT_MAX  M3_LIMIT_PORT
  #define X_LIMIT_PIN_MAX   M3_LIMIT_PIN
  #define X_LIMIT_BIT_MAX   (1<<M3_LIMIT_PIN)
 #endif
#elif X_AUTO_SQUARE
  #error "Auto squared X-axis requires second limit pin input"
#endif
#ifdef M3_LIMIT_PIN_MAX
  #define X_LIMIT_PORT_MAX  M3_LIMIT_PORT_MAX
  #define X_LIMIT_PIN_MAX   M3_LIMIT_PIN_MAX
  #define X_LIMIT_BIT_MAX   (1<<M3_LIMIT_PIN_MAX)
#endif
#ifdef M3_ENABLE_PIN
  #define X2_ENABLE_PORT    M3_ENABLE_PORT
  #define X2_ENABLE_PIN     M3_ENABLE_PIN
  #define X2_ENABLE_BIT     (1<<M3_ENABLE_PIN)
#endif

#elif X_DOUBLED == 2

#ifdef B_AXIS
#error "B-axis motor is used for ganged X motor"
#endif
#define X2_STEP_PORT        M4_STEP_PORT
#define X2_STEP_PIN         M4_STEP_PIN
#define X2_STEP_BIT         (1<<M4_STEP_PIN)
#define X2_DIRECTION_PORT   M4_DIRECTION_PORT
#define X2_DIRECTION_PIN    M4_DIRECTION_PIN
#define X2_DIRECTION_BIT    (1<<M4_DIRECTION_PIN)
#ifdef M4_LIMIT_PIN
 #if X_AUTO_SQUARE
  #define X2_LIMIT_PORT     M4_LIMIT_PORT
  #define X2_LIMIT_PIN      M4_LIMIT_PIN
  #define X2_LIMIT_BIT      (1<<M4_LIMIT_PIN)
 #elif X_GANGED_LIM_MAX && !defined(M4_LIMIT_PIN_MAX)
  #define X_LIMIT_PORT_MAX  M4_LIMIT_PORT
  #define X_LIMIT_PIN_MAX   M4_LIMIT_PIN
  #define X_LIMIT_BIT_MAX   (1<<M4_LIMIT_PIN)
 #endif
#elif X_AUTO_SQUARE
  #error "Auto squared X-axis requires second limit pin input"
#endif
#ifdef M4_LIMIT_PIN_MAX
  #define X_LIMIT_PORT_MAX  M4_LIMIT_PORT_MAX
  #define X_LIMIT_PIN_MAX   M4_LIMIT_PIN_MAX
  #define X_LIMIT_BIT_MAX   (1<<M4_LIMIT_PIN_MAX)
#endif
#ifdef M4_ENABLE_PIN
  #define X2_ENABLE_PORT    M4_ENABLE_PORT
  #define X2_ENABLE_PIN     M4_ENABLE_PIN
  #define X2_ENABLE_BIT     (1<<M4_ENABLE_PIN)
#endif

#elif X_DOUBLED == 3

#ifdef C_AXIS
#error "C-axis motor is used for ganged X motor"
#endif
#define X2_STEP_PORT        M5_STEP_PORT
#define X2_STEP_PIN         M5_STEP_PIN
#define X2_STEP_BIT         (1<<M5_STEP_PIN)
#define X2_DIRECTION_PORT   M5_DIRECTION_PORT
#define X2_DIRECTION_PIN    M5_DIRECTION_PIN
#define X2_DIRECTION_BIT    (1<<M5_DIRECTION_PIN)
#ifdef M5_LIMIT_PIN
 #if X_AUTO_SQUARE
  #define X2_LIMIT_PORT     M5_LIMIT_PORT
  #define X2_LIMIT_PIN      M5_LIMIT_PIN
  #define X2_LIMIT_BIT      (1<<M5_LIMIT_PIN)
 #elif X_GANGED_LIM_MAX && !defined(M5_LIMIT_PIN_MAX)
  #define X_LIMIT_PORT_MAX  M5_LIMIT_PORT
  #define X_LIMIT_PIN_MAX   M5_LIMIT_PIN
  #define X_LIMIT_BIT_MAX   (1<<M5_LIMIT_PIN)
 #endif
#elif X_AUTO_SQUARE
  #error "Auto squared X-axis requires second limit pin input"
#endif
#ifdef M5_LIMIT_PIN_MAX
  #define X_LIMIT_PORT_MAX  M5_LIMIT_PORT_MAX
  #define X_LIMIT_PIN_MAX   M5_LIMIT_PIN_MAX
  #define X_LIMIT_BIT_MAX   (1<<M5_LIMIT_PIN_MAX)
#endif
#ifdef M5_ENABLE_PIN
  #define X2_ENABLE_PORT    M5_ENABLE_PORT
  #define X2_ENABLE_PIN     M5_ENABLE_PIN
  #define X2_ENABLE_BIT     (1<<M5_ENABLE_PIN)
#endif

#endif // X_DOUBLED

#if Y_DOUBLED == 1

#ifdef A_AXIS
#error "A-axis motor is used for ganged Y motor"
#endif
#define Y2_STEP_PORT        M3_STEP_PORT
#define Y2_STEP_PIN         M3_STEP_PIN
#define Y2_STEP_BIT         (1<<M3_STEP_PIN)
#define Y2_DIRECTION_PORT   M3_DIRECTION_PORT
#define Y2_DIRECTION_PIN    M3_DIRECTION_PIN
#define Y2_DIRECTION_BIT    (1<<M3_DIRECTION_PIN)
#ifdef M3_LIMIT_PIN
 #if Y_AUTO_SQUARE
  #define Y2_LIMIT_PORT     M3_LIMIT_PORT
  #define Y2_LIMIT_PIN      M3_LIMIT_PIN
  #define Y2_LIMIT_BIT      (1<<M3_LIMIT_PIN)
 #elif Y_GANGED_LIM_MAX && !defined(M3_LIMIT_PIN_MAX)
  #define Y_LIMIT_PORT_MAX  M3_LIMIT_PORT
  #define Y_LIMIT_PIN_MAX   M3_LIMIT_PIN
  #define Y_LIMIT_BIT_MAX   (1<<M3_LIMIT_PIN)
 #endif
#elif Y_AUTO_SQUARE
  #error "Auto squared Y-axis requires second limit pin input"
#endif
#ifdef M3_LIMIT_PIN_MAX
  #define Y_LIMIT_PORT_MAX  M3_LIMIT_PORT_MAX
  #define Y_LIMIT_PIN_MAX   M3_LIMIT_PIN_MAX
  #define Y_LIMIT_BIT_MAX   (1<<M3_LIMIT_PIN_MAX)
#endif
#ifdef M3_ENABLE_PIN
  #define Y2_ENABLE_PORT    M3_ENABLE_PORT
  #define Y2_ENABLE_PIN     M3_ENABLE_PIN
  #define Y2_ENABLE_BIT     (1<<M3_ENABLE_PIN)
#endif

#elif Y_DOUBLED == 2

#ifdef B_AXIS
#error "B-axis motor is used for ganged Y motor"
#endif
#define Y2_STEP_PORT        M4_STEP_PORT
#define Y2_STEP_PIN         M4_STEP_PIN
#define Y2_STEP_BIT         (1<<M4_STEP_PIN)
#define Y2_DIRECTION_PORT   M4_DIRECTION_PORT
#define Y2_DIRECTION_PIN    M4_DIRECTION_PIN
#define Y2_DIRECTION_BIT    (1<<M4_DIRECTION_PIN)
#ifdef M4_LIMIT_PIN
 #if Y_AUTO_SQUARE
  #define Y2_LIMIT_PORT     M4_LIMIT_PORT
  #define Y2_LIMIT_PIN      M4_LIMIT_PIN
  #define Y2_LIMIT_BIT      (1<<M4_LIMIT_PIN)
 #elif Y_GANGED_LIM_MAX && !defined(M4_LIMIT_PIN_MAX)
  #define Y_LIMIT_PORT_MAX  M4_LIMIT_PORT
  #define Y_LIMIT_PIN_MAX   M4_LIMIT_PIN
  #define Y_LIMIT_BIT_MAX   (1<<M4_LIMIT_PIN)
 #endif
#elif Y_AUTO_SQUARE
  #error "Auto squared Y-axis requires second limit pin input"
#endif
#ifdef M4_LIMIT_PIN_MAX
  #define Y_LIMIT_PORT_MAX  M4_LIMIT_PORT_MAX
  #define Y_LIMIT_PIN_MAX   M4_LIMIT_PIN_MAX
  #define Y_LIMIT_BIT_MAX   (1<<M4_LIMIT_PIN_MAX)
#endif
#ifdef M4_ENABLE_PIN
  #define Y2_ENABLE_PORT    M4_ENABLE_PORT
  #define Y2_ENABLE_PIN     M4_ENABLE_PIN
  #define Y2_ENABLE_BIT     (1<<M4_ENABLE_PIN)
#endif

#elif Y_DOUBLED == 3

#ifdef C_AXIS
#error "C-axis motor is used for ganged Y motor"
#endif
#define Y2_STEP_PORT        M5_STEP_PORT
#define Y2_STEP_PIN         M5_STEP_PIN
#define Y2_STEP_BIT         (1<<M5_STEP_PIN)
#define Y2_DIRECTION_PORT   M5_DIRECTION_PORT
#define Y2_DIRECTION_PIN    M5_DIRECTION_PIN
#define Y2_DIRECTION_BIT    (1<<M5_DIRECTION_PIN)
#ifdef M5_LIMIT_PIN
 #if Y_AUTO_SQUARE
  #define Y2_LIMIT_PORT     M5_LIMIT_PORT
  #define Y2_LIMIT_PIN      M5_LIMIT_PIN
  #define Y2_LIMIT_BIT      (1<<M5_LIMIT_PIN)
 #elif Y_GANGED_LIM_MAX && !defined(M5_LIMIT_PIN_MAX)
  #define Y_LIMIT_PORT_MAX  M5_LIMIT_PORT
  #define Y_LIMIT_PIN_MAX   M5_LIMIT_PIN
  #define Y_LIMIT_BIT_MAX   (1<<M5_LIMIT_PIN)
 #endif
#elif Y_AUTO_SQUARE
  #error "Auto squared Y-axis requires second limit pin input"
#endif
#ifdef M5_LIMIT_PIN_MAX
  #define Y_LIMIT_PORT_MAX  M5_LIMIT_PORT_MAX
  #define Y_LIMIT_PIN_MAX   M5_LIMIT_PIN_MAX
  #define Y_LIMIT_BIT_MAX   (1<<M5_LIMIT_PIN_MAX)
#endif
#ifdef M5_ENABLE_PIN
  #define Y2_ENABLE_PORT    M5_ENABLE_PORT
  #define Y2_ENABLE_PIN     M5_ENABLE_PIN
  #define Y2_ENABLE_BIT     (1<<M5_ENABLE_PIN)
#endif

#endif // Y_DOUBLED

#if Z_DOUBLED == 1

#ifdef A_AZIS
#error "A-axis motor is used for ganged Z motor"
#endif
#define Z2_STEP_PORT        M3_STEP_PORT
#define Z2_STEP_PIN         M3_STEP_PIN
#define Z2_STEP_BIT         (1<<M3_STEP_PIN)
#define Z2_DIRECTION_PORT   M3_DIRECTION_PORT
#define Z2_DIRECTION_PIN    M3_DIRECTION_PIN
#define Z2_DIRECTION_BIT    (1<<M3_DIRECTION_PIN)
#ifdef M3_LIMIT_PIN
 #if Z_AUTO_SQUARE
  #define Z2_LIMIT_PORT     M3_LIMIT_PORT
  #define Z2_LIMIT_PIN      M3_LIMIT_PIN
  #define Z2_LIMIT_BIT      (1<<M3_LIMIT_PIN)
 #elif Z_GANGED_LIM_MAX && !defined(M3_LIMIT_PIN_MAX)
  #define Z_LIMIT_PORT_MAX  M3_LIMIT_PORT
  #define Z_LIMIT_PIN_MAX   M3_LIMIT_PIN
  #define Z_LIMIT_BIT_MAX   (1<<M3_LIMIT_PIN)
 #endif
#elif Z_AUTO_SQUARE
  #error "Auto squared Z-axis requires second limit pin input"
#endif
#ifdef M3_LIMIT_PIN_MAX
  #define Z_LIMIT_PORT_MAX  M3_LIMIT_PORT_MAX
  #define Z_LIMIT_PIN_MAX   M3_LIMIT_PIN_MAX
  #define Z_LIMIT_BIT_MAX   (1<<M3_LIMIT_PIN_MAX)
#endif
#ifdef M3_ENABLE_PIN
  #define Z2_ENABLE_PORT    M3_ENABLE_PORT
  #define Z2_ENABLE_PIN     M3_ENABLE_PIN
  #define Z2_ENABLE_BIT     (1<<M3_ENABLE_PIN)
#endif

#elif Z_DOUBLED == 2

#ifdef B_AZIS
#error "B-axis motor is used for ganged Z motor"
#endif
#define Z2_STEP_PORT        M4_STEP_PORT
#define Z2_STEP_PIN         M4_STEP_PIN
#define Z2_STEP_BIT         (1<<M4_STEP_PIN)
#define Z2_DIRECTION_PORT   M4_DIRECTION_PORT
#define Z2_DIRECTION_PIN    M4_DIRECTION_PIN
#define Z2_DIRECTION_BIT    (1<<M4_DIRECTION_PIN)
#ifdef M4_LIMIT_PIN
 #if Z_AUTO_SQUARE
  #define Z2_LIMIT_PORT     M4_LIMIT_PORT
  #define Z2_LIMIT_PIN      M4_LIMIT_PIN
  #define Z2_LIMIT_BIT      (1<<M4_LIMIT_PIN)
 #elif Z_GANGED_LIM_MAX && !defined(M4_LIMIT_PIN_MAX)
  #define Z_LIMIT_PORT_MAX  M4_LIMIT_PORT
  #define Z_LIMIT_PIN_MAX   M4_LIMIT_PIN
  #define Z_LIMIT_BIT_MAX   (1<<M4_LIMIT_PIN)
 #endif
#elif Z_AUTO_SQUARE
  #error "Auto squared Z-axis requires second limit pin input"
#endif
#ifdef M4_LIMIT_PIN_MAX
  #define Z_LIMIT_PORT_MAX  M4_LIMIT_PORT_MAX
  #define Z_LIMIT_PIN_MAX   M4_LIMIT_PIN_MAX
  #define Z_LIMIT_BIT_MAX   (1<<M4_LIMIT_PIN_MAX)
#endif
#ifdef M4_ENABLE_PIN
  #define Z2_ENABLE_PORT    M4_ENABLE_PORT
  #define Z2_ENABLE_PIN     M4_ENABLE_PIN
  #define Z2_ENABLE_BIT     (1<<M4_ENABLE_PIN)
#endif

#elif Z_DOUBLED == 3

#ifdef C_AZIS
#error "C-axis motor is used for ganged Z motor"
#endif
#define Z2_STEP_PORT        M5_STEP_PORT
#define Z2_STEP_PIN         M5_STEP_PIN
#define Z2_STEP_BIT         (1<<M5_STEP_PIN)
#define Z2_DIRECTION_PORT   M5_DIRECTION_PORT
#define Z2_DIRECTION_PIN    M5_DIRECTION_PIN
#define Z2_DIRECTION_BIT    (1<<M5_DIRECTION_PIN)
#ifdef M5_LIMIT_PIN
 #if Z_AUTO_SQUARE
  #define Z2_LIMIT_PORT     M5_LIMIT_PORT
  #define Z2_LIMIT_PIN      M5_LIMIT_PIN
  #define Z2_LIMIT_BIT      (1<<M5_LIMIT_PIN)
 #elif Z_GANGED_LIM_MAX && !defined(M5_LIMIT_PIN_MAX)
  #define Z_LIMIT_PORT_MAX  M5_LIMIT_PORT
  #define Z_LIMIT_PIN_MAX   M5_LIMIT_PIN
  #define Z_LIMIT_BIT_MAX   (1<<M5_LIMIT_PIN)
 #endif
#elif Z_AUTO_SQUARE
  #error "Auto squared Z-axis requires second limit pin input"
#endif
#ifdef M5_LIMIT_PIN_MAX
  #define Z_LIMIT_PORT_MAX  M5_LIMIT_PORT_MAX
  #define Z_LIMIT_PIN_MAX   M5_LIMIT_PIN_MAX
  #define Z_LIMIT_BIT_MAX   (1<<M5_LIMIT_PIN_MAX)
#endif
#ifdef M5_ENABLE_PIN
  #define Z2_ENABLE_PORT    M5_ENABLE_PORT
  #define Z2_ENABLE_PIN     M5_ENABLE_PIN
  #define Z2_ENABLE_BIT     (1<<M5_ENABLE_PIN)
#endif

#endif // Z_DOUBLED

#ifdef X_DOUBLED
#define X2_MOTOR (N_AXIS + X_DOUBLED - 1)
#endif
#ifdef Y_DOUBLED
#define Y2_MOTOR (N_AXIS + Y_DOUBLED - 1)
#endif
#ifdef Z_DOUBLED
#define Z2_MOTOR (N_AXIS + Z_DOUBLED - 1)
#endif

#endif // N_GANGED

#if defined(X2_LIMIT_PIN) || defined(Y2_LIMIT_PIN) || defined(Z2_LIMIT_PIN)
#define DUAL_LIMIT_SWITCHES
#endif

#if defined(X_LIMIT_PIN_MAX) || defined(Y_LIMIT_PIN_MAX) || defined(Z_LIMIT_PIN_MAX) || defined(A_LIMIT_PIN_MAX) || defined(B_LIMIT_PIN_MAX) || defined(C_LIMIT_PIN_MAX)
#define MAX_LIMIT_SWITCHES
#endif

#ifdef A_AXIS
#ifndef M3_AVAILABLE
  #error "A_AXIS pins are not available"
#endif
#define A_STEP_PORT         M3_STEP_PORT
#define A_STEP_PIN          M3_STEP_PIN
#define A_STEP_BIT          (1<<M3_STEP_PIN)
#define A_DIRECTION_PORT    M3_DIRECTION_PORT
#define A_DIRECTION_PIN     M3_DIRECTION_PIN
#define A_DIRECTION_BIT     (1<<M3_DIRECTION_PIN)
#ifdef M3_LIMIT_PIN
  #define A_LIMIT_PORT      M3_LIMIT_PORT
  #define A_LIMIT_PIN       M3_LIMIT_PIN
  #define A_LIMIT_BIT       (1<<M3_LIMIT_PIN)
#endif
#ifdef M3_LIMIT_PIN_MAX
  #define A_LIMIT_PORT_MAX  M3_LIMIT_PORT_MAX
  #define A_LIMIT_PIN_MAX   M3_LIMIT_PIN_MAX
  #define A_LIMIT_BIT_MAX   (1<<M3_LIMIT_PIN_MAX)
#endif
#ifdef M3_ENABLE_PIN
  #define A_ENABLE_PORT     M3_ENABLE_PORT
  #define A_ENABLE_PIN      M3_ENABLE_PIN
  #define A_ENABLE_BIT      (1<<M3_ENABLE_PIN)
#endif
#endif

#ifdef B_AXIS
#ifndef M4_AVAILABLE
  #error "B_AXIS pins are not available"
#endif
#define B_STEP_PORT         M4_STEP_PORT
#define B_STEP_PIN          M4_STEP_PIN
#define B_STEP_BIT          (1<<M4_STEP_PIN)
#define B_DIRECTION_PORT    M4_DIRECTION_PORT
#define B_DIRECTION_PIN     M4_DIRECTION_PIN
#define B_DIRECTION_BIT     (1<<M4_DIRECTION_PIN)
#ifdef M4_LIMIT_PIN
  #define B_LIMIT_PORT      M4_LIMIT_PORT
  #define B_LIMIT_PIN       M4_LIMIT_PIN
  #define B_LIMIT_BIT       (1<<M4_LIMIT_PIN)
#endif
#ifdef M4_LIMIT_PIN_MAX
  #define B_LIMIT_PORT_MAX  M4_LIMIT_PORT_MAX
  #define B_LIMIT_PIN_MAX   M4_LIMIT_PIN_MAX
  #define B_LIMIT_BIT_MAX   (1<<M4_LIMIT_PIN_MAX)
#endif
#ifdef M4_ENABLE_PIN
  #define B_ENABLE_PORT     M4_ENABLE_PORT
  #define B_ENABLE_PIN      M4_ENABLE_PIN
  #define B_ENABLE_BIT      (1<<M4_ENABLE_PIN)
#endif
#endif

#ifdef C_AXIS
#ifndef M5_AVAILABLE
  #error "C_AXIS pins are not available"
#endif
#define C_STEP_PORT         M5_STEP_PORT
#define C_STEP_PIN          M5_STEP_PIN
#define C_STEP_BIT          (1<<M5_STEP_PIN)
#define C_DIRECTION_PORT    M5_DIRECTION_PORT
#define C_DIRECTION_PIN     M5_DIRECTION_PIN
#define C_DIRECTION_BIT     (1<<M5_DIRECTION_PIN)
#ifdef M5_LIMIT_PIN
  #define C_LIMIT_PORT      M5_LIMIT_PORT
  #define C_LIMIT_PIN       M5_LIMIT_PIN
  #define C_LIMIT_BIT       (1<<M5_LIMIT_PIN)
#endif
#ifdef M5_LIMIT_PIN_MAX
  #define C_LIMIT_PORT_MAX  M5_LIMIT_PORT_MAX
  #define C_LIMIT_PIN_MAX   M5_LIMIT_PIN_MAX
  #define C_LIMIT_BIT_MAX   (1<<M5_LIMIT_PIN_MAX)
#endif
#ifdef M5_ENABLE_PIN
  #define C_ENABLE_PORT     M5_ENABLE_PORT
  #define C_ENABLE_PIN      M5_ENABLE_PIN
  #define C_ENABLE_BIT      (1<<M5_ENABLE_PIN)
#endif
#endif

#ifdef STEP_PORT
#ifndef X_STEP_PORT
#define X_STEP_PORT STEP_PORT
#endif
#ifndef Y_STEP_PORT
#define Y_STEP_PORT STEP_PORT
#endif
#ifndef Z_STEP_PORT
#define Z_STEP_PORT STEP_PORT
#endif
#if defined(A_AXIS) && !defined(A_STEP_PORT)
#define A_STEP_PORT STEP_PORT
#endif
#if defined(B_AXIS) && !defined(B_STEP_PORT)
#define B_STEP_PORT STEP_PORT
#endif
#if defined(C_AXIS) && !defined(C_STEP_PORT)
#define C_STEP_PORT STEP_PORT
#endif
#endif

#ifndef X_STEP_BIT
#define X_STEP_BIT (1<<X_STEP_PIN)
#endif
#ifndef Y_STEP_BIT
#define Y_STEP_BIT (1<<Y_STEP_PIN)
#endif
#ifndef Z_STEP_BIT
#define Z_STEP_BIT (1<<Z_STEP_PIN)
#endif

#ifdef DIRECTION_PORT
#ifndef X_DIRECTION_PORT
#define X_DIRECTION_PORT DIRECTION_PORT
#endif
#ifndef Y_DIRECTION_PORT
#define Y_DIRECTION_PORT DIRECTION_PORT
#endif
#ifndef Z_DIRECTION_PORT
#define Z_DIRECTION_PORT DIRECTION_PORT
#endif
#if defined(A_AXIS) && !defined(A_DIRECTION_PORT)
#define A_DIRECTION_PORT DIRECTION_PORT
#endif
#if defined(B_AXIS) && !defined(B_DIRECTION_PORT)
#define B_DIRECTION_PORT DIRECTION_PORT
#endif
#if defined(C_AXIS) && !defined(B_DIRECTION_PORT)
#define C_DIRECTION_PORT DIRECTION_PORT
#endif
#endif

#ifndef X_DIRECTION_BIT
#define X_DIRECTION_BIT (1<<X_DIRECTION_PIN)
#endif
#ifndef Y_DIRECTION_BIT
#define Y_DIRECTION_BIT (1<<Y_DIRECTION_PIN)
#endif
#ifndef Z_DIRECTION_BIT
#define Z_DIRECTION_BIT (1<<Z_DIRECTION_PIN)
#endif

#ifdef LIMIT_PORT
#ifndef X_LIMIT_PORT
#define X_LIMIT_PORT LIMIT_PORT
#endif
#ifndef Y_LIMIT_PORT
#define Y_LIMIT_PORT LIMIT_PORT
#endif
#ifndef Z_LIMIT_PORT
#define Z_LIMIT_PORT LIMIT_PORT
#endif
#if defined(A_AXIS) && !defined(A_LIMIT_PORT)
#define A_LIMIT_PORT LIMIT_PORT
#endif
#if defined(B_AXIS) && !defined(B_LIMIT_PORT)
#define B_LIMIT_PORT LIMIT_PORT
#endif
#if defined(C_AXIS) && !defined(C_LIMIT_PORT)
#define C_LIMIT_PORT LIMIT_PORT
#endif
#endif

#ifndef X_LIMIT_BIT
#define X_LIMIT_BIT (1<<X_LIMIT_PIN)
#endif
#ifndef Y_LIMIT_BIT
#define Y_LIMIT_BIT (1<<Y_LIMIT_PIN)
#endif
#ifndef Z_LIMIT_BIT
#define Z_LIMIT_BIT (1<<Z_LIMIT_PIN)
#endif

#if !defined(X_ENABLE_BIT) && defined(X_ENABLE_BIT)
#define X_ENABLE_BIT (1<<X_ENABLE_PIN)
#endif
#if !defined(Y_ENABLE_BIT) && defined(Y_ENABLE_PIN)
#define Y_ENABLE_BIT (1<<Y_ENABLE_PIN)
#endif
#if !defined(Z_ENABLE_BIT) && defined(Z_ENABLE_PIN)
#define Z_ENABLE_BIT (1<<Z_ENABLE_PIN)
#endif

#ifndef STEP_MASK
#if N_AXIS == 3
#define STEP_MASK (X_STEP_BIT|Y_STEP_BIT|Z_STEP_BIT)
#elif N_AXIS == 4
#define STEP_MASK (X_STEP_BIT|Y_STEP_BIT|Z_STEP_BIT|A_STEP_BIT)
#elif N_AXIS == 5
#define STEP_MASK (X_STEP_BIT|Y_STEP_BIT|Z_STEP_BIT|A_STEP_BIT|B_STEP_BIT)
#elif N_AXIS == 6
#define STEP_MASK (X_STEP_BIT|Y_STEP_BIT|Z_STEP_BIT|A_STEP_BIT|B_STEP_BIT|C_STEP_BIT)
#elif N_AXIS == 7
#define STEP_MASK (X_STEP_BIT|Y_STEP_BIT|Z_STEP_BIT|A_STEP_BIT|B_STEP_BIT|C_STEP_BIT|U_STEP_BIT)
#else
#define STEP_MASK (X_STEP_BIT|Y_STEP_BIT|Z_STEP_BIT|A_STEP_BIT|B_STEP_BIT|C_STEP_BIT|U_STEP_BIT|V_STEP_BIT)
#endif
#endif

#ifndef DIRECTION_MASK
#if N_AXIS == 3
#define DIRECTION_MASK (X_DIRECTION_BIT|Y_DIRECTION_BIT|Z_DIRECTION_BIT)
#elif N_AXIS == 4
#define DIRECTION_MASK (X_DIRECTION_BIT|Y_DIRECTION_BIT|Z_DIRECTION_BIT|A_DIRECTION_BIT)
#elif N_AXIS == 5
#define DIRECTION_MASK (X_DIRECTION_BIT|Y_DIRECTION_BIT|Z_DIRECTION_BIT|A_DIRECTION_BIT|B_DIRECTION_BIT)
#elif N_AXIS == 6
#define DIRECTION_MASK (X_DIRECTION_BIT|Y_DIRECTION_BIT|Z_DIRECTION_BIT|A_DIRECTION_BIT|B_DIRECTION_BIT|C_DIRECTION_BIT)
#elif N_AXIS == 7
#define DIRECTION_MASK (X_DIRECTION_BIT|Y_DIRECTION_BIT|Z_DIRECTION_BIT|A_DIRECTION_BIT|B_DIRECTION_BIT|C_DIRECTION_BIT|U_DIRECTION_BIT)
#else
#define DIRECTION_MASK (X_DIRECTION_BIT|Y_DIRECTION_BIT|Z_DIRECTION_BIT|A_DIRECTION_BIT|B_DIRECTION_BIT|C_DIRECTION_BIT|U_DIRECTION_BIT|V_DIRECTION_BIT)
#endif
#endif

#ifndef STEPPERS_ENABLE_MASK

#ifdef STEPPERS_ENABLE_BIT
#define STEPPERS_ENABLE_MASK STEPPERS_ENABLE_BIT
#else

#if N_AXIS >=4 && !defined(A_ENABLE_BIT)
#define A_ENABLE_BIT 0
#endif
#if N_AXIS >=5 && !defined(B_ENABLE_BIT)
#define B_ENABLE_BIT 0
#endif
#if N_AXIS >= 6 && !defined(C_ENABLE_BIT)
#define C_ENABLE_BIT 0
#endif

#if N_AXIS == 3
#define STEPPERS_ENABLE_MASK (X_ENABLE_BIT|Y_ENABLE_BIT|Z_ENABLE_BIT)
#elif N_AXIS == 4
#define STEPPERS_ENABLE_MASK (X_ENABLE_BIT|Y_ENABLE_BIT|Z_ENABLE_BIT|A_ENABLE_BIT)
#elif N_AXIS == 5
#define STEPPERS_ENABLE_MASK (X_ENABLE_BIT|Y_ENABLE_BIT|Z_ENABLE_BIT|A_ENABLE_BIT|B_ENABLE_BIT)
#elif N_AXIS == 6
#define STEPPERS_ENABLE_MASK (X_ENABLE_BIT|Y_ENABLE_BIT|Z_ENABLE_BIT|A_ENABLE_BIT|B_ENABLE_BIT|C_ENABLE_BIT)
#elif N_AXIS == 7
#define STEPPERS_ENABLE_MASK (X_ENABLE_BIT|Y_ENABLE_BIT|Z_ENABLE_BIT|A_ENABLE_BIT|B_ENABLE_BIT|C_ENABLE_BIT|U_ENABLE_BIT)
#else
#define STEPPERS_ENABLE_MASK (X_ENABLE_BIT|Y_ENABLE_BIT|Z_ENABLE_BIT|A_ENABLE_BIT|B_ENABLE_BIT|C_ENABLE_BIT|U_ENABLE_BIT|V_ENABLE_BIT)
#endif
#endif

#endif //  STEPPERS_ENABLE_MASK

#ifndef LIMIT_MASK

#if N_AXIS >=4 && !defined(A_LIMIT_BIT)
#ifdef A_LIMIT_PIN
#define A_LIMIT_BIT (1<<A_LIMIT_PIN)
#else
#define A_LIMIT_BIT 0
#endif
#endif
#if N_AXIS >=5 && !defined(B_LIMIT_BIT)
#ifdef B_LIMIT_PIN
#define B_LIMIT_BIT (1<<B_LIMIT_PIN)
#else
#define B_LIMIT_BIT 0
#endif
#endif
#if N_AXIS >= 6 && !defined(C_LIMIT_BIT)
#ifdef A_LIMIT_PIN
#define C_LIMIT_BIT (1<<A_LIMIT_PIN)
#else
#define C_LIMIT_BIT 0
#endif
#endif

#if N_AXIS == 3
#define LIMIT_MASK (X_LIMIT_BIT|Y_LIMIT_BIT|Z_LIMIT_BIT)
#elif N_AXIS == 4
#define LIMIT_MASK (X_LIMIT_BIT|Y_LIMIT_BIT|Z_LIMIT_BIT|A_LIMIT_BIT)
#elif N_AXIS == 5
#define LIMIT_MASK (X_LIMIT_BIT|Y_LIMIT_BIT|Z_LIMIT_BIT|A_LIMIT_BIT|B_LIMIT_BIT)
#elif N_AXIS == 6
#define LIMIT_MASK (X_LIMIT_BIT|Y_LIMIT_BIT|Z_LIMIT_BIT|A_LIMIT_BIT|B_LIMIT_BIT|C_LIMIT_BIT)
#elif N_AXIS == 7
#define LIMIT_MASK (X_LIMIT_BIT|Y_LIMIT_BIT|Z_LIMIT_BIT|A_LIMIT_BIT|B_LIMIT_BIT|C_LIMIT_BIT|U_LIMIT_BIT)
#else
#define LIMIT_MASK (X_LIMIT_BIT|Y_LIMIT_BIT|Z_LIMIT_BIT|A_LIMIT_BIT|B_LIMIT_BIT|C_LIMIT_BIT|U_LIMIT_BIT|V_LIMIT_BIT)
#endif

#endif // LIMIT_MASK

#ifndef N_GANGED
#define N_GANGED 0
#endif

static void motor_iterator (motor_iterator_callback_ptr callback)
{
    motor_map_t motor;

    for(motor.id = 0; motor.id < N_AXIS + N_GANGED; motor.id++)
    {
        if(motor.id < N_AXIS)
            motor.axis = motor.id;
        else switch (motor.id) {
#ifdef X2_MOTOR
            case X2_MOTOR:
                motor.axis = X_AXIS;
                break;
#endif
#ifdef Y2_MOTOR
            case Y2_MOTOR:
                motor.axis = Y_AXIS;
                break;
#endif
#ifdef Z2_MOTOR
            case Z2_MOTOR:
                motor.axis = Z_AXIS;
                break;
#endif
        }
        callback(motor);
    }
}

/*EOF*/
