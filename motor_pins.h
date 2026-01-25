/*
  motor_pins.h - pin mappings resolver for ganged/squared/ABC axes

  NOTE: This file is not used by the core, it may be used by drivers to simplify board map files

  Part of grblHAL

  Copyright (c) 2021-2025 Terje Io

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

#pragma once

#define CAT(a, b) CAT_(a, b)
#define CAT_(a, b) a##b

#define MOTOR_IO(n, t) CAT(M, CAT(n, t))

#define mn_has_limit(a) \
 (a == 3 ? defined(M3_LIMIT_PIN) : \
 (a == 4 ? defined(M4_LIMIT_PIN) : \
 (a == 5 ? defined(M5_LIMIT_PIN) : \
 (a == 6 ? defined(M6_LIMIT_PIN) : \
 (a == 7 ? defined(M7_LIMIT_PIN) : 0)))))

#define mn_has_home(a) \
 (a == 3 ? defined(M3_HOME_PIN) : \
 (a == 4 ? defined(M4_HOME_PIN) : \
 (a == 5 ? defined(M5_HOME_PIN) : \
 (a == 6 ? defined(M6_HOME_PIN) : \
 (a == 7 ? defined(M7_HOME_PIN) : 0)))))

#define mn_has_limit_max(a) \
 (a == 3 ? defined(M3_LIMIT_PIN_MAX) : \
 (a == 4 ? defined(M4_LIMIT_PIN_MAX) : \
 (a == 5 ? defined(M5_LIMIT_PIN_MAX) : \
 (a == 6 ? defined(M6_LIMIT_PIN_MAX) : \
 (a == 7 ? defined(M7_LIMIT_PIN_MAX) : 0)))))

#define mn_has_fault(a) \
 (a == 3 ? defined(M3_MOTOR_FAULT_PIN) : \
 (a == 4 ? defined(M4_MOTOR_FAULT_PIN) : \
 (a == 5 ? defined(M5_MOTOR_FAULT_PIN) : \
 (a == 6 ? defined(M6_MOTOR_FAULT_PIN) : \
 (a == 7 ? defined(M7_MOTOR_FAULT_PIN) : 0)))))

#define mn_has_enable(a) \
 (a == 3 ? defined(M3_ENABLE_PIN) : \
 (a == 4 ? defined(M4_ENABLE_PIN) : \
 (a == 5 ? defined(M5_ENABLE_PIN) : \
 (a == 6 ? defined(M6_ENABLE_PIN) : \
 (a == 7 ? defined(M7_ENABLE_PIN) : 0)))))

#define N_MOTORS (3 + defined(M3_AVAILABLE) + defined(M4_AVAILABLE) + defined(M5_AVAILABLE) + defined(M6_AVAILABLE) + defined(M7_AVAILABLE))

#if N_AXIS > 3
 #if !defined(M3_STEP_BIT)
  #define M3_STEP_BIT (1<<M3_STEP_PIN)
 #endif
 #if !defined(M3_DIRECTION_BIT)
  #define M3_DIRECTION_BIT (1<<M3_DIRECTION_PIN)
 #endif
 #if !defined(M3_ENABLE_BIT)
  #ifdef M3_ENABLE_PIN
   #define M3_ENABLE_BIT (1<<M3_ENABLE_PIN)
  #else
   #define M3_ENABLE_BIT 0
  #endif
 #endif
 #if !defined(M3_HOME_BIT)
  #ifdef M3_HOME_PIN
   #define M3_HOME_BIT (1<<M3_HOME_PIN)
  #else
   #define M3_HOME_BIT 0
  #endif
 #endif
 #if !defined(M3_LIMIT_BIT)
  #ifdef M3_LIMIT_PIN
   #define M3_LIMIT_BIT (1>>M3_LIMIT_PIN)
  #else
   #define M3_LIMIT_BIT 0
  #endif
 #endif
 #if !defined(M3_LIMIT_MAX_BIT)
  #ifdef M3_LIMIT_MAX_PIN
   #define M3_LIMIT_MAX_BIT (1<<M3_LIMIT_MAX_PIN)
  #else
   #define M3_LIMIT_MAX_BIT 0
  #endif
 #endif
 #if !defined(M3_MOTOR_FAULT_BIT)
  #ifdef M3_MOTOR_FAULT_PIN
   #define M3_MOTOR_FAULT_BIT (1<<M3_MOTOR_FAULT_PIN)
  #else
   #define M3_MOTOR_FAULT_BIT 0
  #endif
 #endif
#endif

#if N_AXIS > 4
 #if !defined(M4_STEP_BIT)
  #define M4_STEP_BIT (1<<M4_STEP_PIN)
 #endif
 #if !defined(M4_DIRECTION_BIT)
  #define M4_DIRECTION_BIT (1<<M4_DIRECTION_PIN)
 #endif
 #if !defined(M4_ENABLE_BIT)
  #ifdef M4_ENABLE_PIN
   #define M4_ENABLE_BIT (1<<M4_ENABLE_PIN)
  #else
   #define M4_ENABLE_BIT 0
  #endif
 #endif
 #if !defined(M4_HOME_BIT)
  #ifdef M4_HOME_PIN
   #define M4_HOME_BIT (1<<M4_HOME_PIN)
  #else
   #define M4_HOME_BIT 0
  #endif
 #endif
 #if !defined(M4_LIMIT_BIT)
  #ifdef M4_LIMIT_PIN
   #define M4_LIMIT_BIT (1>>M4_LIMIT_PIN)
  #else
   #define M4_LIMIT_BIT 0
  #endif
 #endif
 #if !defined(M4_LIMIT_MAX_BIT)
  #ifdef M4_LIMIT_MAX_PIN
   #define M4_LIMIT_MAX_BIT (1<<M4_LIMIT_MAX_PIN)
  #else
   #define M4_LIMIT_MAX_BIT 0
  #endif
 #endif
 #if !defined(M4_MOTOR_FAULT_BIT)
  #ifdef M4_MOTOR_FAULT_PIN
   #define M4_MOTOR_FAULT_BIT (1<<M4_MOTOR_FAULT_PIN)
  #else
   #define M4_MOTOR_FAULT_BIT 0
  #endif
 #endif
#endif

#if N_AXIS > 5
 #if !defined(M5_STEP_BIT)
  #define M5_STEP_BIT (1<<M5_STEP_PIN)
 #endif
 #if !defined(M5_DIRECTION_BIT)
  #define M5_DIRECTION_BIT (1<<M5_DIRECTION_PIN)
 #endif
 #if !defined(M5_ENABLE_BIT)
  #ifdef M5_ENABLE_PIN
   #define M5_ENABLE_BIT (1<<M5_ENABLE_PIN)
  #else
   #define M5_ENABLE_BIT 0
  #endif
 #endif
 #if !defined(M5_HOME_BIT)
  #ifdef M5_HOME_PIN
   #define M5_HOME_BIT (1<<M5_HOME_PIN)
  #else
   #define M5_HOME_BIT 0
  #endif
 #endif
 #if !defined(M5_LIMIT_BIT)
  #ifdef M5_LIMIT_PIN
   #define M5_LIMIT_BIT (1>>M5_LIMIT_PIN)
  #else
   #define M5_LIMIT_BIT 0
  #endif
 #endif
 #if !defined(M5_LIMIT_MAX_BIT)
  #ifdef M5_LIMIT_MAX_PIN
   #define M5_LIMIT_MAX_BIT (1<<M5_LIMIT_MAX_PIN)
  #else
   #define M5_LIMIT_MAX_BIT 0
  #endif
 #endif
 #if !defined(M5_MOTOR_FAULT_BIT)
  #ifdef M5_MOTOR_FAULT_PIN
   #define M5_MOTOR_FAULT_BIT (1<<M5_MOTOR_FAULT_PIN)
  #else
   #define M5_MOTOR_FAULT_BIT 0
  #endif
 #endif
#endif

#if N_AXIS > 6
 #if !defined(M6_STEP_BIT)
  #define M6_STEP_BIT (1<<M6_STEP_PIN)
 #endif
 #if !defined(M6_DIRECTION_BIT)
  #define M6_DIRECTION_BIT (1<<M6_DIRECTION_PIN)
 #endif
 #if !defined(M6_ENABLE_BIT)
  #ifdef M6_ENABLE_PIN
   #define M6_ENABLE_BIT (1<<M6_ENABLE_PIN)
  #else
   #define M6_ENABLE_BIT 0
  #endif
 #endif
 #if !defined(M6_HOME_BIT)
  #ifdef M6_HOME_PIN
   #define M6_HOME_BIT (1<<M6_HOME_PIN)
  #else
   #define M6_HOME_BIT 0
  #endif
 #endif
 #if !defined(M6_LIMIT_BIT)
  #ifdef M6_LIMIT_PIN
   #define M6_LIMIT_BIT (1>>M6_LIMIT_PIN)
  #else
   #define M6_LIMIT_BIT 0
  #endif
 #endif
 #if !defined(M6_LIMIT_MAX_BIT)
  #ifdef M6_LIMIT_MAX_PIN
   #define M6_LIMIT_MAX_BIT (1<<M6_LIMIT_MAX_PIN)
  #else
   #define M6_LIMIT_MAX_BIT 0
  #endif
 #endif
 #if !defined(M6_MOTOR_FAULT_BIT)
  #ifdef M6_MOTOR_FAULT_PIN
   #define M6_MOTOR_FAULT_BIT (1<<M6_MOTOR_FAULT_PIN)
  #else
   #define M6_MOTOR_FAULT_BIT 0
  #endif
 #endif
#endif

#if N_AXIS > 7
 #if !defined(M7_STEP_BIT)
  #define M7_STEP_BIT (1<<M7_STEP_PIN)
 #endif
 #if !defined(M7_DIRECTION_BIT)
  #define M7_DIRECTION_BIT (1<<M7_DIRECTION_PIN)
 #endif
 #if !defined(M7_ENABLE_BIT)
  #ifdef M7_ENABLE_PIN
   #define M7_ENABLE_BIT (1<<M7_ENABLE_PIN)
  #else
   #define M7_ENABLE_BIT 0
  #endif
 #endif
 #if !defined(M7_HOME_BIT)
  #ifdef M7_HOME_PIN
   #define M7_HOME_BIT (1<<M7_HOME_PIN)
  #else
   #define M7_HOME_BIT 0
  #endif
 #endif
 #if !defined(M7_LIMIT_BIT)
  #ifdef M7_LIMIT_PIN
   #define M7_LIMIT_BIT (1>>M7_LIMIT_PIN)
  #else
   #define M7_LIMIT_BIT 0
  #endif
 #endif
 #if !defined(M7_LIMIT_MAX_BIT)
  #ifdef M7_LIMIT_MAX_PIN
   #define M7_LIMIT_MAX_BIT (1<<M7_LIMIT_MAX_PIN)
  #else
   #define M7_LIMIT_MAX_BIT 0
  #endif
 #endif
 #if !defined(M7_MOTOR_FAULT_BIT)
  #ifdef M7_MOTOR_FAULT_PIN
   #define M7_MOTOR_FAULT_BIT (1<<M7_MOTOR_FAULT_PIN)
  #else
   #define M7_MOTOR_FAULT_BIT 0
  #endif
 #endif
#endif

#if N_GANGED

#if N_GANGED > N_ABC_MOTORS
#error "Axis configuration is not supported!"
#endif

#if N_AUTO_SQUARED
#define SQUARING_ENABLED
#endif

#if N_GANGED
#define GANGING_ENABLED
#endif

#if Z_GANGED
 #if N_MOTORS == 8
  #define Z2_MOTOR_IDX 7
 #elif N_MOTORS == 7
  #define Z2_MOTOR_IDX 6
 #elif N_MOTORS == 6
  #define Z2_MOTOR_IDX 5
 #elif N_MOTORS == 5
  #define Z2_MOTOR_IDX 4
 #elif N_MOTORS == 4
  #define Z2_MOTOR_IDX 3
 #endif
#elif Y_GANGED
 #if N_MOTORS == 8
  #define Y2_MOTOR_IDX 7
 #elif N_MOTORS == 7
  #define Y2_MOTOR_IDX 6
 #elif N_MOTORS == 6
  #define Y2_MOTOR_IDX 5
 #elif N_MOTORS == 5
  #define Y2_MOTOR_IDX 4
 #elif N_MOTORS == 4
  #define Y2_MOTOR_IDX 3
 #endif
#elif X_GANGED
 #if N_MOTORS == 8
  #define X2_MOTOR_IDX 7
 #elif N_MOTORS == 7
  #define X2_MOTOR_IDX 6
 #elif N_MOTORS == 6
  #define X2_MOTOR_IDX 5
 #elif N_MOTORS == 5
  #define X2_MOTOR_IDX 4
 #elif N_MOTORS == 4
  #define X2_MOTOR_IDX 3
 #endif
#endif

#if Y_GANGED && !defined(Y2_MOTOR_IDX)
 #if N_MOTORS == 8
  #define Y2_MOTOR_IDX 6
 #elif N_MOTORS == 7
  #define Y2_MOTOR_IDX 5
 #elif N_MOTORS == 6
  #define Y2_MOTOR_IDX 4
 #elif N_MOTORS == 5
  #define Y2_MOTOR_IDX 3
 #endif
#elif X_GANGED && !defined(X2_MOTOR_IDX)
 #if N_MOTORS == 8
  #define X2_MOTOR_IDX 6
 #elif N_MOTORS == 7
  #define X2_MOTOR_IDX 5
 #elif N_MOTORS == 6
  #define X2_MOTOR_IDX 4
 #elif N_MOTORS == 5
  #define X2_MOTOR_IDX 3
 #endif
#endif

#if X_GANGED && !defined(X2_MOTOR_IDX)
 #if N_MOTORS == 8
  #define X2_MOTOR_IDX 5
 #elif N_MOTORS == 7
  #define X2_MOTOR_IDX 4
 #elif N_MOTORS == 6
  #define X2_MOTOR_IDX 3
 #endif
#endif

#ifdef X2_MOTOR_IDX

#define X2_STEP_PORT            MOTOR_IO(X2_MOTOR_IDX, _STEP_PORT)
#define X2_STEP_PIN             MOTOR_IO(X2_MOTOR_IDX, _STEP_PIN)
#define X2_STEP_BIT             (1<<X2_STEP_PIN)
#define X2_DIRECTION_PORT       MOTOR_IO(X2_MOTOR_IDX, _DIRECTION_PORT)
#define X2_DIRECTION_PIN        MOTOR_IO(X2_MOTOR_IDX, _DIRECTION_PIN)
#define X2_DIRECTION_BIT        (1<<X2_DIRECTION_PIN)
#if mn_has_home(X2_MOTOR_IDX)
 #if X_AUTO_SQUARE
    #define X2_HOME_PORT        MOTOR_IO(X2_MOTOR_IDX, _HOME_PORT)
    #define X2_HOME_PIN         MOTOR_IO(X2_MOTOR_IDX, _HOME_PIN)
    #define X2_HOME_BIT         (1<<X2_HOME_PIN)
 #endif
#elif X_AUTO_SQUARE && defined(X_HOME_PIN)
  #error "Auto squared Y-axis requires second home pin input"
#endif
#if mn_has_limit(X2_MOTOR_IDX)
 #if X_AUTO_SQUARE
  #define X2_LIMIT_PORT         MOTOR_IO(X2_MOTOR_IDX, _LIMIT_PORT)
  #define X2_LIMIT_PIN          MOTOR_IO(X2_MOTOR_IDX, _LIMIT_PIN)
  #define X2_LIMIT_BIT          (1<<X2_LIMIT_PIN)
 #elif X_GANGED_LIM_MAX && !mn_has_limit_max(X2_MOTOR_IDX)
  #define X2_LIMIT_MAX_PORT     MOTOR_IO(X2_MOTOR_IDX, _LIMIT_MAX_PORT)
  #define X2_LIMIT_MAX_PIN      MOTOR_IO(X2_MOTOR_IDX, _LIMIT_MAX_PIN)
  #define X2_LIMIT_MAX_BIT      (1<<X2_LIMIT_MAX_PIN)
 #endif
#elif X_AUTO_SQUARE
  #error "Auto squared Y-axis requires second limit pin input"
#endif
#if mn_has_limit_max(X2_MOTOR_IDX)
  #define X2_LIMIT_MAX_PORT     MOTOR_IO(X2_MOTOR_IDX, _LIMIT_MAX_PORT)
  #define X2_LIMIT_MAX_PIN      MOTOR_IO(X2_MOTOR_IDX, _LIMIT_MAX_PIN)
  #define X2_LIMIT_MAX_BIT      (1<<X2_LIMIT_MAX_PIN)
#endif
#if mn_has_enable(X2_MOTOR_IDX)
  #define X2_ENABLE_PORT        MOTOR_IO(X2_MOTOR_IDX, _ENABLE_PORT)
  #define X2_ENABLE_PIN         MOTOR_IO(X2_MOTOR_IDX, _ENABLE_PIN)
  #define X2_ENABLE_BIT         (1<<X2_ENABLE_PIN)
#endif
#if mn_has_fault(X2_MOTOR_IDX)
  #define X2_MOTOR_FAULT_PORT   MOTOR_IO(X2_MOTOR_IDX, _MOTOR_FAULT_PORT)
  #define X2_MOTOR_FAULT_PIN    MOTOR_IO(X2_MOTOR_IDX, _MOTOR_FAULT_PIN)
  #define X2_MOTOR_FAULT_BIT    (1<<X2_MOTOR_FAULT_PIN)
#endif

#endif // X_GANGED

#ifdef Y2_MOTOR_IDX

#define Y2_STEP_PORT            MOTOR_IO(Y2_MOTOR_IDX, _STEP_PORT)
#define Y2_STEP_PIN             MOTOR_IO(Y2_MOTOR_IDX, _STEP_PIN)
#define Y2_STEP_BIT             (1<<Y2_STEP_PIN)
#define Y2_DIRECTION_PORT       MOTOR_IO(Y2_MOTOR_IDX, _DIRECTION_PORT)
#define Y2_DIRECTION_PIN        MOTOR_IO(Y2_MOTOR_IDX, _DIRECTION_PIN)
#define Y2_DIRECTION_BIT        (1<<Y2_DIRECTION_PIN)
#if mn_has_home(Y2_MOTOR_IDX)
 #if Y_AUTO_SQUARE
    #define Y2_HOME_PORT        MOTOR_IO(Y2_MOTOR_IDX, _HOME_PORT)
    #define Y2_HOME_PIN         MOTOR_IO(Y2_MOTOR_IDX, _HOME_PIN)
    #define Y2_HOME_BIT         (1<<Y2_HOME_PIN)
 #endif
#elif Y_AUTO_SQUARE && defined(Y_HOME_PIN)
  #error "Auto squared Y-axis requires second home pin input"
#endif
#if mn_has_limit(Y2_MOTOR_IDX)
 #if Y_AUTO_SQUARE
  #define Y2_LIMIT_PORT         MOTOR_IO(Y2_MOTOR_IDX, _LIMIT_PORT)
  #define Y2_LIMIT_PIN          MOTOR_IO(Y2_MOTOR_IDX, _LIMIT_PIN)
  #define Y2_LIMIT_BIT          (1<<Y2_LIMIT_PIN)
 #elif Y_GANGED_LIM_MAX && !mn_has_limit_max(Y2_MOTOR_IDX)
  #define Y2_LIMIT_MAX_PORT     MOTOR_IO(Y2_MOTOR_IDX, _LIMIT_MAX_PORT)
  #define Y2_LIMIT_MAX_PIN      MOTOR_IO(Y2_MOTOR_IDX, _LIMIT_MAX_PIN)
  #define Y2_LIMIT_MAX_BIT      (1<<Y2_LIMIT_MAX_PIN)
 #endif
#elif Y_AUTO_SQUARE
  #error "Auto squared Y-axis requires second limit pin input"
#endif
#if mn_has_limit_max(Y2_MOTOR_IDX)
  #define Y2_LIMIT_MAX_PORT     MOTOR_IO(Y2_MOTOR_IDX, _LIMIT_MAX_PORT)
  #define Y2_LIMIT_MAX_PIN      MOTOR_IO(Y2_MOTOR_IDX, _LIMIT_MAX_PIN)
  #define Y2_LIMIT_MAX_BIT      (1<<Y2_LIMIT_MAX_PIN)
#endif
#if mn_has_enable(Y2_MOTOR_IDX)
  #define Y2_ENABLE_PORT        MOTOR_IO(Y2_MOTOR_IDX, _ENABLE_PORT)
  #define Y2_ENABLE_PIN         MOTOR_IO(Y2_MOTOR_IDX, _ENABLE_PIN)
  #define Y2_ENABLE_BIT         (1<<Y2_ENABLE_PIN)
#endif
#if mn_has_fault(Y2_MOTOR_IDX)
  #define Y2_MOTOR_FAULT_PORT   MOTOR_IO(Y2_MOTOR_IDX, _MOTOR_FAULT_PORT)
  #define Y2_MOTOR_FAULT_PIN    MOTOR_IO(Y2_MOTOR_IDX, _MOTOR_FAULT_PIN)
  #define Y2_MOTOR_FAULT_BIT    (1<<Y2_MOTOR_FAULT_PIN)
#endif

#endif // Y_GANGED

#ifdef Z2_MOTOR_IDX

#define Z2_STEP_PORT            MOTOR_IO(Z2_MOTOR_IDX, _STEP_PORT)
#define Z2_STEP_PIN             MOTOR_IO(Z2_MOTOR_IDX, _STEP_PIN)
#define Z2_STEP_BIT             (1<<<Z2_STEP_PIN)
#define Z2_DIRECTION_PORT       MOTOR_IO(Z2_MOTOR_IDX, _DIRECTION_PORT)
#define Z2_DIRECTION_PIN        MOTOR_IO(Z2_MOTOR_IDX, _DIRECTION_PIN)
#define Z2_DIRECTION_BIT        (1<<Z2_DIRECTION_PIN)
#if mn_has_home(Z2_MOTOR_IDX)
 #if Z_AUTO_SQUARE
    #define Z2_HOME_PORT        MOTOR_IO(Z2_MOTOR_IDX, _HOME_PORT)
    #define Z2_HOME_PIN         MOTOR_IO(Z2_MOTOR_IDX, _HOME_PIN)
    #define Z2_HOME_BIT         (1<<Z2_HOME_PIN)
 #endif
#elif Z_AUTO_SQUARE && defined(Z_HOME_PIN)
  #error "Auto squared Y-axis requires second home pin input"
#endif
#if mn_has_limit(Z2_MOTOR_IDX)
 #if Z_AUTO_SQUARE
  #define Z2_LIMIT_PORT         MOTOR_IO(Z2_MOTOR_IDX, _LIMIT_PORT)
  #define Z2_LIMIT_PIN          MOTOR_IO(Z2_MOTOR_IDX, _LIMIT_PIN)
  #define Z2_LIMIT_BIT          (1<<Z2_LIMIT_PIN)
 #elif Z_GANGED_LIM_MAX && !mn_has_limit_max(Z2_MOTOR_IDX)
  #define Z2_LIMIT_MAX_PORT     MOTOR_IO(Z2_MOTOR_IDX, _LIMIT_MAX_PORT)
  #define Z2_LIMIT_MAX_PIN      MOTOR_IO(Z2_MOTOR_IDX, _LIMIT_MAX_PIN)
  #define Z2_LIMIT_MAX_BIT      (1<<Z2_LIMIT_MAX_PIN)
 #endif
#elif Z_AUTO_SQUARE
  #error "Auto squared Y-axis requires second limit pin input"
#endif
#if mn_has_limit_max(Z2_MOTOR_IDX)
  #define Z2_LIMIT_MAX_PORT     MOTOR_IO(Z2_MOTOR_IDX, _LIMIT_MAX_PORT)
  #define Z2_LIMIT_MAX_PIN      MOTOR_IO(Z2_MOTOR_IDX, _LIMIT_MAX_PIN)
  #define Z2_LIMIT_MAX_BIT      (1<<Z2_LIMIT_MAX_PIN)
#endif
#if mn_has_enable(Z2_MOTOR_IDX)
  #define Z2_ENABLE_PORT        MOTOR_IO(Z2_MOTOR_IDX, _ENABLE_PORT)
  #define Z2_ENABLE_PIN         MOTOR_IO(Z2_MOTOR_IDX, _ENABLE_PIN)
  #define Z2_ENABLE_BIT         (1<<Z2_ENABLE_PIN)
#endif
#if mn_has_fault(Z2_MOTOR_IDX)
  #define Z2_MOTOR_FAULT_PORT   MOTOR_IO(Z2_MOTOR_IDX, _MOTOR_FAULT_PORT)
  #define Z2_MOTOR_FAULT_PIN    MOTOR_IO(Z2_MOTOR_IDX, _MOTOR_FAULT_PIN)
  #define Z2_MOTOR_FAULT_BIT    (1<<Z2_MOTOR_FAULT_PIN)
#endif

#endif // Z_GANGED

#endif // N_GANGED

#if defined(X2_HOME_PIN) || defined(Y2_HOME_PIN) || defined(Z2_HOME_PIN)
#define DUAL_HOME_SWITCHES
#ifndef X2_HOME_BIT
#define X2_HOME_BIT 0
#endif
#ifndef Y2_HOME_BIT
#define Y2_HOME_BIT 0
#endif
#ifndef Z2_HOME_BIT
#define Z2_HOME_BIT 0
#endif
#define HOME2_MASK (X2_HOME_BIT|Y2_HOME_BIT|Z2_HOME_BIT)
#define HOME2_MASK_SUM (X2_HOME_BIT+Y2_HOME_BIT+Z2_HOME_BIT)
#else
#define HOME2_MASK 0
#define HOME2_MASK_SUM 0
#endif

#if defined(X2_MOTOR_FAULT_PIN) || defined(Y2_MOTOR_FAULT_PIN) || defined(Z2_MOTOR_FAULT_PIN)
#define DUAL_MOTOR_FAULT_SWITCHES
#ifndef X2_MOTOR_FAULT_BIT
#define X2_MOTOR_FAULT_BIT 0
#endif
#ifndef Y2_MOTOR_FAULT_BIT
#define Y2_MOTOR_FAULT_BIT 0
#endif
#ifndef Z2_MOTOR_FAULT_BIT
#define Z2_MOTOR_FAULT_BIT 0
#endif
#define MOTOR_FAULT2_MASK (X2_MOTOR_FAULT_BIT|Y2_MOTOR_FAULT_BIT|Z2_MOTOR_FAULT_BIT)
#define MOTOR_FAULT2_MASK_SUM (X2_MOTOR_FAULT_BIT+Y2_MOTOR_FAULT_BIT+Z2_MOTOR_FAULT_BIT)
#else
#define MOTOR_FAULT2_MASK 0
#define MOTOR_FAULT2_MASK_SUM 0
#endif

#if defined(X2_LIMIT_PIN) || defined(Y2_LIMIT_PIN) || defined(Z2_LIMIT_PIN)
#define DUAL_LIMIT_SWITCHES
#ifndef X2_LIMIT_BIT
#define X2_LIMIT_BIT 0
#endif
#ifndef Y2_LIMIT_BIT
#define Y2_LIMIT_BIT 0
#endif
#ifndef Z2_LIMIT_BIT
#define Z2_LIMIT_BIT 0
#endif
#define LIMIT2_MASK (X2_LIMIT_BIT|Y2_LIMIT_BIT|Z2_LIMIT_BIT)
#define LIMIT2_MASK_SUM (X2_LIMIT_BIT+Y2_LIMIT_BIT+Z2_LIMIT_BIT)
#else
#define LIMIT2_MASK 0
#define LIMIT2_MASK_SUM 0
#endif

#if defined(X_LIMIT_PIN_MAX) || defined(Y_LIMIT_PIN_MAX) || defined(Z_LIMIT_PIN_MAX) || defined(A_LIMIT_PIN_MAX) || defined(B_LIMIT_PIN_MAX) || defined(C_LIMIT_PIN_MAX)
#define MAX_LIMIT_SWITCHES
#endif

#ifdef A_AXIS

#if A_AXIS == 3
#define A_AXIS_IDX 3
#elif A_AXIS == 4
#define A_AXIS_IDX 4
#elif A_AXIS == 4
#define A_AXIS_IDX 5
#elif A_AXIS == 4
#define A_AXIS_IDX 6
#elif A_AXIS == 4
#define A_AXIS_IDX 7
#endif

#if A_AXIS_IDX >= N_MOTORS
  #error "No pins are available for A axis motor"
#endif

#define A_STEP_PORT             MOTOR_IO(A_AXIS_IDX, _STEP_PORT)
#define A_STEP_PIN              MOTOR_IO(A_AXIS_IDX, _STEP_PIN)
#define A_STEP_BIT              (1<<A_STEP_PIN)
#define A_DIRECTION_PORT        MOTOR_IO(A_AXIS_IDX, _DIRECTION_PORT)
#define A_DIRECTION_PIN         MOTOR_IO(A_AXIS_IDX, _DIRECTION_PIN)
#define A_DIRECTION_BIT         (1<<A_DIRECTION_PIN)
#if mn_has_home(A_MOTOR_IDX)
  #define A_HOME_PORT           MOTOR_IO(A_AXIS_IDX, _HOME_PORT)
  #define A_HOME_PIN            MOTOR_IO(A_AXIS_IDX, _HOME_PIN)
  #define A_HOME_BIT            (1<<A_HOME_PIN)
#endif
#if mn_has_limit(A_AXIS_IDX)
  #define A_LIMIT_PORT          MOTOR_IO(A_AXIS_IDX, _LIMIT_PORT)
  #define A_LIMIT_PIN           MOTOR_IO(A_AXIS_IDX, _LIMIT_PIN)
  #define A_LIMIT_BIT           (1<<A_LIMIT_PIN)
#endif
#if mn_has_limit_max(A_AXIS_IDX)
  #define A_LIMIT_MAX_PORT      MOTOR_IO(A_AXIS_IDX, _LIMIT_MAX_PORT)
  #define A_LIMIT_MAX_PIN       MOTOR_IO(A_AXIS_IDX, _LIMIT_MAX_PIN)
  #define A_LIMIT_MAX_BIT       (1<<A_LIMIT_MAX_PIN)
#endif
#if mn_has_enable(A_AXIS_IDX)
  #define A_ENABLE_PORT         MOTOR_IO(A_AXIS_IDX, _ENABLE_PORT)
  #define A_ENABLE_PIN          MOTOR_IO(A_AXIS_IDX, _ENABLE_PIN)
  #define A_ENABLE_BIT          (1<<A_ENABLE_PIN)
#endif
#if mn_has_fault(A_AXIS_IDX)
  #define A_MOTOR_FAULT_PORT    MOTOR_IO(A_AXIS_IDX, _MOTOR_FAULT_PORT)
  #define A_MOTOR_FAULT_PIN     MOTOR_IO(A_AXIS_IDX, _MOTOR_FAULT_PIN)
  #define A_MOTOR_FAULT_BIT     (1<<A_MOTOR_FAULT_PIN)
#endif

#endif // A_AXIS

#ifdef B_AXIS

#if B_AXIS == 3
#define B_AXIS_IDX 3
#elif B_AXIS == 4
#define B_AXIS_IDX 4
#elif B_AXIS == 4
#define B_AXIS_IDX 5
#elif B_AXIS == 4
#define B_AXIS_IDX 6
#elif B_AXIS == 4
#define B_AXIS_IDX 7
#endif

#if B_AXIS_IDX >= N_MOTORS
  #error "No pins are available for B axis motor"
#endif

#define B_STEP_PORT             MOTOR_IO(B_AXIS_IDX, _STEP_PORT)
#define B_STEP_PIN              MOTOR_IO(B_AXIS_IDX, _STEP_PIN)
#define B_STEP_BIT              (1<<B_STEP_PIN)
#define B_DIRECTION_PORT        MOTOR_IO(B_AXIS_IDX, _DIRECTION_PORT)
#define B_DIRECTION_PIN         MOTOR_IO(B_AXIS_IDX, _DIRECTION_PIN)
#define B_DIRECTION_BIT         (1<<B_DIRECTION_PIN)
#if mn_has_home(B_MOTOR_IDX)
  #define B_HOME_PORT           MOTOR_IO(B_AXIS_IDX, _HOME_PORT)
  #define B_HOME_PIN            MOTOR_IO(B_AXIS_IDX, _HOME_PIN)
  #define B_HOME_BIT            (1<<B_HOME_PIN)
#endif
#if mn_has_limit(B_AXIS_IDX)
  #define B_LIMIT_PORT          MOTOR_IO(B_AXIS_IDX, _LIMIT_PORT)
  #define B_LIMIT_PIN           MOTOR_IO(B_AXIS_IDX, _LIMIT_PIN)
  #define B_LIMIT_BIT           (1<<B_LIMIT_PIN)
#endif
#if mn_has_limit_max(B_AXIS_IDX)
  #define B_LIMIT_MAX_PORT      MOTOR_IO(B_AXIS_IDX, _LIMIT_MAX_PORT)
  #define B_LIMIT_MAX_PIN       MOTOR_IO(B_AXIS_IDX, _LIMIT_MAX_PIN)
  #define B_LIMIT_MAX_BIT       (1<<B_LIMIT_MAX_PIN)
#endif
#if mn_has_enable(B_AXIS_IDX)
  #define B_ENABLE_PORT         MOTOR_IO(B_AXIS_IDX, _ENABLE_PORT)
  #define B_ENABLE_PIN          MOTOR_IO(B_AXIS_IDX, _ENABLE_PIN)
  #define B_ENABLE_BIT          (1<<B_ENABLE_PIN)
#endif
#if mn_has_fault(B_AXIS_IDX)
  #define B_MOTOR_FAULT_PORT    MOTOR_IO(B_AXIS_IDX, _MOTOR_FAULT_PORT)
  #define B_MOTOR_FAULT_PIN     MOTOR_IO(B_AXIS_IDX, _MOTOR_FAULT_PIN)
  #define B_MOTOR_FAULT_BIT     (1<<B_MOTOR_FAULT_PIN)
#endif

#endif //B_AXIS

#ifdef C_AXIS

#if C_AXIS == 3
#define C_AXIS_IDX 3
#elif C_AXIS == 4
#define C_AXIS_IDX 4
#elif C_AXIS == 4
#define C_AXIS_IDX 5
#elif C_AXIS == 5
#define C_AXIS_IDX 6
#elif C_AXIS == 6
#define C_AXIS_IDX 7
#endif

#if C_AXIS_IDX >= N_MOTORS
  #error "No pins are available for C axis motor"
#endif

#define C_STEP_PORT             MOTOR_IO(C_AXIS_IDX, _STEP_PORT)
#define C_STEP_PIN              MOTOR_IO(C_AXIS_IDX, _STEP_PIN)
#define C_STEP_BIT              (1<<C_STEP_PIN)
#define C_DIRECTION_PORT        MOTOR_IO(C_AXIS_IDX, _DIRECTION_PORT)
#define C_DIRECTION_PIN         MOTOR_IO(C_AXIS_IDX, _DIRECTION_PIN)
#define C_DIRECTION_BIT         (1<<C_DIRECTION_PIN)
#if mn_has_home(C_MOTOR_IDX)
  #define C_HOME_PORT           MOTOR_IO(C_AXIS_IDX, _HOME_PORT)
  #define C_HOME_PIN            MOTOR_IO(C_AXIS_IDX, _HOME_PIN)
  #define C_HOME_BIT            (1<<C_HOME_PIN)
#endif
#if mn_has_limit(C_AXIS_IDX)
  #define C_LIMIT_PORT          MOTOR_IO(C_AXIS_IDX, _LIMIT_PORT)
  #define C_LIMIT_PIN           MOTOR_IO(C_AXIS_IDX, _LIMIT_PIN)
  #define C_LIMIT_BIT           (1<<C_LIMIT_PIN)
#endif
#if mn_has_limit_max(C_AXIS_IDX)
  #define C_LIMIT_MAX_PORT      MOTOR_IO(C_AXIS_IDX, _LIMIT_MAX_PORT)
  #define C_LIMIT_MAX_PIN       MOTOR_IO(C_AXIS_IDX, _LIMIT_MAX_PIN)
  #define C_LIMIT_MAX_BIT       (1<<C_LIMIT_MAX_PIN)
#endif
#if mn_has_enable(C_AXIS_IDX)
  #define C_ENABLE_PORT         MOTOR_IO(C_AXIS_IDX, _ENABLE_PORT)
  #define C_ENABLE_PIN          MOTOR_IO(C_AXIS_IDX, _ENABLE_PIN)
  #define C_ENABLE_BIT          (1<<C_ENABLE_PIN)
#endif
#if mn_has_fault(C_AXIS_IDX)
  #define C_MOTOR_FAULT_PORT    MOTOR_IO(C_AXIS_IDX, _MOTOR_FAULT_PORT)
  #define C_MOTOR_FAULT_PIN     MOTOR_IO(C_AXIS_IDX, _MOTOR_FAULT_PIN)
  #define C_MOTOR_FAULT_BIT     (1<<C_MOTOR_FAULT_PIN)
#endif

#endif // C_AXIS

#ifdef U_AXIS

#if U_AXIS == 3
#define U_AXIS_IDX 3
#elif U_AXIS == 4
#define U_AXIS_IDX 4
#elif U_AXIS == 5
#define U_AXIS_IDX 5
#elif U_AXIS == 6
#define U_AXIS_IDX 6
#elif U_AXIS == 7
#define U_AXIS_IDX 7
#endif

#if U_AXIS_IDX >= N_MOTORS
  #error "No pins are available for U axis motor"
#endif

#define U_STEP_PORT             MOTOR_IO(U_AXIS_IDX, _STEP_PORT)
#define U_STEP_PIN              MOTOR_IO(U_AXIS_IDX, _STEP_PIN)
#define U_STEP_BIT              (1<<U_STEP_PIN)
#define U_DIRECTION_PORT        MOTOR_IO(U_AXIS_IDX, _DIRECTION_PORT)
#define U_DIRECTION_PIN         MOTOR_IO(U_AXIS_IDX, _DIRECTION_PIN)
#define U_DIRECTION_BIT         (1<<U_DIRECTION_PIN)
#if mn_has_home(U_MOTOR_IDX)
  #define U_HOME_PORT           MOTOR_IO(U_AXIS_IDX, _HOME_PORT)
  #define U_HOME_PIN            MOTOR_IO(U_AXIS_IDX, _HOME_PIN)
  #define U_HOME_BIT            (1<<U_HOME_PIN)
#endif
#if mn_has_limit(U_AXIS_IDX)
  #define U_LIMIT_PORT          MOTOR_IO(U_AXIS_IDX, _LIMIT_PORT)
  #define U_LIMIT_PIN           MOTOR_IO(U_AXIS_IDX, _LIMIT_PIN)
  #define U_LIMIT_BIT           (1<<U_LIMIT_PIN)
#endif
#if mn_has_limit_max(U_AXIS_IDX)
  #define U_LIMIT_MAX_PORT      MOTOR_IO(U_AXIS_IDX, _LIMIT_MAX_PORT)
  #define U_LIMIT_MAX_PIN       MOTOR_IO(U_AXIS_IDX, _LIMIT_MAX_PIN)
  #define U_LIMIT_MAX_BIT       (1<<U_LIMIT_MAX_PIN)
#endif
#if mn_has_enable(U_AXIS_IDX)
  #define U_ENABLE_PORT         MOTOR_IO(U_AXIS_IDX, _ENABLE_PORT)
  #define U_ENABLE_PIN          MOTOR_IO(U_AXIS_IDX, _ENABLE_PIN)
  #define U_ENABLE_BIT          (1<<U_ENABLE_PIN)
#endif
#if mn_has_fault(U_AXIS_IDX)
  #define U_MOTOR_FAULT_PORT    MOTOR_IO(U_AXIS_IDX, _MOTOR_FAULT_PORT)
  #define U_MOTOR_FAULT_PIN     MOTOR_IO(U_AXIS_IDX, _MOTOR_FAULT_PIN)
  #define U_MOTOR_FAULT_BIT     (1<<U_MOTOR_FAULT_PIN)
#endif

#endif // U_AXIS

#ifdef V_AXIS

#if V_AXIS == 3
#define V_AXIS_IDX 3
#elif V_AXIS == 4
#define V_AXIS_IDX 4
#elif V_AXIS == 5
#define V_AXIS_IDX 5
#elif V_AXIS == 6
#define V_AXIS_IDX 6
#elif V_AXIS == 7
#define V_AXIS_IDX 7
#endif

#if V_AXIS_IDX >= N_MOTORS
  #error "No pins are available for V axis motor"
#endif

#define V_STEP_PORT             MOTOR_IO(V_AXIS_IDX, _STEP_PORT)
#define V_STEP_PIN              MOTOR_IO(V_AXIS_IDX, _STEP_PIN)
#define V_STEP_BIT              (1<<V_STEP_PIN)
#define V_DIRECTION_PORT        MOTOR_IO(V_AXIS_IDX, _DIRECTION_PORT)
#define V_DIRECTION_PIN         MOTOR_IO(V_AXIS_IDX, _DIRECTION_PIN)
#define V_DIRECTION_BIT         (1<<V_DIRECTION_PIN)
#if mn_has_home(V_MOTOR_IDX)
  #define V_HOME_PORT           MOTOR_IO(V_AXIS_IDX, _HOME_PORT)
  #define V_HOME_PIN            MOTOR_IO(V_AXIS_IDX, _HOME_PIN)
  #define V_HOME_BIT            (1<<V_HOME_PIN)
#endif
#if mn_has_limit(V_AXIS_IDX)
  #define V_LIMIT_PORT          MOTOR_IO(V_AXIS_IDX, _LIMIT_PORT)
  #define V_LIMIT_PIN           MOTOR_IO(V_AXIS_IDX, _LIMIT_PIN)
  #define V_LIMIT_BIT           (1<<V_LIMIT_PIN)
#endif
#if mn_has_limit_max(V_AXIS_IDX)
  #define V_LIMIT_MAX_PORT      MOTOR_IO(V_AXIS_IDX, _LIMIT_MAX_PORT)
  #define V_LIMIT_MAX_PIN       MOTOR_IO(V_AXIS_IDX, _LIMIT_MAX_PIN)
  #define V_LIMIT_MAX_BIT       (1<<V_LIMIT_MAX_PIN)
#endif
#if mn_has_enable(V_AXIS_IDX)
  #define V_ENABLE_PORT         MOTOR_IO(V_AXIS_IDX, _ENABLE_PORT)
  #define V_ENABLE_PIN          MOTOR_IO(V_AXIS_IDX, _ENABLE_PIN)
  #define V_ENABLE_BIT          (1<<V_ENABLE_PIN)
#endif
#if mn_has_fault(V_AXIS_IDX)
  #define V_MOTOR_FAULT_PORT    MOTOR_IO(V_AXIS_IDX, _MOTOR_FAULT_PORT)
  #define V_MOTOR_FAULT_PIN     MOTOR_IO(V_AXIS_IDX, _MOTOR_FAULT_PIN)
  #define V_MOTOR_FAULT_BIT     (1<<V_MOTOR_FAULT_PIN)
#endif

#endif // V_AXIS

#ifdef W_AXIS

#if W_AXIS == 3
#define W_AXIS_IDX 3
#elif W_AXIS == 4
#define W_AXIS_IDX 4
#elif W_AXIS == 5
#define W_AXIS_IDX 5
#elif W_AXIS == 6
#define W_AXIS_IDX 6
#elif W_AXIS == 7
#define W_AXIS_IDX 7
#endif

#if W_AXIS_IDX >= N_MOTORS
  #error "No pins are available for W axis motor"
#endif

#define W_STEP_PORT             MOTOR_IO(W_AXIS_IDX, _STEP_PORT)
#define W_STEP_PIN              MOTOR_IO(W_AXIS_IDX, _STEP_PIN)
#define W_STEP_BIT              (1<<W_STEP_PIN)
#define W_DIRECTION_PORT        MOTOR_IO(W_AXIS_IDX, _DIRECTION_PORT)
#define W_DIRECTION_PIN         MOTOR_IO(W_AXIS_IDX, _DIRECTION_PIN)
#define W_DIRECTION_BIT         (1<<W_DIRECTION_PIN)
#if mn_has_home(W_MOTOR_IDX)
  #define W_HOME_PORT           MOTOR_IO(W_AXIS_IDX, _HOME_PORT)
  #define W_HOME_PIN            MOTOR_IO(W_AXIS_IDX, _HOME_PIN)
  #define W_HOME_BIT            (1<<W_HOME_PIN)
#endif
#if mn_has_limit(W_AXIS_IDX)
  #define W_LIMIT_PORT          MOTOR_IO(W_AXIS_IDX, _LIMIT_PORT)
  #define W_LIMIT_PIN           MOTOR_IO(W_AXIS_IDX, _LIMIT_PIN)
  #define W_LIMIT_BIT           (1<<W_LIMIT_PIN)
#endif
#if mn_has_limit_max(W_AXIS_IDX)
  #define W_LIMIT_MAX_PORT      MOTOR_IO(W_AXIS_IDX, _LIMIT_MAX_PORT)
  #define W_LIMIT_MAX_PIN       MOTOR_IO(W_AXIS_IDX, _LIMIT_MAX_PIN)
  #define W_LIMIT_MAX_BIT       (1<<W_LIMIT_MAX_PIN)
#endif
#if mn_has_enable(W_AXIS_IDX)
  #define W_ENABLE_PORT         MOTOR_IO(W_AXIS_IDX, _ENABLE_PORT)
  #define W_ENABLE_PIN          MOTOR_IO(W_AXIS_IDX, _ENABLE_PIN)
  #define W_ENABLE_BIT          (1<<W_ENABLE_PIN)
#endif
#if mn_has_fault(W_AXIS_IDX)
  #define W_MOTOR_FAULT_PORT    MOTOR_IO(W_AXIS_IDX, _MOTOR_FAULT_PORT)
  #define W_MOTOR_FAULT_PIN     MOTOR_IO(W_AXIS_IDX, _MOTOR_FAULT_PIN)
  #define W_MOTOR_FAULT_BIT     (1<<W_MOTOR_FAULT_PIN)
#endif

#endif // W axis

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
#if defined(U_AXIS) && !defined(U_STEP_PORT)
#define U_STEP_PORT STEP_PORT
#endif
#if defined(V_AXIS) && !defined(V_STEP_PORT)
#define V_STEP_PORT STEP_PORT
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
#if defined(C_AXIS) && !defined(C_DIRECTION_PORT)
#define C_DIRECTION_PORT DIRECTION_PORT
#endif
#if defined(U_AXIS) && !defined(U_DIRECTION_PORT)
#define U_DIRECTION_PORT DIRECTION_PORT
#endif
#if defined(V_AXIS) && !defined(V_DIRECTION_PORT)
#define V_DIRECTION_PORT DIRECTION_PORT
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
#endif

#ifndef X_HOME_BIT
#ifdef X_HOME_PIN
#define X_HOME_BIT (1<<X_HOME_PIN)
#else
#define X_HOME_BIT 0
#endif
#endif
#ifndef Y_HOME_BIT
#ifdef Y_HOME_PIN
#define Y_HOME_BIT (1<<Y_HOME_PIN)
#else
#define Y_HOME_BIT 0
#endif
#endif
#ifndef Z_HOME_BIT
#ifdef Z_HOME_PIN
#define Z_HOME_BIT (1<<Z_HOME_PIN)
#else
#define Z_HOME_BIT 0
#endif
#endif

#ifndef X_MOTOR_FAULT_BIT
#ifdef X_MOTOR_FAULT_PIN
#define X_MOTOR_FAULT_BIT (1<<X_MOTOR_FAULT_PIN)
#else
#define X_MOTOR_FAULT_BIT 0
#endif
#endif
#ifndef Y_MOTOR_FAULT_BIT
#ifdef Y_MOTOR_FAULT_PIN
#define Y_MOTOR_FAULT_BIT (1<<Y_MOTOR_FAULT_PIN)
#else
#define Y_MOTOR_FAULT_BIT 0
#endif
#endif
#ifndef Z_MOTOR_FAULT_BIT
#ifdef Z_MOTOR_FAULT_PIN
#define Z_MOTOR_FAULT_BIT (1<<Z_MOTOR_FAULT_PIN)
#else
#define Z_MOTOR_FAULT_BIT 0
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
#ifndef X_LIMIT_MAX_BIT
#ifdef X_LIMIT_PIN_MAX
#define X_LIMIT_MAX_BIT (1<<X_LIMIT_PIN_MAX)
#else
#define X_LIMIT_MAX_BIT 0
#endif
#endif
#ifndef Y_LIMIT_MAX_BIT
#ifdef Y_LIMIT_PIN_MAX
#define Y_LIMIT_MAX_BIT (1<<Y_LIMIT_PIN_MAX)
#else
#define Y_LIMIT_MAX_BIT 0
#endif
#endif
#ifndef Z_LIMIT_MAX_BIT
#ifdef Z_LIMIT_PIN_MAX
#define Z_LIMIT_MAX_BIT (1<<Z_LIMIT_PIN_MAX)
#else
#define Z_LIMIT_MAX_BIT 0
#endif
#endif

#define LIMIT_MAX_MASK_BASE (X_LIMIT_MAX_BIT|Y_LIMIT_MAX_BIT|Z_LIMIT_MAX_BIT)
#define LIMIT_MAX_MASK_BASE_SUM (X_LIMIT_MAX_BIT+Y_LIMIT_MAX_BIT+Z_LIMIT_MAX_BIT)

#if N_AXIS == 3
#define LIMIT_MAX_MASK LIMIT_MAX_MASK_BASE
#define LIMIT_MAX_MASK_SUM LIMIT_MAX_MASK_BASE_SUM
#elif N_AXIS == 4
#define LIMIT_MAX_MASK (LIMIT_MAX_MASK_BASE|M3_LIMIT_MAX_BIT)
#define LIMIT_MAX_MASK_SUM (LIMIT_MAX_MASK_BASE_SUM+M3_LIMIT_MAX_BIT)
#elif N_AXIS == 5
#define LIMIT_MAX_MASK (LIMIT_MAX_MASK_BASE|M3_LIMIT_MAX_BIT|M4_LIMIT_MAX_BIT)
#define LIMIT_MAX_MASK_SUM (LIMIT_MAX_MASK_BASE_SUM+M3_LIMIT_MAX_BIT+M4_LIMIT_MAX_BIT)
#elif N_AXIS == 6
#define LIMIT_MAX_MASK (LIMIT_MAX_MASK_BASE|M3_LIMIT_MAX_BIT|M4_LIMIT_MAX_BIT|M5_LIMIT_MAX_BIT)
#define LIMIT_MAX_MASK_SUM (LIMIT_MAX_MASK_BASE_SUM+M3_LIMIT_MAX_BIT+M4_LIMIT_MAX_BIT+M5_LIMIT_MAX_BIT)
#elif N_AXIS == 7
#define LIMIT_MAX_MASK (LIMIT_MAX_MASK_BASE|M3_LIMIT_MAX_BIT|M4_LIMIT_MAX_BIT|M5_LIMIT_MAX_BIT|M6_LIMIT_MAX_BIT)
#define LIMIT_MAX_MASK_SUM (LIMIT_MAX_MASK_BASE_SUM+M3_LIMIT_MAX_BIT+M4_LIMIT_MAX_BIT+M5_LIMIT_MAX_BIT+M6_LIMIT_MAX_BIT)
#else
#define LIMIT_MAX_MASK (LIMIT_MAX_MASK_BASE|M3_LIMIT_MAX_BIT|M4_LIMIT_MAX_BIT|M5_LIMIT_MAX_BIT|M6_LIMIT_MAX_BIT|M7_LIMIT_MAX_BIT)
#define LIMIT_MAX_MASK_SUM (LIMIT_MAX_MASK_BASE_SUM+M3_LIMIT_MAX_BIT+M4_LIMIT_MAX_BIT+M5_LIMIT_MAX_BIT+M6_LIMIT_MAX_BIT+M7_LIMIT_MAX_BIT)
#endif

#if !defined(X_ENABLE_BIT) && defined(X_ENABLE_PIN)
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
#define STEP_MASK (X_STEP_BIT|Y_STEP_BIT|Z_STEP_BIT|M3_STEP_BIT)
#elif N_AXIS == 5
#define STEP_MASK (X_STEP_BIT|Y_STEP_BIT|Z_STEP_BIT|M3_STEP_BIT|M4_STEP_BIT)
#elif N_AXIS == 6
#define STEP_MASK (X_STEP_BIT|Y_STEP_BIT|Z_STEP_BIT|M3_STEP_BIT|M4_STEP_BIT|M5_STEP_BIT)
#elif N_AXIS == 7
#define STEP_MASK (X_STEP_BIT|Y_STEP_BIT|Z_STEP_BIT|M3_STEP_BIT|M4_STEP_BIT|M5_STEP_BIT|M6_STEP_BIT)
#else
#define STEP_MASK (X_STEP_BIT|Y_STEP_BIT|Z_STEP_BIT|M3_STEP_BIT|M4_STEP_BIT|M5_STEP_BIT|M6_STEP_BIT|M7_STEP_BIT)
#endif
#endif

#ifndef DIRECTION_MASK
#if N_AXIS == 3
#define DIRECTION_MASK (X_DIRECTION_BIT|Y_DIRECTION_BIT|Z_DIRECTION_BIT)
#elif N_AXIS == 4
#define DIRECTION_MASK (X_DIRECTION_BIT|Y_DIRECTION_BIT|Z_DIRECTION_BIT|M3_DIRECTION_BIT)
#elif N_AXIS == 5
#define DIRECTION_MASK (X_DIRECTION_BIT|Y_DIRECTION_BIT|Z_DIRECTION_BIT|M3_DIRECTION_BIT|M4_DIRECTION_BIT)
#elif N_AXIS == 6
#define DIRECTION_MASK (X_DIRECTION_BIT|Y_DIRECTION_BIT|Z_DIRECTION_BIT|M3_DIRECTION_BIT|M4_DIRECTION_BIT|M5_DIRECTION_BIT)
#elif N_AXIS == 7
#define DIRECTION_MASK (X_DIRECTION_BIT|Y_DIRECTION_BIT|Z_DIRECTION_BIT|M3_DIRECTION_BIT|M4_DIRECTION_BIT|M5_DIRECTION_BIT|M6_DIRECTION_BIT)
#else
#define DIRECTION_MASK (X_DIRECTION_BIT|Y_DIRECTION_BIT|Z_DIRECTION_BIT|M3_DIRECTION_BIT|M4_DIRECTION_BIT|M5_DIRECTION_BIT|M6_DIRECTION_BIT|M7_DIRECTION_BIT)
#endif
#endif

#if defined(STEPPERS_ENABLE_PIN) && !defined(STEPPERS_ENABLE_BIT)
#define STEPPERS_ENABLE_BIT (1<<STEPPERS_ENABLE_PIN)
#endif

#ifndef STEPPERS_ENABLE_MASK

#ifdef STEPPERS_ENABLE_BIT
#define STEPPERS_ENABLE_MASK STEPPERS_ENABLE_BIT
#else

#if N_AXIS == 3
#define STEPPERS_ENABLE_MASK (X_ENABLE_BIT|Y_ENABLE_BIT|Z_ENABLE_BIT)
#elif N_AXIS == 4
#define STEPPERS_ENABLE_MASK (X_ENABLE_BIT|Y_ENABLE_BIT|Z_ENABLE_BIT|M3_ENABLE_BIT)
#elif N_AXIS == 5
#define STEPPERS_ENABLE_MASK (X_ENABLE_BIT|Y_ENABLE_BIT|Z_ENABLE_BIT|M3_ENABLE_BIT|M4_ENABLE_BIT)
#elif N_AXIS == 6
#define STEPPERS_ENABLE_MASK (X_ENABLE_BIT|Y_ENABLE_BIT|Z_ENABLE_BIT|M3_ENABLE_BIT|M4_ENABLE_BIT|M5_ENABLE_BIT)
#elif N_AXIS == 7
#define STEPPERS_ENABLE_MASK (X_ENABLE_BIT|Y_ENABLE_BIT|Z_ENABLE_BIT|M3_ENABLE_BIT|M4_ENABLE_BIT|M5_ENABLE_BIT|M6_ENABLE_BIT)
#else
#define STEPPERS_ENABLE_MASK (X_ENABLE_BIT|Y_ENABLE_BIT|Z_ENABLE_BIT|M3_ENABLE_BIT|M4_ENABLE_BIT|M5_ENABLE_BIT|M6_ENABLE_BIT|M7_ENABLE_BIT)
#endif
#endif

#endif // STEPPERS_ENABLE_MASK

#ifndef HOME_MASK

#define HOME_MASK_BASE (X_HOME_BIT|Y_HOME_BIT|Z_HOME_BIT|HOME2_MASK)
#define HOME_MASK_BASE_SUM (X_HOME_BIT+Y_HOME_BIT+Z_HOME_BIT+HOME2_MASK_SUM)

#if N_AXIS == 3
#define HOME_MASK HOME_MASK_BASE
#define HOME_MASK_SUM HOME_MASK_BASE_SUM
#define HOME_MIN_CAP AXES_BITMASK
#elif N_AXIS == 4
#define HOME_MASK (HOME_MASK_BASE|M3_HOME_BIT)
#define HOME_MASK_SUM (HOME_MASK_BASE_SUM+M3_HOME_BIT)
#elif N_AXIS == 5
#define HOME_MASK (HOME_MASK_BASE|M3_HOME_BIT|M4_HOME_BIT)
#define HOME_MASK_SUM (HOME_MASK_BASE_SUM+M3_HOME_BIT+M4_HOME_BIT)
#elif N_AXIS == 6
#define HOME_MASK (HOME_MASK_BASE|M3_HOME_BIT|M4_HOME_BIT|M5_HOME_BIT)
#define HOME_MASK_SUM (HOME_MASK_BASE_SUM+M3_HOME_BIT+M4_HOME_BIT+M5_HOME_BIT)
#elif N_AXIS == 7
#define HOME_MASK (HOME_MASK_BASE|M3_HOME_BIT|M4_HOME_BIT|M5_HOME_BIT|M6_HOME_BIT)
#define HOME_MASK_SUM (HOME_MASK_BASE_SUM+M3_HOME_BIT+M4_HOME_BIT+M5_HOME_BIT+M6_HOME_BIT)
#else
#define HOME_MASK (HOME_MASK_BASE|M3_HOME_BIT|M4_HOME_BIT|M5_HOME_BIT|M6_HOME_BIT|M7_HOME_BIT)
#define HOME_MASK_SUM (HOME_MASK_BASE_SUM+M3_HOME_BIT+M4_HOME_BIT+M5_HOME_BIT+M6_HOME_BIT+M7_HOME_BIT)
#endif

#endif // HOME_MASK

#ifndef LIMIT_MASK

#ifdef Z_LIMIT_POLL
#define LIMIT_MASK_BASE (X_LIMIT_BIT|Y_LIMIT_BIT|LIMIT2_MASK|LIMIT_MAX_MASK)
#define LIMIT_MASK_BASE_SUM (X_LIMIT_BIT+Y_LIMIT_BIT+LIMIT2_MASK_SUM+LIMIT_MAX_SUM)
#else
#define LIMIT_MASK_BASE (X_LIMIT_BIT|Y_LIMIT_BIT|Z_LIMIT_BIT|LIMIT2_MASK|LIMIT_MAX_MASK)
#define LIMIT_MASK_BASE_SUM (X_LIMIT_BIT+Y_LIMIT_BIT+Z_LIMIT_BIT+LIMIT2_MASK_SUM+LIMIT_MAX_SUM)
#endif

#if N_AXIS == 3
#define LIMIT_MASK LIMIT_MASK_BASE
#define LIMIT_MASK_SUM LIMIT_MASK_BASE_SUM
#define LIMIT_MIN_CAP AXES_BITMASK
#elif N_AXIS == 4
#define LIMIT_MASK (LIMIT_MASK_BASE|M3_LIMIT_BIT)
#define LIMIT_MASK_SUM (LIMIT_MASK_BASE_SUM+M3_LIMIT_BIT)
#elif N_AXIS == 5
#define LIMIT_MASK (LIMIT_MASK_BASE|M3_LIMIT_BIT|M4_LIMIT_BIT)
#define LIMIT_MASK_SUM (LIMIT_MASK_BASE_SUM+M3_LIMIT_BIT+M4_LIMIT_BIT)
#elif N_AXIS == 6
#define LIMIT_MASK (LIMIT_MASK_BASE|M3_LIMIT_BIT|M4_LIMIT_BIT|M5_LIMIT_BIT)
#define LIMIT_MASK_SUM (LIMIT_MASK_BASE_SUM+M3_LIMIT_BIT+M4_LIMIT_BIT+M5_LIMIT_BIT)
#elif N_AXIS == 7
#define LIMIT_MASK (LIMIT_MASK_BASE|M3_LIMIT_BIT|M4_LIMIT_BIT|M5_LIMIT_BIT|M6_LIMIT_BIT)
#define LIMIT_MASK_SUM (LIMIT_MASK_BASE_SUM+M3_LIMIT_BIT+M4_LIMIT_BIT+M5_LIMIT_BIT+M6_LIMIT_BIT)
#else
#define LIMIT_MASK (LIMIT_MASK_BASE|M3_LIMIT_BIT|M4_LIMIT_BIT|M5_LIMIT_BIT|M6_LIMIT_BIT|M7_LIMIT_BIT)
#define LIMIT_MASK_SUM (LIMIT_MASK_BASE_SUM+M3_LIMIT_BIT+M4_LIMIT_BIT+M5_LIMIT_BIT+M6_LIMIT_BIT+M7_LIMIT_BIT)
#endif

#endif // LIMIT_MASK

#ifndef MOTOR_FAULT_MASK

#define MOTOR_FAULT_MASK_BASE (X_MOTOR_FAULT_BIT|Y_MOTOR_FAULT_BIT|Z_MOTOR_FAULT_BIT|MOTOR_FAULT2_MASK)
#define MOTOR_FAULT_MASK_BASE_SUM (X_MOTOR_FAULT_BIT+Y_MOTOR_FAULT_BIT+Z_MOTOR_FAULT_BIT+MOTOR_FAULT2_MASK_SUM)

#if N_AXIS == 3
#define MOTOR_FAULT_MASK MOTOR_FAULT_MASK_BASE
#define MOTOR_FAULT_MASK_SUM MOTOR_FAULT_MASK_BASE_SUM
#define MOTOR_FAULT_MIN_CAP AXES_BITMASK
#elif N_AXIS == 4
#define MOTOR_FAULT_MASK (MOTOR_FAULT_MASK_BASE|A_MOTOR_FAULT_BIT)
#define MOTOR_FAULT_MASK_SUM (MOTOR_FAULT_MASK_BASE_SUM+A_MOTOR_FAULT_BIT)
#elif N_AXIS == 5
#define MOTOR_FAULT_MASK (MOTOR_FAULT_MASK_BASE|M3_MOTOR_FAULT_BIT|M4_MOTOR_FAULT_BIT)
#define MOTOR_FAULT_MASK_SUM (MOTOR_FAULT_MASK_BASE_SUM+M3_MOTOR_FAULT_BIT+M4_MOTOR_FAULT_BIT)
#elif N_AXIS == 6
#define MOTOR_FAULT_MASK (MOTOR_FAULT_MASK_BASE|M3_MOTOR_FAULT_BIT|M4_MOTOR_FAULT_BIT|M5_MOTOR_FAULT_BIT)
#define MOTOR_FAULT_MASK_SUM (MOTOR_FAULT_MASK_BASE_SUM+M3_MOTOR_FAULT_BIT+M4_MOTOR_FAULT_BIT+M5_MOTOR_FAULT_BIT)
#elif N_AXIS == 7
#define MOTOR_FAULT_MASK (MOTOR_FAULT_MASK_BASE|M3_MOTOR_FAULT_BIT|M4_MOTOR_FAULT_BIT|M5_MOTOR_FAULT_BIT|M6_MOTOR_FAULT_BIT)
#define MOTOR_FAULT_MASK_SUM (MOTOR_FAULT_MASK_BASE_SUM+M3_MOTOR_FAULT_BIT+M4_MOTOR_FAULT_BIT+M5_MOTOR_FAULT_BIT+M6_MOTOR_FAULT_BIT)
#else
#define MOTOR_FAULT_MASK (MOTOR_FAULT_MASK_BASE|M3_MOTOR_FAULT_BIT|M4_MOTOR_FAULT_BIT|M5_MOTOR_FAULT_BIT|M6_MOTOR_FAULT_BIT|M7_MOTOR_FAULT_BIT)
#define MOTOR_FAULT_MASK_SUM (MOTOR_FAULT_MASK_BASE_SUM+M3_MOTOR_FAULT_BIT+M4_MOTOR_FAULT_BIT+M5_MOTOR_FAULT_BIT+M6_MOTOR_FAULT_BIT+M7_MOTOR_FAULT_BIT)
#endif

#endif // MOTOR_FAULT_MASK

#ifndef N_GANGED
#define N_GANGED 0
#endif

static void motor_iterator (motor_iterator_callback_ptr callback)
{
    motor_map_t motor;

    if(callback) for(motor.id = 0; motor.id < N_AXIS + N_GANGED; motor.id++)
    {
        if(motor.id < N_AXIS)
            motor.axis = motor.id;
        else switch (motor.id) {
#ifdef X2_MOTOR_IDX
            case X2_MOTOR_IDX:
                motor.axis = X_AXIS;
                break;
#endif
#ifdef Y2_MOTOR_IDX
            case Y2_MOTOR_IDX:
                motor.axis = Y_AXIS;
                break;
#endif
#ifdef Z2_MOTOR_IDX
            case Z2_MOTOR_IDX:
                motor.axis = Z_AXIS;
                break;
#endif
        }
        callback(motor);
    }
}

static inline limit_signals_t get_limits_cap (void)
{
    limit_signals_t limits = {0};

#if X_LIMIT_BIT
    limits.min.x = On;
#endif
#if Y_LIMIT_BIT
    limits.min.y = On;
#endif
#if Z_LIMIT_BIT
    limits.min.z = On;
#endif
#ifdef A_LIMIT_BIT
    limits.min.a = On;
#endif
#ifdef B_LIMIT_BIT
    limits.min.b = On;
#endif
#ifdef C_LIMIT_BIT
    limits.min.c = On;
#endif
#ifdef U_LIMIT_BIT
    limits.min.u = On;
#endif
#ifdef V_LIMIT_BIT
    limits.min.v = On;
#endif

#ifdef X2_LIMIT_BIT
    limits.min2.x = On;
#endif
#ifdef Y2_LIMIT_BIT
    limits.min2.y = On;
#endif
#ifdef Z2_LIMIT_BIT
    limits.min2.z = On;
#endif

#if X_LIMIT_MAX_BIT
    limits.max.x = On;
#endif
#if Y_LIMIT_MAX_BIT
    limits.max.y = On;
#endif
#if Z_LIMIT_MAX_BIT
    limits.max.z = On;
#endif
#ifdef A_LIMIT_MAX_BIT
    limits.max.a = On;
#endif
#ifdef B_LIMIT_MAX_BIT
    limits.max.b = On;
#endif
#ifdef C_LIMIT_MAX_BIT
    limits.max.c = On;
#endif
#ifdef U_LIMIT_MAX_BIT
    limits.max.u = On;
#endif
#ifdef V_LIMIT_MAX_BIT
    limits.max.v = On;
#endif

    return limits;
}

static inline home_signals_t get_home_cap (void)
{
    home_signals_t home = {0};

#if HOME_MASK

#if X_HOME_BIT
    home.a.x = On;
#endif
#if Y_HOME_BIT
    home.a.y = On;
#endif
#if Z_HOME_BIT
    home.a.z = On;
#endif
#ifdef A_HOME_BIT
    home.a.a = On;
#endif
#ifdef B_HOME_BIT
    home.a.b = On;
#endif
#ifdef C_HOME_BIT
    home.a.c = On;
#endif
#if U_HOME_BIT
    home.a.u = On;
#endif
#ifdef V_HOME_BIT
    home.a.v = On;
#endif

#ifdef X2_HOME_BIT
    home.b.x = On;
#endif
#ifdef Y2_HOME_BIT
    home.b.y = On;
#endif
#ifdef Z2_HOME_BIT
    home.b.z = On;
#endif

#endif // HOME_MASK

    return home;
}

static inline home_signals_t get_motor_fault_cap (void)
{
    home_signals_t motor_fault = {0};

#if MOTOR_FAULT_MASK

#if X_MOTOR_FAULT_BIT
    motor_fault.a.x = On;
#endif
#if Y_MOTOR_FAULT_BIT
    motor_fault.a.y = On;
#endif
#if Z_MOTOR_FAULT_BIT
    motor_fault.a.z = On;
#endif
#ifdef A_MOTOR_FAULT_BIT
    motor_fault.a.a = On;
#endif
#ifdef B_MOTOR_FAULT_BIT
    motor_fault.a.b = On;
#endif
#ifdef C_MOTOR_FAULT_BIT
    motor_fault.a.c = On;
#endif
#ifdef U_MOTOR_FAULT_BIT
    motor_fault.a.u = On;
#endif
#ifdef V_MOTOR_FAULT_BIT
    motor_fault.a.v = On;
#endif

#ifdef X2_MOTOR_FAULT_BIT
    motor_fault.b.x = On;
#endif
#ifdef Y2_MOTOR_FAULT_BIT
    motor_fault.b.y = On;
#endif
#ifdef Z2_MOTOR_FAULT_BIT
    motor_fault.b.z = On;
#endif

#endif // MOTOR_FAULT_MASK

    return motor_fault;
}

/*EOF*/
