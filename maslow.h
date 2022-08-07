/*
  maslow.h - Maslow router kinematics implementation

  Part of grblHAL

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

  The basis for this code has been pulled from MaslowDue created by Larry D O'Cull.
  <https://github.com/ldocull/MaslowDue>

  Some portions of that package directly or indirectly has been pulled from from the Maslow CNC
  firmware for Aduino Mega. Those parts are Copyright 2014-2017 Bar Smith.
  <https://www.maslowcnc.com/>

  It has been adapted for grbl by Terje Io.

  *** TO BE COMPLETED ***

*/

#include "grbl.h"

#ifndef _MASLOW_H_
#define _MASLOW_H_

#define FP_SCALING 1024.0f
#define SPROCKET_RADIUS_MM (10.1f)
#define MAX_SEG_LENGTH_MM 2.0f /* long lines must be segmented due to circular motion */

  // PID position loop factors              X: Kp = 25000 Ki = 15000 Kd = 22000 Imax = 5000
  // 14.000 fixed point arithmetic S13.10
#ifdef DRIVER_TLE5206
    #define MASLOW_A_KP     10.0f
    #define MASLOW_A_KI     21.0f
    #define MASLOW_A_KD     18.0f
    #define MASLOW_A_IMAX   5000

    #define MASLOW_B_KP     10.0f
    #define MASLOW_B_KI     21.0f
    #define MASLOW_B_KD     18.0f
    #define MASLOW_B_IMAX   5000

    #define MASLOW_Z_KP     10.0f
    #define MASLOW_Z_KI     21.0f
    #define MASLOW_Z_KD     17.0f
    #define MASLOW_Z_IMAX   5000
#else
    #define MASLOW_A_KP     22.0f
    #define MASLOW_A_KI     17.0f
    #define MASLOW_A_KD     20.0f
    #define MASLOW_A_IMAX   5000

    #define MASLOW_B_KP     22.0f
    #define MASLOW_B_KI     17.0f
    #define MASLOW_B_KD     20.0f
    #define MASLOW_B_IMAX   5000

    #define MASLOW_Z_KP     20.0f
    #define MASLOW_Z_KI     17.0f
    #define MASLOW_Z_KD     18.0f
    #define MASLOW_Z_IMAX   5000
#endif

#define MASLOW_MACHINEWIDTH        2400.0f
#define MASLOW_MACHINEHEIGHT       1200.0f
#define MASLOW_DISTBETWEENMOTORS   3000.0f
#define MASLOW_MOTOROFFSETY        600.0f
#define MASLOW_CHAINLENGTH         3000.0f
#define MASLOW_CHAINOVERSPROCKET   0
#define MASLOW_CHAINSAGCORRECTION  59.504839f
#define MASLOW_LEFTCHAINTOLERANCE  0.0f
#define MASLOW_RIGHTCHAINTOLERANCE 0.0f
#define MASLOW_ROTATIONDISKRADIUS  104.3f
#define MASLOW_SLEDHEIGHT          139.0f
#define MASLOW_SLEDWIDTH           310.0f
#define MASLOW_ACORRSCALING        1.003922f
#define MASLOW_BCORRSCALING        1.002611f

typedef enum {
    Maslow_ChainOverSprocket = 260,
    Maslow_MachineWidth,
    Maslow_MachineHeight,
    Maslow_DistBetweenMotors,
    Maslow_MotorOffsetY,
    Maslow_AcorrScaling,
    Maslow_BcorrScaling,
    Maslow_SettingMax,
} maslow_setting_t;

typedef enum {
    AxisSetting_MaslowKP = 10,
    AxisSetting_MaslowKI,
    AxisSetting_MaslowKD,
    AxisSetting_MaslowIMax,
    AxisSetting_MaslowMaxSetting
} maslow_axis_setting_t;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float Imax;
} maslow_pid_coefficients_t;

typedef struct {
    maslow_pid_coefficients_t pid[N_AXIS];

    uint32_t chainOverSprocket;
    float machineWidth;   /* Maslow specific settings */
    float machineHeight;
    float distBetweenMotors;
    float motorOffsetY;
    float chainSagCorrection;
    float leftChainTolerance;
    float rightChainTolerance;
    float rotationDiskRadius;
    float chainLength;
    float sledHeight;
    float sledWidth;

    float XcorrScaling;
    float YcorrScaling;
} maslow_settings_t;

typedef struct {
    float Error;
    float Integral;
    float iterm;
    float DiffTerm;
    float speed;
} maslow_debug_t;

typedef struct {
    maslow_settings_t settings;
    void (*pid_settings_changed)(uint_fast8_t idx);
    void (*move)(uint_fast8_t idx, int_fast16_t distance);
    void (*reset_pid)(uint_fast8_t idx);
    void (*pos_enable)(bool enable);
    void (*tuning_enable)(bool enable);
    int32_t (*set_step_size)(uint_fast8_t idx, int32_t step_size);
    maslow_debug_t *(*get_debug_data)(uint_fast8_t idx);
} maslow_hal_t;

extern maslow_hal_t maslow_hal;

// Initialize HAL pointers for Maslow Router kinematics
bool maslow_init (void);
static status_code_t maslow_tuning (uint_fast16_t state, char *line);

#endif
