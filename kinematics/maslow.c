/*
  maslow.c - Maslow router kinematics implementation

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

  It has been adapted for grblHAL by Terje Io.
*/

#include "../grbl.h"

#if MASLOW_ROUTER

#include <math.h>

#include "driver.h"

#include "../settings.h"
#include "../planner.h"
#include "../nvs_buffer.h"
#include "../kinematics.h"
#include "../maslow.h"
#include "../report.h"

#define A_MOTOR X_AXIS // Must be X_AXIS
#define B_MOTOR Y_AXIS // Must be Y_AXIS

typedef struct {
    float halfWidth;        //Half the machine width
    float halfHeight;       //Half the machine height
    float xCordOfMotor;
    float xCordOfMotor_x4;
    float xCordOfMotor_x2_pow;
    float yCordOfMotor;
    float height_to_bit; //distance between sled attach point and bit
} machine_t;

static machine_t machine = {0};

uint_fast8_t selected_motor = A_MOTOR;
maslow_settings_t maslow;
maslow_hal_t maslow_hal = {0};
static nvs_address_t nvs_address;

static const maslow_settings_t maslow_defaults = {
    .pid[A_MOTOR].Kp = MASLOW_A_KP,
    .pid[A_MOTOR].Ki = MASLOW_A_KI,
    .pid[A_MOTOR].Kd = MASLOW_A_KD,
    .pid[A_MOTOR].Imax = MASLOW_A_IMAX,

    .pid[B_MOTOR].Kp = MASLOW_B_KP,
    .pid[B_MOTOR].Ki = MASLOW_B_KI,
    .pid[B_MOTOR].Kd = MASLOW_B_KD,
    .pid[B_MOTOR].Imax = MASLOW_B_IMAX,

    .pid[Z_AXIS].Kp = MASLOW_Z_KP,
    .pid[Z_AXIS].Ki = MASLOW_Z_KI,
    .pid[Z_AXIS].Kd = MASLOW_Z_KD,
    .pid[Z_AXIS].Imax = MASLOW_Z_IMAX,

    .chainOverSprocket = MASLOW_CHAINOVERSPROCKET,
    .machineWidth = MASLOW_MACHINEWIDTH,
    .machineHeight = MASLOW_MACHINEHEIGHT,
    .distBetweenMotors = MASLOW_DISTBETWEENMOTORS,

    .motorOffsetY = MASLOW_MOTOROFFSETY,
    .chainSagCorrection = MASLOW_CHAINSAGCORRECTION,
    .leftChainTolerance = MASLOW_LEFTCHAINTOLERANCE,
    .rightChainTolerance = MASLOW_RIGHTCHAINTOLERANCE,
    .rotationDiskRadius = MASLOW_ROTATIONDISKRADIUS,

    .chainLength = MASLOW_CHAINLENGTH,
    .sledHeight = MASLOW_SLEDHEIGHT,
    .sledWidth = MASLOW_SLEDWIDTH,

    .XcorrScaling = MASLOW_ACORRSCALING,
    .YcorrScaling = MASLOW_BCORRSCALING
};

static status_code_t set_axis_setting (setting_id_t setting, float value);
static float get_axis_setting (setting_id_t setting);
static void maslow_settings_load (void);
static void maslow_settings_restore (void);

#define AXIS_OPTS { .subgroups = On, .iterations = 1 }

static const setting_detail_t maslow_settings[] = {
#if maslow_MIXED_DRIVERS
    { Setting_maslowDriver, Group_MotorDriver, "maslow driver", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_NonCore, &maslow.driver_enable.mask },
#endif
    { (setting_id_t)Maslow_ChainOverSprocket, Group_MotorDriver, "Chain over sprocket", NULL, Format_Integer, NULL, NULL, NULL, Setting_NonCore, &maslow.chainOverSprocket, NULL },
    { (setting_id_t)Maslow_MachineWidth, Group_MotorDriver, "Machine width", "mm", Format_Decimal, "###0.0", NULL, NULL, Setting_NonCore, &maslow.machineWidth, NULL },
    { (setting_id_t)Maslow_MachineHeight, Group_MotorDriver, "Machine height", "mm", Format_Decimal, "###0.0", NULL, NULL, Setting_NonCore, &maslow.machineHeight, NULL },
    { (setting_id_t)Maslow_DistBetweenMotors, Group_MotorDriver, "Distance between motors", "mm", Format_Decimal, NULL, NULL, NULL, Setting_NonCore, &maslow.distBetweenMotors, NULL },
    { (setting_id_t)Maslow_MotorOffsetY, Group_MotorDriver, "Motor offset Y", "mm", Format_Decimal, "###0.0", NULL, NULL, Setting_NonCore, &maslow.motorOffsetY, NULL },
    { (setting_id_t)Maslow_AcorrScaling, Group_MotorDriver, "Acorr Scaling", NULL, Format_Decimal, "###0.0", NULL, NULL, Setting_NonCore, &maslow.XcorrScaling, NULL },
    { (setting_id_t)Maslow_BcorrScaling, Group_MotorDriver, "BcorrScaling", NULL, Format_Decimal, "###0.0", NULL, NULL, Setting_NonCore, &maslow.XcorrScaling, NULL },
    { (setting_id_t)AxisSetting_MaslowKP, Group_Axis0, "-axis KP", NULL, Format_Decimal, "###0.0", NULL, NULL, Setting_NonCoreFn, set_axis_setting, get_axis_setting, AXIS_OPTS },
    { (setting_id_t)AxisSetting_MaslowKI, Group_Axis0, "-axis KI", NULL, Format_Decimal, "###0.0", NULL, NULL, Setting_NonCoreFn, set_axis_setting, get_axis_setting, AXIS_OPTS },
    { (setting_id_t)AxisSetting_MaslowKD, Group_Axis0, "-axis KIt", NULL, Format_Decimal, "###0.0", NULL, NULL, Setting_NonCoreFn, set_axis_setting, get_axis_setting, AXIS_OPTS },
    { (setting_id_t)AxisSetting_MaslowIMax, Group_Axis0, "-axis I Max", "ma", Format_Decimal, "###0.0", NULL, NULL, Setting_NonCoreFn, set_axis_setting, get_axis_setting, AXIS_OPTS }
};

static void maslow_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&maslow, sizeof(maslow_settings_t), true);
}

static setting_details_t details = {
    .settings = maslow_settings,
    .n_settings = sizeof(maslow_settings) / sizeof(setting_detail_t),
    .load = maslow_settings_load,
    .save = maslow_settings_save,
    .restore = maslow_settings_restore
};

static setting_details_t *on_get_settings (void)
{
    return &details;
}

static status_code_t set_axis_setting (setting_id_t setting, float value)
{
    status_code_t status = Status_OK;

    if((setting_id_t)setting >= Setting_AxisSettingsBase && (setting_id_t)setting <= Setting_AxisSettingsMax) {

        uint_fast16_t base_idx = (uint_fast16_t)setting - (uint_fast16_t)Setting_AxisSettingsBase;
        uint_fast8_t axis_idx = base_idx % AXIS_SETTINGS_INCREMENT;

        if(axis_idx < N_AXIS) switch((base_idx - axis_idx) / AXIS_SETTINGS_INCREMENT) {

            case AxisSetting_MaslowKP:
                status = Status_OK;
                maslow.pid[axis_idx].Kp = value;
                break;

            case AxisSetting_MaslowKI:
                status = Status_OK;
                maslow.pid[axis_idx].Ki = value;
                break;

            case AxisSetting_MaslowKD:
                status = Status_OK;
                maslow.pid[axis_idx].Kd = value;
                break;

            case AxisSetting_MaslowIMax:
                status = Status_OK;
                maslow.pid[axis_idx].Imax = value;

            default:
                status = Status_Unhandled;
                break;
        }
    }

    return status;
}

static float get_axis_setting (setting_id_t setting)
{
    float value = 0;

    if (setting >= Setting_AxisSettingsBase && setting <= Setting_AxisSettingsMax) {

        uint_fast16_t base_idx = (uint_fast16_t)setting - (uint_fast16_t)Setting_AxisSettingsBase;
        uint_fast8_t axis_idx = base_idx % AXIS_SETTINGS_INCREMENT;

        if(axis_idx < N_AXIS) switch((base_idx - axis_idx) / AXIS_SETTINGS_INCREMENT) {

            case AxisSetting_MaslowKP:
                value = maslow.pid[axis_idx].Kp;
                break;

            case AxisSetting_MaslowKI:
                value = maslow.pid[axis_idx].Ki;
                break;

            case AxisSetting_MaslowKD:
                value = maslow.pid[axis_idx].Kd;
                break;

            case AxisSetting_MaslowIMax:
                value = maslow.pid[axis_idx].Imax;
                break;
        }
    }

    return value;
}

static void maslow_settings_restore (void)
{
    memcpy(&maslow, &maslow_defaults, sizeof(maslow_settings_t));

    hal.nvs.memcpy_to_nvs(hal.nvs.driver_area.address, (uint8_t *)&maslow, sizeof(maslow_settings_t), true);
}

static void maslow_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&maslow, nvs_address, sizeof(maslow_settings_t), true) != NVS_TransferResult_OK)
        maslow_settings_restore();
}

/** End settings handling **/

void recomputeGeometry()
{
    /*
    Some variables are computed on initialization for the geometry of the machine to reduce overhead,
    calling this function regenerates those values.
    */
    machine.halfWidth = (maslow.machineWidth / 2.0f);
    machine.halfHeight = (maslow.machineHeight / 2.0f);
    machine.xCordOfMotor = (maslow.distBetweenMotors / 2.0f);
    machine.yCordOfMotor = (machine.halfHeight + maslow.motorOffsetY);
    machine.xCordOfMotor_x4 = machine.xCordOfMotor * 4.0f;
    machine.xCordOfMotor_x2_pow = powf((machine.xCordOfMotor * 2.0f), 2.0f);
}

// limit motion to stay within table (in mm)
void verifyValidTarget (float* xTarget, float* yTarget)
{
    //If the target point is beyond one of the edges of the board, the machine stops at the edge

    recomputeGeometry();
// no limits for now
//      *xTarget = (*xTarget < -halfWidth) ? -halfWidth : (*xTarget > halfWidth) ? halfWidth : *xTarget;
//      *yTarget = (*yTarget < -halfHeight) ? -halfHeight : (*yTarget > halfHeight) ? halfHeight : *yTarget;

}

// Maslow CNC calculation only. Returns x or y-axis "steps" based on Maslow motor steps.
// converts current position two-chain intersection (steps) into x / y cartesian in STEPS..
static void maslow_convert_array_steps_to_mpos (float *position, int32_t *steps)
{
    float a_len = ((float)steps[A_MOTOR] / settings.axis[A_MOTOR].steps_per_mm);
    float b_len = ((float)steps[B_MOTOR] / settings.axis[B_MOTOR].steps_per_mm);

    a_len = (machine.xCordOfMotor_x2_pow - powf(b_len, 2.0f) + powf(a_len, 2.0f)) / machine.xCordOfMotor_x4;
    position[X_AXIS] = a_len - machine.xCordOfMotor;
    a_len = maslow.distBetweenMotors - a_len;
    position[Y_AXIS] = machine.yCordOfMotor - sqrtf(powf(b_len, 2.0f) - powf(a_len, 2.0f));
    position[Z_AXIS] = steps[Z_AXIS] / settings.axis[Z_AXIS].steps_per_mm;

// back out any correction factor
   position[X_AXIS] /= maslow.XcorrScaling;
   position[Y_AXIS] /= maslow.YcorrScaling;
//
}

// calculate left and right (A_MOTOR/B_MOTOR) chain lengths from X-Y cartesian coordinates  (in mm)
// target is an absolute position in the frame
inline static void triangularInverse (int32_t *target_steps, float *target)
{
    //Confirm that the coordinates are on the table
//    verifyValidTarget(&xTarget, &yTarget);

    // scale target (absolute position) by any correction factor
    double xxx = (double)target[A_MOTOR] * (double)maslow.XcorrScaling;
    double yyy = (double)target[B_MOTOR] * (double)maslow.YcorrScaling;
    double yyp = pow((double)machine.yCordOfMotor - yyy, 2.0);

    //Calculate motor axes length to the bit
    target_steps[A_MOTOR] = (int32_t)lround(sqrt(pow((double)machine.xCordOfMotor + xxx, 2.0f) + yyp) * settings.axis[A_MOTOR].steps_per_mm);
    target_steps[B_MOTOR] = (int32_t)lround(sqrt(pow((double)machine.xCordOfMotor - xxx, 2.0f) + yyp) * settings.axis[B_MOTOR].steps_per_mm);
}

// Transform absolute position from cartesian coordinate system (mm) to maslow coordinate system (step)
static void maslow_target_to_steps (int32_t *target_steps, float *target)
{
    uint_fast8_t idx = N_AXIS - 1;

    do {
        target_steps[idx] = lroundf(target[idx] * settings.axis[idx].steps_per_mm);
    } while(--idx > Y_AXIS);

    triangularInverse(target_steps, target);
}

static uint_fast8_t maslow_limits_get_axis_mask (uint_fast8_t idx)
{
    return ((idx == A_MOTOR) || (idx == B_MOTOR)) ? (bit(X_AXIS) | bit(Y_AXIS)) : bit(idx);
}

// MASLOW is circular in motion, so long lines must be divided up
static bool maslow_segment_line (float *target, plan_line_data_t *pl_data, bool init)
{
    static uint_fast16_t iterations;
    static bool segmented;
    static float delta[N_AXIS], segment_target[N_AXIS];
//    static plan_line_data_t plan;

    uint_fast8_t idx = N_AXIS;

    if(init) {

        float max_delta = 0.0f;

        do {
            idx--;
            delta[idx] = target[idx] - gc_state.position[idx];
            max_delta = max(max_delta, fabsf(delta[idx]));
        } while(idx);

        if((segmented = !(pl_data->condition.rapid_motion || pl_data->condition.jog_motion) &&
                          max_delta > MAX_SEG_LENGTH_MM && !(delta[X_AXIS] == 0.0f && delta[Y_AXIS] == 0.0f))) {

            idx = N_AXIS;
            iterations = (uint_fast16_t)ceilf(max_delta / MAX_SEG_LENGTH_MM);

            memcpy(segment_target, gc_state.position, sizeof(segment_target));
//            memcpy(&plan, pl_data, sizeof(plan_line_data_t));

            do {
                delta[--idx] /= (float)iterations;
                target[idx] = gc_state.position[idx];
            } while(idx);

        } else
            iterations = 1;

        iterations++; // return at least one iteration

    } else {

        iterations--;

        if(segmented && iterations) do {
            idx--;
            segment_target[idx] += delta[idx];
            target[idx] = segment_target[idx];
//            memcpy(pl_data, &plan, sizeof(plan_line_data_t));
        } while(idx);

    }

    return iterations != 0;
}

static void maslow_limits_set_target_pos (uint_fast8_t idx) // fn name?
{
    /*
    int32_t axis_position;
    float position[3];
    maslow_convert_array_steps_to_mpos(position, sys.position);

    float aCl,bCl;    // set initial chain lengths to table center when $HOME
    void triangularInverse(float ,float , float* , float* );

    x_axis.axis_Position = 0;
    x_axis.target = 0;
    x_axis.target_PS = 0;
    x_axis.Integral = 0;
    y_axis.axis_Position = 0;
    y_axis.target = 0;
    y_axis.target_PS = 0;
    y_axis.Integral = 0;
    z_axis.axis_Position = 0;
    z_axis.target = 0;
    z_axis.target_PS = 0;
    z_axis.Integral = 0;
    set_axis_position = 0;    // force to center of table -- its a Maslow thing

    triangularInverse((float)(set_axis_position), (float)(set_axis_position), &aCl, &bCl);
    sys.position[A_MOTOR] = (int32_t) lround(aCl * settings.steps_per_mm[A_MOTOR]);
    sys.position[B_MOTOR] = (int32_t) lround(bCl * settings.steps_per_mm[B_MOTOR]);
    sys.position[Z_AXIS] = set_axis_position;

    store_current_machine_pos();    // reset all the way out to stored space
    sys.step_control = STEP_CONTROL_NORMAL_OP; // Return step control to normal operation.
    return;

  sys.position[idx] = set_axis_position;

    switch(idx) {
        case X_AXIS:
            axis_position = system_convert_maslow_to_y_axis_steps(sys.position);
            sys.position[A_MOTOR] = axis_position;
            sys.position[B_MOTOR] = -axis_position;
            break;
        case Y_AXIS:
            sys.position[A_MOTOR] = sys.position[B_MOTOR] = system_convert_maslow_to_x_axis_steps(sys.position);
            break;
        default:
            sys.position[idx] = 0;
            break;
    }
    */
}

// Set machine positions for homed limit switches. Don't update non-homed axes.
// NOTE: settings.max_travel[] is stored as a negative value.
static void maslow_limits_set_machine_positions (axes_signals_t cycle)
{
    /*
     *     uint_fast8_t idx = N_AXIS;

    if(settings.homing.flags.force_set_origin) {
        if (cycle.mask & bit(--idx)) do {
            switch(--idx) {
                case X_AXIS:
                    sys.position[A_MOTOR] = system_convert_maslow_to_y_axis_steps(sys.position);
                    sys.position[B_MOTOR] = - sys.position[A_MOTOR];
                    break;
                case Y_AXIS:
                    sys.position[A_MOTOR] = system_convert_maslow_to_x_axis_steps(sys.position);
                    sys.position[B_MOTOR] = sys.position[A_MOTOR];
                    break;
                default:
                    sys.position[idx] = 0;
                    break;
            }
        } while (idx);
    } else do {
         if (cycle.mask & bit(--idx)) {
             int32_t off_axis_position;
             int32_t set_axis_position = bit_istrue(settings.homing.dir_mask.value, bit(idx))
                                          ? lroundf((settings.max_travel[idx] + settings.homing.pulloff) * settings.steps_per_mm[idx])
                                          : lroundf(-settings.homing.pulloff * settings.steps_per_mm[idx]);
             switch(idx) {
                 case X_AXIS:
                     off_axis_position = system_convert_maslow_to_y_axis_steps(sys.position);
                     sys.position[A_MOTOR] = set_axis_position + off_axis_position;
                     sys.position[B_MOTOR] = set_axis_position - off_axis_position;
                     break;
                 case Y_AXIS:
                     off_axis_position = system_convert_maslow_to_x_axis_steps(sys.position);
                     sys.position[A_MOTOR] = off_axis_position + set_axis_position;
                     sys.position[B_MOTOR] = off_axis_position - set_axis_position;
                     break;
                 default:
                     sys.position[idx] = set_axis_position;
                     break;
             }
         }
    } while(idx);
    */
}

// TODO: format output in grbl fashion: [...]
status_code_t maslow_tuning (sys_state_t state, char *line)
{
    status_code_t retval = Status_OK;

    if(line[1] == 'M') switch(line[2]) {

        case 'C': // commit driver setting changes to non-volatile storage
            settings_dirty.is_dirty = settings_dirty.driver_settings = true;
            break;

        case 'X':
            selected_motor = A_MOTOR;
            hal.stream.write("X-Axis Selected" ASCII_EOL);
            break;

        case 'Y':
            selected_motor = B_MOTOR;
            hal.stream.write("Y-Axis Selected" ASCII_EOL);
            break;

        case 'Z':
            selected_motor = Z_AXIS;
            if(maslow_hal.get_debug_data(selected_motor))
                hal.stream.write("Z-Axis Selected" ASCII_EOL);
            else {
                selected_motor = A_MOTOR;
                hal.stream.write("Z-Axis is not PID controlled, switched to A motor" ASCII_EOL);
            }
            break;

        case 'G':
            maslow_hal.pos_enable(true);
            break;

        case 'R':   // reset current position
            maslow_hal.reset_pid(selected_motor);
            break;

        case '+':   // Move
            maslow_hal.move(selected_motor, 10000);
            break;

        case '-':   // Move
            maslow_hal.move(selected_motor, -10000);
            break;

        case '*':   // Move
            maslow_hal.move(selected_motor, 10);
            break;

        case '/':   // Move
            maslow_hal.move(selected_motor, -10);
            break;

        case 'I':
        case 'M':
        case 'D':
        case 'P':
        case 'S':
        case 'A':;
            if(line[3] == '=' && line[4] != '\0') {
                float parameter;
                uint_fast8_t counter = 4;

                if(!read_float(line, &counter, &parameter))
                    retval = Status_BadNumberFormat;

                else switch(line[2]) {

                    case 'P':
                        maslow.pid[selected_motor].Kp = parameter;
                        hal.stream.write("Kp == ");
                        hal.stream.write(ftoa(maslow.pid[selected_motor].Kp, 3));
                        hal.stream.write(ASCII_EOL);
                        break;

                    case 'D':
                        maslow.pid[selected_motor].Kd = parameter;
                        hal.stream.write("Kd == ");
                        hal.stream.write(ftoa(maslow.pid[selected_motor].Kd, 3));
                        hal.stream.write(ASCII_EOL);
                        break;

                    case 'I':
                        maslow.pid[selected_motor].Ki = parameter;
                        hal.stream.write("Ki == ");
                        hal.stream.write(ftoa(maslow.pid[selected_motor].Ki, 3));
                        hal.stream.write(ASCII_EOL);
                        maslow_hal.pid_settings_changed(selected_motor);
                        break;

                    case 'M':
                        maslow.pid[selected_motor].Imax = parameter;
                        hal.stream.write("Imax == ");
                        hal.stream.write(ftoa(maslow.pid[selected_motor].Imax, 3));
                        hal.stream.write(ASCII_EOL);
                        maslow_hal.pid_settings_changed(selected_motor);
                        break;

                    case 'S':
                        {
                            maslow_hal.tuning_enable(true);
                            int32_t sz = maslow_hal.set_step_size(selected_motor, (int32_t)parameter);
                            hal.stream.write("S == ");
                            hal.stream.write(ftoa((float)sz, 0));
                            hal.stream.write(ASCII_EOL);
                        }
                        break;

                    case 'A': // test kinematics - from X,Y mm to A,B steps back to X,Y mm
                        {
                            float xyz[N_AXIS];
                            int32_t abz[N_AXIS];
                            recomputeGeometry();
                            xyz[X_AXIS] = parameter;
                            if(line[counter++] == ',' && line[counter] != '\0') {
                                if(!read_float(line, &counter, &xyz[Y_AXIS]))
                                    retval = Status_BadNumberFormat;
                            } else
                                retval = Status_BadNumberFormat;

                            if(retval == Status_OK) {

                                triangularInverse(abz, xyz);
                                hal.stream.write("[KINEMATICSTRANSFORM: X,Y = ");
                                hal.stream.write(ftoa(xyz[X_AXIS], 3));
                                hal.stream.write(",");
                                hal.stream.write(ftoa(xyz[Y_AXIS], 3));
                                hal.stream.write(" -> A,B steps: ");
                                hal.stream.write(uitoa((uint32_t)abz[A_MOTOR]));
                                hal.stream.write(",");
                                hal.stream.write(uitoa((uint32_t)abz[B_MOTOR]));

                                maslow_convert_array_steps_to_mpos(xyz, abz);
                                hal.stream.write(" -> X,Y = ");
                                hal.stream.write(ftoa(xyz[X_AXIS], 3));
                                hal.stream.write(",");
                                hal.stream.write(ftoa(xyz[Y_AXIS], 3));
                                hal.stream.write("]" ASCII_EOL);
                            }
                        }
                        break;
                }
            } else
                retval = Status_BadNumberFormat;
            break;

        default:
            {
               maslow_debug_t *debug = maslow_hal.get_debug_data(selected_motor);

               hal.stream.write("[AXISPID:");
               hal.stream.write(axis_letter[selected_motor]);
               hal.stream.write(": Kp = ");
               hal.stream.write(ftoa(maslow.pid[selected_motor].Kp, 3));
               hal.stream.write(" Ki = ");
               hal.stream.write(ftoa(maslow.pid[selected_motor].Ki, 3));
               hal.stream.write(" Kd = ");
               hal.stream.write(ftoa(maslow.pid[selected_motor].Kd, 3));
               hal.stream.write(" Imax = ");
               hal.stream.write(ftoa(maslow.pid[selected_motor].Imax, 3));

               hal.stream.write("]\r\n[PIDDATA:err=");
               hal.stream.write(ftoa(debug->Error, 0));
               hal.stream.write("\t\ti=");
               hal.stream.write(ftoa(debug->Integral, 0));
               hal.stream.write("\tiT=");
               hal.stream.write(ftoa(debug->iterm, 0));
               hal.stream.write("\td=");
               hal.stream.write(ftoa(debug->DiffTerm, 0));
    //           hal.stream.write("\tV=");
    //           hal.stream.write(ftoa(debug->totalSpeed, 0));
               hal.stream.write("\txCMD=");
               hal.stream.write(ftoa(debug->speed, 0));
    //           hal.stream.write("\tyCMD=");
    //           hal.stream.write(uitoa(motor[Y_AXIS]->speed));
    //           hal.stream.write("\tzCMD=");
    //           hal.stream.write(uitoa(motor[Z_AXIS]->speed));
               hal.stream.write("]" ASCII_EOL);
            }
           break;
    } else
        retval = Status_Unhandled;

    return retval;
}

// Initialize API pointers & machine parameters for Maslow router kinematics
bool maslow_init (void)
{
    float xy[2] = {0.0f, 0.0f};

    if((nvs_address = nvs_alloc(sizeof(maslow_settings_t)))) {

        details.on_get_settings = grbl.on_get_settings;
        grbl.on_get_settings = on_get_settings;

        recomputeGeometry();
        triangularInverse(sys.position, xy);

        selected_motor = A_MOTOR;

        kinematics.limits_set_target_pos = maslow_limits_set_target_pos;
        kinematics.limits_get_axis_mask = maslow_limits_get_axis_mask;
        kinematics.limits_set_machine_positions = maslow_limits_set_machine_positions;
        kinematics.plan_target_to_steps = maslow_target_to_steps;
        kinematics.convert_array_steps_to_mpos = maslow_convert_array_steps_to_mpos;
        kinematics.segment_line = maslow_segment_line;

        grbl.on_unknown_sys_command = maslow_tuning;
    }

    return nvs_address != 0;
}

#endif

