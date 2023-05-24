/*
  scara.c - scara kinematics implementation

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
*/

#include "grbl.h"

#if SCARA

#include <math.h>
#include <string.h>
#include <stdio.h>

#include "hal.h"
#include "settings.h"
#include "planner.h"
#include "kinematics.h"
#include "report.h"
#include "system.h"

#include "scara.h"

static bool jog_cancel = false;
static on_report_options_ptr on_report_options;
static on_realtime_report_ptr on_realtime_report;

// ************************ Kinematics Calculations ****************************//

// forward kinematics: (absolute) joint angles to cartesian XY
static xy_t q_to_xy(float q1, float q2) {
    xy_t xy;
    xy.x = SCARA_L1*cosf(q1) + SCARA_L2*cosf(q2);
    xy.y = SCARA_L1*sinf(q1) + SCARA_L2*sinf(q2);
    return xy;
}

// backwards kinematics: cartesian XY to joint angles (absolute joint angles)
static q_t xy_to_q(float x, float y) {
    q_t q;
    float r_sq = x*x + y*y;
    if (r_sq > (SCARA_L1 + SCARA_L2)*(SCARA_L1 + SCARA_L2)) {
        q.q1 = q.q2 = NAN;
    } else {
        float cos_q12 = (r_sq - SCARA_L1*SCARA_L1 - SCARA_L2*SCARA_L2) / (2.0f * SCARA_L1 * SCARA_L2);
        float q12 = acosf(cos_q12); //relative angle between l1 and l2
        q.q1 = atan2f(y, x) - atan2f(SCARA_L2*sinf(q12), SCARA_L1 + SCARA_L2*cos_q12);
        #if SCARA_ABSOLUTE_JOINT_ANGLES
            q.q2 = q.q1 + q12;
        #else
            q.q2 = q12;
        #endif
    }
    return q;
}


// *********************** required grblHAL Kinematics functions ************************ //

// Returns machine position in mm converted from system joint angles
static float *scara_transform_to_cartesian(float *position, float *angles)
{
    // higher axis dont need to be modified
    uint_fast8_t idx = N_AXIS-1;
    do {
        position[idx] = angles[idx];
        idx--;
    } while (idx > Y_AXIS);
    
    // apply forward kinematics
    xy_t xy = q_to_xy(angles[X_AXIS], angles[Y_AXIS]);

    position[X_AXIS] = xy.x;
    position[Y_AXIS] = xy.y;

    return position;
}

// Returns machine position in mm converted from system position steps.
static float *scara_transform_steps_to_cartesian(float *position, int32_t *steps)
{
    uint_fast8_t idx = N_AXIS;
    do {
        idx--;
        position[idx] = steps[idx] / settings.axis[idx].steps_per_mm;
    } while (idx);
    
    // apply forward kinematics
    xy_t xy = q_to_xy(position[A_MOTOR], position[B_MOTOR]);

    position[X_AXIS] = xy.x;
    position[Y_AXIS] = xy.y;

    char msgOut[100] = {0};
    snprintf(msgOut, sizeof(msgOut), "steps_to_car: steps:%d,%d|xy:%.05f,%.05f\n", steps[A_MOTOR], steps[B_MOTOR], xy.x, xy.y);
    hal.stream.write(msgOut);

    return position;
}

// Returns join angles in rad, converted from machine position in mm
static float *scara_transform_from_cartesian(float *target_q, float *position_xy)
{
    hal.stream.write("scara_transform_from_cartesian: ");
    // do not change higher axis
    uint_fast8_t idx = N_AXIS-1;
    do {
        target_q[idx] = position_xy[idx];
        idx--;
    } while (idx > Y_AXIS);

    // apply inverse kinematics
    q_t q = xy_to_q(position_xy[A_MOTOR], position_xy[B_MOTOR]);

    // Check if out of reach
    if (isnan(q.q1) || isnan(q.q2)) {
        // trigger soft limit
        system_raise_alarm(Alarm_SoftLimit);
        return NULL;
    }

    char msgOut[100] = {0};
    snprintf(msgOut, sizeof(msgOut), "xy:%.05f,%.05f|q:%.05f,%.05f\n", position_xy[X_AXIS], position_xy[Y_AXIS], q.q1, q.q2);
    hal.stream.write(msgOut);

    target_q[A_MOTOR] = q.q1 ;
    target_q[B_MOTOR] = q.q2 ;

    return target_q;
}


// segment long lines into smaller segments for non-linear kinematics
// target is cartesian, position transformed (joint angles)
// first call: init = true, position is current motor steps, target is cartesian coordinates
// later calls: init = false, position is null, target = init return value, now return: next segment target in joint steps
static float *scara_segment_line (float *target, float *position, plan_line_data_t *plan_data, bool init)
{
    static bool do_segments;
    static uint_fast16_t iterations;
    static coord_data_t delta, segment_target, current_position, final_target;

    uint_fast8_t idx = N_AXIS;
    char msgOut[200] = {0};

    if (init) {
        // resume motion
        jog_cancel = false;

        // save final target
        memcpy(final_target.values, target, sizeof(coord_data_t));

        // get current position in cartesian coordinates
        scara_transform_to_cartesian(current_position.values, position);

        // calculate total delta
        idx = N_AXIS;
        do {
            idx--;
            delta.values[idx] = target[idx] - current_position.values[idx];
        } while(idx);

        // check if segmentation needed
        float distance = sqrtf(delta.x * delta.x + delta.y * delta.y);
        do_segments = !(plan_data->condition.rapid_motion) && distance > MAX_SEG_LENGTH_MM;

        // calculate amount of segments and delta step size
        if (do_segments) {
            iterations = (uint_fast16_t)ceilf(distance / MAX_SEG_LENGTH_MM);
            idx = N_AXIS;
            do {
                idx--;
                delta.values[idx] = delta.values[idx] / (float)iterations;
            } while(idx);

            // save current position as initial segment target
            memcpy(&segment_target, &current_position, sizeof(coord_data_t));
        } 
        else {
            // no segmentation needed: segment target matches final target
            iterations = 1;
            memcpy(&segment_target, &final_target, sizeof(coord_data_t));
        }

        // ensure at least 1 iteration
        iterations++;

        // print debug info
        snprintf(msgOut, sizeof(msgOut), "seg_line|itrs=%d,do_segments=%d,dist=%f,delta=%f,%f,%f\n", iterations, do_segments, distance, delta.x, delta.y, delta.z);
        hal.stream.write(msgOut);
    } 
    else {
        // return next segment
        iterations--;
        if(do_segments && iterations > 1) {
            // increment segment target for all axes
            idx = N_AXIS;
            do {
                idx--;
                segment_target.values[idx] += delta.values[idx];
            } while(idx);
        } else {
            // last segment: segment target matches final target
            memcpy(&segment_target, &final_target, sizeof(coord_data_t));
        }
    }
    
    // convert to joint angles
    scara_transform_from_cartesian(current_position.values, segment_target.values);

    // more debug info
    snprintf(msgOut, sizeof(msgOut), "seg_line|itrs=%d|target_xy=%0.4f,%0.4f|target_q=%0.6f,%0.6f\n", 
        iterations, segment_target.x, segment_target.y, current_position.x, current_position.y);
    hal.stream.write(msgOut);

    if (iterations == 0 || jog_cancel) {
        return NULL;
    } else {
        return current_position.values;
    }
}


static uint_fast8_t scara_limits_get_axis_mask (uint_fast8_t idx)
{
    hal.stream.write("scara_limits_get_axis_mask\n");
    return ((idx == A_MOTOR) || (idx == B_MOTOR)) ? (bit(X_AXIS) | bit(Y_AXIS)) : bit(idx);
}

static void scara_limits_set_target_pos (uint_fast8_t idx)
{
    hal.stream.write("scara_limits_set_target_pos\n");
    xy_t xy;
    xy.x = sys.position[X_AXIS] / settings.axis[X_AXIS].steps_per_mm;
    xy.y = sys.position[Y_AXIS] / settings.axis[Y_AXIS].steps_per_mm;

    q_t q = xy_to_q(xy.x, xy.y);

    switch(idx) {
        case X_AXIS:
            sys.position[A_MOTOR] = q.q1 * settings.axis[A_MOTOR].steps_per_mm;
            break;
        case Y_AXIS:
            sys.position[B_MOTOR] = q.q2 * settings.axis[B_MOTOR].steps_per_mm;
            break;
        default:
            sys.position[idx] = 0;
            break;
    }
}

// Set machine positions for homed limit switches. Don't update non-homed axes.
// NOTE: settings.max_travel[] is stored as a negative value.
static void scara_limits_set_machine_positions (axes_signals_t cycle)
{
    uint_fast8_t idx = N_AXIS;

    hal.stream.write("scara_limits_set_machine_positions\n");

    xy_t xy;
    xy.x = sys.position[X_AXIS] / settings.axis[X_AXIS].steps_per_mm;
    xy.y = sys.position[Y_AXIS] / settings.axis[Y_AXIS].steps_per_mm;
    q_t q;

    int32_t pulloff = 0;
    if (cycle.mask & bit(--idx)) do {
        if (settings.homing.flags.force_set_origin) {
            pulloff = bit_istrue(settings.homing.dir_mask.value, bit(idx))
                        ? lroundf((settings.axis[idx].max_travel + settings.homing.pulloff) * settings.axis[idx].steps_per_mm)
                        : lroundf(-settings.homing.pulloff * settings.axis[idx].steps_per_mm);
        }
        
        switch(--idx) {
            case X_AXIS:
                q = xy_to_q(xy.x, xy.y);
                sys.position[A_MOTOR] = q.q1 * settings.axis[A_MOTOR].steps_per_mm + pulloff;
                break;
            case Y_AXIS:
                q = xy_to_q(xy.x, xy.y);
                sys.position[B_MOTOR] = q.q2 * settings.axis[B_MOTOR].steps_per_mm + pulloff;
                break;
            default:
                sys.position[idx] = 0;
                break;
        }
    } while (idx);
}


// ********************* other functions ********************** //

static void cancel_jog (sys_state_t state)
{
    jog_cancel = true;
}

static void report_options (bool newopt)
{
    on_report_options(newopt);  // call original report before adding new info
    if(!newopt) {
        hal.stream.write("[KINEMATICS:Scara v0.01]" ASCII_EOL);
    }
}

static void report_angles (stream_write_ptr stream_write, report_tracking_flags_t report)
{
    stream_write("|Qj:");
    stream_write(ftoa(sys.position[A_MOTOR] / settings.axis[A_MOTOR].steps_per_mm, 3));
    stream_write(",");
    stream_write(ftoa(sys.position[B_MOTOR] / settings.axis[B_MOTOR].steps_per_mm, 3));
    
    if (on_realtime_report){
        on_realtime_report(stream_write, report);
    }
}


// Initialize API pointers for scara kinematics
void scara_init(void){
    // set initial angles:
    sys.position[A_MOTOR] = (int32_t)(-3.14159*0.5 * settings.axis[A_MOTOR].steps_per_mm );
    sys.position[B_MOTOR] = 0; //(int32_t)( 3.14159*0.5 * settings.axis[B_MOTOR].steps_per_mm );

    // specify custom kinematics functions
    kinematics.transform_steps_to_cartesian = scara_transform_steps_to_cartesian;
    kinematics.transform_from_cartesian = scara_transform_from_cartesian;
    kinematics.segment_line = scara_segment_line;
    
    kinematics.limits_get_axis_mask = scara_limits_get_axis_mask;
    kinematics.limits_set_target_pos = scara_limits_set_target_pos;
    kinematics.limits_set_machine_positions = scara_limits_set_machine_positions;

    // jog cancel interrupt line segmentation
    grbl.on_jog_cancel = cancel_jog;

    // add additional report info
    on_report_options = grbl.on_report_options;
    grbl.on_report_options = report_options;

    // add q angles to realtime report
    on_realtime_report = grbl.on_realtime_report;
    grbl.on_realtime_report = report_angles; 
}

#endif
