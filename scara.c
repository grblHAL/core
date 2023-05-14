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

#define A_MOTOR X_AXIS // Must be X_AXIS
#define B_MOTOR Y_AXIS // Must be Y_AXIS

#define MAX_SEG_LENGTH_MM 2.0f // mm

#define SCARA_L1 700.0f // mm
#define SCARA_L2 600.0f // mm
#define SCARA_REACH (SCARA_L1 + SCARA_L2)

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

static bool jog_cancel = false;
static on_report_options_ptr on_report_options;
static on_realtime_report_ptr on_realtime_report;

// ************************ Kinematics Calculations ****************************//

// forward kinematics: joint angles to cartesian XY
static xy_t q_to_xy(float q1, float q2) {
    xy_t xy;
    xy.x = SCARA_L1*cosf(q1) + SCARA_L2*cosf(q1 + q2);
    xy.y = SCARA_L1*sinf(q1) + SCARA_L2*sinf(q1 + q2);
    return xy;
}

// backwards kinematics: cartesian XY to joint angles
static q_t xy_to_q(float x, float y) {
    q_t q;
    float r_sq = x*x + y*y;
    if (r_sq > SCARA_REACH*SCARA_REACH) {
        q.q1 = q.q2 = NAN;
    } else {
        float cos_q2 = (r_sq - SCARA_L1*SCARA_L1 - SCARA_L2*SCARA_L2) / (-2.0f * SCARA_L1 * SCARA_L2);
        q.q2 = acosf(cos_q2);
        q.q1 = atan2f(y, x) - atan2f(SCARA_L2*sinf(q.q2), SCARA_L1 + SCARA_L2*cos_q2); //TODO: use faster atan2 approximation
    }
    return q;
}

// *********************** required grblHAL Kinematics functions ************************ //

// Returns machine position in mm converted from system position steps.
static float *scara_convert_array_steps_to_mpos (float *position, int32_t *steps)
{
    q_t q;
    q.q1 = (float)steps[A_MOTOR] / settings.axis[A_MOTOR].steps_per_mm; //actually steps mer radian
    q.q2 = (float)steps[B_MOTOR] / settings.axis[B_MOTOR].steps_per_mm;

    // apply forward kinematics
    xy_t xy = q_to_xy(q.q1, q.q2);

    position[X_AXIS] = xy.x;
    position[Y_AXIS] = xy.y;
    position[Z_AXIS] = steps[Z_AXIS] / settings.axis[Z_AXIS].steps_per_mm;

    return position;
}

static float *scara_transform_from_cartesian(float *target, float *position)
{
    // do not change higher axis
    uint_fast8_t idx = N_AXIS - 1;
    do {
        target[idx] = position[idx];
    } while (--idx > Y_AXIS);

    // apply inverse kinematics
    q_t q = xy_to_q(position[X_AXIS], position[Y_AXIS]);

    target[A_MOTOR] = q.q1 * settings.axis[A_MOTOR].steps_per_mm;
    target[B_MOTOR] = q.q2 * settings.axis[B_MOTOR].steps_per_mm;

    return target;
}

// segment long lines into smaller segments, then apply kinematics for every segment.
// first runs initialization to calculate amount of segment, then repeats segment coordinates after. return NULL when done.
static float *scara_segment_line (float *target, float *position, plan_line_data_t *plan_data, bool init)
{
    static uint_fast16_t iterations;        // number of segments to be generated
    static bool do_segmentation;            // false distance too small or rapid motion
    static coord_data_t delta;  
    static coord_data_t segment_target, final_target, cur_pos;
    //static plan_line_data_t plan;

    uint_fast8_t idx = N_AXIS;

    // initialization: 
    if(init) {
        jog_cancel = false; // resume motion

        // copy from target pointer to static local array
        memcpy(final_target.values, target, sizeof(final_target));

        // convert position in joint space to target in cartesian space
        //scara_transform_to_cartesian(segment_target.values, position);
        q_t q;
        q.q1 = position[A_MOTOR] / settings.axis[A_MOTOR].steps_per_mm;
        q.q2 = position[B_MOTOR] / settings.axis[B_MOTOR].steps_per_mm;
        xy_t xy = q_to_xy(q.q1, q.q2);
        cur_pos.x = xy.x;
        cur_pos.y = xy.y;
        cur_pos.z = position[Z_AXIS] / settings.axis[Z_AXIS].steps_per_mm;

        // delta vector between current position and target
        delta.x = target[X_AXIS] - cur_pos.x;
        delta.y = target[Y_AXIS] - cur_pos.y;
        delta.z = target[Z_AXIS] - cur_pos.z;

        char msgOut[40];
        snprintf(msgOut, sizeof(msgOut), "full delta: X:%f Y:%f Z:%f\r\n", delta.x, delta.y, delta.z);
        hal.stream.write(msgOut);

        // check if segmentation is needed
        float distance = sqrtf(delta.x * delta.x + delta.y * delta.y);

        bool do_segmentation = !(plan_data->condition.rapid_motion) && distance > MAX_SEG_LENGTH_MM;
        if (do_segmentation) {
            iterations = (uint_fast16_t)ceilf(distance / MAX_SEG_LENGTH_MM);

            // adjust delta vector to match segment length
            idx = N_AXIS;
            do {
                --idx;
                delta.values[X_AXIS] = delta.values[X_AXIS] / (float)iterations;
            } while(idx);
        } else {
            // no segmentation needed: segment target matches final target
            iterations = 1;
            memcpy(&segment_target, &final_target, sizeof(coord_data_t));
        }
        snprintf(msgOut, sizeof(msgOut), "segm delta: X:%f Y:%f Z:%f\r\n", delta.x, delta.y, delta.z);
        hal.stream.write(msgOut);

        iterations++; // return at least one iteration

    // return next segment
    } else {
        iterations--;
        if(do_segmentation && iterations > 1) {
            // increment segment target for all axes
            do {
                idx--;
                segment_target.values[idx] += delta.values[idx];
            } while(idx);
        } else
            // last segment: segment target matches final target
            memcpy(&segment_target, &final_target, sizeof(coord_data_t));

        // convert segment target in cartesian space to joint space
        scara_transform_from_cartesian(cur_pos.values, segment_target.values);
    }

    if (iterations == 0 || jog_cancel) {
        return NULL;
    } else {
        return cur_pos.values;
    }
}


static uint_fast8_t scara_limits_get_axis_mask (uint_fast8_t idx)
{
    return ((idx == A_MOTOR) || (idx == B_MOTOR)) ? (bit(X_AXIS) | bit(Y_AXIS)) : bit(idx);
}


static void scara_limits_set_target_pos (uint_fast8_t idx)
{
    xy_t xy;
    xy.x = sys.position[X_AXIS] / settings.axis[X_AXIS].steps_per_mm;
    xy.y = sys.position[Y_AXIS] / settings.axis[Y_AXIS].steps_per_mm;

    q_t q = xy_to_q(xy.x, xy.y);

    switch(idx) {
        case X_AXIS:
        case Y_AXIS:
            sys.position[A_MOTOR] = q.q1 * settings.axis[A_MOTOR].steps_per_mm;
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
    if (true) { // on report.status_report?
        char *q1 = ftoa(sys.position[A_MOTOR] / settings.axis[A_MOTOR].steps_per_mm, 3);
        char *q2 = ftoa(sys.position[B_MOTOR] / settings.axis[B_MOTOR].steps_per_mm, 3);
        stream_write("|Q:");
        stream_write(q1);
        stream_write(",");
        stream_write(q2);
    }
    
    if (on_realtime_report){
        on_realtime_report(stream_write, report);
    }
}


// Initialize API pointers for scara kinematics
void scara_init(void){
    // specify custom kinematics functions
    kinematics.transform_steps_to_cartesian = scara_convert_array_steps_to_mpos;
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
