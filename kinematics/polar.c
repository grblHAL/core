/*
  polar.c - polar robot kinematics implementation

  Part of grblHAL

  Code lifted from Grbl_Esp32 from request by user @ https://github.com/Melkiyby

  Note: homing is not implemented!

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

#include "../grbl.h"

#if POLAR_ROBOT

#include <math.h>
#include <string.h>

#include "../hal.h"
#include "../settings.h"
#include "../planner.h"
#include "../kinematics.h"

#define RADIUS_AXIS X_AXIS
#define POLAR_AXIS Y_AXIS
#define MAX_SEG_LENGTH_MM 0.5f

static bool jog_cancel = false;
static coord_data_t last_pos = {0};
static on_report_options_ptr on_report_options;

// Simple hypotenuse computation function.
inline static float hypot_f (float x, float y)
{
    return sqrtf(x * x + y * y);
}

// Return a 0-360 angle ... fix above 360 and below zero
inline static float abs_angle (float ang)
{
    ang = fmodf(ang, 360.0f);  // 0-360 or 0 to -360

    return ang < 0.0f ? 360.0f + ang : ang;
}

// Returns machine position in mm converted from system position steps.
static float *transform_to_cartesian (float *target, float *position)
{
    target[X_AXIS] = cosf(position[POLAR_AXIS] * RADDEG) * position[RADIUS_AXIS];
    target[Y_AXIS] = sinf(position[POLAR_AXIS] * RADDEG) * position[RADIUS_AXIS];
    target[Z_AXIS] = position[Z_AXIS];  // unchanged

    return target;
}

// Returns machine position in mm converted from system position steps.
static float *polar_convert_array_steps_to_mpos (float *position, int32_t *steps)
{
    float cpos[N_AXIS];

    uint_fast8_t idx = N_AXIS;
    do {
        idx--;
        cpos[idx] = steps[idx] / settings.axis[idx].steps_per_mm;
    } while(idx);

    return transform_to_cartesian(position, cpos);

    return position;
}

// Transform absolute position from cartesian coordinate system to polar coordinate system
static float *transform_from_cartesian (float *target, float *position)
{
    float delta_ang;  // the difference from the last and next angle
    uint_fast8_t idx = N_AXIS - 1;

    do {
        target[idx] = position[idx];
    } while(--idx > Y_AXIS);

    target[RADIUS_AXIS] = hypot_f(position[X_AXIS], position[Y_AXIS]);
    if (target[RADIUS_AXIS] == 0.0f) {
        target[POLAR_AXIS] = last_pos.values[POLAR_AXIS];  // don't care about angle at center
    } else {
        target[POLAR_AXIS] = atan2f(position[Y_AXIS], position[X_AXIS]) * DEGRAD;
        // no negative angles...we want the absolute angle not -90, use 270
        target[POLAR_AXIS] = abs_angle(target[POLAR_AXIS]);
    }

    delta_ang = target[POLAR_AXIS] - abs_angle(last_pos.values[POLAR_AXIS]);
    // if the delta is above 180 degrees it means we are crossing the 0 degree line
    if (fabs(delta_ang) <= 180.0f)
        target[POLAR_AXIS] = last_pos.values[POLAR_AXIS] + delta_ang;
    else
        target[POLAR_AXIS] = last_pos.values[POLAR_AXIS] + (delta_ang > 0.0f ? - (360.0f - delta_ang) : delta_ang + 360.0f);

    return target;
}

static inline float get_distance (float *p0, float *p1)
{
    uint_fast8_t idx = N_AXIS;
    float distance = 0.0f;

    do {
        idx--;
        distance += (p0[idx] - p1[idx]) * (p0[idx] - p1[idx]);
    } while(idx);

    return sqrtf(distance);
}

// Polar is circular in motion, so long lines must be divided up
static float *polar_segment_line (float *target, float *position, plan_line_data_t *pl_data, bool init)
{
    static uint_fast16_t iterations;
    static bool segmented;
    static float r_offset, distance;
    static coord_data_t delta, segment_target, final_target, cpos;
//    static plan_line_data_t plan;

    uint_fast8_t idx = N_AXIS;

    if(init) {

        jog_cancel = false;
        r_offset = gc_get_offset(RADIUS_AXIS) * 2.0f; //??

        memcpy(final_target.values, target, sizeof(final_target));

        transform_to_cartesian(segment_target.values, position);

        delta.x = target[X_AXIS] - segment_target.x;
        delta.y = target[Y_AXIS] - segment_target.y;
        delta.z = target[Z_AXIS] - segment_target.z;

        distance = sqrtf(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);

        if((segmented = !pl_data->condition.rapid_motion && distance > MAX_SEG_LENGTH_MM && !(delta.x == 0.0f && delta.y == 0.0f))) {

            idx = N_AXIS;
            iterations = (uint_fast16_t)ceilf(distance / MAX_SEG_LENGTH_MM);

            do {
                --idx;
                delta.values[idx] = delta.values[idx] / (float)iterations;
            } while(idx);

        } else {
            iterations = 1;
            memcpy(&segment_target, &final_target, sizeof(coord_data_t));
        }

        iterations++; // return at least one iteration

    } else {

        iterations--;

        if(segmented && iterations > 1) {
            do {
                idx--;
                segment_target.values[idx] += delta.values[idx];
            } while(idx);

        } else
            memcpy(&segment_target, &final_target, sizeof(coord_data_t));

        segment_target.values[RADIUS_AXIS] -= r_offset;
        transform_from_cartesian(cpos.values, segment_target.values);
        segment_target.values[RADIUS_AXIS] += r_offset;

        if(!pl_data->condition.rapid_motion) {
            float fr_mul = distance / get_distance(last_pos.values, cpos.values);
            pl_data->feed_rate *= fr_mul == 0.0f ? 1.0 : (fr_mul < 0.5f ? 0.5f : fr_mul);
        }

        memcpy(&last_pos, &cpos, sizeof(coord_data_t));
    }

    return iterations == 0 || jog_cancel ? NULL : cpos.values;
}

static uint_fast8_t polar_limits_get_axis_mask (uint_fast8_t idx)
{
    return 0;
}

static void polar_limits_set_target_pos (uint_fast8_t idx) // fn name?
{
    switch(idx) {
        case X_AXIS:
            break;
        case Y_AXIS:
            break;
        default:
            sys.position[idx] = 0;
            break;
    }
}

// Set machine positions for homed limit switches. Don't update non-homed axes.
// NOTE: settings.max_travel[] is stored as a negative value.
static void polar_limits_set_machine_positions (axes_signals_t cycle)
{
}

static void cancel_jog (sys_state_t state)
{
    jog_cancel = true;
}

static void report_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[KINEMATICS:Polar v0.01]" ASCII_EOL);
}

static bool polar_homing_cycle (axes_signals_t cycle, axes_signals_t auto_square)
{
    report_message("Homing is not implemented!", Message_Warning);

    return false;
}

// Initialize API pointers for Wall Plotter kinematics
void polar_init (void)
{
    kinematics.limits_set_target_pos = polar_limits_set_target_pos;
    kinematics.limits_get_axis_mask = polar_limits_get_axis_mask;
    kinematics.limits_set_machine_positions = polar_limits_set_machine_positions;
    kinematics.transform_from_cartesian = transform_from_cartesian;
    kinematics.transform_steps_to_cartesian = polar_convert_array_steps_to_mpos;
    kinematics.segment_line = polar_segment_line;

    grbl.home_machine = polar_homing_cycle;
    grbl.on_jog_cancel = cancel_jog;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = report_options;
}

#endif
