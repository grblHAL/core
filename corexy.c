/*
  corexy.c - corexy kinematics implementation

  Part of grblHAL

  Copyright (c) 2019-2023 Terje Io
  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC

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

#if COREXY

#include <math.h>

#include "hal.h"
#include "settings.h"
#include "planner.h"
#include "kinematics.h"

// CoreXY motor assignments. DO NOT ALTER.
// NOTE: If the A and B motor axis bindings are changed, this effects the CoreXY equations.
#define A_MOTOR X_AXIS // Must be X_AXIS
#define B_MOTOR Y_AXIS // Must be Y_AXIS

static on_report_options_ptr on_report_options;

// Returns x or y-axis "steps" based on CoreXY motor steps.
inline static int32_t corexy_convert_to_a_motor_steps (int32_t *steps)
{
    return (steps[A_MOTOR] + steps[B_MOTOR]) >> 1;
}

inline static int32_t corexy_convert_to_b_motor_steps (int32_t *steps)
{
    return (steps[A_MOTOR] - steps[B_MOTOR]) >> 1;
}

// Returns machine position of axis 'idx'. Must be sent a 'step' array.
static float *corexy_convert_array_steps_to_mpos (float *position, int32_t *steps)
{
    uint_fast8_t idx;

    position[X_AXIS] = corexy_convert_to_a_motor_steps(steps) / settings.axis[X_AXIS].steps_per_mm;
    position[Y_AXIS] = corexy_convert_to_b_motor_steps(steps) / settings.axis[Y_AXIS].steps_per_mm;

    for(idx = Z_AXIS; idx < N_AXIS; idx++)
        position[idx] = steps[idx] / settings.axis[idx].steps_per_mm;

    return position;
}

// Transform position from cartesian coordinate system to corexy coordinate system
static inline float *transform_from_cartesian (float *target, float *position)
{
    uint_fast8_t idx;

    target[X_AXIS] = position[X_AXIS] + position[Y_AXIS];
    target[Y_AXIS] = position[X_AXIS] - position[Y_AXIS];

    for(idx = Z_AXIS; idx < N_AXIS; idx++)
        target[idx] = position[idx];

    return target;
}

static uint_fast8_t corexy_limits_get_axis_mask (uint_fast8_t idx)
{
    return ((idx == A_MOTOR) || (idx == B_MOTOR)) ? (bit(X_AXIS) | bit(Y_AXIS)) : bit(idx);
}


static void corexy_limits_set_target_pos (uint_fast8_t idx) // fn name?
{
    int32_t axis_position;

    switch(idx) {
        case X_AXIS:
            axis_position = corexy_convert_to_b_motor_steps(sys.position);
            sys.position[A_MOTOR] = axis_position;
            sys.position[B_MOTOR] = -axis_position;
            break;
        case Y_AXIS:
            sys.position[A_MOTOR] = sys.position[B_MOTOR] = corexy_convert_to_a_motor_steps(sys.position);
            break;
        default:
            sys.position[idx] = 0;
            break;
    }
}


// Set machine positions for homed limit switches. Don't update non-homed axes.
// NOTE: settings.max_travel[] is stored as a negative value.
static void corexy_limits_set_machine_positions (axes_signals_t cycle)
{
    uint_fast8_t idx = N_AXIS;

    if(settings.homing.flags.force_set_origin) {
        if (cycle.mask & bit(--idx)) do {
            switch(--idx) {
                case X_AXIS:
                    sys.position[A_MOTOR] = corexy_convert_to_b_motor_steps(sys.position);
                    sys.position[B_MOTOR] = - sys.position[A_MOTOR];
                    break;
                case Y_AXIS:
                    sys.position[A_MOTOR] = corexy_convert_to_a_motor_steps(sys.position);
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
                                          ? lroundf((settings.axis[idx].max_travel + settings.homing.pulloff) * settings.axis[idx].steps_per_mm)
                                          : lroundf(-settings.homing.pulloff * settings.axis[idx].steps_per_mm);
             switch(idx) {
                 case X_AXIS:
                     off_axis_position = corexy_convert_to_b_motor_steps(sys.position);
                     sys.position[A_MOTOR] = set_axis_position + off_axis_position;
                     sys.position[B_MOTOR] = set_axis_position - off_axis_position;
                     break;
                 case Y_AXIS:
                     off_axis_position = corexy_convert_to_a_motor_steps(sys.position);
                     sys.position[A_MOTOR] = off_axis_position + set_axis_position;
                     sys.position[B_MOTOR] = off_axis_position - set_axis_position;
                     break;
                 default:
                     sys.position[idx] = set_axis_position;
                     break;
             }
         }
    } while(idx);
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

// called from mc_line() to segment lines if not overridden, default implementation for pass-through
static float *kinematics_segment_line (float *target, float *position, plan_line_data_t *pl_data, bool init)
{
    static uint_fast8_t iterations;
    static float trsf[N_AXIS];

    if(init) {

        iterations = 2;

        transform_from_cartesian(trsf, target);

        if(!pl_data->condition.rapid_motion) {

            uint_fast8_t idx;
            float cpos[N_AXIS];

            cpos[X_AXIS] = (position[X_AXIS] + position[Y_AXIS]) * .5f;
            cpos[Y_AXIS] = (position[X_AXIS] - position[Y_AXIS]) * .5f;
            for(idx = Z_AXIS; idx < N_AXIS; idx++)
                cpos[idx] = position[idx];

            pl_data->feed_rate *= get_distance(trsf, position) / get_distance(target, cpos);
        }
    }

    return iterations-- == 0 ? NULL : trsf;
}

static bool homing_cycle_validate (axes_signals_t cycle)
{
    return (cycle.mask & (X_AXIS_BIT|Y_AXIS_BIT)) == 0 || cycle.mask < 3;
}

static float homing_cycle_get_feedrate (float feedrate, axes_signals_t cycle)
{
    return feedrate * sqrtf(2.0f);
}

static void report_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[KINEMATICS:CoreXY v2.00]" ASCII_EOL);
}

// Initialize API pointers for CoreXY kinematics
void corexy_init (void)
{
    kinematics.limits_set_target_pos = corexy_limits_set_target_pos;
    kinematics.limits_get_axis_mask = corexy_limits_get_axis_mask;
    kinematics.limits_set_machine_positions = corexy_limits_set_machine_positions;
    kinematics.transform_from_cartesian = transform_from_cartesian;
    kinematics.transform_steps_to_cartesian = corexy_convert_array_steps_to_mpos;
    kinematics.segment_line = kinematics_segment_line;
    kinematics.homing_cycle_validate = homing_cycle_validate;
    kinematics.homing_cycle_get_feedrate = homing_cycle_get_feedrate;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = report_options;
}

#endif
