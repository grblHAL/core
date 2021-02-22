/*
  wall_plotter.c - wall plotter kinematics implementation

  Part of grblHAL

  Code lifted from Grbl_Esp32 pull request by user @ https://github.com/rognlien

  Original code here: https://github.com/jasonwebb/grbl-mega-wall-plotter
  Note: homing is not implemented!

  Bits also pulled from: https://github.com/ldocull/MaslowDue

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

#ifdef WALL_PLOTTER

#include "settings.h"
#include "planner.h"
#include "kinematics.h"

#define A_MOTOR X_AXIS // Must be X_AXIS
#define B_MOTOR Y_AXIS // Must be Y_AXIS
#define MAX_SEG_LENGTH_MM 2.0f

typedef struct {
    int32_t width;
    float width_mm;
    float width_pow;
    int32_t height;
    int32_t width_2;
    int32_t height_2;
    int32_t spindlezero[2];
    float spindlezero_mm[2];
} machine_t;

typedef struct {
    float a;
    float b;
} coord_t;

static machine_t machine = {0};

// Returns machine position in mm converted from system position steps.
// TODO: perhaps change to double precision here - float calculation results in errors of a couple of micrometers.
static void wp_convert_array_steps_to_mpos (float *position, int32_t *steps)
{
    coord_t len;

    len.a = (float)steps[A_MOTOR] / settings.axis[A_MOTOR].steps_per_mm;
    len.b = (float)steps[B_MOTOR] / settings.axis[B_MOTOR].steps_per_mm;

    position[X_AXIS] = (machine.width_pow + len.a * len.a - len.b * len.b) / (2.0f * machine.width_mm);
    len.a = machine.width_mm - position[X_AXIS];
    position[Y_AXIS] = sqrtf(len.b * len.b - len.a * len.a );
    position[Z_AXIS] = steps[Z_AXIS] / settings.axis[Z_AXIS].steps_per_mm;
}

// Wall plotter calculation only. Returns x or y-axis "steps" based on wall plotter motor steps.
// A length = sqrt( X^2 + Y^2 )
// B length = sqrt( (MACHINE_WIDTH - X)^2 + Y^2 )
inline static int32_t wp_convert_to_a_motor_steps (float *target)
{
    return (int32_t)lroundf(sqrtf(target[A_MOTOR] * target[A_MOTOR] + target[B_MOTOR] * target[B_MOTOR]) * settings.axis[A_MOTOR].steps_per_mm);
}

inline static int32_t wp_convert_to_b_motor_steps (float *target)
{
    float xpos = machine.width_mm - target[A_MOTOR];
    return (int32_t)lroundf(sqrtf(xpos * xpos + target[B_MOTOR] * target[B_MOTOR]) * settings.axis[B_MOTOR].steps_per_mm);
}

// Transform absolute position from cartesian coordinate system (mm) to wall plotter coordinate system (step)
static void wp_plan_target_to_steps (int32_t *target_steps, float *target)
{
    uint_fast8_t idx = N_AXIS - 1;

    do {
        target_steps[idx] = lroundf(target[idx] * settings.axis[idx].steps_per_mm);
    } while(--idx > Y_AXIS);

    target_steps[A_MOTOR] = wp_convert_to_a_motor_steps(target);
    target_steps[B_MOTOR] = wp_convert_to_b_motor_steps(target);
}

// Wall plotter is circular in motion, so long lines must be divided up
static bool wp_segment_line (float *target, plan_line_data_t *pl_data, bool init)
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


static uint_fast8_t wp_limits_get_axis_mask (uint_fast8_t idx)
{
    return ((idx == A_MOTOR) || (idx == B_MOTOR)) ? (bit(X_AXIS) | bit(Y_AXIS)) : bit(idx);
}


static void wp_limits_set_target_pos (uint_fast8_t idx) // fn name?
{
    float xy[2];
    int32_t axis_position;

    xy[X_AXIS] = sys.position[X_AXIS] / settings.axis[X_AXIS].steps_per_mm;
    xy[Y_AXIS] = sys.position[Y_AXIS] / settings.axis[Y_AXIS].steps_per_mm;

    switch(idx) {
        case X_AXIS:
            axis_position = wp_convert_to_b_motor_steps(xy);
            sys.position[A_MOTOR] = axis_position;
            sys.position[B_MOTOR] = -axis_position;
            break;
        case Y_AXIS:
            sys.position[A_MOTOR] = sys.position[B_MOTOR] = wp_convert_to_a_motor_steps(xy);
            break;
        default:
            sys.position[idx] = 0;
            break;
    }
}


// Set machine positions for homed limit switches. Don't update non-homed axes.
// NOTE: settings.max_travel[] is stored as a negative value.
static void wp_limits_set_machine_positions (axes_signals_t cycle)
{
    float xy[2];
    uint_fast8_t idx = N_AXIS;

    xy[X_AXIS] = sys.position[X_AXIS] / settings.axis[X_AXIS].steps_per_mm;
    xy[Y_AXIS] = sys.position[Y_AXIS] / settings.axis[Y_AXIS].steps_per_mm;

    if(settings.homing.flags.force_set_origin) {
        if (cycle.mask & bit(--idx)) do {
            switch(--idx) {
                case X_AXIS:
                    sys.position[A_MOTOR] = wp_convert_to_b_motor_steps(xy);
                    sys.position[B_MOTOR] = - sys.position[A_MOTOR];
                    break;
                case Y_AXIS:
                    sys.position[A_MOTOR] = wp_convert_to_a_motor_steps(xy);
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
                     off_axis_position = wp_convert_to_b_motor_steps(xy);
                     sys.position[A_MOTOR] = set_axis_position + off_axis_position;
                     sys.position[B_MOTOR] = set_axis_position - off_axis_position;
                     break;
                 case Y_AXIS:
                     off_axis_position = wp_convert_to_a_motor_steps(xy);
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


// Initialize API pointers for Wall Plotter kinematics
void wall_plotter_init (void)
{
    machine.width_mm = -settings.axis[A_MOTOR].max_travel;
    machine.width = (int32_t)(machine.width_mm * settings.axis[A_MOTOR].steps_per_mm);
    machine.width_2 = machine.width >> 1;
    machine.width_pow = machine.width_mm * machine.width_mm;
    machine.height = (int32_t)((float)settings.axis[B_MOTOR].max_travel * settings.axis[B_MOTOR].steps_per_mm);
    machine.height_2 = machine.height >> 1;
    machine.spindlezero[A_MOTOR] = 0; // machine.width_2;
    machine.spindlezero[B_MOTOR] = 0; // machine.height_2;
    machine.spindlezero_mm[A_MOTOR] = (float)machine.spindlezero[A_MOTOR] / settings.axis[A_MOTOR].steps_per_mm;
    machine.spindlezero_mm[B_MOTOR] = (float)machine.spindlezero[B_MOTOR] / settings.axis[B_MOTOR].steps_per_mm;

    sys.position[B_MOTOR] = machine.width;

    kinematics.limits_set_target_pos = wp_limits_set_target_pos;
    kinematics.limits_get_axis_mask = wp_limits_get_axis_mask;
    kinematics.limits_set_machine_positions = wp_limits_set_machine_positions;
    kinematics.plan_target_to_steps = wp_plan_target_to_steps;
    kinematics.convert_array_steps_to_mpos = wp_convert_array_steps_to_mpos;
    kinematics.segment_line = wp_segment_line;
}

#endif
