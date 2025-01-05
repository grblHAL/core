/*
  motion_control.c - high level interface for issuing motion commands

  Part of grblHAL

  Copyright (c) 2017-2024 Terje Io
  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Backlash compensation code based on code copyright (c) 2017 Patrick F. (Schildkroet)

  Bezier splines based on a pull request for Marlin by Giovanni Mascellani

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

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "hal.h"
#include "nuts_bolts.h"
#include "protocol.h"
#include "machine_limits.h"
#include "state_machine.h"
#include "motion_control.h"
#include "tool_change.h"
#ifdef KINEMATICS_API
#include "kinematics.h"
#endif

#if ENABLE_BACKLASH_COMPENSATION

static float target_prev[N_AXIS] = {0};
static axes_signals_t dir_negative = {0}, backlash_enabled = {0};

void mc_backlash_init (axes_signals_t axes)
{
    uint_fast8_t idx = N_AXIS;

    do {
        idx--;
        if(bit_istrue(axes.mask, bit(idx))) {
            BIT_SET(backlash_enabled.mask, bit(idx), settings.axis[idx].backlash > 0.0001f);
            BIT_SET(dir_negative.mask, bit(idx), bit_isfalse(settings.homing.dir_mask.mask, bit(idx)));
        }
    } while(idx);

    mc_sync_backlash_position();
}

void mc_sync_backlash_position (void)
{
    // Update target_prev
    system_convert_array_steps_to_mpos(target_prev, sys.position);
}

#endif

// Execute linear motion in absolute millimeter coordinates. Feed rate given in millimeters/second
// unless invert_feed_rate is true. Then the feed_rate means that the motion should be completed in
// (1 minute)/feed_rate time.
// NOTE: This is the primary gateway to the grbl planner. All line motions, including arc line
// segments, must pass through this routine before being passed to the planner. The separation of
// mc_line and plan_buffer_line is done primarily to place non-planner-type functions from being
// in the planner and to let backlash compensation or canned cycle integration simple and direct.
bool mc_line (float *target, plan_line_data_t *pl_data)
{
#ifdef KINEMATICS_API
    float feed_rate = pl_data->feed_rate;
    pl_data->rate_multiplier = 1.0f;
    target = kinematics.segment_line(target, plan_get_position(), pl_data, true);
#endif

    // If enabled, check for soft limit violations. Placed here all line motions are picked up
    // from everywhere in grblHAL.
    if(!(pl_data->condition.target_validated && pl_data->condition.target_valid))
        limits_soft_check(target, pl_data->condition);

    // If in check gcode mode, prevent motion by blocking planner. Soft limits still work.
    if(state_get() != STATE_CHECK_MODE && protocol_execute_realtime()) {

        // NOTE: Backlash compensation may be installed here. It will need direction info to track when
        // to insert a backlash line motion(s) before the intended line motion and will require its own
        // plan_check_full_buffer() and check for system abort loop. Also for position reporting
        // backlash steps will need to be also tracked, which will need to be kept at a system level.
        // There are likely some other things that will need to be tracked as well. However, we feel
        // that backlash compensation should NOT be handled by grblHAL itself, because there are a myriad
        // of ways to implement it and can be effective or ineffective for different CNC machines. This
        // would be better handled by the interface as a post-processor task, where the original g-code
        // is translated and inserts backlash motions that best suits the machine.
        // NOTE: Perhaps as a middle-ground, all that needs to be sent is a flag or special command that
        // indicates to grblHAL what is a backlash compensation motion, so that grblHAL executes the move but
        // doesn't update the machine position values. Since the position values used by the g-code
        // parser and planner are separate from the system machine positions, this is doable.

#ifdef KINEMATICS_API
      while(kinematics.segment_line(target, NULL, pl_data, false)) {
#endif

#if ENABLE_BACKLASH_COMPENSATION

        if(backlash_enabled.mask) {

            bool backlash_comp = false;
            uint_fast8_t idx = N_AXIS, axismask = bit(N_AXIS - 1);

            do {
                idx--;
                if(backlash_enabled.mask & axismask) {
                    if(target[idx] > target_prev[idx]) {
                        if (dir_negative.value & axismask) {
                            dir_negative.value &= ~axismask;
                            target_prev[idx] += settings.axis[idx].backlash;
                            backlash_comp = true;
                        }
                    } else if(target[idx] < target_prev[idx] && !(dir_negative.value & axismask)) {
                        dir_negative.value |= axismask;
                        target_prev[idx] -= settings.axis[idx].backlash;
                        backlash_comp = true;
                    }
                }
                axismask >>= 1;
            } while(idx);

            if(backlash_comp) {

                plan_line_data_t pl_backlash;

                plan_data_init(&pl_backlash);
                pl_backlash.condition.rapid_motion = On;
                pl_backlash.condition.backlash_motion = On;
                pl_backlash.line_number = pl_data->line_number;
                pl_backlash.spindle.rpm = pl_data->spindle.rpm;

                // If the buffer is full: good! That means we are well ahead of the robot.
                // Remain in this loop until there is room in the buffer.
                while(plan_check_full_buffer()) {
                    protocol_auto_cycle_start();     // Auto-cycle start when buffer is full.
                    if(!protocol_execute_realtime()) // Check for any run-time commands
                        return false;                // Bail, if system abort.
                }

                plan_buffer_line(target_prev, &pl_backlash);
            }

            memcpy(target_prev, target, sizeof(float) * N_AXIS);
        }

#endif // Backlash comp

        // If the buffer is full: good! That means we are well ahead of the robot.
        // Remain in this loop until there is room in the buffer.
         do {
            if(!protocol_execute_realtime())    // Check for any run-time commands
                return false;                   // Bail, if system abort.
            if(plan_check_full_buffer())
                protocol_auto_cycle_start();    // Auto-cycle start when buffer is full.
            else
                break;
        } while(true);

        // Plan and queue motion into planner buffer.
        // While in M3 laser mode also set spindle state and force a buffer sync
        // if there is a coincident position passed.
        if(!plan_buffer_line(target, pl_data) && pl_data->spindle.hal->cap.laser && pl_data->spindle.state.on && !pl_data->spindle.state.ccw) {
            protocol_buffer_synchronize();
            pl_data->spindle.hal->set_state(pl_data->spindle.hal, pl_data->spindle.state, pl_data->spindle.rpm);
        }

#ifdef KINEMATICS_API
        if(pl_data->condition.jog_motion) {
            sys_state_t state = state_get();
            if ((state == STATE_IDLE || state == STATE_TOOL_CHANGE) && plan_get_current_block() != NULL) { // Check if there is a block to execute.
                state_set(STATE_JOG);
                st_prep_buffer();
                st_wake_up();  // NOTE: Manual start. No state machine required.
            }
        }

        pl_data->feed_rate = feed_rate;
      } // while(kinematics.segment_line()

      pl_data->feed_rate = feed_rate;
#endif
    }

    return !ABORTED;
}

// Execute an arc in offset mode format. position == current xyz, target == target xyz,
// offset == offset from current xyz, plane.axis_X defines circle plane in tool space, plane.axis_linear is
// the direction of helical travel, radius == circle radius, turns > 0 for CCW arcs. Used
// for vector transformation direction and number of full turns to add (abs(turns) - 1).
// The arc is approximated by generating a huge number of tiny, linear segments. The chordal tolerance
// of each segment is configured in settings.arc_tolerance, which is defined to be the maximum normal
// distance from segment to the circle when the end points both lie on the circle.
void mc_arc (float *target, plan_line_data_t *pl_data, float *position, float *offset, float radius, plane_t plane, int32_t turns)
{
    typedef union {
        double values[2];
        struct {
            double x;
            double y;
        };
    } point_2dd_t;

    point_2dd_t rv = {  // Radius vector from center to current location
        .x = -(double)offset[plane.axis_0],
        .y = -(double)offset[plane.axis_1]
    };
    point_2dd_t center = {
        .x = (double)position[plane.axis_0] - rv.x,
        .y = (double)position[plane.axis_1] - rv.y
    };
    point_2dd_t rt = {
        .x = (double)target[plane.axis_0] - center.x,
        .y = (double)target[plane.axis_1] - center.y
    };
    // CCW angle between position and target from circle center. Only one atan2() trig computation required.
    float angular_travel = (float)atan2(rv.x * rt.y - rv.y * rt.x, rv.x * rt.x + rv.y * rt.y);

    if (turns > 0) { // Correct atan2 output per direction
        if (angular_travel <= ARC_ANGULAR_TRAVEL_EPSILON)
            angular_travel += 2.0f * M_PI;
    } else if (angular_travel >= -ARC_ANGULAR_TRAVEL_EPSILON)
        angular_travel -= 2.0f * M_PI;

    if(!pl_data->condition.target_validated && grbl.check_arc_travel_limits) {
        pl_data->condition.target_validated = On;
        pl_data->condition.target_valid = grbl.check_arc_travel_limits((coord_data_t *)target, (coord_data_t *)position,
                                                                        (point_2d_t){ .x = (float)center.x, .y = (float)center.y },
                                                                         radius, plane, turns);
    }

    if(labs(turns) > 1) {

        uint32_t n_turns = labs(turns) - 1;
        float arc_travel = 2.0f * M_PI * (float)n_turns + (turns > 0 ? angular_travel : -angular_travel);
        coord_data_t arc_target;
#if N_AXIS > 3
        uint_fast8_t idx = N_AXIS;
        float linear_per_turn[N_AXIS];
        do {
            idx--;
            if(!(idx == plane.axis_0 || idx == plane.axis_1))
                linear_per_turn[idx] = (target[idx] - position[idx]) / arc_travel * 2.0f * M_PI;
        } while(idx);
#else
        float linear_per_turn = (target[plane.axis_linear] - position[plane.axis_linear]) / arc_travel * 2.0f * M_PI;
#endif

        memcpy(&arc_target, target, sizeof(coord_data_t));

        arc_target.values[plane.axis_0] = position[plane.axis_0];
        arc_target.values[plane.axis_1] = position[plane.axis_1];
        arc_target.values[plane.axis_linear] = position[plane.axis_linear];

        while(n_turns--) {
#if N_AXIS > 3
            idx = N_AXIS;
            do {
                idx--;
                if(!(idx == plane.axis_0 || idx == plane.axis_1))
                    arc_target.values[idx] += linear_per_turn[idx];
            } while(idx);
#else
            arc_target.values[plane.axis_linear] += linear_per_turn;
#endif
            mc_arc(arc_target.values, pl_data, position, offset, radius, plane, turns > 0 ? 1 : -1);
            memcpy(position, arc_target.values, sizeof(coord_data_t));
        }
    }

    // NOTE: Segment end points are on the arc, which can lead to the arc diameter being smaller by up to
    // (2x) settings.arc_tolerance. For 99% of users, this is just fine. If a different arc segment fit
    // is desired, i.e. least-squares, midpoint on arc, just change the mm_per_arc_segment calculation.
    // For the intended uses of grblHAL, this value shouldn't exceed 2000 for the strictest of cases.

    uint_fast16_t segments = 0;

    if(2.0f * radius > settings.arc_tolerance)
        segments = (uint_fast16_t)floorf(fabsf(0.5f * angular_travel * radius) / sqrtf(settings.arc_tolerance * (2.0f * radius - settings.arc_tolerance)));

    if(segments) {

        // Multiply inverse feed_rate to compensate for the fact that this movement is approximated
        // by a number of discrete segments. The inverse feed_rate should be correct for the sum of
        // all segments.
        if (pl_data->condition.inverse_time) {
            pl_data->feed_rate *= segments;
            pl_data->condition.inverse_time = Off; // Force as feed absolute mode over arc segments.
        }

        float theta_per_segment = angular_travel / segments;
#if N_AXIS > 3
        uint_fast8_t idx = N_AXIS;
        float linear_per_segment[N_AXIS];
        do {
            idx--;
            if(!(idx == plane.axis_0 || idx == plane.axis_1))
                linear_per_segment[idx] = (target[idx] - position[idx]) / segments;
        } while(idx);
#else
        float linear_per_segment = (target[plane.axis_linear] - position[plane.axis_linear]) / segments;
#endif

    /* Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
       and phi is the angle of rotation. Solution approach by Jens Geisler.
           r_T = [cos(phi) -sin(phi);
                  sin(phi)  cos(phi] * r ;

       For arc generation, the center of the circle is the axis of rotation and the radius vector is
       defined from the circle center to the initial position. Each line segment is formed by successive
       vector rotations. Single precision values can accumulate error greater than tool precision in rare
       cases. So, exact arc path correction is implemented. This approach avoids the problem of too many very
       expensive trig operations [sin(),cos(),tan()] which can take 100-200 usec each to compute.

       Small angle approximation may be used to reduce computation overhead further. A third-order approximation
       (second order sin() has too much error) holds for most, if not, all CNC applications. Note that this
       approximation will begin to accumulate a numerical drift error when theta_per_segment is greater than
       ~0.25 rad(14 deg) AND the approximation is successively used without correction several dozen times. This
       scenario is extremely unlikely, since segment lengths and theta_per_segment are automatically generated
       and scaled by the arc tolerance setting. Only a very large arc tolerance setting, unrealistic for CNC
       applications, would cause this numerical drift error. However, it is best to set N_ARC_CORRECTION from a
       low of ~4 to a high of ~20 or so to avoid trig operations while keeping arc generation accurate.

       This approximation also allows mc_arc to immediately insert a line segment into the planner
       without the initial overhead of computing cos() or sin(). By the time the arc needs to be applied
       a correction, the planner should have caught up to the lag caused by the initial mc_arc overhead.
       This is important when there are successive arc motions.
    */

        // Computes: cos_T = 1 - theta_per_segment^2/2, sin_T = theta_per_segment - theta_per_segment^3/6) in ~52usec
        float cos_T = 2.0f - theta_per_segment * theta_per_segment;
        float sin_T = theta_per_segment * 0.16666667f * (cos_T + 4.0f);
        cos_T *= 0.5f;

        float sin_Ti;
        float cos_Ti;
        float r_axisi;
        uint_fast16_t i, count = 0;

        for (i = 1; i < segments; i++) { // Increment (segments-1).

            if (count < N_ARC_CORRECTION) {
                // Apply vector rotation matrix.
                r_axisi = rv.x * sin_T + rv.y * cos_T;
                rv.x = rv.x * cos_T - rv.y * sin_T;
                rv.y = r_axisi;
                count++;
            } else {
                // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments.
                // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
                cos_Ti = cosf(i * theta_per_segment);
                sin_Ti = sinf(i * theta_per_segment);
                rv.x = -offset[plane.axis_0] * cos_Ti + offset[plane.axis_1] * sin_Ti;
                rv.y = -offset[plane.axis_0] * sin_Ti - offset[plane.axis_1] * cos_Ti;
                count = 0;
            }

            // Update arc_target location
            position[plane.axis_0] = center.x + rv.x;
            position[plane.axis_1] = center.y + rv.y;
#if N_AXIS > 3
            idx = N_AXIS;
            do {
                idx--;
                if(!(idx == plane.axis_0 || idx == plane.axis_1))
                    position[idx] += linear_per_segment[idx];
            } while(idx);
#else
            position[plane.axis_linear] += linear_per_segment;
#endif

            // Bail mid-circle on system abort. Runtime command check already performed by mc_line.
            if(!mc_line(position, pl_data))
                return;
        }
    }

    // Ensure last segment arrives at target location.
    mc_line(target, pl_data);
}

// Bezier splines, from a pull request for Marlin
// By Giovanni Mascellani - https://github.com/giomasce/Marlin

// Compute the linear interpolation between two real numbers.
static inline float interp (const float a, const float b, const float t)
{
    return (1.0f - t) * a + t * b;
}

/**
 * Compute a B�zier curve using the De Casteljau's algorithm (see
 * https://en.wikipedia.org/wiki/De_Casteljau's_algorithm), which is
 * easy to code and has good numerical stability (very important,
 * since Arudino works with limited precision real numbers).
 */
static inline float eval_bezier (const float a, const float b, const float c, const float d, const float t)
{
    const float iab = interp(a, b, t),
                ibc = interp(b, c, t),
                icd = interp(c, d, t),
                iabc = interp(iab, ibc, t),
                ibcd = interp(ibc, icd, t);

    return interp(iabc, ibcd, t);
}

/**
 * We approximate Euclidean distance with the sum of the coordinates
 * offset (so-called "norm 1"), which is quicker to compute.
 */
static inline float dist1 (const float x1, const float y1, const float x2, const float y2)
{
    return fabsf(x1 - x2) + fabsf(y1 - y2);
}

/**
 * The algorithm for computing the step is loosely based on the one in Kig
 * (See https://sources.debian.net/src/kig/4:15.08.3-1/misc/kigpainter.cpp/#L759)
 * However, we do not use the stack.
 *
 * The algorithm goes as it follows: the parameters t runs from 0.0 to
 * 1.0 describing the curve, which is evaluated by eval_bezier(). At
 * each iteration we have to choose a step, i.e., the increment of the
 * t variable. By default the step of the previous iteration is taken,
 * and then it is enlarged or reduced depending on how straight the
 * curve locally is. The step is always clamped between MIN_STEP/2 and
 * 2*MAX_STEP. MAX_STEP is taken at the first iteration.
 *
 * For some t, the step value is considered acceptable if the curve in
 * the interval [t, t+step] is sufficiently straight, i.e.,
 * sufficiently close to linear interpolation. In practice the
 * following test is performed: the distance between eval_bezier(...,
 * t+step/2) is evaluated and compared with 0.5*(eval_bezier(...,
 * t)+eval_bezier(..., t+step)). If it is smaller than SIGMA, then the
 * step value is considered acceptable, otherwise it is not. The code
 * seeks to find the larger step value which is considered acceptable.
 *
 * At every iteration the recorded step value is considered and then
 * iteratively halved until it becomes acceptable. If it was already
 * acceptable in the beginning (i.e., no halving were done), then
 * maybe it was necessary to enlarge it; then it is iteratively
 * doubled while it remains acceptable. The last acceptable value
 * found is taken, provided that it is between MIN_STEP and MAX_STEP
 * and does not bring t over 1.0.
 *
 * Caveat: this algorithm is not perfect, since it can happen that a
 * step is considered acceptable even when the curve is not linear at
 * all in the interval [t, t+step] (but its mid point coincides "by
 * chance" with the midpoint according to the parametrization). This
 * kind of glitches can be eliminated with proper first derivative
 * estimates; however, given the improbability of such configurations,
 * the mitigation offered by MIN_STEP and the small computational
 * power available on Arduino, I think it is not wise to implement it.
 */

void mc_cubic_b_spline (float *target, plan_line_data_t *pl_data, float *position, float *first, float *second)
{
    float bez_target[N_AXIS];

    memcpy(bez_target, position, sizeof(float) * N_AXIS);

    float t = 0.0f, step = BEZIER_MAX_STEP;

    while (t < 1.0f) {

        // First try to reduce the step in order to make it sufficiently
        // close to a linear interpolation.
        bool did_reduce = false;
        float new_t = t + step;

        if(new_t > 1.0f)
            new_t = 1.0f;

        float new_pos0 = eval_bezier(position[X_AXIS], first[X_AXIS], second[X_AXIS], target[X_AXIS], new_t),
              new_pos1 = eval_bezier(position[Y_AXIS], first[Y_AXIS], second[Y_AXIS], target[Y_AXIS], new_t);

        while(new_t - t >= (BEZIER_MIN_STEP)) {

//            if (new_t - t < (BEZIER_MIN_STEP))
//                break;

            const float candidate_t = 0.5f * (t + new_t),
                      candidate_pos0 = eval_bezier(position[X_AXIS], first[X_AXIS], second[X_AXIS], target[X_AXIS], candidate_t),
                      candidate_pos1 = eval_bezier(position[Y_AXIS], first[Y_AXIS], second[Y_AXIS], target[Y_AXIS], candidate_t),
                      interp_pos0 = 0.5f * (bez_target[X_AXIS] + new_pos0),
                      interp_pos1 = 0.5f * (bez_target[Y_AXIS] + new_pos1);

            if (dist1(candidate_pos0, candidate_pos1, interp_pos0, interp_pos1) <= (BEZIER_SIGMA))
                break;

            new_t = candidate_t;
            new_pos0 = candidate_pos0;
            new_pos1 = candidate_pos1;
            did_reduce = true;
        }

        // If we did not reduce the step, maybe we should enlarge it.
        if (!did_reduce) while (new_t - t <= BEZIER_MAX_STEP) {

//            if (new_t - t > BEZIER_MAX_STEP)
//                break;

            const float candidate_t = t + 2.0f * (new_t - t);

            if (candidate_t >= 1.0f)
                break;

            const float candidate_pos0 = eval_bezier(position[X_AXIS], first[X_AXIS], second[X_AXIS], target[X_AXIS], candidate_t),
                      candidate_pos1 = eval_bezier(position[Y_AXIS], first[Y_AXIS], second[Y_AXIS], target[Y_AXIS], candidate_t),
                      interp_pos0 = 0.5f * (bez_target[X_AXIS] + candidate_pos0),
                      interp_pos1 = 0.5f * (bez_target[Y_AXIS] + candidate_pos1);

            if (dist1(new_pos0, new_pos1, interp_pos0, interp_pos1) > (BEZIER_SIGMA))
                break;

            new_t = candidate_t;
            new_pos0 = candidate_pos0;
            new_pos1 = candidate_pos1;
        }

        // Check some postcondition; they are disabled in the actual
        // Marlin build, but if you test the same code on a computer you
        // may want to check they are respect.
        /*
          assert(new_t <= 1.0);
          if (new_t < 1.0) {
            assert(new_t - t >= (MIN_STEP) / 2.0);
            assert(new_t - t <= (MAX_STEP) * 2.0);
          }
        */

        step = new_t - t;
        t = new_t;

        bez_target[X_AXIS] = new_pos0;
        bez_target[Y_AXIS] = new_pos1;

        // Bail mid-spline on system abort. Runtime command check already performed by mc_line.
        if(!mc_line(bez_target, pl_data))
            return;
    }
}

// end Bezier splines

void mc_canned_drill (motion_mode_t motion, float *target, plan_line_data_t *pl_data, float *position, plane_t plane, uint32_t repeats, gc_canned_t *canned)
{
    pl_data->condition.rapid_motion = On; // Set rapid motion condition flag.

    // if current Z < R, rapid move to R
    if(position[plane.axis_linear] < canned->retract_position) {
        position[plane.axis_linear] = canned->retract_position;
        if(!mc_line(position, pl_data))
            return;
    }

    float position_linear = position[plane.axis_linear],
          retract_to = canned->retract_mode == CCRetractMode_RPos ? canned->retract_position : position_linear;

    // rapid move to X, Y
    memcpy(position, target, sizeof(float) * N_AXIS);
    position[plane.axis_linear] = position_linear;
    if(!mc_line(position, pl_data))
        return;

    while(repeats--) {

        // if current Z > R, rapid move to R
        if(position[plane.axis_linear] > canned->retract_position) {
            position[plane.axis_linear] = canned->retract_position;
            if(!mc_line(position, pl_data))
                return;
        }

        position_linear = position[plane.axis_linear];

        while(position_linear > canned->xyz[plane.axis_linear]) {

            position_linear -= canned->delta;
            if(position_linear < canned->xyz[plane.axis_linear])
                position_linear = canned->xyz[plane.axis_linear];

            pl_data->condition.rapid_motion = Off;

            position[plane.axis_linear] = position_linear;
            if(!mc_line(position, pl_data)) // drill
                return;

            if(canned->dwell > 0.0f)
                mc_dwell(canned->dwell);

            if(canned->spindle_off)
                pl_data->spindle.hal->set_state(pl_data->spindle.hal, (spindle_state_t){0}, 0.0f);

            // rapid retract
            switch(motion) {

                case MotionMode_DrillChipBreak:
                    position[plane.axis_linear] = position[plane.axis_linear] == canned->xyz[plane.axis_linear]
                                                   ? retract_to
                                                   : position[plane.axis_linear] + settings.g73_retract;
                    break;

                default:
                    position[plane.axis_linear] = retract_to;
                    break;
            }

            pl_data->condition.rapid_motion = canned->rapid_retract;
            if(!mc_line(position, pl_data))
                return;

            if(canned->spindle_off)
                spindle_set_state_synced(pl_data->spindle.hal, pl_data->spindle.state, pl_data->spindle.rpm);
        }

        pl_data->condition.rapid_motion = On; // Set rapid motion condition flag.

       // rapid move to next position if incremental mode
        if(repeats && gc_state.modal.distance_incremental) {
            position[plane.axis_0] += canned->xyz[plane.axis_0];
            position[plane.axis_1] += canned->xyz[plane.axis_1];
            if(!mc_line(position, pl_data))
                return;
        }
    }

    memcpy(target, position, sizeof(float) * N_AXIS);
}

// Calculates depth-of-cut (DOC) for a given threading pass.
inline static float calc_thread_doc (uint_fast16_t pass, float cut_depth, float inv_degression)
{
    return cut_depth * powf((float)pass, inv_degression);
}

// Repeated cycle for threading
// G76 P- X- Z- I- J- R- K- Q- H- E- L-
// P - picth, X - main taper distance, Z - final position, I - thread peak offset, J - initial depth, K - full depth
// R - depth regression, Q - compound slide angle, H - spring passes, E - taper, L - taper end

// TODO: change pitch to follow any tapers

void mc_thread (plan_line_data_t *pl_data, float *position, gc_thread_data *thread, bool feed_hold_disabled)
{
    uint_fast16_t pass = 1, passes = 0;
    float doc = thread->initial_depth, inv_degression = 1.0f / thread->depth_degression, thread_length;
    float entry_taper_length = thread->end_taper_type & Taper_Entry ? thread->end_taper_length : 0.0f;
    float exit_taper_length = thread->end_taper_type & Taper_Exit ? thread->end_taper_length : 0.0f;
    float infeed_factor = tanf(thread->infeed_angle * RADDEG);
    float target[N_AXIS], start_z = position[Z_AXIS] + thread->depth * infeed_factor;

    memcpy(target, position, sizeof(float) * N_AXIS);

    // Calculate number of passes
    while(calc_thread_doc(++passes, doc, inv_degression) < thread->depth);

    passes += thread->spring_passes + 1;

    if((thread_length = thread->z_final - position[Z_AXIS]) > 0.0f) {
        if(thread->end_taper_type & Taper_Entry)
            entry_taper_length = -entry_taper_length;
        if(thread->end_taper_type & Taper_Exit)
            exit_taper_length = - exit_taper_length;
    }

    thread_length += entry_taper_length + exit_taper_length;

    if(thread->main_taper_height != 0.0f)
        thread->main_taper_height = thread->main_taper_height * thread_length / (thread_length - (entry_taper_length + exit_taper_length));

    pl_data->condition.rapid_motion = On; // Set rapid motion condition flag.

    // TODO: Add to initial move to compensate for acceleration distance?
    /*
    float acc_distance = pl_data->feed_rate * pl_data->spindle.hal->get_data(SpindleData_RPM)->rpm / settings.acceleration[Z_AXIS];
    acc_distance = acc_distance * acc_distance * settings.acceleration[Z_AXIS] * 0.5f;
     */

    // Initial Z-move for compound slide angle offset.
    if(infeed_factor != 0.0f) {
        target[Z_AXIS] = start_z - doc * infeed_factor;
        if(!mc_line(target, pl_data))
            return;
    }

    while(--passes) {

        if(thread->end_taper_type & Taper_Entry)
            target[X_AXIS] = position[X_AXIS] + (thread->peak + doc - thread->depth) * thread->cut_direction;
        else
            target[X_AXIS] = position[X_AXIS] + (thread->peak + doc) * thread->cut_direction;

        if(!mc_line(target, pl_data))
            return;

        if(!protocol_buffer_synchronize() && state_get() != STATE_IDLE) // Wait until any previous moves are finished.
            return;

        pl_data->condition.rapid_motion = Off;      // Clear rapid motion condition flag,
        pl_data->spindle.state.synchronized = On;   // enable spindle sync for cut
        pl_data->overrides.feed_hold_disable = On;  // and disable feed hold

        // Cut thread pass

        // 1. Entry taper
        if(thread->end_taper_type & Taper_Entry) {

            target[X_AXIS] += thread->depth * thread->cut_direction;
            target[Z_AXIS] -= entry_taper_length;
            if(!mc_line(target, pl_data))
                return;
        }

        // 2. Main part
        target[Z_AXIS] += thread_length;
        if(!mc_line(target, pl_data))
            return;

        // 3. Exit taper
        if(thread->end_taper_type & Taper_Exit) {

            target[X_AXIS] -= thread->depth * thread->cut_direction;
            target[Z_AXIS] -= exit_taper_length;
            if(!mc_line(target, pl_data))
                return;
        }

        pl_data->condition.rapid_motion = On;       // Set rapid motion condition flag and
        pl_data->spindle.state.synchronized = Off;  // disable spindle sync for retract & reposition

        if(passes > 1) {

            // Get DOC of next pass.
            doc = calc_thread_doc(++pass, thread->initial_depth, inv_degression);
            doc = min(doc, thread->depth);

            // 4. Retract
            target[X_AXIS] = position[X_AXIS] + (doc - thread->depth) * thread->cut_direction;
            if(!mc_line(target, pl_data))
                return;

            // Restore disable feed hold status for reposition move.
            pl_data->overrides.feed_hold_disable = feed_hold_disabled;

            // 5. Back to start, add compound slide angle offset when commanded.
            target[Z_AXIS] = start_z - (infeed_factor != 0.0f ? doc * infeed_factor : 0.0f);
            if(!mc_line(target, pl_data))
                return;

        } else {

            doc = thread->depth;
            target[X_AXIS] = position[X_AXIS];
            if(!mc_line(target, pl_data))
                return;
        }
    }
}

// Sets up valid jog motion received from g-code parser, checks for soft-limits, and executes the jog.
status_code_t mc_jog_execute (plan_line_data_t *pl_data, parser_block_t *gc_block, float *position)
{
    // Initialize planner data struct for jogging motions.
    // NOTE: Spindle and coolant are allowed to fully function with overrides during a jog.
    pl_data->feed_rate = gc_block->values.f;
    pl_data->condition.no_feed_override =
    pl_data->condition.jog_motion =
    pl_data->condition.target_valid =
    pl_data->condition.target_validated = On;
    pl_data->line_number = gc_block->values.n;

    if(settings.limits.flags.jog_soft_limited)
        grbl.apply_jog_limits(gc_block->values.xyz, position);
    else if(sys.soft_limits.mask && !grbl.check_travel_limits(gc_block->values.xyz, sys.soft_limits, true))
        return Status_TravelExceeded;

    // Valid jog command. Plan, set state, and execute.
    mc_line(gc_block->values.xyz, pl_data);

#ifndef KINEMATICS_API // kinematics may segment long jog moves triggering auto start (RUN)...
    sys_state_t state = state_get();
    if ((state == STATE_IDLE || state == STATE_TOOL_CHANGE) && plan_get_current_block() != NULL) { // Check if there is a block to execute.
        state_set(STATE_JOG);
        st_prep_buffer();
        st_wake_up();  // NOTE: Manual start. No state machine required.
    }
#endif

    return Status_OK;
}

// Execute dwell in seconds.
void mc_dwell (float seconds)
{
    if (state_get() != STATE_CHECK_MODE) {
        protocol_buffer_synchronize();
        delay_sec(seconds, DelayMode_Dwell);
    }
}

// Perform homing cycle to locate and set machine zero. Only '$H' executes this command.
// NOTE: There should be no motions in the buffer and grblHAL must be in an idle state before
// executing the homing cycle. This prevents incorrect buffered plans after homing.
status_code_t mc_homing_cycle (axes_signals_t cycle)
{
    bool home_all = cycle.mask == 0;
    status_code_t homed_status = Status_OK;

    memset(&sys.last_event.limits, 0, sizeof(limit_signals_t));

    if(settings.homing.flags.manual && (home_all ? sys.homing.mask : (cycle.mask & sys.homing.mask)) == 0) {

        if(home_all)
            cycle.mask = AXES_BITMASK;

        sys.homed.mask |= cycle.mask;
#ifdef KINEMATICS_API
        kinematics.limits_set_machine_positions(cycle);
#else
        limits_set_machine_positions(cycle, false);
#endif
    } else {

        if(settings.axis[0].homing_seek_rate <= 0.0f)
            return Status_HomingDisabled;

        // Check and abort homing cycle, if hard limits are already enabled. Helps prevent problems
        // with machines with limits wired on both ends of travel to one limit pin.
        // TODO: Move the pin-specific LIMIT_BIT call to limits.c as a function.
        if (settings.limits.flags.two_switches && hal.home_cap.a.mask == 0 && limit_signals_merge(hal.limits.get_state()).value) {
            mc_reset(); // Issue system reset and ensure spindle and coolant are shutdown.
            system_set_exec_alarm(Alarm_HardLimit);
            return Status_Unhandled;
        }

#ifdef KINEMATICS_API
        if(kinematics.homing_cycle_validate) {

            uint_fast8_t idx = N_AXIS;

            if (!home_all) { // Perform homing cycle based on mask.
                if(!kinematics.homing_cycle_validate(cycle)) {
                    system_set_exec_alarm(Alarm_HomingFail);
                    return Status_Unhandled;
                }
            } else do {
                if(settings.homing.cycle[--idx].mask) {
                    if(!kinematics.homing_cycle_validate(settings.homing.cycle[idx])) {
                        system_set_exec_alarm(Alarm_HomingFail);
                        return Status_Unhandled;
                    }
                }
            } while(idx);

        }
#endif

        state_set(STATE_HOMING);                        // Set homing system state.
#if COMPATIBILITY_LEVEL == 0
        if(!settings.status_report.when_homing) {
            system_set_exec_state_flag(EXEC_STATUS_REPORT); // Force a status report and
            delay_sec(0.1f, DelayMode_Dwell);               // delay a bit to get it sent (or perhaps wait a bit for a request?)
        }
#endif
        // Turn off spindle and coolant (and update parser state)
        if(spindle_is_on())
            gc_spindle_off();

        if(hal.coolant.get_state().mask)
            gc_coolant((coolant_state_t){0});

        // ---------------------------------------------------------------------------
        // Perform homing routine. NOTE: Special motion case. Only system reset works.

        if (!home_all) // Perform homing cycle based on mask.
            homed_status = limits_go_home(cycle);
        else {

            uint_fast8_t idx = 0;

            sys.homed.mask &= ~sys.homing.mask;

            do {
                if(settings.homing.cycle[idx].mask) {
                    cycle.mask = settings.homing.cycle[idx].mask;
                    if((homed_status = limits_go_home(cycle)) != Status_OK)
                        break;
                }
            } while(++idx < N_AXIS);
        }

        // If hard limits feature enabled, re-enable hard limits pin change register after homing cycle.
        // NOTE: always call at end of homing regadless of setting, may be used to disable
        // sensorless homing or switch back to limit switches input (if different from homing switches)
        hal.limits.enable(settings.limits.flags.hard_enabled, (axes_signals_t){0});
    }

    if(cycle.mask) {

        if(!protocol_execute_realtime()) {  // Check for reset and set system abort.

            if(grbl.on_homing_completed)
                grbl.on_homing_completed(cycle, false);

            return Status_Unhandled;        // Did not complete. Alarm state set by mc_alarm.
        }

        if(homed_status != Status_OK) {

            if(state_get() == STATE_HOMING)
                state_set(STATE_IDLE);

            if(grbl.on_homing_completed)
                grbl.on_homing_completed(cycle, false);

            return homed_status;
        }

        if(home_all && settings.homing.flags.manual)
        {
            cycle.mask = AXES_BITMASK & ~sys.homing.mask;
            sys.homed.mask = AXES_BITMASK;
#ifdef KINEMATICS_API
            kinematics.limits_set_machine_positions(cycle);
#else
            limits_set_machine_positions(cycle, false);
#endif
        }

        // Homing cycle complete! Setup system for normal operation.
        // ---------------------------------------------------------

        // Sync gcode parser and planner positions to homed position.
        sync_position();
    }

    system_add_rt_report(Report_Homed);

    homed_status = settings.limits.flags.hard_enabled &&
                    settings.limits.flags.check_at_init &&
                     (limit_signals_merge(hal.limits.get_state()).value & sys.hard_limits.mask)
                    ? Status_LimitsEngaged
                    : Status_OK;

    if(homed_status == Status_OK)
        limits_set_work_envelope();

    if(grbl.on_homing_completed)
        grbl.on_homing_completed(cycle, homed_status == Status_OK);

    return homed_status;
}

// Perform tool length probe cycle. Requires probe switch.
// NOTE: Upon probe failure, the program will be stopped and placed into ALARM state.
gc_probe_t mc_probe_cycle (float *target, plan_line_data_t *pl_data, gc_parser_flags_t parser_flags)
{
    uint_fast8_t idx = N_AXIS;

    // TODO: Need to update this cycle so it obeys a non-auto cycle start.
    if (state_get() == STATE_CHECK_MODE)
        return GCProbe_CheckMode;

    do {
        idx--;
        sys.probe_position[idx] = lroundf(target[idx] * settings.axis[idx].steps_per_mm);
    } while(idx);

    sys.probe_coordsys_id = gc_state.modal.coord_system.id;

    // Finish all queued commands and empty planner buffer before starting probe cycle.
    if (!protocol_buffer_synchronize())
        return GCProbe_Abort; // Return if system reset has been issued.

    // Initialize probing control variables
    sys.flags.probe_succeeded = Off; // Re-initialize probe history before beginning cycle.
    hal.probe.configure(parser_flags.probe_is_away, true);

#if COMPATIBILITY_LEVEL <= 1
    bool at_g59_3 = false, probe_toolsetter = grbl.on_probe_toolsetter != NULL && state_get() != STATE_TOOL_CHANGE && (sys.homed.mask & (X_AXIS_BIT|Y_AXIS_BIT));

    if(probe_toolsetter)
        grbl.on_probe_toolsetter(NULL, NULL, at_g59_3 = system_xy_at_fixture(CoordinateSystem_G59_3, TOOLSETTER_RADIUS), true);
#endif

    // After syncing, check if probe is already triggered or not connected. If so, halt and issue alarm.
    // NOTE: This probe initialization error applies to all probing cycles.
    probe_state_t probe = hal.probe.get_state();
    if (probe.triggered || !probe.connected) { // Check probe state.
        system_set_exec_alarm(Alarm_ProbeFailInitial);
        protocol_execute_realtime();
        hal.probe.configure(false, false); // Re-initialize invert mask before returning.
        return GCProbe_FailInit; // Nothing else to do but bail.
    }

    if(grbl.on_probe_start) {

        uint_fast8_t idx = N_AXIS;
        axes_signals_t axes = {0};
        coord_data_t position;

        system_convert_array_steps_to_mpos(position.values, sys.position);

        do {
            idx--;
            if(fabsf(target[idx] - position.values[idx]) > TOLERANCE_EQUAL)
                bit_true(axes.mask, bit(idx));
        } while(idx);

        grbl.on_probe_start(axes, target, pl_data);
    }

    // Setup and queue probing motion. Auto cycle-start should not start the cycle.
    if(!mc_line(target, pl_data))
        return GCProbe_Abort;

    // Activate the probing state monitor in the stepper module.
    sys.probing_state = Probing_Active;

    // Perform probing cycle. Wait here until probe is triggered or motion completes.
    system_set_exec_state_flag(EXEC_CYCLE_START);
    do {
        if(!protocol_execute_realtime()) // Check for system abort
            return GCProbe_Abort;
    } while (!(state_get() == STATE_IDLE || state_get() == STATE_TOOL_CHANGE));

    // Probing cycle complete!

    // Set state variables and error out, if the probe failed and cycle with error is enabled.
    if(sys.probing_state == Probing_Active) {
        memcpy(sys.probe_position, sys.position, sizeof(sys.position));
        if(!parser_flags.probe_is_no_error)
            system_set_exec_alarm(Alarm_ProbeFailContact);
    } else
        sys.flags.probe_succeeded = On; // Indicate to system the probing cycle completed successfully.

    sys.probing_state = Probing_Off;    // Ensure probe state monitor is disabled.
    hal.probe.configure(false, false);  // Re-initialize invert mask.
    protocol_execute_realtime();        // Check and execute run-time commands

#if COMPATIBILITY_LEVEL <= 1
    if(probe_toolsetter)
        grbl.on_probe_toolsetter(NULL, NULL, at_g59_3, false);
#endif

    // Reset the stepper and planner buffers to remove the remainder of the probe motion.
    st_reset();             // Reset step segment buffer.
    plan_reset();           // Reset planner buffer. Zero planner positions. Ensure probing motion is cleared.
    plan_sync_position();   // Sync planner position to current machine position.
#if ENABLE_BACKLASH_COMPENSATION
    mc_sync_backlash_position();
#endif

    // All done! Output the probe position as message if configured.
    if(settings.status_report.probe_coordinates)
        report_probe_parameters();

    if(grbl.on_probe_completed)
        grbl.on_probe_completed();

    // Successful probe cycle or Failed to trigger probe within travel. With or without error.
    return sys.flags.probe_succeeded ? GCProbe_Found : GCProbe_FailEnd;
}


// Plans and executes the single special motion case for parking. Independent of main planner buffer.
// NOTE: Uses the always free planner ring buffer head to store motion parameters for execution.
bool mc_parking_motion (float *parking_target, plan_line_data_t *pl_data)
{
    bool ok;

    if (sys.abort)
        return false; // Block during abort.

    if ((ok = plan_buffer_line(parking_target, pl_data))) {
        sys.step_control.execute_sys_motion = On;
        sys.step_control.end_motion = Off;  // Allow parking motion to execute, if feed hold is active.
        st_parking_setup_buffer();          // Setup step segment buffer for special parking motion case.
        st_prep_buffer();
        st_wake_up();
    }

    return ok;
}

void mc_override_ctrl_update (gc_override_flags_t override_state)
{
// Finish all queued commands before altering override control state
    protocol_buffer_synchronize();
    if (!sys.abort)
        sys.override.control = override_state;
}

// Method to ready the system to reset by setting the realtime reset command and killing any
// active processes in the system. This also checks if a system reset is issued while grblHAL
// is in a motion state. If so, kills the steppers and sets the system alarm to flag position
// lost, since there was an abrupt uncontrolled deceleration. Called at an interrupt level by
// realtime abort command and hard limits. So, keep to a minimum.
ISR_CODE void ISR_FUNC(mc_reset)(void)
{
    // Only this function can set the system reset. Helps prevent multiple kill calls.
    if (bit_isfalse(sys.rt_exec_state, EXEC_RESET)) {

        system_set_exec_state_flag(EXEC_RESET);

        if(hal.stream.suspend_read)
            hal.stream.suspend_read(false);

        // Kill steppers only if in any motion state, i.e. cycle, actively holding, or homing.
        // NOTE: If steppers are kept enabled via the step idle delay setting, this also keeps
        // the steppers enabled by avoiding the go_idle call altogether, unless the motion state is
        // violated, by which, all bets are off.
        if ((state_get() & (STATE_CYCLE|STATE_HOMING|STATE_JOG)) || sys.step_control.execute_hold || sys.step_control.execute_sys_motion) {

            sys.position_lost = true;

            if (state_get() != STATE_HOMING)
                system_set_exec_alarm(Alarm_AbortCycle);
            else if (!sys.rt_exec_alarm)
                system_set_exec_alarm(Alarm_HomingFailReset);

            st_go_idle(); // Force kill steppers. Position has likely been lost.
        }

        control_signals_t signals = hal.control.get_state();

        if(signals.e_stop)
            system_set_exec_alarm(Alarm_EStop);
        else if(signals.motor_fault)
            system_set_exec_alarm(Alarm_MotorFault);

        if(grbl.on_reset)
            grbl.on_reset();
    }
}
