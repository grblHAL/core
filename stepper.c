/*
  stepper.c - stepper motor driver: executes motion plans using stepper motors

  Part of grblHAL

  Copyright (c) 2016-2024 Terje Io
  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

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
#include "protocol.h"
#include "state_machine.h"

//#define MINIMIZE_PROBE_OVERSHOOT

//#include "debug.h"

//! \cond

// Some useful constants.
#define DT_SEGMENT (1.0f / (ACCELERATION_TICKS_PER_SECOND * 60.0f)) // min/segment
#define REQ_MM_INCREMENT_SCALAR 1.25f

typedef enum {
    Ramp_Accel,
    Ramp_Cruise,
    Ramp_Decel,
    Ramp_DecelOverride
} ramp_type_t;

typedef union {
    uint8_t flags;
    struct {
        uint8_t velocity_profile   :1,
                hold_partial_block :1,
                parking            :1,
                decel_override     :1,
                unassigned         :4;
    };
} prep_flags_t;

// Holds the planner block Bresenham algorithm execution data for the segments in the segment
// buffer. Normally, this buffer is partially in-use, but, for the worst case scenario, it will
// never exceed the number of accessible stepper buffer segments (SEGMENT_BUFFER_SIZE-1).
// NOTE: This data is copied from the prepped planner blocks so that the planner blocks may be
// discarded when entirely consumed and completed by the segment buffer. Also, AMASS alters this
// data for its own use.
static st_block_t st_block_buffer[SEGMENT_BUFFER_SIZE - 1];

// Primary stepper segment ring buffer. Contains small, short line segments for the stepper
// algorithm to execute, which are "checked-out" incrementally from the first block in the
// planner buffer. Once "checked-out", the steps in the segments buffer cannot be modified by
// the planner, where the remaining planner block steps still can.
static segment_t segment_buffer[SEGMENT_BUFFER_SIZE];

// Stepper ISR data struct. Contains the running data for the main stepper ISR.
static stepper_t st;

#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
typedef struct {
    uint32_t level_1;
    uint32_t level_2;
    uint32_t level_3;
} amass_t;

static amass_t amass;
#endif

// Used for blocking new segments being added to the seqment buffer until deceleration starts
// after probe signal has been asserted.
static volatile bool probe_asserted = false;

// Stepper timer ticks per minute
static float cycles_per_min;

// Step segment ring buffer pointers
static volatile segment_t *segment_buffer_tail;
static segment_t *segment_buffer_head, *segment_next_head;

// Pointers for the step segment being prepped from the planner buffer. Accessed only by the
// main program. Pointers may be planning segments or planner blocks ahead of what being executed.
static plan_block_t *pl_block;     // Pointer to the planner block being prepped
static st_block_t *st_prep_block;  // Pointer to the stepper block data being prepped
static st_block_t st_hold_block;   // Copy of stepper block data for block put on hold during parking

// Segment preparation data struct. Contains all the necessary information to compute new segments
// based on the current executing planner block.
typedef struct {
    prep_flags_t recalculate;

    float dt_remainder;
    uint32_t steps_remaining;
    float steps_per_mm;
    float req_mm_increment;

    st_block_t *last_st_block;
    uint32_t last_steps_remaining;
    float last_steps_per_mm;
    float last_dt_remainder;

    ramp_type_t ramp_type;  // Current segment ramp state
    float mm_complete;      // End of velocity profile from end of current planner block in (mm).
                            // NOTE: This value must coincide with a step(no mantissa) when converted.
    float current_speed;    // Current speed at the end of the segment buffer (mm/min)
    float maximum_speed;    // Maximum speed of executing block. Not always nominal speed. (mm/min)
    float exit_speed;       // Exit speed of executing block (mm/min)
#ifdef KINEMATICS_API
    float rate_multiplier;  // Rate multiplier of executing block.
#endif
    float accelerate_until; // Acceleration ramp end measured from end of block (mm)
    float decelerate_after; // Deceleration ramp start measured from end of block (mm)
    float target_position;  //
    float target_feed;      //
    float inv_feedrate;     // Used by PWM laser mode to speed up segment calculations.
    float current_spindle_rpm;
} st_prep_t;

//! \endcond

static st_prep_t prep;

extern void gc_output_message (char *message);

/*    BLOCK VELOCITY PROFILE DEFINITION
          __________________________
         /|                        |\     _________________         ^
        / |                        | \   /|               |\        |
       /  |                        |  \ / |               | \       s
      /   |                        |   |  |               |  \      p
     /    |                        |   |  |               |   \     e
    +-----+------------------------+---+--+---------------+----+    e
    |               BLOCK 1            ^      BLOCK 2          |    d
                                       |
                  time ----->      EXAMPLE: Block 2 entry speed is at max junction velocity

  The planner block buffer is planned assuming constant acceleration velocity profiles and are
  continuously joined at block junctions as shown above. However, the planner only actively computes
  the block entry speeds for an optimal velocity plan, but does not compute the block internal
  velocity profiles. These velocity profiles are computed ad-hoc as they are executed by the
  stepper algorithm and consists of only 7 possible types of profiles: cruise-only, cruise-
  deceleration, acceleration-cruise, acceleration-only, deceleration-only, full-trapezoid, and
  triangle(no cruise).

                                        maximum_speed (< nominal_speed) ->  +
                    +--------+ <- maximum_speed (= nominal_speed)          /|\
                   /          \                                           / | \
 current_speed -> +            \                                         /  |  + <- exit_speed
                  |             + <- exit_speed                         /   |  |
                  +-------------+                     current_speed -> +----+--+
                   time -->  ^  ^                                           ^  ^
                             |  |                                           |  |
                decelerate_after(in mm)                             decelerate_after(in mm)
                    ^           ^                                           ^  ^
                    |           |                                           |  |
                accelerate_until(in mm)                             accelerate_until(in mm)

  The step segment buffer computes the executing block velocity profile and tracks the critical
  parameters for the stepper algorithm to accurately trace the profile. These critical parameters
  are shown and defined in the above illustration.
*/

//

// Callback from delay to deenergize steppers after movement, might been cancelled
void st_deenergize (void *data)
{
    if(sys.steppers_deenergize) {
        hal.stepper.enable(settings.steppers.energize, true);
        sys.steppers_deenergize = false;
    }
}

// Stepper state initialization. Cycle should only start if the st.cycle_start flag is
// enabled. Startup init and limits call this function but shouldn't start the cycle.
void st_wake_up (void)
{
    // Initialize stepper data to ensure first ISR call does not step and
    // cancel any pending steppers deenergize
    //st.exec_block = NULL;
    sys.steppers_deenergize = false;
    hal.stepper.wake_up();
}

// Stepper shutdown
ISR_CODE void ISR_FUNC(st_go_idle)(void)
{
    // Disable Stepper Driver Interrupt. Allow Stepper Port Reset Interrupt to finish, if active.

    sys_state_t state = state_get();

    hal.stepper.go_idle(false);

    // Set stepper driver idle state, disabled or enabled, depending on settings and circumstances.
    if(((settings.steppers.idle_lock_time != 255) || sys.rt_exec_alarm || state == STATE_SLEEP) && state != STATE_HOMING) {
        if(settings.steppers.idle_lock_time == 0 || state == STATE_SLEEP)
            hal.stepper.enable((axes_signals_t){0}, true);
        else {
            // Force stepper dwell to lock axes for a defined amount of time to ensure the axes come to a complete
            // stop and not drift from residual inertial forces at the end of the last movement.
            task_delete(st_deenergize, NULL); // Cancel any pending steppers deenergize task
            sys.steppers_deenergize = task_add_delayed(st_deenergize, NULL, settings.steppers.idle_lock_time);
        }
    } else
        hal.stepper.enable(settings.steppers.idle_lock_time == 255 ? (axes_signals_t){AXES_BITMASK} : settings.steppers.energize, true);
}


/* "The Stepper Driver Interrupt" - This timer interrupt is the workhorse of grblHAL. grblHAL employs
   the venerable Bresenham line algorithm to manage and exactly synchronize multi-axis moves.
   Unlike the popular DDA algorithm, the Bresenham algorithm is not susceptible to numerical
   round-off errors and only requires fast integer counters, meaning low computational overhead
   and maximizing the microcontrollers capabilities. However, the downside of the Bresenham algorithm
   is, for certain multi-axis motions, the non-dominant axes may suffer from un-smooth step
   pulse trains, or aliasing, which can lead to strange audible noises or shaking. This is
   particularly noticeable or may cause motion issues at low step frequencies (0-5kHz), but
   is usually not a physical problem at higher frequencies, although audible.
     To improve Bresenham multi-axis performance, grblHAL uses what we call an Adaptive Multi-Axis
   Step Smoothing (AMASS) algorithm, which does what the name implies. At lower step frequencies,
   AMASS artificially increases the Bresenham resolution without effecting the algorithm's
   innate exactness. AMASS adapts its resolution levels automatically depending on the step
   frequency to be executed, meaning that for even lower step frequencies the step smoothing
   level increases. Algorithmically, AMASS is achieved by a simple bit-shifting of the Bresenham
   step count for each AMASS level. For example, for a Level 1 step smoothing, we bit shift
   the Bresenham step event count, effectively multiplying it by 2, while the axis step counts
   remain the same, and then double the stepper ISR frequency. In effect, we are allowing the
   non-dominant Bresenham axes step in the intermediate ISR tick, while the dominant axis is
   stepping every two ISR ticks, rather than every ISR tick in the traditional sense. At AMASS
   Level 2, we simply bit-shift again, so the non-dominant Bresenham axes can step within any
   of the four ISR ticks, the dominant axis steps every four ISR ticks, and quadruple the
   stepper ISR frequency. And so on. This, in effect, virtually eliminates multi-axis aliasing
   issues with the Bresenham algorithm and does not significantly alter grblHAL's performance, but
   in fact, more efficiently utilizes unused CPU cycles overall throughout all configurations.
     AMASS retains the Bresenham algorithm exactness by requiring that it always executes a full
   Bresenham step, regardless of AMASS Level. Meaning that for an AMASS Level 2, all four
   intermediate steps must be completed such that baseline Bresenham (Level 0) count is always
   retained. Similarly, AMASS Level 3 means all eight intermediate steps must be executed.
   Although the AMASS Levels are in reality arbitrary, where the baseline Bresenham counts can
   be multiplied by any integer value, multiplication by powers of two are simply used to ease
   CPU overhead with bitshift integer operations.
     This interrupt is simple and dumb by design. All the computational heavy-lifting, as in
   determining accelerations, is performed elsewhere. This interrupt pops pre-computed segments,
   defined as constant velocity over n number of steps, from the step segment buffer and then
   executes them by pulsing the stepper pins appropriately via the Bresenham algorithm. This
   ISR is supported by The Stepper Port Reset Interrupt which it uses to reset the stepper port
   after each pulse. The bresenham line tracer algorithm controls all stepper outputs
   simultaneously with these two interrupts.

   NOTE: This interrupt must be as efficient as possible and complete before the next ISR tick.
   NOTE: This ISR expects at least one step to be executed per segment.
*/

//! \cond

ISR_CODE void ISR_FUNC(stepper_driver_interrupt_handler)(void)
{
#if ENABLE_BACKLASH_COMPENSATION
    static bool backlash_motion;
#endif

    // Start a step pulse when there is a block to execute.
    if(st.exec_block) {

        hal.stepper.pulse_start(&st);

        st.new_block = st.dir_change = false;

        if (st.step_count == 0) // Segment is complete. Discard current segment.
            st.exec_segment = NULL;
    }

    // If there is no step segment, attempt to pop one from the stepper buffer
    if (st.exec_segment == NULL) {
        // Anything in the buffer? If so, load and initialize next step segment.
        if (segment_buffer_tail != segment_buffer_head) {

            // Initialize new step segment and load number of steps to execute
            st.exec_segment = (segment_t *)segment_buffer_tail;

            // Initialize step segment timing per step and load number of steps to execute.
            hal.stepper.cycles_per_tick(st.exec_segment->cycles_per_tick);
            st.step_count = st.exec_segment->n_step; // NOTE: Can sometimes be zero when moving slow.

            // If the new segment starts a new planner block, initialize stepper variables and counters.
            if (st.exec_block != st.exec_segment->exec_block) {

                if((st.dir_change = st.exec_block == NULL || st.dir_outbits.value != st.exec_segment->exec_block->direction_bits.value))
                    st.dir_outbits = st.exec_segment->exec_block->direction_bits;

                if(st.exec_block != NULL && st.exec_block->offset_id != st.exec_segment->exec_block->offset_id)
                    sys.report.wco = sys.report.force_wco = On; // Do not generate grbl.on_rt_reports_added event!

                st.exec_block = st.exec_segment->exec_block;
                st.step_event_count = st.exec_block->step_event_count;
                st.new_block = true;
#if ENABLE_BACKLASH_COMPENSATION
                backlash_motion = st.exec_block->backlash_motion;
#endif

                if(st.exec_block->overrides.sync)
                    sys.override.control = st.exec_block->overrides;

                // Execute output commands to be synchronized with motion
                while(st.exec_block->output_commands) {
                    output_command_t *cmd = st.exec_block->output_commands;
                    cmd->is_executed = true;
                    if(cmd->is_digital)
                        hal.port.digital_out(cmd->port, cmd->value != 0.0f);
                    else
                        hal.port.analog_out(cmd->port, cmd->value);
                    st.exec_block->output_commands = cmd->next;
                }

                // Enqueue any message to be printed (by foreground process)
                if(st.exec_block->message) {
                    if(!protocol_enqueue_foreground_task((foreground_task_ptr)gc_output_message, st.exec_block->message))
                        free(st.exec_block->message);
                    st.exec_block->message = NULL;
                }

                // Initialize Bresenham line and distance counters
                st.counter_x = st.counter_y = st.counter_z
                #ifdef A_AXIS
                  = st.counter_a
                #endif
                #ifdef B_AXIS
                  = st.counter_b
                #endif
                #ifdef C_AXIS
                  = st.counter_c
                #endif
                #ifdef U_AXIS
                  = st.counter_u
                #endif
                #ifdef V_AXIS
                  = st.counter_v
                #endif
                  = st.step_event_count >> 1;

              #ifndef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
                memcpy(st.steps, st.exec_block->steps, sizeof(st.steps));
              #endif
            }

          #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
            // With AMASS enabled, adjust Bresenham axis increment counters according to AMASS level.
            st.amass_level = st.exec_segment->amass_level;
            st.steps[X_AXIS] = st.exec_block->steps[X_AXIS] >> st.amass_level;
            st.steps[Y_AXIS] = st.exec_block->steps[Y_AXIS] >> st.amass_level;
            st.steps[Z_AXIS] = st.exec_block->steps[Z_AXIS] >> st.amass_level;
           #ifdef A_AXIS
            st.steps[A_AXIS] = st.exec_block->steps[A_AXIS] >> st.amass_level;
           #endif
           #ifdef B_AXIS
            st.steps[B_AXIS] = st.exec_block->steps[B_AXIS] >> st.amass_level;
           #endif
           #ifdef C_AXIS
            st.steps[C_AXIS] = st.exec_block->steps[C_AXIS] >> st.amass_level;
           #endif
           #ifdef U_AXIS
            st.steps[U_AXIS] = st.exec_block->steps[U_AXIS] >> st.amass_level;
           #endif
           #ifdef V_AXIS
            st.steps[V_AXIS] = st.exec_block->steps[V_AXIS] >> st.amass_level;
           #endif
         #endif

            if(st.exec_segment->update_pwm)
                st.exec_segment->update_pwm(st.exec_block->spindle, st.exec_segment->spindle_pwm);
            else if(st.exec_segment->update_rpm)
                st.exec_segment->update_rpm(st.exec_block->spindle, st.exec_segment->spindle_rpm);
        } else {
            // Segment buffer empty. Shutdown.
            st_go_idle();

            // Ensure pwm is set properly upon completion of rate-controlled motion.
            if(st.exec_block->dynamic_rpm && st.exec_block->spindle->cap.laser) {
                prep.current_spindle_rpm = 0.0f;
                st.exec_block->spindle->update_pwm(st.exec_block->spindle, st.exec_block->spindle->pwm_off_value);
            }

            st.exec_block = NULL;
            system_set_exec_state_flag(EXEC_CYCLE_COMPLETE); // Flag main program for cycle complete

            return; // Nothing to do but exit.
        }
    }

    // Check probing state.
    // Monitors probe pin state and records the system position when detected.
    // NOTE: This function must be extremely efficient as to not bog down the stepper ISR.
    if (sys.probing_state == Probing_Active && hal.probe.get_state().triggered) {

        sys.probing_state = Probing_Off;
        memcpy(sys.probe_position, sys.position, sizeof(sys.position));
        bit_true(sys.rt_exec_state, EXEC_MOTION_CANCEL);

#ifdef MINIMIZE_PROBE_OVERSHOOT
        // "Flush" segment buffer if full in order to start deceleration early.
        if((probe_asserted = segment_buffer_head->next == segment_buffer_tail)) {
            segment_buffer_head = segment_buffer_tail->next;
            if(st.step_count < 3 || st.step_count < (st.exec_segment->n_step >> 3))
                segment_buffer_head = segment_buffer_head->next;
            segment_next_head = segment_next_head->next;
        }
#endif
    }

    register axes_signals_t step_outbits = (axes_signals_t){0};

    // Execute step displacement profile by Bresenham line algorithm

    st.counter_x += st.steps[X_AXIS];
    if (st.counter_x > st.step_event_count) {
        step_outbits.x = On;
        st.counter_x -= st.step_event_count;
#if ENABLE_BACKLASH_COMPENSATION
        if(!backlash_motion)
#endif
            sys.position[X_AXIS] = sys.position[X_AXIS] + (st.dir_outbits.x ? -1 : 1);
    }

    st.counter_y += st.steps[Y_AXIS];
    if (st.counter_y > st.step_event_count) {
        step_outbits.y = On;
        st.counter_y -= st.step_event_count;
#if ENABLE_BACKLASH_COMPENSATION
        if(!backlash_motion)
#endif
            sys.position[Y_AXIS] = sys.position[Y_AXIS] + (st.dir_outbits.y ? -1 : 1);
    }

    st.counter_z += st.steps[Z_AXIS];
    if (st.counter_z > st.step_event_count) {
        step_outbits.z = On;
        st.counter_z -= st.step_event_count;
#if ENABLE_BACKLASH_COMPENSATION
        if(!backlash_motion)
#endif
            sys.position[Z_AXIS] = sys.position[Z_AXIS] + (st.dir_outbits.z ? -1 : 1);
    }

  #ifdef A_AXIS
      st.counter_a += st.steps[A_AXIS];
      if (st.counter_a > st.step_event_count) {
          step_outbits.a = On;
          st.counter_a -= st.step_event_count;
#if ENABLE_BACKLASH_COMPENSATION
        if(!backlash_motion)
#endif
              sys.position[A_AXIS] = sys.position[A_AXIS] + (st.dir_outbits.a ? -1 : 1);
      }
  #endif

  #ifdef B_AXIS
      st.counter_b += st.steps[B_AXIS];
      if (st.counter_b > st.step_event_count) {
          step_outbits.b = On;
          st.counter_b -= st.step_event_count;
#if ENABLE_BACKLASH_COMPENSATION
        if(!backlash_motion)
#endif
              sys.position[B_AXIS] = sys.position[B_AXIS] + (st.dir_outbits.b ? -1 : 1);
      }
  #endif

  #ifdef C_AXIS
      st.counter_c += st.steps[C_AXIS];
      if (st.counter_c > st.step_event_count) {
          step_outbits.c = On;
          st.counter_c -= st.step_event_count;
#if ENABLE_BACKLASH_COMPENSATION
        if(!backlash_motion)
#endif
              sys.position[C_AXIS] = sys.position[C_AXIS] + (st.dir_outbits.c ? -1 : 1);
      }
  #endif

  #ifdef U_AXIS
    st.counter_u += st.steps[U_AXIS];
    if (st.counter_u > st.step_event_count) {
        step_outbits.u = On;
        st.counter_u -= st.step_event_count;
#if ENABLE_BACKLASH_COMPENSATION
      if(!backlash_motion)
#endif
            sys.position[U_AXIS] = sys.position[U_AXIS] + (st.dir_outbits.u ? -1 : 1);
    }
  #endif

  #ifdef V_AXIS
    st.counter_v += st.steps[V_AXIS];
    if (st.counter_v > st.step_event_count) {
        step_outbits.v = On;
        st.counter_v -= st.step_event_count;
#if ENABLE_BACKLASH_COMPENSATION
      if(!backlash_motion)
#endif
            sys.position[V_AXIS] = sys.position[V_AXIS] + (st.dir_outbits.v ? -1 : 1);
    }
  #endif

    st.step_outbits.value = step_outbits.value;

    // During a homing cycle, lock out and prevent desired axes from moving.
    if (state_get() == STATE_HOMING)
        st.step_outbits.value &= sys.homing_axis_lock.mask;

    if (st.step_count == 0 || --st.step_count == 0) {
        // Segment is complete. Advance segment tail pointer.
        segment_buffer_tail = segment_buffer_tail->next;
    }
}

//! \endcond

// Reset and clear stepper subsystem variables
void st_reset (void)
{
    if(hal.probe.configure)
        hal.probe.configure(false, false);

    // Initialize stepper driver idle state, clear step and direction port pins.
    st_go_idle();
   // hal.stepper.go_idle(true);

    // NOTE: buffer indices starts from 1 for simpler driver coding!

    // Set up stepper block ringbuffer as circular linked list and add id
    uint_fast8_t idx, idx_max = (sizeof(st_block_buffer) / sizeof(st_block_t)) - 1;
    for(idx = 0 ; idx <= idx_max ; idx++) {
        st_block_buffer[idx].next = &st_block_buffer[idx == idx_max ? 0 : idx + 1];
        st_block_buffer[idx].id = idx + 1;
    }

    // Set up segments ringbuffer as circular linked list, add id and clear AMASS level
    idx_max = (sizeof(segment_buffer) / sizeof(segment_t)) - 1;
    for(idx = 0 ; idx <= idx_max ; idx++) {
        segment_buffer[idx].next = &segment_buffer[idx == idx_max ? 0 : idx + 1];
        segment_buffer[idx].id = idx + 1;
        segment_buffer[idx].amass_level = 0;
    }

    st_prep_block = &st_block_buffer[0];

    // Initialize stepper algorithm variables.
    pl_block = NULL;  // Planner block pointer used by segment buffer
    segment_buffer_tail = segment_buffer_head = &segment_buffer[0]; // empty = tail
    segment_next_head = segment_buffer_head->next;

    memset(&prep, 0, sizeof(st_prep_t));
    memset(&st, 0, sizeof(stepper_t));

#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    // TODO: move to driver?
    // AMASS_LEVEL0: Normal operation. No AMASS. No upper cutoff frequency. Starts at LEVEL1 cutoff frequency.
    // Defined as step timer frequency / Cutoff frequency in Hz
    amass.level_1 = hal.f_step_timer / 8000;
    amass.level_2 = hal.f_step_timer / 4000;
    amass.level_3 = hal.f_step_timer / 2000;
#endif

    cycles_per_min = (float)hal.f_step_timer * 60.0f;
}

// Called by spindle_set_state() to inform about RPM changes.
// Used by st_prep_buffer() to determine if spindle needs update when dynamic RPM is called for.
void st_rpm_changed (float rpm)
{
    prep.current_spindle_rpm = rpm;
}

// Called by planner_recalculate() when the executing block is updated by the new plan.
void st_update_plan_block_parameters (void)
{
    if (pl_block != NULL) { // Ignore if at start of a new block.
        prep.recalculate.velocity_profile = On;
        pl_block->entry_speed_sqr = prep.current_speed * prep.current_speed; // Update entry speed.
        pl_block = NULL; // Flag st_prep_segment() to load and check active velocity profile.
    }
}

// Changes the run state of the step segment buffer to execute the special parking motion.
void st_parking_setup_buffer (void)
{
    // Store step execution data of partially completed block, if necessary.
    if (prep.recalculate.hold_partial_block && !prep.recalculate.parking) {
        prep.last_st_block = st_prep_block;
        memcpy(&st_hold_block, st_prep_block, sizeof(st_block_t));
        prep.last_steps_remaining = prep.steps_remaining;
        prep.last_dt_remainder = prep.dt_remainder;
        prep.last_steps_per_mm = prep.steps_per_mm;
    }
    // Set flags to execute a parking motion
    prep.recalculate.parking = On;
    prep.recalculate.velocity_profile = Off;
    pl_block = NULL; // Always reset parking motion to reload new block.
}


// Restores the step segment buffer to the normal run state after a parking motion.
void st_parking_restore_buffer (void)
{
    // Restore step execution data and flags of partially completed block, if necessary.
    if (prep.recalculate.hold_partial_block) {
        memcpy(prep.last_st_block, &st_hold_block, sizeof(st_block_t));
        st_prep_block = prep.last_st_block;
        prep.steps_remaining = prep.last_steps_remaining;
        prep.dt_remainder = prep.last_dt_remainder;
        prep.steps_per_mm = prep.last_steps_per_mm;
        prep.recalculate.flags = 0;
        prep.recalculate.hold_partial_block = prep.recalculate.velocity_profile = On;
        prep.req_mm_increment = REQ_MM_INCREMENT_SCALAR / prep.steps_per_mm; // Recompute this value.
    } else
        prep.recalculate.flags = 0;

    pl_block = NULL; // Set to reload next block.
}

/* Prepares step segment buffer. Continuously called from main program.

   The segment buffer is an intermediary buffer interface between the execution of steps
   by the stepper algorithm and the velocity profiles generated by the planner. The stepper
   algorithm only executes steps within the segment buffer and is filled by the main program
   when steps are "checked-out" from the first block in the planner buffer. This keeps the
   step execution and planning optimization processes atomic and protected from each other.
   The number of steps "checked-out" from the planner buffer and the number of segments in
   the segment buffer is sized and computed such that no operation in the main program takes
   longer than the time it takes the stepper algorithm to empty it before refilling it.
   Currently, the segment buffer conservatively holds roughly up to 40-50 msec of steps.
   NOTE: Computation units are in steps, millimeters, and minutes.
*/
void st_prep_buffer (void)
{
    // Block step prep buffer, while in a suspend state and there is no suspend motion to execute.
    if (sys.step_control.end_motion)
        return;

    while (segment_buffer_tail != segment_next_head) { // Check if we need to fill the buffer.

        // Determine if we need to load a new planner block or if the block needs to be recomputed.
        if (pl_block == NULL) {

            // Query planner for a queued block

            pl_block = sys.step_control.execute_sys_motion ? plan_get_system_motion_block() : plan_get_current_block();

            if (pl_block == NULL)
                return; // No planner blocks. Exit.

            // Check if we need to only recompute the velocity profile or load a new block.
            if (prep.recalculate.velocity_profile) {
                if(settings.parking.flags.enabled) {
                    if (prep.recalculate.parking)
                        prep.recalculate.velocity_profile = Off;
                    else
                        prep.recalculate.flags = 0;
                } else
                    prep.recalculate.flags = 0;
            } else {

                // Prepare and copy Bresenham algorithm segment data from the new planner block, so that
                // when the segment buffer completes the planner block, it may be discarded when the
                // segment buffer finishes the prepped block, but the stepper ISR is still executing it.

                st_prep_block = st_prep_block->next;

                uint_fast8_t idx = N_AXIS;
              #ifndef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
                do {
                    idx--;
                    st_prep_block->steps[idx] = (pl_block->steps[idx] << 1);
                } while(idx);
                st_prep_block->step_event_count = (pl_block->step_event_count << 1);
              #else
                // With AMASS enabled, simply bit-shift multiply all Bresenham data by the max AMASS
                // level, such that we never divide beyond the original data anywhere in the algorithm.
                // If the original data is divided, we can lose a step from integer roundoff.
                do {
                    idx--;
                    st_prep_block->steps[idx] = pl_block->steps[idx] << MAX_AMASS_LEVEL;
                } while(idx);
                st_prep_block->step_event_count = pl_block->step_event_count << MAX_AMASS_LEVEL;
              #endif

                st_prep_block->direction_bits = pl_block->direction_bits;
                st_prep_block->programmed_rate = pl_block->programmed_rate;
//                st_prep_block->r = pl_block->programmed_rate;
                st_prep_block->millimeters = pl_block->millimeters;
                st_prep_block->steps_per_mm = (float)pl_block->step_event_count / pl_block->millimeters;
                st_prep_block->spindle = pl_block->spindle.hal;
                st_prep_block->output_commands = pl_block->output_commands;
                st_prep_block->overrides = pl_block->overrides;
                st_prep_block->offset_id = pl_block->offset_id;
                st_prep_block->backlash_motion = pl_block->condition.backlash_motion;
                st_prep_block->message = pl_block->message;
                pl_block->message = NULL;

                // Initialize segment buffer data for generating the segments.
                prep.steps_per_mm = st_prep_block->steps_per_mm;
                prep.steps_remaining = pl_block->step_event_count;
                prep.req_mm_increment = REQ_MM_INCREMENT_SCALAR / prep.steps_per_mm;
                prep.dt_remainder = prep.target_position = 0.0f; // Reset for new segment block
#ifdef KINEMATICS_API
                prep.rate_multiplier = pl_block->rate_multiplier;
#endif
                if (sys.step_control.execute_hold || prep.recalculate.decel_override) {
                    // New block loaded mid-hold. Override planner block entry speed to enforce deceleration.
                    prep.current_speed = prep.exit_speed;
                    pl_block->entry_speed_sqr = prep.exit_speed * prep.exit_speed;
                    prep.recalculate.decel_override = Off;
                } else
                    prep.current_speed = sqrtf(pl_block->entry_speed_sqr);

                // Setup laser mode variables. RPM rate adjusted motions will always complete a motion with the
                // spindle off.
                if ((st_prep_block->dynamic_rpm = pl_block->condition.is_rpm_rate_adjusted)) {
                    // Pre-compute inverse programmed rate to speed up RPM updating per step segment.
                    prep.inv_feedrate = pl_block->condition.is_laser_ppi_mode ? 1.0f : 1.0f / pl_block->programmed_rate;
                } else
                    st_prep_block->dynamic_rpm = !!pl_block->spindle.css;
            }

            /* ---------------------------------------------------------------------------------
             Compute the velocity profile of a new planner block based on its entry and exit
             speeds, or recompute the profile of a partially-completed planner block if the
             planner has updated it. For a commanded forced-deceleration, such as from a feed
             hold, override the planner velocities and decelerate to the target exit speed.
            */
            prep.mm_complete = 0.0f; // Default velocity profile complete at 0.0mm from end of block.
            float inv_2_accel = 0.5f / pl_block->acceleration;

            if (sys.step_control.execute_hold) { // [Forced Deceleration to Zero Velocity]
                // Compute velocity profile parameters for a feed hold in-progress. This profile overrides
                // the planner block profile, enforcing a deceleration to zero speed.
                prep.ramp_type = Ramp_Decel;
                // Compute decelerate distance relative to end of block.
                float decel_dist = pl_block->millimeters - inv_2_accel * pl_block->entry_speed_sqr;
                if(decel_dist < -0.0001f) {
                    // Deceleration through entire planner block. End of feed hold is not in this block.
                    prep.exit_speed = sqrtf(pl_block->entry_speed_sqr - 2.0f * pl_block->acceleration * pl_block->millimeters);
                } else {
                    prep.mm_complete = decel_dist < 0.0001f ? 0.0f : decel_dist; // End of feed hold.
                    prep.exit_speed = 0.0f;
                }
            } else { // [Normal Operation]
                // Compute or recompute velocity profile parameters of the prepped planner block.
                prep.ramp_type = Ramp_Accel; // Initialize as acceleration ramp.
                prep.accelerate_until = pl_block->millimeters;

                float exit_speed_sqr;
                if (sys.step_control.execute_sys_motion)
                    prep.exit_speed = exit_speed_sqr = 0.0f; // Enforce stop at end of system motion.
                else {
                    exit_speed_sqr = plan_get_exec_block_exit_speed_sqr();
                    prep.exit_speed = sqrtf(exit_speed_sqr);
                }

                float nominal_speed = plan_compute_profile_nominal_speed(pl_block);
                float nominal_speed_sqr = nominal_speed * nominal_speed;
                float intersect_distance = 0.5f * (pl_block->millimeters + inv_2_accel * (pl_block->entry_speed_sqr - exit_speed_sqr));

                prep.target_feed = nominal_speed;

                if (pl_block->entry_speed_sqr > nominal_speed_sqr) { // Only occurs during override reductions.

                    prep.accelerate_until = pl_block->millimeters - inv_2_accel * (pl_block->entry_speed_sqr - nominal_speed_sqr);

                    if (prep.accelerate_until <= 0.0f) { // Deceleration-only.
                        prep.ramp_type = Ramp_Decel;
                        // prep.decelerate_after = pl_block->millimeters;
                        // prep.maximum_speed = prep.current_speed;

                        // Compute override block exit speed since it doesn't match the planner exit speed.
                        prep.exit_speed = sqrtf(pl_block->entry_speed_sqr - 2.0f * pl_block->acceleration * pl_block->millimeters);
                        prep.recalculate.decel_override = On; // Flag to load next block as deceleration override.

                        // TODO: Determine correct handling of parameters in deceleration-only.
                        // Can be tricky since entry speed will be current speed, as in feed holds.
                        // Also, look into near-zero speed handling issues with this.

                    } else {
                        // Decelerate to cruise or cruise-decelerate types. Guaranteed to intersect updated plan.
                        prep.decelerate_after = inv_2_accel * (nominal_speed_sqr - exit_speed_sqr); // Should always be >= 0.0 due to planner reinit.
                        prep.maximum_speed = nominal_speed;
                        prep.ramp_type = Ramp_DecelOverride;
                    }
                } else if (intersect_distance > 0.0f) {
                    if (intersect_distance < pl_block->millimeters) { // Either trapezoid or triangle types
                        // NOTE: For acceleration-cruise and cruise-only types, following calculation will be 0.0.
                        prep.decelerate_after = inv_2_accel * (nominal_speed_sqr - exit_speed_sqr);
                        if (prep.decelerate_after < intersect_distance) { // Trapezoid type
                            prep.maximum_speed = nominal_speed;
                            if (pl_block->entry_speed_sqr == nominal_speed_sqr) {
                                // Cruise-deceleration or cruise-only type.
                                prep.ramp_type = Ramp_Cruise;
                            } else {
                                // Full-trapezoid or acceleration-cruise types
                                prep.accelerate_until -= inv_2_accel * (nominal_speed_sqr - pl_block->entry_speed_sqr);
                            }
                        } else { // Triangle type
                            prep.accelerate_until = prep.decelerate_after = intersect_distance;
                            prep.maximum_speed = sqrtf(2.0f * pl_block->acceleration * intersect_distance + exit_speed_sqr);
                        }
                    } else { // Deceleration-only type
                        prep.ramp_type = Ramp_Decel;
                        // prep.decelerate_after = pl_block->millimeters;
                        // prep.maximum_speed = prep.current_speed;
                    }
                } else { // Acceleration-only type
                    prep.accelerate_until = 0.0f;
                    // prep.decelerate_after = 0.0f;
                    prep.maximum_speed = prep.exit_speed;
                }
            }

            if(state_get() != STATE_HOMING)
                sys.step_control.update_spindle_rpm |= pl_block->spindle.hal->cap.laser; // Force update whenever updating block in laser mode.

            probe_asserted = false;
        }

        // Block adding new segments after probe is asserted until deceleration is started.
        if(probe_asserted)
            return;

        // Initialize new segment
        segment_t *prep_segment = segment_buffer_head;

        // Set new segment to point to the current segment data block.
        prep_segment->exec_block = st_prep_block;
        prep_segment->update_rpm = NULL;
        prep_segment->update_pwm = NULL;

        /*------------------------------------------------------------------------------------
            Compute the average velocity of this new segment by determining the total distance
          traveled over the segment time DT_SEGMENT. The following code first attempts to create
          a full segment based on the current ramp conditions. If the segment time is incomplete
          when terminating at a ramp state change, the code will continue to loop through the
          progressing ramp states to fill the remaining segment execution time. However, if
          an incomplete segment terminates at the end of the velocity profile, the segment is
          considered completed despite having a truncated execution time less than DT_SEGMENT.
            The velocity profile is always assumed to progress through the ramp sequence:
          acceleration ramp, cruising state, and deceleration ramp. Each ramp's travel distance
          may range from zero to the length of the block. Velocity profiles can end either at
          the end of planner block (typical) or mid-block at the end of a forced deceleration,
          such as from a feed hold.
        */
        float dt_max = DT_SEGMENT; // Maximum segment time
        float dt = 0.0f; // Initialize segment time
        float time_var = dt_max; // Time worker variable
#if ENABLE_JERK_ACCELERATION   
        float last_segment_accel = 0.0f; // Acceleration value of last computed segment. Initialize as 0.0
#endif
        float mm_var; // mm - Distance worker variable
        float speed_var; // Speed worker variable
        float mm_remaining = pl_block->millimeters; // New segment distance from end of block.
        float minimum_mm = mm_remaining - prep.req_mm_increment; // Guarantee at least one step.

        if (minimum_mm < 0.0f)
            minimum_mm = 0.0f;

        do {

            switch (prep.ramp_type) {

                case Ramp_DecelOverride:
                    speed_var = pl_block->acceleration * time_var;
                    if ((prep.current_speed - prep.maximum_speed) <= speed_var) {
                        // Cruise or cruise-deceleration types only for deceleration override.
                        mm_remaining = prep.accelerate_until;
                        time_var = 2.0f * (pl_block->millimeters - mm_remaining) / (prep.current_speed + prep.maximum_speed);
                        prep.ramp_type = Ramp_Cruise;
                        prep.current_speed = prep.maximum_speed;
                    } else {// Mid-deceleration override ramp.
                        mm_remaining -= time_var * (prep.current_speed - 0.5f * speed_var);
                        prep.current_speed -= speed_var;
                    }
                    break;

                case Ramp_Accel:
                    // NOTE: Acceleration ramp only computes during first do-while loop.
#if ENABLE_JERK_ACCELERATION   
                    if (((mm_remaining - prep.accelerate_until) / (prep.current_speed + 0.001f)) <= (last_segment_accel / pl_block->jerk)) { 
                        //+0.001f to avoid divide by 0 speed, minor effect on jerk ramp (+1.0f was too large for low jerk values)
                        // Check if we are on ramp up or ramp down. Ramp down if time to end of acceleration is less than time needed to reach 0 acceleration.
                        // Then limit acceleration change by jerk up to max acceleration and update for next segment.
                        // Minimum acceleration jerk per time_var to ensure acceleartion completes. Acceleration change at end of ramp is in acceptable jerk range.
                        last_segment_accel = max(last_segment_accel - pl_block->jerk * time_var, pl_block->jerk * time_var); 
                    } else {
                        last_segment_accel = min(last_segment_accel + pl_block->jerk * time_var, pl_block->max_acceleration); 
                    }
                    speed_var = last_segment_accel * time_var;
#else
                    speed_var = pl_block->acceleration * time_var;
#endif
                    mm_remaining -= time_var * (prep.current_speed + 0.5f * speed_var);
                    if (mm_remaining < prep.accelerate_until) { // End of acceleration ramp.
                        // Acceleration-cruise, acceleration-deceleration ramp junction, or end of block.
                        mm_remaining = prep.accelerate_until; // NOTE: 0.0 at EOB
                        time_var = 2.0f * (pl_block->millimeters - mm_remaining) / (prep.current_speed + prep.maximum_speed);
                        prep.ramp_type = mm_remaining == prep.decelerate_after ? Ramp_Decel : Ramp_Cruise;
                        prep.current_speed = prep.maximum_speed;
#if ENABLE_JERK_ACCELERATION
                        last_segment_accel = 0.0f;  // reset acceleration variable to 0 for next accel ramp
#endif
                    } else // Acceleration only.
                        prep.current_speed += speed_var;
                    break;

                case Ramp_Cruise:
                    // NOTE: mm_var used to retain the last mm_remaining for incomplete segment time_var calculations.
                    // NOTE: If maximum_speed*time_var value is too low, round-off can cause mm_var to not change. To
                    //   prevent this, simply enforce a minimum speed threshold in the planner.
                    mm_var = mm_remaining - prep.maximum_speed * time_var;
                    if (mm_var < prep.decelerate_after) { // End of cruise.
                        // Cruise-deceleration junction or end of block.
                        time_var = (mm_remaining - prep.decelerate_after) / prep.maximum_speed;
                        mm_remaining = prep.decelerate_after; // NOTE: 0.0 at EOB
                        prep.ramp_type = Ramp_Decel;
                    } else // Cruising only.
                        mm_remaining = mm_var;
                    break;

                default: // case Ramp_Decel:
                    // NOTE: mm_var used as a misc worker variable to prevent errors when near zero speed.
#if ENABLE_JERK_ACCELERATION   
                    if ((mm_remaining / (prep.current_speed + 0.001f)) <= (last_segment_accel / pl_block->jerk)) { 
                        //+0.001f to avoid divide by 0 speed, minor effect on jerk ramp (+1.0f was too large for low jerk values)
                        // Check if we are on ramp up or ramp down. Ramp down if time to end of deceleration is less than time needed to reach 0 acceleration.
                        // Then limit acceleration change by jerk up to max acceleration and update for next segment.
                        // Minimum acceleration of jerk per time_var to ensure acceleration completes. Acceleration change at end of ramp is in acceptable jerk range.
                        last_segment_accel = max(last_segment_accel - pl_block->jerk * time_var, pl_block->jerk * time_var); 
                    } else {
                        last_segment_accel = min(last_segment_accel + pl_block->jerk * time_var, pl_block->max_acceleration); 
                    }
                    speed_var = last_segment_accel * time_var; // Used as delta speed (mm/min)
#else
                    speed_var = pl_block->acceleration * time_var; // Used as delta speed (mm/min)
#endif
                    if (prep.current_speed > speed_var) { // Check if at or below zero speed.
                        // Compute distance from end of segment to end of block.
                        mm_var = mm_remaining - time_var * (prep.current_speed - 0.5f * speed_var); // (mm)
                        if (mm_var > prep.mm_complete) { // Typical case. In deceleration ramp.
                            mm_remaining = mm_var;
                            prep.current_speed -= speed_var;
                            break; // Segment complete. Exit switch-case statement. Continue do-while loop.
                        }
                    }
                    // Otherwise, at end of block or end of forced-deceleration.
                    time_var = 2.0f * (mm_remaining - prep.mm_complete) / (prep.current_speed + prep.exit_speed);
                    mm_remaining = prep.mm_complete;
                    prep.current_speed = prep.exit_speed;
            }

            dt += time_var; // Add computed ramp time to total segment time.

            if (dt < dt_max)
                time_var = dt_max - dt;// **Incomplete** At ramp junction.
            else {
                if (mm_remaining > minimum_mm) { // Check for very slow segments with zero steps.
                    // Increase segment time to ensure at least one step in segment. Override and loop
                    // through distance calculations until minimum_mm or mm_complete.
                    dt_max += DT_SEGMENT;
                    time_var = dt_max - dt;
                } else
                    break; // **Complete** Exit loop. Segment execution time maxed.
            }

        } while (mm_remaining > prep.mm_complete); // **Complete** Exit loop. Profile complete.

        /* -----------------------------------------------------------------------------------
           Compute spindle spindle speed for step segment
        */

        if (sys.step_control.update_spindle_rpm || st_prep_block->dynamic_rpm) {

            float rpm;

            if (pl_block->spindle.state.on) {
                if(pl_block->spindle.css) {
                    float npos = (float)(pl_block->step_event_count - prep.steps_remaining) / (float)pl_block->step_event_count;
                    rpm = spindle_set_rpm(pl_block->spindle.hal,
                                           pl_block->spindle.rpm + pl_block->spindle.css->delta_rpm * npos,
                                            pl_block->spindle.hal->param->override_pct);
                } else {
                    // NOTE: Feed and rapid overrides are independent of PWM value and do not alter laser power/rate.
                    // If current_speed is zero, then may need to be rpm_min*(100/MAX_SPINDLE_RPM_OVERRIDE)
                    // but this would be instantaneous only and during a motion. May not matter at all.
                    rpm = spindle_set_rpm(pl_block->spindle.hal,
                                           pl_block->condition.is_rpm_rate_adjusted && !pl_block->condition.is_laser_ppi_mode
                                            ? pl_block->spindle.rpm * prep.current_speed * prep.inv_feedrate
                                            : pl_block->spindle.rpm, pl_block->spindle.hal->param->override_pct);
                }
            } else
                pl_block->spindle.hal->param->rpm = rpm = 0.0f;

            if(rpm != prep.current_spindle_rpm) {
                if(pl_block->spindle.hal->get_pwm != NULL) {
                    prep.current_spindle_rpm = rpm;
                    prep_segment->update_pwm = pl_block->spindle.hal->update_pwm;
                    prep_segment->spindle_pwm = pl_block->spindle.hal->get_pwm(pl_block->spindle.hal, rpm);
                } else {
                    prep_segment->update_rpm = pl_block->spindle.hal->update_rpm;
                    prep.current_spindle_rpm = prep_segment->spindle_rpm = rpm;
                }
                sys.step_control.update_spindle_rpm = Off;
            }
        }

        /* -----------------------------------------------------------------------------------
           Compute segment step rate, steps to execute, and apply necessary rate corrections.
           NOTE: Steps are computed by direct scalar conversion of the millimeter distance
           remaining in the block, rather than incrementally tallying the steps executed per
           segment. This helps in removing floating point round-off issues of several additions.
           However, since floats have only 7.2 significant digits, long moves with extremely
           high step counts can exceed the precision of floats, which can lead to lost steps.
           Fortunately, this scenario is highly unlikely and unrealistic in CNC machines
           supported by grblHAL (i.e. exceeding 10 meters axis travel at 200 step/mm).
        */
        float step_dist_remaining = prep.steps_per_mm * mm_remaining; // Convert mm_remaining to steps
        uint32_t n_steps_remaining = (uint32_t)ceilf(step_dist_remaining); // Round-up current steps remaining

        prep_segment->n_step = (uint_fast16_t)(prep.steps_remaining - n_steps_remaining); // Compute number of steps to execute.

        // Bail if we are at the end of a feed hold and don't have a step to execute.
        if (prep_segment->n_step == 0 && sys.step_control.execute_hold) {
            // Less than one step to decelerate to zero speed, but already very close. AMASS
            // requires full steps to execute. So, just bail.
            sys.step_control.end_motion = On;
            if (settings.parking.flags.enabled && !prep.recalculate.parking)
                prep.recalculate.hold_partial_block = On;
            return; // Segment not generated, but current step data still retained.
        }

        // Compute segment step rate. Since steps are integers and mm distances traveled are not,
        // the end of every segment can have a partial step of varying magnitudes that are not
        // executed, because the stepper ISR requires whole steps due to the AMASS algorithm. To
        // compensate, we track the time to execute the previous segment's partial step and simply
        // apply it with the partial step distance to the current segment, so that it minutely
        // adjusts the whole segment rate to keep step output exact. These rate adjustments are
        // typically very small and do not adversely effect performance, but ensures that grblHAL
        // outputs the exact acceleration and velocity profiles as computed by the planner.
        dt += prep.dt_remainder; // Apply previous segment partial step execute time
        float inv_rate = dt / ((float)prep.steps_remaining - step_dist_remaining); // Compute adjusted step rate inverse

        // Compute timer ticks per step for the prepped segment.
        uint32_t cycles = (uint32_t)ceilf(cycles_per_min * inv_rate); // (cycles/step)

        // Record end position of segment relative to block if spindle synchronized motion
        if((prep_segment->spindle_sync = pl_block->spindle.state.synchronized)) {
            prep.target_position += dt * prep.target_feed;
            prep_segment->cruising = prep.ramp_type == Ramp_Cruise;
            prep_segment->target_position = prep.target_position; //st_prep_block->millimeters - pl_block->millimeters;
        }

      #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
        // Compute step timing and multi-axis smoothing level.
        // NOTE: AMASS overdrives the timer with each level, so only one prescalar is required.
        if (cycles < amass.level_1)
            prep_segment->amass_level = 0;
        else {
            prep_segment->amass_level = cycles < amass.level_2 ? 1 : (cycles < amass.level_3 ? 2 : 3);
            cycles >>= prep_segment->amass_level;
            prep_segment->n_step <<= prep_segment->amass_level;
        }
      #endif

        prep_segment->cycles_per_tick = cycles;
        prep_segment->current_rate = prep.current_speed;

        // Segment complete! Increment segment pointers, so stepper ISR can immediately execute it.
        segment_buffer_head = segment_next_head;
        segment_next_head = segment_next_head->next;

        // Update the appropriate planner and segment data.
        pl_block->millimeters = mm_remaining;
        prep.steps_remaining = n_steps_remaining;
        prep.dt_remainder = ((float)n_steps_remaining - step_dist_remaining) * inv_rate;

        // Check for exit conditions and flag to load next planner block.
        if (mm_remaining <= prep.mm_complete) {

            // End of planner block or forced-termination. No more distance to be executed.
            if (mm_remaining > 0.0f) { // At end of forced-termination.
                // Reset prep parameters for resuming and then bail. Allow the stepper ISR to complete
                // the segment queue, where realtime protocol will set new state upon receiving the
                // cycle stop flag from the ISR. Prep_segment is blocked until then.
                sys.step_control.end_motion = On;
                if (settings.parking.flags.enabled && !prep.recalculate.parking)
                    prep.recalculate.hold_partial_block = On;
                return; // Bail!
            } else { // End of planner block
                // The planner block is complete. All steps are set to be executed in the segment buffer.
                if (sys.step_control.execute_sys_motion) {
                    sys.step_control.end_motion = On;
                    return;
                }
                pl_block = NULL; // Set pointer to indicate check and load next planner block.
                plan_discard_current_block();
            }
        }
    }
}


// Called by realtime status reporting to fetch the current speed being executed. This value
// however is not exactly the current speed, but the speed computed in the last step segment
// in the segment buffer. It will always be behind by up to the number of segment blocks (-1)
// divided by the ACCELERATION TICKS PER SECOND in seconds.
float st_get_realtime_rate (void)
{
    return state_get() & (STATE_CYCLE|STATE_HOMING|STATE_HOLD|STATE_JOG|STATE_SAFETY_DOOR)
#ifdef KINEMATICS_API
            ? prep.current_speed * prep.rate_multiplier
#else
            ? prep.current_speed
#endif
            : 0.0f;
}

offset_id_t st_get_offset_id (void)
{
    plan_block_t *pl_block;

    return st.exec_block
            ? st.exec_block->offset_id
            : (sys.holding_state == Hold_Complete && (pl_block = plan_get_current_block())
                ? pl_block->offset_id
                : -1);
}
