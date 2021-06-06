/*
  stepper.h - stepper motor driver: executes motion plans of planner.c using the stepper motors

  Part of grblHAL

  Copyright (c) 2019-2020 Terje Io
  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

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

#include "planner.h"

#ifndef _STEPPER_H_
#define _STEPPER_H_

#ifndef SEGMENT_BUFFER_SIZE
#define SEGMENT_BUFFER_SIZE 10
#endif

typedef enum {
    SquaringMode_Both = 0,
    SquaringMode_A,
    SquaringMode_B,
} squaring_mode_t;

// Holds the planner block Bresenham algorithm execution data for the segments in the segment buffer.
// NOTE: This data is copied from the prepped planner blocks so that the planner blocks may be
// discarded when entirely consumed and completed by the segment buffer. Also, AMASS alters this
// data for its own use.
typedef struct st_block {
    uint_fast8_t id;                  // Id may be used by driver to track changes
    struct st_block *next;            // Pointer to next element in cirular list of blocks
    uint32_t steps[N_AXIS];
    uint32_t step_event_count;
    axes_signals_t direction_bits;
    gc_override_flags_t overrides;    // Block bitfield variable for overrides
    float steps_per_mm;
    float millimeters;
    float programmed_rate;
    char *message;                     // Message to be displayed when block is executed
    output_command_t *output_commands; // Output commands (linked list) to be performed when block is executed
    bool dynamic_rpm;                  // Tracks motions that require dynamic RPM adjustment
    bool backlash_motion;
} st_block_t;

typedef struct st_segment {
    uint_fast8_t id;                // Id may be used by driver to track changes
    struct st_segment *next;        // Pointer to next element in cirular list of segments
    st_block_t *exec_block;         // Pointer to the block data for the segment
    uint32_t cycles_per_tick;       // Step distance traveled per ISR tick, aka step rate.
    float current_rate;
    float target_position;          // Target position of segment relative to block start, used by spindle sync code
    uint_fast16_t n_step;           // Number of step events to be executed for this segment
#ifdef SPINDLE_PWM_DIRECT
    uint_fast16_t spindle_pwm;      // Spindle PWM to be set at the start of segment execution
#else
    float spindle_rpm;              // Spindle RPM to be set at the start of the segment execution
#endif
    bool update_rpm;                // True if set spindle speed at the start of the segment execution
    bool spindle_sync;              // True if block is spindle synchronized
    bool cruising;                  // True when in cruising part of profile, only set for spindle synced moves
    uint_fast8_t amass_level;       // Indicates AMASS level for the ISR to execute this segment
} segment_t;

// Stepper ISR data struct. Contains the running data for the main stepper ISR.
typedef struct {
    // Used by the bresenham line algorithm
    uint32_t counter_x,        // Counter variables for the bresenham line tracer
             counter_y,
             counter_z
    #ifdef A_AXIS
           , counter_a
    #endif
    #ifdef B_AXIS
          , counter_b
    #endif
    #ifdef C_AXIS
          , counter_c
    #endif
    #ifdef C_AXIS
          , counter_u
    #endif
    #ifdef C_AXIS
          , counter_v
    #endif
;
    bool new_block;                 // Set to true when a new block is started, might be used by driver for advanced functionality
    bool dir_change;                // Set to true on direction changes, might be used by driver for advanced functionality
    axes_signals_t step_outbits;    // The next stepping-bits to be output
    axes_signals_t dir_outbits;     // The next direction-bits to be output
    uint32_t steps[N_AXIS];
    uint_fast8_t amass_level;       // AMASS level for this segment
//    uint_fast16_t spindle_pwm;
    uint_fast16_t step_count;       // Steps remaining in line segment motion
    uint32_t step_event_count;
    st_block_t *exec_block;         // Pointer to the block data for the segment being executed
    segment_t *exec_segment;        // Pointer to the segment being executed
} stepper_t;

// Initialize and setup the stepper motor subsystem
void stepper_init();

// Enable steppers, but cycle does not start unless called by motion control or realtime command.
void st_wake_up();

// Immediately disables steppers
void st_go_idle();

// Reset the stepper subsystem variables
void st_reset();

// Called by spindle_set_state() to inform about RPM changes.
void st_rpm_changed(float rpm);

// Changes the run state of the step segment buffer to execute the special parking motion.
void st_parking_setup_buffer();

// Restores the step segment buffer to the normal run state after a parking motion.
void st_parking_restore_buffer();

// Reloads step segment buffer. Called continuously by realtime execution system.
void st_prep_buffer();

// Called by planner_recalculate() when the executing block is updated by the new plan.
void st_update_plan_block_parameters();

// Called by realtime status reporting if realtime rate reporting is enabled in config.h.
float st_get_realtime_rate();

void stepper_driver_interrupt_handler (void);

#endif
