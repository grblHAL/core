/*
  planner.h - buffers movement commands and manages the acceleration profile plan

  Part of grblHAL

  Copyright (c) 2019-2021 Terje Io
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

#ifndef _PLANNER_H_
#define _PLANNER_H_

// The number of linear motions that can be in the plan at any give time
#ifndef BLOCK_BUFFER_SIZE
  #define BLOCK_BUFFER_SIZE 36
#endif

typedef union {
    uint32_t value;
    struct {
        uint16_t rapid_motion         :1,
                 system_motion        :1,
                 jog_motion           :1,
                 backlash_motion      :1,
                 no_feed_override     :1,
                 inverse_time         :1,
                 is_rpm_rate_adjusted :1,
                 is_rpm_pos_adjusted  :1,
                 is_laser_ppi_mode    :1,
                 unassigned           :7;
        spindle_state_t spindle;
        coolant_state_t coolant;
    };
} planner_cond_t;

// This struct stores a linear movement of a g-code block motion with its critical "nominal" values
// are as specified in the source g-code.
typedef struct plan_block {
    // Fields used by the bresenham algorithm for tracing the line
    // NOTE: Used by stepper algorithm to execute the block correctly. Do not alter these values.
    uint32_t steps[N_AXIS];         // Step count along each axis
    uint32_t step_event_count;      // The maximum step axis count and number of steps required to complete this block.
    axes_signals_t direction_bits;  // The direction bit set for this block (refers to *_DIRECTION_PIN in config.h)

    // Block condition data to ensure correct execution depending on states and overrides.
    planner_cond_t condition;       // Block bitfield variable defining block run conditions. Copied from pl_line_data.
    gc_override_flags_t overrides;  // Block bitfield variable for overrides
    int32_t line_number;            // Block line number for real-time reporting. Copied from pl_line_data.

    // Fields used by the motion planner to manage acceleration. Some of these values may be updated
    // by the stepper module during execution of special motion cases for replanning purposes.
    float entry_speed_sqr;      // The current planned entry speed at block junction in (mm/min)^2
    float max_entry_speed_sqr;  // Maximum allowable entry speed based on the minimum of junction limit and
                                // neighboring nominal speeds with overrides in (mm/min)^2
    float acceleration;         // Axis-limit adjusted line acceleration in (mm/min^2). Does not change.
    float millimeters;          // The remaining distance for this block to be executed in (mm).
                                // NOTE: This value may be altered by stepper algorithm during execution.

    // Stored rate limiting data used by planner when changes occur.
    float max_junction_speed_sqr; // Junction entry speed limit based on direction vectors in (mm/min)^2
    float rapid_rate;             // Axis-limit adjusted maximum rate for this block direction in (mm/min)
    float programmed_rate;        // Programmed rate of this block (mm/min).

    // Stored spindle speed data used by spindle overrides and resuming methods.
    spindle_t spindle;    // Block spindle speed. Copied from pl_line_data.

    char *message;                // Message to be displayed when block is executed.
    output_command_t *output_commands;
    struct plan_block *prev, *next; // Linked list pointers, DO NOT MOVE - these MUST be the last elements in the struct!
} plan_block_t;


// Planner data prototype. Must be used when passing new motions to the planner.
typedef struct {
    float feed_rate;                // Desired feed rate for line motion. Value is ignored, if rapid motion.
    //  float blending_tolerance;   // Motion blending tolerance
    spindle_t spindle;              // Desired spindle speed through line motion.
    planner_cond_t condition;       // Bitfield variable to indicate planner conditions. See defines above.
    gc_override_flags_t overrides;  // Block bitfield variable for overrides
    int32_t line_number;            // Desired line number to report when executing.
//    void *parameters;               // TODO: pointer to extra parameters, for canned cycles and threading?
    char *message;                  // Message to be displayed when block is executed.
    output_command_t *output_commands;
} plan_line_data_t;


// Define planner variables
typedef struct {
  int32_t position[N_AXIS];         // The planner position of the tool in absolute steps. Kept separate
                                    // from g-code position for movements requiring multiple line motions,
                                    // i.e. arcs, canned cycles, and backlash compensation.
  float previous_unit_vec[N_AXIS];  // Unit vector of previous path line segment
  float previous_nominal_speed;     // Nominal speed of previous path line segment
} planner_t;

// Initialize and reset the motion plan subsystem
void plan_reset(); // Reset all
//void plan_reset_buffer(); // Reset buffer only.

// Add a new linear movement to the buffer. target[N_AXIS] is the signed, absolute target position
// in millimeters. Feed rate specifies the speed of the motion. If feed rate is inverted, the feed
// rate is taken to mean "frequency" and would complete the operation in 1/feed_rate minutes.
bool plan_buffer_line(float *target, plan_line_data_t *pl_data);

// Called when the current block is no longer needed. Discards the block and makes the memory
// availible for new blocks.
void plan_discard_current_block();

// Gets the planner block for the special system motion cases. (Parking/Homing)
plan_block_t *plan_get_system_motion_block();

// Gets the current block. Returns NULL if buffer empty
plan_block_t *plan_get_current_block();

// Called by step segment buffer when computing executing block velocity profile.
float plan_get_exec_block_exit_speed_sqr();

// Called by main program during planner calculations and step segment buffer during initialization.
float plan_compute_profile_nominal_speed(plan_block_t *block);

// Reset the planner position vector (in steps)
void plan_sync_position();

// Reinitialize plan with a partially completed block
void plan_cycle_reinitialize();

// Returns the number of available blocks in the planner buffer.
uint_fast16_t plan_get_block_buffer_available();

// Returns the status of the block ring buffer. True, if buffer is full.
bool plan_check_full_buffer();

void plan_feed_override (int_fast16_t feed_override, int_fast16_t rapid_override);

#endif
