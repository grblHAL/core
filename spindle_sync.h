/*
  spindle_sync.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Spindle sync data structures

  NOTE: not referenced in the core

  Part of grblHAL

  Copyright (c) 2020-2025 Terje Io

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

#ifndef _SPINDLE_SYNC_H_
#define _SPINDLE_SYNC_H_

#include "pid.h"

// Free running timer log data.
// The free running timer is used to timestamp pulse events from the encoder.
typedef struct {
    volatile uint32_t last_index;   // Timer value at last encoder index pulse
    volatile uint32_t last_pulse;   // Timer value at last encoder pulse
    volatile uint32_t pulse_length; // Last timer tics between spindle encoder pulse interrupts.
} spindle_encoder_timer_t;

// Pulse counter timer log data.
// This counter is used to "prescale" the encoder pulses in order to
// reduce pulse interrupt frequency. This allows the use of high PPR encoders
// without overloading the MCU while still making use of the better resolution.
// NOTE: if a 16bit counter is used then it is important than proper casting
//       is performed in order to handle counter overflow correctly.
typedef struct {
    volatile uint32_t last_count;   // Counter value at last encoder pulse interrupt
    volatile uint32_t last_index;   // Counter value at last encoder index interrupt
    volatile uint32_t index_count;
    volatile uint32_t pulse_count;
} spindle_encoder_counter_t;

typedef struct {
    uint32_t ppr;                       // Encoder pulses per revolution
    float rpm_factor;                   // Inverse of event timer tics per RPM
    float pulse_distance;               // Encoder pulse distance in fraction of one revolution
    uint32_t maximum_tt;                // Maximum timer tics since last spindle encoder pulse before RPM = 0 is returned
    spindle_encoder_timer_t timer;      // Event timestamps
    spindle_encoder_counter_t counter;  // Encoder event counts
    uint32_t error_count;               // Incremented when actual PPR count differs from ppr setting
    uint32_t tics_per_irq;              // Counts per interrupt generated (prescaler value)
    volatile bool spin_lock;
} spindle_encoder_t;

#endif
