/*
  grbl.h - main grblHAL include file for compile time configuration

  Part of grblHAL

  Copyright (c) 2017-2025 Terje Io
  Copyright (c) 2015-2016 Sungeun K. Jeon for Gnea Research LLC

  grblHAL is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  grblHAL is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with grblHAL.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _GRBL_H_
#define _GRBL_H_

#ifdef __SAM3X8E__
#define _WIRING_CONSTANTS_ // for shutting up compiler warnings due to bad framework code for Arduino Due
#endif

#include <stdint.h>
#include <stdbool.h>

#ifdef ARDUINO
#include <Arduino.h>
#endif

#include "config.h"

// grblHAL versioning system
#if COMPATIBILITY_LEVEL == 0
#define GRBL_VERSION "1.1f"
#else
#define GRBL_VERSION "1.1f"
#endif
#define GRBL_BUILD 20250109

#define GRBL_URL "https://github.com/grblHAL"

// The following symbols are set here if not already set by the compiler or in config.h
// Do NOT change here!

#ifdef GRBL_ESP32
#include "esp_attr.h"
#define ISR_CODE IRAM_ATTR
#else
// #define ISR_CODE __attribute__((long_call, section(".data")))
// Used to decorate code run in interrupt context.
// Do not remove or change unless you know what you are doing.
#define ISR_CODE //!< Used by some drivers to force a function to always stay in RAM to improve performance.
#endif

#ifdef RP2040
#include "pico.h"
#define ISR_FUNC(fn) __not_in_flash_func(fn)
#else
#define ISR_FUNC(fn) fn
#endif

#ifndef PROGMEM
#define PROGMEM
#endif

#if (COREXY || WALL_PLOTTER || DELTA_ROBOT || POLAR_ROBOT || MASLOW_ROUTER) && !defined(KINEMATICS_API)
#define KINEMATICS_API
#endif

// The following symbols are default values that are unlikely to be changed
// Do not change unless you know what you are doing!

// Define realtime command special characters. These characters are 'picked-off' directly from the
// serial read data stream and are not passed to the grbl line execution parser. Select characters
// that do not and must not exist in the streamed g-code program. ASCII control characters may be
// used, if they are available per user setup. Also, extended ASCII codes (>127), which are never in
// g-code programs, maybe selected for interface programs.
// NOTE: If changed, manually update help message in report.c.

#define CMD_EXIT 0x03 // ctrl-C (ETX)
#define CMD_REBOOT 0x14 // ctrl-T (DC4) - only acted upon if preceded by 0x1B (ESC)
#define CMD_RESET 0x18 // ctrl-X (CAN)
#define CMD_STOP 0x19 // ctrl-Y (EM)
#define CMD_STATUS_REPORT_LEGACY '?'
#define CMD_CYCLE_START_LEGACY '~'
#define CMD_FEED_HOLD_LEGACY '!'
#define CMD_PROGRAM_DEMARCATION '%'

// NOTE: All override realtime commands must be in the extended ASCII character set, starting
// at character value 128 (0x80) and up to 255 (0xFF). If the normal set of realtime commands,
// such as status reports, feed hold, reset, and cycle start, are moved to the extended set
// space, protocol.c's protocol_process_realtime() will need to be modified to accommodate the change.
#define CMD_STATUS_REPORT 0x80 // TODO: use 0x05 ctrl-E ENQ instead?
#define CMD_CYCLE_START 0x81   // TODO: use 0x06 ctrl-F ACK instead? or SYN/DC2/DC3?
#define CMD_FEED_HOLD 0x82     // TODO: use 0x15 ctrl-U NAK instead?
#define CMD_GCODE_REPORT 0x83
#define CMD_SAFETY_DOOR 0x84
#define CMD_JOG_CANCEL  0x85
//#define CMD_DEBUG_REPORT 0x86 // Only when DEBUG enabled, sends debug report in '{}' braces.
#define CMD_STATUS_REPORT_ALL 0x87
#define CMD_OPTIONAL_STOP_TOGGLE 0x88
#define CMD_SINGLE_BLOCK_TOGGLE 0x89
#define CMD_OVERRIDE_FAN0_TOGGLE 0x8A       // Toggle Fan 0 on/off, not implemented by the core.
#define CMD_MPG_MODE_TOGGLE 0x8B            // Toggle MPG mode on/off, not implemented by the core.
#define CMD_AUTO_REPORTING_TOGGLE 0x8C      // Toggle auto real time reporting if configured.
#define CMD_OVERRIDE_FEED_RESET 0x90        // Restores feed override value to 100%.
#define CMD_OVERRIDE_FEED_COARSE_PLUS 0x91
#define CMD_OVERRIDE_FEED_COARSE_MINUS 0x92
#define CMD_OVERRIDE_FEED_FINE_PLUS 0x93
#define CMD_OVERRIDE_FEED_FINE_MINUS 0x94
#define CMD_OVERRIDE_RAPID_RESET 0x95       // Restores rapid override value to 100%.
#define CMD_OVERRIDE_RAPID_MEDIUM 0x96
#define CMD_OVERRIDE_RAPID_LOW 0x97
// #define CMD_OVERRIDE_RAPID_EXTRA_LOW 0x98 // *NOT SUPPORTED*
#define CMD_OVERRIDE_SPINDLE_RESET 0x99     // Restores spindle override value to 100%.
#define CMD_OVERRIDE_SPINDLE_COARSE_PLUS 0x9A
#define CMD_OVERRIDE_SPINDLE_COARSE_MINUS 0x9B
#define CMD_OVERRIDE_SPINDLE_FINE_PLUS 0x9C
#define CMD_OVERRIDE_SPINDLE_FINE_MINUS 0x9D
#define CMD_OVERRIDE_SPINDLE_STOP 0x9E
#define CMD_OVERRIDE_COOLANT_FLOOD_TOGGLE 0xA0
#define CMD_OVERRIDE_COOLANT_MIST_TOGGLE 0xA1
#define CMD_PID_REPORT 0xA2
#define CMD_TOOL_ACK 0xA3
#define CMD_PROBE_CONNECTED_TOGGLE 0xA4

// System motion line numbers must be zero.
#define JOG_LINE_NUMBER 0

// Number of blocks grblHAL executes upon startup. These blocks are stored in non-volatile storage, where the size
// and addresses are defined in settings.h. With the current settings, up to 2 startup blocks may
// be stored and executed in order. These startup blocks would typically be used to set the g-code
// parser state depending on user preferences.
#define N_STARTUP_LINE 2 // Integer (1-2)

// Number of decimal places (scale) output by grblHAL for certain value types. These settings
// are determined by realistic and commonly observed values in CNC machines. For example, position
// values cannot be less than 0.001mm or 0.0001in, because machines can not be physically more
// precise this. So, there is likely no need to change these, but you can if you need to here.
// NOTE: Must be an integer value from 0 to ~4. More than 4 may exhibit round-off errors.
#define N_DECIMAL_COORDVALUE_INCH 4 // Coordinate or position value in inches
#define N_DECIMAL_COORDVALUE_MM   3 // Coordinate or position value in mm
#define N_DECIMAL_RATEVALUE_INCH  1 // Rate or velocity value in in/min
#define N_DECIMAL_RATEVALUE_MM    0 // Rate or velocity value in mm/min
#define N_DECIMAL_SETTINGVALUE    3 // Floating point setting values
#define N_DECIMAL_RPMVALUE        0 // RPM value in rotations per min
#define N_DECIMAL_PIDVALUE        3 // PID value

// ---------------------------------------------------------------------------------------
// ADVANCED CONFIGURATION OPTIONS:

// Enables code for debugging purposes. Not for general use and always in constant flux.
// #define DEBUG // Uncomment to enable. Default disabled.

// Configure rapid, feed, and spindle override settings. These values define the max and min
// allowable override values and the coarse and fine increments per command received. Please
// note the allowable values in the descriptions following each define.
#define DEFAULT_FEED_OVERRIDE           100 // 100%. Don't change this value.
#ifndef MAX_FEED_RATE_OVERRIDE
#define MAX_FEED_RATE_OVERRIDE          200 // Percent of programmed feed rate (100-65535). Usually 120% or 200%
#endif
#ifndef MIN_FEED_RATE_OVERRIDE
#define MIN_FEED_RATE_OVERRIDE           10 // Percent of programmed feed rate (1-100). Usually 50% or 1%
#endif
#define FEED_OVERRIDE_COARSE_INCREMENT   10 // (1-99). Usually 10%.
#define FEED_OVERRIDE_FINE_INCREMENT      1 // (1-99). Usually 1%.

#define DEFAULT_RAPID_OVERRIDE  100 // 100%. Don't change this value.
#define RAPID_OVERRIDE_MEDIUM    50 // Percent of rapid (1-99). Usually 50%.
#define RAPID_OVERRIDE_LOW       25 // Percent of rapid (1-99). Usually 25%.
// #define RAPID_OVERRIDE_EXTRA_LOW 5 // *NOT SUPPORTED* Percent of rapid (1-99). Usually 5%.

#define DEFAULT_SPINDLE_RPM_OVERRIDE      100 // 100%. Don't change this value.
#ifndef MAX_SPINDLE_RPM_OVERRIDE
#define MAX_SPINDLE_RPM_OVERRIDE          200 // Percent of programmed spindle speed (100-65535). Usually 200%.
#endif
#ifndef MIN_SPINDLE_RPM_OVERRIDE
#define MIN_SPINDLE_RPM_OVERRIDE           10 // Percent of programmed spindle speed (1-100). Usually 10%.
#endif
#define SPINDLE_OVERRIDE_COARSE_INCREMENT  10 // (1-99). Usually 10%.
#define SPINDLE_OVERRIDE_FINE_INCREMENT     1 // (1-99). Usually 1%.

// Adaptive Multi-Axis Step Smoothing (AMASS) is an advanced feature that does what its name implies,
// smoothing the stepping of multi-axis motions. This feature smooths motion particularly at low step
// frequencies below 10kHz, where the aliasing between axes of multi-axis motions can cause audible
// noise and shake your machine. At even lower step frequencies, AMASS adapts and provides even better
// step smoothing. See stepper.c for more details on the AMASS system works.
#define ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING  // Default enabled. Comment to disable.

// Define Adaptive Multi-Axis Step-Smoothing(AMASS) levels and cutoff frequencies. The highest level
// frequency bin starts at 0Hz and ends at its cutoff frequency. The next lower level frequency bin
// starts at the next higher cutoff frequency, and so on. The cutoff frequencies for each level must
// be considered carefully against how much it over-drives the stepper ISR, the accuracy of the 16-bit
// timer, and the CPU overhead. Level 0 (no AMASS, normal operation) frequency bin starts at the
// Level 1 cutoff frequency and up to as fast as the CPU allows (over 30kHz in limited testing).
// NOTE: AMASS cutoff frequency multiplied by ISR overdrive factor must not exceed maximum step frequency.
// NOTE: Current settings are set to overdrive the ISR to no more than 16kHz, balancing CPU overhead
// and timer accuracy.  Do not alter these settings unless you know what you are doing.
#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
  #ifndef MAX_AMASS_LEVEL
    #define MAX_AMASS_LEVEL 3
  #endif
  #if MAX_AMASS_LEVEL <= 0
    error "AMASS must have 1 or more levels to operate correctly."
  #endif
#endif

// Sets which axis the tool length offset is applied. Assumes the spindle is always parallel with
// the selected axis with the tool oriented toward the negative direction. In other words, a positive
// tool length offset value is subtracted from the current location.
#if COMPATIBILITY_LEVEL > 2 && TOOL_LENGTH_OFFSET_AXIS == -1
#undef TOOL_LENGTH_OFFSET_AXIS
#define TOOL_LENGTH_OFFSET_AXIS Z_AXIS // Default z-axis. Valid values are X_AXIS, Y_AXIS, or Z_AXIS.
#endif

// Max length of gcode lines (blocks) stored in non-volatile storage
#if N_AXIS == 6 && COMPATIBILITY_LEVEL <= 1
#define MAX_STORED_LINE_LENGTH 60 // do not change!
#else
#define MAX_STORED_LINE_LENGTH 70 // do not set > 70 unless less than 5 axes are enabled or COMPATIBILITY_LEVEL > 1
#endif

#if N_SPINDLE > 4
#define N_SPINDLE_SELECTABLE 4 // Max 4. for now!
#else
#define N_SPINDLE_SELECTABLE N_SPINDLE
#endif

typedef uint_fast16_t override_t;

#endif
