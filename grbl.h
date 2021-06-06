/*
  grbl.h - main Grbl include file for compile time configuration

  Part of grblHAL

  Copyright (c) 2017-2021 Terje Io
  Copyright (c) 2015-2016 Sungeun K. Jeon for Gnea Research LLC

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

#ifndef _GRBL_H_
#define _GRBL_H_

#include <stdint.h>
#include <stdbool.h>

#include "config.h"

// Grbl versioning system
#if COMPATIBILITY_LEVEL == 0
#define GRBL_VERSION "1.1f"
#else
#define GRBL_VERSION "1.1f"
#endif
#define GRBL_VERSION_BUILD "20210604"

// The following symbols are set here if not already set by the compiler or in config.h
// Do NOT change here!

#ifdef GRBL_ESP32
#include "esp_attr.h"
#define ISR_CODE IRAM_ATTR
#else
//#define ISR_CODE __attribute__((long_call, section(".data")))
// Used to decorate code run in interrupt context.
// Do not remove or change unless you know what you are doing.
#define ISR_CODE
#endif

#ifdef ARDUINO
#include <Arduino.h>
#endif

#ifndef PROGMEM
#define PROGMEM
#endif

#ifndef N_AXIS
#define N_AXIS 3 // Number of axes
#endif

#ifndef COMPATIBILITY_LEVEL
#define COMPATIBILITY_LEVEL 0
#endif

#if (defined(COREXY) || defined(WALL_PLOTTER) || defined(MASLOW_ROUTER)) && !defined(KINEMATICS_API)
#define KINEMATICS_API
#endif

#ifndef CHECK_MODE_DELAY
#define CHECK_MODE_DELAY 0 // ms
#endif

#ifndef SAFETY_DOOR_SPINDLE_DELAY
#define SAFETY_DOOR_SPINDLE_DELAY 4.0f // Float (seconds)
#endif

#ifndef SAFETY_DOOR_COOLANT_DELAY
#define SAFETY_DOOR_COOLANT_DELAY 1.0f // Float (seconds)
#endif

#ifndef SPINDLE_RPM_CONTROLLED
#define SPINDLE_PWM_DIRECT
#endif

#ifndef SLEEP_DURATION
#define SLEEP_DURATION 5.0f // Number of minutes before sleep mode is entered.
#endif

#ifndef BUFFER_NVSDATA_DISABLE
#define BUFFER_NVSDATA
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
#define CMD_REBOOT 0x14 // ctrl-T (DC4) - only acted upon if preceeded by 0x1B (ESC)
#define CMD_RESET 0x18 // ctrl-X (CAN)
#define CMD_STOP 0x19 // ctrl-Y (EM)
#define CMD_STATUS_REPORT_LEGACY '?'
#define CMD_CYCLE_START_LEGACY '~'
#define CMD_FEED_HOLD_LEGACY '!'
#define CMD_PROGRAM_DEMARCATION '%'

// NOTE: All override realtime commands must be in the extended ASCII character set, starting
// at character value 128 (0x80) and up to 255 (0xFF). If the normal set of realtime commands,
// such as status reports, feed hold, reset, and cycle start, are moved to the extended set
// space, protocol.c's protocol_process_realtime() will need to be modified to accomodate the change.
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
#define CMD_OVERRIDE_FEED_RESET 0x90         // Restores feed override value to 100%.
#define CMD_OVERRIDE_FEED_COARSE_PLUS 0x91
#define CMD_OVERRIDE_FEED_COARSE_MINUS 0x92
#define CMD_OVERRIDE_FEED_FINE_PLUS 0x93
#define CMD_OVERRIDE_FEED_FINE_MINUS 0x94
#define CMD_OVERRIDE_RAPID_RESET 0x95        // Restores rapid override value to 100%.
#define CMD_OVERRIDE_RAPID_MEDIUM 0x96
#define CMD_OVERRIDE_RAPID_LOW 0x97
// #define CMD_OVERRIDE_RAPID_EXTRA_LOW 0x98 // *NOT SUPPORTED*
#define CMD_OVERRIDE_SPINDLE_RESET 0x99      // Restores spindle override value to 100%.
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

// Number of blocks Grbl executes upon startup. These blocks are stored in non-volatile storage, where the size
// and addresses are defined in settings.h. With the current settings, up to 2 startup blocks may
// be stored and executed in order. These startup blocks would typically be used to set the g-code
// parser state depending on user preferences.
#define N_STARTUP_LINE 2 // Integer (1-2)

// Number of decimal places (scale) output by Grbl for certain value types. These settings
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
#define MAX_FEED_RATE_OVERRIDE          200 // Percent of programmed feed rate (100-255). Usually 120% or 200%
#define MIN_FEED_RATE_OVERRIDE           10 // Percent of programmed feed rate (1-100). Usually 50% or 1%
#define FEED_OVERRIDE_COARSE_INCREMENT   10 // (1-99). Usually 10%.
#define FEED_OVERRIDE_FINE_INCREMENT      1 // (1-99). Usually 1%.

#define DEFAULT_RAPID_OVERRIDE  100 // 100%. Don't change this value.
#define RAPID_OVERRIDE_MEDIUM    50 // Percent of rapid (1-99). Usually 50%.
#define RAPID_OVERRIDE_LOW       25 // Percent of rapid (1-99). Usually 25%.
// #define RAPID_OVERRIDE_EXTRA_LOW 5 // *NOT SUPPORTED* Percent of rapid (1-99). Usually 5%.

// #define ENABLE_SPINDLE_LINEARIZATION        // Uncomment to enable spindle RPM linearization. Requires compatible driver if enabled.
#define SPINDLE_NPWM_PIECES                 4 // Maximum number of pieces for spindle RPM linearization, do not change unless more are needed.
#define DEFAULT_SPINDLE_RPM_OVERRIDE      100 // 100%. Don't change this value.
#define MAX_SPINDLE_RPM_OVERRIDE          200 // Percent of programmed spindle speed (100-255). Usually 200%.
#define MIN_SPINDLE_RPM_OVERRIDE           10 // Percent of programmed spindle speed (1-100). Usually 10%.
#define SPINDLE_OVERRIDE_COARSE_INCREMENT  10 // (1-99). Usually 10%.
#define SPINDLE_OVERRIDE_FINE_INCREMENT     1 // (1-99). Usually 1%.

// Some status report data isn't necessary for realtime, only intermittently, because the values don't
// change often. The following macros configures how many times a status report needs to be called before
// the associated data is refreshed and included in the status report. However, if one of these value
// changes, Grbl will automatically include this data in the next status report, regardless of what the
// count is at the time. This helps reduce the communication overhead involved with high frequency reporting
// and agressive streaming. There is also a busy and an idle refresh count, which sets up Grbl to send
// refreshes more often when its not doing anything important. With a good GUI, this data doesn't need
// to be refreshed very often, on the order of a several seconds.
// NOTE: WCO refresh must be 2 or greater. OVERRIDE refresh must be 1 or greater.
//#define REPORT_OVERRIDE_REFRESH_BUSY_COUNT 20   // (1-255)
//#define REPORT_OVERRIDE_REFRESH_IDLE_COUNT 10   // (1-255) Must be less than or equal to the busy count
//#define REPORT_WCO_REFRESH_BUSY_COUNT 30        // (2-255)
//#define REPORT_WCO_REFRESH_IDLE_COUNT 10        // (2-255) Must be less than or equal to the busy count

// The temporal resolution of the acceleration management subsystem. A higher number gives smoother
// acceleration, particularly noticeable on machines that run at very high feedrates, but may negatively
// impact performance. The correct value for this parameter is machine dependent, so it's advised to
// set this only as high as needed. Approximate successful values can widely range from 50 to 200 or more.
// NOTE: Changing this value also changes the execution time of a segment in the step segment buffer.
// When increasing this value, this stores less overall time in the segment buffer and vice versa. Make
// certain the step segment buffer is increased/decreased to account for these changes.
//#define ACCELERATION_TICKS_PER_SECOND 100

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

// Sets the maximum step rate allowed to be written as a Grbl setting. This option enables an error
// check in the settings module to prevent settings values that will exceed this limitation. The maximum
// step rate is strictly limited by the CPU speed and will change if something other than an AVR running
// at 16MHz is used.
// NOTE: For now disabled, will enable if flash space permits.
// #define MAX_STEP_RATE_HZ 30000 // Hz

// With this enabled, Grbl sends back an echo of the line it has received, which has been pre-parsed (spaces
// removed, capitalized letters, no comments) and is to be immediately executed by Grbl. Echoes will not be
// sent upon a line buffer overflow, but should for all normal lines sent to Grbl. For example, if a user
// sendss the line 'g1 x1.032 y2.45 (test comment)', Grbl will echo back in the form '[echo: G1X1.032Y2.45]'.
// NOTE: Only use this for debugging purposes!! When echoing, this takes up valuable resources and can effect
// performance. If absolutely needed for normal operation, the serial write buffer should be greatly increased
// to help minimize transmission waiting within the serial write protocol.
// #define REPORT_ECHO_LINE_RECEIVED // Default disabled. Uncomment to enable.

// Sets which axis the tool length offset is applied. Assumes the spindle is always parallel with
// the selected axis with the tool oriented toward the negative direction. In other words, a positive
// tool length offset value is subtracted from the current location.
#if COMPATIBILITY_LEVEL > 2 && defined(TOOL_LENGTH_OFFSET_AXIS) == 0
#define TOOL_LENGTH_OFFSET_AXIS Z_AXIS // Default z-axis. Valid values are X_AXIS, Y_AXIS, or Z_AXIS.
#endif

// Max length of gcode lines (blocks) stored in non-volatile storage
#if N_AXIS == 6 && COMPATIBILITY_LEVEL <= 1
#define MAX_STORED_LINE_LENGTH 60 // do not change!
#else
#define MAX_STORED_LINE_LENGTH 70 // do not set > 70 unless less than 5 axes are enabled or COMPATIBILITY_LEVEL > 1
#endif

#endif
