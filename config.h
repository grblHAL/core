/*
  config.h - compile time configuration and default setting values

  Part of grblHAL

  Copyright (c) 2020-2022 Terje Io

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

// This file contains compile-time configurations for Grbl's internal system. For the most part,
// users will not need to directly modify these, but they are here for specific needs, i.e.
// performance tuning or adjusting to non-typical machines.

// IMPORTANT: Any changes here requires a full re-compiling of the source code to propagate them.
//            A reset of non-volatile storage with $RST=* after reflashing is also required as
//            most are just default values for settings.

#ifndef _GRBL_CONFIG_H_
#define _GRBL_CONFIG_H_

// Compile time only default configuration

#ifndef N_AXIS
/*! Defines number of axes supported - minimum 3, maximum 6

If more than 3 axes are configured a compliant driver and map file is needed.
*/
#define N_AXIS 3 // Number of axes
#endif

#ifndef N_SPINDLE
/*! Defines number of spindles supported - minimum 1, maximum 32
*/
#define N_SPINDLE 1
#endif

#ifndef COMPATIBILITY_LEVEL
/*! Define compatibility level with the grbl 1.1 protocol.

Additional G- and M-codes are not disabled except when level is set to >= 10.
This does not apply to G- and M-codes dependent on driver and/or configuration settings disabled by setting level > 1.
<br>Set to 0 for reporting itself as "GrblHAL" with protocol extensions enabled.
<br>Set to 1 to disable some extensions, and for reporting itself as "Grbl".
<br>Set to 2 to disable new settings as well, use #define parameters for setting default values.
<br>These can be found in in this file and in defaults.h.
<br>Set to 10 to also disable new coordinate system offsets (G59.1 - G59.3) and some $# report extensions.

__NOTE:__ if switching to a level > 1 please reset non-volatile storage with \a $RST=* after reflashing!
*/
#define COMPATIBILITY_LEVEL 0
#endif

//#define KINEMATICS_API // Remove comment to add HAL entry points for custom kinematics

// Enable Maslow router kinematics.
// Experimental - testing required and homing needs to be worked out.
//#define MASLOW_ROUTER // Default disabled. Uncomment to enable.

// Enable wall plotter kinematics.
// Experimental - testing required and homing needs to be worked out.
//#define WALL_PLOTTER // Default disabled. Uncomment to enable.

// Enable CoreXY kinematics. Use ONLY with CoreXY machines.
// IMPORTANT: If homing is enabled, you must reconfigure the homing cycle #defines above to
//#define HOMING_CYCLE_0 X_AXIS_BIT and #define HOMING_CYCLE_1 Y_AXIS_BIT
// NOTE: This configuration option alters the motion of the X and Y axes to principle of operation
// defined at (http://corexy.com/theory.html). Motors are assumed to positioned and wired exactly as
// described, if not, motions may move in strange directions. Grbl requires the CoreXY A and B motors
// have the same steps per mm internally.
//#define COREXY // Default disabled. Uncomment to enable.

// Add a short delay for each block processed in Check Mode to
// avoid overwhelming the sender with fast reply messages.
// This is likely to happen when streaming is done via a protocol where
// the speed is not limited to 115200 baud. An example is native USB streaming.
//#define CHECK_MODE_DELAY 0 // ms

// After the safety door switch has been toggled and restored, this setting sets the power-up delay
// between restoring the spindle and coolant and resuming the cycle.
//#define SAFETY_DOOR_SPINDLE_DELAY 4.0f // Float (seconds)
//#define SAFETY_DOOR_COOLANT_DELAY 1.0f // Float (seconds)

/*! @name Control signals bit definitions and mask.

__NOTE:__ these definitions are only referenced in this file. Do __NOT__ change!
*/
///@{
#define SIGNALS_RESET_BIT (1<<0)
#define SIGNALS_FEEDHOLD_BIT (1<<1)
#define SIGNALS_CYCLESTART_BIT (1<<2)
#define SIGNALS_SAFETYDOOR_BIT (1<<3)
#define SIGNALS_BLOCKDELETE_BIT (1<<4)
#define SIGNALS_STOPDISABLE_BIT (1<<5)
#define SIGNALS_ESTOP_BIT (1<<6)
#define SIGNALS_PROBE_CONNECTED_BIT (1<<7)
#define SIGNALS_MOTOR_FAULT_BIT (1<<8)
#define SIGNALS_BITMASK (SIGNALS_RESET_BIT|SIGNALS_FEEDHOLD_BIT|SIGNALS_CYCLESTART_BIT|SIGNALS_SAFETYDOOR_BIT|SIGNALS_BLOCKDELETE_BIT|SIGNALS_STOPDISABLE_BIT|SIGNALS_ESTOP_BIT|SIGNALS_PROBE_CONNECTED_BIT|SIGNALS_MOTOR_FAULT_BIT)
///@}

// ---------------------------------------------------------------------------------------
// ADVANCED CONFIGURATION OPTIONS:

// Enables code for debugging purposes. Not for general use and always in constant flux.
//#define DEBUG // Uncomment to enable. Default disabled.
//#define DEBUGOUT 0 // Uncomment to claim serial port with given instance number and add HAL entry point for debug output.

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

// Sets the maximum step rate allowed to be written as a Grbl setting. This option enables an error
// check in the settings module to prevent settings values that will exceed this limitation. The maximum
// step rate is strictly limited by the CPU speed and will change if something other than an AVR running
// at 16MHz is used.
// NOTE: For now disabled, will enable if flash space permits.
//#define MAX_STEP_RATE_HZ 30000 // Hz

// With this enabled, Grbl sends back an echo of the line it has received, which has been pre-parsed (spaces
// removed, capitalized letters, no comments) and is to be immediately executed by Grbl. Echoes will not be
// sent upon a line buffer overflow, but should for all normal lines sent to Grbl. For example, if a user
// sendss the line 'g1 x1.032 y2.45 (test comment)', Grbl will echo back in the form '[echo: G1X1.032Y2.45]'.
// NOTE: Only use this for debugging purposes!! When echoing, this takes up valuable resources and can effect
// performance. If absolutely needed for normal operation, the serial write buffer should be greatly increased
// to help minimize transmission waiting within the serial write protocol.
//#define REPORT_ECHO_LINE_RECEIVED // Default disabled. Uncomment to enable.

// Sets which axis the tool length offset is applied. Assumes the spindle is always parallel with
// the selected axis with the tool oriented toward the negative direction. In other words, a positive
// tool length offset value is subtracted from the current location.
//#define TOOL_LENGTH_OFFSET_AXIS Z_AXIS // Default z-axis. Valid values are X_AXIS, Y_AXIS, or Z_AXIS.

// Minimum planner junction speed. Sets the default minimum junction speed the planner plans to at
// every buffer block junction, except for starting from rest and end of the buffer, which are always
// zero. This value controls how fast the machine moves through junctions with no regard for acceleration
// limits or angle between neighboring block line move directions. This is useful for machines that can't
// tolerate the tool dwelling for a split second, i.e. 3d printers or laser cutters. If used, this value
// should not be much greater than zero or to the minimum value necessary for the machine to work.
//#define MINIMUM_JUNCTION_SPEED 0.0f // (mm/min)

// Sets the minimum feed rate the planner will allow. Any value below it will be set to this minimum
// value. This also ensures that a planned motion always completes and accounts for any floating-point
// round-off errors. Although not recommended, a lower value than 1.0 mm/min will likely work in smaller
// machines, perhaps to 0.1mm/min, but your success may vary based on multiple factors.
//#define MINIMUM_FEED_RATE 1.0f // (mm/min)

// Number of arc generation iterations by small angle approximation before exact arc trajectory
// correction with expensive sin() and cos() calculations. This parameter maybe decreased if there
// are issues with the accuracy of the arc generations, or increased if arc execution is getting
// bogged down by too many trig calculations.
//#define N_ARC_CORRECTION 12 // Integer (1-255)

// The arc G2/3 g-code standard is problematic by definition. Radius-based arcs have horrible numerical
// errors when arc at semi-circles(pi) or full-circles(2*pi). Offset-based arcs are much more accurate
// but still have a problem when arcs are full-circles (2*pi). This define accounts for the floating
// point issues when offset-based arcs are commanded as full circles, but get interpreted as extremely
// small arcs with around machine epsilon (1.2e-7rad) due to numerical round-off and precision issues.
// This define value sets the machine epsilon cutoff to determine if the arc is a full-circle or not.
// NOTE: Be very careful when adjusting this value. It should always be greater than 1.2e-7 but not too
// much greater than this. The default setting should capture most, if not all, full arc error situations.
//#define ARC_ANGULAR_TRAVEL_EPSILON 5E-7f // Float (radians)

// Default constants for G5 Cubic splines
//
//#define BEZIER_MIN_STEP 0.002f
//#define BEZIER_MAX_STEP 0.1f
//#define BEZIER_SIGMA 0.1f

// Time delay increments performed during a dwell. The default value is set at 50ms, which provides
// a maximum time delay of roughly 55 minutes, more than enough for most any application. Increasing
// this delay will increase the maximum dwell time linearly, but also reduces the responsiveness of
// run-time command executions, like status reports, since these are performed between each dwell
// time step.
//#define DWELL_TIME_STEP 50 // Integer (1-255) (milliseconds)

// The number of linear motions in the planner buffer to be planned at any give time. The vast
// majority of RAM that Grbl uses is based on this buffer size. Only increase if there is extra
// available RAM, like when re-compiling for MCU with ample amounts of RAM. Or decrease if the MCU begins to
// crash due to the lack of available RAM or if the CPU is having trouble keeping up with planning
// new incoming motions as they are executed.
//#define BLOCK_BUFFER_SIZE 36 // Uncomment to override default in planner.h.

// Governs the size of the intermediary step segment buffer between the step execution algorithm
// and the planner blocks. Each segment is set of steps executed at a constant velocity over a
// fixed time defined by ACCELERATION_TICKS_PER_SECOND. They are computed such that the planner
// block velocity profile is traced exactly. The size of this buffer governs how much step
// execution lead time there is for other Grbl processes have to compute and do their thing
// before having to come back and refill this buffer, currently at ~50msec of step moves.
//#define SEGMENT_BUFFER_SIZE 10 // Uncomment to override default in stepper.h.

// Configures the position after a probing cycle during Grbl's check mode. Disabled sets
// the position to the probe target, when enabled sets the position to the start position.
//#define SET_CHECK_MODE_PROBE_TO_START // Default disabled. Uncomment to enable.

// Force Grbl to check the state of the hard limit switches when the processor detects a pin
// change inside the hard limit ISR routine. By default, Grbl will trigger the hard limits
// alarm upon any pin change, since bouncing switches can cause a state check like this to
// misread the pin. When hard limits are triggered, they should be 100% reliable, which is the
// reason that this option is disabled by default. Only if your system/electronics can guarantee
// that the switches don't bounce, we recommend enabling this option. This will help prevent
// triggering a hard limit when the machine disengages from the switch.
// NOTE: This option has no effect if SOFTWARE_DEBOUNCE is enabled.
//#define HARD_LIMIT_FORCE_STATE_CHECK // Default disabled. Uncomment to enable.

// Adjusts homing cycle search and locate scalars. These are the multipliers used by Grbl's
// homing cycle to ensure the limit switches are engaged and cleared through each phase of
// the cycle. The search phase uses the axes max-travel setting times the SEARCH_SCALAR to
// determine distance to look for the limit switch. Once found, the locate phase begins and
// uses the homing pull-off distance setting times the LOCATE_SCALAR to pull-off and re-engage
// the limit switch.
// NOTE: Both of these values must be greater than 1.0 to ensure proper function.
//#define HOMING_AXIS_SEARCH_SCALAR  1.5f // Uncomment to override defaults in limits.c.
//#define HOMING_AXIS_LOCATE_SCALAR  10.0f // Uncomment to override defaults in limits.c.

// Enable the '$RST=*', '$RST=$', and '$RST=#' non-volatile storage restore commands. There are cases where
// these commands may be undesirable. Simply comment the desired macro to disable it.
// NOTE: See SETTINGS_RESTORE_ALL macro for customizing the `$RST=*` command.
//#define DISABLE_RESTORE_NVS_WIPE_ALL         // '$RST=*' Default enabled. Uncomment to disable.
//#define DISABLE_RESTORE_NVS_DEFAULT_SETTINGS // '$RST=$' Default enabled. Uncomment to disable.
//#define DISABLE_RESTORE_NVS_CLEAR_PARAMETERS // '$RST=#' Default enabled. Uncomment to disable.
//#define DISABLE_RESTORE_DRIVER_PARAMETERS    // '$RST=&' Default enabled. Uncomment to disable. For drivers that implements non-generic settings.

// Defines the non-volatile data restored upon a settings version change and `$RST=*` command. Whenever the
// the settings or other non-volatile data structure changes between Grbl versions, Grbl will automatically
// wipe and restore the non-volatile data. These macros controls what data is wiped and restored. This is useful
// particularily for OEMs that need to retain certain data. For example, the BUILD_INFO string can be
// written into non-volatile storage via a separate program to contain product data. Altering these
// macros to not restore the build info non-volatile storage will ensure this data is retained after firmware upgrades.
//#define SETTINGS_RESTORE_DEFAULTS          0 // Default enabled, uncomment to disable
//#define SETTINGS_RESTORE_PARAMETERS        0 // Default enabled, uncomment to disable
//#define SETTINGS_RESTORE_STARTUP_LINES     0 // Default enabled, uncomment to disable
//#define SETTINGS_RESTORE_BUILD_INFO        0 // Default enabled, uncomment to disable
//#define SETTINGS_RESTORE_DRIVER_PARAMETERS 0 // Default enabled, uncomment to disable

// Enable the '$I=(string)' build info write command. If disabled, any existing build info data must
// be placed into non-volatile storage via external means with a valid checksum value. This macro option is useful
// to prevent this data from being over-written by a user, when used to store OEM product data.
// NOTE: If disabled and to ensure Grbl can never alter the build info line, you'll also need to enable
// the SETTING_RESTORE_ALL macro above and remove SETTINGS_RESTORE_BUILD_INFO from the mask.
// NOTE: See the included grblWrite_BuildInfo.ino example file to write this string seperately.
//#define DISABLE_BUILD_INFO_WRITE_COMMAND // '$I=' Default enabled. Uncomment to disable.

// Enables and configures Grbl's sleep mode feature. If the spindle or coolant are powered and Grbl
// is not actively moving or receiving any commands, a sleep timer will start. If any data or commands
// are received, the sleep timer will reset and restart until the above condition are not satisfied.
// If the sleep timer elaspes, Grbl will immediately execute the sleep mode by shutting down the spindle
// and coolant and entering a safe sleep state. If parking is enabled, Grbl will park the machine as
// well. While in sleep mode, only a hard/soft reset will exit it and the job will be unrecoverable.
// NOTE: Sleep mode is a safety feature, primarily to address communication disconnect problems. To
// keep Grbl from sleeping, employ a stream of '?' status report commands as a connection "heartbeat".
//#define SLEEP_ENABLE  // Default disabled. Uncomment to enable.
//#define SLEEP_DURATION 5.0f // Number of minutes before sleep mode is entered.

// Disable non-volatile storage emulation/buffering in RAM (allocated from heap)
// Can be used for MCUs with no non-volatile storage or as buffer in order to avoid writing to
// non-volatile storage when not in idle state.
// The buffer will be written to non-volatile storage when in idle state.
//#define BUFFER_NVSDATA_DISABLE

//#define ENABLE_BACKLASH_COMPENSATION

// End compile time only default configuration

// When the HAL driver supports spindle sync then this option sets the number of pulses per revolution
// for the spindle encoder. Depending on the driver this may lead to the "spindle at speed" detection
// beeing enabled. When this is enabled grbl will wait for the spindle to reach the programmed speed
// before continue processing. NOTE: Currently there is no timeout for this wait.
// Default value is 0, meaning spindle sync is disabled
//#define DEFAULT_SPINDLE_PPR 0 // Pulses per revolution. Default 0.

// This option will automatically disable the laser during a feed hold by invoking a spindle stop
// override immediately after coming to a stop. However, this also means that the laser still may
// be reenabled by disabling the spindle stop override, if needed. This is purely a safety feature
// to ensure the laser doesn't inadvertently remain powered while at a stop and cause a fire.
//#define DEFAULT_ENABLE_LASER_DURING_HOLD // Default enabled. Uncomment to disable.

// This option is for what should happen on resume from feed hold.
// Default action is to restore spindle and coolant status (if overridden), this contradicts the
// behaviour of industrial controllers but is in line with earlier versions of Grbl.
//#define DEFAULT_NO_RESTORE_AFTER_FEED_HOLD // Default enabled. Uncomment to disable.

// When Grbl powers-cycles or is hard reset with the MCU reset button, Grbl boots up with no ALARM
// by default. This is to make it as simple as possible for new users to start using Grbl. When homing
// is enabled and a user has installed limit switches, Grbl will boot up in an ALARM state to indicate
// Grbl doesn't know its position and to force the user to home before proceeding. This option forces
// Grbl to always initialize into an ALARM state regardless of homing or not. This option is more for
// OEMs and LinuxCNC users that would like this power-cycle behavior.
//#define DEFAULT_FORCE_INITIALIZATION_ALARM // Default disabled. Uncomment to enable.

// At power-up or a reset, Grbl will check the limit switch states to ensure they are not active
// before initialization. If it detects a problem and the hard limits setting is enabled, Grbl will
// simply message the user to check the limits and enter an alarm state, rather than idle. Grbl will
// not throw an alarm message.
//#define DEFAULT_CHECK_LIMITS_AT_INIT // Default disabled. Uncomment to enable.

// Configure options for the parking motion, if enabled.
//#define DEFAULT_PARKING_AXIS Z_AXIS // Define which axis that performs the parking motion
//#define DEFAULT_PARKING_TARGET -5.0f // Parking axis target. In mm, as machine coordinate [-max_travel,0].
//#define DEFAULT_PARKING_RATE 500.0f // Parking fast rate after pull-out in mm/min.
//#define DEFAULT_PARKING_PULLOUT_RATE 100.0f // Pull-out/plunge slow feed rate in mm/min.
//#define DEFAULT_PARKING_PULLOUT_INCREMENT 5.0f // Spindle pull-out and plunge distance in mm. Incremental distance.
                                                 // Must be positive value or equal to zero.


// Enables a special set of M-code commands that enables and disables the parking motion.
// These are controlled by `M56`, `M56 P1`, or `M56 Px` to enable and `M56 P0` to disable.
// The command is modal and will be set after a planner sync. Since it is g-code, it is
// executed in sync with g-code commands. It is not a real-time command.
// NOTE: PARKING_ENABLE is required. By default, M56 is active upon initialization. Use
// DEACTIVATE_PARKING_UPON_INIT to set M56 P0 as the power-up default.
//#define DEFAULT_ENABLE_PARKING_OVERRIDE_CONTROL  // Default disabled. Uncomment to enable
//#define DEFAULT_DEACTIVATE_PARKING_UPON_INIT // Default disabled. Uncomment to enable.

// Using printable ASCII characters for realtime commands can cause issues with
// files containing such characters in comments or settings. If the GCode sender support the
// use of the top-bit set alternatives for these then they may be disabled.
// NOTE: support for the top-bit set alternatives is always enabled.
// NOTE: when disabled they are still active outside of comments and $ settings
//       allowing their use from manual input, eg. from a terminal or MDI.
//#define DEFAULT_NO_LEGACY_RTCOMMANDS // Default disabled. Uncomment to enable.

//#define DEFAULT_TOOLCHANGE_MODE 0               // 0 = Normal mode, 1 = Manual change, 2 = Manual change @ G59.3,  3 = Manual change and probe sensor @ G59.3 - sets TLO
//#define DEFAULT_TOOLCHANGE_PROBING_DISTANCE 30  // max probing distance in mm for mode 3
//#define DEFAULT_TOOLCHANGE_FEED_RATE 25.0f      // mm/min
//#define DEFAULT_TOOLCHANGE_SEEK_RATE 200.0f     // mm/min
//#define DEFAULT_TOOLCHANGE_PULLOFF_RATE 200.0f  // mm/min

// The grbl.on_probe_fixture event handler is called by the default tool change algorithm when probing at G59.3.
// In addition it will be called on a "normal" probe sequence if the XY position is
// within the radius of the G59.3 position defined below.
// Uncomment and change if the default value of 5mm is not suitable or set it to 0.0f to disable.
// NOTE: A grbl.on_probe_fixture event handler is not installed by the core, it has to be provided
//       by a driver or a plugin.
//#define TOOLSETTER_RADIUS 5.0f

// By default, Grbl sets all input pins to normal-low operation with their internal pull-up resistors
// enabled. This simplifies the wiring for users by requiring only a normally closed (NC) switch connected
// to ground. It is not recommended to use normally-open (NO) switches as this increases the risk
// of electrical noise spuriously triggering the inputs. If normally-open (NO) switches are used the
// logic of the input signals should be be inverted with the invert settings below.
// The following options disable the internal pull-up resistors, and switches must be now connect to Vcc
// instead of ground.
// WARNING: When the pull-ups are disabled, this might require additional wiring with pull-down resistors!
//          Please check driver code and/or documentation.
//#define DISABLE_LIMIT_BITS_PULL_UP_MASK AXES_BITMASK
//#define DISABLE_LIMIT_BITS_PULL_UP_MASK (X_AXIS_BIT|Y_AXIS_BIT)
//#define DISABLE_CONTROL_PINS_PULL_UP_MASK SIGNALS_BITMASK
//#define DISABLE_CONTROL_PINS_PULL_UP_MASK (SIGNALS_SAFETYDOOR_BIT|SIGNALS_RESET_BIT)
//#define DISABLE_PROBE_BIT_PULL_UP

// If your machine has two limits switches wired in parallel to one axis, you will need to enable
// this feature. Since the two switches are sharing a single pin, there is no way for Grbl to tell
// which one is enabled. This option only effects homing, where if a limit is engaged, Grbl will
// alarm out and force the user to manually disengage the limit switch. Otherwise, if you have one
// limit switch for each axis, don't enable this option. By keeping it disabled, you can perform a
// homing cycle while on the limit switch and not have to move the machine off of it.
//#define DEFAULT_LIMITS_TWO_SWITCHES_ON_AXES //  Default disabled. Uncomment to enable.

// By default, Grbl disables feed rate overrides for all G38.x probe cycle commands. Although this
// may be different than some pro-class machine control, it's arguable that it should be this way.
// Most probe sensors produce different levels of error that is dependent on rate of speed. By
// keeping probing cycles to their programmed feed rates, the probe sensor should be a lot more
// repeatable. If needed, you can disable this behavior by uncommenting the define below.
//#define ALLOW_FEED_OVERRIDE_DURING_PROBE_CYCLES // Default disabled. Uncomment to enable.

// Inverts logic of the stepper enable signal(s).
#if COMPATIBILITY_LEVEL <= 2
// NOTE: Not universally available for individual axes - check driver documentation.
//       Specify at least X_AXIS_BIT if a common enable signal is used.
//#define INVERT_ST_ENABLE_MASK (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT) // Default disabled. Uncomment to enable.
#else
//#define INVERT_ST_ENABLE_MASK 1 // Default disabled. Uncomment to enable.
#endif
// Mask to be OR'ed with stepper disable signal(s). Axes configured will not be disabled.
// NOTE: Not universally available for individual axes - check driver documentation.
//       Specify at least X_AXIS_BIT if a common enable signal is used.
//#define ST_DEENERGIZE_MASK (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT) // Default disabled. Uncomment to enable.
//#define DEFAULT_STEPPING_INVERT_MASK 0
//#define DEFAULT_DIRECTION_INVERT_MASK 0

// Designate ABC axes as rotational. This will disable scaling (to mm) in inches mode.
// Set steps/mm for the axes to the value that represent the desired movement per unit.
// For the controller the distance is unitless and and can be in degrees, radians, rotations, ...
// NOTE: $376 can be used to configure rotational axes at run-time.
//#define ST_ROTATIONAL_MASK (A_AXIS_BIT|B_AXIS_BIT|C_AXIS_BIT) // Default disabled. Uncomment and possibly remove axis bit(s) as needed to enable.

// Inverts logic of the input signals based on a mask. This essentially means you are using
// normally-open (NO) switches on the specified pins, rather than the default normally-closed (NC) switches.
// NOTE: The first option will invert all control pins. The second option is an example of
// inverting only a few pins. See the start of this file for other signal definitions.
//#define INVERT_CONTROL_PIN_MASK SIGNALS_BITMASK // Default disabled. Uncomment to enable.
//#define INVERT_CONTROL_PIN_MASK (SIGNALS_SAFETYDOOR_BIT|SIGNALS_RESET_BIT) // Default disabled. Uncomment to enable.
//#define INVERT_LIMIT_BIT_MASK AXES_BITMASK // Default disabled. Uncomment to enable. Uncomment to enable.
//#define INVERT_LIMIT_BIT_MASK (X_AXIS_BIT|Y_AXIS_BIT) // Default disabled. Uncomment to enable.
// For inverting the probe pin use DEFAULT_INVERT_PROBE_BIT in defaults.h

// Inverts the selected spindle output signals from active high to active low. Useful for some pre-built electronic boards.
//#define INVERT_SPINDLE_ENABLE_PIN 1 // Default disabled. Uncomment to enable.
//#define INVERT_SPINDLE_CCW_PIN 1    // Default disabled. Uncomment to enable. NOTE: not supported by all drivers.
//#define INVERT_SPINDLE_PWM_PIN 1    // Default disabled. Uncomment to enable. NOTE: not supported by all drivers.

// Inverts the selected coolant signals from active high to active low. Useful for some pre-built electronic boards.
//#define INVERT_COOLANT_FLOOD_PIN 1 // Default disabled. Uncomment to enable.
//#define INVERT_COOLANT_MIST_PIN 1  // Default disabled. Note: not supported by all drivers.


// Used by variable spindle output only. This forces the PWM output to a minimum duty cycle when enabled.
// The PWM pin will still read 0V when the spindle is disabled. Most users will not need this option, but
// it may be useful in certain scenarios. This minimum PWM settings coincides with the spindle rpm minimum
// setting, like rpm max to max PWM. This is handy if you need a larger voltage difference between 0V disabled
// and the voltage set by the minimum PWM for minimum rpm. This difference is 0.02V per PWM value. So, when
// minimum PWM is at 1, only 0.02 volts separate enabled and disabled. At PWM 5, this would be 0.1V. Keep
// in mind that you will begin to lose PWM resolution with increased minimum PWM values, since you have less
// and less range over the total 255 PWM levels to signal different spindle speeds.
// NOTE: Compute duty cycle at the minimum PWM by this equation: (% duty cycle)=(SPINDLE_PWM_MIN_VALUE/255)*100
//#define DEFAULT_SPINDLE_PWM_MIN_VALUE 5.0f // Default disabled. Uncomment to enable. Must be greater than zero. Integer (1-255).

// Number of homing cycles performed after when the machine initially jogs to limit switches.
// This help in preventing overshoot and should improve repeatability. This value should be one or
// greater.
//#define DEFAULT_N_HOMING_LOCATE_CYCLE 1 // Integer (1-127)


// In Grbl v0.9 and prior, there is an old outstanding bug where the `WPos:` work position reported
// may not correlate to what is executing, because `WPos:` is based on the g-code parser state, which
// can be several motions behind. This option forces the planner buffer to empty, sync, and stop
// motion whenever there is a command that alters the work coordinate offsets `G10,G43.1,G92,G54-59`.
// This is the simplest way to ensure `WPos:` is always correct. Fortunately, it's exceedingly rare
// that any of these commands are used need continuous motions through them.
//#define DEFAULT_NO_FORCE_BUFFER_SYNC_DURING_WCO_CHANGE // Default enabled. Uncomment to disable.

// Upon a successful probe cycle, this option provides immediately feedback of the probe coordinates
// through an automatically generated message. If disabled, users can still access the last probe
// coordinates through Grbl '$#' print parameters.
//#define DEFAULT_NO_REPORT_PROBE_COORDINATES 1 // Default enabled. Uncomment to disable.

// The status report change for Grbl v1.1 and after also removed the ability to disable/enable most data
// fields from the report. This caused issues for GUI developers, who've had to manage several scenarios
// and configurations. The increased efficiency of the new reporting style allows for all data fields to
// be sent without potential performance issues.
// NOTE: The options below are here only provide a way to disable certain data fields if a unique
// situation demands it, but be aware GUIs may depend on this data. If disabled, it may not be compatible.
//#define DEFAULT_REPORT_MACHINE_POSITION  // Default disabled. Uncomment to enable.
//#define DEFAULT_NO_REPORT_BUFFER_STATE
//#define DEFAULT_NO_REPORT_LINE_NUMBERS
//#define DEFAULT_NO_REPORT_CURRENT_FEED_SPEED
//#define DEFAULT_NO_REPORT_PIN_STATE
//#define DEFAULT_NO_REPORT_WORK_COORD_OFFSET
//#define DEFAULT_NO_REPORT_OVERRIDES
//#define DEFAULT_REPORT_PARSER_STATE
//#define DEFAULT_REPORT_ALARM_SUBSTATE

// G92 offsets is by default stored to non-volatile storage (NVS) on changes and restored on startup
// if COMPATIBILITY_LEVEL is <= 1. If COMPATIBILITY_LEVEL is <= 1 then setting $384 can be used to change this at run-time.
// To allow store/restore of the G92 offset when COMPATIBILITY_LEVEL > 1 uncomment the line below and reset settings with $RST=*.
//#define DISABLE_G92_PERSISTENCE 0

#if COMPATIBILITY_LEVEL == 0
// Number of tools in tool table, uncomment and edit if neccesary to enable (max. 16 allowed)
//#define N_TOOLS 8
#endif

// Sanity checks - N_TOOLS may have been defined on the compiler command line.
#if defined(N_TOOLS) && N_TOOLS == 0
#undef N_TOOLS
#endif

#if defined(N_TOOLS) && N_TOOLS > 16
#undef N_TOOLS
#define N_TOOLS 16
#endif

// Max number of entries in log for PID data reporting, to be used for tuning
//#define PID_LOG 1000 // Default disabled. Uncomment to enable.

//#define DEFAULT_X_STEPS_PER_MM 250.0f
//#define DEFAULT_Y_STEPS_PER_MM 250.0f
//#define DEFAULT_Z_STEPS_PER_MM 250.0f
//#define DEFAULT_X_MAX_RATE 500.0f // mm/min
//#define DEFAULT_Y_MAX_RATE 500.0f // mm/min
//#define DEFAULT_Z_MAX_RATE 500.0f // mm/min
//#define DEFAULT_X_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
//#define DEFAULT_Y_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
//#define DEFAULT_Z_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
//#define DEFAULT_X_MAX_TRAVEL 200.0f // mm NOTE: Must be a positive value.
//#define DEFAULT_Y_MAX_TRAVEL 200.0f // mm NOTE: Must be a positive value.
//#define DEFAULT_Z_MAX_TRAVEL 200.0f // mm NOTE: Must be a positive value.
//#define DEFAULT_X_CURRENT 0.0 // amps
//#define DEFAULT_Y_CURRENT 0.0 // amps
//#define DEFAULT_Z_CURRENT 0.0 // amps
//#define DEFAULT_A_CURRENT 0.0 // amps
//#define DEFAULT_SPINDLE_PWM_FREQ 5000 // Hz
//#define DEFAULT_SPINDLE_PWM_OFF_VALUE 0.0f // Percent
//#define DEFAULT_SPINDLE_PWM_MAX_VALUE 100.0f // Percent
//#define DEFAULT_SPINDLE_AT_SPEED_TOLERANCE 0.0f // Percent - 0 means not checked
//#define DEFAULT_SPINDLE_RPM_MAX 1000.0 // rpm
//#define DEFAULT_SPINDLE_RPM_MIN 0.0 // rpm
//#define DEFAULT_SPINDLE_PWM_ACTION 0 // 0 = NONE, 1 = ENABLE_OFF_WITH_ZERO_SPEED, 2 =
//#define DEFAULT_STEP_PULSE_MICROSECONDS 10.0f
//#define DEFAULT_STEP_PULSE_DELAY 5.0f // uncomment to set default > 0.0f
//#define DEFAULT_STEPPER_IDLE_LOCK_TIME 25 // msec (0-65535, 255 keeps steppers enabled)
//#define DEFAULT_JUNCTION_DEVIATION 0.01f // mm
//#define DEFAULT_ARC_TOLERANCE 0.002f // mm
//#define DEFAULT_REPORT_INCHES
//#define DEFAULT_INVERT_LIMIT_BITS
//#define DEFAULT_SOFT_LIMIT_ENABLE
//#define DEFAULT_JOG_LIMIT_ENABLE
//#define DEFAULT_HARD_LIMIT_ENABLE
//#define DEFAULT_INVERT_PROBE_BIT
//#define DEFAULT_LASER_MODE
//#define DEFAULT_LATHE_MODE
//#define DEFAULT_HOMING_ENABLE
//#define DEFAULT_HOMING_ALLOW_MANUAL
//#define DEFAULT_HOMING_DIR_MASK 0 // move positive dir
//#define DEFAULT_HOMING_FEED_RATE 25.0f // mm/min
//#define DEFAULT_HOMING_SEEK_RATE 500.0f // mm/min
//#define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
//#define DEFAULT_HOMING_PULLOFF 1.0f // mm

//#define DEFAULT_A_STEPS_PER_MM 250.0f
//#define DEFAULT_A_MAX_RATE 500.0f // mm/min
//#define DEFAULT_A_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
//#define DEFAULT_A_MAX_TRAVEL 200.0f // mm

//#define DEFAULT_B_STEPS_PER_MM 250.0f
//#define DEFAULT_B_MAX_RATE 500.0f // mm/min
//#define DEFAULT_B_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
//#define DEFAULT_B_MAX_TRAVEL 200.0f // mm

//#define DEFAULT_C_STEPS_PER_MM 250.0f
//#define DEFAULT_C_MAX_RATE 500.0f // mm/min
//#define DEFAULT_C_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
//#define DEFAULT_C_MAX_TRAVEL 200.0f // mm

//#define DEFAULT_G73_RETRACT 0.1f // mm

#ifdef DEFAULT_HOMING_ENABLE

// Number of homing cycles performed after when the machine initially jogs to limit switches.
// This help in preventing overshoot and should improve repeatability. This value should be one or
// greater.
//#define DEFAULT_N_HOMING_LOCATE_CYCLE 1 // Integer (1-127)

// If homing is enabled, homing init lock sets Grbl into an alarm state upon power up or a soft reset.
// This forces the user to perform the homing cycle before doing anything else. This is
// mainly a safety feature to remind the user to home, since position is unknown to Grbl.
//#define DEFAULT_HOMING_INIT_LOCK // Default disabled. Uncomment to enable.

// If homing init lock is enabled this sets Grbl into an alarm state upon power up or a soft reset.
// To allow a soft reset to override the lock uncomment the line below.
//#define DEFAULT_HOMING_OVERRIDE_LOCKS // Default disabled. Uncomment to enable.

// Define the homing cycle patterns with bitmasks. The homing cycle first performs a search mode
// to quickly engage the limit switches, followed by a slower locate mode, and finished by a short
// pull-off motion to disengage the limit switches. The following HOMING_CYCLE_x defines are executed
// in order starting with suffix 0 and completes the homing routine for the specified-axes only. If
// an axis is omitted from the defines, it will not home, nor will the system update its position.
// Meaning that this allows for users with non-standard cartesian machines, such as a lathe (x then z,
// with no y), to configure the homing cycle behavior to their needs.
// NOTE: The homing cycle is designed to allow sharing of limit pins, if the axes are not in the same
// cycle, but this requires some pin settings changes in cpu_map.h file. For example, the default homing
// cycle can share the Z limit pin with either X or Y limit pins, since they are on different cycles.
// By sharing a pin, this frees up a precious IO pin for other purposes. In theory, all axes limit pins
// may be reduced to one pin, if all axes are homed with seperate cycles, or vice versa, all three axes
// on separate pin, but homed in one cycle. Also, it should be noted that the function of hard limits
// will not be affected by pin sharing.
// NOTE: Defaults are set for a traditional 3-axis CNC machine. Z-axis first to clear, followed by X & Y.

//#define HOMING_CYCLE_0 (Z_AXIS_BIT)             // REQUIRED: First move Z to clear workspace.
//#define HOMING_CYCLE_1 (X_AXIS_BIT|Y_AXIS_BIT)  // OPTIONAL: Then move X,Y at the same time.
//#define HOMING_CYCLE_2 0                        // OPTIONAL: Uncomment and add axes mask to enable
#if N_AXIS > 3
//#define HOMING_CYCLE_3 0                        // OPTIONAL: Uncomment and add axes mask to enable
//#define HOMING_CYCLE_4 0                        // OPTIONAL: Uncomment and add axes mask to enable
//#define HOMING_CYCLE_5 0                        // OPTIONAL: Uncomment and add axes mask to enable
#endif

// Enables single axis homing commands. $HX, $HY, and $HZ for X, Y, and Z-axis homing. The full homing
// cycle is still invoked by the $H command. This is disabled by default. It's here only to address
// users that need to switch between a two-axis and three-axis machine. This is actually very rare.
// If you have a two-axis machine, DON'T USE THIS. Instead, just alter the homing cycle for two-axes.
//#define HOMING_SINGLE_AXIS_COMMANDS // Default disabled. Uncomment to enable.

// After homing, Grbl will set by default the entire machine space into negative space, as is typical
// for professional CNC machines, regardless of where the limit switches are located. Set this
// define to 1 to force Grbl to always set the machine origin at the homed location despite switch orientation.
//#define HOMING_FORCE_SET_ORIGIN // Default disabled. Uncomment to enable.
#define HOMING_FORCE_SET_ORIGIN     1

// To prevent the homing cycle from racking the dual axis, when one limit triggers before the
// other due to switch failure or noise, the homing cycle will automatically abort if the second
// motor's limit switch does not trigger within the three distance parameters defined below.
// Axis length percent will automatically compute a fail distance as a percentage of the max
// travel of the other non-dual axis, i.e. if dual axis select is X_AXIS at 5.0%, then the fail
// distance will be computed as 5.0% of y-axis max travel. Fail distance max and min are the
// limits of how far or little a valid fail distance is.
//#define DUAL_AXIS_HOMING_FAIL_AXIS_LENGTH_PERCENT  5.0f  // Float (percent)
//#define DUAL_AXIS_HOMING_FAIL_DISTANCE_MAX  25.0f  // Float (mm)
//#define DUAL_AXIS_HOMING_FAIL_DISTANCE_MIN  2.5f // Float (mm)

// Enables and configures parking motion methods upon a safety door state. Primarily for OEMs
// that desire this feature for their integrated machines. At the moment, Grbl assumes that
// the parking motion only involves one axis, although the parking implementation was written
// to be easily refactored for any number of motions on different axes by altering the parking
// source code. At this time, Grbl only supports parking one axis (typically the Z-axis) that
// moves in the positive direction upon retracting and negative direction upon restoring position.
// The motion executes with a slow pull-out retraction motion, power-down, and a fast park.
// Restoring to the resume position follows these set motions in reverse: fast restore to
// pull-out position, power-up with a time-out, and plunge back to the original position at the
// slower pull-out rate.
// NOTE: Still a work-in-progress. Machine coordinates must be in all negative space and
// does not work with HOMING_FORCE_SET_ORIGIN enabled. Parking motion also moves only in
// positive direction.
//#define DEFAULT_PARKING_ENABLE // Default disabled. Uncomment to enable.

// End default values for run time configurable settings

#endif // DEFAULT_HOMING_ENABLE

// Uncomment to enable experimental support for parameters and expressions
//#define NGC_EXPRESSIONS_ENABLE 1

#ifndef TOOLSETTER_RADIUS
#define TOOLSETTER_RADIUS 5.0f // Default value - do not change here!
#endif

#endif
