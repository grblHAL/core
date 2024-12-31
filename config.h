/*
  config.h - compile time configuration and default setting values

  Part of grblHAL

  Copyright (c) 2020-2024 Terje Io

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

/*! \file
\brief This file contains compile-time and run-time configurations for grblHAL's internal system.
For the most part, users will not need to directly modify these, but they are here for
specific needs, i.e. performance tuning or adjusting to non-typical machines.
<br>__IMPORTANT:__ Symbol/macro names starting with `DEFAULT_` contains default values for run-time
                   configurable settings that can be changed with `$=<setting id>` commands.
                   Any changes to these requires a full re-compiling of the source code to propagate them.
                   A reset of non-volatile storage with `$RST=*` after reflashing is also required.
*/

#ifndef _GRBL_CONFIG_H_
#define _GRBL_CONFIG_H_

// Compile time only default configuration

/*! \def N_AXIS
\brief Defines number of axes supported - minimum 3, maximum 8.
If more than 3 axes are configured a compliant driver and board map file is needed.
 */
#ifndef N_AXIS
#define N_AXIS 3 // Number of axes
#endif

/*! \def AXIS_REMAP_ABC2UVW
\brief Remap `ABC` axis letters to `UVW`
<br>__NOTE:__ Experimental, if more than 3 and less than 7 axes are configured the `ABC`
              axis letters can be remapped to `UWV`.
*/
#if (!defined AXIS_REMAP_ABC2UVW && (N_AXIS > 3 && N_AXIS < 7)) || defined __DOXYGEN__
#define AXIS_REMAP_ABC2UVW Off
#endif

/*! \def N_SPINDLE
\brief Defines number of spindles supported - minimum 1, maximum 32.
*/
#if !defined N_SPINDLE || defined __DOXYGEN__
#define N_SPINDLE 1
#endif

/*! \def N_SYS_SPINDLE
\brief Defines number of simultaneously active spindles supported - minimum 1 (none), maximum 8.
*/
#if !defined N_SYS_SPINDLE || defined __DOXYGEN__
#define N_SYS_SPINDLE 1
#endif

/*! \def BUILD_INFO
\brief Defines string to be output as part of the `$I` or `$I+` command response.
*/
#if !defined BUILD_INFO || defined __DOXYGEN__
#define BUILD_INFO ""
#endif

/*! \def COMPATIBILITY_LEVEL
\brief Define compatibility level with the legacy Grbl 1.1 protocol.

Additional G- and M-codes are not disabled except when level is set to >= 10.
This does not apply to G- and M-codes dependent on driver and/or configuration
settings disabled by setting level > 1.
<br>Set to `0` for reporting itself as "GrblHAL" with protocol extensions enabled.
<br>Set to `1` to disable some extensions, and for reporting itself as "Grbl".
<br>Set to `2` to disable new settings as well, use \#define symbols/macros for setting default values.
<br>These can be found in in this file and in defaults.h.
<br>Set to `10` to also disable new coordinate system offsets (G59.1 - G59.3) and some `$#` report extensions.

__NOTE:__ if switching to a level > 1 please reset non-volatile storage with `$RST=*` after reflashing!
*/
#if !defined COMPATIBILITY_LEVEL || defined __DOXYGEN__
#define COMPATIBILITY_LEVEL 0
#endif

/*! \def ENABLE_SPINDLE_LINEARIZATION
\brief This feature alters the spindle PWM/speed to a nonlinear output with a simple piecewise linear curve.

Useful for spindles that don't produce the right RPM from Grbl's standard spindle PWM
linear model. Requires a solution by the 'fit_nonlinear_spindle.py' script in the /doc/script
folder of the repo. See file comments on how to gather spindle data and run the script to
generate a solution.
*/
#if !defined ENABLE_SPINDLE_LINEARIZATION || defined __DOXYGEN__
#define ENABLE_SPINDLE_LINEARIZATION 0  // Set to 1 to enable spindle RPM linearization. Requires compatible driver if enabled.
#endif

/*! \def SPINDLE_NPWM_PIECES
\brief Number of pieces used for spindle RPM linearization, enabled by setting \ref ENABLE_SPINDLE_LINEARIZATION to 1.
*/
#if !defined SPINDLE_NPWM_PIECES || defined __DOXYGEN__
#define SPINDLE_NPWM_PIECES 4 // Number of pieces for spindle RPM linearization, max 4.
#endif

#include "nuts_bolts.h"

//#define KINEMATICS_API // Uncomment to add HAL entry points for custom kinematics

/*! \def MASLOW_ROUTER
\brief Enable Maslow router kinematics.
Experimental - testing required and homing needs to be worked out.
*/
#if !defined MASLOW_ROUTER || defined __DOXYGEN__
// Enable Maslow router kinematics.
// Experimental - testing required and homing needs to be worked out.
#define MASLOW_ROUTER Off
#endif

/*! \def WALL_PLOTTER
\brief Enable wall plotter kinematics.
Experimental - testing required and homing needs to be worked out.
*/
#if !defined WALL_PLOTTER || defined __DOXYGEN__
#define WALL_PLOTTER Off
#endif

/*! \def DELTA_ROBOT
\brief Enable delta kinematics.
Experimental - testing required and homing needs to be worked out.
*/
#if !defined DELTA_ROBOT || defined __DOXYGEN__
#define DELTA_ROBOT Off
#endif

// Reduce minimum feedrate for delta robots
#if DELTA_ROBOT && !defined MINIMUM_FEED_RATE
#define MINIMUM_FEED_RATE 0.1f // (radians/min)
#endif

/*! \def POLAR_ROBOT
\brief Enable polar kinematics.
Experimental - testing required and homing needs to be worked out.
*/
#if !defined POLAR_ROBOT || defined __DOXYGEN__
#define POLAR_ROBOT Off
#endif


/*! \def COREXY
\brief Enable CoreXY kinematics. Use ONLY with CoreXY machines.
<br>__IMPORTANT:__ If homing is enabled, you must reconfigure the homing cycle \#defines above to
\#define \ref DEFAULT_HOMING_CYCLE_1 `X_AXIS_BIT` and \#define \ref DEFAULT_HOMING_CYCLE_2 `Y_AXIS_BIT`
<br>__NOTE:__ This configuration option alters the motion of the X and Y axes to principle of operation
defined at [corexy.com](http://corexy.com/theory.html). Motors are assumed to positioned and wired exactly as
described, if not, motions may move in strange directions. grblHAL requires the CoreXY A and B motors
have the same steps per mm internally.
*/
#if !defined COREXY || defined __DOXYGEN__
#define COREXY Off
#endif

/*! \def CHECK_MODE_DELAY
\brief
Add a short delay for each block processed in Check Mode to
avoid overwhelming the sender with fast reply messages.
This is likely to happen when streaming is done via a protocol where
the speed is not limited to 115200 baud. An example is native USB streaming.
*/
#if !defined CHECK_MODE_DELAY || defined __DOXYGEN__
#define CHECK_MODE_DELAY 0 // ms
#endif

/*! \def DEBOUNCE_DELAY
\brief
When > 0 adds a short delay when an input changes state to avoid switch bounce
or EMI triggering the related interrupt falsely or too many times.
*/
#if !defined DEBOUNCE_DELAY || defined __DOXYGEN__
#define DEBOUNCE_DELAY 40 // ms
#endif

// ---------------------------------------------------------------------------------------
// ADVANCED CONFIGURATION OPTIONS:

// EXPERIMENTAL OPTIONS

#define ENABLE_PATH_BLENDING Off // Do NOT enable unless working on adding this feature!

#if !defined ENABLE_ACCELERATION_PROFILES || defined __DOXYGEN__
#define ENABLE_ACCELERATION_PROFILES Off // Enable to allow G-Code changeable acceleration profiles.
#endif

#if !defined ENABLE_JERK_ACCELERATION || defined __DOXYGEN__
#define ENABLE_JERK_ACCELERATION Off // Enable to use 3rd order acceleration calculations. May need more processing power, a FPU will help.
#endif

// -

// Enables code for debugging purposes. Not for general use and always in constant flux.
//#define DEBUG // Uncomment to enable. Default disabled.
//#define DEBUGOUT 0 // Uncomment to claim serial port with given instance number and add HAL entry point for debug output.

/*! @name Status report frequency
Some status report data isn't necessary for realtime, only intermittently, because the values don't
change often. The following macros configures how many times a status report needs to be called before
the associated data is refreshed and included in the status report. However, if one of these value
changes, grblHAL will automatically include this data in the next status report, regardless of what the
count is at the time. This helps reduce the communication overhead involved with high frequency reporting
and aggressive streaming. There is also a busy and an idle refresh count, which sets up grblHAL to send
refreshes more often when its not doing anything important. With a good GUI, this data doesn't need
to be refreshed very often, on the order of a several seconds.
<br>__NOTE:__ `WCO` refresh must be 2 or greater. `OVERRIDE` refresh must be 1 or greater.
*/
///@{
#if !defined REPORT_OVERRIDE_REFRESH_BUSY_COUNT || defined __DOXYGEN__
#define REPORT_OVERRIDE_REFRESH_BUSY_COUNT 20   // (1-255)
#endif
#if !defined REPORT_OVERRIDE_REFRESH_IDLE_COUNT || defined __DOXYGEN__
#define REPORT_OVERRIDE_REFRESH_IDLE_COUNT 10   // (1-255) Must be less than or equal to the busy count
#endif
#if !defined REPORT_WCO_REFRESH_BUSY_COUNT || defined __DOXYGEN__
#define REPORT_WCO_REFRESH_BUSY_COUNT 30        // (2-255)
#endif
#if !defined REPORT_WCO_REFRESH_IDLE_COUNT || defined __DOXYGEN__
#define REPORT_WCO_REFRESH_IDLE_COUNT 10        // (2-255) Must be less than or equal to the busy count
#endif
///@}

/*! \def ACCELERATION_TICKS_PER_SECOND
\brief The temporal resolution of the acceleration management subsystem.
A higher number gives smoother
acceleration, particularly noticeable on machines that run at very high feedrates, but may negatively
impact performance. The correct value for this parameter is machine dependent, so it's advised to
set this only as high as needed. Approximate successful values can widely range from 50 to 200 or more.
<br>__NOTE:__ Changing this value also changes the execution time of a segment in the step segment buffer.
When increasing this value, this stores less overall time in the segment buffer and vice versa. Make
certain the step segment buffer is increased/decreased to account for these changes.
*/
#if !defined ACCELERATION_TICKS_PER_SECOND || defined __DOXYGEN__
#define ACCELERATION_TICKS_PER_SECOND 100
#endif

// Sets the maximum step rate allowed to be written as a grblHAL setting. This option enables an error
// check in the settings module to prevent settings values that will exceed this limitation. The maximum
// step rate is strictly limited by the CPU speed and will change if something other than an AVR running
// at 16MHz is used.
// NOTE: For now disabled, will enable if flash space permits.
//#define MAX_STEP_RATE_HZ 30000 // Hz

/*! \def REPORT_ECHO_LINE_RECEIVED
\brief
With this enabled, grblHAL sends back an echo of the line it has received, which has been pre-parsed (spaces
removed, capitalized letters, no comments) and is to be immediately executed by grblHAL. Echoes will not be
sent upon a line buffer overflow, but should for all normal lines sent to grblHAL. For example, if a user
sends the line 'g1 x1.032 y2.45 (test comment)', grblHAL will echo back in the form '[echo: G1X1.032Y2.45]'.
NOTE: Only use this for debugging purposes!! When echoing, this takes up valuable resources and can effect
performance. If absolutely needed for normal operation, the serial write buffer should be greatly increased
to help minimize transmission waiting within the serial write protocol.
 */
#if !defined REPORT_ECHO_LINE_RECEIVED || defined __DOXYGEN__
#define REPORT_ECHO_LINE_RECEIVED Off // Default disabled. Set to \ref On or 1 to enable.
#endif

/*! \def TOOL_LENGTH_OFFSET_AXIS
\brief Sets which \ref axis the tool length offset is applied.
Assumes the spindle is always parallel with the selected axis with the tool oriented toward
the negative direction. In other words, a positive tool length offset value is subtracted
from the current location.
*/
#if !defined TOOL_LENGTH_OFFSET_AXIS || defined __DOXYGEN__
#define TOOL_LENGTH_OFFSET_AXIS -1 // Default is all axes.
#endif

/*! \def MINIMUM_JUNCTION_SPEED
\brief Minimum planner junction speed.
Sets the default minimum junction speed the planner plans to at every buffer block junction,
except for starting from rest and end of the buffer, which are always zero. This value controls
how fast the machine moves through junctions with no regard for acceleration limits or angle
between neighboring block line move directions. This is useful for machines that can't
tolerate the tool dwelling for a split second, i.e. 3d printers or laser cutters. If used, this value
should not be much greater than zero or to the minimum value necessary for the machine to work.
*/
#if !defined MINIMUM_JUNCTION_SPEED || defined __DOXYGEN__
#define MINIMUM_JUNCTION_SPEED 0.0f // (mm/min)
#endif

/*! \def MINIMUM_FEED_RATE
\brief
Sets the minimum feed rate the planner will allow. Any value below it will be set to this minimum
value. This also ensures that a planned motion always completes and accounts for any floating-point
round-off errors. Although not recommended, a lower value than 1.0 mm/min will likely work in smaller
machines, perhaps to 0.1mm/min, but your success may vary based on multiple factors.
*/
#if !defined MINIMUM_FEED_RATE || defined __DOXYGEN__
#define MINIMUM_FEED_RATE 1.0f // (mm/min)
#endif

/*! \def N_ARC_CORRECTION
\brief
Number of arc generation iterations by small angle approximation before exact arc trajectory
correction with expensive sin() and cos() calculations. This parameter maybe decreased if there
are issues with the accuracy of the arc generations, or increased if arc execution is getting
bogged down by too many trig calculations.
*/
#if !defined N_ARC_CORRECTION || defined __DOXYGEN__
#define N_ARC_CORRECTION 12 // Integer (1-255)
#endif

/*! \def ARC_ANGULAR_TRAVEL_EPSILON
\brief
The arc G2/3 g-code standard is problematic by definition. Radius-based arcs have horrible numerical
errors when arc at semi-circles(pi) or full-circles(2*pi). Offset-based arcs are much more accurate
but still have a problem when arcs are full-circles (2*pi). This define accounts for the floating
point issues when offset-based arcs are commanded as full circles, but get interpreted as extremely
small arcs with around machine epsilon (1.2e-7rad) due to numerical round-off and precision issues.
This define value sets the machine epsilon cutoff to determine if the arc is a full-circle or not.
<br>__NOTE:__ Be very careful when adjusting this value. It should always be greater than 1.2e-7 but not too
much greater than this. The default setting should capture most, if not all, full arc error situations.
*/
#if !defined ARC_ANGULAR_TRAVEL_EPSILON || defined __DOXYGEN__
#define ARC_ANGULAR_TRAVEL_EPSILON 5E-7f // Float (radians)
#endif

/*! @name Default constants for G5 Cubic splines
//
*/
///@{
#if !defined BEZIER_MIN_STEP || defined __DOXYGEN__
#define BEZIER_MIN_STEP 0.002f
#endif
#if !defined BEZIER_MAX_STEP || defined __DOXYGEN__
#define BEZIER_MAX_STEP 0.1f
#endif
#if !defined BEZIER_SIGMA || defined __DOXYGEN__
#define BEZIER_SIGMA 0.1f
#endif
///@}


/*! \def DWELL_TIME_STEP
\brief Time delay increments performed during a dwell.
The default value is set at 50ms, which provides a maximum time delay of roughly 55 minutes,
more than enough for most any application. Increasing this delay will increase the maximum
dwell time linearly, but also reduces the responsiveness of run-time command executions,
like status reports, since these are performed between each dwell time step.
*/
#if !defined DWELL_TIME_STEP || defined __DOXYGEN__
#define DWELL_TIME_STEP 50 // Integer (1-255) (milliseconds)
#endif

/*! \def SEGMENT_BUFFER_SIZE
\brief
Governs the size of the intermediary step segment buffer between the step execution algorithm
and the planner blocks. Each segment is set of steps executed at a constant velocity over a
fixed time defined by \ref ACCELERATION_TICKS_PER_SECOND. They are computed such that the planner
block velocity profile is traced exactly. The size of this buffer governs how much step
execution lead time there is for other grblHAL processes have to compute and do their thing
before having to come back and refill this buffer, currently at ~50msec of step moves.
*/
#if !defined SEGMENT_BUFFER_SIZE || defined __DOXYGEN__
#define SEGMENT_BUFFER_SIZE 10 // Uncomment to override default in stepper.h.
#endif

/*! \def SET_CHECK_MODE_PROBE_TO_START
\brief
Configures the position after a probing cycle during grblHAL's check mode. Disabled sets
the position to the probe target, when enabled sets the position to the start position.
*/
#if !defined SET_CHECK_MODE_PROBE_TO_START || defined __DOXYGEN__
#define SET_CHECK_MODE_PROBE_TO_START Off // Default disabled. Set to \ref On or 1 to enable.
#endif

/*! \def HARD_LIMIT_FORCE_STATE_CHECK
\brief
Force grblHAL to check the state of the hard limit switches when the processor detects a pin
change inside the hard limit ISR routine. By default, grblHAL will trigger the hard limits
alarm upon any pin change, since bouncing switches can cause a state check like this to
misread the pin. When hard limits are triggered, they should be 100% reliable, which is the
reason that this option is disabled by default. Only if your system/electronics can guarantee
that the switches don't bounce, we recommend enabling this option. This will help prevent
triggering a hard limit when the machine disengages from the switch.
<br>__NOTE:__ This option has no effect if SOFTWARE_DEBOUNCE is enabled.
*/
#if !defined HARD_LIMIT_FORCE_STATE_CHECK || defined __DOXYGEN__
#define HARD_LIMIT_FORCE_STATE_CHECK Off // Default disabled. Set to \ref On or 1 to enable.
#endif

/*! @name Homing cycle search and locate scalars
\brief
Adjusts homing cycle search and locate scalars. These are the multipliers used by grblHAL's
homing cycle to ensure the limit switches are engaged and cleared through each phase of
the cycle. The search phase uses the axes max-travel setting times the
`HOMING_AXIS_SEARCH_SCALAR` to determine distance to look for the limit switch. Once found,
the locate phase begins and uses the homing pull-off distance setting times the
`HOMING_AXIS_LOCATE_SCALAR` to pull-off and re-engage the limit switch.
<br>__NOTE:__ Both of these values must be greater than 1.0 to ensure proper function.
*/
///@{
#if !defined HOMING_AXIS_SEARCH_SCALAR || defined __DOXYGEN__
#define HOMING_AXIS_SEARCH_SCALAR  1.5f // Must be > 1 to ensure limit switch will be engaged.
#endif
#if !defined HOMING_AXIS_LOCATE_SCALAR || defined __DOXYGEN__
#define HOMING_AXIS_LOCATE_SCALAR  10.0f// Must be > 1 to ensure limit switch is cleared.
#endif
///@}

/*! @name Non-volatile storage restore commands
\brief
Enable the `$RST=*`, `$RST=$`, `$RST=#` and `$RST=&` non-volatile storage restore commands. There are
cases where these commands may be undesirable. Simply change desired macro to \ref Off or 0 to disable it.
<br>__NOTE:__ See _Non-volatile storage restore options_ below for customizing the `$RST=*` command.
*/
///@{
#if !defined ENABLE_RESTORE_NVS_WIPE_ALL || defined __DOXYGEN__
#define ENABLE_RESTORE_NVS_WIPE_ALL On //!< `$RST=*` Default enabled. Set to \ref Off or 0 to disable.
#endif
#if !defined ENABLE_RESTORE_NVS_DEFAULT_SETTINGS || defined __DOXYGEN__
#define ENABLE_RESTORE_NVS_DEFAULT_SETTINGS On //!< `$RST=$`  Default enabled. Set to \ref Off or 0 to disable.
#endif
#if !defined ENABLE_RESTORE_NVS_CLEAR_PARAMETERS || defined __DOXYGEN__
#define ENABLE_RESTORE_NVS_CLEAR_PARAMETERS On //!< `$RST=#`  Default enabled. Set to \ref Off or 0 to disable.
#endif
#if !defined ENABLE_RESTORE_NVS_DRIVER_PARAMETERS || defined __DOXYGEN__
#define ENABLE_RESTORE_NVS_DRIVER_PARAMETERS On //!< `$RST=&` Default enabled. Set to \ref Off or 0 to disable. For drivers that implements non-generic settings.
#endif
///@}

/*! @name Non-volatile storage restore options
\brief
// Defines the non-volatile data restored upon a settings version change and `$RST=*` command. Whenever the
// the settings or other non-volatile data structure changes between grblHAL versions, grblHAL will automatically
// wipe and restore the non-volatile data. These macros controls what data is wiped and restored. This is useful
// particularly for OEMs that need to retain certain data. For example, the \ref BUILD_INFO string can be
// written into non-volatile storage via a separate program to contain product data. Altering these
// macros to not restore the build info non-volatile storage will ensure this data is retained after firmware upgrades.
*/
///@{
#if !defined SETTINGS_RESTORE_DEFAULTS || defined __DOXYGEN__
#define SETTINGS_RESTORE_DEFAULTS On //!< Default enabled. Set to \ref Off or 0 to disable.
#endif
#if !defined SETTINGS_RESTORE_PARAMETERS || defined __DOXYGEN__
#define SETTINGS_RESTORE_PARAMETERS On //!< Default enabled. Set to \ref Off or 0 to disable.
#endif
#if !defined SETTINGS_RESTORE_STARTUP_LINES || defined __DOXYGEN__
#define SETTINGS_RESTORE_STARTUP_LINES On //!< Default enabled. Set to \ref Off or 0 to disable.
#endif
#if !defined SETTINGS_RESTORE_BUILD_INFO || defined __DOXYGEN__
#define SETTINGS_RESTORE_BUILD_INFO  On //!< Default enabled. Set to \ref Off or 0 to disable.
#endif
#if !defined SETTINGS_RESTORE_DRIVER_PARAMETERS || defined __DOXYGEN__
#define SETTINGS_RESTORE_DRIVER_PARAMETERS On //!< Default enabled. Set to \ref Off or 0 to disable.
#endif
/*! \def DISABLE_BUILD_INFO_WRITE_COMMAND
\brief Disable the `$I=(string)` build info write command.
If disabled, any existing build info data must be placed into non-volatile storage via external
means with a valid checksum value. This macro option is useful to prevent this data from being
over-written by a user, when used to store OEM product data.
<br>__NOTE:__ If disabled and to ensure grblHAL can never alter the build info line, you'll also need
to set the \ref SETTINGS_RESTORE_BUILD_INFO symbol to \ref Off or 0.
*/
#if !defined DISABLE_BUILD_INFO_WRITE_COMMAND || defined __DOXYGEN__
#define DISABLE_BUILD_INFO_WRITE_COMMAND Off //!< `$I=` Default enabled. Uncomment to disable.
#endif
///@}

/*! \def SLEEP_DURATION
\brief Configures grblHAL's sleep mode feature.
If the spindle or coolant are powered and grblHAL is not actively moving or receiving any
commands, a sleep timer will start. If any data or commands are received, the sleep timer
will reset and restart until the above condition are not satisfied. If the sleep timer elaspes,
grblHAL will immediately execute the sleep mode by shutting down the spindle and coolant and
entering a safe sleep state. If parking is enabled, grblHAL will park the machine as well.
While in sleep mode, only a hard/soft reset will exit it and the job will be unrecoverable.
Sleep mode is enabled by setting \ref DEFAULT_SLEEP_ENABLE to \ref On or 1, overridable by the
`$62` setting.
<br>__NOTE:__ Sleep mode is a safety feature, primarily to address communication disconnect problems. To
keep grblHAL from sleeping, employ a stream of '?' status report commands as a connection "heartbeat".
*/
#if !defined SLEEP_DURATION || defined __DOXYGEN__
#define SLEEP_DURATION 5.0f // Number of minutes before sleep mode is entered.
#endif

/*! \def NVSDATA_BUFFER_ENABLE
\brief Disable non-volatile storage (NVS) emulation/buffering in RAM (allocated from heap memory).
The NVS buffer can be used for MCUs with no non-volatile storage or for delaying writing to
non-volatile storage until the controller is in IDLE state.
*/
#if !defined NVSDATA_BUFFER_ENABLE || defined __DOXYGEN__
#define NVSDATA_BUFFER_ENABLE On // Default on, set to \ref off or 0 to disable.
#endif

/*! \def TOOLSETTER_RADIUS
\brief
The grbl.on_probe_toolsetter event handler is called by the default tool change algorithm when probing at G59.3.
In addition it will be called on a "normal" probe sequence if the XY position is
within the radius of the G59.3 position defined below.
Change if the default value of 5mm is not suitable or set it to 0.0f to disable.
<br>__NOTE:__ A grbl.on_probe_toolsetter event handler is not installed by the core, it has to be provided
by a driver or a plugin.
*/
#if !defined TOOLSETTER_RADIUS || defined __DOXYGEN__
#define TOOLSETTER_RADIUS 5.0f
#endif

#if !defined ENABLE_BACKLASH_COMPENSATION || defined __DOXYGEN__
#define ENABLE_BACKLASH_COMPENSATION Off
#endif

#if COMPATIBILITY_LEVEL == 0 || defined __DOXYGEN__
/*! \def N_TOOLS
\brief
Number of tools in tool table, edit to enable (max. 32 allowed)
*/
#if !defined N_TOOLS || defined __DOXYGEN__
#define N_TOOLS 0
#endif
#endif

/*! \def NGC_EXPRESSIONS_ENABLE
\brief
Set to \ref On or 1 to enable experimental support for expressions.

Some LinuxCNC extensions are supported, conditionals and subroutines are not.
*/
#if !defined NGC_EXPRESSIONS_ENABLE || defined __DOXYGEN__
#define NGC_EXPRESSIONS_ENABLE Off
#endif

/*! \def NGC_PARAMETERS_ENABLE
\brief
Set to \ref On or 1 to enable experimental support for parameters.
*/
#if !defined NGC_PARAMETERS_ENABLE || defined __DOXYGEN__
#define NGC_PARAMETERS_ENABLE On
#endif

/*! \def NGC_N_ASSIGN_PARAMETERS_PER_BLOCK
\brief
Maximum number of parameters allowed in a block.
*/
#if (NGC_EXPRESSIONS_ENABLE && !defined NGC_N_ASSIGN_PARAMETERS_PER_BLOCK) || defined __DOXYGEN__
#define NGC_N_ASSIGN_PARAMETERS_PER_BLOCK 10
#endif

/*! \def LATHE_UVW_OPTION
\brief
Allow use of UVW axis words for non-modal relative lathe motion.
*/
#if !defined LATHE_UVW_OPTION || defined __DOXYGEN__
#define LATHE_UVW_OPTION Off
#endif

// Max number of entries in log for PID data reporting, to be used for tuning
//#define PID_LOG 1000 // Default disabled. Uncomment to enable.

// End compile time only default configuration

// ---------------------------------------------------------------------------------------
// SETTINGS DEFAULT VALUE OVERRIDES:

// General settings (Group_General)

/*! @name $10 - Setting_StatusReportMask
The status report change for grblHAL v1.1 and after also removed the ability to disable/enable most data
fields from the report. This caused issues for GUI developers, who've had to manage several scenarios
and configurations. The increased efficiency of the new reporting style allows for all data fields to
be sent without potential performance issues.
<br>__NOTE:__ The options below are here only provide a way to disable certain data fields if a unique
situation demands it, but be aware GUIs may depend on this data. If disabled, it may not be compatible.
*/
///@{
/*! \def DEFAULT_REPORT_MACHINE_POSITION
\brief
If set to \ref Off or 0 position is reported with all offsets added.
\internal Bit 0 in settings.status_report.
*/
#if !defined DEFAULT_REPORT_MACHINE_POSITION || defined __DOXYGEN__
#define DEFAULT_REPORT_MACHINE_POSITION On // Default on. Set to \ref Off or 0 to disable.
#endif

/*! \def DEFAULT_REPORT_BUFFER_STATE
\brief
If set to \ref Off or 0 the `|Bf:` buffer state element is not included in the real time report.
\internal Bit 1 in settings.status_report.
*/
#if !defined DEFAULT_REPORT_BUFFER_STATE || defined __DOXYGEN__
#define DEFAULT_REPORT_BUFFER_STATE On // Default on. Set to \ref Off or 0 to disable.
#endif

/*! \def DEFAULT_REPORT_LINE_NUMBERS
\brief
If set to \ref Off or 0 the `|Ln:` line number element is not included
in the real time report.
<br>__NOTE:__ Line numbers are only reported if present in the gcode.
\internal Bit 2 in settings.status_report.
*/
#if !defined DEFAULT_REPORT_LINE_NUMBERS || defined __DOXYGEN__
#define DEFAULT_REPORT_LINE_NUMBERS On // Default on. Set to \ref Off or 0 to disable.
#endif

/*! \def DEFAULT_REPORT_CURRENT_FEED_SPEED
\brief
If set to \ref Off or 0 the `|FS:` current feed & speed element is not included
in the real time report.
\internal Bit 3 in settings.status_report.
*/
#if !defined DEFAULT_REPORT_CURRENT_FEED_SPEED || defined __DOXYGEN__
#define DEFAULT_REPORT_CURRENT_FEED_SPEED On // Default on. Set to \ref Off or 0 to disable.
#endif

/*! \def DEFAULT_REPORT_PIN_STATE
\brief
If set to \ref Off or 0 the `|Pn:` input pins state element is not included
in the real time report.
\internal Bit 4 in settings.status_report.
*/
#if !defined DEFAULT_REPORT_PIN_STATE || defined __DOXYGEN__
#define DEFAULT_REPORT_PIN_STATE On // Default on. Set to \ref Off or 0 to disable.
#endif

/*! \def DEFAULT_REPORT_WORK_COORD_OFFSET
\brief
If set to \ref Off or 0 the `|WCO:` work coordinate offset element is not included
in the real time report.
\internal Bit 5 in settings.status_report.
*/
#if !defined DEFAULT_REPORT_WORK_COORD_OFFSET || defined __DOXYGEN__
#define DEFAULT_REPORT_WORK_COORD_OFFSET On // Default on. Set to \ref Off or 0 to disable.
#endif

/*! \def DEFAULT_REPORT_OVERRIDES
\brief
If set to \ref Off or 0 the `|Pn:` input pins state element is not included
in the real time report.
\internal Bit 6 in settings.status_report.
*/
#if !defined DEFAULT_REPORT_OVERRIDES || defined __DOXYGEN__
#define DEFAULT_REPORT_OVERRIDES On // Default on. Set to \ref Off or 0 to disable.
#endif

/*! \def DEFAULT_REPORT_PROBE_COORDINATES
\brief
Upon a successful probe cycle, this option provides immediately feedback of the probe coordinates
through an automatically generated message. If disabled, users can still access the last probe
coordinates through grblHAL `$#` print parameters command.
\internal Bit 7 in settings.status_report.
*/
#if !defined DEFAULT_REPORT_PROBE_COORDINATES || defined __DOXYGEN__
#define DEFAULT_REPORT_PROBE_COORDINATES On // Default on. Set to \ref Off or 0 to disable.
#endif

/*! \def DEFAULT_REPORT_SYNC_ON_WCO_CHANGE
\brief
In Grbl v0.9 and prior, there is an old outstanding bug where the `WPos:` work position reported
may not correlate to what is executing, because `WPos:` is based on the g-code parser state, which
can be several motions behind. This option forces the planner buffer to empty, sync, and stop
motion whenever there is a command that alters the work coordinate offsets `G10,G43.1,G92,G54-59.3`.
This is the simplest way to ensure `WPos:` is always correct. Fortunately, it's exceedingly rare
that any of these commands are used need continuous motions through them.
\internal Bit 8 in settings.status_report.
*/
#if !defined DEFAULT_REPORT_SYNC_ON_WCO_CHANGE || defined __DOXYGEN__
#define DEFAULT_REPORT_SYNC_ON_WCO_CHANGE On //!< ok
#endif

/*! \def DEFAULT_REPORT_PARSER_STATE
\brief
When enabled adds automatic report of the parser state following a status report request
if the state was changed since the last report. The output is the same as provided by
the `$G` command.
\internal Bit 9 in settings.status_report.
*/
#if !defined DEFAULT_REPORT_PARSER_STATE || defined __DOXYGEN__
#define DEFAULT_REPORT_PARSER_STATE Off // Default off. Set to \ref On or 1 to enable.
#endif

/*! \def DEFAULT_REPORT_ALARM_SUBSTATE
\brief
Many controllers cannot be hard reset on startup due to using native USB or network
protocols for communication. If the grblHAL for some reason is in `ALARM` state there is
normally no way to determine the cause of the alarm. Enabling this setting adds the alarm
code (see \ref alarm_code_t) as a substate, separated by a colon, to the _Alarm_ state in
the real time report.
<br>__NOTE:__ Enabling this option may break senders.
\internal Bit 10 in settings.status_report.
*/
#if !defined DEFAULT_REPORT_ALARM_SUBSTATE || defined __DOXYGEN__
#define DEFAULT_REPORT_ALARM_SUBSTATE Off // Default off. Set to \ref On or 1 to enable.
#endif

/*! \def DEFAULT_REPORT_RUN_SUBSTATE
\brief
Enabling this setting may add a code, separated by a colon, to the _Run_ state in the real time report.
The following codes are defined:
+ `1` - a feed hold is pending, waiting for spindle synchronized motion to complete.
+ `2` - the motion is a probe.
<br>__NOTE:__ Enabling this option may break senders.
\internal Bit 11 in settings.status_report.
*/
#if !defined DEFAULT_REPORT_RUN_SUBSTATE || defined __DOXYGEN__
#define DEFAULT_REPORT_RUN_SUBSTATE Off // Default off. Set to \ref On or 1 to enable.
#endif

/*! \def DEFAULT_REPORT_WHEN_HOMING
\brief
Enabling this setting enables status reporting while homing.
<br>__NOTE:__ Enabling this option may break senders.
\internal Bit 12 in settings.status_report.
*/
#if !defined DEFAULT_REPORT_WHEN_HOMING || defined __DOXYGEN__
#define DEFAULT_REPORT_WHEN_HOMING Off // Default off. Set to \ref On or 1 to enable.
#endif

///@}

/*! @name $11 - Setting_JunctionDeviation
 *
 */
///@{
#if !defined DEFAULT_JUNCTION_DEVIATION || defined __DOXYGEN__
#define DEFAULT_JUNCTION_DEVIATION 0.01f // mm
#endif
///@}

/*! @name $12 - Setting_ArcTolerance
 *
 */
///@{
#if !defined DEFAULT_ARC_TOLERANCE || defined __DOXYGEN__
#define DEFAULT_ARC_TOLERANCE 0.002f // mm
#endif
///@}

/*! @name $13 - Setting_ReportInches
If set to \ref On or 1 reported positions, offsets etc will be converted to inches
with 4 digits of precision.
*/
///@{
#if !defined DEFAULT_REPORT_INCHES || defined __DOXYGEN__
#define DEFAULT_REPORT_INCHES Off
#endif
///@}

/*! @name $28 - Setting_G73Retract
The retract motion distance used for chip breaking by the `G73` canned cycle, executed
after each delta increment specified by the `Q` word.
*/
///@{
#if !defined DEFAULT_G73_RETRACT || defined __DOXYGEN__
#define DEFAULT_G73_RETRACT 0.1f // mm
#endif
///@}

/*! @name $32 - Setting_Mode
__NOTE:__ only one mode can be enabled.
 */
///@{
#if !defined DEFAULT_LASER_MODE || defined __DOXYGEN__
#define DEFAULT_LASER_MODE Off
#endif
#if !defined DEFAULT_LATHE_MODE || defined __DOXYGEN__
#define DEFAULT_LATHE_MODE Off
#endif
///@}

/*! @name $39 - Setting_EnableLegacyRTCommands
Using printable ASCII characters for realtime commands can cause issues with files
containing such characters in comments or settings. If the GCode sender support the
use of the top-bit set alternatives for these then they may be disabled.
<br>__NOTE:__ support for the top-bit set alternatives is always enabled.
<br>__NOTE:__ when disabled they are still active outside of comments and $ settings
allowing their use from manual input, eg. from a terminal or MDI.
*/
///@{
#if !defined DEFAULT_LEGACY_RTCOMMANDS || defined __DOXYGEN__
#define DEFAULT_LEGACY_RTCOMMANDS On
#endif
///@}

/*! @name $60 - Setting_RestoreOverrides
*/
///@{
#if !defined DEFAULT_RESET_OVERRIDES || defined __DOXYGEN__
#define DEFAULT_RESET_OVERRIDES Off
#endif
///@}

/*! @name $62 - Setting_SleepEnable
*/
///@{
#if !defined DEFAULT_SLEEP_ENABLE || defined __DOXYGEN__
#define DEFAULT_SLEEP_ENABLE Off
#endif
///@}

/*! @name $63 - Setting_HoldActions
This option will automatically disable the laser during a feed hold by invoking a spindle stop
override immediately after coming to a stop. However, this also means that the laser still may
be re-enabled by disabling the spindle stop override, if needed. This is purely a safety feature
to ensure the laser doesn't inadvertently remain powered while at a stop and cause a fire.
*/
///@{
#if !defined DEFAULT_DISABLE_LASER_DURING_HOLD || defined __DOXYGEN__
#define DEFAULT_DISABLE_LASER_DURING_HOLD On
#endif
///@}

/*! @name This option is for what should happen on resume from feed hold.
Default action is to restore spindle and coolant status (if overridden), this contradicts the
behaviour of industrial controllers but is in line with earlier versions of Grbl.
*/
///@{
#if !defined DEFAULT_RESTORE_AFTER_FEED_HOLD || defined __DOXYGEN__
#define DEFAULT_RESTORE_AFTER_FEED_HOLD On
#endif
///@}

/*! @name $64 - Setting_ForceInitAlarm
When grblHAL powers-cycles or is hard reset with the MCU reset button, grblHAL boots up with no ALARM
by default. This is to make it as simple as possible for new users to start using grblHAL. When homing
is enabled and a user has installed limit switches, grblHAL will boot up in an ALARM state to indicate
grblHAL doesn't know its position and to force the user to home before proceeding. This option forces
grblHAL to always initialize into an ALARM state regardless of homing or not. This option is more for
OEMs and LinuxCNC users that would like this power-cycle behavior.
*/
///@{
#if !defined DEFAULT_FORCE_INITIALIZATION_ALARM || defined __DOXYGEN__
#define DEFAULT_FORCE_INITIALIZATION_ALARM Off
#endif
///@}

/*! @name $384 - Setting_DisableG92Persistence
G92 offsets is by default stored to non-volatile storage (NVS) on changes and restored on startup
if \ref COMPATIBILITY_LEVEL is <= 1. If \ref COMPATIBILITY_LEVEL is <= 1 then setting $384 can be used to change this at run-time.
To allow store/restore of the G92 offset when \ref COMPATIBILITY_LEVEL > 1 uncomment the line below and reset settings with $RST=*.
*/
///@{
#if !defined DEFAULT_DISABLE_G92_PERSISTENCE || defined __DOXYGEN__
#if COMPATIBILITY_LEVEL <= 1
#define DEFAULT_DISABLE_G92_PERSISTENCE Off
#else
#define DEFAULT_DISABLE_G92_PERSISTENCE On
#endif
#endif
///@}

/*! @name $398 - Setting_PlannerBlocks
\brief The number of linear motions in the planner buffer to be planned at any give time.
The vast majority of RAM that grblHAL uses is based on this buffer size. Only increase if
there is extra available RAM, like when compiling for MCU with ample amounts of RAM.
Or decrease if the MCU begins to crash due to the lack of available RAM or if the CPU is
having trouble keeping up with planning new incoming motions as they are executed.
 */
///@{
#if !defined DEFAULT_PLANNER_BUFFER_BLOCKS || defined __DOXYGEN__
#define DEFAULT_PLANNER_BUFFER_BLOCKS 100
#endif
///@}

// Control signals settings (Group_ControlSignals)

#ifndef __DOXYGEN__ // For now do not include in documentation

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

#endif

/*! @name $14 - Setting_ControlInvertMask
Inverts logic of the control input signals based on a \ref signalmask. This essentially means you are using
normally-open (NO) switches on the specified pins, rather than the default normally-closed (NC) switches.
<br>__NOTE:__ See above for other signal definitions.
*/
///@{
#if !defined DEFAULT_CONTROL_SIGNALS_INVERT_MASK || defined __DOXYGEN__
#define DEFAULT_CONTROL_SIGNALS_INVERT_MASK 0 // Set to SIGNALS_BITMASK or -1 to invert all signals
#endif
///@}

/*! @name $17 - Setting_ControlPullUpDisableMask
By default, grblHAL sets all input pins to normal-low operation with their internal pull-up resistors
enabled. This simplifies the wiring for users by requiring only a normally closed (NC) switch connected
to ground. It is not recommended to use normally-open (NO) switches as this increases the risk
of electrical noise spuriously triggering the inputs. If normally-open (NO) switches are used the
logic of the input signals should be be inverted with the invert settings below.
The following options disable the internal pull-up resistors, and switches must be now connect to Vcc
instead of ground.
<br>__WARNING:__ When the pull-ups are disabled, this might require additional wiring with pull-down resistors!
Please check driver code and/or documentation.
<br>__NOTE:__ The first example will disable pull-up for all control pins. The second is an example of
disabling pull-up for only a few pins. See above for other signal definitions.
*/
///@{
#if !defined DEFAULT_DISABLE_CONTROL_PINS_PULL_UP_MASK || defined __DOXYGEN__
#define DEFAULT_DISABLE_CONTROL_PINS_PULL_UP_MASK 0 // Set to SIGNALS_BITMASK or -1 to invert all signals
#endif
///@}

// Limits settings (Group_Limits)

/*! @name $5 - Setting_LimitPinsInvertMask
By default, grblHAL sets all input pins to normal-low operation with their internal pull-up resistors
enabled. This simplifies the wiring for users by requiring only a normally closed (NC) switch connected
to ground. It is _not_ recommended to use normally-open (NO) switches as this increases the risk
of electrical noise or cable breaks spuriously triggering the inputs. If normally-open (NO) switches
are used the logic of the input signals should be be inverted with the \ref axismask below.
*/
///@{
#if !defined DEFAULT_LIMIT_SIGNALS_INVERT_MASK || defined __DOXYGEN__
#define DEFAULT_LIMIT_SIGNALS_INVERT_MASK 0 // Set to -1 or AXES_BITMASK to invert for all axes
#endif
///@}

/*! @name $18 - Setting_LimitPullUpDisableMask
The following options disable the internal pull-up resistors by \ref axismask, and switches must
be now connect to Vcc instead of ground.
<br>__WARNING:__ When the pull-ups are disabled, this might require additional wiring with
                 pull-down resistors! Please check driver code and/or documentation.
*/
///@{
#if !defined DEFAULT_LIMIT_SIGNALS_PULLUP_DISABLE_MASK || defined __DOXYGEN__
#define DEFAULT_LIMIT_SIGNALS_PULLUP_DISABLE_MASK 0 // Set to -1 or AXES_BITMASK to disable pullup for all axes
#endif
///@}

/*! @name $20 - Setting_SoftLimitsEnable
*/
///@{
#if !defined DEFAULT_SOFT_LIMIT_ENABLE || defined __DOXYGEN__
#define DEFAULT_SOFT_LIMIT_ENABLE Off
#endif
///@}

/*! @name $21 - Setting_HardLimitsEnable
*/
///@{
/*! \def DEFAULT_HARD_LIMIT_ENABLE
\brief
At power-up or a reset, grblHAL will check the limit switch states to ensure they are not active
before initialization. If it detects a problem and the hard limits setting is enabled, grblHAL will
simply message the user to check the limits and enter an alarm state, rather than idle. grblHAL will
not throw an alarm message.
*/
#if !defined DEFAULT_HARD_LIMIT_ENABLE || defined __DOXYGEN__
#define DEFAULT_HARD_LIMIT_ENABLE Off
#endif
#if !defined DEFAULT_CHECK_LIMITS_AT_INIT || defined __DOXYGEN__
#define DEFAULT_CHECK_LIMITS_AT_INIT Off
#endif
#if !defined DEFAULT_HARD_LIMITS_DISABLE_FOR_ROTARY || defined __DOXYGEN__
#define DEFAULT_HARD_LIMITS_DISABLE_FOR_ROTARY Off
#endif

/*! @name Group_Limits_DualAxis
\brief Dual axis limits settings (Group_Limits_DualAxis)

To prevent the homing cycle from racking the dual axis, when one limit triggers before the
other due to switch failure or noise, the homing cycle will automatically abort if the second
motor's limit switch does not trigger within the three distance parameters defined below.
Axis length percent will automatically compute a fail distance as a percentage of the max
travel of the other non-dual axis, i.e. if dual axis select is X_AXIS at 5.0%, then the fail
distance will be computed as 5.0% of y-axis max travel. Fail distance max and min are the
limits of how far or little a valid fail distance is.
*/
///@{
/*! @name $347 - Setting_DualAxisLengthFailPercent
*/
///@{
#if !defined DEFAULT_DUAL_AXIS_HOMING_FAIL_AXIS_LENGTH_PERCENT || defined __DOXYGEN__
#define DEFAULT_DUAL_AXIS_HOMING_FAIL_AXIS_LENGTH_PERCENT 5.0f  // Float (percent)
#endif
///@}

/*! @name $348 - Setting_DualAxisLengthFailMin
*/
///@{
#if !defined DEFAULT_DUAL_AXIS_HOMING_FAIL_DISTANCE_MIN || defined __DOXYGEN__
#define DEFAULT_DUAL_AXIS_HOMING_FAIL_DISTANCE_MIN 2.5f // Float (mm)
#endif
///@}

/*! @name $348 - Setting_DualAxisLengthFailMin

 */
///@{
#if !defined DEFAULT_DUAL_AXIS_HOMING_FAIL_DISTANCE_MAX || defined __DOXYGEN__
#define DEFAULT_DUAL_AXIS_HOMING_FAIL_DISTANCE_MAX 25.0f  // Float (mm)
#endif
///@}

// Coolant settings (Group_Coolant)

/*! @name $15 - Setting_CoolantInvertMask
Inverts the selected coolant signals from active high to active low.
Useful for some pre-built electronic boards.
*/
///@{
#if !defined DEFAULT_INVERT_COOLANT_FLOOD_PIN || defined __DOXYGEN__
#define DEFAULT_INVERT_COOLANT_FLOOD_PIN Off
#endif
#if !defined DEFAULT_INVERT_COOLANT_MIST_PIN || defined __DOXYGEN__
#define DEFAULT_INVERT_COOLANT_MIST_PIN Off  // NOTE: not supported by all drivers.
#endif
///@}

// Spindle settings (Group_Spindle)

/*! @name $9 - Setting_SpindlePWMOptions
*/
///@{
#if !defined DEFAULT_SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED || defined __DOXYGEN__
#define DEFAULT_SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED Off
#endif
#if !defined DEFAULT_PWM_SPINDLE_DISABLE_LASER_MODE || defined __DOXYGEN__
#define DEFAULT_PWM_SPINDLE_DISABLE_LASER_MODE Off
#endif
///@}

/*! @name $16 - Setting_SpindleInvertMask
Inverts the selected spindle output signals from active high to active low. Useful for some pre-built electronic boards.
*/
///@{
#if !defined DEFAULT_INVERT_SPINDLE_ENABLE_PIN || defined __DOXYGEN__
#define DEFAULT_INVERT_SPINDLE_ENABLE_PIN Off
#endif
#if !defined DEFAULT_INVERT_SPINDLE_CCW_PIN || defined __DOXYGEN__
#define DEFAULT_INVERT_SPINDLE_CCW_PIN Off // NOTE: not supported by all drivers.
#endif
#if !defined DEFAULT_INVERT_SPINDLE_PWM_PIN || defined __DOXYGEN__
#define DEFAULT_INVERT_SPINDLE_PWM_PIN Off // NOTE: not supported by all drivers.
#endif
///@}

/*! @name $30 - Setting_RpmMax
*/
///@{
#if !defined DEFAULT_SPINDLE_RPM_MAX || defined __DOXYGEN__
#define DEFAULT_SPINDLE_RPM_MAX 1000.0f // rpm
#endif
///@}

/*! @name $31 - Setting_RpmMin
*/
///@{
#if !defined DEFAULT_SPINDLE_RPM_MIN || defined __DOXYGEN__
#define DEFAULT_SPINDLE_RPM_MIN 0.0f // rpm
#endif
///@}

/*! @name $33 - Setting_PWMFreq
*/
///@{
#if !defined DEFAULT_SPINDLE_PWM_FREQ || defined __DOXYGEN__
#define DEFAULT_SPINDLE_PWM_FREQ 5000 // Hz
#endif
///@}

/*! @name $34 - Setting_PWMOffValue
*/
///@{
#if !defined DEFAULT_SPINDLE_PWM_OFF_VALUE || defined __DOXYGEN__
#define DEFAULT_SPINDLE_PWM_OFF_VALUE 0.0f // Percent
#endif
///@}

/*! @name $35 - Setting_PWMMinValue
Used by variable spindle output only. This forces the PWM output to a minimum duty cycle when enabled.
The PWM pin will still read 0V when the spindle is disabled. Most users will not need this option, but
it may be useful in certain scenarios. This minimum PWM settings coincides with the spindle rpm minimum
setting, like rpm max to max PWM. This is handy if you need a larger voltage difference between 0V disabled
and the voltage set by the minimum PWM for minimum rpm. This difference is 0.02V per PWM value. So, when
minimum PWM is at 1, only 0.02 volts separate enabled and disabled. At PWM 5, this would be 0.1V. Keep
in mind that you will begin to lose PWM resolution with increased minimum PWM values, since you have less
and less range over the total 255 PWM levels to signal different spindle speeds.
<br>__!! NOTE:__ Compute duty cycle at the minimum PWM by this equation: (% duty cycle)=(SPINDLE_PWM_MIN_VALUE/255)*100
*/
///@{
#if !defined DEFAULT_SPINDLE_PWM_MIN_VALUE || defined __DOXYGEN__
#define DEFAULT_SPINDLE_PWM_MIN_VALUE 0.0f // Must be greater than zero. Integer (+-255).
#endif
///@}

/*! @name $36 - Setting_PWMMaxValue
*/
///@{
#if !defined DEFAULT_SPINDLE_PWM_MAX_VALUE || defined __DOXYGEN__
#define DEFAULT_SPINDLE_PWM_MAX_VALUE 100.0f // Percent
#endif
///@}

/*! @name $38 - Setting_SpindlePPR
When the HAL driver supports spindle sync then this option sets the number of pulses per revolution
for the spindle encoder. Depending on the driver this may lead to the "spindle at speed" detection
being enabled. When this is enabled grblHAL will wait for the spindle to reach the programmed speed
before continue processing. NOTE: Currently there is no timeout for this wait.
Default value is 0, meaning spindle sync is disabled
*/
///@{
#if !defined DEFAULT_SPINDLE_PPR || defined __DOXYGEN__
#define DEFAULT_SPINDLE_PPR 0 // Pulses per revolution.
#endif
///@}

/*! @name $340 - Setting_SpindleAtSpeedTolerance
*/
///@{
#if !defined DEFAULT_SPINDLE_AT_SPEED_TOLERANCE || defined __DOXYGEN__
#define DEFAULT_SPINDLE_AT_SPEED_TOLERANCE 0.0f // Percent - 0 means not checked
#endif
///@}

/*! @name $395 - Setting_SpindleType
*/
///@{
#if !defined DEFAULT_SPINDLE || defined __DOXYGEN__
#define DEFAULT_SPINDLE SPINDLE_PWM0 // Spindle number from spindle_control.h
#endif
///@}

// Closed loop spindle settings (Group_Spindle_ClosedLoop)

// $9 - Setting_SpindlePWMOptions
// bit 0
// always defaults to on
// bit 1

// Closed loop spindle settings (Group_Spindle_ClosedLoop)

#ifndef DEFAULT_SPINDLE_P_GAIN
#define DEFAULT_SPINDLE_P_GAIN  1.0f
#endif
#ifndef DEFAULT_SPINDLE_I_GAIN
#define DEFAULT_SPINDLE_I_GAIN  0.01f
#endif
#ifndef DEFAULT_SPINDLE_D_GAIN
#define DEFAULT_SPINDLE_D_GAIN  0.0f
#endif
#ifndef DEFAULT_SPINDLE_I_MAX
#define DEFAULT_SPINDLE_I_MAX   10.0f
#endif

#if ENABLE_SPINDLE_LINEARIZATION || defined __DOXYGEN__

/*! @name $66 - Setting_LinearSpindlePiece1
Defines the parameters for the first entry in the spindle RPM linearization table.
*/
///@{
#if !defined DEFAULT_RPM_POINT01 || defined __DOXYGEN__
#define DEFAULT_RPM_POINT01 NAN // DEFAULT_SPINDLE_RPM_MIN  // Replace NAN with DEFAULT_SPINDLE_RPM_MIN to enable.
#endif
#if !defined DEFAULT_RPM_LINE_A1 || defined __DOXYGEN__
#define DEFAULT_RPM_LINE_A1 3.197101e-03f
#endif
#if !defined DEFAULT_RPM_LINE_B1 || defined __DOXYGEN__
#define DEFAULT_RPM_LINE_B1 -3.526076e-1f
#endif
///@}

/*! @name $67 - Setting_LinearSpindlePiece2
Defines the parameters for the second entry in the spindle RPM linearization table.
*/
///@{
#if !defined DEFAULT_RPM_POINT12 || defined __DOXYGEN__
#define DEFAULT_RPM_POINT12 NAN  // Change NAN to a float constant to enable.
#endif
#if !defined DEFAULT_RPM_LINE_A2 || defined __DOXYGEN__
#define DEFAULT_RPM_LINE_A2  1.722950e-2f
#endif
#if !defined DEFAULT_RPM_LINE_B2 || defined __DOXYGEN__
#define DEFAULT_RPM_LINE_B2  1.0f,
#endif
///@}

/*! @name $68 - Setting_LinearSpindlePiece3
Defines the parameters for the third entry in the spindle RPM linearization table.
*/
///@{
#if !defined DEFAULT_RPM_POINT23 || defined __DOXYGEN__
#define DEFAULT_RPM_POINT23 NAN  // Change NAN to a float constant to enable.
#endif
#if !defined DEFAULT_RPM_LINE_A3 || defined __DOXYGEN__
#define DEFAULT_RPM_LINE_A3 5.901518e-02f
#endif
#if !defined DEFAULT_RPM_LINE_B3 || defined __DOXYGEN__
#define DEFAULT_RPM_LINE_B3 4.881851e+02f
#endif
///@}

/*! @name $69 - Setting_LinearSpindlePiece4
Defines the parameters for the fourth entry in the spindle RPM linearization table.
*/
///@{
#if !defined DEFAULT_RPM_POINT34 || defined __DOXYGEN__
#define DEFAULT_RPM_POINT34 NAN  // Change NAN to a float constant to enable.
#endif
#if !defined DEFAULT_RPM_LINE_A4 || defined __DOXYGEN__
#define DEFAULT_RPM_LINE_A4  1.203413e-01f
#endif
#if !defined DEFAULT_RPM_LINE_B4 || defined __DOXYGEN__
#define DEFAULT_RPM_LINE_B4  1.151360e+03f
#endif
///@}

#endif // ENABLE_SPINDLE_LINEARIZATION

// Settings for second PWM spindle

/*! @name $716 - Setting_SpindleInvertMask1
Inverts the selected spindle output signals from active high to active low. Useful for some pre-built electronic boards.
*/
///@{
#if !defined DEFAULT_INVERT_SPINDLE1_ENABLE_PIN || defined __DOXYGEN__
#define DEFAULT_INVERT_SPINDLE1_ENABLE_PIN Off
#endif
#if !defined DEFAULT_INVERT_SPINDLE1_CCW_PIN || defined __DOXYGEN__
#define DEFAULT_INVERT_SPINDLE1_CCW_PIN Off // NOTE: not supported by all drivers.
#endif
#if !defined DEFAULT_INVERT_SPINDLE1_PWM_PIN || defined __DOXYGEN__
#define DEFAULT_INVERT_SPINDLE1_PWM_PIN Off // NOTE: not supported by all drivers.
#endif
///@}

/*! @name $730 - Setting_RpmMax1
*/
///@{
#if !defined DEFAULT_SPINDLE1_RPM_MAX || defined __DOXYGEN__
#define DEFAULT_SPINDLE1_RPM_MAX 1000.0f // rpm
#endif
///@}

/*! @name $731 - Setting_RpmMin1
*/
///@{
#if !defined DEFAULT_SPINDLE1_RPM_MIN || defined __DOXYGEN__
#define DEFAULT_SPINDLE1_RPM_MIN 0.0f // rpm
#endif
///@}

/*! @name $733 - Setting_PWMFreq1
*/
///@{
#if !defined DEFAULT_SPINDLE1_PWM_FREQ || defined __DOXYGEN__
#define DEFAULT_SPINDLE1_PWM_FREQ 5000 // Hz
#endif
///@}

/*! @name $734 - Setting_PWMOffValue1
*/
///@{
#if !defined DEFAULT_SPINDLE1_PWM_OFF_VALUE || defined __DOXYGEN__
#define DEFAULT_SPINDLE1_PWM_OFF_VALUE 0.0f // Percent
#endif
///@}

/*! @name $735 - Setting_PWMMinValue1
Used by variable spindle output only. This forces the PWM output to a minimum duty cycle when enabled.
The PWM pin will still read 0V when the spindle is disabled. Most users will not need this option, but
it may be useful in certain scenarios. This minimum PWM settings coincides with the spindle rpm minimum
setting, like rpm max to max PWM. This is handy if you need a larger voltage difference between 0V disabled
and the voltage set by the minimum PWM for minimum rpm. This difference is 0.02V per PWM value. So, when
minimum PWM is at 1, only 0.02 volts separate enabled and disabled. At PWM 5, this would be 0.1V. Keep
in mind that you will begin to lose PWM resolution with increased minimum PWM values, since you have less
and less range over the total 255 PWM levels to signal different spindle speeds.
<br>__!! NOTE:__ Compute duty cycle at the minimum PWM by this equation: (% duty cycle)=(SPINDLE1_PWM_MIN_VALUE/255)*100
*/
///@{
#if !defined DEFAULT_SPINDLE1_PWM_MIN_VALUE || defined __DOXYGEN__
#define DEFAULT_SPINDLE1_PWM_MIN_VALUE 0.0f // Must be greater than zero. Integer (+-255).
#endif
///@}

/*! @name $736 - Setting_PWMMaxValue
*/
///@{
#if !defined DEFAULT_SPINDLE1_PWM_MAX_VALUE || defined __DOXYGEN__
#define DEFAULT_SPINDLE1_PWM_MAX_VALUE 100.0f // Percent
#endif
///@}


// Tool change settings (Group_Toolchange)

/*! @name $341 - Setting_ToolChangeMode
0 = Normal mode, 1 = Manual change, 2 = Manual change @ G59.3,  3 = Manual change and probe sensor @ G59.3 - sets TLO
*/
///@{
#if !defined DEFAULT_TOOLCHANGE_MODE || defined __DOXYGEN__
#define DEFAULT_TOOLCHANGE_MODE 0
#endif
///@}

/*! @name $342 - Setting_ToolChangeProbingDistance
*/
///@{
#if !defined DEFAULT_TOOLCHANGE_PROBING_DISTANCE || defined __DOXYGEN__
#define DEFAULT_TOOLCHANGE_PROBING_DISTANCE 30  // max probing distance in mm for mode 3
#endif
///@}

/*! @name $343 - Setting_ToolChangeFeedRate
*/
///@{
#if !defined DEFAULT_TOOLCHANGE_FEED_RATE || defined __DOXYGEN__
#define DEFAULT_TOOLCHANGE_FEED_RATE 25.0f      // mm/min
#endif
///@}

/*! @name $344 - Setting_ToolChangeSeekRate

 */
///@{
#if !defined DEFAULT_TOOLCHANGE_SEEK_RATE || defined __DOXYGEN__
#define DEFAULT_TOOLCHANGE_SEEK_RATE 200.0f     // mm/min
#endif
///@}

/*! @name $345 - Setting_ToolChangePulloffRate
*/
///@{
#if !defined DEFAULT_TOOLCHANGE_PULLOFF_RATE || defined __DOXYGEN__
#define DEFAULT_TOOLCHANGE_PULLOFF_RATE 200.0f  // mm/min
#endif
///@}

/*! @name $346 - Setting_ToolChangeRestorePosition
*/
///@{
#if !defined DEFAULT_TOOLCHANGE_NO_RESTORE_POSITION || defined __DOXYGEN__
#define DEFAULT_TOOLCHANGE_NO_RESTORE_POSITION Off
#endif
///@}

// Homing settings (Group_Homing)

/*! @name $22 - Setting_HomingEnable
\brief Enable homing.
Requires homing cycles to be defined by \ref DEFAULT_HOMING_CYCLE_0 - \ref DEFAULT_HOMING_CYCLE_2 +.
\internal Bit 0 in settings.homing.flags.
*/
///@{
#if !defined DEFAULT_HOMING_ENABLE || defined __DOXYGEN__
#define DEFAULT_HOMING_ENABLE Off // Default disabled. Set to \ref On or 1 to enable.
#endif

/*! /def DEFAULT_HOMING_SINGLE_AXIS_COMMANDS
\brief Enables single axis homing commands.
`$HX`, `$HY`, `$HZ` etc. for homing the respective axes.The full homing
cycle is still invoked by the `$H` command. This is disabled by default.
If you have a two-axis machine, _DON'T USE THIS_. Instead, just alter the homing cycle for two-axes.
\internal Bit 1 in settings.homing.flags.
*/
#if !defined DEFAULT_HOMING_SINGLE_AXIS_COMMANDS || defined __DOXYGEN__
#define DEFAULT_HOMING_SINGLE_AXIS_COMMANDS Off // Default disabled. Set to \ref On or 1 to enable.
#endif

/*! /def DEFAULT_HOMING_INIT_LOCK
\brief
If homing is enabled, homing init lock sets grblHAL into an alarm state upon power up or a soft reset.
This forces the user to perform the homing cycle before doing anything else. This is
mainly a safety feature to remind the user to home, since position is unknown to grblHAL.
\internal Bit 2 in settings.homing.flags.
*/
#if !defined DEFAULT_HOMING_INIT_LOCK || defined __DOXYGEN__
#define DEFAULT_HOMING_INIT_LOCK Off // Default disabled. Set to \ref On or 1 to enable.
#endif

/*! /def DEFAULT_HOMING_FORCE_SET_ORIGIN
\brief
After homing, grblHAL will set by default the entire machine space into negative space, as is typical
for professional CNC machines, regardless of where the limit switches are located. Set this
define to \ref On or 1 to force grblHAL to always set the machine origin at the homed location despite switch orientation.
\internal Bit 3 in settings.homing.flags.
*/
#if !defined DEFAULT_HOMING_FORCE_SET_ORIGIN || defined __DOXYGEN__
#define DEFAULT_HOMING_FORCE_SET_ORIGIN Off // Default disabled. Set to \ref On or 1 to enable.
#endif

/*! \def DEFAULT_LIMITS_TWO_SWITCHES_ON_AXES
\brief
If your machine has two limits switches wired in parallel to one axis, you will need to enable
this feature. Since the two switches are sharing a single pin, there is no way for grblHAL to tell
which one is enabled. This option only effects homing, where if a limit is engaged, grblHAL will
alarm out and force the user to manually disengage the limit switch. Otherwise, if you have one
limit switch for each axis, don't enable this option. By keeping it disabled, you can perform a
homing cycle while on the limit switch and not have to move the machine off of it.
\internal Bit 4 in settings.limits.flags.
*/
#if !defined DEFAULT_LIMITS_TWO_SWITCHES_ON_AXES || defined __DOXYGEN__
#define DEFAULT_LIMITS_TWO_SWITCHES_ON_AXES Off // Default disabled. Set to \ref On or 1 to enable.
#endif
///@}

/*! /def DEFAULT_HOMING_ALLOW_MANUAL
\brief
If enabled this allows using the homing $-commands to set the home position to the
current axis position.
\internal Bit 4 in settings.homing.flags.
*/
#if !defined DEFAULT_HOMING_ALLOW_MANUAL || defined __DOXYGEN__
#define DEFAULT_HOMING_ALLOW_MANUAL Off // Default disabled. Set to \ref On or 1 to enable.
#endif

/*! /def DEFAULT_HOMING_OVERRIDE_LOCKS
\brief
If homing init lock is enabled this sets grblHAL into an alarm state upon power up or a soft reset.
To allow a soft reset to override the lock uncomment the line below.
\internal Bit 5 in settings.homing.flags.
*/
#if !defined DEFAULT_HOMING_OVERRIDE_LOCKS || defined __DOXYGEN__
#define DEFAULT_HOMING_OVERRIDE_LOCKS Off // Default disabled. Set to \ref On or 1 to enable.
#endif

/*! /def DEFAULT_HOMING_KEEP_STATUS_ON_RESET
\brief
Enable this setting to keep homed status over a soft reset - if position was not lost due
to a reset during motion.
\internal Bit 6 in settings.homing.flags.
*/
#if !defined DEFAULT_HOMING_KEEP_STATUS_ON_RESET || defined __DOXYGEN__
#define DEFAULT_HOMING_KEEP_STATUS_ON_RESET Off // Default disabled. Set to \ref On or 1 to enable.
#endif
///@}

/*! /def DEFAULT_HOMING_USE_LIMIT_SWITCHES
\brief
Enable this setting to force using limit switches for homing.
\internal Bit 7 in settings.homing.flags.
*/
#if !defined DEFAULT_HOMING_USE_LIMIT_SWITCHES || defined __DOXYGEN__
#define DEFAULT_HOMING_USE_LIMIT_SWITCHES Off // Default disabled. Set to \ref On or 1 to enable.
#endif
///@}

/*! @name $23 - Setting_HomingDirMask
\ref axismask controlling the direction of movement during homing.
Unset bits in the mask results in movement in positive direction.
*/
///@{
#if !defined DEFAULT_HOMING_DIR_MASK || defined __DOXYGEN__
#define DEFAULT_HOMING_DIR_MASK 0
#endif
///@}

/*! @name $24 - Setting_HomingFeedRate
*/
///@{
#if !defined DEFAULT_HOMING_FEED_RATE || defined __DOXYGEN__
#define DEFAULT_HOMING_FEED_RATE 25.0f // mm/min
#endif
///@}

/*! @name $25 - Setting_HomingSeekRate
*/
///@{
#if !defined DEFAULT_HOMING_SEEK_RATE || defined __DOXYGEN__
#define DEFAULT_HOMING_SEEK_RATE 500.0f // mm/min
#endif
///@}

/*! @name $26 - Setting_HomingDebounceDelay
*/
///@{
#if !defined DEFAULT_HOMING_DEBOUNCE_DELAY || defined __DOXYGEN__
#define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
#endif
///@}

/*! @name $27 - Setting_HomingPulloff
*/
///@{
#if !defined DEFAULT_HOMING_PULLOFF || defined __DOXYGEN__
#define DEFAULT_HOMING_PULLOFF 1.0f // mm
#endif
///@}

/*! @name $43 - Setting_HomingLocateCycles
Number of homing cycles performed after when the machine initially jogs to limit switches.
This help in preventing overshoot and should improve repeatability. This value should be one or
greater.
*/
///@{
#if !defined DEFAULT_N_HOMING_LOCATE_CYCLE || defined __DOXYGEN__
#define DEFAULT_N_HOMING_LOCATE_CYCLE 1 // Integer (1-127)
#endif
///@}

/*! @name Setting_HomingCycle_1 - Setting_HomingCycle_6
// Define the homing cycle patterns with bitmasks. The homing cycle first performs a search mode
// to quickly engage the limit switches, followed by a slower locate mode, and finished by a short
// pull-off motion to disengage the limit switches. The following DEFAULT_HOMING_CYCLE_x defines are executed
// in order starting with suffix 0 and completes the homing routine for the specified-axes only. If
// an axis is omitted from the defines, it will not home, nor will the system update its position.
// Meaning that this allows for users with non-standard cartesian machines, such as a lathe (x then z,
// with no y), to configure the homing cycle behavior to their needs.
// NOTE: The homing cycle is designed to allow sharing of limit pins, if the axes are not in the same
// cycle, but this requires some pin settings changes in cpu_map.h file. For example, the default homing
// cycle can share the Z limit pin with either X or Y limit pins, since they are on different cycles.
// By sharing a pin, this frees up a precious IO pin for other purposes. In theory, all axes limit pins
// may be reduced to one pin, if all axes are homed with separate cycles, or vice versa, all three axes
// on separate pin, but homed in one cycle. Also, it should be noted that the function of hard limits
// will not be affected by pin sharing.
// NOTE: Defaults are set for a traditional 3-axis CNC machine. Z-axis first to clear, followed by X & Y.
*/

/*! @name $44 - Setting_HomingCycle_1
*/
///@{
#if !defined DEFAULT_HOMING_CYCLE_0 || defined __DOXYGEN__
#define DEFAULT_HOMING_CYCLE_0 (Z_AXIS_BIT)             // REQUIRED: First move Z to clear workspace.
#endif
///@}

/*! @name $45 - Setting_HomingCycle_2
*/
///@{
#if !defined DEFAULT_HOMING_CYCLE_1 || defined __DOXYGEN__
#if COREXY
#define DEFAULT_HOMING_CYCLE_1 (X_AXIS_BIT)             // OPTIONAL: Then move X.
#else
#define DEFAULT_HOMING_CYCLE_1 (X_AXIS_BIT|Y_AXIS_BIT)  // OPTIONAL: Then move X,Y at the same time.
#endif
#endif
///@}

/*! @name $46 - Setting_HomingCycle_3
*/
///@{
#if !defined DEFAULT_HOMING_CYCLE_2 || defined __DOXYGEN__
#if COREXY
#define DEFAULT_HOMING_CYCLE_2 (Y_AXIS_BIT)             // OPTIONAL: Then move Y.
#else
#define DEFAULT_HOMING_CYCLE_2 0                        // OPTIONAL: Uncomment and add axes mask to enable
#endif
#endif
///@}

/*! @name $47 - Setting_HomingCycle_4
*/
///@{
#if (defined A_AXIS && !defined DEFAULT_HOMING_CYCLE_3) || defined __DOXYGEN__
#define DEFAULT_HOMING_CYCLE_3 0                        // OPTIONAL: Uncomment and add axes mask to enable
#endif
///@}

/*! @name $48 - Setting_HomingCycle_5
\ref axismask
 */
///@{
#if (defined B_AXIS && !defined DEFAULT_HOMING_CYCLE_4) || defined __DOXYGEN__
#define DEFAULT_HOMING_CYCLE_4 0                        // OPTIONAL: Uncomment and add axes mask to enable
#endif
///@}

/*! @name $49 - Setting_HomingCycle_6
*/
///@{
#if (defined C_AXIS && !defined DEFAULT_HOMING_CYCLE_5) || defined __DOXYGEN__
#define DEFAULT_HOMING_CYCLE_5 0                        // OPTIONAL: Uncomment and add axes mask to enable
#endif
///@}

/*! @name $671 - Setting_HomePinsInvertMask
By default, grblHAL sets all input pins to normal-low operation with their internal pull-up resistors
enabled. This simplifies the wiring for users by requiring only a normally closed (NC) switch connected
to ground. It is _not_ recommended to use normally-open (NO) switches as this increases the risk
of electrical noise or cable breaks spuriously triggering the inputs. If normally-open (NO) switches
are used the logic of the input signals should be be inverted with the \ref axismask below.
*/
///@{
#if !defined DEFAULT_HOME_SIGNALS_INVERT_MASK || defined __DOXYGEN__
#define DEFAULT_HOME_SIGNALS_INVERT_MASK 0 // Set to -1 or AXES_BITMASK to invert for all axes
#endif
///@}


// Probing settings (Group_Probing)

/*! @name $6 - Setting_InvertProbePin
*/
///@{
#if !defined DEFAULT_PROBE_SIGNAL_INVERT || defined __DOXYGEN__
#define DEFAULT_PROBE_SIGNAL_INVERT Off
#endif
#if !defined DEFAULT_TOOLSETTER_SIGNAL_INVERT || defined __DOXYGEN__
#define DEFAULT_TOOLSETTER_SIGNAL_INVERT Off
#endif
///@}

/*! @name $19 - Setting_ProbePullUpDisable
*/
///@{
#if !defined DEFAULT_PROBE_SIGNAL_DISABLE_PULLUP || defined __DOXYGEN__
#define DEFAULT_PROBE_SIGNAL_DISABLE_PULLUP Off
#endif
#if !defined DEFAULT_TOOLSETTER_SIGNAL_DISABLE_PULLUP || defined __DOXYGEN__
#define DEFAULT_TOOLSETTER_SIGNAL_DISABLE_PULLUP Off
#endif
///@}

/*! @name $65 - Setting_ProbingFeedOverride
// By default, grblHAL disables feed rate overrides for all G38.x probe cycle commands. Although this
// may be different than some pro-class machine control, it's arguable that it should be this way.
// Most probe sensors produce different levels of error that is dependent on rate of speed. By
// keeping probing cycles to their programmed feed rates, the probe sensor should be a lot more
// repeatable. If needed, you can disable this behavior by uncommenting the define below.
*/
///@{
#if !defined DEFAULT_ALLOW_FEED_OVERRIDE_DURING_PROBE_CYCLES || defined __DOXYGEN__
#define DEFAULT_ALLOW_FEED_OVERRIDE_DURING_PROBE_CYCLES Off
#endif
///@}

// Safety door/parking settings (Group_SafetyDoor)

#ifdef DEFAULT_HOMING_ENABLE

/*! @name $41 - Setting_ParkingEnable
// Enables and configures parking motion methods upon a safety door state. Primarily for OEMs
// that desire this feature for their integrated machines. At the moment, grblHAL assumes that
// the parking motion only involves one axis, although the parking implementation was written
// to be easily refactored for any number of motions on different axes by altering the parking
// source code. At this time, grblHAL only supports parking one axis (typically the Z-axis) that
// moves in the positive direction upon retracting and negative direction upon restoring position.
// The motion executes with a slow pull-out retraction motion, power-down, and a fast park.
// Restoring to the resume position follows these set motions in reverse: fast restore to
// pull-out position, power-up with a time-out, and plunge back to the original position at the
// slower pull-out rate.
// NOTE: Still a work-in-progress. Machine coordinates must be in all negative space and
// does not work with DEFAULT_HOMING_FORCE_SET_ORIGIN enabled. Parking motion also moves only in
// positive direction.
 *  // Default disabled. Uncomment to enable.
*/
///@{
#if !defined DEFAULT_PARKING_ENABLE || defined __DOXYGEN__
#define DEFAULT_PARKING_ENABLE Off // bit 0
// Enables a special set of M-code commands that enables and disables the parking motion.
// These are controlled by `M56`, `M56 P1`, or `M56 Px` to enable and `M56 P0` to disable.
// The command is modal and will be set after a planner sync. Since it is g-code, it is
// executed in sync with g-code commands. It is not a real-time command.
// NOTE: PARKING_ENABLE is required. By default, M56 is active upon initialization. Use
// DEACTIVATE_PARKING_UPON_INIT to set M56 P0 as the power-up default.
#endif
#if !defined DEFAULT_DEACTIVATE_PARKING_UPON_INIT || defined __DOXYGEN__
#define DEFAULT_DEACTIVATE_PARKING_UPON_INIT Off // bit 1
#endif
#if !defined DEFAULT_ENABLE_PARKING_OVERRIDE_CONTROL || defined __DOXYGEN__
#define DEFAULT_ENABLE_PARKING_OVERRIDE_CONTROL Off // bit 2
#endif
///@}

/*! @name $42 - Setting_ParkingAxis
Define which axis that performs the parking motion
*/
///@{
#if !defined DEFAULT_PARKING_AXIS || defined __DOXYGEN__
#define DEFAULT_PARKING_AXIS Z_AXIS //
#endif
///@}

/*! @name $56 - Setting_ParkingPulloutIncrement
*/
///@{
#if !defined DEFAULT_PARKING_PULLOUT_INCREMENT || defined __DOXYGEN__
#define DEFAULT_PARKING_PULLOUT_INCREMENT 5.0f //
#endif
///@}

/*! @name $57 - Setting_ParkingPulloutRate
Spindle pull-out and plunge distance in mm. Incremental distance.
Must be positive value or equal to zero.*/
///@{
#if !defined DEFAULT_PARKING_PULLOUT_RATE || defined __DOXYGEN__
#define DEFAULT_PARKING_PULLOUT_RATE 100.0f // mm/min.
#endif
///@}

/*! @name $58 - Setting_ParkingTarget
Parking axis target. In mm, as machine coordinate [-max_travel, 0].
*/
///@{
#if !defined DEFAULT_PARKING_TARGET || defined __DOXYGEN__
#define DEFAULT_PARKING_TARGET -5.0f // mm
#endif
///@}

/*! @name $59 - Setting_ParkingFastRate
Parking fast rate after pull-out in mm/min.
*/
///@{
#if !defined DEFAULT_PARKING_RATE || defined __DOXYGEN__
#define DEFAULT_PARKING_RATE 500.0f // mm/min
#endif
///@}

/*! @name $61 - Setting_DoorOptions
*/
///@{
/*!
\brief
If set to \ref On or 1 ignore the door open signal command when in IDLE state.
This can be useful for jogging laser cutters/engravers when the lid is open.
 */
#if !defined DEFAULT_DOOR_IGNORE_WHEN_IDLE || defined __DOXYGEN__
#define DEFAULT_DOOR_IGNORE_WHEN_IDLE Off
#endif
/*!
\brief
If set to \ref On or 1 keep the coolant on when door is opened.
 */
#if !defined DEFAULT_DOOR_KEEP_COOLANT_ON || defined __DOXYGEN__
#define DEFAULT_DOOR_KEEP_COOLANT_ON Off
#endif
///@}
/*! @name 392 - Setting_DoorSpindleOnDelay
\brief
After the safety door switch has been toggled and restored, this setting sets the power-up delay
between restoring the spindle and resuming the cycle.
*/
///@{
#if !defined DEFAULT_SAFETY_DOOR_SPINDLE_DELAY || defined __DOXYGEN__
#define DEFAULT_SAFETY_DOOR_SPINDLE_DELAY 4.0f // Float (seconds)
#endif
///@}

/*! @name 393 - Setting_DoorCoolantOnDelay
\brief
After the safety door switch has been toggled and restored, this setting sets the power-up delay
between restoring the coolant and resuming the cycle.
*/
///@{
#if !defined DEFAULT_SAFETY_DOOR_COOLANT_DELAY || defined __DOXYGEN__
#define DEFAULT_SAFETY_DOOR_COOLANT_DELAY 1.0f // Float (seconds)
#endif
///@}

#endif // DEFAULT_HOMING_ENABLE

// Jogging settings (Group_Jogging)

/*! @name $40 - Setting_JogSoftLimited
\brief Soft limit jog commands to stay within machine limits.
Unlike the general soft limits that raises an alarm for motions that exceeds
machine limits this setting, when enabled, keeps them with the limits.
<br>__NOTE:__ Requires the machine to be homed and have correctly set machine limts.
*/
///@{
#if !defined DEFAULT_JOG_LIMIT_ENABLE || defined __DOXYGEN__
#define DEFAULT_JOG_LIMIT_ENABLE Off
#endif
///@}

// Stepper settings (Group_Stepper)

/*! @name $0 - Setting_PulseMicroseconds
\brief Stepper pulse length in microseconds.
<br>__NOTE:__ The different MCUs supported have different interrupt latencies
and some drivers may enable features that are not available on others. This may
lead to this setting not beeing respected exactly over the supported range.
Typically drivers are calibrated to be correct for 10 microsecond pulse lengths,
however if a precise pulse length is required it should be measured and
adjusted either by changing this value or by changing the `STEP_PULSE_LATENCY` symbol
value that many drivers supports. Note that `STEP_PULSE_LATENCY` symbol is driver
specific - it is _not_ defined in the core.
*/
///@{
#if !defined DEFAULT_STEP_PULSE_MICROSECONDS || defined __DOXYGEN__
#define DEFAULT_STEP_PULSE_MICROSECONDS 10.0f
#endif
///@}

/*! @name $1 - Setting_StepperIdleLockTime
Delay in milliseconds before steppers are disabled when idle, range 0-65535.
Which axes are disabled can be defined by the \ref DEFAULT_STEPPER_DEENERGIZE_MASK
symbol. E.g. it might be desirable to unlock X and Y and keep the Z-axis enabled
since the spindle may pull down the Z due to its weight.
<br>__NOTE:__ Setting this value to 255 keeps all steppers enabled.
*/
///@{
#if !defined DEFAULT_STEPPER_IDLE_LOCK_TIME || defined __DOXYGEN__
#define DEFAULT_STEPPER_IDLE_LOCK_TIME 25 // milliseconds
#endif
///@}

/*! @name $2 - Setting_StepInvertMask
\brief \ref axismask controlling the polarity of the step signals. The default is positive pulses.
Set this value to -1 or AXES_BITMASK to invert for all steppers or specify which by mask.
*/
///@{
#if !defined DEFAULT_STEP_SIGNALS_INVERT_MASK || defined __DOXYGEN__
#define DEFAULT_STEP_SIGNALS_INVERT_MASK 0
#endif
///@}

/*! @name $3 - Setting_DirInvertMask
\brief \ref axismask controling the polarity of the stepper direction signals. The default
is positive voltage for motions in negative direction.
Set this value to -1 or AXES_BITMASK to invert for all steppers or specify which by mask.*/
///@{
#if !defined DEFAULT_DIR_SIGNALS_INVERT_MASK || defined __DOXYGEN__
#define DEFAULT_DIR_SIGNALS_INVERT_MASK 0
#endif
///@}

/*! @name $4 - Setting_InvertStepperEnable
\brief \ref axismask for inverting the polarity of the stepper enable signal(s).

Set this value to -1 or AXES_BITMASK to invert for all steppers or specify which by mask.
<br>__NOTE:__ If \ref COMPATIBILITY_LEVEL > 2 this setting reverts to the legacy
              Grbl behaviour where 0 inverts the enable signals for all drivers
              and 1 does not.
<br>__NOTE:__ This setting is not universally available for individual axes - check
              board documentation.
              Specify at least \ref X_AXIS_BIT if a common enable signal is used.
*/
///@{
#if !defined DEFAULT_ENABLE_SIGNALS_INVERT_MASK || defined __DOXYGEN__
#define DEFAULT_ENABLE_SIGNALS_INVERT_MASK AXES_BITMASK
#endif
///@}

/*! @name $8 - Setting_GangedDirInvertMask
\brief \ref axismask for inverting the polarity of the stepper direction signal(s) for the
second motor for ganged/auto squared axes.
*/
///@{
#if !defined DEFAULT_GANGED_DIRECTION_INVERT_MASK || defined __DOXYGEN__
#define DEFAULT_GANGED_DIRECTION_INVERT_MASK 0
#endif
///@}

/*! @name $29 - Setting_PulseDelayMicroseconds
*/
///@{
#if !defined DEFAULT_STEP_PULSE_DELAY || defined __DOXYGEN__
#define DEFAULT_STEP_PULSE_DELAY 0.0f
#endif
///@}

/*! @name $37 - Setting_StepperDeenergizeMask
\ref axismask to be OR'ed with stepper disable signal(s). Axes configured will not be disabled.
<br>__NOTE:__ Not universally available for individual axes - check driver documentation.
Specify at least \ref X_AXIS_BIT if a common enable signal is used.
*/
///@{
#if !defined DEFAULT_STEPPER_DEENERGIZE_MASK || defined __DOXYGEN__
#define DEFAULT_STEPPER_DEENERGIZE_MASK 0
#endif
///@}

/*! @name $376 - Settings_Axis_Rotational
Designate ABC axes as rotational by \ref axismask. This will disable scaling (to mm) in inches mode.
Set steps/mm for the axes to the value that represent the desired movement per unit.
For the controller the distance is unitless and and can be in degrees, radians, rotations, ...
*/
///@{
#if !defined DEFAULT_AXIS_ROTATIONAL_MASK || defined __DOXYGEN__
#define DEFAULT_AXIS_ROTATIONAL_MASK 0
#endif
///@}

/*! @name $481 - Setting_AutoReportInterval
Auto status report interval, allowed range is 100 - 1000. Set to 0 to disable.
*/
///@{
#if !defined DEFAULT_AUTOREPORT_INTERVAL || defined __DOXYGEN__
#define DEFAULT_AUTOREPORT_INTERVAL 0
#endif
///@}

/*! @name $482 - Setting_TimeZoneOffset
Timezone offset from UTC in hours, allowed range is -12.0 - 12.0.
*/
///@{
#if !defined DEFAULT_TIMEZONE_OFFSET || defined __DOXYGEN__
#define DEFAULT_TIMEZONE_OFFSET 0.0f
#endif
///@}

/*! @name $484 - Setting_UnlockAfterEStop
Specifices whether unlock ($X) is needed to clear an E-Stop alarm.
NOTE: The logic is inverted in the stored setting.
*/
///@{
#if !defined DEFAULT_NO_UNLOCK_AFTER_ESTOP || defined __DOXYGEN__
#define DEFAULT_NO_UNLOCK_AFTER_ESTOP Off
#endif
///@}

/*! @name $536 - Setting_RGB_StripLengt0
Number of LEDs in NeoPixel/WS2812 strip 1.
*/
///@{
#if !defined DEFAULT_RGB_STRIP0_LENGTH || defined __DOXYGEN__
#define DEFAULT_RGB_STRIP0_LENGTH 0
#endif
///@}

/*! @name $537 - Setting_RGB_StripLengt1
Number of LEDs in NeoPixel/WS2812 strip 2.
*/
///@{
#if !defined DEFAULT_RGB_STRIP1_LENGTH || defined __DOXYGEN__
#define DEFAULT_RGB_STRIP1_LENGTH 0
#endif
///@}

/*! @name $538 - Setting_RotaryWrap
Enable fast return to G28 position for rotary axes by \ref axismask.
Use:
G91G28<axisletter>0
G90
*/
///@{
#if !defined DEFAULT_AXIS_ROTARY_WRAP_MASK || defined __DOXYGEN__
#define DEFAULT_AXIS_ROTARY_WRAP_MASK 0
#endif
///@}

// Axis settings (Group_XAxis - Group_VAxis)

/*! @name $10x - Setting_AxisStepsPerMM

 */
///@{
#if !defined DEFAULT_X_STEPS_PER_MM || defined __DOXYGEN__
#define DEFAULT_X_STEPS_PER_MM 250.0f
#endif
#if !defined DEFAULT_Y_STEPS_PER_MM || defined __DOXYGEN__
#define DEFAULT_Y_STEPS_PER_MM 250.0f
#endif
#if !defined DEFAULT_Z_STEPS_PER_MM || defined __DOXYGEN__
#define DEFAULT_Z_STEPS_PER_MM 250.0f
#endif
#if (defined A_AXIS && !defined DEFAULT_A_STEPS_PER_MM) || defined __DOXYGEN__
#define DEFAULT_A_STEPS_PER_MM 250.0f
#endif
#if (defined B_AXIS && !defined DEFAULT_B_STEPS_PER_MM) || defined __DOXYGEN__
#define DEFAULT_B_STEPS_PER_MM 250.0f
#endif
#if (defined C_AXIS && !defined DEFAULT_C_STEPS_PER_MM) || defined __DOXYGEN__
#define DEFAULT_C_STEPS_PER_MM 250.0f
#endif
#if (defined U_AXIS && !defined DEFAULT_U_STEPS_PER_MM) || defined __DOXYGEN__
#define DEFAULT_U_STEPS_PER_MM 250.0f
#endif
#if (defined V_AXIS && !defined DEFAULT_V_STEPS_PER_MM) || defined __DOXYGEN__
#define DEFAULT_V_STEPS_PER_MM 250.0f
#endif
///@}

/*! @name $11x - Setting_AxisMaxRate

*/
///@{
#if !defined DEFAULT_X_MAX_RATE || defined __DOXYGEN__
#define DEFAULT_X_MAX_RATE 500.0f // mm/min
#endif
#if !defined DEFAULT_Y_MAX_RATE || defined __DOXYGEN__
#define DEFAULT_Y_MAX_RATE 500.0f // mm/min
#endif
#if !defined DEFAULT_Z_MAX_RATE || defined __DOXYGEN__
#define DEFAULT_Z_MAX_RATE 500.0f // mm/min
#endif
#if (defined A_AXIS && !defined DEFAULT_A_MAX_RATE) || defined __DOXYGEN__
#define DEFAULT_A_MAX_RATE 500.0f // mm/min
#endif
#if (defined B_AXIS && !defined DEFAULT_B_MAX_RATE) || defined __DOXYGEN__
#define DEFAULT_B_MAX_RATE 500.0f // mm/min
#endif
#if (defined C_AXIS && !defined DEFAULT_C_MAX_RATE) || defined __DOXYGEN__
#define DEFAULT_C_MAX_RATE 500.0f // mm/min
#endif
#if (defined U_AXIS && !defined DEFAULT_U_MAX_RATE) || defined __DOXYGEN__
#define DEFAULT_U_MAX_RATE 500.0f // mm/min
#endif
#if (defined V_AXIS && !defined DEFAULT_V_MAX_RATE) || defined __DOXYGEN__
#define DEFAULT_V_MAX_RATE 500.0f // mm/min
#endif
///@}

/*! @name 12x - Setting_AxisAcceleration
*/
///@{
#if !defined DEFAULT_X_ACCELERATION || defined __DOXYGEN__
#define DEFAULT_X_ACCELERATION 10.0f // mm/sec^2
#endif
#if !defined DEFAULT_Y_ACCELERATION || defined __DOXYGEN__
#define DEFAULT_Y_ACCELERATION 10.0f // mm/sec^2
#endif
#if !defined DEFAULT_Z_ACCELERATION || defined __DOXYGEN__
#define DEFAULT_Z_ACCELERATION 10.0f // mm/sec^2
#endif
#if (defined A_AXIS && !defined DEFAULT_A_ACCELERATION) || defined __DOXYGEN__
#define DEFAULT_A_ACCELERATION 10.0f // mm/sec^2
#endif
#if (defined B_AXIS && !defined DEFAULT_B_ACCELERATION) || defined __DOXYGEN__
#define DEFAULT_B_ACCELERATION 10.0f // mm/sec^2
#endif
#if (defined C_AXIS && !defined DEFAULT_C_ACCELERATION) || defined __DOXYGEN__
#define DEFAULT_C_ACCELERATION 10.0f // mm/sec^2
#endif
#if (defined U_AXIS && !defined DEFAULT_U_ACCELERATION) || defined __DOXYGEN__
#define DEFAULT_U_ACCELERATION 10.0f // mm/sec^2
#endif
#if (defined V_AXIS && !defined DEFAULT_V_ACCELERATION) || defined __DOXYGEN__
#define DEFAULT_V_ACCELERATION 10.0f // mm/sec^2
#endif
///@}

/*! @name 22x - Setting_AxisJerk
*/
///@{
#if !defined DEFAULT_X_JERK|| defined __DOXYGEN__
#define DEFAULT_X_JERK 100.0f // mm/sec^3
#endif
#if !defined DEFAULT_Y_JERK|| defined __DOXYGEN__
#define DEFAULT_Y_JERK 100.0f // mm/sec^3
#endif
#if !defined DEFAULT_Z_JERK || defined __DOXYGEN__
#define DEFAULT_Z_JERK 100.0f // mm/sec^3
#endif
#if (defined A_AXIS && !defined DEFAULT_A_JERK) || defined __DOXYGEN__
#define DEFAULT_A_JERK 100.0f // mm/sec^3
#endif
#if (defined B_AXIS && !defined DEFAULT_B_JERK) || defined __DOXYGEN__
#define DEFAULT_B_JERK 100.0f // mm/sec^3
#endif
#if (defined C_AXIS && !defined DEFAULT_C_JERK) || defined __DOXYGEN__
#define DEFAULT_C_JERK 100.0f // mm/sec^3
#endif
#if (defined U_AXIS && !defined DEFAULT_U_JERK) || defined __DOXYGEN__
#define DEFAULT_U_JERK 100.0f // mm/sec^3
#endif
#if (defined V_AXIS && !defined DEFAULT_V_JERK) || defined __DOXYGEN__
#define DEFAULT_V_JERK 100.0f // mm/sec^3
#endif
///@}

/*! @name 13x - Setting_AxisMaxTravel
__NOTE:__ Must be a positive values.
*/
///@{
#if !defined DEFAULT_X_MAX_TRAVEL || defined __DOXYGEN__
#define DEFAULT_X_MAX_TRAVEL 200.0f // mm
#endif
#if !defined DEFAULT_Y_MAX_TRAVEL || defined __DOXYGEN__
#define DEFAULT_Y_MAX_TRAVEL 200.0f // mm
#endif
#if !defined DEFAULT_Z_MAX_TRAVEL || defined __DOXYGEN__
#define DEFAULT_Z_MAX_TRAVEL 200.0f // mm
#endif
#if (defined A_AXIS && !defined DEFAULT_A_MAX_TRAVEL) || defined __DOXYGEN__
#define DEFAULT_A_MAX_TRAVEL 200.0f // mm
#endif
#if (defined B_AXIS && !defined DEFAULT_B_MAX_TRAVEL) || defined __DOXYGEN__
#define DEFAULT_B_MAX_TRAVEL 200.0f // mm
#endif
#if (defined C_AXIS && !defined DEFAULT_C_MAX_TRAVEL) || defined __DOXYGEN__
#define DEFAULT_C_MAX_TRAVEL 200.0f // mm
#endif
#if (defined U_AXIS && !defined DEFAULT_U_MAX_TRAVEL) || defined __DOXYGEN__
#define DEFAULT_U_MAX_TRAVEL 200.0f // mm
#endif
#if (defined V_AXIS && !defined DEFAULT_V_MAX_TRAVEL) || defined __DOXYGEN__
#define DEFAULT_V_MAX_TRAVEL 200.0f // mm
#endif
///@}

/*! @name 14x - Setting_AxisStepperCurrent
 *
 */
///@{
#if !defined DEFAULT_X_CURRENT || defined __DOXYGEN__
#define DEFAULT_X_CURRENT 500.0f // mA RMS
#endif
#if !defined DEFAULT_Y_CURRENT || defined __DOXYGEN__
#define DEFAULT_Y_CURRENT 500.0f // mA RMS
#endif
#if !defined DEFAULT_Z_CURRENT || defined __DOXYGEN__
#define DEFAULT_Z_CURRENT 500.0f // mA RMS
#endif
#if (defined A_AXIS && !defined DEFAULT_A_CURRENT) || defined __DOXYGEN__
#define DEFAULT_A_CURRENT 500.0f // mA RMS
#endif
#if (defined B_AXIS && !defined DEFAULT_B_CURRENT) || defined __DOXYGEN__
#define DEFAULT_B_CURRENT 500.0f // mA RMS
#endif
#if (defined C_AXIS && !defined DEFAULT_C_CURRENT) || defined __DOXYGEN__
#define DEFAULT_C_CURRENT 500.0f // mA RMS
#endif
#if (defined U_AXIS && !defined DEFAULT_U_CURRENT) || defined __DOXYGEN__
#define DEFAULT_U_CURRENT 500.0f // mA RMS
#endif
#if (defined V_AXIS && !defined DEFAULT_V_CURRENT) || defined __DOXYGEN__
#define DEFAULT_V_CURRENT 500.0f // mA RMS
#endif
///@}

// Sanity checks

// N_TOOLS may have been defined on the compiler command line.
#if defined(N_TOOLS) && N_TOOLS == 0
#undef N_TOOLS
#endif

#if defined(N_TOOLS) && N_TOOLS > 32
#undef N_TOOLS
#define N_TOOLS 32
#endif

#if N_SYS_SPINDLE > N_SPINDLE
#undef N_SYS_SPINDLE
#define N_SYS_SPINDLE N_SPINDLE
#endif

#if N_SYS_SPINDLE < 1
#undef N_SYS_SPINDLE
#define N_SYS_SPINDLE 1
#endif

#if N_SYS_SPINDLE > 8
#undef N_SYS_SPINDLE
#define N_SYS_SPINDLE 8
#endif

#if NGC_EXPRESSIONS_ENABLE && !NGC_PARAMETERS_ENABLE
#undef NGC_PARAMETERS_ENABLE
#define NGC_PARAMETERS_ENABLE On
#endif

#if (REPORT_WCO_REFRESH_BUSY_COUNT < REPORT_WCO_REFRESH_IDLE_COUNT)
  #error "WCO busy refresh is less than idle refresh."
#endif
#if (REPORT_OVERRIDE_REFRESH_BUSY_COUNT < REPORT_OVERRIDE_REFRESH_IDLE_COUNT)
  #error "Override busy refresh is less than idle refresh."
#endif
#if (REPORT_WCO_REFRESH_IDLE_COUNT < 2)
  #error "WCO refresh must be greater than one."
#endif
#if (REPORT_OVERRIDE_REFRESH_IDLE_COUNT < 1)
  #error "Override refresh must be greater than zero."
#endif

#if DEFAULT_LASER_MODE && DEFAULT_LATHE_MODE
#error "Cannot enable laser and lathe mode at the same time!"
#endif

#if LATHE_UVW_OPTION && (N_AXIS > 6 || AXIS_REMAP_ABC2UVW)
#warning "Cannot enable lathe UVW option when N_AXIS > 6 or ABC words are remapped!"
#undef LATHE_UVW_OPTION
#define LATHE_UVW_OPTION Off
#endif

#if DEFAULT_CONTROL_SIGNALS_INVERT_MASK < 0
#undef DEFAULT_CONTROL_SIGNALS_INVERT_MASK
#define DEFAULT_CONTROL_SIGNALS_INVERT_MASK SIGNALS_BITMASK
#endif

#if DEFAULT_LIMIT_SIGNALS_INVERT_MASK < 0
#undef DEFAULT_LIMIT_SIGNALS_INVERT_MASK
#define DEFAULT_LIMIT_SIGNALS_INVERT_MASK AXES_BITMASK
#endif

#if DEFAULT_LIMIT_SIGNALS_PULLUP_DISABLE_MASK < 0
#undef DEFAULT_LIMIT_SIGNALS_PULLUP_DISABLE_MASK
#define DEFAULT_LIMIT_SIGNALS_PULLUP_DISABLE_MASK AXES_BITMASK
#endif

#if DEFAULT_STEP_SIGNALS_INVERT_MASK < 0
#undef DEFAULT_STEP_SIGNALS_INVERT_MASK
#define DEFAULT_STEP_SIGNALS_INVERT_MASK AXES_BITMASK
#endif

#if DEFAULT_ENABLE_SIGNALS_INVERT_MASK < 0
#undef DEFAULT_ENABLE_SIGNALS_INVERT_MASK
#define DEFAULT_ENABLE_SIGNALS_INVERT_MASK AXES_BITMASK
#endif

#if DEFAULT_PARKING_ENABLE > 0
  #if DEFAULT_HOMING_FORCE_SET_ORIGIN > 0
    #error "DEFAULT_HOMING_FORCE_SET_ORIGIN is not supported with DEFAULT_PARKING_ENABLE at this time."
  #endif
#endif

#if DEFAULT_ENABLE_PARKING_OVERRIDE_CONTROL > 0
  #if DEFAULT_PARKING_ENABLE < 1
    #error "DEFAULT_ENABLE_PARKING_OVERRIDE_CONTROL must be enabled with DEFAULT_PARKING_ENABLE."
  #endif
#endif

/*! \page page1 Documentation of some common concepts used in config.h
\section axismask axismask
Example axismasks, replace XXX with the symbol name you want to define sans `_MASK`:
<br>No axes:
<br>`#define XXX_MASK 0`
<br>All axes:
<br>`#define XXX_MASK AXES_BITMASK`
<br>`#define XXX_MASK -1` // the value may be used from the compiler command line.
<br>Single axis:
<br>`#define XXX_MASK Z_AXIS`
<br>`#define XXX_MASK 4` // same as above but as a bitfield value, the value may be used from the compiler command line.
<br>Specific axes:
<br>`#define XXX_MASK (X_AXIS || Y_AXIS)`
<br>`#define XXX_MASK 3` // same as above but as a bitfield value, the value may be used from the compiler command line.

\section axis axis
Bitfield values are the sum of the individual bit values for each axis, these are:
<br>`X_AXIS = 1`
<br>`Y_AXIS = 2`
<br>`Z_AXIS = 4`
<br>`A_AXIS = 8`
<br>`B_AXIS = 16`
<br>`C_AXIS = 32`
<br>`U_AXIS = 64`
<br>`V_AXIS = 128`

\section signalmask signalmask
Example signalmasks, replace XXX with the symbol name you want to define sans `_MASK`:
<br>No signals:
<br>`#define XXX_MASK 0`
<br>All signals:
<br>`#define XXX_MASK SIGNALS_BITMASK`
<br>`#define XXX_MASK -1` // the value may be used from the compiler command line.
<br>Single axis:
<br>`#define XXX_MASK SIGNALS_ESTOP_BIT`
<br>`#define XXX_MASK 64` // same as above but as a bitfield value, the value may be used from the compiler command line.
<br>Specific axes:
<br>`#define XXX_MASK (SIGNALS_FEEDHOLD_BIT || SIGNALS_CYCLESTART_BIT)`
<br>`#define XXX_MASK 6` // same as above but as a bitfield value, the value may be used from the compiler command line.

\section signal signal

<br>`SIGNALS_RESET_BIT = 1`
<br>`SIGNALS_FEEDHOLD_BIT = 2`
<br>`SIGNALS_CYCLESTART_BIT= 4`
<br>`SIGNALS_SAFETYDOOR_BIT = 8`
<br>`SIGNALS_BLOCKDELETE_BIT = 16`
<br>`SIGNALS_STOPDISABLE_BIT = 32`
<br>`SIGNALS_ESTOP_BIT = 64`
<br>`SIGNALS_PROBE_CONNECTED_BIT = 128`
<br>`SIGNALS_MOTOR_FAULT_BIT = 256`
<br>`SIGNALS_BITMASK = 512`
*/

#endif // _GRBL_CONFIG_H_
