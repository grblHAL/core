/*
  system.h - Header for system level commands and real-time processes

  Part of grblHAL

  Copyright (c) 2017-2022 Terje Io
  Copyright (c) 2014-2016 Sungeun K. Jeon for Gnea Research LLC

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

#ifndef _SYSTEM_H_
#define _SYSTEM_H_

#include "gcode.h"
#include "probe.h"
#include "alarms.h"
#include "messages.h"

/*! @name System executor bit map.
\anchor rt_exec

Used internally by realtime protocol as realtime command flags,
which notifies the main program to execute the specified realtime command asynchronously.

__NOTE__: The system executor uses an unsigned 16-bit volatile variable (16 flag limit.) The default
flags are always false, so the realtime protocol only needs to check for a non-zero value to
know when there is a realtime command to execute.
*/
///@{
#define EXEC_STATUS_REPORT  bit(0)
#define EXEC_CYCLE_START    bit(1)
#define EXEC_CYCLE_COMPLETE bit(2)
#define EXEC_FEED_HOLD      bit(3)
#define EXEC_STOP           bit(4)
#define EXEC_RESET          bit(5)
#define EXEC_SAFETY_DOOR    bit(6)
#define EXEC_MOTION_CANCEL  bit(7)
#define EXEC_SLEEP          bit(8)
#define EXEC_TOOL_CHANGE    bit(9)
#define EXEC_PID_REPORT     bit(10)
#define EXEC_GCODE_REPORT   bit(11)
#define EXEC_TLO_REPORT     bit(12)
#define EXEC_RT_COMMAND     bit(13)
#define EXEC_DOOR_CLOSED    bit(14)
///@}

//! \def sys_state
/*! @name System state bit map.
\anchor sys_state

The state variable (type \ref sys_state_t) primarily tracks the individual states of grblHAL to manage each without overlapping.
It is also used as a messaging flag for critical events.

It is encapsulated by the main state machine in state_machine.c and can only be set via the state_set() or state_update() functions and read via the state_get() function.

__NOTE:__ flags are mutually exclusive, bit map allows testing for multiple states (except #STATE_IDLE) in a single statement
*/
///@{
#define STATE_IDLE          0      //!< Must be zero. No flags.
#define STATE_ALARM         bit(0) //!< In alarm state. Locks out all g-code processes. Allows settings access.
#define STATE_CHECK_MODE    bit(1) //!< G-code check mode. Locks out planner and motion only.
#define STATE_HOMING        bit(2) //!< Performing homing cycle
#define STATE_CYCLE         bit(3) //!< Cycle is running or motions are being executed.
#define STATE_HOLD          bit(4) //!< Active feed hold
#define STATE_JOG           bit(5) //!< Jogging mode.
#define STATE_SAFETY_DOOR   bit(6) //!< Safety door is ajar. Feed holds and de-energizes system.
#define STATE_SLEEP         bit(7) //!< Sleep state.
#define STATE_ESTOP         bit(8) //!< EStop mode, reports and is mainly handled similar to alarm state
#define STATE_TOOL_CHANGE   bit(9) //!< Manual tool change, similar to #STATE_HOLD - but stops spindle and allows jogging.
///@}

typedef enum {
    Mode_Standard = 0,
    Mode_Laser,
    Mode_Lathe
} machine_mode_t;

typedef enum {
    Parking_DoorClosed = 0, //!< 0
    Parking_DoorAjar,       //!< 1
    Parking_Retracting,     //!< 2
    Parking_Cancel,         //!< 3
    Parking_Resuming        //!< 4
} parking_state_t;

typedef enum {
    Hold_NotHolding = 0,    //!< 0
    Hold_Complete = 1,      //!< 1
    Hold_Pending = 2        //!< 2
} hold_state_t;

typedef uint_fast16_t rt_exec_t; //!< See \ref rt_exec
typedef uint_fast16_t sys_state_t; //!< See \ref sys_state

// Define step segment generator state flags.
typedef union {
    uint8_t flags;
    struct {
        uint8_t end_motion         :1,
                execute_hold       :1,
                execute_sys_motion :1,
                update_spindle_rpm :1,
                unassigned         :4;
    };
} step_control_t;


typedef union {
    uint16_t value;
    uint16_t mask;
    struct {
        uint16_t reset              :1,
                 feed_hold          :1,
                 cycle_start        :1,
                 safety_door_ajar   :1,
                 block_delete       :1,
                 stop_disable       :1, //! M1
                 e_stop             :1,
                 probe_disconnected :1,
                 motor_fault        :1,
                 motor_warning      :1,
                 limits_override    :1,
                 single_block       :1,
                 unassigned         :2,
                 probe_triggered    :1, //! used for probe protection
                 deasserted         :1; //! this flag is set if signals are deasserted. Note: do NOT pass on to the control_interrupt_handler if set.
    };
} control_signals_t;


// Define spindle stop override control states.
typedef union {
    uint8_t value;
    struct {
        uint8_t enabled       :1,
                initiate      :1,
                restore       :1,
                restore_cycle :1,
                unassigned    :4;
    };
} spindle_stop_t;

#ifdef PID_LOG

typedef struct {
    uint_fast16_t idx;
    float setpoint;
    float t_sample;
    float target[PID_LOG];
    float actual[PID_LOG];
} pid_data_t;

#endif

typedef union {
    uint32_t value;
    struct {
        uint32_t mpg_mode      :1, //!< MPG mode changed.
                 scaling       :1, //!< Scaling (G50/G51) changed.
                 homed         :1, //!< Homed state changed.
                 xmode         :1, //!< Lathe radius/diameter mode changed.
                 spindle       :1, //!< Spindle state changed.
                 coolant       :1, //!< Coolant state changed.
                 overrides     :1, //!< Overrides changed.
                 tool          :1, //!< Tool changed.
                 wco           :1, //!< Add work coordinates.
                 gwco          :1, //!< Add work coordinate.
                 tool_offset   :1, //!< Tool offsets changed.
                 m66result     :1, //!< M66 result updated
                 pwm           :1, //!< Add PWM information (optional: to be added by driver).
                 motor         :1, //!< Add motor information (optional: to be added by driver).
                 encoder       :1, //!< Add encoder information (optional: to be added by driver).
                 tlo_reference :1, //!< Tool length offset reference changed
                 fan           :1, //!< Fan on/off changed
                 unassigned   :14, //
                 all           :1; //!< Set when CMD_STATUS_REPORT_ALL is requested, may be used by user code
    };
} report_tracking_flags_t;

typedef struct {
    uint8_t feed_rate;              //!< Feed rate override value in percent
    uint8_t rapid_rate;             //!< Rapids override value in percent
    uint8_t spindle_rpm;            //!< Spindle speed override value in percent
    spindle_stop_t spindle_stop;    //!< Tracks spindle stop override states
    gc_override_flags_t control;    //!< Tracks override control states.
} overrides_t;

typedef union {
    uint16_t value;
    struct {
        uint16_t mpg_mode              :1, //!< MPG mode flag. Set when switched to secondary input stream. (unused for now).
                 probe_succeeded       :1, //!< Tracks if last probing cycle was successful.
                 soft_limit            :1, //!< Tracks soft limit errors for the state machine.
                 exit                  :1, //!< System exit flag. Used in combination with abort to terminate main loop.
                 block_delete_enabled  :1, //!< Set to true to enable block delete.
                 feed_hold_pending     :1,
                 delay_overrides       :1,
                 optional_stop_disable :1,
                 single_block          :1, //!< Set to true to disable M1 (optional stop), via realtime command.
                 keep_input            :1, //!< Set to true to not flush stream input buffer on executing STOP.
                 unused                :6;
    };
} system_flags_t;

typedef struct
{
    control_signals_t control;
    limit_signals_t limits;
} signal_event_t;

//! Global system variables struct.
// NOTE: probe_position and position variables may need to be declared as volatiles, if problems arise.
typedef struct system {
    bool abort;                             //!< System abort flag. Forces exit back to main loop for reset.
    bool cancel;                            //!< System cancel flag.
    bool suspend;                           //!< System suspend state flag.
    bool position_lost;                     //!< Set when mc_reset is called when machine is moving.
    volatile bool steppers_deenergize;      //!< Set to true to deenergize stepperes
    axes_signals_t tlo_reference_set;       //!< Axes with tool length reference offset set
    int32_t tlo_reference[N_AXIS];          //!< Tool length reference offset
    alarm_code_t alarm_pending;             //!< Delayed alarm, currently used for probe protection
    system_flags_t flags;                   //!< Assorted state flags
    step_control_t step_control;            //!< Governs the step segment generator depending on system state.
    axes_signals_t homing_axis_lock;        //!< Locks axes when limits engage. Used as an axis motion mask in the stepper ISR.
    axes_signals_t homing;                  //!< Axes with homing enabled.
    overrides_t override;                   //!< Override values & states
    report_tracking_flags_t report;         //!< Tracks when to add data to status reports.
    parking_state_t parking_state;          //!< Tracks parking state
    hold_state_t holding_state;             //!< Tracks holding state
    int32_t probe_position[N_AXIS];         //!< Last probe position in machine coordinates and steps.
    volatile probing_state_t probing_state; //!< Probing state value. Used to coordinate the probing cycle with stepper ISR.
    volatile rt_exec_t rt_exec_state;       //!< Realtime executor bitflag variable for state management. See EXEC bitmasks.
    volatile uint_fast16_t rt_exec_alarm;   //!< Realtime executor bitflag variable for setting various alarms.
    float spindle_rpm;                      //!< Current spindle RPM
    int32_t var5399;                        //!< Last result from M66 - wait on input
#ifdef PID_LOG
    pid_data_t pid_log;
#endif
//! @name The following variables are only cleared upon soft reset if position is likely lost, do NOT move. homed must be first!
//@{
    axes_signals_t homed;                   //!< Indicates which axes has been homed.
    float home_position[N_AXIS];            //!< Home position for homed axes
//@}
//!  @name The following variables are not cleared upon soft reset, do NOT move. alarm must be first!
//@{
    alarm_code_t alarm;                     //!< Current alarm, only valid if system state is STATE_ALARM.
    bool cold_start;                        //!< Set to true on boot, is false on subsequent soft resets.
    bool driver_started;                    //!< Set to true when driver initialization is completed.
    bool mpg_mode;                          //!< To be moved to system_flags_t
    machine_mode_t mode;                    //!< Current machine mode, copied from settings.mode on startup.
    signal_event_t last_event;              //!< Last signal events (control and limits signal).
    int32_t position[N_AXIS];               //!< Real-time machine (aka home) position vector in steps.
//@}
} system_t;

typedef status_code_t (*sys_command_ptr)(sys_state_t state, char *args);

typedef struct
{
    const char *command;
    bool noargs;
    sys_command_ptr execute;
} sys_command_t;

typedef struct sys_commands_str {
    const uint8_t n_commands;
    const sys_command_t *commands;
    struct sys_commands_str *(*on_get_commands)(void);
} sys_commands_t;

extern system_t sys;

//! Executes an internal system command, defined as a string starting with a '$'
status_code_t system_execute_line (char *line);

//! Execute the startup script lines stored in non-volatile storage upon initialization
void system_execute_startup (void);

void system_flag_wco_change (void);

// Returns machine position of axis 'idx'. Must be sent a 'step' array.
//float system_convert_axis_steps_to_mpos(int32_t *steps, uint_fast8_t idx);

//! Updates a machine 'position' array based on the 'step' array sent.
void system_convert_array_steps_to_mpos (float *position, int32_t *steps);

//! Checks if XY position is within coordinate system XY with given tolerance.
bool system_xy_at_fixture (coord_system_id_t id, float tolerance);

//! Checks and reports if target array exceeds machine travel limits.
bool system_check_travel_limits (float *target);

//! Checks and limit jog commands to within machine travel limits.
void system_apply_jog_limits (float *target);

//! Raise and report alarm state
void system_raise_alarm (alarm_code_t alarm);

//! Provide system command help
void system_command_help (void);

// Special handlers for setting and clearing Grbl's real-time execution flags.
#define system_set_exec_state_flag(mask) hal.set_bits_atomic(&sys.rt_exec_state, (mask))
#define system_clear_exec_state_flag(mask) hal.clear_bits_atomic(&sys.rt_exec_state, (mask))
#define system_clear_exec_states() hal.set_value_atomic(&sys.rt_exec_state, 0)
#define system_set_exec_alarm(code) hal.set_value_atomic(&sys.rt_exec_alarm, (uint_fast16_t)(code))
#define system_clear_exec_alarm() hal.set_value_atomic(&sys.rt_exec_alarm, 0)

void control_interrupt_handler (control_signals_t signals);

#endif
