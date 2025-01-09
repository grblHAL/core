/*
  system.h - Header for system level commands and real-time processes

  Part of grblHAL

  Copyright (c) 2017-2024 Terje Io
  Copyright (c) 2014-2016 Sungeun K. Jeon for Gnea Research LLC

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

#ifndef _SYSTEM_H_
#define _SYSTEM_H_

#include "gcode.h"
#include "probe.h"
#include "alarms.h"
#include "messages.h"
#if NGC_EXPRESSIONS_ENABLE
#include "vfs.h"
#endif

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

#ifdef ARDUINO

typedef enum  {
    Mode_Standard = 0,      //!< 0
    Mode_Laser,             //!< 1
    Mode_Lathe              //!< 2
} machine_mode_t;

#else

typedef uint8_t machine_mode_t;

enum machine_mode_t {
    Mode_Standard = 0,      //!< 0
    Mode_Laser,             //!< 1
    Mode_Lathe              //!< 2
};

#endif

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

// NOTE: the pin_function_t enum must be kept in sync with any changes!
typedef union {
    uint16_t bits;
    uint16_t mask;
    uint16_t value;
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
                 unassigned         :1,
                 probe_overtravel   :1, //! used for probe protection
                 probe_triggered    :1, //! used for probe protection
                 deasserted         :1; //! this flag is set if signals are deasserted.
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

typedef enum {
    Report_ClearAll = 0,
    Report_MPGMode = (1 << 0),
    Report_Scaling = (1 << 1),
    Report_Homed   = (1 << 2),
    Report_LatheXMode = (1 << 3),
    Report_Spindle = (1 << 4),
    Report_Coolant = (1 << 5),
    Report_Overrides = (1 << 6),
    Report_Tool = (1 << 7),
    Report_WCO = (1 << 8),
    Report_GWCO = (1 << 9),
    Report_ToolOffset = (1 << 10),
    Report_M66Result = (1 << 11),
    Report_PWM = (1 << 12),
    Report_Motor = (1 << 13),
    Report_Encoder = (1 << 14),
    Report_TLOReference = (1 << 15),
    Report_Fan = (1 << 16),
    Report_SpindleId = (1 << 17),
    Report_ForceWCO = (1 << 29),
    Report_CycleStart = (1 << 30),
    Report_All = 0x8003FFFF
} report_tracking_t;

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
                 tlo_reference :1, //!< Tool length offset reference changed.
                 fan           :1, //!< Fan on/off changed.
                 spindle_id    :1, //!< Spindle changed
                 unassigned   :11, //
                 force_wco     :1, //!< Add work coordinates (due to WCO changed during motion).
                 cycle_start   :1, //!< Cycle start signal triggered. __NOTE:__ do __NOT__ add to Report_All enum above!
                 all           :1; //!< Set when CMD_STATUS_REPORT_ALL is requested, may be used by user code.
    };
} report_tracking_flags_t;

typedef struct {
    override_t feed_rate;           //!< Feed rate override value in percent
    override_t rapid_rate;          //!< Rapids override value in percent
    override_t spindle_rpm;         //!< __NOTE:__ Not used by the core, it maintain per spindle override in \ref spindle_param_t
    spindle_stop_t spindle_stop;    //!< Tracks spindle stop override states
    gc_override_flags_t control;    //!< Tracks override control states.
} overrides_t;

typedef union {
    uint8_t flags;
    struct {
        uint8_t feedrate :1,
                coolant  :1,
                spindle  :1,
                unused   :5;
    };
} system_override_delay_t;

typedef union {
    uint16_t value;
    struct {
        uint16_t mpg_mode                :1, //!< MPG mode flag. Set when switched to secondary input stream. (unused for now).
                 probe_succeeded         :1, //!< Tracks if last probing cycle was successful.
                 soft_limit              :1, //!< Tracks soft limit errors for the state machine.
                 exit                    :1, //!< System exit flag. Used in combination with abort to terminate main loop.
                 block_delete_enabled    :1, //!< Set to true to enable block delete.
                 feed_hold_pending       :1,
                 optional_stop_disable   :1,
                 single_block            :1, //!< Set to true to disable M1 (optional stop), via realtime command.
                 keep_input              :1, //!< Set to true to not flush stream input buffer on executing STOP.
                 auto_reporting          :1, //!< Set to true when auto real time reporting is enabled.
                 synchronizing           :1, //!< Set to true when protocol_buffer_synchronize() is running.
                 travel_changed          :1, //!< Set to true when maximum travel settings has changed.
                 unused                  :4;
    };
} system_flags_t;

typedef struct {
    control_signals_t control;
    limit_signals_t limits;
} signal_event_t;

typedef struct {
    coord_data_t min;
    coord_data_t max;
} work_envelope_t;

//! Global system variables struct.
// NOTE: probe_position and position variables may need to be declared as volatiles, if problems arise.
typedef struct system {
    bool abort;                             //!< System abort flag. Forces exit back to main loop for reset.
    bool cancel;                            //!< System cancel flag.
    bool suspend;                           //!< System suspend state flag.
    bool position_lost;                     //!< Set when mc_reset is called when machine is moving.
    bool reset_pending;                     //!< Set when reset processing is underway.
    bool blocking_event;                    //!< Set when a blocking event that requires reset to clear is active.
    volatile bool steppers_deenergize;      //!< Set to true to deenergize stepperes
    axes_signals_t tlo_reference_set;       //!< Axes with tool length reference offset set
    int32_t tlo_reference[N_AXIS];          //!< Tool length reference offset
    alarm_code_t alarm_pending;             //!< Delayed alarm, currently used for probe protection
    system_flags_t flags;                   //!< Assorted state flags
    step_control_t step_control;            //!< Governs the step segment generator depending on system state.
    axes_signals_t homing_axis_lock;        //!< Locks axes when limits engage. Used as an axis motion mask in the stepper ISR.
    axes_signals_t homing;                  //!< Axes with homing enabled.
    overrides_t override;                   //!< Override values & states
    system_override_delay_t override_delay; //!< Flags for delayed overrides.
    report_tracking_flags_t report;         //!< Tracks when to add data to status reports.
    parking_state_t parking_state;          //!< Tracks parking state
    hold_state_t holding_state;             //!< Tracks holding state
    coord_system_id_t probe_coordsys_id;    //!< Coordinate system in which last probe took place.
    int32_t probe_position[N_AXIS];         //!< Last probe position in machine coordinates and steps.
    volatile probing_state_t probing_state; //!< Probing state value. Used to coordinate the probing cycle with stepper ISR.
    volatile rt_exec_t rt_exec_state;       //!< Realtime executor bitflag variable for state management. See EXEC bitmasks.
    volatile uint_fast16_t rt_exec_alarm;   //!< Realtime executor bitflag variable for setting various alarms.
    int32_t var5399;                        //!< Last result from M66 - wait on input.
#ifdef PID_LOG
    pid_data_t pid_log;
#endif
//! @name The following variables are only cleared upon soft reset if position is likely lost, do NOT move. homed must be first!
//@{
    axes_signals_t homed;                   //!< Indicates which axes has been homed.
    float home_position[N_AXIS];            //!< Home position for homed axes.
    work_envelope_t work_envelope;          //!< Work envelope, only valid for homed axes.
//@}
//!  @name The following variables are not cleared upon soft reset, do NOT move. alarm must be first!
//@{
    alarm_code_t alarm;                     //!< Current alarm, only valid if system state is STATE_ALARM.
    bool cold_start;                        //!< Set to true on boot, is false on subsequent soft resets.
    bool driver_started;                    //!< Set to true when driver initialization is completed.
    bool mpg_mode;                          //!< To be moved to system_flags_t
    signal_event_t last_event;              //!< Last signal events (control and limits signal).
    int32_t position[N_AXIS];               //!< Real-time machine (aka home) position vector in steps.
    axes_signals_t hard_limits; //!< temporary?, will be removed when available in settings.
    axes_signals_t soft_limits; //!< temporary, will be removed when available in settings.
//@}
} system_t;

typedef status_code_t (*sys_command_ptr)(sys_state_t state, char *args);
typedef const char *(*sys_help_ptr)(const char *command);

typedef union {
    uint8_t flags;
    struct {
        uint8_t noargs         :1, //!< System command does not handle arguments.
                allow_blocking :1, //!< System command can be used when blocking event is active.
                help_fn        :1,
                unused         :5;
    };
} sys_command_flags_t;

typedef struct
{
    const char *command;
    sys_command_ptr execute;
    sys_command_flags_t flags;
    union {
        const char *str;
        sys_help_ptr fn;
    } help;
} sys_command_t;

typedef struct sys_commands_str {
    const uint8_t n_commands;
    const sys_command_t *commands;
    struct sys_commands_str *next;
    struct sys_commands_str *(*on_get_commands)(void); //!< deprecated, to be removed
} sys_commands_t;

extern system_t sys;

status_code_t system_execute_line (char *line);
void system_execute_startup (void);
void system_flag_wco_change (void);
void system_convert_array_steps_to_mpos (float *position, int32_t *steps);
bool system_xy_at_fixture (coord_system_id_t id, float tolerance);
void system_raise_alarm (alarm_code_t alarm);
void system_init_switches (void);
void system_command_help (void);
void system_output_help (const sys_command_t *commands, uint32_t num_commands);
void system_register_commands (sys_commands_t *commands);

void system_clear_tlo_reference (axes_signals_t homing_cycle);
void system_add_rt_report (report_tracking_t report);
report_tracking_flags_t system_get_rt_report_flags (void);

// Special handlers for setting and clearing grblHAL's real-time execution flags.
#define system_set_exec_state_flag(mask) hal.set_bits_atomic(&sys.rt_exec_state, (mask))
#define system_clear_exec_state_flag(mask) hal.clear_bits_atomic(&sys.rt_exec_state, (mask))
#define system_clear_exec_states() hal.set_value_atomic(&sys.rt_exec_state, 0)
#define system_set_exec_alarm(code) hal.set_value_atomic(&sys.rt_exec_alarm, (uint_fast16_t)(code))
#define system_clear_exec_alarm() hal.set_value_atomic(&sys.rt_exec_alarm, 0)

void control_interrupt_handler (control_signals_t signals);

#endif
