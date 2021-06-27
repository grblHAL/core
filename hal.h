/*
  hal.h - HAL (Hardware Abstraction Layer) entry points structures and capabilities type

  Part of grblHAL

  Copyright (c) 2016-2021 Terje Io

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

/*! \file
    \brief HAL and core function pointer and data structures definitions.
*/


#ifndef _HAL_H_
#define _HAL_H_

#include "grbl.h"
#include "gcode.h"
#include "system.h"
#include "coolant_control.h"
#include "spindle_control.h"
#include "crossbar.h"
#include "stepper.h"
#include "nvs.h"
#include "stream.h"
#include "probe.h"
#include "plugins.h"
#include "settings.h"
#include "report.h"

#define HAL_VERSION 8

/// Bitmap flags for driver capabilities, to be set by driver in driver_init(), flags may be cleared after to switch off option.
typedef union {
    uint32_t value; //!< All bitmap flags.
    struct {
        uint32_t mist_control              :1, //!< Mist control (M7) is supported.
                 variable_spindle          :1, //!< Variable spindle speed is supported.
                 spindle_dir               :1, //!< Spindle direction (M4) is supported.
                 software_debounce         :1, //!< Software debounce of input switches signals is supported.
                 step_pulse_delay          :1, //!< Stepper step pulse delay is supported.
                 limits_pull_up            :1, //!< Pullup resistors for limit inputs are are supported.
                 control_pull_up           :1, //!< Pullup resistors for control inputs are supported.
                 probe_pull_up             :1, //!< Pullup resistors for probe inputs are supported.
                 amass_level               :2, // 0...3 Deprecated?
                 spindle_at_speed          :1, //!< Spindle at speed feedback is supported.
                 laser_ppi_mode            :1, //!< Laser PPI (Pulses Per Inch) mode is supported.
                 spindle_sync              :1, //!< Spindle synced motion is supported.
                 sd_card                   :1,
                 bluetooth                 :1,
                 ethernet                  :1,
                 wifi                      :1,
                 spindle_pwm_invert        :1, //!< Spindle PWM output can be inverted.
                 spindle_pid               :1,
                 mpg_mode                  :1,
                 spindle_pwm_linearization :1,
                 atc                       :1, //!< Automatic tool changer (ATC) is supported.
                 no_gcode_message_handling :1,
                 dual_spindle              :1,
                 odometers                 :1,
                 unassigned                :7;
    };
} driver_cap_t;


/*! \brief Pointer to function to be called when a soft reset occurs. */
typedef void (*driver_reset_ptr)(void);

/****************
 *  I/O stream  *
 ****************/

/*! \brief Pointer to function for getting number of characters available or free in a stream buffer.
\returns number of characters available or free.
*/
typedef uint16_t (*get_stream_buffer_count_ptr)(void);

/*! \brief Pointer to function for reading a single character from a input stream.
\returns character or -1 if none available.
*/
typedef int16_t (*stream_read_ptr)(void);

/*! \brief Pointer to function for writing a null terminated string to the output stream.
\param s pointer to null terminated string.

__NOTE:__ output might be buffered until an #ASCII_LF is output, this is usually done by USB or network driver code to improve throughput.
*/
typedef void (*stream_write_ptr)(const char *s);

/*! \brief Pointer to function for writing a \a n character long string to the output stream.
\param s pointer to string.
\param c number of characters to write.
*/
typedef void (*stream_write_n_ptr)(const char *s, uint16_t len);


/*! \brief Pointer to function for writing a single character to the output stream.
\param \a n characters to write.
*/
typedef bool (*stream_write_char_ptr)(const char c);

/*! \brief Pointer to function for extracting real-time commands from the input stream and enqueue them for processing.
This should be called by driver code prior to inserting a character into the input buffer.
\param c character to check.
\returns true if extracted, driver code should not insert the character into the input buffer if so.

__NOTE:__ This is set up by the core at startup and may be changed at runtime by the core and/or plugin code.
*/
typedef bool (*enqueue_realtime_command_ptr)(char data);

/*! \brief Pointer to function for setting the stream baud rate.
\param baud_rate
\returns true if successful.
*/
typedef bool (*set_baud_rate_ptr)(uint32_t baud_rate);

/*! \brief Pointer to function for flushing a stream buffer. */
typedef void (*flush_stream_buffer_ptr)(void);

/*! \brief Pointer to function for flushing the input buffer and inserting an #ASCII_CAN character.

This function is typically called by the _enqueue_realtime_command_ handler when CMD_STOP or CMD_JOG_CANCEL character is processed.
The #ASCII_CAN character might be checked for and used by upstream code to flush any buffers it may have.
*/
typedef void (*cancel_read_buffer_ptr)(void);

/*! \brief Pointer to function for blocking reads from and restoring a input buffer.

This function is called with the _await_ parameter true on executing a tool change command (M6),
 this shall block further reading from the input buffer.
The core function stream_rx_suspend() can be called with the _await_ parameter to do this,
it will replace the _hal.stream.read_ handler with a pointer to the dummy function stream_get_null().

Reading from the input is blocked until a tool change aknowledge character #CMD_TOOL_ACK is received,
 when the driver receives this the input buffer is to be saved away and reading from the input resumed by
 restoring the _hal.stream.read_ handler with its own read character function.
Driver code can do this by calling the core function stream_rx_backup().

When the tool change is complete or a soft reset is executed the core will call this function with the _await_ parameter false,
 if the driver code called stream_rx_suspend() to block input it shall then call it again with the _await_ parameter as input to restore it.

\param await bool
\returns \a true if there is data in the buffer, \a false otherwise.
*/
typedef bool (*suspend_read_ptr)(bool await);

/*! \brief Pointer to function for disabling/enabling stream input.

Typically used to disable receive interrupts so that real-time command processing for the stream is blocked.
Usually it is desirable to block processing when another stream provides the input, but sometimes not.
E.g. when input is from a SD card real-time commands from the stream that initiated SD card streaming is needed
for handing feed-holds, overrides, soft resets etc.

\param disable \a true to disable stream, \a false to enable,
*/

typedef bool (*disable_stream_ptr)(bool disable);

//! Properties and handlers for stream I/O
typedef struct {
    stream_type_t type;                                     //!< Type of stream.
    bool connected;                                         //!< Set to true by the driver if stream is connected. _Optional._ Under consideration.
    get_stream_buffer_count_ptr get_rx_buffer_free;         //!< Handler for getting number of free characters in the input buffer.
    stream_write_ptr write;                                 //!< Handler for writing string to current output stream only.
    stream_write_ptr write_all;                             //!< Handler for writing string to all active output streams.
    stream_write_char_ptr write_char;                       //!< Handler for writing a single character to current stream only.
    stream_read_ptr read;                                   //!< Handler for reading a single character from the input stream.
    flush_stream_buffer_ptr reset_read_buffer;              //!< Handler for flushing the input buffer.
    cancel_read_buffer_ptr cancel_read_buffer;              //!< Handler for flushing the input buffer and inserting an #ASCII_CAN character.
    suspend_read_ptr suspend_read;                          //!< Optional handler for saving away and restoring the current input buffer.
    stream_write_n_ptr write_n;                             //!< Optional handler for writing n characters to current output stream only. Required for Modbus support.
    disable_stream_ptr disable;                             //!< Optional handler for disabling/enabling a stream. Recommended?
    get_stream_buffer_count_ptr get_rx_buffer_count;        //!< Optional handler for getting number of characters in the input buffer.
    get_stream_buffer_count_ptr get_tx_buffer_count;        //!< Optional handler for getting number of characters in the output buffer(s). Count shall include any unsent characters in any transmit FIFO and/or transmit register. Required for Modbus support.
    flush_stream_buffer_ptr reset_write_buffer;             //!< Optional handler for flushing the output buffer. Any transmit FIFO shall be flushed as well. Required for Modbus support.
    set_baud_rate_ptr set_baud_rate;                        //!< Optional handler for setting the stream baud rate. Required for Modbus support, recommended for Bluetooth support.
    enqueue_realtime_command_ptr enqueue_realtime_command;  //!< Handler for extracting real-time commands from the input stream. _Set by the core at startup._
} io_stream_t;

/*! \brief Optional pointer to function for switching between I/O streams.
\param stream pointer to io_stream_t
\returns true if switch was successful

__NOTE:__ required if the networking plugin is to be supported.
*/
typedef bool (*stream_select_ptr)(const io_stream_t *stream);


/*************
 *  Aux I/O  *
 *************/

/*! \brief Pointer to function for setting a digital output.
\param port port number
\param on true to set ouput high, false to set it low
*/
typedef void (*digital_out_ptr)(uint8_t port, bool on);

/*! \brief Pointer to function for setting an analog output.
\param port port number.
\param value
\returns true if successful, false otherwise.
*/
typedef bool (*analog_out_ptr)(uint8_t port, float value);

/*! \brief Pointer to function for reading a digital or analog input.

__NOTE:__ The latest value read is stored in \ref #sys  \ref #sys#var5933.

\param digital true if port is digital, false if analog.
\param port port number.
\param wait_mode a #wait_mode_t enum value.
\param timeout in seconds, ignored if wait_mode is #WaitMode_Immediate (0).
\returns read value if successful, -1 otherwise.
*/
typedef int32_t (*wait_on_input_ptr)(bool digital, uint8_t port, wait_mode_t wait_mode, float timeout);

/*! \brief Pointer to function for getting information about a digital or analog port.
\param digital true if port is digital, false if analog.
\param output true if port is an output, false if an input.
\param port port number.
\returns pointer to port information in a xbar_t struct if successful, NULL if not.
*/
typedef xbar_t *(*get_pin_info_ptr)(bool digital, bool output, uint8_t port);

/*! \brief Pointer to callback function for input port interrupt events.
\param port port number.
\param state true if port level is high, false if it is low.
*/
typedef void (*ioport_interrupt_callback_ptr)(uint8_t port, bool state);

/*! \brief Pointer to function for registering or deregistering an interrupt handler for a digital input port.
\param port port number.
\param irq_mode a \a #pin_irq_mode_t enum value.
\param interrupt_callback pointer to the callback function to register or NULL to deregister the current callback.
\returns true if successful, false otherwise.
*/
typedef bool (*ioport_register_interrupt_handler_ptr)(uint8_t port, pin_irq_mode_t irq_mode, ioport_interrupt_callback_ptr interrupt_callback);

//! Properties and handlers for auxillary digital and analog I/O.
typedef struct {
    uint8_t num_digital_in;             //!< Number of digital inputs available.
    uint8_t num_digital_out;            //!< Number of digital outputs available.
    uint8_t num_analog_in;              //!< Number of analog inputs available.
    uint8_t num_analog_out;             //!< Number of analog outputs available.
    digital_out_ptr digital_out;        //!< Optional handler for setting a digital output.
    analog_out_ptr analog_out;          //!< Optional handler for setting an analog output.
    wait_on_input_ptr wait_on_input;    //!< Optional handler for reading a digital or analog input.
    get_pin_info_ptr get_pin_info;      //!< Optional handler for getting information about an auxillary pin.
    ioport_register_interrupt_handler_ptr register_interrupt_handler;
} io_port_t;

/*! \brief Pointer to callback function for pin enumerations.
\param pin pointer to the \a xbar_t structure holding the pin information.
*/
typedef void (*pin_info_ptr)(xbar_t *pin);

/*! \brief Pointer to function for enumerate pin information.
\param low_level true if low level information is required, false if used for reporting.
\param callback pointer to a \a pin_info_ptr type function to receive the pin information.
The callback function will be called for each pin.
*/
typedef void (*enumerate_pins_ptr)(bool low_level, pin_info_ptr callback);


/*************
 *  Spindle  *
 *************/

/*! \brief Pointer to function for setting the spindle state.
\param state a \a spindle_state_t union variable.
\param rpm spindle RPM.
*/
typedef void (*spindle_set_state_ptr)(spindle_state_t state, float rpm);

/*! \brief Pointer to function for getting the spindle state.
\returns state in a \a spindle_state_t union variable.
*/
typedef spindle_state_t (*spindle_get_state_ptr)(void);

#ifdef SPINDLE_PWM_DIRECT

/*! \brief Pointer to function for converting a RPM value to a PWM value.

Typically this is a wrapper for the spindle_compute_pwm_value() function provided by the core.

\param rpm spindle RPM.
\returns the corresponding PWM value.
*/
typedef uint_fast16_t (*spindle_get_pwm_ptr)(float rpm);

/*! \brief Pointer to function for updating spindle speed on the fly.
\param pwm new spindle PWM value.
\returns the actual PWM value used.

__NOTE:__ this function will be called from an interrupt context.
*/
typedef void (*spindle_update_pwm_ptr)(uint_fast16_t pwm);

#else

typedef void (*spindle_update_rpm_ptr)(float rpm);

#endif

/*! \brief Pointer to function for getting spindle data.
\param request request type as a \a #spindle_data_request_t enum.
\returns pointer to the requested information in a spindle_data_t structure.

__NOTE:__ this function requires input from a spindle encoder.
*/
typedef spindle_data_t *(*spindle_get_data_ptr)(spindle_data_request_t request);

/*! \brief Pointer to function for resetting spindle data. */
typedef void (*spindle_reset_data_ptr)(void);

/*! \brief Pointer to function for outputting a spindle on pulse.
Used for Pulses Per Inch (PPI) laser mode.
\param pulse_length spindle on length in microseconds.
*/
typedef void (*spindle_pulse_on_ptr)(uint_fast16_t pulse_length);

//! Handlers for spindle support.
typedef struct {
    spindle_set_state_ptr set_state;    //!< Handler for setting spindle state.
    spindle_get_state_ptr get_state;    //!< Handler for getting spindle state.
#ifdef SPINDLE_PWM_DIRECT
    spindle_get_pwm_ptr get_pwm;        //!< Handler for calculating spindle PWM value from RPM.
    spindle_update_pwm_ptr update_pwm;  //!< Handler for updating spindle PWM output.
#else
    spindle_update_rpm_ptr update_rpm;  //!< Handler for updating spindle RPM.
#endif
    // Optional entry points:
    spindle_get_data_ptr get_data;      //!< Optional handler for getting spindle data. Required for spindle sync.
    spindle_reset_data_ptr reset_data;  //!< Optional handler for resetting spindle data. Required for spindle sync.
    spindle_pulse_on_ptr pulse_on;      //!< Optional handler for Pulses Per Inch (PPI) mode. Required for the laser PPI plugin.
} spindle_ptrs_t;


/*************
 *  Coolant  *
 *************/

/*! \brief Pointer to function for setting the coolant state.
\param state a \a coolant_state_t union variable.
*/
typedef void (*coolant_set_state_ptr)(coolant_state_t state);

/*! \brief Pointer to function for getting the coolant state.
\returns state in a \a coolant_state_t union variable.
*/
typedef coolant_state_t (*coolant_get_state_ptr)(void);

//! Handlers for coolant support.
typedef struct {
    coolant_set_state_ptr set_state;    //!< Handler for setting coolant state.
    coolant_get_state_ptr get_state;    //!< Handler for getting coolant state.
} coolant_ptrs_t;


/********************
 *  Limit switches  *
 ********************/

/*! \brief Pointer to function for enabling/disabling limit switches functionality.
\param on true to enable limit switches interrupts.
\param homing true when machine is in a homing cycle. Usually ignored by driver code, may be used if Trinamic drivers are supported.
*/
typedef void (*limits_enable_ptr)(bool on, bool homing);

/*! \brief Pointer to function for getting limit switches state.
\returns switch states in a limit_signals_t struct.
*/
typedef limit_signals_t (*limits_get_state_ptr)(void);

/*! \brief Pointer to callback function for reporting limit switches events (interrupts). _Set by the core on startup._
\param switch states in a limit_signals_t struct.
*/
typedef void (*limit_interrupt_callback_ptr)(limit_signals_t state);

//! Limit switches handlers
typedef struct {
    limits_enable_ptr enable;                           //!< Handler for enabling limits handling mode.
    limits_get_state_ptr get_state;                     //!< Handler for getting limit switches status.
    limit_interrupt_callback_ptr interrupt_callback;    //!< Callback for informing about limit switches events. _Set by the core at startup._
} limits_ptrs_t;


/************
 *  Homing  *
 ************/

//! Limit switches handler for homing cycle.
typedef struct {
    limits_get_state_ptr get_state;                     //!< Handler for getting limit switches status. Usually set to the same function as _hal.limits.get_state_.
} homing_ptrs_t;


/*****************************
 *  Control signal switches  *
 *****************************/

/*! \brief Pointer to function for getting control switches state.
\returns switch states in a control_signals_t struct.
*/
typedef control_signals_t (*control_signals_get_state_ptr)(void);

/*! \brief Pointer to callback function for reporting control switches events (interrupts). _Set by the core on startup._
\param switch states in a control_signals_t struct.
*/
typedef void (*control_signals_callback_ptr)(control_signals_t signals);

//! Control switches handlers
typedef struct {
    control_signals_get_state_ptr get_state;            //!< Handler for getting limit switches status.
    control_signals_callback_ptr interrupt_callback;    //!< Callback for informing about control switches events. _Set by the core at startup.
} control_signals_ptrs_t;

/**************
 *  Steppers  *
 **************/

/*! \brief Pointer to function for enabling all stepper motors and the main stepper interrupt.

The first interrupt should be generated after a short delay to allow the drivers time to apply power to the motors.
*/
typedef void (*stepper_wake_up_ptr)(void);


/*! \brief Pointer to function for disabling the main stepper interrupt.

\param clear_signals when true stepper motor outputs can be reset to the default state. This parameter can often be ignored.

__NOTE:__ this function will be called from an interrupt context
*/
typedef void (*stepper_go_idle_ptr)(bool clear_signals);

/*! \brief Pointer to function for enabling/disabling stepper motors.

\param enable a \a axes_signals_t union containing separate flags for each motor to enable/disable.

__NOTE:__ this function may be called from an interrupt context
*/
typedef void (*stepper_enable_ptr)(axes_signals_t enable);

/*! \brief Pointer to function for enabling/disabling step signals for individual motors.

Used by auto squaring functionality where two motors are employed for one or more axes.

\param enable a \a axes_signals_t union containing separate flags for each motor to enable/disable.
\param mode a \a #squaring_mode_t enum for which side to enable/disable.
*/
typedef void (*stepper_disable_motors_ptr)(axes_signals_t axes, squaring_mode_t mode);

/*! \brief Pointer to function for setting the step pulse interrupt rate for the next motion segment.

The driver should limit the maximum time between step interrupts to a few seconds if a 32-bit timer is used.

\param cycles_per_tick number of clock ticks between main stepper interrupts.

__NOTE:__ this function will be called from an interrupt context.
*/
typedef void (*stepper_cycles_per_tick_ptr)(uint32_t cycles_per_tick);

/*! \brief Pointer to function for setting up steppers for the next step pulse.

For plain drivers the most of the information pointed to by the \a stepper argument can be ignored.
The most used fields are these:
<br>\ref stepper.step_outbits
<br>\ref stepper.dir_outbits - these are only necessary to ouput to the drivers when \ref stepper.dir_change is true.

If the driver is to support spindle synced motion many more needs to be referenced...

\param stepper pointer to a \ref stepper struct containing information about the stepper signals to be output.

__NOTE:__ this function will be called from an interrupt context
*/
typedef void (*stepper_pulse_start_ptr)(stepper_t *stepper);

/*! \brief Pointer to function for outputting a single step pulse and direction signal.

This is for an experimental implementation of plasma Torch Height Control (THC) and may be removed and/or changed.

\param step_outbits a \a #axes_signals_t union containing the axes to output a step signal for.
\param step_outbits a \a #axes_signals_t union containing the axes to output a direction signal for.

__NOTE:__ this function will be called from an interrupt context
*/
typedef void (*stepper_output_step_ptr)(axes_signals_t step_outbits, axes_signals_t dir_outbits);

/*! \brief Pointer to function for getting which axes are configured for auto squaring.

\returns which axes are configured for auto squaring in an \a axes_signals_t union.
*/
typedef axes_signals_t (*stepper_get_auto_squared_ptr)(void);

/*! \brief Pointer to callback function for outputting the next direction and step pulse signals. _Set by the core on startup._

To be called by the driver from the main stepper interrupt handler (when the timer times out).
*/
typedef void (*stepper_interrupt_callback_ptr)(void);

//! Stepper motor handlers
typedef struct {
    stepper_wake_up_ptr wake_up;                        //!< Handler for enabling stepper motor power and main stepper interrupt.
    stepper_go_idle_ptr go_idle;                        //!< Handler for disabling main stepper interrupt and optionally reset stepper signals.
    stepper_enable_ptr enable;                          //!< Handler for enabling/disabling stepper motor power for individual motors.
    stepper_disable_motors_ptr disable_motors;          //!< Optional handler for enabling/disabling stepper motor step signals for individual motors.
    stepper_cycles_per_tick_ptr cycles_per_tick;        //!< Handler for setting the step pulse rate for the next motion segment.
    stepper_pulse_start_ptr pulse_start;                //!< Handler for starting outputting direction signals and a step pulse.
    stepper_interrupt_callback_ptr interrupt_callback;  //!< Callback for informing about the next step pulse to output. _Set by the core at startup._
    stepper_get_auto_squared_ptr get_auto_squared;      //!< Optional handler getting which axes are configured for auto squaring.
    stepper_output_step_ptr output_step;                //!< Optional handler for outputting a single step pulse. _Experimental._
} stepper_ptrs_t;


/**************
 *  ms delay  *
 **************/

/*! \brief Signature of delay callback functions. */
typedef void (*delay_callback_ptr)(void);

//! Delay struct, currently not used by core - may be used by drivers.
typedef struct {
   volatile uint32_t ms;
   delay_callback_ptr callback;
} delay_t;


/***********
 *  Probe  *
 ***********/

/*! \brief Pointer to function for getting probe status.
\returns probe state in a \a #probe_state_t enum.
*/
typedef probe_state_t (*probe_get_state_ptr)(void);

/*! \brief Pointer to function for setting probe operation mode.
\param is_probe_away true if probing away from the workpiece, false otherwise. When probing away the signal must be inverted in the probe_get_state_ptr() implementation.
\param probing true if probe cycle is active, false otherwise.
*/
typedef void (*probe_configure_ptr)(bool is_probe_away, bool probing);

/*! \brief Pointer to function for toggling probe connected status.

If the driver does not support a probe connected input signal this can be used to implement
toggling of probe connected status via a #CMD_PROBE_CONNECTED_TOGGLE real time command.
*/
typedef void (*probe_connected_toggle_ptr)(void);

//! Handlers for probe input(s).
typedef struct {
    probe_configure_ptr configure;                  //!< Optional handler for setting probe operation mode.
    probe_get_state_ptr get_state;                  //!< Optional handler for getting probe status.
    probe_connected_toggle_ptr connected_toggle;    //!< Optional handler for toggling probe connected status.
} probe_ptrs_t;


/*******************************
 *  Tool selection and change  *
 *******************************/

/*! \brief Pointer to function for selecting a tool.
\param tool pointer to tool_data_t struct.
\param next \a true if tool is selected for next the next tool change (M6), \a false to as set current tool.
*/
typedef void (*tool_select_ptr)(tool_data_t *tool, bool next);

/*! \brief Pointer to function for executing a tool change.
\param gc_state pointer to parser_state_t struct.
*/
typedef status_code_t (*tool_change_ptr)(parser_state_t *gc_state);

/*! \brief Handlers for tool changes.

If the driver (or a plugin) does not set these handlers the core will set them to its own
handlers for manual or semi-automatic tool change if the current input stream supports
the tool change protocol.
 */
typedef struct {
    tool_select_ptr select; //!< Optional handler for selecting a tool.
    tool_change_ptr change; //!< Optional handler for executing a tool change (M6).
} tool_ptrs_t;

/*****************
 *  User M-code  *
 *****************/

/*! \brief Pointer to function for checking if user defined M-code is supported.
\param mcode as a #user_mcode_t enum.
\returns the \a mcode argument if M-code is handled, #UserMCode_Ignore if not.
*/
typedef user_mcode_t (*user_mcode_check_ptr)(user_mcode_t mcode);

/*! \brief Pointer to function for validating parameters for a user defined M-code.

The M-code to validate is found in \a gc_block->user_mcode, parameter values in \a gc_block->values
 and corresponding parameter letters in the \a gc_block->words bitfield union.

Parameter values claimed by the M-code must be flagged in the \a gc_block->words bitfield union by setting the
 respective parameter letters to \a false or the parser will raise the #Status_GcodeUnusedWords error.

The validation function may set \a gc_block->user_mcode_sync to \a true if it is to be executed
after all buffered motions are completed, otherwise it will be executed immediately.

__NOTE:__ Valueless parameter letters are allowed for floats, these are set to NAN (not a number) if so.
The validation function should always test these by using the isnan() function in addition to any range checks.

\param gc_block pointer to a parser_block_t structure.
\param parameter_words pointer to a parameter_words_t structure. __NOTE:__ this parameter is deprecated and will be removed, use \a gc_block->words instead.
\returns #Status_OK enum value if validated ok, appropriate \ref status_code_t enum value if not or #Status_Unhandled if the M-code is not recognized.
*/
typedef status_code_t (*user_mcode_validate_ptr)(parser_block_t *gc_block, parameter_words_t *parameter_words);

/*! \brief Pointer to function for executing a user defined M-code.

The M-code to execute is found in \a gc_block->user_mcode, parameter values in \a gc_block->values
 and claimed/validated parameter letters in the \a gc_block->words bitfield union.

\param state as a #sys_state_t variable.
\param gc_block pointer to a parser_block_t structure.
\returns #Status_OK enum value if validated ok, appropriate \ref status_code_t enum value if not or #Status_Unhandled if M-code is not recognized.
*/
typedef void (*user_mcode_execute_ptr)(sys_state_t state, parser_block_t *gc_block);

/*! \brief Optional handlers for user defined M-codes.

Handlers may be chained so that several plugins can add M-codes.
Chaining is achieved by saving a copy of the current user_mcode_ptrs_t struct
 when the plugin is initialized and calling the same handler via the copy when a
 M-code is not recognized.
 */
typedef struct {
    user_mcode_check_ptr check;         //!< Handler for checking if a user defined M-code is supported.
    user_mcode_validate_ptr validate;   //!< Handler for validating parameters for a user defined M-code.
    user_mcode_execute_ptr execute;     //!< Handler for executing a user defined M-code.
} user_mcode_ptrs_t;

/*******************
 *  Encoder input  *
 *******************/

/*! \brief Pointer to function for getting number of encoders supported.
\returns number of encoders.
*/
typedef uint8_t (*encoder_get_n_encoders_ptr)(void);

/*! \brief Pointer to callback function to receive encoder events.
\param encoder pointer to a \a encoder_t struct.
\param position encoder position.
*/
typedef void (*encoder_on_event_ptr)(encoder_t *encoder, int32_t position);

/*! \brief Pointer to function for resetting encoder data.
\param id encoder id.
*/
typedef void (*encoder_reset_ptr)(uint_fast8_t id);

typedef struct {
    encoder_get_n_encoders_ptr get_n_encoders;  //!< Optional handler for getting number of encoders supported.
    encoder_on_event_ptr on_event;              //!< Optional callback handler for receiving encoder events.
    encoder_reset_ptr reset;                    //!< Optional handler for resetting data for an encoder.
} encoder_ptrs_t;

/*! \brief HAL structure used for the driver interface.

This structure contains properties and function pointers (to handlers) that the core uses to communicate with the driver.

Required function pointers has to be set up by the driver in the driver_init() function.

__NOTE:__ Those not marked as optional are required. If not assigned by the driver behaviour is undefined and
is likely to cause a controller crash at some point.
*/
typedef struct {
    uint32_t version;               //!< HAL version, _set by the core_. Driver should check against this in the _driver_init()_ function.
    char *info;                     //!< Pointer to driver info string, typically name of processor/platform.
    char *driver_version;           //!< Pointer to driver version date string in YYMMDD format.
    char *driver_options;           //!< Pointer to optional comma separated string with driver options.
    char *board;                    //!< Pointer to optional board name string.
    uint32_t f_step_timer;          //!< Frequency of main stepper timer in Hz.
    uint32_t rx_buffer_size;        //!< Input stream buffer size in bytes.
    uint32_t max_step_rate;         //!< Currently unused.
    uint8_t driver_axis_settings;   //!< Currently unused.

    /*! \brief Driver setup handler.
    Called once by the core after settings has been loaded. The driver should enable MCU peripherals in the provided function.
    \param settings pointer to settings_t structure.
    \returns true if completed sucessfully and the driver supports the _settings->version_ number, false otherwise.
    */
    bool (*driver_setup)(settings_t *settings);

    /*! \brief Millisecond delay.

    If the callback parameter is \a NULL the call blocks until the delay has expired otherwise it returns immediately
    and the callback function is called when the timeout expires.

    A pending callback can be terminated by calling the function with the delay set to \a 0 and the callback to \a NULL.
    \param ms number of milliseconds to delay.
    \param callback function as a \a delay_callback_ptr function pointer.
    */
    void (*delay_ms)(uint32_t ms, delay_callback_ptr callback);

    /*! \brief Atomically set bits.
    \param value pointer to 16 bit unsigned integer.
    \param bits bits to set.
    */
    void (*set_bits_atomic)(volatile uint_fast16_t *value, uint_fast16_t bits);

    /*! \brief Atomically clear bits.
    \param value pointer to 16 bit unsigned integer.
    \param bits bits to clear.
    \returns value before bits were cleared.
    */
    uint_fast16_t (*clear_bits_atomic)(volatile uint_fast16_t *value, uint_fast16_t v);

    /*! \brief Atomically set value.
    \param value pointer to 16 bit unsigned integer.
    \param v value to set.
    \returns value before value were set.
    */
    uint_fast16_t (*set_value_atomic)(volatile uint_fast16_t *value, uint_fast16_t bits);

    //! \brief Optional handler to disable global interrupts.
    void (*irq_enable)(void);

    //! \brief Optional handler to enable global interrupts.
    void (*irq_disable)(void);

    limits_ptrs_t limits;                   //!< Handlers for limit switches.
    homing_ptrs_t homing;                   //!< Handlers for limit switches, used by homing cycle.
    control_signals_ptrs_t control;         //!< Handlers for control switches.
    coolant_ptrs_t coolant;                 //!< Handlers for coolant.
    spindle_ptrs_t spindle;                 //!< Handlers for spindle.
    stepper_ptrs_t stepper;                 //!< Handlers for stepper motors.
    io_stream_t stream;                     //!< Handlers for stream I/O.
    stream_select_ptr stream_select;        //!< Optional handler for switching between I/O streams.
    settings_changed_ptr settings_changed;  //!< Callback handler to be called on settings loaded or settings changed events.
    probe_ptrs_t probe;                     //!< Optional handlers for probe input(s).
    tool_ptrs_t tool;                       //!< Optional handlers for tool changes.
    io_port_t port;                         //!< Optional handlers for axuillary I/O (adds support for M62-M66).
    driver_reset_ptr driver_reset;          //!< Optional handler, called on soft resets. Set to a dummy handler by the core at startup.
    nvs_io_t nvs;                           //!< Optional handlers for storing/retrieving settings and data to/from non-volatile storage (NVS).
    enumerate_pins_ptr enumerate_pins;      //!< Optional handler for enumerating pins used by the driver.
    bool (*driver_release)(void);           //!< Optional handler for releasing hardware resources before exiting.
    uint32_t (*get_elapsed_ticks)(void);    //!< Optional handler for getting number of elapsed 1ms tics since startup. Required by a number of plugins.
    void (*pallet_shuttle)(void);           //!< Optional handler for performing a pallet shuttle on program end (M60).
    void (*reboot)(void);                   //!< Optoional handler for rebooting the controller. This will be called when #ASCII_ESC followed by #CMD_REBOOT is received.

    user_mcode_ptrs_t user_mcode;           //!< Optional handlers for user defined M-codes.
    encoder_ptrs_t encoder;                 //!< Optional handlers for encoder support.

    /*! \brief Optional handler for getting the current axis positions.
    \returns the axis positions in an int32_array.
    */
    bool (*get_position)(int32_t (*position)[N_AXIS]);

#ifdef DEBUGOUT
    void (*debug_out)(bool on);
#endif

    /*! \brief Check for a soft reset or abort in blocking calls.
    Set up by core at startup.

    Typically called from stream output functions when the output buffer is full and
    they are blocking waiting for space in the buffer to become available.
    \returns false when the blocking loop should exit, true otherwise.
    */
    bool (*stream_blocking_callback)(void);

    driver_cap_t driver_cap;                //!< Basic driver capabilities flags.
    control_signals_t signals_cap;          //!< Control input signals supported by the core.

} grbl_hal_t;

// TODO: move the following structs to grbl.h?

/* TODO: add to grbl pointers so that a different formatting (xml, json etc) of reports may be implemented by driver?
typedef struct {
    status_code_t (*report_status_message)(status_code_t status_code);
    alarm_code_t (*report_alarm_message)(alarm_code_t alarm_code);
    message_code_t (*report_feedback_message)(message_code_t message_code);
    void (*report_init_message)(void);
    void (*report_grbl_help)(void);
    void (*report_echo_line_received)(char *line);
    void (*report_realtime_status)(void);
    void (*report_probe_parameters)(void);
    void (*report_ngc_parameters)(void);
    void (*report_gcode_modes)(void);
    void (*report_startup_line)(uint8_t n, char *line);
    void (*report_execute_startup_message)(char *line, status_code_t status_code);
} grbl_report_t;
*/

// Report entry points set by core at reset.

typedef status_code_t (*status_message_ptr)(status_code_t status_code);
typedef message_code_t (*feedback_message_ptr)(message_code_t message_code);

typedef struct {
    setting_output_ptr setting;
    status_message_ptr status_message;
    feedback_message_ptr feedback_message;
} report_t;

// Core event handler and other entry points.
// Most of the event handlers defaults to NULL, a few is set up to call a dummy handler for simpler code.

typedef void (*on_state_change_ptr)(sys_state_t state);
typedef void (*on_probe_completed_ptr)(void);
typedef void (*on_program_completed_ptr)(program_flow_t program_flow, bool check_mode);
typedef void (*on_execute_realtime_ptr)(sys_state_t state);
typedef void (*on_unknown_accessory_override_ptr)(uint8_t cmd);
typedef bool (*on_unknown_realtime_cmd_ptr)(char c);
typedef void (*on_report_options_ptr)(bool newopt);
typedef void (*on_report_command_help_ptr)(void);
typedef void (*on_global_settings_restore_ptr)(void);
typedef setting_details_t *(*on_get_settings_ptr)(void); // NOTE: this must match the signature of the same definition in
                                                         // the setting_details_t structure in settings.h!
typedef void (*on_realtime_report_ptr)(stream_write_ptr stream_write, report_tracking_flags_t report);
typedef void (*on_unknown_feedback_message_ptr)(stream_write_ptr stream_write);
typedef bool (*on_laser_ppi_enable_ptr)(uint_fast16_t ppi, uint_fast16_t pulse_length);
typedef status_code_t (*on_unknown_sys_command_ptr)(sys_state_t state, char *line); // return Status_Unhandled.
typedef status_code_t (*on_user_command_ptr)(char *line);
typedef sys_commands_t *(*on_get_commands_ptr)(void);

typedef struct {
    // report entry points set by core at reset.
    report_t report;
    // grbl core events - may be subscribed to by drivers or by the core.
    on_state_change_ptr on_state_change;
    on_probe_completed_ptr on_probe_completed;
    on_program_completed_ptr on_program_completed;
    on_execute_realtime_ptr on_execute_realtime;
    on_unknown_accessory_override_ptr on_unknown_accessory_override;
    on_report_options_ptr on_report_options;
    on_report_command_help_ptr on_report_command_help;
    on_global_settings_restore_ptr on_global_settings_restore;
    on_get_settings_ptr on_get_settings;
    on_realtime_report_ptr on_realtime_report;
    on_unknown_feedback_message_ptr on_unknown_feedback_message;
    on_unknown_realtime_cmd_ptr on_unknown_realtime_cmd;
    on_unknown_sys_command_ptr on_unknown_sys_command; // return Status_Unhandled if not handled.
    on_get_commands_ptr on_get_commands;
    on_user_command_ptr on_user_command;
    on_laser_ppi_enable_ptr on_laser_ppi_enable;
    // core entry points - set up by core before driver_init() is called.
    bool (*protocol_enqueue_gcode)(char *data);
} grbl_t;

extern grbl_t grbl;
extern grbl_hal_t hal; //!< Global HAL struct.

/*! \brief Driver main entry point.
This will be called once by the core after the HAL structure has been nulled and handlers provided by the core is set up.
The driver should initialize all the required handlers and capabilities flags in this function.
On successful return from this function the core will load settings from non-volatile storage (NVS) if handler functions were provided or load default values if not.
Then _hal.driver_setup()_ will be called so that the driver can configure the remaining MCU peripherals.

__NOTE__: This is the only driver function that is called directly from the core, all others are called via HAL function pointers.

\returns true if completed sucessfully and the driver matches the _hal.version number_, false otherwise.
*/
extern bool driver_init (void);

#endif
