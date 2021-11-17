/*
  ioports.h - typedefs, API structure and functions for auxillary I/O

  Part of grblHAL

  Copyright (c) 2021 Terje Io

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

#pragma once

typedef enum {
    Port_Analog = 0,    //!< 0
    Port_Digital = 1    //!< 1
} io_port_type_t;

typedef enum {
    Port_Input = 0,    //!< 0
    Port_Output = 1    //!< 1
} io_port_direction_t;

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

__NOTE:__ The latest value read is stored in \ref #sys  \ref #sys#var5399.

\param type as an \a #io_port_type_t enum value.
\param port port number.
\param wait_mode a #wait_mode_t enum value.
\param timeout in seconds, ignored if wait_mode is #WaitMode_Immediate (0).
\returns read value if successful, -1 otherwise.
*/
typedef int32_t (*wait_on_input_ptr)(io_port_type_t type, uint8_t port, wait_mode_t wait_mode, float timeout);

/*! \brief Pointer to function for setting pin description for a digital or analog port.
\param type as an \a #io_port_type_t enum value.
\param dir as an \a #io_port_direction_t enum value.
\param port port number.
\param s pointer to null terminated description string.
*/
typedef void (*set_pin_description_ptr)(io_port_type_t type, io_port_direction_t dir, uint8_t port, const char *s);

/*! \brief Pointer to function for getting information about a digital or analog port.
\param type as an \a #io_port_type_t enum value.
\param dir as an \a #io_port_direction_t enum value.
\param port port number.
\returns pointer to port information in a xbar_t struct if successful, NULL if not.
*/
typedef xbar_t *(*get_pin_info_ptr)(io_port_type_t type, io_port_direction_t dir, uint8_t port);

/*! \brief Pointer to function for claiming a digital or analog port for exclusive use.
\param type as an \a #io_port_type_t enum value.
\param dir as an \a #io_port_direction_t enum value.
\param port port number.
\returns true if successful, false if not.
*/
typedef bool (*claim_port_ptr)(io_port_type_t type, io_port_direction_t dir, uint8_t *port, const char *description);

/*! \brief Pointer to function for swapping two digital or analog ports.
\param type as an \a #io_port_type_t enum value.
\param dir as an \a #io_port_direction_t enum value.
\param port_from port number.
\param port_to port number.
\returns true if successful, false if not.
*/
typedef bool (*swap_pins_ptr)(io_port_type_t type, io_port_direction_t dir, uint8_t port_from, uint8_t port_to);

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
    uint8_t num_digital_in;                         //!< Number of digital inputs available.
    uint8_t num_digital_out;                        //!< Number of digital outputs available.
    uint8_t num_analog_in;                          //!< Number of analog inputs available.
    uint8_t num_analog_out;                         //!< Number of analog outputs available.
    digital_out_ptr digital_out;                    //!< Optional handler for setting a digital output.
    analog_out_ptr analog_out;                      //!< Optional handler for setting an analog output.
    wait_on_input_ptr wait_on_input;                //!< Optional handler for reading a digital or analog input.
    set_pin_description_ptr set_pin_description;    //!< Optional handler for setting a description of an auxillary pin.
    get_pin_info_ptr get_pin_info;                  //!< Optional handler for getting information about an auxillary pin.
    claim_port_ptr claim;                           //!< Optional handler for claiming an auxillary pin for exclusive use.
    swap_pins_ptr swap_pins;                        //!< Optional handler for swapping pins.
    ioport_register_interrupt_handler_ptr register_interrupt_handler;
} io_port_t;

uint8_t ioports_available (io_port_type_t type, io_port_direction_t dir);
bool ioport_claim (io_port_type_t type, io_port_direction_t dir, uint8_t *port, const char *description);
bool ioport_can_claim_explicit (void);

/*EOF*/
