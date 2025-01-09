/*
  ioports.h - typedefs, API structure and functions for auxiliary I/O

  Part of grblHAL

  Copyright (c) 2021-2025 Terje Io

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
\param on true to set output high, false to set it low
*/
typedef void (*digital_out_ptr)(uint8_t port, bool on);

/*! \brief Pointer to function for setting an analog output.
\param port port number.
\param value
\returns true if successful, false otherwise.
*/
typedef bool (*analog_out_ptr)(uint8_t port, float value);

/*! \brief Pointer to function for reading a digital or analog input.

__NOTE:__ The latest value read is stored in \ref #sys sys.var5399.

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
<br>__NOTE:__ The port information pointed to will be overwritten by the next call to this function.
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
\param description description of the pin function.
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

typedef bool (*ioports_enumerate_callback_ptr)(xbar_t *properties, uint8_t port, void *data);

//! Properties and handlers for auxiliary digital and analog I/O.
typedef struct {
    uint8_t num_digital_in;                         //!< Number of digital inputs available.
    uint8_t num_digital_out;                        //!< Number of digital outputs available.
    uint8_t num_analog_in;                          //!< Number of analog inputs available.
    uint8_t num_analog_out;                         //!< Number of analog outputs available.
    digital_out_ptr digital_out;                    //!< Optional handler for setting a digital output.
    analog_out_ptr analog_out;                      //!< Optional handler for setting an analog output.
    wait_on_input_ptr wait_on_input;                //!< Optional handler for reading a digital or analog input.
    set_pin_description_ptr set_pin_description;    //!< Optional handler for setting a description of an auxiliary pin.
    get_pin_info_ptr get_pin_info;                  //!< Optional handler for getting information about an auxiliary pin.
    claim_port_ptr claim;                           //!< Optional handler for claiming an auxiliary pin for exclusive use.
    swap_pins_ptr swap_pins;                        //!< Optional handler for swapping pins.
    ioport_register_interrupt_handler_ptr register_interrupt_handler;
} io_port_t;

uint8_t ioports_available (io_port_type_t type, io_port_direction_t dir);
xbar_t *ioport_get_info (io_port_type_t type, io_port_direction_t dir, uint8_t port);
bool ioport_claim (io_port_type_t type, io_port_direction_t dir, uint8_t *port, const char *description);
bool ioport_can_claim_explicit (void);
uint8_t ioport_find_free (io_port_type_t type, io_port_direction_t dir, const char *description);
bool ioports_enumerate (io_port_type_t type, io_port_direction_t dir, pin_cap_t filter, ioports_enumerate_callback_ptr callback, void *data);
void ioport_assign_function (aux_ctrl_t *aux_ctrl, pin_function_t *function);
void ioport_assign_out_function (aux_ctrl_out_t *aux_ctrl, pin_function_t *function);
bool ioport_analog_out_config (uint8_t port, pwm_config_t *config);
bool ioport_digital_pwm_config (uint8_t port, pwm_config_t *config);
bool ioport_digital_in_config (uint8_t port, gpio_in_config_t *config);
bool ioport_enable_irq (uint8_t port, pin_irq_mode_t irq_mode, ioport_interrupt_callback_ptr handler);
bool ioport_digital_out_config (uint8_t port, gpio_out_config_t *config);

//

struct io_ports_data;

typedef struct {
    uint8_t n_ports;
    uint8_t n_start;
    uint8_t *map;
} io_ports_detail_t;

typedef struct io_ports_data {
    char *pnum;
    io_ports_detail_t in;
    io_ports_detail_t out;
    char *(*get_pnum)(struct io_ports_data *data, uint8_t port);
} io_ports_data_t;

//!* \brief Precalculated values that may be set/used by HAL driver to speed up analog input to PWM conversions. */
typedef struct {
    uint32_t f_clock;
    uint_fast16_t period;
    uint_fast16_t off_value;    //!< NOTE: this value holds the inverted version if software PWM inversion is enabled by the driver.
    uint_fast16_t min_value;
    uint_fast16_t max_value;
    float min;                  //!< Minimum analog input value.
    float pwm_gradient;
    bool invert_pwm;            //!< NOTE: set (by driver) when inversion is done in code
    bool always_on;
} ioports_pwm_t;

bool ioports_add (io_ports_data_t *ports, io_port_type_t type, uint8_t n_in, uint8_t n_out);
void ioports_add_settings (driver_settings_load_ptr settings_loaded, setting_changed_ptr setting_changed);
void ioport_save_input_settings (xbar_t *xbar, gpio_in_config_t *config);
void ioport_save_output_settings (xbar_t *xbar, gpio_out_config_t *config);
void ioport_setting_changed (setting_id_t id);
#define iports_get_pnum(type, port) type.get_pnum(&type, port)
#define ioports_map(type, port) ( type.map ? type.map[port] : port )
uint8_t ioports_map_reverse (io_ports_detail_t *type, uint8_t port);
bool ioports_precompute_pwm_values (pwm_config_t *config, ioports_pwm_t *pwm_data, uint32_t clock_hz);
uint_fast16_t ioports_compute_pwm_value (ioports_pwm_t *pwm_data, float value);

/*EOF*/
