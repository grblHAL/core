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

//#define IOPORTS_KEEP_DEPRECATED

#define IOPORT_UNASSIGNED 255

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
\param on \a true to set output high, \a false to set it low
*/
typedef void (*digital_out_ptr)(uint8_t port, bool on);

/*! \brief Pointer to function for setting an analog output.
\param port port number.
\param value
\returns \a true if successful, \a false otherwise.
*/
typedef bool (*analog_out_ptr)(uint8_t port, float value);

/*! \brief Pointer to function for reading a digital or analog input.

__NOTE:__ The latest value read is stored in \ref #sys sys.var5399.

\param type as an \a #io_port_type_t enum value.
\param port port number.
\param wait_mode a #wait_mode_t enum value.
\param timeout in seconds, ignored if wait_mode is #WaitMode_Immediate (0).
\returns read value if successful, \a -1 otherwise.
*/
typedef int32_t (*wait_on_input_ptr)(io_port_type_t type, uint8_t port, wait_mode_t wait_mode, float timeout);

/*! \brief Pointer to function for reading a digital or analog input.

__NOTE:__ The latest value read is stored in \ref #sys sys.var5399.

\param type as an \a #io_port_type_t enum value.
\param port port number.
\param wait_mode a #wait_mode_t enum value.
\param timeout in seconds, ignored if wait_mode is #WaitMode_Immediate (0).
\returns read value if successful, \a -1 otherwise.
*/
typedef int32_t (*ll_wait_on_input_ptr)(uint8_t port, wait_mode_t wait_mode, float timeout);


/*! \brief Pointer to function for setting pin description for a digital or analog port.
\param type as an \a #io_port_type_t enum value.
\param dir as an \a #io_port_direction_t enum value.
\param port port number.
\param s pointer to null terminated description string.
*/
typedef void (*set_pin_description_ptr)(io_port_type_t type, io_port_direction_t dir, uint8_t port, const char *s);

/*! \brief Pointer to function for setting pin description for a digital or analog port.
\param type as an \a #io_port_type_t enum value.
\param dir as an \a #io_port_direction_t enum value.
\param port port number.
\param s pointer to null terminated description string.
*/
typedef void (*ll_set_pin_description_ptr)(io_port_direction_t dir, uint8_t port, const char *s);

/*! \brief Pointer to function for getting information about a digital or analog port.
<br>__NOTE:__ The port information pointed to will be overwritten by the next call to this function.
\param type as an \a #io_port_type_t enum value.
\param dir as an \a #io_port_direction_t enum value.
\param port port number.
\returns pointer to port information in a \a #xbar_t struct if successful, \a NULL if not.
*/
typedef xbar_t *(*get_pin_info_ptr)(io_port_type_t type, io_port_direction_t dir, uint8_t port);

/*! \brief Pointer to function for getting information about a digital or analog port.
<br>__NOTE:__ The port information pointed to will be overwritten by the next call to this function.
\param type as an \a #io_port_type_t enum value.
\param dir as an \a #io_port_direction_t enum value.
\param port port number.
\returns pointer to port information in a \a #xbar_t struct if successful, \a  NULL if not.
*/
typedef xbar_t *(*ll_get_pin_info_ptr)(io_port_direction_t dir, uint8_t port);

/*! \brief Pointer to function for claiming a digital or analog port for exclusive use.
\param type as an \a #io_port_type_t enum value.
\param dir as an \a #io_port_direction_t enum value.
\param port port number.
\param description description of the pin function.
\returns \a true if successful, \a false if not.
*/
typedef bool (*claim_port_ptr)(io_port_type_t type, io_port_direction_t dir, uint8_t *port, const char *description);

/*! \brief Pointer to function for claiming a digital or analog port for exclusive use.
\param type as an \a #io_port_type_t enum value.
\param dir as an \a #io_port_direction_t enum value.
\param port port number.
\param description description of the pin function.
\returns \a true if successful, \a false if not.
*/
typedef bool (*ll_claim_port_ptr)(io_port_direction_t dir, uint8_t port, uint8_t user_port, const char *description);


/*! \brief Pointer to function for swapping two digital or analog ports.
\param type as an \a #io_port_type_t enum value.
\param dir as an \a #io_port_direction_t enum value.
\param port_from port number.
\param port_to port number.
\returns \a true if successful, \a false if not.
*/
typedef bool (*swap_pins_ptr)(io_port_type_t type, io_port_direction_t dir, uint8_t port_from, uint8_t port_to);

/*! \brief Pointer to callback function for input port interrupt events.
\param port port number.
\param state \a true if port level is high, \a false if it is low.
*/
typedef void (*ioport_interrupt_callback_ptr)(uint8_t port, bool state);

/*! \brief Pointer to function for registering or deregistering an interrupt handler for a digital input port.
\param port port number.
\param irq_mode a \a #pin_irq_mode_t enum value.
\param interrupt_callback pointer to the callback function to register or NULL to deregister the current callback.
\returns \a true if successful, \a false otherwise.
*/
typedef bool (*ioport_register_interrupt_handler_ptr)(uint8_t port, pin_irq_mode_t irq_mode, ioport_interrupt_callback_ptr interrupt_callback);

/*! \brief Pointer to function for registering or deregistering an interrupt handler for a digital input port.
\param port port number.
\param user_port port number to be used for the callback.
\param irq_mode a \a #pin_irq_mode_t enum value.
\param interrupt_callback pointer to the callback function to register or NULL to deregister the current callback.
\returns \a true if successful, \a false otherwise.
*/
typedef bool (*ll_ioport_register_interrupt_handler_ptr)(uint8_t port, uint8_t user_port, pin_irq_mode_t irq_mode, ioport_interrupt_callback_ptr interrupt_callback);

typedef bool (*ioports_enumerate_callback_ptr)(xbar_t *properties, uint8_t port, void *data);

//! Properties and handlers for auxiliary digital and analog I/O.
typedef struct {
    uint8_t num_digital_in;                         //!< Deprecated, use ioports_unclaimed() to get count.
    uint8_t num_digital_out;                        //!< Deprecated, use ioports_unclaimed() to get count.
    uint8_t num_analog_in;                          //!< Deprecated, use ioports_unclaimed() to get count.
    uint8_t num_analog_out;                         //!< Deprecated, use ioports_unclaimed() to get count.
    digital_out_ptr digital_out;                    //!< Optional handler for setting a digital output.
    analog_out_ptr analog_out;                      //!< Optional handler for setting an analog output.
    wait_on_input_ptr wait_on_input;                //!< Optional handler for reading a digital or analog input.
    set_pin_description_ptr set_pin_description;    //!< Optional handler for setting a description of an auxiliary pin.
    get_pin_info_ptr get_pin_info;                  //!< Optional handler for getting information about an auxiliary pin.
    claim_port_ptr claim;                           //!< Optional handler for claiming an auxiliary pin for exclusive use.
    swap_pins_ptr swap_pins;                        //!< Optional handler for swapping pins.
    ioport_register_interrupt_handler_ptr register_interrupt_handler;
} io_port_t;

typedef union {
    control_signals_t *control;
    coolant_state_t *coolant;
    limit_signals_t *limits;
} driver_caps_t __attribute__ ((__transparent_union__));

typedef union {
    uint8_t io;
    struct {
        uint8_t claim_explicit :1,
                digital_out    :1,
                analog_out     :1,
                wait_on_input  :1,
                configure      :1;
    };
} io_port_cando_t;

struct ioports_cfg;
struct ioports_handle; // members defined in ioports.c

typedef status_code_t (*ioport_set_value_ptr)(struct ioports_cfg *p, uint8_t *port, pin_cap_t caps, float value);
typedef float (*ioport_get_value_ptr)(struct ioports_cfg *p, uint8_t port);
typedef uint8_t (*ioport_get_next_ptr)(struct ioports_cfg *p, uint8_t port, const char *description, pin_cap_t caps);
typedef xbar_t *(*ioport_claim_ptr)(struct ioports_cfg *p, uint8_t *port, const char *description, pin_cap_t caps);

struct ioports_cfg {
    struct ioports_handle *handle;
    uint8_t n_ports;
    uint8_t port_max;
    const char port_maxs[4];
    ioport_get_value_ptr get_value;
    ioport_set_value_ptr set_value;
    ioport_get_next_ptr get_next;
    ioport_claim_ptr claim;
};

typedef struct ioports_cfg io_port_cfg_t;

io_port_cfg_t *ioports_cfg (io_port_cfg_t *p, io_port_type_t type, io_port_direction_t dir);
uint8_t ioports_available (io_port_type_t type, io_port_direction_t dir);
uint8_t ioports_unclaimed (io_port_type_t type, io_port_direction_t dir);
xbar_t *ioport_get_info (io_port_type_t type, io_port_direction_t dir, uint8_t port);
xbar_t *ioport_claim (io_port_type_t type, io_port_direction_t dir, uint8_t *port, const char *description);
bool ioport_claimable (io_port_type_t type, io_port_direction_t dir, uint8_t port);
io_port_cando_t ioports_can_do (void);
uint8_t ioport_find_free (io_port_type_t type, io_port_direction_t dir, pin_cap_t filter, const char *description);
bool ioports_enumerate (io_port_type_t type, io_port_direction_t dir, pin_cap_t filter, ioports_enumerate_callback_ptr callback, void *data);
bool ioport_set_description (io_port_type_t type, io_port_direction_t dir, uint8_t port, const char *description);
bool ioport_set_function (xbar_t *pin, pin_function_t function, driver_caps_t caps);
bool ioport_analog_out (uint8_t port, float value);
bool ioport_digital_out (uint8_t port, uint32_t value);
int32_t ioport_wait_on_input (io_port_type_t type, uint8_t port, wait_mode_t wait_mode, float timeout);
bool ioport_analog_out_config (uint8_t port, pwm_config_t *config);
bool ioport_digital_pwm_config (uint8_t port, pwm_config_t *config);
bool ioport_digital_in_config (uint8_t port, gpio_in_config_t *config);
bool ioport_enable_irq (uint8_t port, pin_irq_mode_t irq_mode, ioport_interrupt_callback_ptr handler);
bool ioport_digital_out_config (uint8_t port, gpio_out_config_t *config);

struct io_ports_data;

typedef struct {
    uint8_t n_ports;
    uint8_t n_start;
    uint8_t idx_last;
#ifdef IOPORTS_KEEP_DEPRECATED
    uint8_t *map;               //!< Deprecated - do not reference in new code!
#endif
} io_ports_detail_t;

typedef struct io_ports_data {
    union {
        io_ports_detail_t cfg[2];
        struct {
            io_ports_detail_t in;
            io_ports_detail_t out;
        };
    };
#ifdef IOPORTS_KEEP_DEPRECATED
    const char *pnum;                                                  //!< Deprecated - do not reference in new code!
    const char *(*get_pnum)(struct io_ports_data *data, uint8_t port); //!< Deprecated - do not reference in new code!
#endif
} io_ports_data_t;

typedef struct {
    io_ports_data_t *ports;
    analog_out_ptr analog_out;                         //!< Handler for setting an analog output.
    ll_wait_on_input_ptr wait_on_input;                //!< Handler for reading a digital or analog input.
    ll_set_pin_description_ptr set_pin_description;    //!< Handler for setting a description of an auxiliary pin.
    ll_get_pin_info_ptr get_pin_info;                  //!< Handler for getting information about an auxiliary pin.
} io_analog_t;

typedef struct {
    io_ports_data_t *ports;
    digital_out_ptr digital_out;                       //!< Handler for setting a digital output.
    ll_ioport_register_interrupt_handler_ptr register_interrupt_handler;
    ll_wait_on_input_ptr wait_on_input;                //!< Handler for reading a digital or analog input.
    ll_set_pin_description_ptr set_pin_description;    //!< Handler for setting a description of an auxiliary pin.
    ll_get_pin_info_ptr get_pin_info;                  //!< Handler for getting information about an auxiliary pin.
} io_digital_t;

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

bool ioports_add_analog (io_analog_t *ports);
bool ioports_add_digital (io_digital_t *ports);
void ioports_add_settings (driver_settings_load_ptr settings_loaded, setting_changed_ptr setting_changed);
bool ioport_remap (io_port_type_t type, io_port_direction_t dir, uint8_t from, uint8_t to);
void ioport_save_input_settings (xbar_t *xbar, gpio_in_config_t *config);
void ioport_save_output_settings (xbar_t *xbar, gpio_out_config_t *config);
void ioport_setting_changed (setting_id_t id);
uint8_t ioports_map_reverse (io_ports_detail_t *type, uint8_t port);
bool ioports_precompute_pwm_values (pwm_config_t *config, ioports_pwm_t *pwm_data, uint32_t clock_hz);
uint_fast16_t ioports_compute_pwm_value (ioports_pwm_t *pwm_data, float value);
#ifdef IOPORTS_KEEP_DEPRECATED
#define iports_get_pnum(type, port) type.get_pnum(&type, port)
#define ioports_map(type, port) ( type.map ? type.map[port] : port )
#endif

//
bool ioports_add (io_ports_data_t *ports, io_port_type_t type, uint8_t n_in, uint8_t n_out); //!< Deprecated - use ioports_add_analog() or ioports_add_digital() instead.
bool ioport_can_claim_explicit (void);                                                       //!< Deprecated - use ioports_can_do() instead.
void ioport_assign_function (aux_ctrl_t *aux_ctrl, pin_function_t *function);                //!< Deprecated - use ioport_set_function() instead.
void ioport_assign_out_function (aux_ctrl_out_t *aux_ctrl, pin_function_t *function);        //!< Deprecated - use ioport_set_function() instead.
//

/*EOF*/
