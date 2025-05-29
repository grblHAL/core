/*
  ioports.c - some wrapper functions for the ioports HAL API

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

/**
 * @file
 *
 * Some wrapper functions for the #io_port_t API.
 * They perform the necessary checks for both availability of ports
 * and advanced functionality simplifying plugin code that uses them.
 */

#include <math.h>

#include "hal.h"
#include "settings.h"

#define MAX_PORTS (Output_AuxMax - Output_Aux0 + 1)

typedef enum {
    Port_AnalogIn = 0,
    Port_AnalogOut,
    Port_DigitalIn,
    Port_DigitalOut
} ioport_type_xxx_t;

typedef struct {
    ioport_type_xxx_t type;
    io_ports_detail_t *ports;
    const char *pnum;
    char port_names[8 * 6 + (MAX_PORTS - 8) * 7];
    ioport_bus_t enabled;
    ioport_bus_t claimed;
    uint8_t last_claimed;
    int16_t count;
    int16_t free;
    int16_t n_ports;
    int16_t n_max;
    pin_function_t min_fn;
    pin_function_t max_fn;
    uint8_t map[MAX_PORTS];
    ioport_bus_t bus;
} io_ports_private_t;

typedef struct {
    digital_out_ptr digital_out;                       //!< Optional handler for setting a digital output.
    analog_out_ptr analog_out;                         //!< Optional handler for setting an analog output.
    ll_wait_on_input_ptr wait_on_input;                //!< Optional handler for reading a digital or analog input.
    ll_set_pin_description_ptr set_pin_description;    //!< Optional handler for setting a description of an auxiliary pin.
    ll_get_pin_info_ptr get_pin_info;                  //!< Optional handler for getting information about an auxiliary pin.
    ll_claim_port_ptr claim;                           //!< Optional handler for claiming an auxiliary pin for exclusive use.
    ll_ioport_register_interrupt_handler_ptr register_interrupt_handler;
} ll_io_port_t;

typedef struct io_ports_list_t {
    io_port_type_t type;
    ll_io_port_t hal;
    io_ports_data_t *ports_id;
    struct io_ports_list_t *next;
} io_ports_list_t;

static io_ports_list_t *ports = NULL;
static driver_settings_load_ptr on_settings_loaded = NULL;
static setting_changed_ptr on_setting_changed = NULL;
static on_settings_changed_ptr on_settings_changed;
static io_ports_private_t ports_cfg[] = {
    {
         .type = Port_AnalogIn, .count = -1, .free = -1, .min_fn = Input_Analog_Aux0, .max_fn = Input_Analog_AuxMax,
         .n_max = (Input_Analog_AuxMax - Input_Analog_Aux0 + 1), .last_claimed = (Input_Analog_AuxMax - Input_Analog_Aux0)
    },
    {
         .type = Port_AnalogOut, .count = -1, .free = -1, .min_fn = Output_Analog_Aux0, .max_fn = Output_Analog_AuxMax,
         .n_max = (Output_Analog_AuxMax - Output_Analog_Aux0 + 1), .last_claimed = (Output_Analog_AuxMax - Output_Analog_Aux0)
    },
    {
        .type = Port_DigitalIn, .count = -1, .free = -1, .min_fn = Input_Aux0, .max_fn = Input_AuxMax,
        .n_max = (Input_AuxMax - Input_Aux0 + 1), .last_claimed = (Input_AuxMax - Input_Aux0)
    },
    {
        .type = Port_DigitalOut, .count = -1, .free = -1, .min_fn = Output_Aux0, .max_fn = Output_AuxMax,
        .n_max = (Output_AuxMax - Output_Aux0 + 1), .last_claimed = (Output_AuxMax - Output_Aux0)
    }
};

PROGMEM static const char *apnum = "E0\0E1\0E2\0E3\0E4\0E5\0E6\0E7\0E8\0E9\0E10\0E11\0E12\0E13\0E14\0E15";
PROGMEM static const char *dpnum = "P0\0P1\0P2\0P3\0P4\0P5\0P6\0P7\0P8\0P9\0P10\0P11\0P12\0P13\0P14\0P15\0P16\0P17\0P18\0P19\0P20\0P21\0P22\0P23";

__STATIC_FORCEINLINE io_ports_private_t *get_port_data (io_port_type_t type, io_port_direction_t dir)
{
    return &ports_cfg[(type << 1) | dir];
}

static uint8_t map_reverse (io_ports_private_t *p_data, uint8_t port)
{
    uint_fast8_t idx = p_data->n_max;

    do {
        if(p_data->map[--idx] == port) {
            port = idx;
            break;
        }
    } while(idx);

    return port;
}

__STATIC_FORCEINLINE uint8_t is_aux (io_ports_private_t *p_data, pin_function_t function)
{
    return function >= p_data->min_fn && function <= p_data->max_fn;
}

// TODO: change to always use ioports_map_reverse()? add range check?
__STATIC_FORCEINLINE uint8_t resolve_portnum (io_ports_private_t *p_data, xbar_t *port)
{
    return is_aux(p_data, port->function) ? (port->function - p_data->min_fn) : map_reverse(p_data, port->id);
}

static uint8_t ioports_count (io_port_type_t type, io_port_direction_t dir, io_ports_private_t *p_data)
{
    xbar_t *port;
    uint8_t n_ports = 0, n_remapped = 0;

    // determine how many ports, including claimed ports, that are available. remapped ports may be excluded.
    if(hal.port.get_pin_info) do {
        if((port = hal.port.get_pin_info(type, dir, n_ports))) {
            n_ports++;
            if(p_data && (port->function < p_data->min_fn || port->function > p_data->max_fn))
                n_remapped++;
        }
    } while(port != NULL);

    return n_ports - n_remapped;
}

/*! \brief Get number of digital or analog ports available.
\param type as an \a #io_port_type_t enum value.
\param dir as an \a #io_port_direction_t enum value.
\returns number of ports available excluding remapped ports but including claimed ports if the API implementation supports that.
*/
uint8_t ioports_available (io_port_type_t type, io_port_direction_t dir)
{
    io_ports_private_t *p_data = get_port_data(type, dir);

    if(p_data->count == -1)
        p_data->count = ioports_count(type, dir, get_port_data(type, dir));

    return p_data->count;
}

/*! \brief Get number of unclaimed digital or analog ports available.
\param type as an \a #io_port_type_t enum value.
\param dir as an \a #io_port_direction_t enum value.
\returns number of ports available.
*/
uint8_t ioports_unclaimed (io_port_type_t type, io_port_direction_t dir)
{
    io_ports_private_t *p_data = get_port_data(type, dir);

    if(p_data->free == -1) {

        xbar_t *port;
        uint8_t idx = 0;

        p_data->free = 0;

        if(hal.port.get_pin_info) do {
            if((port = hal.port.get_pin_info(type, dir, idx++)) && !port->mode.claimed)
                p_data->free++;
        } while(port);
    }

    return p_data->free;
}

static struct ff_data {
    uint8_t port;
    const char *description;
} ff_data;

static bool match_port (xbar_t *properties, uint8_t port, void *data)
{
    if(((struct ff_data *)data)->description && (!properties->description || strcmp(properties->description, ((struct ff_data *)data)->description)))
        return false;

    ((struct ff_data *)data)->port = port;

    return true;
}

/*! \brief find first free or claimed digital or analog port.
\param type as an \a #io_port_type_t enum value.
\param dir as an \a #io_port_direction_t enum value.
\param description pointer to a \a char constant for the pin description of a previousely claimed port or \a NULL if searching for the first free port.
\returns the port number if successful, 0xFF (255) if not.
*/
uint8_t ioport_find_free (io_port_type_t type, io_port_direction_t dir, pin_cap_t filter, const char *description)
{
    ff_data.port = IOPORT_UNASSIGNED;
    ff_data.description = (description && *description) ? description : NULL;

    // TODO: pass modified filter with .claimable off when looking for description match?
    if(ff_data.description && !ioports_enumerate(type, dir, (pin_cap_t){}, match_port, (void *)&ff_data)) {
        ff_data.description = NULL;
        ioports_enumerate(type, dir, filter, match_port, (void *)&ff_data);
    }

    return ff_data.port;
}

/*! \brief Return information about a digital or analog port.
\param type as an \a #io_port_type_t enum value.
\param dir as an \a #io_port_direction_t enum value.
\param port the port aux number.
\returns pointer to \a xbar_t struct if successful, \a NULL if not.
*/
static xbar_t *get_info (io_port_type_t type, io_port_direction_t dir, uint8_t port, bool claim)
{
    bool ok = false;
    xbar_t *portinfo = NULL;
    io_ports_private_t *p_data = get_port_data(type, dir);
    uint8_t n_ports = p_data->ports ? p_data->n_ports : 0;

    if(hal.port.get_pin_info && n_ports) do {
        ok = (portinfo = hal.port.get_pin_info(type, dir, --n_ports)) && !(claim && portinfo->mode.claimed) && resolve_portnum(p_data, portinfo) == port;
    } while(n_ports && !ok);

    return ok ? portinfo : NULL;
}

/*! \brief Return information about a digital or analog port.
\param type as an \a #io_port_type_t enum value.
\param dir as an \a #io_port_direction_t enum value.
\param port the port aux number.
\returns pointer to \a xbar_t struct if successful, \a NULL if not.
*/
xbar_t *ioport_get_info (io_port_type_t type, io_port_direction_t dir, uint8_t port)
{
    return get_info(type, dir, port, false);
}

/* code to keep deprecated data updated, to be removed */

static inline uint8_t get_hcount (io_port_type_t type, io_port_direction_t dir)
{
    return type == Port_Digital
            ? (dir == Port_Input ? hal.port.num_digital_in : hal.port.num_digital_out)
            : (dir == Port_Input ? hal.port.num_analog_in : hal.port.num_analog_out);
}

static inline void dec_hcount (io_port_type_t type, io_port_direction_t dir)
{
    if(type == Port_Digital) {
        if(dir == Port_Input)
            hal.port.num_digital_in--;
        else
            hal.port.num_digital_out--;
    } else if(dir == Port_Input)
        hal.port.num_analog_in--;
    else
        hal.port.num_analog_out--;
}

/* end deprecated */

/*! \brief Claim a digital or analog port for exclusive use.
\param type as an \a #io_port_type_t enum value.
\param dir as an \a #io_port_direction_t enum value.
\param port pointer to a \a uint8_t holding the ports aux number, returns the actual port number to use if successful.
\param description pointer to a \a char constant for the pin description.
\returns pointer to a \a #xbar_t structure with details about the claimed port if successful, \a NULL if not.
*/
xbar_t *ioport_claim (io_port_type_t type, io_port_direction_t dir, uint8_t *port, const char *description)
{
    xbar_t *portinfo = NULL;

    if(hal.port.claim) {

        uint8_t hcnt = get_hcount(type, dir);

        if((portinfo = get_info(type, dir, *port, true)) &&
         //             portinfo->cap.claimable && TODO: add?
                         !portinfo->mode.claimed &&
                          (portinfo->mode.claimed = hal.port.claim(type, dir, port, description))) {

            get_port_data(type, dir)->free = -1;

            if(get_hcount(type, dir) == hcnt)
                dec_hcount(type, dir);
        } else {
            portinfo = NULL;
            *port = IOPORT_UNASSIGNED;
        }
    }

    return portinfo;
}

// Deprecated
void ioport_assign_function (aux_ctrl_t *aux_ctrl, pin_function_t *function)
{
    xbar_t *input;

    if((input = hal.port.get_pin_info(Port_Digital, Port_Input, aux_ctrl->aux_port))) {

        *function = aux_ctrl->function;
        ports_cfg[Port_DigitalIn].bus.mask &= ~(1 << input->id);
        ports_cfg[Port_DigitalIn].count = ports_cfg[Port_DigitalIn].free = -1;
        hal.signals_cap.mask |= aux_ctrl->cap.mask;

        if(aux_ctrl->function == Input_Probe || xbar_fn_to_signals_mask(aux_ctrl->function).mask)
            setting_remove_elements(Settings_IoPort_InvertIn, ports_cfg[Port_DigitalIn].bus.mask);
    }
}

// Deprecated
void ioport_assign_out_function (aux_ctrl_out_t *aux_ctrl, pin_function_t *function)
{
    xbar_t *output;

    if((output = hal.port.get_pin_info(Port_Digital, Port_Output, aux_ctrl->aux_port))) {

        *function = aux_ctrl->function;
        ports_cfg[Port_DigitalOut].bus.mask &= ~(1UL << output->id);
        ports_cfg[Port_DigitalOut].count = ports_cfg[Port_DigitalOut].free = -1;

        setting_remove_elements(Settings_IoPort_InvertOut, ports_cfg[Port_DigitalOut].bus.mask);
    }
}

/*! \brief Set pin function.
\param port pointer to a \a xbar_t structure.
\param function a \a #pin_function_t enum value.
\param caps pointer to \a #driver_caps_t capability flags.
*/
bool ioport_set_function (xbar_t *pin, pin_function_t function, driver_caps_t caps)
{
    bool ok = false;
    io_ports_list_t *io_port = ports;
    io_ports_private_t *cfg = get_port_data((io_port_type_t)!pin->mode.analog, (io_port_direction_t)pin->mode.output);

    if(io_port) do {
        if(io_port->ports_id == pin->ports_id && (ok = pin->set_function && pin->set_function(pin, function))) {

            cfg->bus.mask &= ~(1 << (pin->id + io_port->ports_id->cfg[pin->mode.output].n_start));
            cfg->count = cfg->free = -1;

            switch(cfg->type) {

                case Port_DigitalIn:
                    if(caps.control)
                        hal.signals_cap.mask |= caps.control->mask;
                    if(function == Input_Probe || xbar_fn_to_signals_mask(function).mask)
                        setting_remove_elements(Settings_IoPort_InvertIn, cfg->bus.mask);
                    break;

                case Port_DigitalOut:
                    switch(function) {

                        case Output_CoolantMist:
                            hal.coolant_cap.mist = On;
                            break;

                        case Output_CoolantFlood:
                            hal.coolant_cap.flood = On;
                            break;

                        default: break;
                    }
                    setting_remove_elements(Settings_IoPort_InvertOut, cfg->bus.mask);
                    break;

                default: break;
            }
        }
    } while(!ok && (io_port = io_port->next));

    return ok;
}

/*! \brief Get basic ioports capabilities.
\returns a \a #io_port_cando_t union.
*/
io_port_cando_t ioports_can_do (void)
{
    io_port_cando_t can_do = {};

    if(hal.port.get_pin_info) {
        can_do.claim_explicit = !!hal.port.claim;
        can_do.analog_out = !!hal.port.analog_out;
        can_do.digital_out = !!hal.port.digital_out;
        can_do.wait_on_input = !!hal.port.wait_on_input;
    }

    return can_do;
}

// Deprecated
bool ioport_can_claim_explicit (void)
{
    return ioports_can_do().claim_explicit;
}

/*! \brief Enumerate ports.
\param type as an \a #io_port_type_t enum value.
\param dir as an \a #io_port_direction_t enum value.
\param filter a \a #pin_cap_t union with fields set that must match the port capabilities.
\param callback pointer to a \a #ioports_enumerate_callback_ptr function that will be called for each matching port.
If the function returns \a true the enumeration will end.
\param data a pointer to context data passed to the callback function.
*/
bool ioports_enumerate (io_port_type_t type, io_port_direction_t dir, pin_cap_t filter, ioports_enumerate_callback_ptr callback, void *data)
{
    bool ok = false;
    io_ports_private_t *p_data = get_port_data(type, dir);

    if(p_data->ports && p_data->n_ports && ioport_can_claim_explicit()) {

       xbar_t *portinfo;
       uint_fast16_t n_ports;

       if(filter.mask) {

           uint_fast16_t n_ports = p_data->n_ports;

           do {
                if((portinfo = hal.port.get_pin_info(type, dir, --n_ports)) && (portinfo->cap.mask & filter.mask) == filter.mask) {
                    if((ok = callback(portinfo, resolve_portnum(p_data, portinfo), data)))
                        break;
                }
            } while(n_ports);

       } else for(n_ports = 0; n_ports < p_data->n_ports; n_ports++) {
           if((portinfo = hal.port.get_pin_info(type, dir, n_ports))) {
               if((ok = callback(portinfo, resolve_portnum(p_data, portinfo), data)))
                   break;
           }
       }
    }

    return ok;
}

/*! \brief Set pin description.
\param type as an \a #io_port_type_t enum value.
\param dir as an \a #io_port_direction_t enum value.
\param port the port aux number.
\param description pointer to a \a char constant for the pin description.
*/
bool ioport_set_description (io_port_type_t type, io_port_direction_t dir, uint8_t port, const char *description)
{
    if(hal.port.set_pin_description)
        hal.port.set_pin_description(type, dir, port, description);

    return !!hal.port.set_pin_description;
}

bool ioport_analog_out (uint8_t port, float value)
{
    return hal.port.analog_out && hal.port.analog_out(port, value);
}

bool ioport_digital_out (uint8_t port, uint32_t value)
{
    if(hal.port.digital_out)
        hal.port.digital_out(port, value != 0);

    return !!hal.port.digital_out;
}

int32_t ioport_wait_on_input (io_port_type_t type, uint8_t port, wait_mode_t wait_mode, float timeout)
{
    return hal.port.wait_on_input ? hal.port.wait_on_input(type, port, wait_mode, timeout) : -1;
}

bool ioport_analog_out_config (uint8_t port, pwm_config_t *config)
{
    xbar_t *pin;
    bool ok = (pin = hal.port.get_pin_info(Port_Analog, Port_Output, port)) && pin->config;

    return ok && pin->config(pin, config, false);
}

bool ioport_digital_in_config (uint8_t port, gpio_in_config_t *config)
{
    xbar_t *pin;
    bool ok = (pin = hal.port.get_pin_info(Port_Digital, Port_Input, port)) && pin->config;

    return ok && pin->config(pin, config, false);
}

bool ioport_enable_irq (uint8_t port, pin_irq_mode_t irq_mode, ioport_interrupt_callback_ptr handler)
{
    return hal.port.register_interrupt_handler && hal.port.register_interrupt_handler(port, irq_mode, handler);
}

bool ioport_digital_out_config (uint8_t port, gpio_out_config_t *config)
{
    xbar_t *pin;
    bool ok = (pin = hal.port.get_pin_info(Port_Digital, Port_Output, port)) && pin->config && !(pin->mode.pwm || pin->mode.servo_pwm);

    return ok && pin->config(pin, config, false);
}

bool ioport_digital_pwm_config (uint8_t port, pwm_config_t *config)
{
    xbar_t *pin;
    bool ok = (pin = hal.port.get_pin_info(Port_Digital, Port_Output, port)) && pin->config && pin->mode.claimed && pin->cap.pwm;

    return ok && pin->config(pin, config, false);
}

// HAL wrapper/veneers

__STATIC_FORCEINLINE bool is_match (io_ports_list_t *io_port, io_port_type_t type, io_port_direction_t dir, uint8_t port)
{
   return io_port->type == type && port >= io_port->ports_id->cfg[dir].n_start && port <= io_port->ports_id->cfg[dir].idx_last;
}

__STATIC_FORCEINLINE const char *pnum_to_string (uint8_t port, const char *pnum)
{
    return pnum ? (pnum + (port * 3) + (port > 9 ? port - 10 : 0)) : NULL;
}

static xbar_t *io_get_pin_info (io_port_type_t type, io_port_direction_t dir, uint8_t port)
{
    xbar_t *pin = NULL;
    io_ports_list_t *io_port = ports;
    io_ports_private_t *cfg = get_port_data(type, dir);

    port = cfg->map[port];

    do {
        if(is_match(io_port, type, dir, port)) {
            if((pin = io_port->hal.get_pin_info(dir, port - io_port->ports_id->cfg[dir].n_start))) {
                pin->ports_id = io_port->ports_id;
                pin->mode.claimed = cfg->claimed.mask & (1UL << (pin->id + io_port->ports_id->cfg[dir].n_start));
            }
        }
    } while(pin == NULL && (io_port = io_port->next));

    return pin;
}

static void io_set_pin_description (io_port_type_t type, io_port_direction_t dir, uint8_t port, const char *s)
{
    io_ports_list_t *io_port = ports;
    io_ports_private_t *cfg = get_port_data(type, dir);

    port = cfg->map[port];

    do {
        if(is_match(io_port, type, dir, port)) {
            io_port->hal.set_pin_description(dir, port - io_port->ports_id->cfg[dir].n_start, s);
            break;
        }
    } while((io_port = io_port->next));
}

static bool io_claim (io_port_type_t type, io_port_direction_t dir, uint8_t *port, const char *description)
{
    bool ok = false;
    io_ports_list_t *io_port = ports;
    io_ports_private_t *cfg = get_port_data(type, dir);

    if(!(cfg->claimed.mask & (1UL << *port))) do {

        if(is_match(io_port, type, dir, *port)) {

            xbar_t *pin;

            if((ok = (pin = io_port->hal.get_pin_info(dir, *port - io_port->ports_id->cfg[dir].n_start))) && pin->cap.claimable) {

                uint_fast8_t idx = 0;

                cfg->claimed.mask |= (1UL << *port);

                while(cfg->map[idx] != *port)
                    idx++;

                for(; idx < cfg->last_claimed ; idx++) {
                    if((cfg->map[idx] = cfg->map[idx + 1]) != 255)
                        io_set_pin_description(type, dir, idx, pnum_to_string(idx, cfg->pnum));
                }

                io_port->hal.set_pin_description(dir, *port - io_port->ports_id->cfg[dir].n_start, description);

                pin->ports_id = io_port->ports_id;
                cfg->map[cfg->last_claimed] = *port;
                *port = cfg->last_claimed--;

                break;
            }
        }
    } while((io_port = io_port->next));

    return ok;
}

static bool io_analog_out (uint8_t port, float value)
{
    io_ports_list_t *io_port = ports;
    io_ports_private_t *cfg = get_port_data(Port_Analog, Port_Output);

    port = cfg->map[port];

    do {
        if(io_port->hal.analog_out && is_match(io_port, Port_Analog, Port_Output, port))
            return io_port->hal.analog_out(port - io_port->ports_id->cfg[Port_Output].n_start, value);
    } while((io_port = io_port->next));

    return false;
}

static void io_digital_out (uint8_t port, bool on)
{
    io_ports_list_t *io_port = ports;
    io_ports_private_t *cfg = get_port_data(Port_Digital, Port_Output);

    port = cfg->map[port];

    do {
        if(io_port->hal.digital_out && is_match(io_port, Port_Digital, Port_Output, port)) {
            io_port->hal.digital_out(port - io_port->ports_id->cfg[Port_Output].n_start, on);
            break;
        }
    } while((io_port = io_port->next));
}

static int32_t io_wait_on_input (io_port_type_t type, uint8_t port, wait_mode_t wait_mode, float timeout)
{
    int32_t value = -1;

    io_ports_list_t *io_port = ports;
    io_ports_private_t *cfg = get_port_data(type, Port_Input);

    port = cfg->map[port];

    do {
        if(io_port->hal.wait_on_input && is_match(io_port, type, Port_Input, port)) {
            value = io_port->hal.wait_on_input(port - io_port->ports_id->cfg[Port_Input].n_start, wait_mode, timeout);
            break;
        }
    } while((io_port = io_port->next));

    return value;
}

static bool io_register_interrupt_handler (uint8_t port, pin_irq_mode_t irq_mode, ioport_interrupt_callback_ptr interrupt_callback)
{
    uint8_t user_port = port;
    io_ports_list_t *io_port = ports;
    io_ports_private_t *cfg = get_port_data(Port_Digital, Port_Input);

    port = cfg->map[port];

    do {
        if(io_port->hal.register_interrupt_handler && is_match(io_port, Port_Digital, Port_Input, port))
            return io_port->hal.register_interrupt_handler(port - io_port->ports_id->cfg[Port_Input].n_start, user_port, irq_mode, interrupt_callback);
    } while((io_port = io_port->next));

    return false;
}

/**/

static io_ports_list_t *insert_ports (void)
{
    io_ports_list_t *io_ports;

    if((io_ports = calloc(sizeof(io_ports_list_t), 1))) {

        if(ports == NULL)
            ports = io_ports;
        else {
            io_ports_list_t *add = ports;

            while(add->next)
                add = add->next;

            add->next = io_ports;
        }
    }

    return io_ports;
}

static bool claim_hal (void)
{
    io_port_t empty = {};

    if(!memcmp(&hal.port, &empty, sizeof(io_port_t))) {
        hal.port.get_pin_info = io_get_pin_info;
        hal.port.set_pin_description = io_set_pin_description;
        hal.port.claim = io_claim;
        hal.port.analog_out = io_analog_out;
        hal.port.digital_out = io_digital_out;
        hal.port.wait_on_input = io_wait_on_input;
        hal.port.register_interrupt_handler = io_register_interrupt_handler;
    }

    return hal.port.get_pin_info == io_get_pin_info;
}

#ifdef IOPORTS_KEEP_DEPRECATED

ISR_CODE uint8_t ISR_FUNC(ioports_map_reverse)(io_ports_detail_t *type, uint8_t port)
{
    if(type->map) {
        uint_fast8_t idx = type->n_ports;
        do {
            if(type->map[--idx] == port) {
                port = idx;
                break;
            }
        } while(idx);
    }

    return port;
}

static const char *get_pnum (io_ports_data_t *ports, uint8_t port)
{
    return ports->pnum ? (ports->pnum + (port * 3) + (port > 9 ? port - 10 : 0)) : NULL;
}

#endif

static uint8_t add_ports (io_ports_detail_t *ports, uint8_t *map, io_port_type_t type, io_port_direction_t dir, uint8_t n_ports)
{
    io_ports_private_t *p_data = get_port_data(type, dir);

    if(n_ports) {

        ports->n_start = p_data->n_ports;

        if(ports->n_start + n_ports > p_data->n_max)
            n_ports = p_data->n_max - ports->n_start;

        ports->idx_last = ports->n_start + n_ports - 1;
#ifdef IOPORTS_KEEP_DEPRECATED
        ports->map = map;
#endif
        if(p_data->ports == NULL)
            p_data->ports = ports;
        p_data->count = -1;
    } else
        ports->n_start = 255;

    p_data->n_ports += n_ports;
    ports->n_ports = n_ports;

    return n_ports;
}

static bool _ioports_add (io_ports_data_t *ports, io_port_type_t type, uint8_t n_in, uint8_t n_out, set_pin_description_ptr set_description)
{
    uint_fast8_t n_ports;
    io_ports_private_t *cfg_in = get_port_data(type, Port_Input),
                       *cfg_out = get_port_data(type, Port_Output);

#ifdef IOPORTS_KEEP_DEPRECATED
    ports->get_pnum = get_pnum;
#endif

    if(type == Port_Digital) {

        hal.port.num_digital_in += add_ports(&ports->in, cfg_in->map, type, Port_Input, n_in);
        hal.port.num_digital_out += add_ports(&ports->out, cfg_out->map, type, Port_Output, n_out);

    } else {

        hal.port.num_analog_in += add_ports(&ports->in, cfg_in->map, type, Port_Input, n_in);
        hal.port.num_analog_out += add_ports(&ports->out, cfg_out->map, type, Port_Output, n_out);
    }

    if((n_ports = max(cfg_in->n_ports, cfg_out->n_ports)) > 0) {

        uint_fast8_t i = MAX_PORTS, idx_in = 0, idx_out = 0;

        if(cfg_in->pnum) {

            while(cfg_in->map[idx_in] != 255)
                idx_in++;

            while(cfg_out->map[idx_out] != 255)
                idx_out++;

        } else do {
            i--;
            cfg_in->map[i] = cfg_out->map[i] = 255;
        } while(i);

        cfg_in->pnum = cfg_out->pnum /* = ports->pnum*/ = type == Port_Digital ? dpnum : apnum;

        for(i = 0; i <= n_ports; i++) {

            if(ports->in.n_ports && i >= ports->in.n_start && i <= ports->in.idx_last) {

                cfg_in->map[idx_in] = i;
                if(set_description)
                    set_description(type, Port_Input, i - ports->in.n_start, pnum_to_string(idx_in, cfg_in->pnum));

                idx_in++;

                if(i < MAX_PORTS) {
                    cfg_in->bus.mask |= (1 << i);
                    strcat(cfg_in->port_names, i == 0 ? "Aux " : ",Aux ");
                    strcat(cfg_in->port_names, uitoa(i));
                }
            }

            if(ports->out.n_ports && i >= ports->out.n_start && i <= ports->out.idx_last) {

                cfg_out->map[idx_out] = i;
                if(set_description)
                    set_description(type, Port_Output, i - ports->out.n_start, pnum_to_string(idx_out, cfg_out->pnum));

                if(i < MAX_PORTS) {
                    cfg_out->bus.mask |= (1 << i);
                    strcat(cfg_out->port_names, i == 0 ? "Aux " : ",Aux ");
                    strcat(cfg_out->port_names, uitoa(i));
                }

                idx_out++;
            }
        }

        cfg_in->enabled.mask = cfg_in->bus.mask;
        cfg_out->enabled.mask = cfg_out->bus.mask;
    }

    return n_ports > 0;
}

bool ioports_add (io_ports_data_t *ports, io_port_type_t type, uint8_t n_in, uint8_t n_out)
{
    return hal.port.get_pin_info != io_get_pin_info && _ioports_add(ports, type, n_in, n_out, hal.port.set_pin_description);
}

static ll_set_pin_description_ptr set_descr_veneer;

static void set_description (io_port_type_t type, io_port_direction_t dir, uint8_t port, const char *s)
{
    set_descr_veneer(dir, port, s);
}

bool ioports_add_analog (io_analog_t *analog)
{
    if(analog->ports->in.n_ports + analog->ports->out.n_ports == 0)
        return false;

    bool ok;

    set_descr_veneer = analog->set_pin_description;

    if((ok = claim_hal() && _ioports_add(analog->ports, Port_Analog, analog->ports->in.n_ports, analog->ports->out.n_ports, set_description))) {

        io_ports_list_t *ports;

        if((ports = insert_ports())) {

            ports->type = Port_Analog;
            ports->ports_id = analog->ports;
            ports->hal.set_pin_description = analog->set_pin_description;
            ports->hal.get_pin_info = analog->get_pin_info;
            if(analog->ports->out.n_ports)
                ports->hal.analog_out = analog->analog_out;
            if(analog->ports->in.n_ports)
                ports->hal.wait_on_input = analog->wait_on_input;
        }
    }

    return ok;
}

bool ioports_add_digital (io_digital_t *digital)
{
    if(digital->ports->in.n_ports + digital->ports->out.n_ports == 0)
        return false;

    bool ok;

    set_descr_veneer = digital->set_pin_description;

    if((ok = claim_hal() && _ioports_add(digital->ports, Port_Digital, digital->ports->in.n_ports, digital->ports->out.n_ports, set_description))) {

        io_ports_list_t *io_ports;

        if((io_ports = insert_ports())) {

            io_ports->type = Port_Digital;
            io_ports->ports_id = digital->ports;
            io_ports->hal.set_pin_description = digital->set_pin_description;
            io_ports->hal.get_pin_info = digital->get_pin_info;
            if(digital->ports->out.n_ports)
                io_ports->hal.digital_out = digital->digital_out;
            if(digital->ports->in.n_ports) {
                io_ports->hal.wait_on_input = digital->wait_on_input;
                io_ports->hal.register_interrupt_handler = digital->register_interrupt_handler;
            }
        }

        ioports_add_settings(NULL, NULL);
    }

    return ok;
}

/*! \brief calculate inverted pwm value if configured
\param pwm_data pointer t a \a spindle_pwm_t structure.
\param pwm_value non inverted PWM value.
\returns the inverted PWM value to use.
*/
__STATIC_FORCEINLINE uint_fast16_t invert_pwm (ioports_pwm_t *pwm_data, uint_fast16_t pwm_value)
{
    return pwm_data->invert_pwm ? pwm_data->period - pwm_value - 1 : pwm_value;
}

/*! \brief Precompute PWM values for faster conversion.
\param config pointer to a \ref pwm_config_t structure.
\param pwm_data pointer to a \a ioports_pwm_t structure, to hold the precomputed values.
\param clock_hz timer clock frequency used for PWM generation.
\returns \a true if successful, \a false if no PWM range possible - driver should then revert to simple on/off control.
*/
bool ioports_precompute_pwm_values (pwm_config_t *config, ioports_pwm_t *pwm_data, uint32_t clock_hz)
{
    pwm_data->f_clock = clock_hz;

    if(config->max > config->min) {
        pwm_data->min = config->min;
        pwm_data->period = (uint_fast16_t)((float)clock_hz / config->freq_hz);
        pwm_data->min_value = (uint_fast16_t)(pwm_data->period * config->min_value / 100.0f);
        pwm_data->max_value = (uint_fast16_t)(pwm_data->period * config->max_value / 100.0f); // + pwm_data->offset;
        pwm_data->pwm_gradient = (float)(pwm_data->max_value - pwm_data->min_value) / (config->max - config->min);
        if(!(pwm_data->always_on = config->off_value != 0.0f))
            pwm_data->off_value = pwm_data->invert_pwm ? pwm_data->period : 0;
        else if(!config->servo_mode && config->off_value > 0.0f)
            pwm_data->off_value = invert_pwm(pwm_data, (uint_fast16_t)(pwm_data->period * config->off_value / 100.0f));
        else
            pwm_data->off_value = pwm_data->min_value;
    }

    return config->max > config->min;
}

/*! \brief Analog value to PWM conversion.
\param pwm_data pointer to a \a ioports_pwm_t structure.
\param value analog value to be converted.
\returns the PWM value to use.

__NOTE:__ \a ioports_precompute_pwm_values() must be called to precompute values before this function is called.
Typically this is done by the ioports initialization code.
*/
uint_fast16_t ioports_compute_pwm_value (ioports_pwm_t *pwm_data, float value)
{
    uint_fast16_t pwm_value;

    if(value > pwm_data->min) {

        pwm_value = (uint_fast16_t)floorf((value - pwm_data->min) * pwm_data->pwm_gradient) + pwm_data->min_value;

        if(pwm_value >= pwm_data->max_value)
            pwm_value = pwm_data->max_value;
        else if(pwm_value < pwm_data->min_value)
            pwm_value = pwm_data->min_value;

        pwm_value = invert_pwm(pwm_data, pwm_value);
    } else
        pwm_value = value == 0.0f ? pwm_data->off_value : invert_pwm(pwm_data, pwm_data->min_value);

    return pwm_value;
}

void ioport_save_input_settings (xbar_t *xbar, gpio_in_config_t *config)
{
    io_ports_list_t *io_port = ports;
    io_ports_private_t *cfg = get_port_data(!xbar->mode.analog, xbar->mode.output);

    if(io_port) do {
        if(io_port->ports_id == xbar->ports_id) {

            uint32_t bit = 1UL << (xbar->id + io_port->ports_id->cfg[xbar->mode.output].n_start);

            if(cfg->bus.mask & bit) {
               if(config->inverted)
                   settings.ioport.invert_in.mask |= bit;
               else
                   settings.ioport.invert_in.mask &= ~bit;
            }
            if(cfg->enabled.mask & bit) {
               if(config->pull_mode != PullMode_Up)
                   settings.ioport.pullup_disable_in.mask |= bit;
               else
                   settings.ioport.pullup_disable_in.mask &= ~bit;
            }
            break;
        }
    } while((io_port = io_port->next));

    // TODO: remove this block?
    if(xbar->function == Input_Probe)
        settings.probe.invert_probe_pin = config->inverted;
    else if(xbar->function < Input_Probe) {
        control_signals_t ctrl;
        if((ctrl = xbar_fn_to_signals_mask(xbar->function)).mask) {
            if(config->inverted)
                settings.control_invert.mask |= ctrl.mask;
            else
                settings.control_invert.mask &= ~ctrl.mask;
        }
    }

    settings_write_global();
}

void ioport_save_output_settings (xbar_t *xbar, gpio_out_config_t *config)
{
    io_ports_list_t *io_port = ports;
    io_ports_private_t *cfg = get_port_data(!xbar->mode.analog, xbar->mode.output);

    if(io_port) do {
        if(io_port->ports_id == xbar->ports_id) {

            uint32_t bit = 1UL << (xbar->id + io_port->ports_id->cfg[xbar->mode.output].n_start);

            if(cfg->bus.mask & bit) {
               if(config->inverted)
                   settings.ioport.invert_out.mask |= bit;
               else
                   settings.ioport.invert_out.mask &= ~bit;
            }
            if(cfg->enabled.mask & bit) {
               if(config->open_drain)
                   settings.ioport.od_enable_out.mask |= bit;
               else
                   settings.ioport.od_enable_out.mask &= ~bit;
            }
            break;
        }
    } while((io_port = io_port->next));

    settings_write_global();
}

static bool is_setting_available (const setting_detail_t *setting, uint_fast16_t offset)
{
    bool available = false;

    switch(setting->id) {

        case Settings_IoPort_InvertIn:
        case Settings_IoPort_Pullup_Disable:
            available = ports_cfg[Port_DigitalIn].ports && ports_cfg[Port_DigitalIn].bus.mask;
            break;

        case Settings_IoPort_InvertOut:
        case Settings_IoPort_OD_Enable:
            available = ports_cfg[Port_DigitalOut].ports && ports_cfg[Port_DigitalOut].bus.mask;
            break;

        default:
            break;
    }

    return available;
}

static status_code_t aux_set_value (setting_id_t id, uint_fast16_t value)
{
    xbar_t *xbar;
    uint8_t port = 0;
    ioport_bus_t change, changed;

    switch(id) {

        case Settings_IoPort_InvertIn:

            change.mask = value & ports_cfg[Port_DigitalIn].bus.mask;

            if((changed.mask = settings.ioport.invert_in.mask ^ change.mask)) {

                gpio_in_config_t config = {0};

                do {
                    if((changed.mask & 0x01) && (xbar = hal.port.get_pin_info(Port_Digital, Port_Input, map_reverse(&ports_cfg[Port_DigitalIn], port)))) {
                        if(xbar->config && is_aux(&ports_cfg[Port_DigitalIn], xbar->function)) {
                            config.pull_mode = (pull_mode_t)xbar->mode.pull_mode;
                            config.inverted = !!(change.mask & (1 << port));
                            xbar->config(xbar, &config, false);
                        }
                    }
                    port++;
                } while(changed.mask >>= 1);
            }

            settings.ioport.invert_in.mask = change.mask;

            if(on_setting_changed)
                on_setting_changed(id);
            break;

        case Settings_IoPort_Pullup_Disable:

            change.mask = value & ports_cfg[Port_DigitalIn].enabled.mask;

            if((changed.mask = settings.ioport.pullup_disable_in.mask ^ change.mask)) {

                gpio_in_config_t config = {0};

                do {
                    if((changed.mask & 0x01) && (xbar = hal.port.get_pin_info(Port_Digital, Port_Input, map_reverse(&ports_cfg[Port_DigitalIn], port)))) {
                        if(xbar->config) {
                            config.pull_mode = change.mask & (1 << port) ? PullMode_Down : PullMode_Up;
                            config.inverted = xbar->mode.inverted && is_aux(&ports_cfg[Port_DigitalIn], xbar->function);
                            config.debounce = xbar->mode.inverted;
                            xbar->config(xbar, &config, false);
                        }
                    }
                    port++;
                } while(changed.mask >>= 1);
            }

            settings.ioport.pullup_disable_in.mask = change.mask;

            if(on_setting_changed)
                on_setting_changed(id);
            break;

        case Settings_IoPort_InvertOut:

            change.mask = value & ports_cfg[Port_DigitalOut].bus.mask;

            if((changed.mask = settings.ioport.invert_out.mask ^ change.mask)) {

                gpio_out_config_t config = {0};

                do {
                    if((changed.mask & 0x01) && (xbar = hal.port.get_pin_info(Port_Digital, Port_Output, map_reverse(&ports_cfg[Port_DigitalOut], port)))) {
                        if(xbar->config && !(xbar->mode.pwm || xbar->mode.servo_pwm) && is_aux(&ports_cfg[Port_DigitalOut], xbar->function)) {
                            config.inverted = !!(change.mask & (1 << port));
                            config.open_drain = xbar->mode.open_drain;
                            xbar->config(xbar, &config, false);
                        }
                    }
                    port++;
                } while(changed.mask >>= 1);

                settings.ioport.invert_out.mask = change.mask;

                if(on_setting_changed)
                    on_setting_changed(id);
            }
            break;

        case Settings_IoPort_OD_Enable:

            change.mask = value & ports_cfg[Port_DigitalOut].enabled.mask;

            if((changed.mask = settings.ioport.od_enable_out.mask ^ change.mask)) {

                gpio_out_config_t config = {0};

                do {
                    if((changed.mask & 0x01) && (xbar = hal.port.get_pin_info(Port_Digital, Port_Output, map_reverse(&ports_cfg[Port_DigitalOut], port)))) {
                        if(xbar->config && !(xbar->mode.pwm || xbar->mode.servo_pwm)) {
                            config.inverted = xbar->mode.inverted && is_aux(&ports_cfg[Port_DigitalOut], xbar->function);
                            config.open_drain = !!(change.mask & (1 << port));
                            xbar->config(xbar, &config, false);
                        }
                    }
                    port++;
                } while(changed.mask >>= 1);

                settings.ioport.od_enable_out.mask = change.mask;

                if(on_setting_changed)
                    on_setting_changed(id);
            }
            break;

        default:
            break;
    }

    return Status_OK;
}

static uint32_t aux_get_value (setting_id_t id)
{
    uint32_t value = 0;

    switch(id) {

        case Settings_IoPort_InvertIn:
            value = settings.ioport.invert_in.mask & ports_cfg[Port_DigitalIn].enabled.mask;
            break;

        case Settings_IoPort_Pullup_Disable:
            value = settings.ioport.pullup_disable_in.mask & ports_cfg[Port_DigitalIn].enabled.mask;
            break;

        case Settings_IoPort_InvertOut:
            value = settings.ioport.invert_out.mask & ports_cfg[Port_DigitalOut].enabled.mask;
            break;

        case Settings_IoPort_OD_Enable:
            value = settings.ioport.od_enable_out.mask & ports_cfg[Port_DigitalOut].enabled.mask;
            break;

        default:
            break;
    }

    return value;
}

static const setting_group_detail_t ioport_groups[] = {
    { Group_Root, Group_AuxPorts, "Aux ports"}
};

static const setting_detail_t ioport_settings[] = {
    { Settings_IoPort_InvertIn, Group_AuxPorts, "Invert I/O Port inputs", NULL, Format_Bitfield, ports_cfg[Port_DigitalIn].port_names, NULL, NULL, Setting_NonCoreFn, aux_set_value, aux_get_value, is_setting_available },
#ifdef AUX_SETTINGS_PULLUP
    { Settings_IoPort_Pullup_Disable, Group_AuxPorts, "I/O Port inputs pullup disable", NULL, Format_Bitfield, digital.in.port_names, NULL, NULL, Setting_NonCoreFn, aux_set_value, aux_get_value, is_setting_available },
#endif
    { Settings_IoPort_InvertOut, Group_AuxPorts, "Invert I/O Port outputs", NULL, Format_Bitfield, ports_cfg[Port_DigitalOut].port_names, NULL, NULL, Setting_NonCoreFn, aux_set_value, aux_get_value, is_setting_available },
//    { Settings_IoPort_OD_Enable, Group_AuxPorts, "I/O Port outputs as open drain", NULL, Format_Bitfield, digital.out.port_names, NULL, NULL, Setting_NonCoreFn, aux_set_value, aux_get_value, is_setting_available }
};

#ifndef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t ioport_settings_descr[] = {
    { Settings_IoPort_InvertIn, "Invert IOPort inputs." },
//    { Settings_IoPort_Pullup_Disable, "Disable IOPort input pullups." },
    { Settings_IoPort_InvertOut, "Invert IOPort output." },
//    { Settings_IoPort_OD_Enable, "Set IOPort outputs as open drain (OD)." }
};

#endif

static bool config_probe_pins (pin_function_t function, gpio_in_config_t *config)
{
    bool ok = true;

    switch(function) {

        case Input_Probe:
            config->debounce  = Off;
            config->inverted  = settings.probe.invert_probe_pin;
            config->pull_mode = settings.probe.disable_probe_pullup ? PullMode_None : PullMode_Up;
            break;

        case Input_Probe2:
            config->debounce  = Off;
            config->inverted  = settings.probe.invert_probe2_input;
            config->pull_mode = settings.probe.disable_probe_pullup ? PullMode_None : PullMode_Up;
            break;

        case Input_Toolsetter:
            config->debounce  = Off;
            config->inverted  = settings.probe.invert_toolsetter_input;
            config->pull_mode = settings.probe.disable_toolsetter_pullup ? PullMode_None : PullMode_Up;
            break;

        default:
            ok = false;
            break;
    }

    return ok;
}

void ioport_setting_changed (setting_id_t id)
{
    if(on_setting_changed)
        on_setting_changed(id);

    else if(ports_cfg[Port_DigitalIn].ports && ports_cfg[Port_DigitalIn].n_ports) switch(id) {

        case Setting_InvertProbePin:
        case Setting_ProbePullUpDisable:
            {
                xbar_t *xbar;
                gpio_in_config_t in_config = {0};
                uint8_t port = ports_cfg[Port_DigitalIn].n_ports;

                do {
                    if((xbar = hal.port.get_pin_info(Port_Digital, Port_Input, map_reverse(&ports_cfg[Port_DigitalIn], --port)))) {
                        if(xbar->config && config_probe_pins(xbar->function, &in_config)) {
                            if(in_config.inverted)
                                settings.ioport.invert_in.mask |= (1 << port);
                            else
                                settings.ioport.invert_in.mask &= ~(1 << port);
                            xbar->config(xbar, &in_config, false);
                        }
                    }
                } while(port);
            }
            break;

        case Setting_ControlInvertMask:
        case Setting_ControlPullUpDisableMask:
            {
                xbar_t *xbar;
                gpio_in_config_t in_config = {0};
                control_signals_t ctrl;
                uint8_t port = ports_cfg[Port_DigitalIn].n_ports;

                do {
                    if((xbar = hal.port.get_pin_info(Port_Digital, Port_Input, map_reverse(&ports_cfg[Port_DigitalIn], --port)))) {
                        if(xbar->config && xbar->function < Input_Probe) {

                            in_config.debounce = xbar->mode.debounce;
                            in_config.inverted = !!(settings.ioport.invert_in.mask & (1 << port));
                            in_config.pull_mode = (pull_mode_t)xbar->mode.pull_mode;

                            if((ctrl = xbar_fn_to_signals_mask(xbar->function)).mask) {
                                in_config.inverted = !!(settings.control_invert.mask & ctrl.mask);
                                in_config.pull_mode = (settings.control_disable_pullup.mask & ctrl.mask) ? PullMode_None : PullMode_Up;
                            }

                            if(in_config.inverted)
                                settings.ioport.invert_in.mask |= (1 << port);
                            else
                                settings.ioport.invert_in.mask &= ~(1 << port);

                            xbar->config(xbar, &in_config, false);
                        }
                    }
                } while(port);
            }
            break;

        default:
            break;
    }
}

static void ioports_configure (settings_t *settings)
{
    uint8_t port;
    xbar_t *xbar;
    gpio_in_config_t in_config = {0};
    gpio_out_config_t out_config = {0};
    io_ports_private_t *cfg;

    settings->ioport.invert_in.mask &= ports_cfg[Port_DigitalIn].bus.mask;
    settings->ioport.pullup_disable_in.mask &= ports_cfg[Port_DigitalIn].bus.mask;
    settings->ioport.invert_out.mask &= ports_cfg[Port_DigitalOut].bus.mask;
    settings->ioport.od_enable_out.mask &= ports_cfg[Port_DigitalOut].bus.mask;

    cfg = get_port_data(Port_Digital, Port_Input);

    if(cfg->ports && (port = cfg->n_ports)) do {
        if((xbar = hal.port.get_pin_info(Port_Digital, Port_Input, map_reverse(cfg, --port))) && xbar->config) {

            in_config.debounce = xbar->mode.debounce;

            if(is_aux(cfg, xbar->function)) {
                in_config.inverted = !!(settings->ioport.invert_in.mask & (1 << port));
#ifdef AUX_SETTINGS_PULLUP
                in_config.pull_mode = (settings->ioport.pullup_disable_in.mask & (1 << port)) ? PullMode_None : PullMode_Up;
#else
                in_config.pull_mode = (pull_mode_t)xbar->mode.pull_mode;
#endif
            } else { // For probe and control signals higher level config takes priority
                in_config.inverted = Off;
                if(!config_probe_pins(xbar->function, &in_config) && xbar->function < Input_Probe) {
                    control_signals_t ctrl;
                    if((ctrl = xbar_fn_to_signals_mask(xbar->function)).mask) {
#ifdef RP2040 // RP2xxx MCUs use hardware signal inversion
                        in_config.inverted = !!(settings->control_invert.mask & ctrl.mask);
#endif
                        in_config.pull_mode = (settings->control_disable_pullup.mask & ctrl.mask) ? PullMode_None : PullMode_Up;
                    }
                }
            }
            xbar->config(xbar, &in_config, false);
        }
    } while(port);

    cfg = get_port_data(Port_Digital, Port_Output);

    if(cfg->ports && (port = cfg->n_ports)) do {
        if((xbar = hal.port.get_pin_info(Port_Digital, Port_Output, map_reverse(cfg, --port)))) {
            if(xbar->config && (cfg->bus.mask & (1 << port)) && !(xbar->mode.pwm || xbar->mode.servo_pwm)) {
                out_config.inverted = (settings->ioport.invert_out.mask & (1 << port)) && is_aux(cfg, xbar->function);
                out_config.open_drain = !!(settings->ioport.od_enable_out.mask & (1 << port));
                xbar->config(xbar, &out_config, false);
            } else // TODO: same for inputs?
                setting_remove_elements(Settings_IoPort_InvertOut, cfg->bus.mask & ~(1 << port));
        }
    } while(port);

    if(on_settings_loaded)
        on_settings_loaded();
}

static void onSettingsChanged (settings_t *settings, settings_changed_flags_t changed)
{
    if(on_settings_changed)
        on_settings_changed(settings, changed);

    if(sys.ioinit_pending)
        ioports_configure(settings);
}

void ioports_add_settings (driver_settings_load_ptr settings_loaded, setting_changed_ptr setting_changed)
{
    static bool ok = false;
    static setting_details_t setting_details = {
        .is_core = true,
        .groups = ioport_groups,
        .n_groups = sizeof(ioport_groups) / sizeof(setting_group_detail_t),
        .settings = ioport_settings,
        .n_settings = sizeof(ioport_settings) / sizeof(setting_detail_t),
    #ifndef NO_SETTINGS_DESCRIPTIONS
        .descriptions = ioport_settings_descr,
        .n_descriptions = sizeof(ioport_settings_descr) / sizeof(setting_descr_t),
    #endif
        .save = settings_write_global
    };

    if(settings_loaded && on_settings_loaded == NULL)
        on_settings_loaded = settings_loaded;

    if(setting_changed && on_setting_changed == NULL)
        on_setting_changed = setting_changed;

    if(!ok) {

        ok = true;

        on_settings_changed = grbl.on_settings_changed;
        grbl.on_settings_changed = onSettingsChanged;

        settings_register(&setting_details);
    }
}
