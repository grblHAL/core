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

typedef struct {
    io_ports_detail_t *ports;
    char port_names[50];
    ioport_bus_t enabled;
} io_ports_private_t;

typedef struct {
    io_ports_private_t in;
    io_ports_private_t out;
    ioport_bus_t inx;
    ioport_bus_t outx;
} io_ports_cfg_t;

static driver_settings_load_ptr on_settings_loaded = NULL;
static setting_changed_ptr on_setting_changed = NULL;
static io_ports_cfg_t analog, digital;
static int16_t digital_in = -1, digital_out = -1, analog_in = -1, analog_out = -1;

static uint8_t ioports_count (io_port_type_t type, io_port_direction_t dir)
{
    xbar_t *port;
    uint8_t n_ports = 0;

    // determine how many ports, including claimed ports, that are available
    do {
        if((port = hal.port.get_pin_info(type, dir, n_ports)))
            n_ports++;
    } while(port != NULL);

    return n_ports;
}

/*! \brief Get number of digital or analog ports available.
\param type as an \a #io_port_type_t enum value.
\param dir as an \a #io_port_direction_t enum value.
\returns number of ports available including claimed ports if the API implementation supports that.
*/
uint8_t ioports_available (io_port_type_t type, io_port_direction_t dir)
{
    uint8_t ports = 0;

    if(hal.port.get_pin_info) {

        if(type == Port_Digital) {
            if(dir == Port_Input)
                ports = digital_in == -1 ? (digital_in = ioports_count(type, dir)) : (uint8_t)digital_in;
            else
                ports = digital_out == -1 ? (digital_out = ioports_count(type, dir)) : (uint8_t)digital_out;
        } else {
            if(dir == Port_Input)
                ports = analog_in == -1 ? (analog_in = ioports_count(type, dir)) : (uint8_t)analog_in;
            else
                ports = analog_out == -1 ? (analog_out = ioports_count(type, dir)) : (uint8_t)analog_out;
        }
    } else {
        if(type == Port_Digital)
            ports = dir == Port_Input ? hal.port.num_digital_in : hal.port.num_digital_out;
        else
            ports = dir == Port_Input ? hal.port.num_analog_in : hal.port.num_analog_out;
    }

    return ports;
}

/*! \brief find first free or claimed digital or analog port.
\param type as an \a #io_port_type_t enum value.
\param dir as an \a #io_port_direction_t enum value.
\param description pointer to a \a char constant for the pin description of a previousely claimed port or NULL if searching for the first free port.
\returns the port number if successful, 255 if not.
*/
uint8_t ioport_find_free (io_port_type_t type, io_port_direction_t dir, const char *description)
{
    uint8_t port;
    bool found = false;
    xbar_t *pin;

    if(description) {
        port = ioports_available(type, dir);
        do {
            if((pin = hal.port.get_pin_info(type, dir, --port))) {
                if((found = pin->description && !strcmp(pin->description, description)))
                    port = pin->id;
            }
        } while(port && !found);
    }

    if(!found) {
        port = ioports_available(type, dir);
        do {
            if((pin = hal.port.get_pin_info(type, dir, --port))) {
                if((found = !pin->mode.claimed))
                    port = pin->id;
            }
        } while(port && !found);
    }

    return found ? port : 255;
}

/*! \brief Return information about a digital or analog port.
\param type as an \a #io_port_type_t enum value.
\param dir as an \a #io_port_direction_t enum value.
\param port the port aux number.
\returns pointer to \a xbar_t struct if successful, NULL if not.
*/
xbar_t *ioport_get_info (io_port_type_t type, io_port_direction_t dir, uint8_t port)
{
    bool ok = false;
    uint8_t n_ports = ioports_available(type, dir);
    uint8_t base = type == Port_Digital
                    ? (dir == Port_Input ? Input_Aux0 : Output_Aux0)
                    : (dir == Port_Input ? Input_Analog_Aux0 : Output_Analog_Aux0);
    xbar_t *portinfo = NULL;

    if(hal.port.get_pin_info && n_ports) do {
        ok = (portinfo = hal.port.get_pin_info(type, dir, --n_ports)) && (portinfo->function - base) == port;
    } while(n_ports && !ok);

    return ok ? portinfo : NULL;
}

/*! \brief Claim a digital or analog port for exclusive use.
\param type as an \a #io_port_type_t enum value.
\param dir as an \a #io_port_direction_t enum value.
\param port pointer to a \a uint8_t holding the ports aux number, returns the actual port number to use if successful.
\param description pointer to a \a char constant for the pin description.
\returns true if successful, false if not.
*/
bool ioport_claim (io_port_type_t type, io_port_direction_t dir, uint8_t *port, const char *description)
{
    bool ok;

    if(hal.port.claim) {

        xbar_t *portinfo;

        ok = (portinfo = ioport_get_info(type, dir, *port)) && !portinfo->mode.claimed && hal.port.claim(type, dir, port, description);

    } else if((ok = ioports_available(type, dir) > 0)) {

        if(type == Port_Digital)
            *port = dir == Port_Input ? --hal.port.num_digital_in : --hal.port.num_digital_out;
        else
            *port = dir == Port_Input ? --hal.port.num_analog_in : --hal.port.num_analog_out;

        if(hal.port.set_pin_description)
            hal.port.set_pin_description(type, dir, *port, description);
    }

    return ok;
}

/*! \brief Reassign pin function.
\param port pointer to an \a aux_ctrl_t structure with a valid port number.
\param function pointer to a \a #pin_function_t enum value to be updated.
*/
void ioport_assign_function (aux_ctrl_t *aux_ctrl, pin_function_t *function)
{
    xbar_t *input;

    if((input = hal.port.get_pin_info(Port_Digital, Port_Input, aux_ctrl->aux_port))) {

        *function = aux_ctrl->function;
        digital.inx.mask &= ~(1 << input->id);
        hal.signals_cap.mask |= aux_ctrl->cap.mask;

        if(aux_ctrl->function == Input_Probe || xbar_fn_to_signals_mask(aux_ctrl->function).mask)
            setting_remove_elements(Settings_IoPort_InvertIn, digital.inx.mask);
    }
}

void ioport_assign_out_function (aux_ctrl_out_t *aux_ctrl, pin_function_t *function)
{
    xbar_t *output;

    if((output = hal.port.get_pin_info(Port_Digital, Port_Output, aux_ctrl->aux_port))) {

        *function = aux_ctrl->function;
        digital.outx.mask &= ~(1 << output->id);

        setting_remove_elements(Settings_IoPort_InvertOut, digital.outx.mask);
    }
}

/*! \brief Check if ports can be claimed by aux number or not.
\returns true if ports can be claimed by aux number, false if claimed ports are allocated by the API.
*/
bool ioport_can_claim_explicit (void)
{
    return !!hal.port.claim && !!hal.port.get_pin_info;
}

bool ioports_enumerate (io_port_type_t type, io_port_direction_t dir, pin_cap_t filter, ioports_enumerate_callback_ptr callback, void *data)
{
    bool ok = false;
    uint8_t n_ports = ioports_available(type, dir),
            base = type == Port_Digital
                    ? (dir == Port_Input ? Input_Aux0 : Output_Aux0)
                    : (dir == Port_Input ? Input_Analog_Aux0 : Output_Analog_Aux0);
    xbar_t *portinfo;

    if(n_ports && ioport_can_claim_explicit()) do {
        portinfo = hal.port.get_pin_info(type, dir, --n_ports);
        if((portinfo->cap.mask & filter.mask) == filter.mask && (ok = callback(portinfo, portinfo->function - base, data)))
            break;
    } while(n_ports);

    return ok;
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

/* experimental code follows */

static char *get_pnum (io_ports_data_t *ports, uint8_t port)
{
    return ports->pnum ? (ports->pnum + (port * 3) + (port > 9 ? port - 10 : 0)) : NULL;
}

bool ioports_add (io_ports_data_t *ports, io_port_type_t type, uint8_t n_in, uint8_t n_out)
{
    uint_fast8_t n_ports;
    io_ports_cfg_t *cfg;

    ports->get_pnum = get_pnum;

    if(type == Port_Digital) {

        cfg = &digital;
        digital_in = digital_out = -1;

        if(n_in) {
            ports->in.n_start = hal.port.num_digital_in;
            hal.port.num_digital_in += (ports->in.n_ports = n_in);
            ports->in.map = malloc(ports->in.n_ports * sizeof(ports->in.n_ports));
            digital.in.ports = &ports->in;
        }

        if(n_out) {
            ports->out.n_start = hal.port.num_digital_out;
            hal.port.num_digital_out += (ports->out.n_ports = n_out);
            ports->out.map = malloc(ports->out.n_ports * sizeof(ports->out.n_ports));
            digital.out.ports = &ports->out;
        }

    } else {

        cfg = &analog;
        analog_in = analog_out = -1;

        if(n_in) {
            ports->in.n_start = hal.port.num_analog_in;
            hal.port.num_analog_in += (ports->in.n_ports = n_in);
            ports->in.map = malloc(ports->in.n_ports * sizeof(ports->in.n_ports));
            analog.in.ports = &ports->in;
        }

        if(n_out) {
            ports->out.n_start = hal.port.num_analog_out;
            hal.port.num_analog_out += (ports->out.n_ports = n_out);
            ports->out.map = malloc(ports->out.n_ports * sizeof(ports->out.n_ports));
            analog.out.ports = &ports->out;
        }
    }

    if((n_ports = max(ports->in.n_ports, ports->out.n_ports)) > 0)  {

        char *pn;
        uint_fast8_t i;

        if((ports->pnum = pn = malloc((3 * n_ports + (n_ports > 9 ? n_ports - 10 : 0)) + 1)))
          for(i = 0; i < n_ports; i++) {

            if(pn) {
                *pn = type == Port_Digital ? 'P' : 'E';
                strcpy(pn + 1, uitoa(i));
            }

            if(ports->in.n_ports && i < ports->in.n_ports) {
                if(ports->in.map)
                    ports->in.map[i] = i;
                if(hal.port.set_pin_description)
                    hal.port.set_pin_description(type, Port_Input, i, get_pnum(ports, i));
                if(i < 8) {
                    cfg->inx.mask = (cfg->inx.mask << 1) + 1;
                    strcat(cfg->in.port_names, i == 0 ? "Aux " : ",Aux ");
                    strcat(cfg->in.port_names, uitoa(i));
                }
            }

            if(ports->out.n_ports && i < ports->out.n_ports) {
                if(ports->out.map)
                    ports->out.map[i] = i;
                if(hal.port.set_pin_description)
                    hal.port.set_pin_description(type, Port_Output, i, get_pnum(ports, i));
                if(i < 8) {
                    cfg->outx.mask = (cfg->outx.mask << 1) + 1;
                    strcat(cfg->out.port_names, i == 0 ? "Aux " : ",Aux ");
                    strcat(cfg->out.port_names, uitoa(i));
                }
            }

            if(pn)
                pn += i > 9 ? 4 : 3;
        }

        cfg->in.enabled.mask = cfg->inx.mask;
        cfg->out.enabled.mask = cfg->outx.mask;
    }

    return n_ports > 0;
}

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

/*! \brief calculate inverted pwm value if configured
\param pwm_data pointer t a \a spindle_pwm_t structure.
\param pwm_value non inverted PWM value.
\returns the inverted PWM value to use.
*/
static inline uint_fast16_t invert_pwm (ioports_pwm_t *pwm_data, uint_fast16_t pwm_value)
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
    if(digital.inx.mask & (1 << xbar->id)) {
        if(config->inverted)
            settings.ioport.invert_in.mask |= (1 << xbar->id);
        else
            settings.ioport.invert_in.mask &= ~(1 << xbar->id);
    }

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
    if(digital.outx.mask & (1 << xbar->id)) {
        if(config->inverted)
            settings.ioport.invert_out.mask |= (1 << xbar->id);
        else
            settings.ioport.invert_out.mask &= ~(1 << xbar->id);
    }

    settings_write_global();
}

static bool is_setting_available (const setting_detail_t *setting)
{
    bool available = false;

    switch(setting->id) {

        case Settings_IoPort_InvertIn:
        case Settings_IoPort_Pullup_Disable:
            available = digital.in.ports && digital.inx.mask;
            break;

        case Settings_IoPort_InvertOut:
        case Settings_IoPort_OD_Enable:
            available = digital.out.ports && digital.outx.mask;
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

            change.mask = value & digital.inx.mask;

            if((changed.mask = settings.ioport.invert_in.mask ^ change.mask)) {

                gpio_in_config_t config = {0};

                do {
                    if((changed.mask & 0x01) && (xbar = hal.port.get_pin_info(Port_Digital, Port_Input, ioports_map_reverse(digital.in.ports, port)))) {
                        if(xbar->config) {
                            config.pull_mode = (pull_mode_t)xbar->mode.pull_mode;
                            config.inverted = !!(change.mask & (1 << xbar->id));
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

            change.mask = value & digital.inx.mask;

            if((changed.mask = settings.ioport.pullup_disable_in.mask ^ change.mask)) {

                gpio_in_config_t config = {0};

                do {
                    if((changed.mask & 0x01) && (xbar = hal.port.get_pin_info(Port_Digital, Port_Input, ioports_map_reverse(digital.in.ports, port)))) {
                        if(xbar->config) {
                            config.pull_mode = change.mask & (1 << xbar->id)  ? PullMode_Down : PullMode_Up;
                            config.inverted = xbar->mode.inverted;
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

            change.mask = value & digital.outx.mask;

            if((changed.mask = settings.ioport.invert_out.mask ^ change.mask)) {

                gpio_out_config_t config = {0};

                do {
                    if((changed.mask & 0x01) && (xbar = hal.port.get_pin_info(Port_Digital, Port_Output, ioports_map_reverse(digital.out.ports, port)))) {
                        if(xbar->config && !(xbar->mode.pwm || xbar->mode.servo_pwm)) {
                            config.inverted = !!(change.mask & (1 << xbar->id));
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

            change.mask = value & digital.outx.mask;

            if((changed.mask = settings.ioport.od_enable_out.mask ^ change.mask)) {

                gpio_out_config_t config = {0};

                do {
                    if((changed.mask & 0x01) && (xbar = hal.port.get_pin_info(Port_Digital, Port_Output, ioports_map_reverse(digital.out.ports, port)))) {
                        if(xbar->config && !(xbar->mode.pwm || xbar->mode.servo_pwm)) {
                            config.inverted = xbar->mode.inverted;
                            config.open_drain = !!(change.mask & (1 << xbar->id));
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
            value = settings.ioport.invert_in.mask & digital.in.enabled.mask;
            break;

        case Settings_IoPort_Pullup_Disable:
            value = settings.ioport.pullup_disable_in.mask & digital.in.enabled.mask;
            break;

        case Settings_IoPort_InvertOut:
            value = settings.ioport.invert_out.mask & digital.out.enabled.mask;
            break;

        case Settings_IoPort_OD_Enable:
            value = settings.ioport.od_enable_out.mask & digital.out.enabled.mask;
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
    { Settings_IoPort_InvertIn, Group_AuxPorts, "Invert I/O Port inputs", NULL, Format_Bitfield, digital.in.port_names, NULL, NULL, Setting_NonCoreFn, aux_set_value, aux_get_value, is_setting_available },
#ifdef AUX_SETTINGS_PULLUP
    { Settings_IoPort_Pullup_Disable, Group_AuxPorts, "I/O Port inputs pullup disable", NULL, Format_Bitfield, digital.in.port_names, NULL, NULL, Setting_NonCoreFn, aux_set_value, aux_get_value, is_setting_available },
#endif
    { Settings_IoPort_InvertOut, Group_AuxPorts, "Invert I/O Port outputs", NULL, Format_Bitfield, digital.out.port_names, NULL, NULL, Setting_NonCoreFn, aux_set_value, aux_get_value, is_setting_available },
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

static void ioport_settings_load (void)
{
    uint8_t port;
    xbar_t *xbar;
    gpio_in_config_t in_config = {0};
    gpio_out_config_t out_config = {0};

    settings.ioport.invert_in.mask &= digital.inx.mask;
    settings.ioport.pullup_disable_in.mask &= digital.inx.mask;
    settings.ioport.invert_out.mask &= digital.outx.mask;
    settings.ioport.od_enable_out.mask &= digital.outx.mask;

    if(digital.in.ports && (port = digital.in.ports->n_ports)) do {
        if((xbar = hal.port.get_pin_info(Port_Digital, Port_Input, ioports_map_reverse(digital.in.ports, --port)))) {
            if(xbar->config) {

                in_config.debounce = xbar->mode.debounce;
#ifdef AUX_SETTINGS_PULLUP
                in_config.pull_mode = settings.ioport.pullup_disable_in.mask & (1 << xbar->id) ? PullMode_None : PullMode_Up;
#else
                in_config.pull_mode = (pull_mode_t)xbar->mode.pull_mode;
#endif
                in_config.inverted = !!(settings.ioport.invert_in.mask & (1 << xbar->id));

                // For probe and control signals higher level config takes priority
                if(xbar->function == Input_Probe)
                    in_config.inverted = settings.probe.invert_probe_pin;
                else if(xbar->function < Input_Probe) {
                    control_signals_t ctrl;
                    if((ctrl = xbar_fn_to_signals_mask(xbar->function)).mask)
                        in_config.inverted = !!(settings.control_invert.mask & ctrl.mask);
                }

                if(in_config.inverted)
                    settings.ioport.invert_in.mask |= (1 << xbar->id);
                else
                    settings.ioport.invert_in.mask &= ~(1 << xbar->id);

                xbar->config(xbar, &in_config, false);
            }
        }
    } while(port);

    if(digital.out.ports && (port = digital.out.ports->n_ports)) do {
        if((xbar = hal.port.get_pin_info(Port_Digital, Port_Output, ioports_map_reverse(digital.out.ports, --port)))) {
            if(xbar->config && !(xbar->mode.pwm || xbar->mode.servo_pwm)) {
                out_config.inverted = !!(settings.ioport.invert_out.mask & (1 << xbar->id));
                out_config.open_drain = !!(settings.ioport.od_enable_out.mask & (1 << xbar->id));
                xbar->config(xbar, &out_config, false);
            }
        }
    } while(port);

    if(on_settings_loaded)
        on_settings_loaded();
}

void ioport_setting_changed (setting_id_t id)
{
    if(on_setting_changed)
        on_setting_changed(id);

    else if(digital.in.ports && digital.in.ports->n_ports) switch(id) {

        case Setting_InvertProbePin:
        case Setting_ProbePullUpDisable:
            {
                xbar_t *xbar;
                gpio_in_config_t in_config = {0};
                uint8_t port = digital.in.ports->n_ports;

                do {
                    if((xbar = hal.port.get_pin_info(Port_Digital, Port_Input, ioports_map_reverse(digital.in.ports, --port)))) {
                        if(xbar->config && xbar->function == Input_Probe) {

                            in_config.debounce  = Off;
                            in_config.inverted  = settings.probe.invert_probe_pin;
                            in_config.pull_mode = settings.probe.disable_probe_pullup ? PullMode_None : PullMode_Up;

                            if(in_config.inverted)
                                settings.ioport.invert_in.mask |= (1 << xbar->id);
                            else
                                settings.ioport.invert_in.mask &= ~(1 << xbar->id);

                            xbar->config(xbar, &in_config, false);
                        } else if(xbar->config && xbar->function == Input_Toolsetter) {

                            in_config.debounce  = Off;
                            in_config.inverted  = settings.probe.invert_toolsetter_input;
                            in_config.pull_mode = settings.probe.disable_toolsetter_pullup ? PullMode_None : PullMode_Up;

                            if(in_config.inverted)
                                settings.ioport.invert_in.mask |= (1 << xbar->id);
                            else
                                settings.ioport.invert_in.mask &= ~(1 << xbar->id);

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
                uint8_t port = digital.in.ports->n_ports;

                do {
                    if((xbar = hal.port.get_pin_info(Port_Digital, Port_Input, ioports_map_reverse(digital.in.ports, --port)))) {
                        if(xbar->config && xbar->function < Input_Probe) {

                            in_config.debounce = xbar->mode.debounce;
                            in_config.inverted = !!(settings.ioport.invert_in.mask & (1 << xbar->id));
                            in_config.pull_mode = (pull_mode_t)xbar->mode.pull_mode;

                            if((ctrl = xbar_fn_to_signals_mask(xbar->function)).mask) {
                                in_config.inverted = !!(settings.control_invert.mask & ctrl.mask);
                                in_config.pull_mode = (settings.control_disable_pullup.mask & ctrl.mask) ? PullMode_None : PullMode_Up;
                            }

                            if(in_config.inverted)
                                settings.ioport.invert_in.mask |= (1 << xbar->id);
                            else
                                settings.ioport.invert_in.mask &= ~(1 << xbar->id);

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

void ioports_add_settings (driver_settings_load_ptr settings_loaded, setting_changed_ptr setting_changed)
{
    static setting_details_t setting_details = {
        .groups = ioport_groups,
        .n_groups = sizeof(ioport_groups) / sizeof(setting_group_detail_t),
        .settings = ioport_settings,
        .n_settings = sizeof(ioport_settings) / sizeof(setting_detail_t),
    #ifndef NO_SETTINGS_DESCRIPTIONS
        .descriptions = ioport_settings_descr,
        .n_descriptions = sizeof(ioport_settings_descr) / sizeof(setting_descr_t),
    #endif
        .load = ioport_settings_load,
        .save = settings_write_global
    };

    if(settings_loaded)
        on_settings_loaded = settings_loaded;

    if(setting_changed)
        on_setting_changed = setting_changed;

    settings_register(&setting_details);
}
