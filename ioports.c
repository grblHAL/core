/*
  ioports.c - some wrapper functions for the ioports HAL API

  Part of grblHAL

  Copyright (c) 2021-2023 Terje Io

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

/**
 * @file
 *
 * Some wrapper functions for the #io_port_t API.
 * They perform the necessary checks for both availability of ports
 * and advanced functionality simplifying plugin code that uses them.
 */

#include <math.h>

#include "hal.h"

typedef struct {
    io_ports_detail_t *ports;
    char port_names[50];
} io_ports_private_t;

typedef struct {
    io_ports_private_t in;
    io_ports_private_t out;
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

/*! \brief Claim a digital or analog port for exclusive use.
\param type as an \a #io_port_type_t enum value.
\param dir as an \a #io_port_direction_t enum value.
\param port pointer to a \a uint8_t holding the ports aux number, returns the actual port number to use if successful.
\param description pointer to a \a char constant for the pin description.
\returns true if successful, false if not.
*/
bool ioport_claim (io_port_type_t type, io_port_direction_t dir, uint8_t *port, const char *description)
{
    bool ok = false;
    uint8_t n_ports = ioports_available(type, dir);
    uint8_t base = type == Port_Digital
                    ? (dir == Port_Input ? Input_Aux0 : Output_Aux0)
                    : (dir == Port_Input ? Input_Analog_Aux0 : Output_Analog_Aux0);

    if(hal.port.claim != NULL) {

        xbar_t *portinfo;

        if(n_ports > 0) do {
            n_ports--;
            portinfo = hal.port.get_pin_info(type, dir, n_ports);
            if((ok = portinfo && !portinfo->mode.claimed && (portinfo->function - base) == *port)) {
                ok = hal.port.claim(type, dir, port, description);
                break;
            }
        } while(n_ports && !ok);

    } else if((ok = n_ports > 0)) {

        if(type == Port_Digital)
            *port = dir == Port_Input ? --hal.port.num_digital_in : --hal.port.num_digital_out;
        else
            *port = dir == Port_Input ? --hal.port.num_analog_in : --hal.port.num_analog_out;

        if(hal.port.set_pin_description)
            hal.port.set_pin_description(type, dir, *port, description);
    }

    return ok;
}

/*! \brief Check if ports can be claimed by aux number or not.
\returns true if ports can be claimed by aux number, false if claimed ports are allocated by the API.
*/
bool ioport_can_claim_explicit (void)
{
    return !(hal.port.claim == NULL || hal.port.get_pin_info == NULL);
}

bool ioports_enumerate (io_port_type_t type, io_port_direction_t dir, pin_mode_t filter, bool claimable, ioports_enumerate_callback_ptr callback)
{
    bool ok = false;
    uint8_t n_ports = ioports_available(type, dir);
    xbar_t *portinfo;

    if(n_ports && ioport_can_claim_explicit()) do {
        portinfo = hal.port.get_pin_info(type, dir, --n_ports);
        if(claimable && portinfo->mode.claimed)
            continue;
        if((portinfo->mode.mask & filter.mask) == filter.mask && (ok = callback(portinfo, n_ports)))
            break;
    } while(n_ports);

    return ok;
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
    }

    return n_ports > 0;
}

uint8_t ioports_map_reverse (io_ports_detail_t *type, uint8_t port)
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
    if(config->max > config->min) {
        pwm_data->min = config->min;
        pwm_data->period = (uint_fast16_t)((float)clock_hz / config->freq_hz);
        if(config->off_value == 0.0f)
            pwm_data->off_value = pwm_data->invert_pwm ? pwm_data->period : 0;
        else
            pwm_data->off_value = invert_pwm(pwm_data, (uint_fast16_t)(pwm_data->period * config->off_value / 100.0f));
        pwm_data->min_value = (uint_fast16_t)(pwm_data->period * config->min_value / 100.0f);
        pwm_data->max_value = (uint_fast16_t)(pwm_data->period * config->max_value / 100.0f); // + pwm_data->offset;
        pwm_data->pwm_gradient = (float)(pwm_data->max_value - pwm_data->min_value) / (config->max - config->min);
        pwm_data->always_on = config->off_value != 0.0f;
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


static bool is_setting_available (const setting_detail_t *setting)
{
    bool available = false;

    switch(setting->id) {

        case Settings_IoPort_InvertIn:
        case Settings_IoPort_Pullup_Disable:
            available = digital.in.ports->n_ports > 0;
            break;

        case Settings_IoPort_InvertOut:
        case Settings_IoPort_OD_Enable:
            available = digital.out.ports->n_ports > 0;
            break;

        default:
            break;
    }

    return available;
}

static status_code_t aux_set_invert_out (setting_id_t id, uint_fast16_t value)
{
    ioport_bus_t invert;
    invert.mask = (uint8_t)value & digital.outx.mask;

    if(settings.ioport.invert_out.mask != invert.mask) {

        settings.ioport.invert_out.mask = invert.mask;

        if(on_setting_changed)
            on_setting_changed(id);
    }

    return Status_OK;
}

static uint32_t aux_get_invert_out (setting_id_t setting)
{
    return settings.ioport.invert_out.mask;
}

static const setting_group_detail_t ioport_groups[] = {
    { Group_Root, Group_AuxPorts, "Aux ports"}
};

static const setting_detail_t ioport_settings[] = {
    { Settings_IoPort_InvertIn, Group_AuxPorts, "Invert I/O Port inputs", NULL, Format_Bitfield, digital.in.port_names, NULL, NULL, Setting_NonCore, &settings.ioport.invert_in.mask, NULL, is_setting_available },
//    { Settings_IoPort_Pullup_Disable, Group_AuxPorts, "I/O Port inputs pullup disable", NULL, Format_Bitfield, "Port 0,Port 1,Port 2,Port 3,Port 4,Port 5,Port 6,Port 7", NULL, NULL },
    { Settings_IoPort_InvertOut, Group_AuxPorts, "Invert I/O Port outputs", NULL, Format_Bitfield, digital.out.port_names, NULL, NULL, Setting_NonCoreFn, aux_set_invert_out, aux_get_invert_out, is_setting_available },
//    { Settings_IoPort_OD_Enable, Group_AuxPorts, "I/O Port outputs as open drain", NULL, Format_Bitfield, "Port 0,Port 1,Port 2,Port 3,Port 4,Port 5,Port 6,Port 7", NULL, NULL }
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
    if(on_settings_loaded)
        on_settings_loaded();
}

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

void ioports_add_settings (driver_settings_load_ptr settings_loaded, setting_changed_ptr setting_changed)
{
    if(settings_loaded)
        on_settings_loaded = settings_loaded;

    if(setting_changed)
        on_setting_changed = setting_changed,

    settings_register(&setting_details);
}
