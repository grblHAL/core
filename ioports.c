/*
  ioports.c - some wrapper functions for the ioports HAL API

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

/**
 * @file
 *
 * Some wrapper functions for the #io_port_t API.
 * They perform the neccesary checks for both availablity of ports
 * and advanced functionality simplifying plugin code that uses them.
 */

#include "hal.h"

static int16_t digital_in = -1,  digital_out = -1, analog_in = -1,  analog_out = -1;

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
                    : (dir == Port_Input ? Input_Aux0 : Output_Aux0); // TODO add analog ports?

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
