/*

  modbus.h - a lightweight ModBus implementation, interface wrapper

  Part of grblHAL

  Copyright (c) 2023-2025 Terje Io

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

#include "modbus.h"

#include <string.h>

#include "nuts_bolts.h"

#define N_MODBUS_API 2

static uint_fast16_t n_api = 0, tcp_api = N_MODBUS_API, rtu_api = N_MODBUS_API;
static modbus_api_t modbus[N_MODBUS_API] = {0};

modbus_cap_t modbus_isup (void)
{
    uint_fast16_t idx = n_api;
    modbus_cap_t cap = {};

    if(idx) do {
        idx--;
        if(modbus[idx].is_up()) switch(modbus[idx].interface) {

            case Modbus_InterfaceRTU:
                cap.rtu = On;
                break;

            case Modbus_InterfaceASCII:
                cap.ascii = On;
                break;

            case Modbus_InterfaceTCP:
                cap.tcp = On;
                break;
        }
    } while(idx);

    return cap;
}

bool modbus_enabled (void)
{
    return n_api > 0;
}

void modbus_flush_queue (void)
{
    uint_fast16_t idx = n_api;

    if(idx) do {
        modbus[--idx].flush_queue();
    } while(idx);
}

void modbus_set_silence (const modbus_silence_timeout_t *timeout)
{
    if(rtu_api != N_MODBUS_API)
        modbus[rtu_api].set_silence(timeout);
}

bool modbus_send (modbus_message_t *msg, const modbus_callbacks_t *callbacks, bool block)
{
    bool ok = false;

    if(tcp_api != N_MODBUS_API)
        ok = modbus[tcp_api].send(msg, callbacks, block);

    return ok || (rtu_api != N_MODBUS_API && modbus[rtu_api].send(msg, callbacks, block));
}

uint16_t modbus_read_u16 (uint8_t *p)
{
    return (*p << 8) | *(p + 1);
}

void modbus_write_u16 (uint8_t *p, uint16_t value)
{
    *p = (uint8_t)(value >> 8);
    *(p + 1) = (uint8_t)(value & 0x00FF);
}

bool modbus_register_api (const modbus_api_t *api)
{
    bool ok;

    if((ok = n_api < N_MODBUS_API)) {
        memcpy(&modbus[n_api], api, sizeof(modbus_api_t));
        if(api->interface == Modbus_InterfaceTCP)
            tcp_api = n_api;
        else if(api->interface == Modbus_InterfaceRTU)
            rtu_api = n_api;
        n_api++;
    }

    return ok;
}
