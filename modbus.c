/*

  modbus.c - a lightweight ModBus implementation, interface wrapper

  Part of grblHAL

  Copyright (c) 2023-2026 Terje Io

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

FLASHMEM modbus_cap_t modbus_isup (void)
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

bool modbus_isbusy (void)
{
    bool busy = false;

    uint_fast16_t idx = n_api;

    if(idx) do {
        idx--;
        if(modbus[idx].is_busy) switch(modbus[idx].interface) {

            case Modbus_InterfaceRTU:
                busy = modbus[idx].is_busy();
                break;

            case Modbus_InterfaceASCII:
                busy = modbus[idx].is_busy();
                break;

            case Modbus_InterfaceTCP:
                busy = modbus[idx].is_busy();
                break;
        }
    } while(idx && !busy);

    return busy;
}

FLASHMEM bool modbus_enabled (void)
{
    return n_api > 0;
}

FLASHMEM void modbus_flush_queue (void)
{
    uint_fast16_t idx = n_api;

    if(idx) do {
        modbus[--idx].flush_queue();
    } while(idx);
}

FLASHMEM void modbus_set_silence (const modbus_silence_timeout_t *timeout)
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

// Dummy exception handler
FLASHMEM void modbus_null_exception_handler (uint8_t code, void *context)
{
//    NOOP
}

FLASHMEM uint16_t modbus_read_u16 (uint8_t *p)
{
    return (*p << 8) | *(p + 1);
}

FLASHMEM void modbus_write_u16 (uint8_t *p, uint16_t value)
{
    *p = (uint8_t)(value >> 8);
    *(p + 1) = (uint8_t)(value & 0x00FF);
}

FLASHMEM bool modbus_register_api (const modbus_api_t *api)
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

// Experimental high level API

static void rx_packet (modbus_message_t *msg);
static void rx_exception (uint8_t code, void *context);
static void rx_timeout (uint8_t code, void *context);

PROGMEM static const modbus_function_properties_t cmds[] = {
    { 0, false, false },
    { ModBus_ReadCoils, false, false },
    { ModBus_ReadDiscreteInputs, false, false },
    { ModBus_ReadHoldingRegisters, false, false },
    { ModBus_ReadInputRegisters, false, false },
    { ModBus_WriteCoil, true, true },
    { ModBus_WriteRegister, true, true },
    { ModBus_ReadExceptionStatus, false, true },
    { 0, true, false }, // ModBus_Diagnostics
    { 0, false, false },
    { 0, false, false },
    { 0, false, false },
    { 0, false, false },
    { 0, false, false },
    { 0, false, false },
    { ModBus_WriteCoils, true, false },
    { ModBus_WriteRegisters, true, false }
};
PROGMEM static const uint8_t max_function = (sizeof(cmds) / sizeof(modbus_function_properties_t)) - 1;
PROGMEM static const modbus_callbacks_t callbacks = {
    .retries = 5,
    .retry_delay = 100,
    .on_rx_packet = rx_packet,
    .on_rx_exception = rx_exception,
    .on_rx_timeout = rx_timeout
};

static uint8_t tx_id;
static modbus_callback_ptr xcallback;
static modbus_response_t response;

FLASHMEM static void rx_exception (uint8_t code, void *context)
{
    response.exception = code;

    if(xcallback)
        xcallback(&response);
}

FLASHMEM static void rx_timeout (uint8_t code, void *context)
{
    response.exception = ModBus_Timeout;

    if(xcallback)
        xcallback(&response);
}

FLASHMEM static void rx_packet (modbus_message_t *msg)
{
    if(!(msg->adu[0] & 0x80)) {

        uint_fast8_t idx;

        response.function = msg->adu[1];

        if(response.function <= max_function && cmds[response.function].function != 0)
          switch(response.function) {

            case ModBus_ReadExceptionStatus:
                response.num_values = 1;
                response.values[0] = msg->adu[2];
                break;

            case ModBus_Diagnostics:
                response.num_values = 0; // TBA
                break;

            default:;
                response.num_values = cmds[response.function].single_register || cmds[response.function].is_write ? 2 : msg->adu[2] / 2;
                response.num_values = min(response.num_values, 3);
                uint_fast8_t pos = cmds[response.function].single_register || cmds[response.function].is_write ? 2 : 3;
                for(idx = 0; idx < response.num_values; idx++)
                    response.values[idx] = modbus_read_u16(&msg->adu[pos + (idx << 1)]);
                break;
        } else
            response.exception = 255;

        if(xcallback)
            xcallback(&response);
    }
}

FLASHMEM const modbus_function_properties_t *modbus_get_function_properties (modbus_function_t function)
{
    const modbus_function_properties_t *details = NULL;

    if(function <= max_function && cmds[function].function)
        details = &cmds[function];

    return details;
}

FLASHMEM status_code_t modbus_message (uint8_t server, modbus_function_t function, uint16_t address, uint16_t *values, uint8_t registers, modbus_callback_ptr callback)
{
    if(function > max_function || !cmds[function].function)
        return Status_InvalidStatement;

    uint_fast8_t idx;
    status_code_t status;
    modbus_message_t cmd = {
        .context = (void *)((uint32_t)tx_id++),
        .crc_check = true,
        .adu[0] = (uint8_t)server,
        .adu[1] = (uint8_t)function,
        .adu[2] = (uint8_t)(address >> 8),
        .adu[3] = (uint8_t)(address & 0xFF)
    };

    xcallback = callback;

    memset(&response, 0, sizeof(modbus_response_t));

    if(function == ModBus_ReadExceptionStatus) {
        cmd.tx_length = 4;
        cmd.rx_length = 5;
    } else {

        cmd.tx_length = 6 + 2 * registers;
        cmd.rx_length = cmd.tx_length - 1;

        if(cmds[function].is_write) {
            if(cmds[function].single_register) {
                cmd.tx_length = cmd.rx_length = 8;
                for(idx = 0; idx < registers; idx++) {
                    cmd.adu[(idx << 1) + 4] = (uint8_t)(values[idx] >> 8);
                    cmd.adu[(idx << 1) + 5] = (uint8_t)(values[idx] & 0xFF);
                }
            } else {
                cmd.tx_length += 3; cmd.rx_length = 8;
                cmd.adu[4] = (uint8_t)(registers >> 8);
                cmd.adu[5] = (uint8_t)(registers & 0xFF);
                cmd.adu[6] = (uint8_t)(registers << 1);
                for(idx = 0; idx < registers; idx++) {
                    cmd.adu[(idx << 1) + 7] = (uint8_t)(values[idx] >> 8);
                    cmd.adu[(idx << 1) + 8] = (uint8_t)(values[idx] & 0xFF);
                }
            }
        } else { // read
            cmd.adu[4] = (uint8_t)(registers >> 8);
            cmd.adu[5] = (uint8_t)(registers & 0xFF);
            cmd.rx_length = 5 + (registers << 1);
        }
    }

    status = modbus_send(&cmd, &callbacks, true) ? Status_OK : Status_AccessDenied;

    return status;
}

/**/
