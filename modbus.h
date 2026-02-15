/*

  modbus.h - a lightweight ModBus implementation

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

#ifndef _MODBUS_H_
#define _MODBUS_H_

#include <stdint.h>
#include <stdbool.h>

#include "errors.h"

#ifndef MODBUS_MAX_ADU_SIZE
#define MODBUS_MAX_ADU_SIZE 12
#endif
#ifndef MODBUS_QUEUE_LENGTH
#define MODBUS_QUEUE_LENGTH 8
#endif

#define MODBUS_SET_MSB16(v) ((v) >> 8)
#define MODBUS_SET_LSB16(v) ((v) & 0xFF)
#define MODBUS_MAX_REGISTERS ((MODBUS_MAX_ADU_SIZE - 6) / 2)

typedef enum {
    Modbus_InterfaceRTU = 0,
    Modbus_InterfaceASCII,
    Modbus_InterfaceTCP
} modbus_if_t;

typedef enum {
    ModBus_ReadCoils = 1,
    ModBus_ReadDiscreteInputs = 2,
    ModBus_ReadHoldingRegisters = 3,
    ModBus_ReadInputRegisters = 4,
    ModBus_WriteCoil = 5,
    ModBus_WriteRegister = 6,
    ModBus_ReadExceptionStatus = 7,
    ModBus_Diagnostics = 8,
    ModBus_WriteCoils = 15,
    ModBus_WriteRegisters = 16
} modbus_function_t;

typedef enum {
    ModBus_NoException = 0, // For internal use, not defined in standard
    ModBus_IllegalFunction = 1,
    ModBus_IllegalDataAddress = 2,
    ModBus_IllegalDataValue = 3,
    ModBus_ServerDeviceFailure = 4,
    ModBus_Acknowledge = 5,
    ModBus_ServerDeviceBusy = 6,
    ModBus_MemoryParityError = 7,
    ModBus_GatewayPathUnavailable = 8, // 0x0A
    ModBus_GatewayTargetUnresponsive = 9, // 0x0B
// Internal exception codes, not defined in standard
    ModBus_UnknownException = 252,
    ModBus_IllegalSize = 253,
    ModBus_CRCError = 254,
    ModBus_Timeout = 255
} __attribute__ ((__packed__)) modbus_exception_t;

typedef struct {
    void *context;
    bool crc_check;
    uint8_t tx_length;
    uint8_t rx_length;
    uint8_t adu[MODBUS_MAX_ADU_SIZE];
} modbus_message_t;

typedef struct {
    uint8_t retries;
    uint16_t retry_delay;
    void (*on_rx_packet)(modbus_message_t *msg);
    void (*on_rx_exception)(uint8_t code, void *context);
    void (*on_rx_timeout)(uint8_t code, void *context); // Set to modbus_null_exception_handler() to disable.
} modbus_callbacks_t;

typedef union {
    uint16_t timeout[6];
    struct {
        uint16_t b2400;
        uint16_t b4800;
        uint16_t b9600;
        uint16_t b19200;
        uint16_t b38400;
        uint16_t b115200;
    };
} modbus_silence_timeout_t;

typedef union {
    uint8_t ok;
    struct {
        uint8_t rtu         :1,
                ascii       :1,
                tcp         :1,
                unassigned  :6;
    };
} modbus_cap_t;

struct modbus_response;

typedef void (*modbus_callback_ptr)(struct modbus_response *response);

typedef struct modbus_response {
    uint8_t function;
    modbus_exception_t exception;
    uint8_t num_values;
    uint16_t values[MODBUS_MAX_REGISTERS];
} modbus_response_t;

typedef struct {
    modbus_function_t function;
    bool is_write;
    bool single_register;
} modbus_function_properties_t;

typedef bool (*modbus_is_up_ptr)(void);
typedef void (*modbus_flush_queue_ptr)(void);
typedef void (*modbus_set_silence_ptr)(const modbus_silence_timeout_t *timeout);
typedef bool (*modbus_send_ptr)(modbus_message_t *msg, const modbus_callbacks_t *callbacks, bool block);
typedef bool (*modbus_is_busy_ptr)(void);

typedef struct {
    modbus_if_t interface;
    modbus_is_up_ptr is_up;
    modbus_flush_queue_ptr flush_queue;
    modbus_set_silence_ptr set_silence;
    modbus_send_ptr send;
    modbus_is_busy_ptr is_busy;
} modbus_api_t;

modbus_cap_t modbus_isup (void);
bool modbus_isbusy (void);
bool modbus_enabled (void);
void modbus_flush_queue (void);
void modbus_set_silence (const modbus_silence_timeout_t *timeout);
bool modbus_send (modbus_message_t *msg, const modbus_callbacks_t *callbacks, bool block);
void modbus_null_exception_handler (uint8_t code, void *context);
uint16_t modbus_read_u16 (uint8_t *p);
void modbus_write_u16 (uint8_t *p, uint16_t value);
bool modbus_register_api (const modbus_api_t *api);

// Experimental high level API

const modbus_function_properties_t *modbus_get_function_properties (modbus_function_t function);
status_code_t modbus_message (uint8_t server, modbus_function_t function, uint16_t address, uint16_t *values, uint8_t registers, modbus_callback_ptr callback);

#endif
