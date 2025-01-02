/*

  modbus.h - a lightweight ModBus implementation

  Part of grblHAL

  Copyright (c) 2023 Terje Io

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

#ifndef _MODBUS_H_
#define _MODBUS_H_

#ifndef MODBUS_MAX_ADU_SIZE
#define MODBUS_MAX_ADU_SIZE 12
#endif
#ifndef MODBUS_QUEUE_LENGTH
#define MODBUS_QUEUE_LENGTH 8
#endif

#include <stdint.h>
#include <stdbool.h>

#define MODBUS_SET_MSB16(v) ((v) >> 8)
#define MODBUS_SET_LSB16(v) ((v) & 0xFF)

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

typedef bool (*modbus_is_up_ptr)(void);
typedef void (*modbus_flush_queue_ptr)(void);
typedef void (*modbus_set_silence_ptr)(const modbus_silence_timeout_t *timeout);
typedef bool (*modbus_send_ptr)(modbus_message_t *msg, const modbus_callbacks_t *callbacks, bool block);

typedef struct {
    modbus_if_t interface;
    modbus_is_up_ptr is_up;
    modbus_flush_queue_ptr flush_queue;
    modbus_set_silence_ptr set_silence;
    modbus_send_ptr send;
} modbus_api_t;

bool modbus_isup (void);
bool modbus_enabled (void);
void modbus_flush_queue (void);
void modbus_set_silence (const modbus_silence_timeout_t *timeout);
bool modbus_send (modbus_message_t *msg, const modbus_callbacks_t *callbacks, bool block);
uint16_t modbus_read_u16 (uint8_t *p);
void modbus_write_u16 (uint8_t *p, uint16_t value);
bool modbus_register_api (const modbus_api_t *api);

#endif
