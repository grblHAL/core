/*

  modbus_rtu.h - a lightweight ModBus RTU implementation

  Part of grblHAL

  Copyright (c) 2020-2025 Terje Io

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

#ifndef _MODBUS_RTU_H_
#define _MODBUS_RTU_H_

#include "grbl/modbus.h"

typedef enum {
    ModBus_Idle,
    ModBus_Silent,
    ModBus_TX,
    ModBus_AwaitReply,
    ModBus_Timeout,
    ModBus_GotReply,
    ModBus_Exception,
    ModBus_Retry
} modbus_state_t;

typedef void (*stream_set_direction_ptr)(bool tx);

typedef struct {
    set_baud_rate_ptr set_baud_rate;
    stream_set_direction_ptr set_direction; // NULL if auto direction
    get_stream_buffer_count_ptr get_tx_buffer_count;
    get_stream_buffer_count_ptr get_rx_buffer_count;
    stream_write_n_ptr write;
    stream_read_ptr read;
    flush_stream_buffer_ptr flush_tx_buffer;
    flush_stream_buffer_ptr flush_rx_buffer;
} modbus_stream_t;

void modbus_rtu_init (void);
bool modbus_rtu_send (modbus_message_t *msg, const modbus_callbacks_t *callbacks, bool block);

#endif
