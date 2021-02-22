/*
  stream.h - some ASCII control character definitions and optional structures for stream buffers

  Part of grblHAL

  Copyright (c) 2019-2020 Terje Io

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

#ifndef _STREAM_H_
#define _STREAM_H_

#define ASCII_ETX  0x03
#define ASCII_ACK  0x06
#define ASCII_BS   0x08
#define ASCII_TAB  0x09
#define ASCII_LF   0x0A
#define ASCII_CR   0x0D
#define ASCII_XON  0x11
#define ASCII_XOFF 0x13
#define ASCII_NAK  0x15
#define ASCII_EOF  0x1A
#define ASCII_CAN  0x18
#define ASCII_EM   0x19
#define ASCII_ESC  0x1B
#define ASCII_DEL  0x7F
#define ASCII_EOL  "\r\n"

#ifndef RX_BUFFER_SIZE
#define RX_BUFFER_SIZE 1024 // must be a power of 2
#endif

#ifndef TX_BUFFER_SIZE
#define TX_BUFFER_SIZE 512  // must be a power of 2
#endif

#ifndef BLOCK_TX_BUFFER_SIZE
#define BLOCK_TX_BUFFER_SIZE 256
#endif

// Serial baud rate
#ifndef BAUD_RATE
#define BAUD_RATE 115200
#endif

// Value to be returned from input stream when no data is available
#ifndef SERIAL_NO_DATA
#define SERIAL_NO_DATA -1
#endif

#define BUFCOUNT(head, tail, size) ((head >= tail) ? (head - tail) : (size - tail + head))

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    StreamType_Serial = 0,
    StreamType_MPG,
    StreamType_Bluetooth,
    StreamType_Telnet,
    StreamType_WebSocket,
    StreamType_SDCard,
    StreamType_FlashFs,
    StreamType_Redirected,
    StreamType_Null
} stream_type_t;

// These structures are not referenced in the core code, may be used by drivers

typedef struct {
    volatile uint_fast16_t head;
    volatile uint_fast16_t tail;
    bool overflow;
#ifdef SERIAL_RTS_HANDSHAKE
    volatile bool rts_state;
#endif
    bool backup;
    char data[RX_BUFFER_SIZE];
} stream_rx_buffer_t;

typedef struct {
    volatile uint_fast16_t head;
    volatile uint_fast16_t tail;
    char data[TX_BUFFER_SIZE];
} stream_tx_buffer_t;

typedef struct {
    uint_fast16_t length;
    uint_fast16_t max_length;
    char *s;
    char data[BLOCK_TX_BUFFER_SIZE];
} stream_block_tx_buffer_t;

#endif
