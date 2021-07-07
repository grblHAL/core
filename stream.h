/*
  stream.h - some ASCII control character definitions and optional structures for stream buffers

  Part of grblHAL

  Copyright (c) 2019-2021 Terje Io

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

/*! \file
Stream I/O functions structure for the HAL.

Helper functions for saving away and restoring a stream input buffer. _Not referenced by the core._
*/

#ifndef _STREAM_H_
#define _STREAM_H_

#define ASCII_SOH  0x01
#define ASCII_STX  0x02
#define ASCII_ETX  0x03
#define ASCII_EOT  0x04
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

/*! \brief Pointer to function for getting number of characters available or free in a stream buffer.
\returns number of characters available or free.
*/
typedef uint16_t (*get_stream_buffer_count_ptr)(void);

/*! \brief Pointer to function for reading a single character from a input stream.
\returns character or -1 if none available.
*/
typedef int16_t (*stream_read_ptr)(void);

/*! \brief Pointer to function for writing a null terminated string to the output stream.
\param s pointer to null terminated string.

__NOTE:__ output might be buffered until an #ASCII_LF is output, this is usually done by USB or network driver code to improve throughput.
*/
typedef void (*stream_write_ptr)(const char *s);

/*! \brief Pointer to function for writing a \a n character long string to the output stream.
\param s pointer to string.
\param c number of characters to write.
*/
typedef void (*stream_write_n_ptr)(const char *s, uint16_t len);


/*! \brief Pointer to function for writing a single character to the output stream.
\param \a n characters to write.
*/
typedef bool (*stream_write_char_ptr)(const char c);

/*! \brief Pointer to function for extracting real-time commands from the input stream and enqueue them for processing.
This should be called by driver code prior to inserting a character into the input buffer.
\param c character to check.
\returns true if extracted, driver code should not insert the character into the input buffer if so.

__NOTE:__ This is set up by the core at startup and may be changed at runtime by the core and/or plugin code.
*/
typedef bool (*enqueue_realtime_command_ptr)(char data);

/*! \brief Pointer to function for setting the stream baud rate.
\param baud_rate
\returns true if successful.
*/
typedef bool (*set_baud_rate_ptr)(uint32_t baud_rate);

/*! \brief Pointer to function for flushing a stream buffer. */
typedef void (*flush_stream_buffer_ptr)(void);

/*! \brief Pointer to function for flushing the input buffer and inserting an #ASCII_CAN character.

This function is typically called by the _enqueue_realtime_command_ handler when CMD_STOP or CMD_JOG_CANCEL character is processed.
The #ASCII_CAN character might be checked for and used by upstream code to flush any buffers it may have.
*/
typedef void (*cancel_read_buffer_ptr)(void);

/*! \brief Pointer to function for blocking reads from and restoring a input buffer.

This function is called with the _await_ parameter true on executing a tool change command (M6),
 this shall block further reading from the input buffer.
The core function stream_rx_suspend() can be called with the _await_ parameter to do this,
it will replace the _hal.stream.read_ handler with a pointer to the dummy function stream_get_null().

Reading from the input is blocked until a tool change aknowledge character #CMD_TOOL_ACK is received,
 when the driver receives this the input buffer is to be saved away and reading from the input resumed by
 restoring the _hal.stream.read_ handler with its own read character function.
Driver code can do this by calling the core function stream_rx_backup().

When the tool change is complete or a soft reset is executed the core will call this function with the _await_ parameter false,
 if the driver code called stream_rx_suspend() to block input it shall then call it again with the _await_ parameter as input to restore it.

\param await bool
\returns \a true if there is data in the buffer, \a false otherwise.
*/
typedef bool (*suspend_read_ptr)(bool await);

/*! \brief Pointer to function for disabling/enabling stream input.

Typically used to disable receive interrupts so that real-time command processing for the stream is blocked.
Usually it is desirable to block processing when another stream provides the input, but sometimes not.
E.g. when input is from a SD card real-time commands from the stream that initiated SD card streaming is needed
for handing feed-holds, overrides, soft resets etc.

\param disable \a true to disable stream, \a false to enable,
*/

typedef bool (*disable_stream_ptr)(bool disable);

//! Properties and handlers for stream I/O
typedef struct {
    stream_type_t type;                                     //!< Type of stream.
    bool connected;                                         //!< Set to true by the driver if stream is connected. _Optional._ Under consideration.
    get_stream_buffer_count_ptr get_rx_buffer_free;         //!< Handler for getting number of free characters in the input buffer.
    stream_write_ptr write;                                 //!< Handler for writing string to current output stream only.
    stream_write_ptr write_all;                             //!< Handler for writing string to all active output streams.
    stream_write_char_ptr write_char;                       //!< Handler for writing a single character to current stream only.
    stream_read_ptr read;                                   //!< Handler for reading a single character from the input stream.
    flush_stream_buffer_ptr reset_read_buffer;              //!< Handler for flushing the input buffer.
    cancel_read_buffer_ptr cancel_read_buffer;              //!< Handler for flushing the input buffer and inserting an #ASCII_CAN character.
    suspend_read_ptr suspend_read;                          //!< Optional handler for saving away and restoring the current input buffer.
    stream_write_n_ptr write_n;                             //!< Optional handler for writing n characters to current output stream only. Required for Modbus support.
    disable_stream_ptr disable;                             //!< Optional handler for disabling/enabling a stream. Recommended?
    get_stream_buffer_count_ptr get_rx_buffer_count;        //!< Optional handler for getting number of characters in the input buffer.
    get_stream_buffer_count_ptr get_tx_buffer_count;        //!< Optional handler for getting number of characters in the output buffer(s). Count shall include any unsent characters in any transmit FIFO and/or transmit register. Required for Modbus support.
    flush_stream_buffer_ptr reset_write_buffer;             //!< Optional handler for flushing the output buffer. Any transmit FIFO shall be flushed as well. Required for Modbus support.
    set_baud_rate_ptr set_baud_rate;                        //!< Optional handler for setting the stream baud rate. Required for Modbus support, recommended for Bluetooth support.
//@}
//!  @name The following variable is not changed on a soft reset, do NOT move.
//@{
    enqueue_realtime_command_ptr enqueue_realtime_command;  //!< Handler for extracting real-time commands from the input stream. _Set by the core at startup._
} io_stream_t;


// The following structures and functions are not referenced in the core code, may be used by drivers

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

#ifdef __cplusplus
extern "C" {
#endif

/*! \brief Dummy function for reading data from a virtual empty input buffer.
\returns always -1 as there is no data available.
*/
int16_t stream_get_null (void);

/*! \brief Function for blocking reads from or restoring an input buffer.
\param rxbuffer pointer to a stream_rx_buffer_t.
\param suspend when true _hal.stream.read_ is changed to stream_get_null(), if false it is restored if already saved.
\returns true if there is data in the buffer, false otherwise.
*/
bool stream_rx_suspend (stream_rx_buffer_t *rxbuffer, bool suspend);

/*! \brief Function for saving away an input buffer.
\param rxbuffer pointer to a stream_rx_buffer_t.
*/
void stream_rx_backup (stream_rx_buffer_t *rxbuffer);

/*! \brief Function for enabling/disabling input from a secondary input stream.
\param mpg_stream pointer to a io_stream_t.
\param mpg_mode \a true if switching input to mpg stream, \a false when restoring original input.
\returns \a true when succsessful, \a false otherwise.
*/
bool stream_enable_mpg (const io_stream_t *mpg_stream, bool mpg_mode);

#ifdef __cplusplus
}
#endif

#endif
