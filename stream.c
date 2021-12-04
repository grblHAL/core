/*
  stream.c - stream RX handling for tool change protocol

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

#include <string.h>

#include "hal.h"
#include "protocol.h"
#include "state_machine.h"

static stream_rx_buffer_t rxbackup;

typedef struct {
    enqueue_realtime_command_ptr enqueue_realtime_command;
    stream_read_ptr read;
    stream_rx_buffer_t *rxbuffer;
} stream_state_t;

static stream_state_t stream = {0};
static io_stream_details_t *streams = NULL;

void stream_register_streams (io_stream_details_t *details)
{
    details->next = streams;
    streams = details;
}

bool stream_enumerate_streams (stream_enumerate_callback_ptr callback)
{
    if(streams == NULL || callback == NULL)
        return false;

    bool claimed = false;
    io_stream_details_t *details = streams;

    while(details && !claimed) {
        uint_fast8_t idx;
        for(idx = 0; idx < details->n_streams; idx++) {
            if((claimed = callback(&details->streams[idx])))
                break;
        }
        details = details->next;
    };

    return claimed;
}

// called from stream drivers while tx is blocking, returns false to terminate
bool stream_tx_blocking (void)
{
    // TODO: Restructure st_prep_buffer() calls to be executed here during a long print.

    grbl.on_execute_realtime(state_get());

    return !(sys.rt_exec_state & EXEC_RESET);
}

// "dummy" version of serialGetC
int16_t stream_get_null (void)
{
    return SERIAL_NO_DATA;
}

ISR_CODE static bool await_toolchange_ack (char c)
{
    if(c == CMD_TOOL_ACK && !stream.rxbuffer->backup) {
        memcpy(&rxbackup, stream.rxbuffer, sizeof(stream_rx_buffer_t));
        stream.rxbuffer->backup = true;
        stream.rxbuffer->tail = stream.rxbuffer->head;
        hal.stream.read = stream.read; // restore normal input
        hal.stream.set_enqueue_rt_handler(stream.enqueue_realtime_command);
        stream.enqueue_realtime_command = NULL;
    } else
        return stream.enqueue_realtime_command(c);

    return true;
}

bool stream_rx_suspend (stream_rx_buffer_t *rxbuffer, bool suspend)
{
    if(suspend) {
        stream.rxbuffer = rxbuffer;
        stream.read = hal.stream.read;
        stream.enqueue_realtime_command = hal.stream.set_enqueue_rt_handler(await_toolchange_ack);
        hal.stream.read = stream_get_null;
    } else {
        if(rxbuffer->backup)
            memcpy(rxbuffer, &rxbackup, sizeof(stream_rx_buffer_t));
        if(stream.enqueue_realtime_command) {
            hal.stream.read = stream.read; // restore normal input
            hal.stream.set_enqueue_rt_handler(stream.enqueue_realtime_command);
            stream.enqueue_realtime_command = NULL;
        }
    }

    return rxbuffer->tail != rxbuffer->head;
}

ISR_CODE bool stream_buffer_all (char c)
{
    return false;
}

ISR_CODE bool stream_enqueue_realtime_command (char c)
{
    return hal.stream.enqueue_rt_command ? hal.stream.enqueue_rt_command(c) : protocol_enqueue_realtime_command(c);
}

ISR_CODE bool stream_enable_mpg (const io_stream_t *mpg_stream, bool mpg_mode)
{
    static io_stream_t org_stream = {
        .type = StreamType_Redirected
    };

    sys_state_t state = state_get();

    // Deny entering MPG mode if busy
    if(mpg_mode == sys.mpg_mode || (mpg_mode && (gc_state.file_run || !(state == STATE_IDLE || (state & (STATE_ALARM|STATE_ESTOP)))))) {
        protocol_enqueue_realtime_command(CMD_STATUS_REPORT_ALL);
        return false;
    }

    if(mpg_mode) {
        if(org_stream.type == StreamType_Redirected) {
            memcpy(&org_stream, &hal.stream, sizeof(io_stream_t));
            if(hal.stream.disable_rx)
                hal.stream.disable_rx(true);
            mpg_stream->disable_rx(false);
            mpg_stream->set_enqueue_rt_handler(org_stream.set_enqueue_rt_handler(NULL));
            hal.stream.read = mpg_stream->read;
            hal.stream.get_rx_buffer_free = mpg_stream->get_rx_buffer_free;
            hal.stream.cancel_read_buffer = mpg_stream->cancel_read_buffer;
            hal.stream.reset_read_buffer = mpg_stream->reset_read_buffer;
        }
    } else if(org_stream.type != StreamType_Redirected) {
        mpg_stream->disable_rx(true);
        memcpy(&hal.stream, &org_stream, sizeof(io_stream_t));
        org_stream.type = StreamType_Redirected;
        if(hal.stream.disable_rx)
            hal.stream.disable_rx(false);
    }

    hal.stream.reset_read_buffer();

    sys.mpg_mode = mpg_mode;
    sys.report.mpg_mode = On;

    // Force a realtime status report, all reports when MPG mode active
    protocol_enqueue_realtime_command(mpg_mode ? CMD_STATUS_REPORT_ALL : CMD_STATUS_REPORT);

    return true;
}

#ifdef DEBUGOUT

static stream_write_ptr dbg_write = NULL;

static void debug_stream_warning (uint_fast16_t state)
{
    report_message("Failed to initialize debug stream!", Message_Warning);
}

void debug_write (const char *s)
{
    if(dbg_write) {
        dbg_write(s);
        while(hal.debug.get_tx_buffer_count()); // Wait until message is delivered
    }
}

static bool debug_claim_stream (io_stream_properties_t const *stream)
{
    io_stream_t const *claimed = NULL;

    if(stream->type == StreamType_Serial && stream->flags.claimable && !stream->flags.claimed) {

        if(stream->instance == DEBUGOUT && (claimed = stream->claim(115200))) {

            memcpy(&hal.debug, claimed, sizeof(io_stream_t));
            dbg_write = hal.debug.write;
            hal.debug.write = debug_write;

            if(hal.periph_port.set_pin_description)
                hal.periph_port.set_pin_description(Output_TX, hal.debug.instance == 0 ? PinGroup_UART : PinGroup_UART2, "Debug out");
        }
    }

    return claimed != NULL;
}

bool debug_stream_init (void)
{
    if(stream_enumerate_streams(debug_claim_stream))
        hal.debug.write(ASCII_EOL "UART debug active:" ASCII_EOL);
    else
        protocol_enqueue_rt_command(debug_stream_warning);

    return hal.debug.write == debug_write;
}

#endif
