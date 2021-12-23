/*
  stream.c - high level (serial) stream handling

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

#include <stdlib.h>
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

typedef struct stream_connection {
    const io_stream_t *stream;
    bool is_up;
    struct stream_connection *next;
} stream_connection_t;

static const io_stream_properties_t null_stream = {
    .type = StreamType_Null,
    .instance = 0,
    .flags.claimable = On,
    .flags.claimed = Off,
    .flags.connected = On,
    .flags.can_set_baud = On,
    .claim = stream_null_init
};

static io_stream_details_t null_streams = {
    .n_streams = 1,
    .streams = (io_stream_properties_t *)&null_stream,
};

static stream_state_t stream = {0};
static io_stream_details_t *streams = &null_streams;
static stream_connection_t base = {0}, *connections = &base;

void stream_register_streams (io_stream_details_t *details)
{
    details->next = streams;
    streams = details;
}

bool stream_enumerate_streams (stream_enumerate_callback_ptr callback)
{
    if(callback == NULL)
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

static void stream_write_all (const char *s)
{
    stream_connection_t *connection = connections;

    while(connection) {
        if(connection->is_up)
            connection->stream->write(s);
        connection = connection->next;
    }
}

static bool stream_select (const io_stream_t *stream, bool add)
{
    static const io_stream_t *active_stream = NULL;

    stream_connection_t *connection, *last = connections;

    if(stream == base.stream) {
        base.is_up = add;
        return true;
    }

    if(add) {

        if(base.stream == NULL) {
            base.stream = stream;
            base.is_up = stream->state.connected == On;
        } else if((connection = malloc(sizeof(stream_connection_t)))) {
            connection->stream = stream;
            connection->is_up = stream->state.connected == On || stream->state.is_usb == On; // TODO: add connect/disconnect event to driver code
            connection->next = NULL;
            while(last->next) {
                last = last->next;
                if(last->stream == stream) {
                    free(connection);
                    return true;
                }
            }
            last->next = connection;
        } else
            return false;

    } else { // disconnect

        stream_connection_t *prev;

        while(last->next) {
            prev = last;
            last = last->next;
            if(last->stream == stream) {
                prev->next = last->next;
                free(last);
                if(prev->next)
                    return false;
                else {
                    stream = prev->stream;
                    break;
                }
            }
        }
    }

    bool webui_connected = hal.stream.state.webui_connected;

    switch(stream->type) {

        case StreamType_Serial:
            if(active_stream && active_stream->type != StreamType_Serial && stream->state.connected) {
                hal.stream.write = stream->write;
                report_message("SERIAL STREAM ACTIVE", Message_Plain);
            }
            break;

        case StreamType_Telnet:
            if(hal.stream.state.connected)
                report_message("TELNET STREAM ACTIVE", Message_Plain);
            if(add && sys.driver_started) {
                hal.stream.write_all = stream->write;
                report_init_message();
            }
            break;

        case StreamType_WebSocket:
            if(hal.stream.state.connected)
                report_message("WEBSOCKET STREAM ACTIVE", Message_Plain);
            if(add && sys.driver_started && !hal.stream.state.webui_connected) {
                hal.stream.write_all = stream->write;
                report_init_message();
            }
            break;

        case StreamType_Bluetooth:
            if(hal.stream.state.connected)
                report_message("BLUETOOTH STREAM ACTIVE", Message_Plain);
            if(add && sys.driver_started) {
                hal.stream.write_all = stream->write;
                report_init_message();
            }
            break;

        default:
            break;
    }

    memcpy(&hal.stream, stream, sizeof(io_stream_t));

    if(!hal.stream.write_all)
        hal.stream.write_all = base.next != NULL ? stream_write_all : hal.stream.write;

    if(stream->type == StreamType_WebSocket)
        hal.stream.state.webui_connected = webui_connected;

    hal.stream.set_enqueue_rt_handler(protocol_enqueue_realtime_command);

    if(hal.stream.disable_rx)
        hal.stream.disable_rx(false);

    if(grbl.on_stream_changed)
        grbl.on_stream_changed(hal.stream.type);

    active_stream = stream;

    return true;
}

const io_stream_t *stream_get_base (void)
{
    return base.stream;
}

bool stream_connect (const io_stream_t *stream)
{
    bool ok = hal.stream_select ? hal.stream_select(stream) : stream_select(stream, true);

    if(ok && stream->type == StreamType_Serial && hal.periph_port.set_pin_description) {
        hal.periph_port.set_pin_description(Input_RX, (pin_group_t)(PinGroup_UART + stream->instance), "Primary UART");
        hal.periph_port.set_pin_description(Output_TX, (pin_group_t)(PinGroup_UART + stream->instance), "Primary UART");
    }

    return ok;
}

static struct {
    uint8_t instance;
    uint32_t baud_rate;
} connection;

static bool _connect_instance (io_stream_properties_t const *stream)
{
    io_stream_t const *claimed = NULL;

    if(stream->type == StreamType_Serial && stream->instance == connection.instance && stream->flags.claimable && !stream->flags.claimed)
        stream_connect(claimed = stream->claim(connection.baud_rate));

    return claimed != NULL;
}

bool stream_connect_instance (uint8_t instance, uint32_t baud_rate)
{
    connection.instance = instance;
    connection.baud_rate = baud_rate;

    return stream_enumerate_streams(_connect_instance);
}

void stream_disconnect (const io_stream_t *stream)
{
    if(hal.stream_select)
        hal.stream_select(NULL);
    else if(stream)
        stream_select(stream, false);
}

// null stream, discards output and returns no input

static enqueue_realtime_command_ptr enqueue_realtime_command = protocol_enqueue_realtime_command;

static uint16_t null_rx_free (void)
{
    return RX_BUFFER_SIZE;
}

static uint16_t null_count (void)
{
    return 0;
}

static bool null_put_c (const char c)
{
    return true;
}

static void null_write_string (const char *s)
{
}

static void null_write(const char *s, uint16_t length)
{
}

static bool null_suspend_disable (bool suspend)
{
    return true;
}

static bool null_set_baudrate (uint32_t baud_rate)
{
    return true;
}

static bool null_enqueue_rt_command (char c)
{
    return enqueue_realtime_command(c);
}

static enqueue_realtime_command_ptr null_set_rt_handler (enqueue_realtime_command_ptr handler)
{
    enqueue_realtime_command_ptr prev = enqueue_realtime_command;

    if(handler)
        enqueue_realtime_command = handler;

    return prev;
}

const io_stream_t *stream_null_init (uint32_t baud_rate)
{
    static const io_stream_t stream = {
        .type = StreamType_Null,
        .state.connected = On,
        .read = stream_get_null,
        .write = null_write_string,
        .write_n =  null_write,
        .write_char = null_put_c,
        .enqueue_rt_command = null_enqueue_rt_command,
        .get_rx_buffer_free = null_rx_free,
        .get_rx_buffer_count = null_count,
        .get_tx_buffer_count = null_count,
        .reset_write_buffer = dummy_handler,
        .reset_read_buffer = dummy_handler,
        .cancel_read_buffer = dummy_handler,
        .suspend_read = null_suspend_disable,
        .disable_rx = null_suspend_disable,
        .set_baud_rate = null_set_baudrate,
        .set_enqueue_rt_handler = null_set_rt_handler
    };

    return &stream;
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
