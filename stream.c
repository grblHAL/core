/*
  stream.c - high level (serial) stream handling

  Part of grblHAL

  Copyright (c) 2021-2025 Terje Io

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

#include <stdlib.h>
#include <string.h>

#include "hal.h"
#include "protocol.h"
#include "state_machine.h"

#if defined(DEBUG) || defined(DEBUGOUT)
#include <stdio.h>
#include <stdarg.h>
#ifndef DEBUG_BUFFER
#define DEBUG_BUFFER 100
#endif
#endif

static stream_rx_buffer_t rxbackup;

typedef struct {
    enqueue_realtime_command_ptr enqueue_realtime_command;
    stream_read_ptr read;
    stream_rx_buffer_t *rxbuffer;
} stream_state_t;

typedef union {
    uint8_t value;
    struct {
        uint8_t mpg_control :1,
                is_mpg_tx   :1,
                unused      :6;
    };
} stream_connection_flags_t;

typedef struct stream_connection {
    const io_stream_t *stream;
    stream_is_connected_ptr is_up;
    stream_connection_flags_t flags;
    struct stream_connection *next, *prev;
} stream_connection_t;

static const io_stream_properties_t null_stream = {
    .type = StreamType_Null,
    .instance = 0,
    .flags.claimable = On,
    .flags.claimed = Off,
    .flags.can_set_baud = On,
    .claim = stream_null_init
};

static io_stream_details_t null_streams = {
    .n_streams = 1,
    .streams = (io_stream_properties_t *)&null_stream,
};

static stream_state_t stream = {0};
static io_stream_details_t *streams = &null_streams;
static stream_connection_t base = {0}, mpg = {0}, *connections = &base;
static stream_write_char_ptr mpg_write_char = NULL;

void stream_register_streams (io_stream_details_t *details)
{
    if(details->n_streams) {
        details->next = streams;
        streams = details;
    }
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

ISR_CODE static bool ISR_FUNC(await_toolchange_ack)(char c)
{
    if(c == CMD_TOOL_ACK && !stream.rxbuffer->backup) {
        memcpy(&rxbackup, stream.rxbuffer, sizeof(stream_rx_buffer_t));
        stream.rxbuffer->backup = true;
        stream.rxbuffer->tail = stream.rxbuffer->head;
        hal.stream.read = stream.read; // restore normal input
        hal.stream.set_enqueue_rt_handler(stream.enqueue_realtime_command);
        stream.enqueue_realtime_command = NULL;
        if(grbl.on_toolchange_ack)
            grbl.on_toolchange_ack();
    } else
        return stream.enqueue_realtime_command(c);

    return true;
}

bool stream_rx_suspend (stream_rx_buffer_t *rxbuffer, bool suspend)
{
    if(suspend) {
        if(stream.rxbuffer == NULL) {
            stream.rxbuffer = rxbuffer;
            stream.read = hal.stream.read;
            stream.enqueue_realtime_command = hal.stream.set_enqueue_rt_handler(await_toolchange_ack);
            hal.stream.read = stream_get_null;
        }
    } else if(stream.rxbuffer) {
        if(rxbuffer->backup)
            memcpy(rxbuffer, &rxbackup, sizeof(stream_rx_buffer_t));
        if(stream.enqueue_realtime_command) {
            hal.stream.read = stream.read; // restore normal input
            hal.stream.set_enqueue_rt_handler(stream.enqueue_realtime_command);
            stream.enqueue_realtime_command = NULL;
        }
        stream.rxbuffer = NULL;
    }

    return rxbuffer->tail != rxbuffer->head;
}

ISR_CODE bool ISR_FUNC(stream_buffer_all)(char c)
{
    return false;
}

ISR_CODE bool ISR_FUNC(stream_enqueue_realtime_command)(char c)
{
	bool drop = hal.stream.enqueue_rt_command ? hal.stream.enqueue_rt_command(c) : protocol_enqueue_realtime_command(c);

    if(drop && (c == CMD_CYCLE_START || c == CMD_CYCLE_START_LEGACY))
        sys.report.cycle_start = settings.status_report.pin_state;

    return drop;
}

// helper function for (UART) stream implementations.

bool stream_connected (void)
{
    return true;
}

static bool is_not_connected (void)
{
    return false;
}

static bool connection_is_up (io_stream_t *stream)
{
    if(stream->is_connected)
        return stream->is_connected();

    stream_connection_t *connection = connections;

    while(connection) {
        if(connection->stream->type == stream->type &&
            connection->stream->instance == stream->instance &&
             connection->stream->state.is_usb == stream->state.is_usb) {

            if(connection->stream->state.is_usb)
                connection->is_up = is_not_connected;

            return connection->is_up();
        }
        connection = connection->next;
    }

    return false;
}

static void stream_write_all (const char *s)
{
    stream_connection_t *connection = connections;

    while(connection) {
        if(connection->is_up())
            connection->stream->write(s);
        connection = connection->next;
    }
}

static stream_connection_t *add_connection (const io_stream_t *stream)
{
    stream_connection_t *connection, *last = connections;

    if(base.stream == NULL) {
        base.stream = stream;
        connection = &base;
    } else if((connection = malloc(sizeof(stream_connection_t)))) {
        connection->stream = stream;
        connection->next = NULL;
        while(last->next) {
            last = last->next;
            if(last->stream == stream) {
                free(connection);
                return NULL;
            }
        }
        connection->prev = last;
        last->next = connection;
    }

    connection->is_up = stream->is_connected ?
                         stream->is_connected :
                          (stream->state.is_usb && base.stream != stream ? is_not_connected : stream_connected);

    return connection;
}

static bool stream_select (const io_stream_t *stream, bool add)
{
    static const io_stream_t *active_stream = NULL;

    bool send_init_message = false, mpg_enable = false;
    static struct {
        const io_stream_t *stream;
        on_linestate_changed_ptr on_linestate_changed;
    } usb = {};

    if(stream == base.stream) {
        base.is_up = add ? (stream->is_connected ? stream->is_connected : stream_connected) : is_not_connected;
        return true;
    }

    if(active_stream != NULL && hal.stream.state.is_usb) {
        usb.stream = active_stream;
        usb.on_linestate_changed = hal.stream.on_linestate_changed;
    }

    if(!add) { // disconnect

        if(stream == base.stream || stream == mpg.stream)
        	return false;

        bool disconnected = false;
        stream_connection_t *connection = connections->next;

        while(connection) {
        	if(stream == connection->stream) {
        		if((connection->prev->next = connection->next))
        			connection->next->prev = connection->prev;
                if((stream = connection->prev->stream) == mpg.stream) {
                	mpg_enable = mpg.flags.mpg_control;
                	if((stream = connection->prev->prev->stream) == NULL)
                		stream = base.stream;
                }
                free(connection);
        		connection = NULL;
        		disconnected = true;
        	} else
        		connection = connection->next;
        }

        if(!disconnected)
        	return false;

	} else if(add_connection(stream) == NULL)
        return false;

    bool webui_connected = hal.stream.state.webui_connected;

    switch(stream->type) {

        case StreamType_Serial:
            if(active_stream && active_stream->type != StreamType_Serial && connection_is_up((io_stream_t *)stream)) {
                hal.stream.write = stream->write;
                report_message("SERIAL STREAM ACTIVE", Message_Plain);
                if(stream->get_tx_buffer_count)
                    while(stream->get_tx_buffer_count());
                else
                    hal.delay_ms(100, NULL);
            }
            break;

        case StreamType_Telnet:
            if(connection_is_up(&hal.stream))
                report_message("TELNET STREAM ACTIVE", Message_Plain);
            send_init_message = add && sys.driver_started;
            break;

        case StreamType_WebSocket:
            if(connection_is_up(&hal.stream))
                report_message("WEBSOCKET STREAM ACTIVE", Message_Plain);
            send_init_message = add && sys.driver_started && !hal.stream.state.webui_connected;
            break;

        case StreamType_Bluetooth:
            if(connection_is_up(&hal.stream))
                report_message("BLUETOOTH STREAM ACTIVE", Message_Plain);
            send_init_message = add && sys.driver_started;
            break;

        default:
            break;
    }

    if(hal.stream.type == StreamType_MPG) {
        stream_mpg_enable(false);
        mpg.flags.mpg_control = On;
    } else if(mpg_enable)
		protocol_enqueue_foreground_task(stream_mpg_set_mode, (void *)1);

    memcpy(&hal.stream, stream, sizeof(io_stream_t));

    if(stream == usb.stream)
        hal.stream.on_linestate_changed = usb.on_linestate_changed;

    if(stream == base.stream && base.is_up == is_not_connected)
        base.is_up = stream_connected;

    if(hal.stream.is_connected == NULL)
        hal.stream.is_connected = stream == base.stream ? base.is_up : stream_connected;

    if(stream->type == StreamType_WebSocket && !stream->state.webui_connected)
        hal.stream.state.webui_connected = webui_connected;

    if(send_init_message)
        grbl.report.init_message(stream->write);

    hal.stream.write_all = stream_write_all;
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

io_stream_flags_t stream_get_flags (io_stream_t stream)
{
    io_stream_flags_t flags = {0};
    io_stream_details_t *details = streams;

    while(details) {
        uint_fast8_t idx;
        for(idx = 0; idx < details->n_streams; idx++) {
            if(stream.type == details->streams[idx].type && stream.instance == details->streams[idx].instance) {
                flags = details->streams[idx].flags;
                break;
            }
        }
        details = details->next;
    };

    return flags;
}

bool stream_set_description (const io_stream_t *stream, const char *description)
{
    bool ok;

    if((ok = stream->type == StreamType_Serial && !stream->state.is_usb && hal.periph_port.set_pin_description)) {
        hal.periph_port.set_pin_description(Output_TX, (pin_group_t)(PinGroup_UART + stream->instance), description);
        hal.periph_port.set_pin_description(Input_RX, (pin_group_t)(PinGroup_UART + stream->instance), description);
    }

    return ok;
}

bool stream_connect (const io_stream_t *stream)
{
    bool ok;

    if((ok = stream_select(stream, true)))
        stream_set_description(stream, "Primary UART");

    return ok;
}

static struct {
    uint8_t instance;
    uint32_t baud_rate;
    io_stream_t const *stream;
} connection;

static bool _open_instance (io_stream_properties_t const *stream)
{
    if(stream->type == StreamType_Serial && (connection.instance == 255 || stream->instance == connection.instance) && stream->flags.claimable && !stream->flags.claimed)
        connection.stream = stream->claim(connection.baud_rate);

    return connection.stream != NULL;
}

bool stream_connect_instance (uint8_t instance, uint32_t baud_rate)
{
    connection.instance = instance;
    connection.baud_rate = baud_rate;
    connection.stream = NULL;

    return stream_enumerate_streams(_open_instance) && stream_connect(connection.stream);
}

void stream_disconnect (const io_stream_t *stream)
{
    if(stream)
        stream_select(stream, false);
}

io_stream_t const *stream_open_instance (uint8_t instance, uint32_t baud_rate, stream_write_char_ptr rx_handler, const char *description)
{
    connection.instance = instance;
    connection.baud_rate = baud_rate;
    connection.stream = NULL;

    if(stream_enumerate_streams(_open_instance)) {
        connection.stream->set_enqueue_rt_handler(rx_handler);
        if(description)
            stream_set_description(connection.stream, description);
    }

    return connection.stream;
}

// MPG stream

void stream_mpg_set_mode (void *data)
{
    stream_mpg_enable(data != NULL);
}

ISR_CODE bool ISR_FUNC(stream_mpg_check_enable)(char c)
{
    if(c == CMD_MPG_MODE_TOGGLE)
    	task_add_immediate(stream_mpg_set_mode, (void *)1);
    else {
        protocol_enqueue_realtime_command(c);
        if((c == CMD_CYCLE_START || c == CMD_CYCLE_START_LEGACY) && settings.status_report.pin_state)
            sys.report.cycle_start |= state_get() == STATE_IDLE;
    }

    return true;
}

bool stream_mpg_register (const io_stream_t *stream, bool rx_only, stream_write_char_ptr write_char)
{
    if(stream == NULL || stream->type != StreamType_Serial || stream->disable_rx == NULL)
        return false;

//    base.flags.is_up = On;

    mpg_write_char = write_char;

    if(stream->write == NULL || rx_only) {

        mpg.stream = stream;
        mpg.is_up = stream_connected;

        if(hal.periph_port.set_pin_description)
            hal.periph_port.set_pin_description(Input_RX, (pin_group_t)(PinGroup_UART + stream->instance), "MPG");

        return true;
    }

    stream_connection_t *connection = add_connection(stream);

    if(connection) {

        memcpy(&mpg, connection, sizeof(stream_connection_t));

        mpg.flags.is_mpg_tx = On;
        mpg.flags.mpg_control = Off;

        if(mpg_write_char)
            mpg.stream->set_enqueue_rt_handler(mpg_write_char);
        else
            mpg.stream->disable_rx(true);

        hal.stream.write_all = stream_write_all;

        stream_set_description(stream, "MPG");
    }

    return connection != NULL;
}

bool stream_mpg_enable (bool on)
{
    static io_stream_t org_stream = {
        .type = StreamType_Redirected
    };

    if(mpg.stream == NULL)
        return false;

    sys_state_t state = state_get();

    // Deny entering MPG mode if busy
    if(on == sys.mpg_mode || (on && (gc_state.file_run || !(state == STATE_IDLE || (state & (STATE_ALARM|STATE_ESTOP)))))) {
        protocol_enqueue_realtime_command(CMD_STATUS_REPORT_ALL);
        return false;
    }

    if(on) {
        if(org_stream.type == StreamType_Redirected) {
            memcpy(&org_stream, &hal.stream, sizeof(io_stream_t));
            if(hal.stream.disable_rx)
                hal.stream.disable_rx(true);
            mpg.stream->disable_rx(false);
            mpg.stream->set_enqueue_rt_handler(org_stream.set_enqueue_rt_handler(NULL));
            hal.stream.type = StreamType_MPG;
            hal.stream.read = mpg.stream->read;
            if(mpg.flags.is_mpg_tx)
                hal.stream.write = mpg.stream->write;
            hal.stream.get_rx_buffer_free = mpg.stream->get_rx_buffer_free;
            hal.stream.cancel_read_buffer = mpg.stream->cancel_read_buffer;
            hal.stream.reset_read_buffer = mpg.stream->reset_read_buffer;
        }
    } else if(org_stream.type != StreamType_Redirected) {
        if(mpg_write_char)
            mpg.stream->set_enqueue_rt_handler(mpg_write_char);
        else
            mpg.stream->disable_rx(true);
        memcpy(&hal.stream, &org_stream, sizeof(io_stream_t));
        org_stream.type = StreamType_Redirected;
        if(hal.stream.disable_rx)
            hal.stream.disable_rx(false);
    }

    hal.stream.reset_read_buffer();

    sys.mpg_mode = on;
    mpg.flags.mpg_control = Off;
    system_add_rt_report(Report_MPGMode);

    // Force a realtime status report, all reports when MPG mode active
    protocol_enqueue_realtime_command(on ? CMD_STATUS_REPORT_ALL : CMD_STATUS_REPORT);

    return true;
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
        .is_connected = stream_connected,
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

#if DEBUGOUT == -1

__attribute__((weak)) void debug_write (const char *s)
{
    // NOOP
}

void debug_writeln (const char *s)
{
    debug_write(s);
    debug_write(ASCII_EOL);
}

void debug_printf (const char *fmt, ...)
{
    char debug_out[DEBUG_BUFFER];

    va_list args;
    va_start(args, fmt);
    vsnprintf(debug_out, sizeof(debug_out) - 1, fmt, args);
    va_end(args);

    debug_writeln(debug_out);
}

bool debug_stream_init (void)
{
    return true;
}

#else

static stream_write_ptr dbg_write = NULL;

void debug_write (const char *s)
{
    if(dbg_write) {
        dbg_write(s);
        while(hal.debug.get_tx_buffer_count()) // Wait until message is delivered
            grbl.on_execute_realtime(state_get());
    }
}

void debug_writeln (const char *s)
{
    if(dbg_write) {
        dbg_write(s);
        dbg_write(ASCII_EOL);
        while(hal.debug.get_tx_buffer_count()) // Wait until message is delivered
            grbl.on_execute_realtime(state_get());
    }
}

void debug_printf (const char *fmt, ...)
{
    char debug_out[DEBUG_BUFFER];

    va_list args;
    va_start(args, fmt);
    vsnprintf(debug_out, sizeof(debug_out) - 1, fmt, args);
    va_end(args);

    debug_writeln(debug_out);
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
                hal.periph_port.set_pin_description(Output_TX, (pin_group_t)(PinGroup_UART + hal.debug.instance), "Debug out");
        }
    }

    return claimed != NULL;
}

bool debug_stream_init (void)
{
    if(stream_enumerate_streams(debug_claim_stream))
        hal.debug.write(ASCII_EOL "UART debug active:" ASCII_EOL);
    else
        protocol_enqueue_foreground_task(report_warning, "Failed to initialize debug stream!");

    return hal.debug.write == debug_write;
}

#endif // DEBUGOUT

#elif defined(DEBUG)

void debug_printf (const char *fmt, ...)
{
    char debug_out[DEBUG_BUFFER];

    va_list args;
    va_start(args, fmt);
    vsnprintf(debug_out, sizeof(debug_out) - 1, fmt, args);
    va_end(args);

    if(hal.stream.write) {
        report_message(debug_out, Message_Debug);
        if(hal.stream.get_tx_buffer_count) {
            while(hal.stream.get_tx_buffer_count()) // Wait until message is delivered
                grbl.on_execute_realtime(state_get());
        }
    }
}

#else

void debug_printf (const char *fmt, ...)
{
    // NOOP
}

#endif
