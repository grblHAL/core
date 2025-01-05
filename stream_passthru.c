/*
  stream_passthru.c - stream redirector for programming coprocessor

  Part of grblHAL

  Copyright (c) 2024-2025 Terje Io

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

#include "hal.h"
#include "task.h"
#include "protocol.h"

//static xbar_t *dtr, *rts;
static uint8_t boot0_port = 0xFF, reset_port = 0xFF;
static io_stream_t dest;
static bool conn_ok = false;
static on_linestate_changed_ptr on_linestate_changed;
//static on_execute_realtime_ptr on_execute_realtime = NULL;

// Weak implementation of low level function to be provided by the driver

__attribute__((weak)) void stream_passthru_enter (void)
{
    // The driver implementation calls stream_passthru_init() with start parameter set true after setting up for execution.
}

// ****


/*
static void onExecuteRealtime (uint_fast16_t state)
{
    static bool lock = false;

    int16_t c;

    if(!lock) {
        lock = true;
        if((c = hal.stream.read()) != SERIAL_NO_DATA)
            dest.write_char(c);
        lock = false;
    }

    on_execute_realtime(state);
}
*/

ISR_CODE static bool ISR_FUNC(forward_usb_rx)(char c)
{
    dest.write_char(c);

    return true;
}

ISR_CODE static bool ISR_FUNC(sink_uart_rx)(char c)
{
    return true;
}

static void forward_uart_rx (void *data)
{
    static char buf[64];
    static uint_fast8_t idx = 0;

    int16_t c;

    while((c = dest.read()) != SERIAL_NO_DATA) {

        idx = 1;
        *buf = c;

        while(idx < sizeof(buf) && (c = dest.read()) != SERIAL_NO_DATA)
            buf[idx++] = c;

        hal.stream.write_n(buf, idx);
    }

    task_add_delayed(forward_uart_rx, NULL, 8);
}

static void onLinestateChanged (serial_linestate_t state)
{
    /*
    Auto program
    DTR RTS-->EN IO0
    1 1 1 1
    0 0 1 1
    1 0 0 1
    0 1 1 0
     */
    if(conn_ok) {

        if(state.dtr == state.rts) {
            hal.port.digital_out(boot0_port, 0);
            hal.port.digital_out(reset_port, 1);
        } else if(state.dtr) {
            hal.port.digital_out(reset_port, 0);
            hal.port.digital_out(boot0_port, 0);
        } else {
            hal.port.digital_out(boot0_port, 0);
            hal.port.digital_out(reset_port, 1);
        }
    }
}

static void passthru_start2 (void *data)
{
    conn_ok = true;
    hal.port.digital_out(reset_port, 1);

    task_add_delayed(forward_uart_rx, NULL, 8);

//    on_execute_realtime = grbl.on_execute_realtime;
//    grbl.on_execute_realtime = onExecuteRealtime;

    dest.set_enqueue_rt_handler(stream_buffer_all);
    dest.cancel_read_buffer();
}

static void passthru_start1 (void *data)
{
    hal.port.digital_out(boot0_port, 0);
    hal.port.digital_out(reset_port, 0);

    on_linestate_changed = hal.stream.on_linestate_changed;
    hal.stream.on_linestate_changed = onLinestateChanged;

    task_add_delayed(passthru_start2, NULL, 1250); // delay a bit to allow the USB stack to start
}

static bool get_ports (xbar_t *properties, uint8_t port, void *data)
{
    switch(properties->function) {

        case Output_CoProc_Reset:
            reset_port = port;
            break;

        case Output_CoProc_Boot0:
            boot0_port = port;
            break;

        default:
            break;
    }

    return boot0_port != 0xFF && reset_port != 0xFF;
}

static status_code_t passthru_enter (sys_state_t state, char *args)
{
    report_message("Entering passthru mode", Message_Warning);
    hal.delay_ms(100, NULL);

    stream_passthru_enter();

    return Status_OK;
}

void stream_passthru_init (uint8_t instance, uint32_t baud_rate, bool start)
{
    static const sys_command_t passthru_command_list[] = {
        {"PTRGH", passthru_enter, { .noargs = On }, { .str = "enter passthru mode" } }
    };

    static sys_commands_t passthru_commands = {
        .n_commands = sizeof(passthru_command_list) / sizeof(sys_command_t),
        .commands = passthru_command_list
    };

    if(hal.stream.type == StreamType_Serial &&
        hal.stream.state.is_usb &&
         hal.stream.state.linestate_event &&
          ioports_enumerate(Port_Digital, Port_Output, (pin_cap_t){ .output = On }, get_ports, NULL)) {

        if(start) {

            io_stream_t const *stream = stream_open_instance(instance, baud_rate, NULL, "Passthru");

            if((hal.stream.state.passthru = stream != NULL)) {

                protocol_enqueue_foreground_task(passthru_start1, NULL); // enter passthrouh mode after finished booting grblHAL

                memcpy(&dest, stream, sizeof(io_stream_t));
                dest.set_enqueue_rt_handler(sink_uart_rx);
                hal.stream.set_enqueue_rt_handler(forward_usb_rx);
            } else
                report_message("Entering passthru mode failed!", Message_Warning);

        } else
            system_register_commands(&passthru_commands);
    }
}
