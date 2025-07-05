/*

  modbus_rtu.c - a lightweight ModBus RTU implementation

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


#include <stdio.h>
#include <string.h>

#include "hal.h"
#include "platform.h"
#include "protocol.h"
#include "settings.h"
#include "crc.h"
#include "nvs_buffer.h"
#include "state_machine.h"
#include "modbus.h"

#ifndef MODBUS_BAUDRATE
#define MODBUS_BAUDRATE 3 // 19200
#endif

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

typedef struct queue_entry {
    bool async;
    modbus_message_t msg;
    modbus_callbacks_t callbacks;
    struct queue_entry *next;
} queue_entry_t;

static const uint32_t baud[] = { 2400, 4800, 9600, 19200, 38400, 115200 };
static const modbus_silence_timeout_t dflt_timeout =
{
    .b2400   = 16,
    .b4800   = 8,
    .b9600   = 4,
    .b19200  = 2,
    .b38400  = 2,
    .b115200 = 2
};

static modbus_stream_t stream;
static int8_t stream_instance = -1;
static uint32_t rx_timeout = 0, silence_until = 0, silence_timeout;
static int16_t exception_code = 0;
static modbus_silence_timeout_t silence;
static queue_entry_t queue[MODBUS_QUEUE_LENGTH];
static modbus_settings_t modbus;
static volatile bool spin_lock = false, is_up = false;
static volatile queue_entry_t *tail, *head, *packet = NULL;
static volatile modbus_state_t state = ModBus_Idle;
static uint8_t dir_port = IOPORT_UNASSIGNED;

static struct {
    uint32_t tx_count;
    uint32_t retries;
    uint32_t timeouts;
    uint32_t crc_errors;
    uint32_t rx_exceptions;
} stats = {};

static driver_reset_ptr driver_reset;
static on_report_options_ptr on_report_options;
static nvs_address_t nvs_address;

/*
static bool valid_crc (const char *buf, uint_fast16_t len)
{
    uint16_t crc = modbus_crc16x(buf, len - 2);

    return buf[len - 1] == (crc >> 8) && buf[len - 2] == (crc & 0xFF);
}
*/

#if DEBUG
static void log_modbus_frame(const char *label,
                             const uint8_t *data,
                             uint8_t len)
{
    char frame[3 * MODBUS_MAX_ADU_SIZE + 4];
    char *ptr = frame;
    ptr += sprintf(ptr, "%s:", label);
    for(uint_fast8_t i = 0; i < len; i++) {
        ptr += sprintf(ptr, " %02X", data[i]);
    }
    debug_printf("%s", frame);
}
#endif

static void retry_exception (uint8_t code, void *context)
{
    if(packet && packet->callbacks.retries) {
        state = ModBus_Retry;
        silence_until = hal.get_elapsed_ticks() + silence_timeout + packet->callbacks.retry_delay;
        stats.retries++;
    }
}

static inline queue_entry_t *add_message (queue_entry_t *packet, modbus_message_t *msg, bool async, const modbus_callbacks_t *callbacks)
{
    stats.tx_count++;

    memcpy(&packet->msg, msg, sizeof(modbus_message_t));

    packet->async = async;

    if(callbacks && !sys.reset_pending) {
        memcpy(&packet->callbacks, callbacks, sizeof(modbus_callbacks_t));
        if(!packet->async && packet->callbacks.retries)
            packet->callbacks.on_rx_exception = retry_exception;
    } else {
        packet->callbacks.retries = 0;
        packet->callbacks.on_rx_packet = NULL;
        packet->callbacks.on_rx_exception = NULL;
    }

    return packet;
}

static void tx_message (volatile queue_entry_t *msg)
{
    if(stream.set_direction)
        stream.set_direction(true);

    packet = msg;
    state = ModBus_TX;
    rx_timeout = modbus.rx_timeout;

    stream.flush_rx_buffer();
#if DEBUG
    log_modbus_frame("TX", (const uint8_t *)msg->msg.adu, msg->msg.tx_length);
#endif
    stream.write((char *)((queue_entry_t *)msg)->msg.adu, ((queue_entry_t *)msg)->msg.tx_length);
}

// called once every ms
static void modbus_poll (void *data)
{
    if(spin_lock)
        return;

    spin_lock = true;

    switch(state) {

        case ModBus_Idle:
            if(tail != head && !packet) {
                tx_message(tail);
                tail = tail->next;
            }
            break;

        case ModBus_Silent:
            if((int32_t)(silence_until - hal.get_elapsed_ticks()) <= 0) {
                silence_until = 0;
                state = ModBus_Idle;
            }
            break;

        case ModBus_TX:
            if(!stream.get_tx_buffer_count()) {

                // When an auto-direction sense circuit supports higher baudrates is used at slower rates, it can switch during the off time (TXD is high) of some bit sequences.
                // In some cases (teensy4.1) this can result in garbage characters in the RX buffer after a message is transmitted.
                // Flushing the buffer prevents these characters from appearing as an RX message.
                // Since Modbus is half-duplex, there should never be valid data recived during a message transmit.
                stream.flush_rx_buffer();
                
                state = ModBus_AwaitReply;

                if(stream.set_direction)
                    stream.set_direction(false);
            }
            break;

        case ModBus_AwaitReply:
            if(rx_timeout && --rx_timeout == 0) {

                stats.timeouts++;

                if(packet->async) {
                    state = ModBus_Silent;
                    if(packet->callbacks.on_rx_exception)
                        packet->callbacks.on_rx_exception(0, packet->msg.context);
                    packet = NULL;
                } else if(stream.read() == packet->msg.adu[0] && (stream.read() & 0x80)) {
                    exception_code = stream.read();
                    state = ModBus_Exception;
                    stats.rx_exceptions++;
                } else
                    state = ModBus_Timeout;

                spin_lock = false;
                if(state != ModBus_AwaitReply)
                    silence_until = hal.get_elapsed_ticks() + silence_timeout;
                return;
            }

            if(stream.get_rx_buffer_count() >= packet->msg.rx_length) {

                char *buf = (char *)((queue_entry_t *)packet)->msg.adu;
                uint16_t rx_len = packet->msg.rx_length; // store original length for CRC check

                do {
                    *buf++ = stream.read();
                } while(--packet->msg.rx_length);

                if(packet->msg.crc_check) {

                    uint_fast16_t crc = modbus_crc16x(((queue_entry_t *)packet)->msg.adu, rx_len - 2);
                    if(packet->msg.adu[rx_len - 2] != (crc & 0xFF) || packet->msg.adu[rx_len - 1] != (crc >> 8)) {
                        // CRC check error
                        stats.crc_errors++;
                        if((state = packet->async ? ModBus_Silent : ModBus_Exception) == ModBus_Silent) {
                            if(packet->callbacks.on_rx_exception)
                                packet->callbacks.on_rx_exception(0, packet->msg.context);
                            packet = NULL;
                        }
                        silence_until = hal.get_elapsed_ticks() + silence_timeout;
                        break;
                    }
                }

#if DEBUG
                log_modbus_frame("RX", (const uint8_t *)((queue_entry_t *)packet)->msg.adu, rx_len);
#endif
                if((state = packet->async ? ModBus_Silent : ModBus_GotReply) == ModBus_Silent) {
                    if(packet->callbacks.on_rx_packet) {
                        packet->msg.rx_length = rx_len;
                        packet->callbacks.on_rx_packet(&((queue_entry_t *)packet)->msg);
                    }
                    packet = NULL;
                }

                silence_until = hal.get_elapsed_ticks() + silence_timeout;
            }
            break;

        case ModBus_Timeout:
            if(packet->async)
                state = ModBus_Silent;
            silence_until = hal.get_elapsed_ticks() + silence_timeout;
            break;

        default:
            break;
    }

    spin_lock = false;
}

static bool modbus_send_rtu (modbus_message_t *msg, const modbus_callbacks_t *callbacks, bool block)
{
    static bool poll = false;
    static queue_entry_t sync_msg = {0};

    if(msg->tx_length > MODBUS_MAX_ADU_SIZE || msg->rx_length > MODBUS_MAX_ADU_SIZE) {
        if(callbacks->on_rx_exception)
            callbacks->on_rx_exception(0, msg->context);
        return false;
    }

    uint_fast16_t crc = modbus_crc16x(msg->adu, msg->tx_length - 2);

    msg->adu[msg->tx_length - 1] = crc >> 8;
    msg->adu[msg->tx_length - 2] = crc & 0xFF;

    while(spin_lock);

    if(block) {

        if(poll)
            return false;

        poll = true;

        do {
            grbl.on_execute_realtime(state_get());
        } while(state != ModBus_Idle);

        tx_message(add_message(&sync_msg, msg, false, callbacks));

        while(poll) {

            grbl.on_execute_realtime(state_get());

            switch(state) {

                case ModBus_Timeout:
                    if(packet->callbacks.on_rx_exception)
                        packet->callbacks.on_rx_exception(0, packet->msg.context);
                    poll = packet->callbacks.retries > 0;
                    break;

                case ModBus_Exception:
                    if(packet->callbacks.on_rx_exception)
                        packet->callbacks.on_rx_exception(exception_code == -1 ? 0 : (uint8_t)(exception_code & 0xFF), packet->msg.context);
                    poll = packet->callbacks.retries > 0;
                    break;

                case ModBus_GotReply:
                    if(packet->callbacks.on_rx_packet)
                        packet->callbacks.on_rx_packet(&((queue_entry_t *)packet)->msg);
                    poll = block = false;
                    break;

                case ModBus_Retry:
                    if((int32_t)(silence_until - hal.get_elapsed_ticks()) <= 0) {
                        silence_until = 0;
                        if(--packet->callbacks.retries == 0)
                            packet->callbacks.on_rx_exception = callbacks->on_rx_exception;
                        packet = add_message(&sync_msg, msg, false, (const modbus_callbacks_t *)&packet->callbacks);
                        tx_message(packet);
                    }
                    break;

                default:
                    break;
            }
        }

        poll = false;
        packet = NULL;
        state = silence_until > 0 ? ModBus_Silent : ModBus_Idle;

    } else if(packet != &sync_msg) {
        if(head->next != tail) {
            add_message((queue_entry_t *)head, msg, true, callbacks);
            head = head->next;
        }
    }

    return !block;
}

static void modbus_reset (void)
{
    while(spin_lock);

    if(sys.abort) {

        if(packet) {
            packet = NULL;
            packet->callbacks.retries = 0;
            packet->callbacks.on_rx_exception = NULL;
        }

        tail = head;
        silence_until = hal.get_elapsed_ticks() + 500;
        state = ModBus_Silent;

        stream.flush_tx_buffer();
        stream.flush_rx_buffer();
    }

    if(state == ModBus_Retry) {
        silence_until = hal.get_elapsed_ticks() + 500;
        state = ModBus_Silent;
    }

    driver_reset();
}

static uint32_t get_baudrate (uint32_t rate)
{
    uint32_t idx = sizeof(baud) / sizeof(uint32_t);

    do {
        if(baud[--idx] == rate)
            return idx;
    } while(idx);

    return MODBUS_BAUDRATE;
}

static const setting_group_detail_t modbus_groups [] = {
    { Group_Root, Group_ModBus, "ModBus"}
};

static status_code_t modbus_set_baud (setting_id_t id, uint_fast16_t value)
{
    modbus.baud_rate = baud[(uint32_t)value];
    silence_timeout = silence.timeout[(uint32_t)value];
    stream.set_baud_rate(modbus.baud_rate);

    return Status_OK;
}

static uint32_t modbus_get_baud (setting_id_t setting)
{
    return get_baudrate(modbus.baud_rate);
}

static const setting_detail_t modbus_settings[] = {
    { Settings_ModBus_BaudRate, Group_ModBus, "ModBus baud rate", NULL, Format_RadioButtons, "2400,4800,9600,19200,38400,115200", NULL, NULL, Setting_NonCoreFn, modbus_set_baud, modbus_get_baud, NULL },
    { Settings_ModBus_RXTimeout, Group_ModBus, "ModBus RX timeout", "milliseconds", Format_Integer, "####0", "50", "250", Setting_NonCore, &modbus.rx_timeout, NULL, NULL }
};

static void modbus_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&modbus, sizeof(modbus_settings_t), true);
}

static void modbus_settings_restore (void)
{
    modbus.rx_timeout = 50;
    modbus.baud_rate = baud[MODBUS_BAUDRATE];

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&modbus, sizeof(modbus_settings_t), true);
}

static void modbus_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&modbus, nvs_address, sizeof(modbus_settings_t), true) != NVS_TransferResult_OK ||
         modbus.baud_rate != baud[get_baudrate(modbus.baud_rate)])
        modbus_settings_restore();

    is_up = true;
    silence_timeout = silence.timeout[get_baudrate(modbus.baud_rate)];

    stream.set_baud_rate(modbus.baud_rate);
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("MODBUS", "0.19");
}

static bool modbus_rtu_isup (void)
{
    return is_up;
}

static bool modbus_is_busy (void)
{
    return state != STATE_IDLE;
}

static void modbus_rtu_flush_queue (void)
{
    while(spin_lock);

    tail = head;
}

static void modbus_rtu_set_silence (const modbus_silence_timeout_t *timeout)
{
    if(timeout)
        memcpy(&silence, timeout, sizeof(modbus_silence_timeout_t));
    else
        memcpy(&silence, &dflt_timeout, sizeof(modbus_silence_timeout_t));

    silence_timeout = silence.timeout[get_baudrate(modbus.baud_rate)];
}

static bool stream_is_valid (const io_stream_t *stream)
{
    return stream &&
            !(stream->set_baud_rate == NULL ||
               stream->get_tx_buffer_count == NULL ||
                stream->get_rx_buffer_count == NULL ||
                 stream->write_n == NULL ||
                  stream->read == NULL ||
                   stream->reset_write_buffer == NULL ||
                    stream->reset_read_buffer == NULL ||
                     stream->set_enqueue_rt_handler == NULL);
}

static void modbus_set_direction (bool tx)
{
    ioport_digital_out(dir_port, tx);
}

static bool claim_stream (io_stream_properties_t const *sstream)
{
    io_stream_t const *claimed = NULL;

    if(sstream->type == StreamType_Serial && (stream_instance >= 0
                                               ? sstream->instance == (uint8_t)stream_instance
                                               : sstream->flags.modbus_ready && !sstream->flags.claimed)) {
        if((claimed = sstream->claim(baud[MODBUS_BAUDRATE])) && stream_is_valid(claimed)) {

            claimed->set_enqueue_rt_handler(stream_buffer_all);

            stream.set_baud_rate = claimed->set_baud_rate;
            stream.get_tx_buffer_count = claimed->get_tx_buffer_count;
            stream.get_rx_buffer_count = claimed->get_rx_buffer_count;
            stream.write = claimed->write_n;
            stream.read = claimed->read;
            stream.flush_tx_buffer = claimed->reset_write_buffer;
            stream.flush_rx_buffer = claimed->reset_read_buffer;
            stream.set_direction = dir_port != IOPORT_UNASSIGNED ? modbus_set_direction : NULL;

            if(hal.periph_port.set_pin_description) {
                hal.periph_port.set_pin_description(Output_TX, (pin_group_t)(PinGroup_UART + claimed->instance), "Modbus");
                hal.periph_port.set_pin_description(Input_RX, (pin_group_t)(PinGroup_UART + claimed->instance), "Modbus");
            }
        } else
            claimed = NULL;
    }

    return claimed != NULL;
}
static status_code_t report_stats (sys_state_t state, char *args)
{
    char buf[110];

    snprintf(buf, sizeof(buf) - 1, "TX: " UINT32FMT ", retries: " UINT32FMT ", timeouts: " UINT32FMT ", RX exceptions: " UINT32FMT ", CRC errors: " UINT32FMT,
              stats.tx_count, stats.retries, stats.timeouts, stats.rx_exceptions, stats.crc_errors);

    report_message(buf, Message_Info);

    if(args && (*args == 'r' || *args == 'R'))
        stats.tx_count = stats.retries = stats.timeouts = stats.rx_exceptions = stats.crc_errors = 0;

    return Status_OK;
}

void modbus_rtu_init (int8_t stream, int8_t dir_aux)
{
    static const modbus_api_t api = {
        .interface = Modbus_InterfaceRTU,
        .is_up = modbus_rtu_isup,
        .flush_queue = modbus_rtu_flush_queue,
        .set_silence = modbus_rtu_set_silence,
        .send = modbus_send_rtu,
        .is_busy = modbus_is_busy
    };

    static setting_details_t setting_details = {
        .groups = modbus_groups,
        .n_groups = sizeof(modbus_groups) / sizeof(setting_group_detail_t),
        .settings = modbus_settings,
        .n_settings = sizeof(modbus_settings) / sizeof(setting_detail_t),
        .save = modbus_settings_save,
        .load = modbus_settings_load,
        .restore = modbus_settings_restore
    };

    static const sys_command_t command_list[] = {
        {"MODBUSSTATS", report_stats, { .allow_blocking = On }, { .str = "output Modbus RTU statistics" } },
    };

    static sys_commands_t commands = {
        .n_commands = sizeof(command_list) / sizeof(sys_command_t),
        .commands = command_list
    };

    if(dir_aux != -2) {

        int8_t n_out = ioports_available(Port_Digital, Port_Output);

        dir_port = dir_aux != -1 ? dir_aux : (n_out ? n_out - 1 : IOPORT_UNASSIGNED);

        if(!(dir_port != IOPORT_UNASSIGNED && ioport_claim(Port_Digital, Port_Output, &dir_port, "Modbus RX/TX direction"))) {
            task_run_on_startup(report_warning, "Modbus failed to initialize!");
            system_raise_alarm(Alarm_SelftestFailed);
            return;
        }
    }

    stream_instance = stream;

    if(stream_enumerate_streams(claim_stream) && (nvs_address = nvs_alloc(sizeof(modbus_settings_t)))) {

        driver_reset = hal.driver_reset;
        hal.driver_reset = modbus_reset;

        task_add_systick(modbus_poll, NULL);

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

        //TODO: subscribe to grbl.on_reset event to terminate polling?

        settings_register(&setting_details);

        head = tail = &queue[0];

        uint_fast8_t idx;
        for(idx = 0; idx < MODBUS_QUEUE_LENGTH; idx++)
            queue[idx].next = idx == MODBUS_QUEUE_LENGTH - 1 ? &queue[0] : &queue[idx + 1];

        modbus_register_api(&api);

        system_register_commands(&commands);

        modbus_set_silence(NULL);

    } else {
        task_run_on_startup(report_warning, "Modbus failed to initialize!");
        system_raise_alarm(Alarm_SelftestFailed);
    }
}