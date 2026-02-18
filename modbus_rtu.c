/*

  modbus_rtu.c - a lightweight ModBus RTU implementation

  Part of grblHAL

  Copyright (c) 2020-2026 Terje Io

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

typedef struct {
    uint32_t baud_rate;
    uint32_t rx_timeout;
} rtu_settings_t;

typedef enum {
    ModBus_Idle,
    ModBus_Silent,
    ModBus_TX,
    ModBus_AwaitReply,
    ModBus_TimeoutException,
    ModBus_GotReply,
    ModBus_Exception,
    ModBus_Retry
} modbus_state_t;

typedef struct queue_entry {
    bool async;
    modbus_message_t msg;
    modbus_callbacks_t callbacks;
    struct queue_entry *next;
} queue_entry_t;

PROGMEM static const uint32_t baud[] = { 2400, 4800, 9600, 19200, 38400, 115200 };
PROGMEM static const modbus_silence_timeout_t dflt_timeout =
{
    .b2400   = 16,
    .b4800   = 8,
    .b9600   = 4,
    .b19200  = 2,
    .b38400  = 2,
    .b115200 = 2
};

static modbus_rtu_stream_t stream;
static int8_t stream_instance = -1;
static uint32_t rx_timeout = 0, silence_until = 0, silence_timeout;
static modbus_exception_t exception_code = ModBus_NoException;
static modbus_silence_timeout_t silence;
static queue_entry_t queue[MODBUS_QUEUE_LENGTH];
static rtu_settings_t modbus;
static volatile bool spin_lock = false, is_blocking = false, is_up = false;
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
        if(callbacks->on_rx_timeout == NULL) // for backwards comaptibility
            packet->callbacks.on_rx_timeout = packet->callbacks.on_rx_exception;
        if(!packet->async && packet->callbacks.retries)
            packet->callbacks.on_rx_exception = packet->callbacks.on_rx_timeout = retry_exception;
    } else
        memset(&packet->callbacks, 0, sizeof(modbus_callbacks_t));

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
    stream.write(((queue_entry_t *)msg)->msg.adu, ((queue_entry_t *)msg)->msg.tx_length);
}

// called once every ms
static void modbus_poll (void *data)
{
    if(spin_lock)
        return;

    spin_lock = true;

    switch(state) {

        case ModBus_Idle:
            if(tail != head && !packet && !is_blocking) {
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
                    if(packet->callbacks.on_rx_timeout)
                        packet->callbacks.on_rx_timeout(0, packet->msg.context);
                    packet = NULL;
                } else if(stream.read() == packet->msg.adu[0] && (stream.read() & 0x80)) {
                    int32_t code = stream.read();
                    exception_code = code == SERIAL_NO_DATA ? ModBus_UnknownException : (modbus_exception_t)(code & 0xFF);
                    state = ModBus_Exception;
                    stats.rx_exceptions++;
                } else
                    state = ModBus_TimeoutException;

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
                                packet->callbacks.on_rx_exception(ModBus_CRCError, packet->msg.context);
                            packet = NULL;
                        }
                        silence_until = hal.get_elapsed_ticks() + silence_timeout;
                        break;
                    }
                }

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

        case ModBus_TimeoutException:
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
    static queue_entry_t sync_msg = {0};

    if(msg->tx_length > MODBUS_MAX_ADU_SIZE || msg->rx_length > MODBUS_MAX_ADU_SIZE) {
        if(callbacks->on_rx_exception)
            callbacks->on_rx_exception(ModBus_IllegalSize, msg->context);
        return false;
    }

    uint_fast16_t crc = modbus_crc16x(msg->adu, msg->tx_length - 2);

    msg->adu[msg->tx_length - 1] = crc >> 8;
    msg->adu[msg->tx_length - 2] = crc & 0xFF;

    while(spin_lock);

    if(block) {

        if(is_blocking)
            return false;

        is_blocking = true;

        while(state != ModBus_Idle)
            grbl.on_execute_realtime(state_get());

        tx_message(add_message(&sync_msg, msg, false, callbacks));

        while(is_blocking) {

            grbl.on_execute_realtime(state_get());

            switch(state) {

                case ModBus_TimeoutException:
                    if(packet->callbacks.on_rx_timeout)
                        packet->callbacks.on_rx_timeout(ModBus_Timeout, packet->msg.context);
                    is_blocking = packet->callbacks.retries > 0;
                    break;

                case ModBus_Exception:
                    if(packet->callbacks.on_rx_exception)
                        packet->callbacks.on_rx_exception((uint8_t)exception_code, packet->msg.context);
                    is_blocking = packet->callbacks.retries > 0;
                    break;

                case ModBus_GotReply:
                    if(packet->callbacks.on_rx_packet)
                        packet->callbacks.on_rx_packet(&((queue_entry_t *)packet)->msg);
                    is_blocking = block = false;
                    break;

                case ModBus_Retry:
                    if((int32_t)(silence_until - hal.get_elapsed_ticks()) <= 0) {
                        silence_until = 0;
                        if(--packet->callbacks.retries == 0) {
                            packet->callbacks.on_rx_exception = callbacks->on_rx_exception;
                            packet->callbacks.on_rx_timeout = callbacks->on_rx_timeout ? callbacks->on_rx_timeout : callbacks->on_rx_exception;
                        }
                        packet = add_message(&sync_msg, msg, false, (const modbus_callbacks_t *)&packet->callbacks);
                        tx_message(packet);
                    }
                    break;

                default:
                    break;
            }
        }

        packet = NULL;
        is_blocking = false;
        state = silence_until > 0 ? ModBus_Silent : ModBus_Idle;

    } else if(packet != &sync_msg) {
        if(head->next != tail) {
            add_message((queue_entry_t *)head, msg, true, callbacks);
            head = head->next;
        }
    }

    return !block;
}

FLASHMEM static void modbus_reset (void)
{
    while(spin_lock);

    if(sys.abort) {

        if(packet) {
            memset((void *)&packet->callbacks, 0, sizeof(modbus_callbacks_t));
            packet = NULL;
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

FLASHMEM static uint32_t get_baudrate (uint32_t rate)
{
    uint32_t idx = sizeof(baud) / sizeof(uint32_t);

    do {
        if(baud[--idx] == rate)
            return idx;
    } while(idx);

    return DEFAULT_MODBUS_STREAM_BAUD;
}

PROGMEM static const setting_group_detail_t modbus_groups [] = {
    { Group_Root, Group_ModBus, "ModBus"}
};

FLASHMEM static status_code_t modbus_set_baud (setting_id_t id, uint_fast16_t value)
{
    settings.modbus_baud = (uint8_t)value;
    modbus.baud_rate = baud[settings.modbus_baud];
    silence_timeout = silence.timeout[settings.modbus_baud];
    stream.set_baud_rate(modbus.baud_rate);

    return Status_OK;
}

FLASHMEM static uint32_t modbus_get_baud (setting_id_t setting)
{
    return get_baudrate(modbus.baud_rate);
}

FLASHMEM static status_code_t modbus_set_format (setting_id_t id, uint_fast16_t value)
{
    if(stream.set_format) {
        settings.modbus_stream_format.parity = (serial_parity_t)value;
        stream.set_format(settings.modbus_stream_format);
        settings_write_global();
    }

    return stream.set_format ? Status_OK : Status_SettingDisabled;
}

FLASHMEM static uint32_t modbus_get_format (setting_id_t setting)
{
    return (uint32_t)settings.modbus_stream_format.parity;
}

FLASHMEM static bool can_set_format (const setting_detail_t *setting, uint_fast16_t offset)
{
    return stream.set_format != NULL;
}

PROGMEM static const setting_detail_t modbus_settings[] = {
    { Settings_ModBus_BaudRate, Group_ModBus, "ModBus baud rate", NULL, Format_RadioButtons, "2400,4800,9600,19200,38400,115200", NULL, NULL, Setting_NonCoreFn, modbus_set_baud, modbus_get_baud, NULL },
    { Settings_ModBus_RXTimeout, Group_ModBus, "ModBus RX timeout", "milliseconds", Format_Integer, "####0", "50", "250", Setting_NonCore, &modbus.rx_timeout, NULL, NULL },
    { Setting_ModBus_StreamFormat, Group_ModBus, "ModBus serial format", NULL, Format_RadioButtons, "8-bit no parity, 8-bit even parity, 8-bit odd parity", NULL, NULL, Setting_NonCoreFn, modbus_set_format, modbus_get_format, can_set_format }
};

FLASHMEM static void modbus_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&modbus, sizeof(rtu_settings_t), true);
}

FLASHMEM static void modbus_settings_restore (void)
{
    modbus.rx_timeout = 50;
    modbus.baud_rate = baud[DEFAULT_MODBUS_STREAM_BAUD];

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&modbus, sizeof(rtu_settings_t), true);
}

FLASHMEM static void modbus_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&modbus, nvs_address, sizeof(rtu_settings_t), true) != NVS_TransferResult_OK ||
         modbus.baud_rate != baud[get_baudrate(modbus.baud_rate)])
        modbus_settings_restore();

    is_up = true;
    silence_timeout = silence.timeout[get_baudrate(modbus.baud_rate)];

    stream.set_baud_rate(modbus.baud_rate);

    if(stream.set_format)
        stream.set_format(settings.modbus_stream_format);
}

FLASHMEM static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        report_plugin("MODBUS", "0.22");
}

static bool modbus_rtu_isup (void)
{
    return is_up;
}

static bool modbus_is_busy (void)
{
    return state != STATE_IDLE;
}

FLASHMEM static void modbus_rtu_flush_queue (void)
{
    while(spin_lock);

    tail = head;
}

FLASHMEM static void modbus_rtu_set_silence (const modbus_silence_timeout_t *timeout)
{
    if(timeout)
        memcpy(&silence, timeout, sizeof(modbus_silence_timeout_t));
    else
        memcpy(&silence, &dflt_timeout, sizeof(modbus_silence_timeout_t));

    silence_timeout = silence.timeout[get_baudrate(modbus.baud_rate)];
}

FLASHMEM static bool stream_is_valid (const io_stream_t *stream)
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

FLASHMEM static bool claim_stream (io_stream_properties_t const *sstream, void *data)
{
    io_stream_t const *claimed = NULL;

    if(sstream->type == StreamType_Serial && (stream_instance >= 0
                                               ? sstream->instance == (uint8_t)stream_instance
                                               : sstream->flags.modbus_ready && !sstream->flags.claimed)) {
        if((claimed = sstream->claim(baud[DEFAULT_MODBUS_STREAM_BAUD])) && stream_is_valid(claimed)) {

            claimed->set_enqueue_rt_handler(stream_buffer_all);

            stream.set_baud_rate = claimed->set_baud_rate;
            stream.set_format = claimed->set_format;                              //!< Optional handler for setting the stream format.
            stream.get_tx_buffer_count = claimed->get_tx_buffer_count;
            stream.get_rx_buffer_count = claimed->get_rx_buffer_count;
            stream.write = claimed->write_n;
            stream.read = claimed->read;
            stream.flush_tx_buffer = claimed->reset_write_buffer;
            stream.flush_rx_buffer = claimed->reset_read_buffer;
            stream.set_direction = claimed->set_direction;

            if(hal.periph_port.set_pin_description) {
                hal.periph_port.set_pin_description(Output_TX, (pin_group_t)(PinGroup_UART + claimed->instance), "Modbus");
                hal.periph_port.set_pin_description(Input_RX, (pin_group_t)(PinGroup_UART + claimed->instance), "Modbus");
            }
        } else
            claimed = NULL;
    }

    return claimed != NULL;
}

FLASHMEM static status_code_t report_stats (sys_state_t state, char *args)
{
    char buf[110];

    snprintf(buf, sizeof(buf) - 1, "TX: " UINT32FMT ", retries: " UINT32FMT ", timeouts: " UINT32FMT ", RX exceptions: " UINT32FMT ", CRC errors: " UINT32FMT,
              stats.tx_count, stats.retries, stats.timeouts, stats.rx_exceptions, stats.crc_errors);

    report_message(buf, Message_Info);

    if(args && (*args == 'r' || *args == 'R'))
        stats.tx_count = stats.retries = stats.timeouts = stats.rx_exceptions = stats.crc_errors = 0;

    return Status_OK;
}

modbus_rtu_stream_t *modbus_get_rtu_stream (void)
{
    return stream.read == NULL ? NULL : &stream;
}

FLASHMEM void modbus_rtu_init (int8_t instance, int8_t dir_aux)
{
    PROGMEM static const modbus_api_t api = {
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

    PROGMEM static const sys_command_t command_list[] = {
        {"MODBUSSTATS", report_stats, { .allow_blocking = On }, { .str = "output Modbus RTU statistics" } },
    };

    static sys_commands_t commands = {
        .n_commands = sizeof(command_list) / sizeof(sys_command_t),
        .commands = command_list
    };

    stream_instance = instance;

    if((hal.driver_cap.modbus_rtu = stream_enumerate_streams(claim_stream, NULL) && (nvs_address = nvs_alloc(sizeof(rtu_settings_t))))) {

        if(stream.set_direction == NULL && dir_aux != -2) {

            xbar_t *dir_pin; // TODO: move to top and use for direct access
            io_port_cfg_t d_out;

            ioports_cfg(&d_out, Port_Digital, Port_Output);

            dir_port = dir_aux != -1 ? dir_aux : (d_out.n_ports ? d_out.n_ports - 1 : IOPORT_UNASSIGNED);

            if((dir_pin = d_out.claim(&d_out, &dir_port, NULL, (pin_cap_t){}))) {
                stream.set_direction = modbus_set_direction;
                ioport_set_function(dir_pin, Output_RS485_Direction, NULL);
            }

            hal.driver_cap.modbus_rtu = !!stream.set_direction;
        }

        if((hal.driver_cap.modbus_rtu = hal.driver_cap.modbus_rtu && task_add_systick(modbus_poll, NULL))) {

            driver_reset = hal.driver_reset;
            hal.driver_reset = modbus_reset;

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
        }
    }

    if(!hal.driver_cap.modbus_rtu) {
        task_run_on_startup(report_warning, "Modbus RTU failed to initialize!");
        system_raise_alarm(Alarm_SelftestFailed);
    }
}
