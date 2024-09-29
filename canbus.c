/*

  canbus.c -

  Part of grblHAL

  Copyright (c) 2022 Jon Escombe
  Copyright (c) 2024 Terje Io

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
#include "canbus.h"

#ifndef CANBUS_BUFFER_LEN
#define CANBUS_BUFFER_LEN 8
#endif
#ifndef CANBUS_BAUDRATE
#define CANBUS_BAUDRATE 0  // 125000
#endif

typedef struct {
    volatile canbus_message_t message;
    can_rx_ptr callback;
} canbus_rx_message;

typedef struct {
    volatile canbus_message_t message;
    bool ext_id;
} canbus_tx_message;

typedef struct {
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile canbus_tx_message tx[CANBUS_BUFFER_LEN];
} canbus_tx_buffer_t;

typedef struct {
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile canbus_rx_message rx[CANBUS_BUFFER_LEN];
} canbus_rx_buffer_t;

static bool isEnabled = false;
static const uint32_t baud[] = { 125000, 250000, 500000, 1000000 };
static canbus_tx_buffer_t tx_buffer = {0};
static canbus_rx_buffer_t rx_buffer = {0};

// Weak implementations of low level functions to be provided by the driver

__attribute__((weak)) bool can_start (uint32_t baud, can_rx_enqueue_fn callback)
{
    return false;
}

__attribute__((weak)) bool can_stop (void)
{
    return false;
}

__attribute__((weak)) bool can_set_baud (uint32_t baud)
{
    return false;
}

__attribute__((weak)) bool can_put (canbus_message_t msg, bool ext_id)
{
    return false;
}

__attribute__((weak)) bool can_add_filter (uint32_t id, uint32_t mask, bool ext_id, can_rx_ptr callback)
{
    return false;
}

// ---

ISR_CODE static bool ISR_FUNC(canbus_queue_rx)(canbus_message_t message, can_rx_ptr callback)
{
    bool ok;
    uint8_t next_head = (rx_buffer.head + 1) % CANBUS_BUFFER_LEN;

    if((ok = next_head != rx_buffer.tail)) {
        rx_buffer.rx[rx_buffer.head].callback = callback;
        rx_buffer.rx[rx_buffer.head].message = message;

        rx_buffer.head = next_head;
    }

    return ok;
}

// called every 1 ms
static void canbus_poll (void *data)
{
    /* if have TX data, sends one message per iteration.. */
    if(tx_buffer.head != tx_buffer.tail && can_put(tx_buffer.tx[tx_buffer.tail].message, tx_buffer.tx[tx_buffer.tail].ext_id))
        tx_buffer.tail = (tx_buffer.tail + 1) % CANBUS_BUFFER_LEN;

    /* if have RX data, process one message per iteration.. */
    if(rx_buffer.head != rx_buffer.tail) {
        if(rx_buffer.rx[rx_buffer.tail].callback)
            rx_buffer.rx[rx_buffer.tail].callback(rx_buffer.rx[rx_buffer.tail].message);
        rx_buffer.tail = (rx_buffer.tail + 1) % CANBUS_BUFFER_LEN;
    }
}

static bool canbus_start (uint32_t baud)
{
    if((isEnabled = can_start(baud, canbus_queue_rx)))
        task_add_systick(canbus_poll, NULL);

    return isEnabled;
}

static status_code_t canbus_set_baud (setting_id_t id, uint_fast16_t value)
{
    settings.canbus_baud = value;

    return can_set_baud(baud[settings.canbus_baud]) ? Status_OK : Status_SettingValueOutOfRange;
}

static uint32_t canbus_get_baud (setting_id_t setting)
{
    return settings.canbus_baud < (sizeof(baud) / sizeof(uint32_t)) ? settings.canbus_baud : CANBUS_BAUDRATE;
}

static const setting_group_detail_t canbus_groups [] = {
    { Group_Root, Group_CANbus, "CAN bus"}
};

static const setting_detail_t canbus_setting_detail[] = {
    { Setting_CANbus_BaudRate, Group_CANbus, "CAN bus baud rate", NULL, Format_RadioButtons, "125000,250000,500000,1000000", NULL, NULL, Setting_NonCoreFn, canbus_set_baud, canbus_get_baud, NULL },
};

static void canbus_settings_restore (void)
{
    settings.canbus_baud = CANBUS_BAUDRATE;

    settings_write_global();
}

static void canbus_settings_load (void)
{
    canbus_start(baud[canbus_get_baud(Setting_CANbus_BaudRate)]);
}

static setting_details_t setting_details = {
    .groups = canbus_groups,
    .n_groups = sizeof(canbus_groups) / sizeof(setting_group_detail_t),
    .settings = canbus_setting_detail,
    .n_settings = sizeof(canbus_setting_detail) / sizeof(setting_detail_t),
    .save = settings_write_global,
    .load = canbus_settings_load,
    .restore = canbus_settings_restore
};

// Public API

bool canbus_enabled (void)
{
    return isEnabled;
}

bool canbus_queue_tx (canbus_message_t message, bool ext_id)
{
    bool ok;
    uint8_t next_head = (tx_buffer.head + 1) % CANBUS_BUFFER_LEN;

    if((ok = next_head != tx_buffer.tail)) {
        tx_buffer.tx[tx_buffer.head].ext_id = ext_id;
        tx_buffer.tx[tx_buffer.head].message = message;
        tx_buffer.head = next_head;
    }

    return ok;
}

bool canbus_add_filter (uint32_t id, uint32_t mask, bool ext_id, can_rx_ptr callback)
{
    return can_add_filter(id, mask, ext_id, callback);
}

void canbus_init (void)
{
    static bool init_ok = false;

    if(!init_ok) {
        init_ok = true;
        settings_register(&setting_details);
    }
}
