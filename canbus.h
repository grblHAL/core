/*

  canbus.h -

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

#ifndef _CANBUS_H_
#define _CANBUS_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint32_t id;
    uint8_t  len;
    uint8_t  data[8];
} canbus_message_t;

typedef bool (*can_rx_ptr)(canbus_message_t);
typedef bool (*can_rx_enqueue_fn)(canbus_message_t msg, can_rx_ptr callback); // used internally by the driver

/*
 * Function prototypes
 */
void canbus_init (void);
bool canbus_enabled (void);
bool canbus_queue_tx (canbus_message_t message, bool ext_id);
bool canbus_add_filter (uint32_t id, uint32_t mask, bool ext_id, can_rx_ptr callback);

#endif /* _CANBUS_H_ */
