/*
  stream.h - shared stream rx buffer copy for tool change protocol

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

static stream_rx_buffer_t rxbackup;

// "dummy" version of serialGetC
static int16_t stream_get_null (void)
{
    return SERIAL_NO_DATA;
}

bool stream_rx_suspend (stream_rx_buffer_t *rxbuffer, bool suspend)
{
    if(suspend)
        hal.stream.read = stream_get_null;
    else if(rxbuffer->backup)
        memcpy(rxbuffer, &rxbackup, sizeof(stream_rx_buffer_t));

    return rxbuffer->tail != rxbuffer->head;
}

ISR_CODE void stream_rx_backup (stream_rx_buffer_t *rxbuffer)
{
    memcpy(&rxbackup, rxbuffer, sizeof(stream_rx_buffer_t));
    rxbuffer->backup = true;
    rxbuffer->tail = rxbuffer->head;
}
