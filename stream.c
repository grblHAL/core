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
#include "state_machine.h"

static stream_rx_buffer_t rxbackup;

// "dummy" version of serialGetC
int16_t stream_get_null (void)
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

ISR_CODE bool stream_enable_mpg (const io_stream_t *mpg_stream, bool mpg_mode)
{
    static io_stream_t org_stream = {
        .type = StreamType_Redirected
    };

    sys_state_t state = state_get();

    // Deny entering MPG mode if busy
    if(mpg_mode == sys.mpg_mode || (mpg_mode && (gc_state.file_run || !(state == STATE_IDLE || (state & (STATE_ALARM|STATE_ESTOP)))))) {
        hal.stream.enqueue_realtime_command(CMD_STATUS_REPORT_ALL);
        return false;
    }

    if(mpg_mode) {
        if(org_stream.type == StreamType_Redirected) {
            memcpy(&org_stream, &hal.stream, offsetof(io_stream_t, enqueue_realtime_command));
            if(hal.stream.disable)
                hal.stream.disable(true);
            mpg_stream->disable(false);
            hal.stream.read = mpg_stream->read;
            hal.stream.get_rx_buffer_free = mpg_stream->get_rx_buffer_free;
            hal.stream.cancel_read_buffer = mpg_stream->cancel_read_buffer;
            hal.stream.reset_read_buffer = mpg_stream->reset_read_buffer;
        }
    } else if(org_stream.type != StreamType_Redirected) {
        mpg_stream->disable(true);
        memcpy(&hal.stream, &org_stream, offsetof(io_stream_t, enqueue_realtime_command));
        org_stream.type = StreamType_Redirected;
        if(hal.stream.disable)
            hal.stream.disable(false);
    }

    hal.stream.reset_read_buffer();

    sys.mpg_mode = mpg_mode;
    sys.report.mpg_mode = On;

    // Force a realtime status report, all reports when MPG mode active
    hal.stream.enqueue_realtime_command(mpg_mode ? CMD_STATUS_REPORT_ALL : CMD_STATUS_REPORT);

    return true;
}
