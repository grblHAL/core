/*
  nvs_buffer.h - RAM based non-volatile storage buffer/emulation

  Part of grblHAL

  Copyright (c) 2017-2021 Terje Io
  Copyright (c) 2009-2011 Simen Svale Skogsrud

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

#ifndef _NVS_BUFFER_H_
#define _NVS_BUFFER_H_

typedef uint32_t nvs_address_t;

typedef struct {
    bool is_dirty;
    bool version;
    bool global_settings;
    bool build_info;
    bool driver_settings;
    uint8_t startup_lines;
    uint16_t coord_data;
#ifdef N_TOOLS
    uint16_t tool_data;
#endif
} settings_dirty_t;

extern settings_dirty_t settings_dirty;

bool nvs_buffer_init (void);
bool nvs_buffer_alloc (void);
void nvs_buffer_free (void);
nvs_address_t nvs_alloc (size_t size);
void nvs_buffer_sync_physical (void);
nvs_io_t *nvs_buffer_get_physical (void);
void nvs_memmap (void);

#endif
