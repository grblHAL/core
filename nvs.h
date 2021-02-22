/*
  nvs.h - non-volative storage data structures

  Part of grblHAL

  Copyright (c) 2017-2020 Terje Io
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

#ifndef _NVS_H_
#define _NVS_H_

#ifndef NVS_SIZE
#define NVS_SIZE 2048
#endif

#define GRBL_NVS_SIZE 1024
#define NVS_CRC_BYTES 1

// Define persistent storage memory address location values for Grbl settings and parameters
// NOTE: 1KB persistent storage is the minimum required. The upper half is reserved for parameters and
// the startup script. The lower half contains the global settings and space for future
// developments.
#define NVS_ADDR_GLOBAL         1U
#define NVS_ADDR_PARAMETERS     512U
#define NVS_ADDR_BUILD_INFO     942U
#define NVS_ADDR_STARTUP_BLOCK  (NVS_ADDR_BUILD_INFO - 1 - N_STARTUP_LINE * (sizeof(stored_line_t) + NVS_CRC_BYTES))
#ifdef N_TOOLS
#define NVS_ADDR_TOOL_TABLE     (NVS_ADDR_PARAMETERS - 1 - N_TOOLS * (sizeof(tool_data_t) + NVS_CRC_BYTES))
#endif

typedef enum {
    NVS_None = 0,
    NVS_EEPROM,
    NVS_FRAM,
    NVS_Flash,
    NVS_Emulated
} nvs_type;

typedef struct {
    uint8_t *mem_address;
    uint16_t address;
    uint16_t size;
} nvs_driver_area_t;

typedef enum {
    NVS_TransferResult_Failed = 0,
    NVS_TransferResult_Busy,
    NVS_TransferResult_OK,
} nvs_transfer_result_t;

typedef struct {
    nvs_type type;
    uint16_t size;
    nvs_driver_area_t driver_area;
    uint8_t (*get_byte)(uint32_t addr);
    void (*put_byte)(uint32_t addr, uint8_t new_value);
    nvs_transfer_result_t (*memcpy_to_nvs)(uint32_t destination, uint8_t *source, uint32_t size, bool with_checksum);
    nvs_transfer_result_t (*memcpy_from_nvs)(uint8_t *destination, uint32_t source, uint32_t size, bool with_checksum);
    bool (*memcpy_from_flash)(uint8_t *dest);
    bool (*memcpy_to_flash)(uint8_t *source);
} nvs_io_t;

#endif
