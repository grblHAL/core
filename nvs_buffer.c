/*
  nvs_buffer.c - RAM based non-volatile storage buffer/emulation

  Part of grblHAL

  Copyright (c) 2017-2023 Terje Io
  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License.
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

//
// Can be used by MCUs with no nonvolatile storage options, be sure to allocate enough heap memory before use
//

#include <string.h>
#include <stdlib.h>
#include <assert.h>

#include "hal.h"
#include "nvs_buffer.h"
#include "protocol.h"
#include "settings.h"
#include "gcode.h"
#include "nvs.h"

static uint8_t *nvsbuffer = NULL;
static nvs_io_t physical_nvs;
static bool dirty;

settings_dirty_t settings_dirty;

typedef struct {
    uint16_t addr;
    uint8_t type;
    uint8_t offset;
} emap_t;

#define NVS_GROUP_GLOBAL 0
#define NVS_GROUP_TOOLS 1
#define NVS_GROUP_PARAMETERS 2
#define NVS_GROUP_STARTUP 3
#define NVS_GROUP_BUILD 4


#define PARAMETER_ADDR(n) (NVS_ADDR_PARAMETERS + n * (sizeof(coord_data_t) + NVS_CRC_BYTES))
#define STARTLINE_ADDR(n) (NVS_ADDR_STARTUP_BLOCK + n * (sizeof(stored_line_t) + NVS_CRC_BYTES))
#if N_TOOLS
#define TOOL_ADDR(n) (NVS_ADDR_TOOL_TABLE + n * (sizeof(tool_data_t) + NVS_CRC_BYTES))
#endif

static const emap_t target[] = {
    {NVS_ADDR_GLOBAL, NVS_GROUP_GLOBAL, 0},

    {PARAMETER_ADDR(0), NVS_GROUP_PARAMETERS, 0},
    {PARAMETER_ADDR(1), NVS_GROUP_PARAMETERS, 1},
    {PARAMETER_ADDR(2), NVS_GROUP_PARAMETERS, 2},
    {PARAMETER_ADDR(3), NVS_GROUP_PARAMETERS, 3},
    {PARAMETER_ADDR(4), NVS_GROUP_PARAMETERS, 4},
    {PARAMETER_ADDR(5), NVS_GROUP_PARAMETERS, 5},
    {PARAMETER_ADDR(6), NVS_GROUP_PARAMETERS, 6},
    {PARAMETER_ADDR(7), NVS_GROUP_PARAMETERS, 7},
    {PARAMETER_ADDR(8), NVS_GROUP_PARAMETERS, 8},
    {PARAMETER_ADDR(9), NVS_GROUP_PARAMETERS, 9},
    {PARAMETER_ADDR(10), NVS_GROUP_PARAMETERS, 10},
    {PARAMETER_ADDR(11), NVS_GROUP_PARAMETERS, 11},
    {STARTLINE_ADDR(0), NVS_GROUP_STARTUP, 0},
    {STARTLINE_ADDR(1), NVS_GROUP_STARTUP, 1},
#if N_STARTUP_LINE > 2
#error Increase number of startup line entries!
#endif
    {NVS_ADDR_BUILD_INFO, NVS_GROUP_BUILD, 0},
#if N_TOOLS
    {TOOL_ADDR(0), NVS_GROUP_TOOLS, 0},
    {TOOL_ADDR(1), NVS_GROUP_TOOLS, 1},
    {TOOL_ADDR(2), NVS_GROUP_TOOLS, 2},
    {TOOL_ADDR(3), NVS_GROUP_TOOLS, 3},
    {TOOL_ADDR(4), NVS_GROUP_TOOLS, 4},
    {TOOL_ADDR(5), NVS_GROUP_TOOLS, 5},
    {TOOL_ADDR(6), NVS_GROUP_TOOLS, 6},
    {TOOL_ADDR(7), NVS_GROUP_TOOLS, 7},
#if N_TOOLS > 8
    {TOOL_ADDR(8), NVS_GROUP_TOOLS, 8},
    {TOOL_ADDR(9), NVS_GROUP_TOOLS,  9},
    {TOOL_ADDR(10), NVS_GROUP_TOOLS, 10},
    {TOOL_ADDR(11), NVS_GROUP_TOOLS, 11},
    {TOOL_ADDR(12), NVS_GROUP_TOOLS, 12},
    {TOOL_ADDR(13), NVS_GROUP_TOOLS, 13},
    {TOOL_ADDR(14), NVS_GROUP_TOOLS, 14},
    {TOOL_ADDR(15), NVS_GROUP_TOOLS, 15},
#endif
#if N_TOOLS > 16
#error Increase number of tool entries!
#endif
#endif
    {0, 0, 0} // list termination - do not remove
};

inline static uint8_t ram_get_byte (uint32_t addr)
{
    return nvsbuffer[addr];
}

inline static void ram_put_byte (uint32_t addr, uint8_t new_value)
{
    if(addr == 0)
        settings_dirty.version = true;
    dirty = dirty || nvsbuffer[addr] != new_value || addr == 0;
    nvsbuffer[addr] = new_value;
}

static nvs_transfer_result_t memcpy_to_ram (uint32_t destination, uint8_t *source, uint32_t size, bool with_checksum)
{
    if(hal.nvs.driver_area.address && destination > hal.nvs.driver_area.address + hal.nvs.driver_area.size)
        return physical_nvs.memcpy_to_nvs(destination, source, size, with_checksum);

    uint32_t dest = destination;
    uint8_t checksum = with_checksum ? calc_checksum(source, size) : 0;

    dirty = false;

    for(; size > 0; size--)
        ram_put_byte(dest++, *(source++));

    if(with_checksum)
        ram_put_byte(dest, checksum);

    if(settings_dirty.version || source == hal.nvs.driver_area.mem_address)
        dirty = true;

    if(dirty && physical_nvs.type != NVS_None) {

        uint8_t idx = 0;

        settings_dirty.is_dirty = true;

        if(hal.nvs.driver_area.address && destination >= hal.nvs.driver_area.address)
            settings_dirty.driver_settings = true;

        else {

            do {
                if(target[idx].addr == destination)
                    break;
            } while(target[++idx].addr);

            if(target[idx].addr) switch(target[idx].type) {

                case NVS_GROUP_GLOBAL:
                    settings_dirty.global_settings = true;
                    break;
#if N_TOOLS
                case NVS_GROUP_TOOLS:
                    settings_dirty.tool_data |= (1 << target[idx].offset);
                    break;
#endif
                case NVS_GROUP_PARAMETERS:
                    settings_dirty.coord_data |= (1 << target[idx].offset);
                    break;

                case NVS_GROUP_STARTUP:
                    settings_dirty.startup_lines |= (1 << target[idx].offset);
                    break;

                case NVS_GROUP_BUILD:
                    settings_dirty.build_info = true;
                    break;
            }
        }
    }

    return NVS_TransferResult_OK;
}

static nvs_transfer_result_t memcpy_from_ram (uint8_t *destination, uint32_t source, uint32_t size, bool with_checksum)
{
    if(hal.nvs.driver_area.address && source > hal.nvs.driver_area.address + hal.nvs.driver_area.size)
        return physical_nvs.memcpy_from_nvs(destination, source, size, with_checksum);

    uint8_t checksum = with_checksum ? calc_checksum(&nvsbuffer[source], size) : 0;

    for(; size > 0; size--)
        *(destination++) = ram_get_byte(source++);

    return with_checksum ? (checksum == ram_get_byte(source) ? NVS_TransferResult_OK : NVS_TransferResult_Failed) : NVS_TransferResult_OK;
}

static void nvs_warning (sys_state_t state)
{
    report_message("Not enough heap for NVS buffer!", Message_Warning);
}

// Try to allocate RAM from heap for buffer/emulation.
bool nvs_buffer_alloc (void)
{
    assert(NVS_SIZE >= GRBL_NVS_SIZE);

    if((nvsbuffer = malloc(NVS_SIZE)))
        memset(nvsbuffer, 0, NVS_SIZE);

    return nvsbuffer != NULL;
}

void nvs_buffer_free (void)
{
    if(nvsbuffer) {
        nvs_buffer_sync_physical();
        free(nvsbuffer);
    }
}
//
// Switch over to RAM based copy.
// Changes to RAM based copy will be written to physical storage when grblHAL is in IDLE state.
bool nvs_buffer_init (void)
{
    hal.nvs.size = ((hal.nvs.size - 1) | 0x03) + 1; // Ensure NVS area ends on a word boundary

    if(nvsbuffer) {

        memcpy(&physical_nvs, &hal.nvs, sizeof(nvs_io_t)); // save pointers to physical storage handler functions

        // Copy physical storage content to RAM when available
        if(physical_nvs.type == NVS_Flash)
            physical_nvs.memcpy_from_flash(nvsbuffer);
        else if(physical_nvs.type != NVS_None)
            physical_nvs.memcpy_from_nvs(nvsbuffer, 0, GRBL_NVS_SIZE + hal.nvs.driver_area.size, false);

        // Switch hal to use RAM version of non-volatile storage data
        hal.nvs.type = NVS_Emulated;
        hal.nvs.get_byte = &ram_get_byte;
        hal.nvs.put_byte = &ram_put_byte;
        hal.nvs.memcpy_to_nvs = &memcpy_to_ram;
        hal.nvs.memcpy_from_nvs = &memcpy_from_ram;
        hal.nvs.memcpy_from_flash = NULL;
        hal.nvs.memcpy_to_flash = NULL;

        // If no physical storage available or if NVS import fails copy default settings to RAM
        // and write out to physical storage when available.
        if(physical_nvs.type == NVS_None || ram_get_byte(0) != SETTINGS_VERSION) {
            settings_restore(settings_all);
            if(physical_nvs.type == NVS_Flash)
                physical_nvs.memcpy_to_flash(nvsbuffer);
            else if(physical_nvs.memcpy_to_nvs)
                physical_nvs.memcpy_to_nvs(0, nvsbuffer, GRBL_NVS_SIZE + hal.nvs.driver_area.size, false);
            if(physical_nvs.type != NVS_None)
                grbl.report.status_message(Status_SettingReadFail);
        }
    } else
        protocol_enqueue_rt_command(nvs_warning);

    // Clear settings dirty flags
    memset(&settings_dirty, 0, sizeof(settings_dirty_t));

    return nvsbuffer != NULL;
}

// Allocate NVS block for driver settings.
// NOTE: allocation has to be done before content is copied from physical storage.
nvs_address_t nvs_alloc (size_t size)
{
    static uint8_t *mem_address;

    nvs_address_t addr = 0;

    // Check if already switched to emulation or buffer allocation failed, return NULL if so.
    if(hal.nvs.type == NVS_Emulated || nvsbuffer == NULL)
        return 0;

    if(hal.nvs.driver_area.address == 0) {
        hal.nvs.driver_area.address = GRBL_NVS_SIZE;
        hal.nvs.driver_area.mem_address = mem_address = nvsbuffer + GRBL_NVS_SIZE;
    }

    size += NVS_CRC_BYTES; // add room for checksum.
    if(hal.nvs.driver_area.size + size < (NVS_SIZE - GRBL_NVS_SIZE)) {
        mem_address = (uint8_t *)((uint32_t)(mem_address - 1) | 0x03) + 1; // Align to word boundary
        addr = mem_address - nvsbuffer;
        mem_address += size;
        hal.nvs.driver_area.size = mem_address - hal.nvs.driver_area.mem_address;
        hal.nvs.size = GRBL_NVS_SIZE + hal.nvs.driver_area.size + 1;
    }

    return addr;
}

// Write RAM changes to physical storage
void nvs_buffer_sync_physical (void)
{
    if(!settings_dirty.is_dirty)
        return;

    if(physical_nvs.memcpy_to_nvs) {

        if(settings_dirty.version)
            settings_dirty.version = physical_nvs.memcpy_to_nvs(0, nvsbuffer, 1, false) != NVS_TransferResult_OK;

        if(settings_dirty.global_settings)
            settings_dirty.global_settings = physical_nvs.memcpy_to_nvs(NVS_ADDR_GLOBAL, (uint8_t *)(nvsbuffer + NVS_ADDR_GLOBAL), sizeof(settings_t) + NVS_CRC_BYTES, false) != NVS_TransferResult_OK;

        if(settings_dirty.build_info)
            settings_dirty.build_info = physical_nvs.memcpy_to_nvs(NVS_ADDR_BUILD_INFO, (uint8_t *)(nvsbuffer + NVS_ADDR_BUILD_INFO), sizeof(stored_line_t) + NVS_CRC_BYTES, false) != NVS_TransferResult_OK;

        uint_fast8_t idx = N_STARTUP_LINE, offset;
        if(settings_dirty.startup_lines) do {
            idx--;
            if(bit_istrue(settings_dirty.startup_lines, bit(idx))) {
                bit_false(settings_dirty.startup_lines, bit(idx));
                offset = NVS_ADDR_STARTUP_BLOCK + idx * (sizeof(stored_line_t) + NVS_CRC_BYTES);
                if(physical_nvs.memcpy_to_nvs(offset, (uint8_t *)(nvsbuffer + offset), sizeof(stored_line_t) + NVS_CRC_BYTES, false) == NVS_TransferResult_OK)
                    bit_false(settings_dirty.startup_lines, bit(idx));
            }
        } while(idx);

        idx = N_CoordinateSystems;
        if(settings_dirty.coord_data) do {
            if(bit_istrue(settings_dirty.coord_data, bit(idx))) {
                offset = NVS_ADDR_PARAMETERS + idx * (sizeof(coord_data_t) + NVS_CRC_BYTES);
                if(physical_nvs.memcpy_to_nvs(offset, (uint8_t *)(nvsbuffer + offset), sizeof(coord_data_t) + NVS_CRC_BYTES, false) == NVS_TransferResult_OK)
                    bit_false(settings_dirty.coord_data, bit(idx));
            }
        } while(idx--);

        if(settings_dirty.driver_settings) {
            if(hal.nvs.driver_area.size > 0)
                settings_dirty.driver_settings = physical_nvs.memcpy_to_nvs(hal.nvs.driver_area.address, (uint8_t *)(nvsbuffer + hal.nvs.driver_area.address), hal.nvs.driver_area.size, false) != NVS_TransferResult_OK;
            else
                settings_dirty.driver_settings = false;
        }

#if N_TOOLS
        idx = N_TOOLS;
        if(settings_dirty.tool_data) do {
            idx--;
            if(bit_istrue(settings_dirty.tool_data, bit(idx))) {
                offset = NVS_ADDR_TOOL_TABLE + idx * (sizeof(tool_data_t) + NVS_CRC_BYTES);
                if(physical_nvs.memcpy_to_nvs(offset, (uint8_t *)(nvsbuffer + offset), sizeof(tool_data_t) + NVS_CRC_BYTES, false) == NVS_TransferResult_OK)
                    bit_false(settings_dirty.tool_data, bit(idx));
            }
        } while(idx);
#endif
        settings_dirty.is_dirty = settings_dirty.coord_data ||
                                   settings_dirty.global_settings ||
                                    settings_dirty.driver_settings ||
                                     settings_dirty.startup_lines ||
#if N_TOOLS
                                      settings_dirty.tool_data ||
#endif
                                       settings_dirty.build_info;

    } else if(physical_nvs.memcpy_to_flash) {
        if(!physical_nvs.memcpy_to_flash(nvsbuffer))
            report_message("Settings write failed!", Message_Warning);
        memset(&settings_dirty, 0, sizeof(settings_dirty_t));
    }
}

nvs_io_t *nvs_buffer_get_physical (void)
{
    return hal.nvs.type == NVS_Emulated ? &physical_nvs : &hal.nvs;
}

#ifdef DEBUGOUT

#include "report.h"

void nvs_memmap (void)
{
    char buf[30];

    report_message("NVS Area: addr size", Message_Plain);

    strcpy(buf, "Global: ");
    strcat(buf, uitoa(NVS_ADDR_GLOBAL));
    strcat(buf, " ");
    strcat(buf, uitoa(sizeof(settings_t) + NVS_CRC_BYTES));
    report_message(buf, Message_Plain);

    strcpy(buf, "Parameters: ");
    strcat(buf, uitoa(NVS_ADDR_PARAMETERS));
    strcat(buf, " ");
    strcat(buf, uitoa(N_CoordinateSystems * (sizeof(coord_data_t) + NVS_CRC_BYTES)));
    report_message(buf, Message_Plain);

    strcpy(buf, "Startup block: ");
    strcat(buf, uitoa(NVS_ADDR_STARTUP_BLOCK));
    strcat(buf, " ");
    strcat(buf, uitoa(N_STARTUP_LINE * (sizeof(stored_line_t) + NVS_CRC_BYTES)));
    report_message(buf, Message_Plain);

    strcpy(buf, "Build info: ");
    strcat(buf, uitoa(NVS_ADDR_BUILD_INFO));
    strcat(buf, " ");
    strcat(buf, uitoa(sizeof(stored_line_t) + NVS_CRC_BYTES));
    report_message(buf, Message_Plain);

#if N_TOOLS
    strcpy(buf, "Tool table: ");
    strcat(buf, uitoa(NVS_ADDR_TOOL_TABLE));
    strcat(buf, " ");
    strcat(buf, uitoa(N_TOOLS * (sizeof(tool_data_t) + NVS_CRC_BYTES)));
    report_message(buf, Message_Plain);
#endif

    strcpy(buf, "Driver: ");
    strcat(buf, uitoa(hal.nvs.driver_area.address));
    strcat(buf, " ");
    strcat(buf, uitoa(hal.nvs.driver_area.size));
    report_message(buf, Message_Plain);
}

#endif

