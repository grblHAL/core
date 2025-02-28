/*
  nvs.h - non-volative storage data structures

  Part of grblHAL

  Copyright (c) 2017-2025 Terje Io
  Copyright (c) 2009-2011 Simen Svale Skogsrud

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

#ifndef _NVS_H_
#define _NVS_H_

#ifndef NVS_SIZE
/*! \brief Total size in bytes of the NVS.
Minimum 1024 bytes required, more if > 5 axes enabled, space for driver and/or plugin data and settings is required.
*/
#define NVS_SIZE 2048
#endif

//! Number of bytes used for storing CRC values. Do not change this!
#ifndef NVS_CRC_BYTES
#define NVS_CRC_BYTES 2
#endif

#define NVS_SIZE_PARAMETERS     ((sizeof(coord_data_t) + NVS_CRC_BYTES) * N_CoordinateSystems)
#define NVS_SIZE_BUILD_INFO     (sizeof(stored_line_t) + NVS_CRC_BYTES)
#define NVS_SIZE_STARTUP_BLOCK  (N_STARTUP_LINE * (sizeof(stored_line_t) + NVS_CRC_BYTES))

/*! \brief Number of bytes at the start of the NVS area reserved for core settings and parameters.
Minimum 1024 bytes required, more if > 5 axes enabled.
*/
#if N_AXIS > 5
#define GRBL_NVS_END (NVS_ADDR_GLOBAL + ((sizeof(settings_t) + NVS_CRC_BYTES + 4) & 0xFFFC) + NVS_SIZE_PARAMETERS + NVS_SIZE_BUILD_INFO + NVS_SIZE_STARTUP_BLOCK + 1)
#else
#define GRBL_NVS_END 1023
#endif

#if NVS_CRC_BYTES > 1
#define calc_checksum(data, length) modbus_crc16x(data, length)
#else
#define calc_checksum(data, length) grbl_crc8(data, length)
#endif

/*! @name Define persistent storage memory address location values for core settings and parameters.
The upper half is reserved for parameters and the startup script.
The lower half contains the global settings and space for future developments.

__NOTE:__ 1024 bytes of persistent storage is the minimum required.
*/
///@{
#define NVS_ADDR_GLOBAL         1U
#if N_AXIS > 5
#define NVS_ADDR_PARAMETERS     ((sizeof(settings_t) + NVS_CRC_BYTES + 4) & 0xFFFC) // align to word boundary
#else
#define NVS_ADDR_PARAMETERS     512U
#endif
#define NVS_ADDR_BUILD_INFO     (GRBL_NVS_END - NVS_SIZE_BUILD_INFO)
#define NVS_ADDR_STARTUP_BLOCK  (NVS_ADDR_BUILD_INFO - NVS_SIZE_STARTUP_BLOCK - 1)
#if N_TOOLS
#define NVS_ADDR_TOOL_TABLE     (GRBL_NVS_END + 1)
#define GRBL_NVS_SIZE           (GRBL_NVS_END + 1 + N_TOOLS * (sizeof(tool_data_t) + NVS_CRC_BYTES))
#else
#define GRBL_NVS_SIZE (GRBL_NVS_END + 1)
#endif
///@}

typedef enum {
    NVS_None = 0,   //!< 0
    NVS_EEPROM,     //!< 1
    NVS_FRAM,       //!< 2
    NVS_Flash,      //!< 3
    NVS_Emulated    //!< 4 - used by the core for buffered read and write
} nvs_type;

//! Structure for keeping track of NVS area used by driver and/or plugin code.
typedef struct {
    uint8_t *mem_address;   //!< Pointer to location in RAM where driver area is located.
    uint16_t address;       //!< Index based address into the storage area where the driver area starts.
    uint16_t size;          //!< Actual size of driver area in bytes.
} nvs_driver_area_t;

/*! \brief Pointer to function for getting a byte from NVS storage.
\param addr index base address into the area.
\returns byte read.
*/
typedef uint8_t (*get_byte_ptr)(uint32_t addr);

/*! \brief Pointer to function for putting a byte into NVS storage.
\param addr index based address into the storage area.
\param new_value byte to write.
*/
typedef void (*put_byte_ptr)(uint32_t addr, uint8_t new_value);

/*! \brief Pointer to function for reading a block of data from NVS storage
\param dest pointer to destination of data.
\param source index based address into the storage area.
\param size number of bytes to write.
\param with_checksum \a true calculate and verify checksum at the end of the data block, \a false do not calculate and verify checksum.
\returns #nvs_transfer_result_t enum.
*/
typedef bool (*memcpy_from_nvs_ptr)(uint8_t *dest, uint32_t source, uint32_t size, bool with_checksum);

/*! \brief Pointer to function for writing a block of data to NVS storage
\param dest index based address into the storage area.
\param source pointer to source data
\param size number of bytes to write
\param with_checksum \a true calculate and add a checksum at the end of the data block, \a false do not add checksum.
\returns #nvs_transfer_result_t enum.
*/
typedef bool (*memcpy_to_nvs_ptr)(uint32_t dest, uint8_t *source, uint32_t size, bool with_checksum);

/*! \brief Pointer to function for reading a block of data from flash based NVS storage.
\param dest pointer to destination of data.
\returns true if successful, false otherwise.
*/
typedef bool (*memcpy_from_flash_ptr)(uint8_t *dest);

/*! \brief Pointer to function for reading a block of data from flash based NVS storage.
\param source pointer to source data.
\returns true if successful, false otherwise.
*/
typedef bool (*memcpy_to_flash_ptr)(uint8_t *source);

//! \brief Handler functions and variables for NVS storage of settings and data.
typedef struct {
    nvs_type type;                              //!< Type of NVS storage.
    uint32_t size;                              //!< Actual size of non-volatile storage area in bytes.
    uint32_t size_max;                          //!< Physical size of non-volatile storage area in bytes.
    nvs_driver_area_t driver_area;
//! @name Handler functions for EEPROM or FRAM based storage.
//@{
    get_byte_ptr get_byte;                      //!< Handler for reading a byte from EEPROM or FRAM.
    put_byte_ptr put_byte;                      //!< Handler for writing a byte to EEPROM or FRAM.
    memcpy_to_nvs_ptr memcpy_to_nvs;            //!< Handler for reading a block of data from EEPROM or FRAM.
    memcpy_from_nvs_ptr memcpy_from_nvs;        //!< Handler for writing a block of data to EEPROM or FRAM.
//@}
//! @name Handler functions for Flash based storage.
//@{
    memcpy_from_flash_ptr memcpy_from_flash;    //!< Handler for reading a block of data from flash.
    memcpy_to_flash_ptr memcpy_to_flash;        //!< Handler for writing a block of data to flash.
//@}
} nvs_io_t;

#endif
