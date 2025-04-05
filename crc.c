/*

  crc.c - crc implementations used by grblHAL

  Part of grblHAL

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

#include <stdint.h>

uint16_t grbl_crc8 (const uint8_t *buf, uint32_t size)
{
    uint8_t checksum = 0;

    while(size--) {
        checksum = (checksum << 1) | (checksum >> 7);
        checksum += *(buf++);
    }

    return (uint16_t)checksum;
}

// Copyright (c) 2006 Christian Walter <wolti@sil.at>
// Lifted from his FreeModbus Libary
uint16_t modbus_crc16x (const uint8_t *buf, uint_fast16_t len)
{
    uint_fast8_t i;
    uint16_t crc = 0xFFFF;
 
    while(len--) {
        crc ^= (uint16_t)*buf++;;       // XOR byte into least sig. byte of crc
        for(i = 8; i != 0; i--) {       // Loop over each bit
            if(crc & 0x0001) {          // If the LSB is set
                crc >>= 1;              // Shift right and XOR 0xA001
                crc ^= 0xA001;
            } else                      // Else LSB is not set
                crc >>= 1;              // Just shift right
        }
    }

    // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
    return crc;
}

// Fast CRC16 implementation
// Original Code: Ashley Roll
// Optimisations: Scott Dattalo
// From http://www.ccsinfo.com/forum/viewtopic.php?t=24977
uint16_t ccitt_crc16 (const uint8_t *buf, uint_fast16_t len)
{
    uint16_t x, crc = 0;

    while(len--) {
        x = (crc >> 8) ^ *buf++;
        x ^= x >> 4;
        crc = (crc << 8) ^ (x << 12) ^ (x << 5) ^ x;
    }

    return crc;
}
