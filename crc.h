/*

  crc.h - crc implementations used by grblHAL

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

#pragma once

uint16_t grbl_crc8 (const uint8_t *data, uint32_t size);
uint16_t modbus_crc16x (const uint8_t *buf, uint_fast16_t len);
uint16_t ccitt_crc16 (const uint8_t *buf, uint_fast16_t len);
