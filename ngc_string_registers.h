/*
  ngc_string_registers.h - get/set NGC string register value by id

  Part of grblHAL

  Copyright (c) 2024-2025 Stig-Rune Skansgård

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

#ifndef _NGC_STRING_REGISTERS_H_
#define _NGC_STRING_REGISTERS_H_

#include "gcode.h"



typedef uint16_t ngc_string_register_id_t;

uint8_t ngc_float_decimals (void);
bool ngc_string_register_get (ngc_string_register_id_t id, char **value);
bool ngc_string_register_set (ngc_string_register_id_t id, char *value);
bool ngc_string_register_exists (ngc_string_register_id_t id);

#endif // _NGC_STRING_REGISTERS_H_
