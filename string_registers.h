/*
  string_registers.h - get/set NGC string register value by id

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

#ifndef _STRING_REGISTERS_H_
#define _STRING_REGISTERS_H_

#include "gcode.h"

typedef uint16_t string_register_id_t;

bool string_register_get (string_register_id_t id, char **value);
status_code_t string_register_set (string_register_id_t id, char *value);
bool string_register_exists (string_register_id_t id);
void string_registers_init(void);

#endif // _STRING_REGISTERS_H_
