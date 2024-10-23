/*
  string_registers.c - get/set string register value by id or name

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

#include "hal.h"

#if STRING_REGISTERS_ENABLE

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "system.h"
#include "settings.h"
#include "ngc_params.h"
#include "string_registers.h"

#ifndef MAX_SR_LENGTH
// 256 is max block-length in gcode, so probably reasonable
#define MAX_SR_LENGTH 256
#endif
typedef struct string_register {
    string_register_id_t id;
    struct string_register *next;
    char value[1];
} string_register_t;

static string_register_t *string_registers = NULL;

string_register_t *find_string_register_with_last (string_register_id_t id, string_register_t **last_register) {
    string_register_t *current_register = string_registers;
    int i = 0;

    while(current_register != NULL) {
        if(current_register->id == id) {
            return current_register;
        } else {
            *last_register = current_register;
            current_register = current_register->next;
        }
    }

    return NULL;
}

string_register_t *find_string_register (string_register_id_t id) {
    string_register_t *last = NULL;
    return find_string_register_with_last(id, &last);
}

bool string_register_get (string_register_id_t id, char **value) {
    string_register_t *string_register = find_string_register(id);

    if (string_register != NULL) {
        *value = &string_register->value[0];
        return true;
    }

    return false;
}

bool string_register_exists (string_register_id_t id) {
    return find_string_register(id) != NULL;
}

bool string_register_set (ngc_param_id_t id, char *value) {
    size_t length = strlen(value);
    if (length > MAX_SR_LENGTH) {
        report_message("String register values cannot be longer than 40 characters", Message_Warning);
        return false;
    }

    string_register_t *last_register = NULL;
    string_register_t *string_register = find_string_register_with_last(id, &last_register);

    bool isNew = string_register == NULL;

    // if a string register is found, we realloc it to fit the new value. If not,
    // calling realloc with a null pointer should result in allocating new memory.
    if ((string_register = realloc(string_register, sizeof(string_register_t) + length))) {
        // Since realloc copies (or preserves) the old object,
        // we only need to write id and next if this is actually a new register
        if (isNew) {
            string_register->id = id;
            string_register->next = NULL;
        }

        strcpy(string_register->value, value);

        // Update the next-pointer of the last register before this one.
        // If none exists, update the string_registers-pointer.
        if (last_register != NULL) {
            last_register->next = string_register;
        } else {
            string_registers = string_register;
        }

        return true;
    }

    return false;
}

#endif // STRING_REGISTERS_ENABLE
