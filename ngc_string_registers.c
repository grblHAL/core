/*
  ngc_params.c - get/set NGC parameter value by id or name

  Part of grblHAL

  Copyright (c) 2021-2024 Terje Io

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

/*
  All predefined parameters defined in NIST RS274NGC version 3 (ref section 3.2.1) are implemented.
  Most additional predefined parameters defined by LinuxCNC (ref section 5.2.3.1) are implemented.
*/

#include "hal.h"

#if NGC_STRING_REGISTERS_ENABLE

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "system.h"
#include "settings.h"
#include "ngc_params.h"
#include "ngc_string_registers.h"



typedef float (*ngc_param_get_ptr)(ngc_param_id_t id);
typedef float (*ngc_named_param_get_ptr)(void);

#ifndef NGC_MAX_SR_LENGTH
#define NGC_MAX_SR_LENGTH 50
#endif
typedef struct ngc_string_register {
    ngc_string_register_id_t id;
    char value[NGC_MAX_SR_LENGTH + 1];
    struct ngc_string_register *next;
} ngc_string_register_t;


static ngc_string_register_t *ngc_string_registers = NULL;

ngc_string_register_t *find_string_register (ngc_string_register_id_t id) {
    ngc_string_register_t *current_register = ngc_string_registers;
    while(current_register) {
        if(current_register->id == id) {
            return current_register;
        } else {
            current_register = current_register->next;
        }
    }

    return NULL;
}

ngc_string_register_t *find_string_register_with_last (ngc_string_register_id_t id, ngc_string_register_t **last_register) {
    ngc_string_register_t *current_register = ngc_string_registers;
    while(current_register) {
        if(current_register->id == id) {
            return current_register;
        } else {
            *last_register = current_register;
            current_register = current_register->next;
        }
    }

    return NULL;
}

bool ngc_string_register_get (ngc_string_register_id_t id, char **value) {
    ngc_string_register_t *string_register = find_string_register(id);
    if (string_register != NULL) {
        *value = &string_register->value[0];
        return true;
    }

    return false;
}

bool ngc_string_register_exists (ngc_string_register_id_t id) {
    return find_string_register(id) != NULL;
}

bool ngc_string_register_set (ngc_param_id_t id, char *value) {
    ngc_string_register_t *last_register = NULL;
    ngc_string_register_t *string_register = find_string_register_with_last(id, &last_register);
    if (string_register != NULL) {
        strcpy(string_register->value, value);
        return true;
    } else if ((string_register = malloc(sizeof(ngc_string_register_t)))) {
        string_register->id = id;
        strcpy(string_register->value, value);
        string_register->next = NULL;
        last_register->next = string_register;
        return true;
    }

    return false;
}

#endif // NGC_STRING_REGISTERS_ENABLE
