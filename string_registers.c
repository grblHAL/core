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
#include "ngc_expr.h"
#include "string_registers.h"

#ifndef MAX_SR_LENGTH
// 256 is max block-length in gcode, so probably reasonable
#define MAX_SR_LENGTH 256
#endif

static on_gcode_message_ptr on_gcode_comment;
static on_string_substitution_ptr on_string_substitution;

typedef struct string_register {
    string_register_id_t id;
    struct string_register *next;
    char value[1];
} string_register_t;

static string_register_t *string_registers = NULL;

string_register_t *find_string_register_with_last (string_register_id_t id, string_register_t **last_register) {
    string_register_t *current_register = string_registers;

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

status_code_t string_register_set (string_register_id_t id, char *value) {
    size_t length = strlen(value);
    if (length > MAX_SR_LENGTH) {
        return Status_ExpressionArgumentOutOfRange;
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

        return Status_OK;
    }

    return Status_FlowControlOutOfMemory;
}

status_code_t read_register_id(char* comment, uint_fast8_t* char_counter, string_register_id_t* value) {
    float register_id;
    if (comment[*char_counter] == '[') {
        if (ngc_eval_expression(comment, char_counter, &register_id) != Status_OK) {
            return Status_ExpressionSyntaxError;   // [Invalid expression syntax]
        }
    } else if (!read_float(comment, char_counter, &register_id)) {
        return Status_BadNumberFormat;   // [Expected register id]
    }

    *value = (string_register_id_t)register_id;
    return Status_OK;
}

/*! \brief Substitute references to string-registers in a string with their values.

_NOTE:_ The returned string must be freed by the caller.

\param comment pointer to the original comment string.
\param message pointer to a char pointer to receive the resulting string.
*/
char *sr_substitute_parameters (char *comment, char **message)
{
    size_t len = 0;
    string_register_id_t registerId;
    char *s, c;
    uint_fast8_t char_counter = 0;

    // Trim leading spaces
    while(*comment == ' ')
        comment++;

    // Calculate length of substituted string
    while((c = comment[char_counter++])) {
        if (c == '&') {
            if(read_register_id(comment, &char_counter, &registerId) == Status_OK) {
                char *strValue;
                if (string_register_get(registerId, &strValue)) {
                    len += strlen(strValue);
                } else {
                    len += 3; // "N/A"
                }
            } else {
                len += 3; // "N/A"
                report_message("unable to parse string register id", Message_Warning);
            }
        } else
            len++;
    }

    // Perform substitution
    if((s = *message = malloc(len + 1))) {
        *s = '\0';
        char_counter = 0;

        while((c = comment[char_counter++])) {
            if (c == '&') {
                if(read_register_id(comment, &char_counter, &registerId) == Status_OK) {
                    char *strValue;
                    if (string_register_get(registerId, &strValue)) {
                        strcat(s, strValue);
                    } else {
                        strcat(s, "N/A");
                    }
                } else {
                    strcat(s, "N/A");
                    report_message("unable to parse string register id", Message_Warning);
                }
                s = strchr(s, '\0');
            } else {
                *s++ = c;
                *s = '\0';
            }
        }
    }

    return *message;
}

status_code_t superOnGcodeComment(char* comment) {
    if(on_gcode_comment) {
        return on_gcode_comment(comment);
    } else {
        return Status_OK;
    }
}

static status_code_t onStringRegisterGcodeComment (char *comment)
{
    uint_fast8_t char_counter = 0;
    if (comment[char_counter++] == '&') {
        if(gc_state.skip_blocks)
            return Status_OK;

        string_register_id_t registerId;
        if (read_register_id(comment, &char_counter, &registerId) != Status_OK) {
            return Status_ExpressionSyntaxError;   // [Expected equal sign]
        }

        if (comment[char_counter++] != '=') {
            return Status_ExpressionSyntaxError;   // [Expected equal sign]
        }

        if (grbl.on_string_substitution) {
            char *strValue;
            grbl.on_string_substitution(comment + char_counter, &strValue);
            status_code_t srResult = string_register_set(registerId, strValue);
            free(strValue);
            return srResult;
        } else {
            return string_register_set(registerId, comment + char_counter);
        }
    }

    return superOnGcodeComment(comment);
}

char* onStringRegisterSubstitution(char *input, char **output) {
    char* result;
    if (on_string_substitution) {
        char *intermediate;
        on_string_substitution(input, &intermediate);
        result = sr_substitute_parameters(intermediate, output);
        free(intermediate);
    } else {
        result = sr_substitute_parameters(input, output);
    }

    return result;
}

void string_registers_init (void)
{
    static bool init_ok = false;

    if(!init_ok) {
        init_ok = true;
        on_gcode_comment = grbl.on_gcode_comment;
        grbl.on_gcode_comment = onStringRegisterGcodeComment;
        on_string_substitution = grbl.on_string_substitution;
        grbl.on_string_substitution = onStringRegisterSubstitution;
    }
}

#endif // STRING_REGISTERS_ENABLE
