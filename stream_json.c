/*
  stream_json.c - stream data to file as json structure

  Part of grblHAL

  Copyright (c) 2025 Terje Io

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

#include <string.h>

#include "vfs.h"
#include "nuts_bolts.h"
#include "stream_json.h"

struct json_out {
   vfs_file_t *file;
   uint32_t level;
   uint32_t max_level;
   uint32_t elements[1];
};

json_out_t *json_start (vfs_file_t *file, uint32_t max_levels)
{
    json_out_t *handle = NULL;

    if(file && (handle = calloc(sizeof(json_out_t) + sizeof(uint32_t) * max_levels, 1))) {
        handle->file = file;
        handle->max_level = max_levels;
        vfs_write("{", 1, 1, handle->file);
    }

    return handle;
}

bool json_end (json_out_t *json)
{
    if(json) {
        vfs_write("}", 1, 1, json->file);
        free(json);
    }

    return json != NULL;
}

static bool add_tag (json_out_t *json, const char *tag)
{
    bool ok;

    if((ok = json->level < json->max_level)) {

        if(json->elements[json->level])
            vfs_write(",", 1, 1, json->file);

        json->elements[json->level]++;

        vfs_write("\"", 1, 1, json->file);
        vfs_puts(tag, json->file);
        vfs_write("\":", 2, 1, json->file);
    }

    return ok;
}


bool json_start_object (json_out_t *json)
{
    if(json->level < json->max_level) {
        if(json->elements[json->level])
            vfs_write(",", 1, 1, json->file);

        json->elements[json->level]++;

        vfs_write("{", 1, 1, json->file);
        json->elements[++json->level] = 0;
    }

    return json->level < json->max_level;
}

bool json_start_tagged_object (json_out_t *json, const char *tag)
{
    if(json->level < json->max_level) {
        add_tag(json, tag);
        vfs_write("{", 1, 1, json->file);
        json->elements[++json->level] = 0;
    }

    return json->level < json->max_level;
}

bool json_end_object (json_out_t *json)
{
    if(json->level < json->max_level) {
        vfs_write("}", 1, 1, json->file);
        json->level--;
    }

    return json->level < json->max_level;
}

bool json_start_array (json_out_t *json, const char *tag)
{
    if(json->level < json->max_level) {
        add_tag(json, tag);
        vfs_write("[", 1, 1, json->file);
        json->elements[++json->level] = 0;
    }

    return json->level < json->max_level;
}

bool json_end_array (json_out_t *json)
{
    if(json->level) {
        vfs_write("]", 1, 1, json->file);
        json->level--;
    }

    return json->level < json->max_level;
}

bool json_add_string (json_out_t *json, const char *tag, const char *s)
{
    bool ok;

    if((ok = add_tag(json, tag))) {

        char c, esc[2] = "\\";
        const char *s1 = s;
        uint8_t escape = 0;

        vfs_write("\"", 1, 1, json->file);

        for(s1 = s; *s1; s1++) {

            switch(*s1) {
                case '\"':
                case '\\':
                case '\b':
                case '\f':
                case '\n':
                case '\r':
                case '\t':
                    escape = 1;
                    break;
            }
        }

        if(!escape)
            vfs_puts(s, json->file);
        else {

            const char *s2 = NULL;

            s1 = s;

            while((c = *s++)) {

                esc[1] = '\0';

                switch(c) {

                    case '\"':
                    case '\\':
                        s2 = s1;
                        esc[1] = c;
                        break;

                    case '\b':
                        s2 = s1;
                        esc[1] = 'b';
                        break;

                    case '\f':
                        s2 = s1;
                        esc[1] = 'f';
                        break;

                    case '\n':
                        s2 = s1;
                        esc[1] = 'n';
                        break;

                    case '\r':
                        s2 = s1;
                        esc[1] = 'r';
                        break;

                    case '\t':
                        s2 = s1;
                        esc[1] = 't';
                        break;
                        break;
                }
            }

            if(s2) {
                vfs_write(s2, s2 - s1, 1, json->file);
                s1 = s;
                s2 = NULL;
                vfs_write(esc, 2, 1, json->file);
            } else if(c == '\0' && *s1 != '\0')
                vfs_puts(s1, json->file);
        }

        vfs_write("\"", 1, 1, json->file);
    }

    return ok;
}

bool json_add_int (json_out_t *json, const char *tag, int32_t value)
{
    bool ok;

    if((ok = add_tag(json, tag))) {
        char *v = uitoa((uint32_t)(value < 0 ? -value : value));
        if(value < 0)
            vfs_write("-", 1, 1, json->file);
        vfs_puts(v, json->file);
    }

    return ok;
}
