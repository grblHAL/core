/*
  stream_json.h - stream data to file as json structure

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

#include <stdlib.h>
#include <stdbool.h>

struct json_out; // members defined in stream_json.c
typedef struct json_out json_out_t;

json_out_t *json_start (vfs_file_t *file, uint32_t max_levels);
bool json_end (json_out_t *json);
bool json_start_object (json_out_t *json);
bool json_start_tagged_object (json_out_t *json, const char *tag);
bool json_end_object (json_out_t *json);
bool json_start_array (json_out_t *json, const char *tag);
bool json_end_array (json_out_t *json);
bool json_add_string (json_out_t *json, const char *tag, const char *s);
bool json_add_int (json_out_t *json, const char *tag, int32_t value);
