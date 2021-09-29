/*
  ngc_params.c - get/set NGC parameter value by id or name

  Part of grblHAL

  Copyright (c) 2021 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
  All predefined parameters defined in NIST RS274NGC version 3 (ref section 3.2.1) are implemented.
  Most additional predefined parameters defined by LinuxCNC (ref section 5.2.3.1) are implemented.
  Currently it is not possible to set any parameters or reference them from gcode.
*/

#ifndef _NGC_PARAMS_H_
#define _NGC_PARAMS_H_

typedef uint16_t ngc_param_id_t;

typedef struct {
    ngc_param_id_t id;
    float value;
} ngc_param_t;

bool ngc_param_get (ngc_param_id_t id, float *value);
bool ngc_param_set (ngc_param_id_t id, float value);
bool ngc_param_is_rw (ngc_param_id_t id);
bool ngc_param_exists (ngc_param_id_t id);
bool ngc_named_param_get (char *name, float *value);
bool ngc_named_param_set (char *name, float value);
bool ngc_named_param_exists (char *name);

#endif
