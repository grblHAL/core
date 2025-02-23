/*
  ngc_params.h - get/set NGC parameter value by id or name

  Part of grblHAL

  Copyright (c) 2021-2025 Terje Io

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
  Currently it is not possible to set any parameters or reference them from gcode.
*/

#ifndef _NGC_PARAMS_H_
#define _NGC_PARAMS_H_

#include "gcode.h"

#ifndef NGC_MAX_PARAM_LENGTH
#define NGC_MAX_PARAM_LENGTH 30
#endif

#define NGC_MAX_PARAM_ID 65535

typedef uint16_t ngc_param_id_t;
typedef uint32_t ngc_string_id_t;

typedef struct {
    ngc_param_id_t id;
    float value;
} ngc_param_t;

typedef enum {
    NGCParam_vmajor,
    NGCParam_vminor,
    NGCParam_line,
    NGCParam_motion_mode,
    NGCParam_plane,
    NGCParam_ccomp,
    NGCParam_metric,
    NGCParam_imperial,
    NGCParam_absolute,
    NGCParam_incremental,
    NGCParam_inverse_time,
    NGCParam_units_per_minute,
    NGCParam_units_per_rev,
    NGCParam_coord_system,
    NGCParam_tool_offset,
    NGCParam_retract_r_plane,
    NGCParam_retract_old_z,
    NGCParam_spindle_rpm_mode,
    NGCParam_spindle_css_mode,
    NGCParam_ijk_absolute_mode,
    NGCParam_lathe_diameter_mode,
    NGCParam_lathe_radius_mode,
    NGCParam_spindle_on,
    NGCParam_spindle_cw,
    NGCParam_mist,
    NGCParam_flood,
    NGCParam_speed_override,
    NGCParam_feed_override,
    NGCParam_adaptive_feed,
    NGCParam_feed_hold,
    NGCParam_feed,
    NGCParam_rpm,
    NGCParam_x,
    NGCParam_y,
    NGCParam_z,
    NGCParam_a,
    NGCParam_b,
    NGCParam_c,
    NGCParam_u,
    NGCParam_v,
    NGCParam_w,
    NGCParam_abs_x,
    NGCParam_abs_y,
    NGCParam_abs_z,
    NGCParam_abs_a,
    NGCParam_abs_b,
    NGCParam_abs_c,
    NGCParam_abs_u,
    NGCParam_abs_v,
    NGCParam_abs_w,
    NGCParam_current_tool,
    NGCParam_current_pocket,
    NGCParam_selected_tool,
    NGCParam_selected_pocket,
    NGCParam_call_level,
    NGCParam_probe_state,
    NGCParam_toolsetter_state,
    NGCParam_Last
} ncg_name_param_id_t;

uint8_t ngc_float_decimals (void);
bool ngc_param_get (ngc_param_id_t id, float *value);
bool ngc_param_set (ngc_param_id_t id, float value);
bool ngc_param_is_rw (ngc_param_id_t id);
bool ngc_param_exists (ngc_param_id_t id);
bool ngc_named_param_get (char *name, float *value);
float ngc_named_param_get_by_id (ncg_name_param_id_t id);
bool ngc_named_param_set (char *name, float value);
bool ngc_named_param_exists (char *name);

bool ngc_string_param_set (ngc_param_id_t id, char *value);
ngc_string_id_t ngc_string_param_set_name (char *name);
char *ngc_string_param_get (ngc_string_id_t id);
bool ngc_string_param_exists (ngc_string_id_t id);
void ngc_string_param_delete (ngc_string_id_t id);

bool ngc_call_push (void *context);
bool ngc_call_pop (void);
uint_fast8_t ngc_call_level (void);
bool ngc_modal_state_save (gc_modal_t *state, bool auto_restore);
bool ngc_modal_state_restore (void);
void ngc_modal_state_invalidate (void);

#endif // _NGC_PARAMS_H_
