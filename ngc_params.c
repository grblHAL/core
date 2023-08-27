/*
  ngc_params.c - get/set NGC parameter value by id or name

  Part of grblHAL

  Copyright (c) 2021-2023 Terje Io

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
*/

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "system.h"
#include "settings.h"
#include "ngc_params.h"

#define MAX_PARAM_LENGTH 20

typedef float (*ngc_param_get_ptr)(ngc_param_id_t id);
typedef float (*ngc_named_param_get_ptr)(void);

typedef struct {
    ngc_param_id_t id_min;
    ngc_param_id_t id_max;
    ngc_param_get_ptr get;
} ngc_ro_param_t;

typedef struct ngc_rw_param {
    ngc_param_id_t id;
    float value;
    struct ngc_rw_param *next;
} ngc_rw_param_t;

typedef struct {
    const char *name;
    ncg_name_param_id_t id;
    ngc_named_param_get_ptr get;
} ngc_named_ro_param_t;

typedef struct ngc_named_rw_param {
    char name[MAX_PARAM_LENGTH + 1];
    float value;
    struct ngc_named_rw_param *next;
} ngc_named_rw_param_t;

ngc_rw_param_t *rw_params = NULL;
ngc_named_rw_param_t *rw_global_params = NULL;

static float _relative_pos (uint_fast8_t axis)
{
    float value;

    if(axis < N_AXIS) {
        value = sys.position[axis] / settings.axis[axis].steps_per_mm - gc_get_offset(axis);
        if(settings.flags.report_inches)
            value *= 25.4f;
    } else
        value = 0.0f;

    return value;
}

// numbered parameters

static float probe_coord (ngc_param_id_t id)
{
    float value = 0.0f;
    uint_fast8_t axis = (id % 10) - 1;
    coord_system_t data;

    if(axis < N_AXIS && (sys.probe_coordsys_id == gc_state.modal.coord_system.id || settings_read_coord_data(sys.probe_coordsys_id, &data.xyz))) {
        value = sys.probe_position[axis] / settings.axis[axis].steps_per_mm -
                 (sys.probe_coordsys_id == gc_state.modal.coord_system.id ? gc_state.modal.coord_system.xyz[axis] : data.xyz[axis]);
        if(settings.flags.report_inches)
            value *= 25.4f;
    }

    return value;
}

static float scaling_factors (ngc_param_id_t id)
{
    float *factors = gc_get_scaling();
    uint_fast8_t axis = id % 10;

    return axis <= N_AXIS ? factors[axis - 1] : 0.0f;
}

static float probe_result (ngc_param_id_t id)
{
    return sys.flags.probe_succeeded ? 1.0f : 0.0f;
}
/*
static float home_pos (ngc_param_id_t id)
{
    uint_fast8_t axis = id % 10;

    return axis <= N_AXIS ? sys.home_position[axis - 1] : 0.0f;
}
*/
static float m66_result (ngc_param_id_t id)
{
    return (float)sys.var5399;
}

static float tool_number (ngc_param_id_t id)
{
    return (float)gc_state.tool->tool_id;
}

static float tool_offset (ngc_param_id_t id)
{
    uint_fast8_t axis = id % 10;

    return axis <= N_AXIS ? gc_state.tool_length_offset[axis] : 0.0f;
}

static float g28_home (ngc_param_id_t id)
{
    float value = 0.0f;
    uint_fast8_t axis = id % 10;
    coord_system_t data;

    if(axis <= N_AXIS && settings_read_coord_data(CoordinateSystem_G28, &data.xyz))
        value = data.xyz[axis - 1];

    return value;
}

static float g30_home (ngc_param_id_t id)
{
    float value = 0.0f;
    uint_fast8_t axis = id % 10;
    coord_system_t data;

#if COMPATIBILITY_LEVEL > 1
    if(id <= CoordinateSystem_G59) {
#endif
    if (axis <= N_AXIS && settings_read_coord_data(CoordinateSystem_G30, &data.xyz))
        value = data.xyz[axis - 1];
#if COMPATIBILITY_LEVEL > 1
    }
#endif

    return value;
}

static float coord_system (ngc_param_id_t id)
{
    return (float)gc_state.modal.coord_system.id + 1;
}

static float coord_system_offset (ngc_param_id_t id)
{
    float value = 0.0f;
    uint_fast8_t axis = id % 10;
    coord_system_t data;

    id = (id - 5220 - axis - (id == 0 ? 10 : 0)) / 20;

    if (axis > 0 && axis <= N_AXIS && settings_read_coord_data((coord_system_id_t)id, &data.xyz))
        value = data.xyz[axis - 1];

    return value;
}

static float g92_offset_applied (ngc_param_id_t id)
{
    return (float)gc_state.g92_coord_offset_applied;
}

static float g92_offset (ngc_param_id_t id)
{
    uint_fast8_t axis = id % 10;

    return axis <= N_AXIS ? gc_state.g92_coord_offset [axis - 1] : 0.0f;
}

static float work_position (ngc_param_id_t id)
{
    float value = 0.0f;
    uint_fast8_t axis = id % 10;

    if(axis < N_AXIS)
        value = _relative_pos(axis);

    return value;
}

PROGMEM static const ngc_ro_param_t ngc_ro_params[] = {
    { .id_min = 5061, .id_max = 5069, .get = probe_coord },        // LinuxCNC
    { .id_min = 5070, .id_max = 5070, .get = probe_result },       // LinuxCNC
    { .id_min = 5161, .id_max = 5169, .get = g28_home },
    { .id_min = 5181, .id_max = 5189, .get = g30_home },
    { .id_min = 5191, .id_max = 5199, .get = scaling_factors },    // Mach3
    { .id_min = 5210, .id_max = 5210, .get = g92_offset_applied }, // LinuxCNC
    { .id_min = 5211, .id_max = 5219, .get = g92_offset },
    { .id_min = 5220, .id_max = 5220, .get = coord_system },
    { .id_min = 5221, .id_max = 5230, .get = coord_system_offset },
    { .id_min = 5241, .id_max = 5250, .get = coord_system_offset },
    { .id_min = 5261, .id_max = 5270, .get = coord_system_offset },
    { .id_min = 5281, .id_max = 5290, .get = coord_system_offset },
    { .id_min = 5301, .id_max = 5310, .get = coord_system_offset },
    { .id_min = 5321, .id_max = 5230, .get = coord_system_offset },
    { .id_min = 5341, .id_max = 5350, .get = coord_system_offset },
    { .id_min = 5361, .id_max = 5370, .get = coord_system_offset },
    { .id_min = 5381, .id_max = 5390, .get = coord_system_offset },
    { .id_min = 5399, .id_max = 5399, .get = m66_result },         // LinuxCNC
    { .id_min = 5400, .id_max = 5400, .get = tool_number },        // LinuxCNC
    { .id_min = 5401, .id_max = 5409, .get = tool_offset },        // LinuxCNC
    { .id_min = 5420, .id_max = 5428, .get = work_position }       // LinuxCNC
};

bool ngc_param_get (ngc_param_id_t id, float *value)
{
    bool found = id > 0 && id < ngc_ro_params[0].id_min;
    uint_fast8_t idx = sizeof(ngc_ro_params) / sizeof(ngc_ro_param_t);

    *value = 0.0f;

    if(found) {
        ngc_rw_param_t *rw_param = rw_params;
        while(rw_param) {
            if(rw_param->id == id) {
                *value = rw_param->value;
                rw_param = NULL;
            } else
                rw_param = rw_param->next;
        }
    } else do {
        idx--;
        if((found = id >= ngc_ro_params[idx].id_min && id <= ngc_ro_params[idx].id_max))
            *value = ngc_ro_params[idx].get(id);
    } while(idx && !found);

    return found;
}

bool ngc_param_is_rw (ngc_param_id_t id)
{
    return id > 0 && id < ngc_ro_params[0].id_min;
}

bool ngc_param_exists (ngc_param_id_t id)
{
    return id > 0 && id <= ngc_ro_params[(sizeof(ngc_ro_params) / sizeof(ngc_ro_param_t)) - 1].id_max;
}

bool ngc_param_set (ngc_param_id_t id, float value)
{
    bool ok = id > 0 && id < ngc_ro_params[0].id_min;

    if(ok) {

        ngc_rw_param_t *rw_param = rw_params, *rw_param_last = rw_params;

        while(rw_param) {
            if(rw_param->id == id) {
                break;
            } else {
                rw_param_last = rw_param;
                rw_param = rw_param->next;
            }
        }

        if(rw_param == NULL && value != 0.0f && (rw_param = malloc(sizeof(ngc_rw_param_t)))) {
            rw_param->id = id;
            rw_param->next = NULL;
            if(rw_params == NULL)
                rw_params = rw_param;
            else
                rw_param_last->next = rw_param;
        }

        if(rw_param)
            rw_param->value = value;
        else
            ok = value == 0.0f;
    }

    return ok;
}

PROGMEM static const ngc_named_ro_param_t ngc_named_ro_param[] = {
    { .name = "_vmajor",              .id = NGCParam_vmajor },
    { .name = "_vminor",              .id = NGCParam_vminor },
    { .name = "_line",                .id = NGCParam_line },
    { .name = "_motion_mode",         .id = NGCParam_motion_mode },
    { .name = "_plane",               .id = NGCParam_plane },
    { .name = "_ccomp",               .id = NGCParam_ccomp },
    { .name = "_metric",              .id = NGCParam_metric },
    { .name = "_imperial",            .id = NGCParam_imperial },
    { .name = "_absolute",            .id = NGCParam_absolute },
    { .name = "_incremental",         .id = NGCParam_incremental },
    { .name = "_inverse_time",        .id = NGCParam_inverse_time },
    { .name = "_units_per_minute",    .id = NGCParam_units_per_minute },
    { .name = "_units_per_rev",       .id = NGCParam_units_per_rev },
    { .name = "_coord_system",        .id = NGCParam_coord_system },
    { .name = "_tool_offset",         .id = NGCParam_tool_offset },
    { .name = "_retract_r_plane",     .id = NGCParam_retract_r_plane },
    { .name = "_retract_old_z",       .id = NGCParam_retract_old_z },
    { .name = "_spindle_rpm_mode",    .id = NGCParam_spindle_rpm_mode },
    { .name = "_spindle_css_mode",    .id = NGCParam_spindle_css_mode },
    { .name = "_ijk_absolute_mode",   .id = NGCParam_ijk_absolute_mode },
    { .name = "_lathe_diameter_mode", .id = NGCParam_lathe_diameter_mode },
    { .name = "_lathe_radius_mode",   .id = NGCParam_lathe_radius_mode },
    { .name = "_spindle_on",          .id = NGCParam_spindle_on },
    { .name = "_spindle_cw",          .id = NGCParam_spindle_cw },
    { .name = "_mist",                .id = NGCParam_mist },
    { .name = "_flood",               .id = NGCParam_flood },
    { .name = "_speed_override",      .id = NGCParam_speed_override },
    { .name = "_feed_override",       .id = NGCParam_feed_override },
    { .name = "_adaptive_feed",       .id = NGCParam_adaptive_feed },
    { .name = "_feed_hold",           .id = NGCParam_feed_hold },
    { .name = "_feed",                .id = NGCParam_feed },
    { .name = "_rpm",                 .id = NGCParam_rpm },
    { .name = "_x",                   .id = NGCParam_x },
    { .name = "_y",                   .id = NGCParam_y },
    { .name = "_z",                   .id = NGCParam_z },
    { .name = "_a",                   .id = NGCParam_a },
    { .name = "_b",                   .id = NGCParam_b },
    { .name = "_c",                   .id = NGCParam_c },
    { .name = "_u",                   .id = NGCParam_u },
    { .name = "_v",                   .id = NGCParam_v },
    { .name = "_w",                   .id = NGCParam_w },
    { .name = "_current_tool",        .id = NGCParam_current_tool },
    { .name = "_current_pocket",      .id = NGCParam_current_pocket },
    { .name = "_selected_tool",       .id = NGCParam_selected_tool },
    { .name = "_selected_pocket",     .id = NGCParam_selected_pocket },
};


// Named parameters

float ngc_named_param_get_by_id (ncg_name_param_id_t id)
{
    float value;

    switch(id) {

        case NGCParam_vmajor:
            value = 1.1f;
            break;

        case NGCParam_vminor:
            value = 0.0f; // TODO: derive from version letter?
            break;

        case NGCParam_line:
            value = (float)gc_state.line_number;
            break;

        case NGCParam_motion_mode:
            value = (float)(gc_state.modal.motion * 10); // TODO: Fix G38.x
            break;

        case NGCParam_plane:
            value = (float)(170 + gc_state.modal.plane_select * 10);
            break;

        case NGCParam_ccomp:
            value = 400.0f;
            break;

        case NGCParam_metric:
            value = gc_state.modal.units_imperial ? 0.0f : 1.0f;
            break;

        case NGCParam_imperial:
            value = gc_state.modal.units_imperial ? 1.0f : 0.0f;
            break;

        case NGCParam_absolute:
            value = gc_state.modal.distance_incremental ? 0.0f : 1.0f;
            break;

        case NGCParam_incremental:
            value = gc_state.modal.distance_incremental ? 1.0f : 0.0f;
            break;

        case NGCParam_inverse_time:
            value = gc_state.modal.feed_mode == FeedMode_InverseTime ? 1.0f : 0.0f;
            break;

        case NGCParam_units_per_minute:
            value = gc_state.modal.feed_mode == FeedMode_UnitsPerMin ? 1.0f : 0.0f;
            break;

        case NGCParam_units_per_rev:
            value = gc_state.modal.feed_mode == FeedMode_UnitsPerRev ? 1.0f : 0.0f;
            break;

        case NGCParam_coord_system:
            {
                uint_fast16_t id = gc_state.modal.coord_system.id * 10;

                if(id > (CoordinateSystem_G59 * 10))
                    id = (CoordinateSystem_G59 * 10) + gc_state.modal.coord_system.id - CoordinateSystem_G59;

                value = (float)(540 + id);
            }
            break;

        case NGCParam_tool_offset:
            value = gc_state.modal.tool_offset_mode >= ToolLengthOffset_Enable ? 1.0f : 0.0f;
            break;

        case NGCParam_retract_r_plane:
            value = gc_state.modal.retract_mode == CCRetractMode_Previous ? 1.0f : 0.0f;
            break;

        case NGCParam_retract_old_z:
            value = gc_state.modal.retract_mode == CCRetractMode_RPos ? 1.0f : 0.0f;
            break;

        case NGCParam_spindle_rpm_mode:
            value = gc_state.modal.spindle.rpm_mode == SpindleSpeedMode_RPM ? 1.0f : 0.0f;
            break;

        case NGCParam_spindle_css_mode:
            value = gc_state.modal.spindle.rpm_mode == SpindleSpeedMode_CSS ? 1.0f : 0.0f;
            break;

        case NGCParam_ijk_absolute_mode:
            value = 0.0f;
            break;

        case NGCParam_lathe_diameter_mode:
            value = gc_state.modal.diameter_mode ? 1.0f : 0.0f;
            break;

        case NGCParam_lathe_radius_mode:
            value = gc_state.modal.diameter_mode ? 0.0f : 1.0f;
            break;

        case NGCParam_spindle_on:
            value = gc_state.modal.spindle.state.on ? 1.0f : 0.0f;
            break;

        case NGCParam_spindle_cw:
            value = gc_state.modal.spindle.state.ccw ? 1.0f : 0.0f;
            break;

        case NGCParam_mist:
            value = gc_state.modal.coolant.mist ? 1.0f : 0.0f;
            break;

        case NGCParam_flood:
            value = gc_state.modal.coolant.flood ? 1.0f : 0.0f;
            break;

        case NGCParam_speed_override:
            value = gc_state.modal.override_ctrl.spindle_rpm_disable ? 0.0f : 1.0f;
            break;

        case NGCParam_feed_override:
            value = gc_state.modal.override_ctrl.feed_rate_disable ? 0.0f : 1.0f;
            break;

        case NGCParam_adaptive_feed:
            value = 0.0f;
            break;

        case NGCParam_feed_hold:
            value = gc_state.modal.override_ctrl.feed_hold_disable ? 0.0f : 1.0f;
            break;

        case NGCParam_feed:
            value = gc_state.feed_rate;
            break;

        case NGCParam_rpm:
            value = gc_state.spindle.rpm;
            break;

        case NGCParam_x:
            //no break
        case NGCParam_y:
            //no break
        case NGCParam_z:
            //no break
        case NGCParam_a:
            //no break
        case NGCParam_b:
            //no break
        case NGCParam_c:
            //no break
        case NGCParam_u:
            //no break
        case NGCParam_v:
            //no break
        case NGCParam_w:
            value = _relative_pos(id - NGCParam_x);
            break;

        case NGCParam_current_tool:
            value = (float)gc_state.tool->tool_id;
            break;

        case NGCParam_current_pocket:
            value = 0.0f;
            break;

        case NGCParam_selected_tool:
            value = gc_state.tool_pending != gc_state.tool->tool_id ? (float)gc_state.tool_pending : -1.0f;
            break;

        case NGCParam_selected_pocket:
            value = 0.0f;
            break;

        default:
            value = NAN;
    }

    return value;
}

bool ngc_named_param_get (char *name, float *value)
{
    char c, *s = name;
    bool found = false;
    uint_fast8_t idx = sizeof(ngc_named_ro_param) / sizeof(ngc_named_ro_param_t);

    // Lowercase name
    while((c = *s))
        *s++ = LCAPS(c);

    *value = 0.0f;

     if(*name == '_') do {
        idx--;
        if((found = !strcmp(name, ngc_named_ro_param[idx].name)))
            *value = ngc_named_param_get_by_id(ngc_named_ro_param[idx].id);
    } while(idx && !found);

    if(!found) {
        ngc_named_rw_param_t *rw_param = rw_global_params;
        while(rw_param && !found) {
            if((found = !strcmp(rw_param->name, name)))
                *value = rw_param->value;
            else
                rw_param = rw_param->next;
        }
    }

    return found;
}

bool ngc_named_param_exists (char *name)
{
    char c, *s1 = name, *s2 = name;
    bool ok = false;
    uint_fast8_t idx = sizeof(ngc_named_ro_param) / sizeof(ngc_named_ro_param_t);

    // Lowercase name, remove control characters and spaces
    while((c = *s1++) && c > ' ')
        *s2++ = LCAPS(c);

    *s2 = '\0';

    // Check if name is supplied, return false if not.
    if((*name == '_' ? *(name + 1) : *name) == '\0')
        return false;

    // Check if it is a (read only) predefined parameter.
    if(*name == '_') do {
        idx--;
        ok = !strcmp(name, ngc_named_ro_param[idx].name);
    } while(idx && !ok);

    // If not predefined attempt to find it.
    if(!ok && rw_global_params && strlen(name) < MAX_PARAM_LENGTH) {

        ngc_named_rw_param_t *rw_param = rw_global_params;

        while(rw_param) {
            if((ok = !strcmp(rw_param->name, name)))
                break;
            rw_param = rw_param->next;
         }
     }

    return ok;
}

bool ngc_named_param_set (char *name, float value)
{
    char c, *s1 = name, *s2 = name;
    bool ok = false;
    uint_fast8_t idx = sizeof(ngc_named_ro_param) / sizeof(ngc_named_ro_param_t);

    // Lowercase name, remove control characters and spaces
    while((c = *s1++) && c > ' ')
        *s2++ = LCAPS(c);

    *s2 = '\0';

    // Check if name is supplied, return false if not.
    if((*name == '_' ? *(name + 1) : *name) == '\0')
        return false;

    // Check if it is a (read only) predefined parameter.
    if(*name == '_') do {
        idx--;
        ok = !strcmp(name, ngc_named_ro_param[idx].name);
    } while(idx && !ok);

    // If not predefined attempt to set it.
    if(!ok && (ok = strlen(name) < MAX_PARAM_LENGTH)) {

        ngc_named_rw_param_t *rw_param = rw_global_params, *rw_param_last = rw_global_params;

         while(rw_param) {
             if(!strcmp(rw_param->name, name)) {
                 break;
             } else {
                 rw_param_last = rw_param;
                 rw_param = rw_param->next;
             }
         }

         if(rw_param == NULL && (rw_param = malloc(sizeof(ngc_named_rw_param_t)))) {
             strcpy(rw_param->name, name);
             rw_param->next = NULL;
             if(rw_global_params == NULL)
                 rw_global_params = rw_param;
             else
                 rw_param_last->next = rw_param;
         }

         if((ok = rw_param != NULL))
             rw_param->value = value;
     }

    return ok;
}
