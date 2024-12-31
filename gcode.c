/*
  gcode.c - rs274/ngc parser.

  Part of grblHAL

  Copyright (c) 2017-2024 Terje Io
  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

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

#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "hal.h"
#include "motion_control.h"
#include "protocol.h"
#include "state_machine.h"

#if NGC_PARAMETERS_ENABLE
#include "ngc_params.h"
#endif

#if NGC_EXPRESSIONS_ENABLE
#include "ngc_expr.h"
#include "ngc_flowctrl.h"
//#include "string_registers.h"
#ifndef NGC_N_ASSIGN_PARAMETERS_PER_BLOCK
#define NGC_N_ASSIGN_PARAMETERS_PER_BLOCK 10
#endif
#endif

// NOTE: Max line number is defined by the g-code standard to be 99999. It seems to be an
// arbitrary value, and some GUIs may require more. So we increased it based on a max safe
// value when converting a float (7.2 digit precision) to an integer.
#define MAX_LINE_NUMBER 10000000
#define MAX_TOOL_NUMBER 4294967294 // Limited by max unsigned 32-bit value - 1

#define MACH3_SCALING

// Do not change, must be same as axis indices
#define I_VALUE X_AXIS
#define J_VALUE Y_AXIS
#define K_VALUE Z_AXIS

// Define modal groups internal bitfield for checking multiple command violations and tracking the
// type of command that is called in the block. A modal group is a group of g-code commands that are
// mutually exclusive, or cannot exist on the same line, because they each toggle a state or execute
// a unique motion. These are defined in the NIST RS274-NGC v3 g-code standard, available online,
// and are similar/identical to other g-code interpreters by manufacturers (Haas,Fanuc,Mazak,etc).
typedef union {
    uint32_t mask;
    struct {
        uint32_t G0 :1, //!< [G4,G10,G28,G28.1,G30,G30.1,G53,G92,G92.1] Non-modal
                 G1 :1, //!< [G0,G1,G2,G3,G33,G33.1,G38.2,G38.3,G38.4,G38.5,G76,G80] Motion
                 G2 :1, //!< [G17,G18,G19] Plane selection
                 G3 :1, //!< [G90,G91] Distance mode
                 G4 :1, //!< [G91.1] Arc IJK distance mode
                 G5 :1, //!< [G93,G94,G95] Feed rate mode
                 G6 :1, //!< [G20,G21] Units
                 G7 :1, //!< [G40] Cutter radius compensation mode. G41/42 NOT SUPPORTED.
                 G8 :1, //!< [G43,G43.1,G49] Tool length offset
                G10 :1, //!< [G98,G99] Return mode in canned cycles
                G11 :1, //!< [G50,G51] Scaling
                G12 :1, //!< [G54,G55,G56,G57,G58,G59,G59.1,G59.2,G59.3] Coordinate system selection
                G13 :1, //!< [G61] Control mode
                G14 :1, //!< [G96,G97] Spindle Speed Mode
                G15 :1, //!< [G7,G8] Lathe Diameter Mode

                 M4 :1, //!< [M0,M1,M2,M30] Stopping
                 M5 :1, //!< [M62,M63,M64,M65,M66,M67,M68] Aux I/O
                 M6 :1, //!< [M6] Tool change
                 M7 :1, //!< [M3,M4,M5] Spindle turning
                 M8 :1, //!< [M7,M8,M9] Coolant control
                 M9 :1, //!< [M49,M50,M51,M53,M56] Override control
                M10 :1; //!< User defined M commands
    };
} modal_groups_t;

typedef enum {
    AxisCommand_None = 0,
    AxisCommand_NonModal,
    AxisCommand_MotionMode,
    AxisCommand_ToolLengthOffset,
    AxisCommand_Scaling
} axis_command_t;

typedef struct {
    parameter_words_t parameter;
    modal_groups_t modal_group;
} word_bit_t;

typedef union {
    uint_fast8_t mask;
    struct {
        uint_fast8_t i :1,
                     j :1,
                     k :1;
    };
} ijk_words_t;

// Declare gc extern struct
parser_state_t gc_state;

#define FAIL(status) return(status);

static gc_thread_data thread;
static output_command_t *output_commands = NULL; // Linked list
static scale_factor_t scale_factor = {
    .ijk[X_AXIS] = 1.0f,
    .ijk[Y_AXIS] = 1.0f,
    .ijk[Z_AXIS] = 1.0f
#ifdef A_AXIS
  , .ijk[A_AXIS] = 1.0f
#endif
#ifdef B_AXIS
  , .ijk[B_AXIS] = 1.0f
#endif
#ifdef C_AXIS
  , .ijk[C_AXIS] = 1.0f
#endif
#ifdef U_AXIS
  , .ijk[U_AXIS] = 1.0f
#endif
#ifdef V_AXIS
  , .ijk[V_AXIS] = 1.0f
#endif
};

// Simple hypotenuse computation function.
inline static float hypot_f (float x, float y)
{
    return sqrtf(x * x + y * y);
}

inline static bool motion_is_lasercut (motion_mode_t motion)
{
    return motion == MotionMode_Linear || motion == MotionMode_CwArc || motion == MotionMode_CcwArc || motion == MotionMode_CubicSpline || motion == MotionMode_QuadraticSpline;
}

inline static bool no_word_value (char letter)
{
    return letter == '\0' || (letter >= 'A' && letter <= 'Z') || letter == '$';
}

parser_state_t *gc_get_state (void)
{
    return &gc_state;
}

static void set_spindle_override (spindle_t *spindle, bool disable)
{
    if(spindle->hal && spindle->hal->param->state.override_disable != disable) {
        if((spindle->state.override_disable = spindle->hal->param->state.override_disable = disable))
            spindle_set_override(gc_state.spindle->hal, DEFAULT_SPINDLE_RPM_OVERRIDE);
    }
}

static void set_scaling (float factor)
{
    uint_fast8_t idx = N_AXIS;
    axes_signals_t state = gc_get_g51_state();

    do {
        scale_factor.ijk[--idx] = factor;
#ifdef MACH3_SCALING
        scale_factor.xyz[idx] = 0.0f;
#endif
    } while(idx);

    gc_state.modal.scaling_active = factor != 1.0f;

    if(state.value != gc_get_g51_state().value)
        system_add_rt_report(Report_Scaling);
}

float *gc_get_scaling (void)
{
    return scale_factor.ijk;
}

axes_signals_t gc_get_g51_state (void)
{
    uint_fast8_t idx = N_AXIS;
    axes_signals_t scaled = {0};

    do {
        scaled.value <<= 1;
        if(scale_factor.ijk[--idx] != 1.0f)
            scaled.value |= 0x01;

    } while(idx);

    return scaled;
}

float gc_get_offset (uint_fast8_t idx, bool real_time)
{
    offset_id_t offset_id;

    if(real_time &&
        !(settings.status_report.machine_position && settings.status_report.sync_on_wco_change) &&
          (offset_id = st_get_offset_id()) >= 0)
        return gc_state.modal.coord_system.xyz[idx] + gc_state.offset_queue[offset_id].values[idx] + gc_state.tool_length_offset[idx];
    else
        return gc_state.modal.coord_system.xyz[idx] + gc_state.g92_coord_offset[idx] + gc_state.tool_length_offset[idx];
}

inline static float gc_get_block_offset (parser_block_t *gc_block, uint_fast8_t idx)
{
    return gc_block->modal.coord_system.xyz[idx] + gc_state.g92_coord_offset[idx] + gc_state.tool_length_offset[idx];
}

void gc_set_tool_offset (tool_offset_mode_t mode, uint_fast8_t idx, int32_t offset)
{
    bool tlo_changed = false;

    switch(mode) {

        case ToolLengthOffset_Cancel:
            idx = N_AXIS;
            do {
                idx--;
                tlo_changed |= gc_state.tool_length_offset[idx] != 0.0f;
                gc_state.tool_length_offset[idx] = 0.0f;
                if(grbl.tool_table.n_tools == 0)
                    gc_state.tool->offset[idx] = 0.0f;
            } while(idx);
            break;

        case ToolLengthOffset_EnableDynamic:
            {
                float new_offset = offset / settings.axis[idx].steps_per_mm;
                tlo_changed |= gc_state.tool_length_offset[idx] != new_offset;
                gc_state.tool_length_offset[idx] = new_offset;
                if(grbl.tool_table.n_tools == 0)
                    gc_state.tool->offset[idx] = new_offset;
            }
            break;

        default:
            break;
    }

    gc_state.modal.tool_offset_mode = mode;

    if(tlo_changed) {
        system_add_rt_report(Report_ToolOffset);
        system_flag_wco_change();
    }
}

plane_t *gc_get_plane_data (plane_t *plane, plane_select_t select)
{
    switch (select) {

        case PlaneSelect_XY:
            plane->axis_0 = X_AXIS;
            plane->axis_1 = Y_AXIS;
            plane->axis_linear = Z_AXIS;
            break;

        case PlaneSelect_ZX:
            plane->axis_0 = Z_AXIS;
            plane->axis_1 = X_AXIS;
            plane->axis_linear = Y_AXIS;
            break;

        default: // case PlaneSelect_YZ:
            plane->axis_0 = Y_AXIS;
            plane->axis_1 = Z_AXIS;
            plane->axis_linear = X_AXIS;
    }

    return plane;
}

#if ENABLE_ACCELERATION_PROFILES

//Acceleration Profiles for G187 P[x] in percent of maximum machine acceleration.
float gc_get_accel_factor (uint8_t profile)
{
    static const float lookup[] = {
        1.0f,   // 100% - Roughing - Max Acceleration Default
        0.8f,   // 80% - Semi Roughing
        0.6f,   // 60% - Semi Finish
        0.4f,   // 40% - Finish
        0.2f    // 20% - Slow AF Mode
    };

    return lookup[profile >= (sizeof(lookup) / sizeof(float)) ? 0 : profile];
}

#endif // ENABLE_ACCELERATION_PROFILES

void gc_init (bool stop)
{
#if COMPATIBILITY_LEVEL > 1
    memset(&gc_state, 0, sizeof(parser_state_t));
    gc_state.tool = &grbl.tool_table.tool[0];
    if(grbl.tool_table.n_tools == 0)
        memset(grbl.tool_table.tool, 0, sizeof(tool_data_t));
#else
    if(sys.cold_start) {
        memset(&gc_state, 0, sizeof(parser_state_t));
        gc_state.tool = &grbl.tool_table.tool[0];
        if(grbl.tool_table.n_tools == 0)
            memset(grbl.tool_table.tool, 0, sizeof(tool_data_t));
    } else {

        coord_system_id_t coord_system_id = gc_state.modal.coord_system.id;
        tool_offset_mode_t tool_offset_mode = gc_state.modal.tool_offset_mode;

        memset(&gc_state, 0, offsetof(parser_state_t, g92_coord_offset));
        gc_state.tool_pending = gc_state.tool->tool_id;
        if(hal.tool.select)
            hal.tool.select(gc_state.tool, false);

        if(stop) {
            // Restore offsets, tool offset mode
            gc_state.modal.coord_system.id = coord_system_id;
            gc_state.modal.tool_offset_mode = tool_offset_mode;
        }
    }
#endif

    // Clear any pending output commands
    while(output_commands) {
        output_command_t *next = output_commands->next;
        free(output_commands);
        output_commands = next;
    }

    // Load default override status
    gc_state.modal.override_ctrl = sys.override.control;

#if N_SYS_SPINDLE > 1
    gc_state.spindle = &gc_state.modal.spindle[0];
    gc_state.modal.spindle[0].hal = spindle_get(0);
#else
    gc_state.spindle = &gc_state.modal.spindle;
    gc_state.modal.spindle.hal = spindle_get(0);
#endif

    set_scaling(1.0f);

    // Load default G54 coordinate system.
    if (!settings_read_coord_data(gc_state.modal.coord_system.id, &gc_state.modal.coord_system.xyz))
        grbl.report.status_message(Status_SettingReadFail);

    if(sys.cold_start && !settings.flags.g92_is_volatile) {
        if(!settings_read_coord_data(CoordinateSystem_G92, &gc_state.g92_coord_offset))
            grbl.report.status_message(Status_SettingReadFail);
        else
            memcpy(&gc_state.offset_queue[gc_state.offset_id], &gc_state.g92_coord_offset, sizeof(coord_data_t));
    }

    if(grbl.on_wco_changed && (!sys.cold_start ||
                                !is0_position_vector(gc_state.modal.coord_system.xyz) ||
                                 !is0_position_vector(gc_state.g92_coord_offset)))
        grbl.on_wco_changed();

#if NGC_EXPRESSIONS_ENABLE
    ngc_flowctrl_init();
#endif
#if NGC_PARAMETERS_ENABLE
    ngc_modal_state_invalidate();
#endif
#if ENABLE_ACCELERATION_PROFILES
    gc_state.modal.acceleration_factor = gc_get_accel_factor(0); // Initialize machine with default
#endif

//    if(settings.flags.lathe_mode)
//        gc_state.modal.plane_select = PlaneSelect_ZX;

    if(grbl.on_parser_init)
        grbl.on_parser_init(&gc_state);
}

inline static bool is_single_spindle_block (parser_block_t *gc_block, modal_groups_t command_words)
{
    return gc_block->words.s ||
            (command_words.G1 && (gc_block->modal.motion == MotionMode_SpindleSynchronized ||
                                   gc_block->modal.motion == MotionMode_RigidTapping ||
                                    gc_block->modal.motion == MotionMode_Threading)) ||
             (command_words.G5 && gc_block->modal.feed_mode == FeedMode_UnitsPerRev) ||
               command_words.G14 ||
               (command_words.M9 && gc_block->override_command == Override_SpindleSpeed);
}

// Set dynamic laser power mode to PPI (Pulses Per Inch)
// Returns true if driver uses hardware implementation.
// Driver support for pulsing the laser on signal is required for this to work.
bool gc_laser_ppi_enable (uint_fast16_t ppi, uint_fast16_t pulse_length)
{
    gc_state.is_laser_ppi_mode = ppi > 0 && pulse_length > 0;

    return grbl.on_laser_ppi_enable && grbl.on_laser_ppi_enable(ppi, pulse_length);
}

spindle_t *gc_spindle_get (spindle_num_t spindle)
{
#if N_SYS_SPINDLE > 1
    return spindle < 0 ? gc_state.spindle : &gc_state.modal.spindle[spindle];
#else
    return &gc_state.modal.spindle;
#endif
}

void gc_spindle_off (void)
{
#if N_SYS_SPINDLE > 1
    uint_fast8_t idx;
    for(idx = 0; idx < N_SYS_SPINDLE; idx++) {
        memset(&gc_state.modal.spindle[idx], 0, offsetof(spindle_t, hal));
    }
#else
    memset(&gc_state.modal.spindle, 0, offsetof(spindle_t, hal));
#endif

    spindle_all_off();
    system_add_rt_report(Report_Spindle);
}

void gc_coolant (coolant_state_t state)
{
    gc_state.modal.coolant = state;
    hal.coolant.set_state(gc_state.modal.coolant);
    system_add_rt_report(Report_Coolant);
}

static void add_offset (void)
{
    gc_state.offset_id = (gc_state.offset_id + 1) & (MAX_OFFSET_ENTRIES - 1);
    memcpy(&gc_state.offset_queue[gc_state.offset_id], &gc_state.g92_coord_offset, sizeof(coord_data_t));
    system_flag_wco_change();
}

static tool_data_t *tool_get_pending (tool_id_t tool_id)
{
    static tool_data_t tool_data = {0};

    if(grbl.tool_table.n_tools)
        return &grbl.tool_table.tool[tool_id];

    memcpy(&tool_data, gc_state.tool, sizeof(tool_data_t));
    tool_data.tool_id = tool_id;

    return &tool_data;
}

static inline void tool_set (tool_data_t *tool)
{
    if(grbl.tool_table.n_tools)
        gc_state.tool = tool;
    else
        gc_state.tool->tool_id = tool->tool_id;
}

// Add output command to linked list
static bool add_output_command (output_command_t *command)
{
    output_command_t *add_cmd;

    if((add_cmd = malloc(sizeof(output_command_t)))) {

        memcpy(add_cmd, command, sizeof(output_command_t));

        if(output_commands == NULL)
            output_commands = add_cmd;
        else {
            output_command_t *cmd = output_commands;
            while(cmd->next)
                cmd = cmd->next;
            cmd->next = add_cmd;
        }
    }

    return add_cmd != NULL;
}

static status_code_t init_sync_motion (plan_line_data_t *pl_data, float pitch)
{
    if(pl_data->spindle.hal->get_data == NULL)
        FAIL(Status_GcodeUnsupportedCommand); // [Spindle not sync capable]

    pl_data->condition.inverse_time = Off;
    pl_data->feed_rate = gc_state.distance_per_rev = pitch;
    pl_data->spindle.css = NULL;                    // Switch off CSS.
    pl_data->overrides = sys.override.control;      // Use current override flags and
    pl_data->overrides.sync = On;                   // set to sync overrides on execution of motion.

    // Disable feed rate and spindle overrides for the duration of the cycle.
    pl_data->overrides.spindle_rpm_disable = sys.override.control.spindle_rpm_disable = On;
    pl_data->overrides.feed_rate_disable = sys.override.control.feed_rate_disable = On;
    pl_data->spindle.hal->param->override_pct = DEFAULT_SPINDLE_RPM_OVERRIDE;
    // TODO: need for gc_state.distance_per_rev to be reset on modal change?
    float feed_rate = pl_data->feed_rate * pl_data->spindle.hal->get_data(SpindleData_RPM)->rpm;

    if(feed_rate == 0.0f)
        FAIL(Status_GcodeSpindleNotRunning); // [Spindle not running]

    if(feed_rate > settings.axis[Z_AXIS].max_rate * 0.9f)
        FAIL(Status_GcodeMaxFeedRateExceeded); // [Feed rate too high]

    return Status_OK;
}

// Output and free previously allocated message
void gc_output_message (char *message)
{
    if(message) {

        if(grbl.on_gcode_message)
            grbl.on_gcode_message(message);

        if(*message)
            report_message(message, Message_Plain);

        free(message);
    }
}

#if NGC_PARAMETERS_ENABLE

static parameter_words_t g65_words = {0};

parameter_words_t gc_get_g65_arguments (void)
{
    return g65_words;
}

bool gc_modal_state_restore (gc_modal_t *copy)
{
    bool ok = false;

    if((ok = !!copy && !ABORTED)) {

        copy->auto_restore = false;
        copy->motion = gc_state.modal.motion;

        if(copy->coolant.value != gc_state.modal.coolant.value) {
            hal.coolant.set_state(copy->coolant);
            delay_sec(settings.safety_door.coolant_on_delay, DelayMode_SysSuspend);
        }

#if N_SYS_SPINDLE > 1
        uint_fast8_t idx = N_SYS_SPINDLE;
        spindle_t *spindle, *spindle_copy;
        do {
            if((spindle = &gc_state.modal.spindle[--idx])->hal) {
                spindle_copy = &copy->spindle[idx];
                if(!memcmp(spindle_copy, spindle, offsetof(spindle_t, hal)))
                    spindle_restore(spindle->hal, spindle_copy->state, spindle_copy->rpm);
            }
        } while(idx);
#else
        if(!memcmp(&copy->spindle, &gc_state.modal.spindle, offsetof(spindle_t, hal)))
            spindle_restore(gc_state.modal.spindle.hal, copy->spindle.state, copy->spindle.rpm);
#endif

        memcpy(&gc_state.modal, copy, sizeof(gc_modal_t));

        gc_state.feed_rate = gc_state.modal.feed_rate;
    }

    return ok;
}

#endif // NGC_PARAMETERS_ENABLE

// Remove whitespace, control characters, comments and if block delete is active block delete lines
// else the block delete character. Remaining characters are converted to upper case.
// If the driver handles message comments then the first is extracted and returned in a dynamically
// allocated memory block, the caller must free this after the message has been processed.

char *gc_normalize_block (char *block, status_code_t *status, char **message)
{
    char c, *s1, *s2, *comment = NULL;

    // Remove leading whitespace & control characters
    while(*block && *block <= ' ')
        block++;

    if(*block == ';' || (*block == '/' && sys.flags.block_delete_enabled)) {
        *block = '\0';
        return block;
    }

    if(*block == '/')
        block++;

    s1 = s2 = block;

    while((c = *s1) != '\0') {

        if(c > ' ') switch(c) {

            case ';':
                if(!comment) {
                    *s1 = '\0';
                    continue;
                }
                break;

            case '(':
                // TODO: generate error if a left parenthesis is found inside a comment...
                comment = s1 + 1;
                break;

            case ')':
                if(comment && !gc_state.skip_blocks) {

                    *s1 = '\0';
                    if(!hal.driver_cap.no_gcode_message_handling) {

                        if(message && *message == NULL) {

                            if(grbl.on_process_gcode_comment)
                                *message = grbl.on_process_gcode_comment(comment);

                            if(*message == NULL) {

                                size_t len = s1 - comment - 3;
                                if(!strncasecmp(comment, "MSG,", 4) && (*message = malloc(len))) {

                                    comment += 4;
                                    while(*comment == ' ') {
                                        comment++;
                                        len--;
                                    }
                                    memcpy(*message, comment, len);
                                }
                            }
                        }
                    }

                    if(*comment && (message == NULL || *message == NULL) && grbl.on_gcode_comment)
                        *status = grbl.on_gcode_comment(comment);
                }
                comment = NULL;
                break;

            default:
                if(comment == NULL)
                    *s2++ = CAPS(c);
                break;
        }
        s1++;
    }

    *s2 = '\0';

    return block;
}

// Parses and executes one block (line) of 0-terminated G-Code.
// In this function, all units and positions are converted and exported to internal functions
// in terms of (mm, mm/min) and absolute machine coordinates, respectively.

status_code_t gc_execute_block (char *block)
{
    static const parameter_words_t axis_words_mask = {
        .x = On,
        .y = On,
        .z = On
#ifdef A_AXIS
      , .a = On
#endif
#ifdef B_AXIS
      , .b = On
#endif
#ifdef C_AXIS
      , .c = On
#endif
#if LATHE_UVW_OPTION
      , .u = On
      , .v = On
      , .w = On
#else
#ifdef U_AXIS
      , .u = On
#endif
#ifdef V_AXIS
      , .v = On
#endif
#endif
    };

    static const parameter_words_t pq_words = {
        .p = On,
        .q = On
    };

    static const parameter_words_t ij_words = {
        .i = On,
        .j = On
    };

    static const parameter_words_t positive_only_words = {
        .d = On,
        .f = On,
        .h = On,
        .n = On,
        .o = On,
        .t = On,
        .s = On
    };

    static const modal_groups_t jog_groups = {
        .G0 = On,
        .G3 = On,
        .G6 = On
    };

    static parser_block_t gc_block;

#if NGC_EXPRESSIONS_ENABLE

    static const parameter_words_t o_label = {
        .o = On
    };

    static ngc_param_t ngc_params[NGC_N_ASSIGN_PARAMETERS_PER_BLOCK];

    uint_fast8_t ngc_param_count = 0;

    // NOTE: this array has to match the parameter_words_t order!
    PROGMEM static const gc_value_ptr_t gc_value_ptr[] = {
       { NULL, ValueType_NA }, // $
#if N_AXIS > 3
       { &gc_block.values.xyz[A_AXIS], ValueType_Float },
#else
       { NULL, ValueType_NA },
#endif
#if N_AXIS > 4
       { &gc_block.values.xyz[B_AXIS], ValueType_Float },
#else
       { NULL, ValueType_NA },
#endif
#if N_AXIS > 5
       { &gc_block.values.xyz[C_AXIS], ValueType_Float },
#else
       { NULL, ValueType_NA },
#endif
       { &gc_block.values.ijk[I_VALUE], ValueType_Float },
       { &gc_block.values.ijk[J_VALUE], ValueType_Float },
       { &gc_block.values.ijk[K_VALUE], ValueType_Float },
       { &gc_block.values.d, ValueType_Float },
       { &gc_block.values.e, ValueType_Float },
       { &gc_block.values.f, ValueType_Float },
       { NULL, ValueType_NA }, // G
       { &gc_block.values.h, ValueType_UInt32 },
       { NULL, ValueType_NA }, // L
       { &gc_block.values.m, ValueType_Float },
       { NULL, ValueType_NA }, // N
       { NULL, ValueType_NA }, // O
       { NULL, ValueType_NA }, // P
       { &gc_block.values.q, ValueType_Float },
       { &gc_block.values.r, ValueType_Float },
       { &gc_block.values.s, ValueType_Float },
       { &gc_block.values.t, ValueType_UInt32 },
#if N_AXIS > 6
       { &gc_block.values.xyz[U_AXIS], ValueType_Float },
#else
       { NULL, ValueType_NA },
#endif
#if N_AXIS > 7
       { &gc_block.values.xyz[V_AXIS], ValueType_Float },
#else
       { NULL, ValueType_NA },
#endif
       { NULL, ValueType_NA }, // W
       { &gc_block.values.xyz[X_AXIS], ValueType_Float },
       { &gc_block.values.xyz[Y_AXIS], ValueType_Float },
       { &gc_block.values.xyz[Z_AXIS], ValueType_Float }
    };

#endif

    char *message = NULL;
    status_code_t status = Status_OK;
    struct {
        float f;
        uint32_t o;
        float s;
        tool_id_t t;
    } single_meaning_value = {0};

    block = gc_normalize_block(block, &status, &message);

    if(status != Status_OK)
        FAIL(status);

    if(block[0] == '\0') {
        if(message)
            gc_output_message(message);
        return status;
    }

    // Determine if the line is a program start/end marker.
    // Old comment from protocol.c:
    // NOTE: This maybe installed to tell grblHAL when a program is running vs manual input,
    // where, during a program, the system auto-cycle start will continue to execute
    // everything until the next '%' sign. This will help fix resuming issues with certain
    // functions that empty the planner buffer to execute its task on-time.
    if (block[0] == CMD_PROGRAM_DEMARCATION && block[1] == '\0') {
        gc_state.file_run = !gc_state.file_run;
        if(message)
            gc_output_message(message);
        return Status_OK;
    }

  /* -------------------------------------------------------------------------------------
     STEP 1: Initialize parser block struct and copy current g-code state modes. The parser
     updates these modes and commands as the block line is parsed and will only be used and
     executed after successful error-checking. The parser block struct also contains a block
     values struct, word tracking variables, and a non-modal commands tracker for the new
     block. This struct contains all of the necessary information to execute the block. */

    memset(&gc_block, 0, sizeof(gc_block));                           // Initialize the parser block struct.
    memcpy(&gc_block.modal, &gc_state.modal, sizeof(gc_state.modal)); // Copy current modes

    bool set_tool = false, spindle_event = false;
    axis_command_t axis_command = AxisCommand_None;
    io_mcode_t port_command = (io_mcode_t)0;
    spindle_t *sspindle = gc_state.spindle;
    plane_t plane;

    // Initialize bitflag tracking variables for axis indices compatible operations.
    axes_signals_t axis_words = {0};    // XYZ tracking
    ijk_words_t ijk_words = {0};        // IJK tracking

    // Initialize command and value words and parser flags variables.
    modal_groups_t command_words = {0};         // Bitfield for tracking G and M command words. Also used for modal group violations.
    gc_parser_flags_t gc_parser_flags = {0};    // Parser flags for handling special cases.
    parameter_words_t user_words = {0};         // User M-code words "taken"

    // Determine if the line is a jogging motion or a normal g-code block.
    if (block[0] == '$') { // NOTE: `$J=` already parsed when passed to this function.
        // Set G1 and G94 enforced modes to ensure accurate error checks.
        gc_parser_flags.jog_motion = On;
        gc_block.modal.motion = MotionMode_Linear;
        gc_block.modal.feed_mode = FeedMode_UnitsPerMin;
        gc_block.spindle_modal.rpm_mode = SpindleSpeedMode_RPM;
        gc_block.values.n = JOG_LINE_NUMBER; // Initialize default line number reported during jog.
    }

  /* -------------------------------------------------------------------------------------
     STEP 2: Import all g-code words in the block. A g-code word is a letter followed by
     a number, which can either be a 'G'/'M' command or sets/assigns a command value. Also,
     perform initial error-checks for command word modal group violations, for any repeated
     words, and for negative values set for the value words F, N, P, T, and S. */

    uint_fast8_t char_counter = gc_parser_flags.jog_motion ? 3 /* Start parsing after `$J=` */ : 0;
    char letter;
    float value;
    uint32_t int_value = 0;
    uint_fast16_t mantissa = 0;
    user_mcode_type_t user_mcode = UserMCode_Unsupported;
    word_bit_t word_bit = { .parameter = {0}, .modal_group = {0} }; // Bit-value for assigning tracking variables

    while ((letter = block[char_counter++]) != '\0') { // Loop until no more g-code words in block.

        // Import the next g-code word, expecting a letter followed by a value. Otherwise, error out.

#if NGC_EXPRESSIONS_ENABLE

        status_code_t status;

        if(letter == '#') {

            if(gc_state.skip_blocks)
                return Status_OK;

            if(block[char_counter] == '<') {

                char name[NGC_MAX_PARAM_LENGTH + 1];

                if((status = ngc_read_name(block, &char_counter, name)) == Status_OK) {
                    if(block[char_counter++] != '=')
                        status = Status_BadNumberFormat;    // [Expected equal sign]
                    else if((status = ngc_read_real_value(block, &char_counter, &value)) == Status_OK) {
                        if(!ngc_named_param_set(name, value))
                            status = Status_BadNumberFormat;    // [Out of memory or attempt to write RO parameter]
                    } // else: [Expected value]
                } // else: [Expected parameter name]
            } else {

                float param;

                if((status = ngc_read_real_value(block, &char_counter, &param)) == Status_OK) {
                    if(!ngc_param_is_rw((ngc_param_id_t)param))
                        status = Status_GcodeValueOutOfRange;   // [Parameter does not exist or is read only]
                    else if(block[char_counter++] != '=')
                        status = Status_BadNumberFormat;   // [Expected equal sign]
                    else if((status = ngc_read_real_value(block, &char_counter, &value)) == Status_OK) {
                        if(ngc_param_count < NGC_N_ASSIGN_PARAMETERS_PER_BLOCK) {
                            ngc_params[ngc_param_count].id = (ngc_param_id_t)param;
                            ngc_params[ngc_param_count++].value = value;
                        } else
                            FAIL(Status_BadNumberFormat);   // [Too many parameters in block]
                    } // else: [Expected parameter value]
                } // else: [Expected parameter number]
            }

            if(status != Status_OK)
                FAIL(status);

            continue;
        } else if(letter == 'O') {

            gc_block.words.n = Off; // Hack to allow line number with O word

            if(block[char_counter] == '[') {
                int32_t value;
                if((status = ngc_read_integer_value(block, &char_counter, &value)) == Status_OK) {
                    gc_block.words.o = On;
                    gc_block.values.o = (uint32_t)value;
                    char_counter++;
                } else
                    FAIL(status);
            } else if(block[char_counter] == '<') {

                char o_slabel[NGC_MAX_PARAM_LENGTH + 1];
                if((status = ngc_read_name(block, &char_counter, o_slabel)) != Status_OK)
                    FAIL(status);
                gc_block.words.o = On;
                if((gc_block.values.o = ngc_string_param_set_name(o_slabel)) == 0)
                    FAIL(Status_FlowControlOutOfMemory);
                continue;
            }
        }

        if((gc_block.words.mask & o_label.mask) && (gc_block.words.mask & ~o_label.mask) == 0) {
            char_counter--;
            return ngc_flowctrl(gc_block.values.o, block, &char_counter, &gc_state.skip_blocks);
        }

        if((letter < 'A' && letter != '$') || letter > 'Z')
            FAIL(Status_ExpectedCommandLetter); // [Expected word letter]

        if(user_mcode == UserMCode_NoValueWords && no_word_value(block[char_counter]))
            value = NAN;
        else if((status = ngc_read_real_value(block, &char_counter, &value)) != Status_OK)
            return status;

        if(gc_state.skip_blocks && letter != 'O')
            return Status_OK;

        if(user_mcode != UserMCode_NoValueWords && isnan(value))
            FAIL(Status_BadNumberFormat);   // [Expected word value]

        g65_words.value = 0;
#else

        if((letter < 'A' && letter != '$') || letter > 'Z')
            FAIL(Status_ExpectedCommandLetter); // [Expected word letter]

        if(letter == 'O') {
            value = NAN;
            if((status = read_uint(block, &char_counter, &int_value)) != Status_OK)
                FAIL(status);
        } else if(!read_float(block, &char_counter, &value)) {
            if(user_mcode == UserMCode_NoValueWords)    // Valueless parameters allowed for user defined M-codes.
                value = NAN;                            // Parameter validation deferred to implementation.
            else
                FAIL(Status_BadNumberFormat);           // [Expected word value]
        }

#endif

        // Convert values to smaller uint8 significand and mantissa values for parsing this word.
        // NOTE: Mantissa is multiplied by 100 to catch non-integer command values. This is more
        // accurate than the NIST gcode requirement of x10 when used for commands, but not quite
        // accurate enough for value words that require integers to within 0.0001. This should be
        // a good enough compromise and catch most all non-integer errors. To make it compliant,
        // we would simply need to change the mantissa to int16, but this add compiled flash space.
        // Maybe update this later.
        if(!isnan(value)) {
            int_value = (uint32_t)truncf(value);
            mantissa = (uint_fast16_t)roundf(100.0f * (value - int_value));
        }
        // NOTE: Rounding must be used to catch small floating point errors.

        // Check if the g-code word is supported or errors due to modal group violations or has
        // been repeated in the g-code block. If ok, update the command or record its value.
        switch(letter) {

          /* 'G' and 'M' Command Words: Parse commands and check for modal group violations.
             NOTE: Modal group numbers are defined in Table 4 of NIST RS274-NGC v3, pg.20 */

            case 'G': // Determine 'G' command and its modal group

                user_mcode = UserMCode_Unsupported;
                word_bit.modal_group.mask = 0;

                switch(int_value) {

                    case 7: case 8:
                        if(settings.mode == Mode_Lathe) {
                            word_bit.modal_group.G15 = On;
                            gc_block.modal.diameter_mode = int_value == 7; // TODO: find specs for implementation, only affects X calculation? reporting? current position?
                        } else
                            FAIL(Status_GcodeUnsupportedCommand); // [G7 & G8 not supported]
                        break;

                    case 10: case 28: case 30: case 92:
                        // Check for G10/28/30/92 being called with G0/1/2/3/38 on same block.
                        // * G43.1 is also an axis command but is not explicitly defined this way.
                        if (mantissa == 0) { // Ignore G28.1, G30.1, and G92.1
                            if (axis_command)
                                FAIL(Status_GcodeAxisCommandConflict); // [Axis word/command conflict]
                            axis_command = AxisCommand_NonModal;
                        }
                        // No break. Continues to next line.

                    case 4: case 53:
                        word_bit.modal_group.G0 = On;
                        gc_block.non_modal_command = (non_modal_t)int_value;
                        if ((int_value == 28) || (int_value == 30)) {
                            if (!((mantissa == 0) || (mantissa == 10)))
                                FAIL(Status_GcodeUnsupportedCommand);
                            gc_block.non_modal_command += mantissa;
                            mantissa = 0; // Set to zero to indicate valid non-integer G command.
                        } else if (int_value == 92) {
                            if (!((mantissa == 0) || (mantissa == 10) || (mantissa == 20) || (mantissa == 30)))
                                FAIL(Status_GcodeUnsupportedCommand);
                            gc_block.non_modal_command += mantissa;
                            mantissa = 0; // Set to zero to indicate valid non-integer G command.
                        }
                        break;

                    case 33: case 76:
                        if(mantissa != 0)
                            FAIL(Status_GcodeUnsupportedCommand); // [G33.1 not yet supported]
                        if (axis_command)
                            FAIL(Status_GcodeAxisCommandConflict); // [Axis word/command conflict]
                        axis_command = AxisCommand_MotionMode;
                        word_bit.modal_group.G1 = On;
                        gc_block.modal.motion = (motion_mode_t)int_value;
//                        if(mantissa == 10)
//                            gc_block.modal.motion = MotionMode_RigidTapping;
                        gc_block.modal.canned_cycle_active = false;
                        break;

                    case 38:
                        if(!(hal.probe.get_state && ((mantissa == 20) || (mantissa == 30) || (mantissa == 40) || (mantissa == 50))))
                            FAIL(Status_GcodeUnsupportedCommand); // [probing not supported by driver or unsupported G38.x command]
                        int_value += (mantissa / 10) + 100;
                        mantissa = 0; // Set to zero to indicate valid non-integer G command.
                        //  No break. Continues to next line.

                    case 0: case 1: case 2: case 3: case 5:
                        // Check for G0/1/2/3/38 being called with G10/28/30/92 on same block.
                        // * G43.1 is also an axis command but is not explicitly defined this way.
                        if (axis_command)
                            FAIL(Status_GcodeAxisCommandConflict); // [Axis word/command conflict]
                        axis_command = AxisCommand_MotionMode;
                        // No break. Continues to next line.

                    case 80:
                        word_bit.modal_group.G1 = On;
                        if(int_value == 5 && mantissa != 0) {
                            if(mantissa == 10) {
                                gc_block.modal.motion = MotionMode_QuadraticSpline;
                                mantissa = 0; // Set to zero to indicate valid non-integer G command.
                            } else
                                FAIL(Status_GcodeUnsupportedCommand);
                        } else
                            gc_block.modal.motion = (motion_mode_t)int_value;
                        gc_block.modal.canned_cycle_active = false;
                        break;

                    case 73: case 81: case 82: case 83: case 85: case 86: case 89:
                        if (axis_command)
                            FAIL(Status_GcodeAxisCommandConflict); // [Axis word/command conflict]
                        axis_command = AxisCommand_MotionMode;
                        word_bit.modal_group.G1 = On;
                        gc_block.modal.canned_cycle_active = true;
                        gc_block.modal.motion = (motion_mode_t)int_value;
                        gc_parser_flags.canned_cycle_change = gc_block.modal.motion != gc_state.modal.motion;
                        break;

                    case 17: case 18: case 19:
                        word_bit.modal_group.G2 = On;
                        gc_block.modal.plane_select = (plane_select_t)(int_value - 17);
                        break;

                    case 90: case 91:
                        if (mantissa == 0) {
                            word_bit.modal_group.G3 = On;
                            gc_block.modal.distance_incremental = int_value == 91;
                        } else {
                            word_bit.modal_group.G4 = On;
                            if ((mantissa != 10) || (int_value == 90))
                                FAIL(Status_GcodeUnsupportedCommand); // [G90.1 not supported]
                            mantissa = 0; // Set to zero to indicate valid non-integer G command.
                            // Otherwise, arc IJK incremental mode is default. G91.1 does nothing.
                        }
                        break;

                    case 93: case 94:
                        word_bit.modal_group.G5 = On;
                        gc_block.modal.feed_mode = (feed_mode_t)(94 - int_value);
                        break;

                    case 95:
                        word_bit.modal_group.G5 = On;
                        gc_block.modal.feed_mode = FeedMode_UnitsPerRev;
                        break;

                    case 20: case 21:
                        word_bit.modal_group.G6 = On;
                        gc_block.modal.units_imperial = int_value == 20;
                        break;

                    case 40:
                        word_bit.modal_group.G7 = On;
                        // NOTE: Not required since cutter radius compensation is always disabled. Only here
                        // to support G40 commands that often appear in g-code program headers to setup defaults.
                        // gc_block.modal.cutter_comp = CUTTER_COMP_DISABLE; // G40
                        break;

                    case 43: case 49:
                        word_bit.modal_group.G8 = On;
                        // NOTE: The NIST g-code standard vaguely states that when a tool length offset is changed,
                        // there cannot be any axis motion or coordinate offsets updated. Meaning G43, G43.1, and G49
                        // all are explicit axis commands, regardless if they require axis words or not.
                        // NOTE: cannot find the NIST statement referenced above, changed to match LinuxCNC behaviour in build 20210513.
                        if(int_value == 49) // G49
                            gc_block.modal.tool_offset_mode = ToolLengthOffset_Cancel;
                        else if(mantissa == 0 && grbl.tool_table.n_tools) // G43
                            gc_block.modal.tool_offset_mode = ToolLengthOffset_Enable;
                        else if(mantissa == 20 && grbl.tool_table.n_tools) // G43.2
                            gc_block.modal.tool_offset_mode = ToolLengthOffset_ApplyAdditional;
                        else if(mantissa == 10) { // G43.1
                            if(axis_command)
                                FAIL(Status_GcodeAxisCommandConflict); // [Axis word/command conflict] }
                            axis_command = AxisCommand_ToolLengthOffset;
                            gc_block.modal.tool_offset_mode = ToolLengthOffset_EnableDynamic;
                        } else
                            FAIL(Status_GcodeUnsupportedCommand); // [Unsupported G43.x command]
                        mantissa = 0; // Set to zero to indicate valid non-integer G command.
                        break;

                    case 54: case 55: case 56: case 57: case 58: case 59:
                        word_bit.modal_group.G12 = On;
                        gc_block.modal.coord_system.id = (coord_system_id_t)(int_value - 54); // Shift to array indexing.
                        if(int_value == 59 && mantissa > 0) {
                            if(N_WorkCoordinateSystems == 9 && (mantissa == 10 || mantissa == 20 || mantissa == 30)) {
                                gc_block.modal.coord_system.id += mantissa / 10;
                                mantissa = 0;
                            } else
                                FAIL(Status_GcodeUnsupportedCommand); // [Unsupported G59.x command]
                        }
                        break;

#if ENABLE_PATH_BLENDING
                    case 61:
                        word_bit.modal_group.G13 = On;
                        if (mantissa != 0 || mantissa != 10)
                            FAIL(Status_GcodeUnsupportedCommand);
                        gc_block.modal.control = mantissa == 0 ? ControlMode_ExactPath : ControlMode_ExactStop;
                        break;

                    case 64:
                        word_bit.modal_group.G13 = On;
                        gc_block.modal.control = ControlMode_PathBlending; // G64
                        break;
#else
                    case 61:
                        word_bit.modal_group.G13 = On;
                        if (mantissa != 0) // [G61.1 not supported]
                            FAIL(Status_GcodeUnsupportedCommand);
                        break;
#endif

                    case 65: // NOTE: Mach 3/4 GCode
                        word_bit.modal_group.G0 = On;
                        gc_block.non_modal_command = (non_modal_t)int_value;
                        if(mantissa != 0 || grbl.on_macro_execute == NULL)
                            FAIL(Status_GcodeUnsupportedCommand);
                        break;

                    case 96: case 97:
                        if(settings.mode == Mode_Lathe) {
                            word_bit.modal_group.G14 = On;
                            gc_block.spindle_modal.rpm_mode = (spindle_rpm_mode_t)((int_value - 96) ^ 1);
                        } else
                            FAIL(Status_GcodeUnsupportedCommand);
                        break;

                    case 98: case 99:
                        word_bit.modal_group.G10 = On;
                        gc_block.modal.retract_mode = (cc_retract_mode_t)(int_value - 98);
                        break;

                    case 50: case 51:
                        axis_command = AxisCommand_Scaling;
                        word_bit.modal_group.G11 = On;
                        gc_block.modal.scaling_active = int_value == 51;
                        break;

#if ENABLE_ACCELERATION_PROFILES
                    case 187:
                        word_bit.modal_group.G0 = On;
                        gc_block.non_modal_command = (non_modal_t)int_value;
                        if(mantissa != 0)
                            FAIL(Status_GcodeUnsupportedCommand);
                        break;
#endif

                    default: FAIL(Status_GcodeUnsupportedCommand); // [Unsupported G command]
                } // end G-value switch

                if (mantissa > 0)
                    FAIL(Status_GcodeCommandValueNotInteger); // [Unsupported or invalid Gxx.x command]

                // Check for more than one command per modal group violations in the current block
                // NOTE: Variable 'word_bit' is always assigned, if the command is valid.
                if (command_words.mask & word_bit.modal_group.mask)
                    FAIL(Status_GcodeModalGroupViolation);

                command_words.mask |= word_bit.modal_group.mask;
                break;

            case 'M': // Determine 'M' command and its modal group

                if(gc_block.non_modal_command == NonModal_MacroCall) {

                    if(gc_block.words.m)
                        FAIL(Status_GcodeWordRepeated); // [Word repeated]

                    gc_block.values.m = value;
                    gc_block.words.m = On; // Flag to indicate parameter assigned.

                    continue;
                }

                if(mantissa > 0)
                    FAIL(Status_GcodeCommandValueNotInteger); // [No Mxx.x commands]

                user_mcode = UserMCode_Unsupported;
                word_bit.modal_group.mask = 0;

                switch(int_value) {

                    case 0: case 1: case 2: case 30: case 60:
                        word_bit.modal_group.M4 = On;
                        switch(int_value) {

                            case 0: // M0 - program pause
                                gc_block.modal.program_flow = ProgramFlow_Paused;
                                break;

                            case 1: // M1 - program pause
                                if(hal.signals_cap.stop_disable ? !hal.control.get_state().stop_disable : !sys.flags.optional_stop_disable)
                                    gc_block.modal.program_flow = ProgramFlow_OptionalStop;
                                break;

                            default: // M2, M30, M60 - program end and reset
                                gc_block.modal.program_flow = (program_flow_t)int_value;
                        }
                        break;

                    case 3: case 4: case 5:
                        word_bit.modal_group.M7 = On;
                        gc_block.spindle_modal.state.on = !(int_value == 5);
                        gc_block.spindle_modal.state.ccw = int_value == 4;
//                        sys.override_delay.spindle = On; TODO: only when spindle sync?
                        break;

                    case 6:
                        if(settings.tool_change.mode != ToolChange_Ignore) {
                            if(hal.stream.suspend_read || hal.tool.change)
                                word_bit.modal_group.M6 = On;
                            else
                                FAIL(Status_GcodeUnsupportedCommand); // [Unsupported M command]
                        }
                        break;

                    case 7: case 8: case 9:
                        word_bit.modal_group.M8 = On;
//                        sys.override_delay.coolant = On; TODO: ?
                        gc_parser_flags.set_coolant = On;
                        switch(int_value) {

                            case 7:
                                if(!hal.coolant_cap.mist)
                                    FAIL(Status_GcodeUnsupportedCommand);
                                gc_block.modal.coolant.mist = On;
                                break;

                            case 8:
                                // TODO: check driver cap?
                                gc_block.modal.coolant.flood = On;
                                break;

                            case 9:
                                gc_block.modal.coolant.value = 0;
                                break;
                        }
                        break;

                    case 56:
                        if(!settings.parking.flags.enable_override_control) // TODO: check if enabled?
                            FAIL(Status_GcodeUnsupportedCommand); // [Unsupported M command]
                        // no break
                    case 48: case 49: case 50: case 51: case 53:
                        word_bit.modal_group.M9 = On;
                        gc_block.override_command = (override_mode_t)int_value;
                        break;

                    case 61:
                        set_tool = true;
                        word_bit.modal_group.M6 = On; //??
                        break;

                    case 62:
                    case 63:
                    case 64:
                    case 65:
                        if(hal.port.digital_out == NULL || hal.port.num_digital_out == 0)
                            FAIL(Status_GcodeUnsupportedCommand); // [Unsupported M command]
                        word_bit.modal_group.M5 = On;
                        port_command = (io_mcode_t)int_value;
                        break;

                    case 66:
                        if(hal.port.wait_on_input == NULL || (hal.port.num_digital_in == 0 && hal.port.num_analog_in == 0))
                            FAIL(Status_GcodeUnsupportedCommand); // [Unsupported M command]
                        word_bit.modal_group.M5 = On;
                        port_command = (io_mcode_t)int_value;
                        break;

                    case 67:
                    case 68:
                        if(hal.port.analog_out == NULL || hal.port.num_analog_out == 0)
                            FAIL(Status_GcodeUnsupportedCommand); // [Unsupported M command]
                        word_bit.modal_group.M5 = On;
                        port_command = (io_mcode_t)int_value;
                        break;

#if NGC_PARAMETERS_ENABLE
                    case 70: case 71: case 72: case 73:
                        //word_bit.modal_group.G0 = On; ??
                        gc_block.state_action = (modal_state_action_t)int_value;
                        break;
#endif

                    case 99:
                        word_bit.modal_group.M4 = On;
                        gc_block.modal.program_flow = ProgramFlow_Return;
                        if(grbl.on_macro_return == NULL)
                            FAIL(Status_GcodeUnsupportedCommand);
                        break;

                    default:
                        if(grbl.user_mcode.check && (user_mcode = grbl.user_mcode.check((user_mcode_t)int_value))) {
                            gc_block.user_mcode = (user_mcode_t)int_value;
                            word_bit.modal_group.M10 = On;
                        } else
                            FAIL(Status_GcodeUnsupportedCommand); // [Unsupported M command]
                } // end M-value switch

                // Check for more than one command per modal group violations in the current block
                // NOTE: Variable 'word_bit' is always assigned, if the command is valid.
                if (command_words.mask & word_bit.modal_group.mask)
                    FAIL(Status_GcodeModalGroupViolation);

                command_words.mask |= word_bit.modal_group.mask;
                break;

            // NOTE: All remaining letters assign values.
            default:

                /* Non-Command Words: This initial parsing phase only checks for repeats of the remaining
                legal g-code words and stores their value. Error-checking is performed later since some
                words (I,J,K,L,P,R) have multiple connotations and/or depend on the issued commands. */

                word_bit.parameter.mask = 0;

                switch(letter) {

#ifdef A_AXIS
  #if !AXIS_REMAP_ABC2UVW
                    case 'A':
  #else
                    case 'U':
  #endif
                        axis_words.a = On;
                        word_bit.parameter.a = On;
                        gc_block.values.xyz[A_AXIS] = value;
                        break;
#else
                    case 'A':
                        word_bit.parameter.a = On;
                        gc_block.values.a = value;
                        break;
#endif

#ifdef B_AXIS
  #if !AXIS_REMAP_ABC2UVW
                    case 'B':
  #else
                    case 'V':
  #endif
                        axis_words.b = On;
                        word_bit.parameter.b = On;
                        gc_block.values.xyz[B_AXIS] = value;
                        break;
#else
                    case 'B':
                        word_bit.parameter.b = On;
                        gc_block.values.b = value;
                        break;
#endif

#ifdef C_AXIS
  #if !AXIS_REMAP_ABC2UVW
                  case 'C':
  #else
                  case 'W':
  #endif
                        axis_words.c = On;
                        word_bit.parameter.c = On;
                        gc_block.values.xyz[C_AXIS] = value;
                        break;
#else
                    case 'C':
                        word_bit.parameter.c = On;
                        gc_block.values.c = value;
                        break;
#endif

                    case 'D':
                        word_bit.parameter.d = On;
                        gc_block.values.d = value;
                        break;

                    case 'E':
                        word_bit.parameter.e = On;
                        gc_block.values.e = value;
                        break;

                    case 'F':
                        word_bit.parameter.f = On;
                        gc_block.values.f = value;
                        break;

                    case 'H':
                        if (mantissa > 0)
                            FAIL(Status_GcodeCommandValueNotInteger);
                        word_bit.parameter.h = On;
                        gc_block.values.h = isnan(value) ? 0xFFFFFFFF : int_value;
                        break;

                    case 'I':
                        ijk_words.i = On;
                        word_bit.parameter.i = On;
                        gc_block.values.ijk[I_VALUE] = value;
                        break;

                    case 'J':
                        ijk_words.j = On;
                        word_bit.parameter.j = On;
                        gc_block.values.ijk[J_VALUE] = value;
                        break;

                    case 'K':
                        ijk_words.k = On;
                        word_bit.parameter.k = On;
                        gc_block.values.ijk[K_VALUE] = value;
                        break;

                    case 'L':
                        if (mantissa > 0)
                            FAIL(Status_GcodeCommandValueNotInteger);
                        word_bit.parameter.l = On;
                        gc_block.values.l = isnan(value) ? 0xFF : (uint8_t)int_value;
                        break;

                    case 'N':
                        word_bit.parameter.n = On;
                        gc_block.values.n = (int32_t)truncf(value);
                        break;

                    case 'O':
                        if (mantissa > 0)
                            FAIL(Status_GcodeCommandValueNotInteger);
                        word_bit.parameter.o = On;
                        gc_block.values.o = int_value;
                        break;

                    case 'P': // NOTE: For certain commands, P value must be an integer, but none of these commands are supported.
                        word_bit.parameter.p = On;
                        gc_block.values.p = value;
                        break;

                    case 'Q': // may be used for user defined mcodes or G61,G76
                        word_bit.parameter.q = On;
                        gc_block.values.q = value;
                        break;

                    case 'R':
                        word_bit.parameter.r = On;
                        gc_block.values.r = value;
                        break;

                    case 'S':
                        word_bit.parameter.s = On;
                        gc_block.values.s = value;
                        break;

                    case 'T':
                        if(mantissa > 0)
                            FAIL(Status_GcodeCommandValueNotInteger);
                        if(int_value > (grbl.tool_table.n_tools ? grbl.tool_table.n_tools : MAX_TOOL_NUMBER))
                            FAIL(Status_GcodeIllegalToolTableEntry);
                        word_bit.parameter.t = On;
                        gc_block.values.t = isnan(value) ? 0xFFFFFFFF : int_value;
                        break;
#if LATHE_UVW_OPTION
                    case 'U':
                        axis_words.x = On;
                        word_bit.parameter.x = word_bit.parameter.u = On;
                        gc_block.values.uvw[X_AXIS] = value / 2.0f; // U is always a diameter
                        break;

                    case 'V':
                        axis_words.y = On;
                        word_bit.parameter.y = word_bit.parameter.v = On;
                        gc_block.values.uvw[Y_AXIS] = value;
                        break;

                    case 'W':
                        axis_words.z = On;
                        word_bit.parameter.z = word_bit.parameter.w = On;
                        gc_block.values.uvw[Z_AXIS] = value;
                        break;
#else

#ifdef U_AXIS
                    case 'U':
                        axis_words.u = On;
                        word_bit.parameter.u = On;
                        gc_block.values.xyz[U_AXIS] = value;
                        break;
#elif !AXIS_REMAP_ABC2UVW
                    case 'U':
                        word_bit.parameter.u = On;
                        gc_block.values.u = value;
                        break;
#endif

#ifdef V_AXIS
                    case 'V':
                        axis_words.v = On;
                        word_bit.parameter.v = On;
                        gc_block.values.xyz[V_AXIS] = value;
                        break;
#elif !AXIS_REMAP_ABC2UVW
                    case 'V':
                        word_bit.parameter.v = On;
                        gc_block.values.v = value;
                        break;
#endif

#if !AXIS_REMAP_ABC2UVW
                    case 'W':
                        word_bit.parameter.w = On;
                        gc_block.values.w = value;
                        break;
#endif

#endif // !LATHE_UVW_OPTION
                    case 'X':
                        axis_words.x = On;
                        word_bit.parameter.x = On;
                        gc_block.values.xyz[X_AXIS] = value;
                        break;

                    case 'Y':
                        axis_words.y = On;
                        word_bit.parameter.y = On;
                        gc_block.values.xyz[Y_AXIS] = value;
                        break;

                    case 'Z':
                        axis_words.z = On;
                        word_bit.parameter.z = On;
                        gc_block.values.xyz[Z_AXIS] = value;
                        break;

                    case '$':
                        if(mantissa > 0)
                            FAIL(Status_GcodeCommandValueNotInteger);
                        word_bit.parameter.$ = On;
                        gc_block.values.$ = (int32_t)value;
                        break;

                    default: FAIL(Status_GcodeUnsupportedCommand);

                } // end parameter letter switch

                // NOTE: Variable 'word_bit' is always assigned, if the non-command letter is valid.
                if (gc_block.words.mask & word_bit.parameter.mask)
                    FAIL(Status_GcodeWordRepeated); // [Word repeated]

                // Check for invalid negative values for words F, H, N, P, T, and S.
                // NOTE: Negative value check is done here simply for code-efficiency.
                if ((word_bit.parameter.mask & positive_only_words.mask) && value < 0.0f)
                    FAIL(Status_NegativeValue); // [Word value cannot be negative]

                gc_block.words.mask |= word_bit.parameter.mask; // Flag to indicate parameter assigned.

        } // end main letter switch
    }

    // Parsing complete!


  /* -------------------------------------------------------------------------------------
     STEP 3: Error-check all commands and values passed in this block. This step ensures all of
     the commands are valid for execution and follows the NIST standard as closely as possible.
     If an error is found, all commands and values in this block are dumped and will not update
     the active system g-code modes. If the block is ok, the active system g-code modes will be
     updated based on the commands of this block, and signal for it to be executed.

     Also, we have to pre-convert all of the values passed based on the modes set by the parsed
     block. There are a number of error-checks that require target information that can only be
     accurately calculated if we convert these values in conjunction with the error-checking.
     This relegates the next execution step as only updating the system g-code modes and
     performing the programmed actions in order. The execution step should not require any
     conversion calculations and would only require minimal checks necessary to execute.
  */

  /* NOTE: At this point, the g-code block has been parsed and the block line can be freed.
     NOTE: It's also possible, at some future point, to break up STEP 2, to allow piece-wise
     parsing of the block on a per-word basis, rather than the entire block. This could remove
     the need for maintaining a large string variable for the entire block and free up some memory.
     To do this, this would simply need to retain all of the data in STEP 1, such as the new block
     data struct, the modal group and value bitflag tracking variables, and axis array indices
     compatible variables. This data contains all of the information necessary to error-check the
     new g-code block when the EOL character is received. However, this would break grblHAL's startup
     lines in how it currently works and would require some refactoring to make it compatible.
  */

 /*
  * Order of execution as per RS274-NGC_3 table 8:
  *
  *      1. comment (includes message)
  *      2. set feed rate mode (G93, G94 - inverse time or per minute)
  *      3. set feed rate (F)
  *      4. set spindle speed (S)
  *      5. select tool (T)
  *      6. change tool (M6)
  *      7. spindle on or off (M3, M4, M5)
  *      8. coolant on or off (M7, M8, M9)
  *      9. enable or disable overrides (M48, M49, M50, M51, M53)
  *      10. dwell (G4)
  *      11. set active plane (G17, G18, G19)
  *      12. set length units (G20, G21)
  *      13. cutter radius compensation on or off (G40, G41, G42)
  *      14. cutter length compensation on or off (G43, G49)
  *      15. coordinate system selection (G54, G55, G56, G57, G58, G59, G59.1, G59.2, G59.3)
  *      16. set path control mode (G61, G61.1, G64)
  *      17. set distance mode (G90, G91)
  *      18. set retract mode (G98, G99)
  *      19. home (G28, G30) or
  *              change coordinate system data (G10) or
  *              set axis offsets (G92, G92.1, G92.2, G94).
  *      20. perform motion (G0 to G3, G33, G80 to G89) as modified (possibly) by G53
  *      21. stop and end (M0, M1, M2, M30, M60)
  */

  // [0. Non-specific/common error-checks and miscellaneous setup]:

    // If a G65 block remove axis and ijk words flags since values are to be passed unmodified.
    if(word_bit.modal_group.G0 && gc_block.non_modal_command == NonModal_MacroCall)
        axis_words.mask = ijk_words.mask = 0;

    // Determine implicit axis command conditions. Axis words have been passed, but no explicit axis
    // command has been sent. If so, set axis command to current motion mode.
    if (axis_words.mask && !axis_command)
        axis_command = AxisCommand_MotionMode; // Assign implicit motion-mode

    if(gc_state.tool_change && axis_command == AxisCommand_MotionMode && !gc_parser_flags.jog_motion)
        FAIL(Status_GcodeToolChangePending); // [Motions (except jogging) not allowed when changing tool]

    // Check for valid line number N value.
    // Line number value cannot be less than zero (done) or greater than max line number.
    if (gc_block.words.n && gc_block.values.n > MAX_LINE_NUMBER)
        FAIL(Status_GcodeInvalidLineNumber); // [Exceeds max line number]

    // bit_false(gc_block.words,bit(Word_N)); // NOTE: Single-meaning value word. Set at end of error-checking.

    // Track for unused words at the end of error-checking.
    // NOTE: Single-meaning value words are removed all at once at the end of error-checking, because
    // they are always used when present. This was done to save a few bytes of flash. For clarity, the
    // single-meaning value words may be removed as they are used. Also, axis words are treated in the
    // same way. If there is an explicit/implicit axis command, XYZ words are always used and are
    // are removed at the end of error-checking.

    // [0. User defined M commands ]:
    if(command_words.M10 && gc_block.user_mcode) {

        user_words.mask = gc_block.words.mask;
        if((int_value = (uint_fast16_t)grbl.user_mcode.validate(&gc_block)))
            FAIL((status_code_t)int_value);
        user_words.mask ^= gc_block.words.mask; // Flag "taken" words for execution

        if(user_words.i)
            ijk_words.i = Off;
        if(user_words.j)
            ijk_words.j = Off;
        if(user_words.k)
            ijk_words.k = Off;
        if(user_words.f) {
            single_meaning_value.f = gc_block.values.f;
            gc_block.values.f = 0.0f;
        }
        if(user_words.o) {
            single_meaning_value.o = gc_block.values.o;
            gc_block.values.o = 0;
        }
        if(user_words.s) {
            single_meaning_value.s = gc_block.values.s;
            gc_block.values.s = 0.0f;
        }
        if(user_words.t) {
            single_meaning_value.t = gc_block.values.t;
            gc_block.values.t = (tool_id_t)0;
        }
        axis_words.mask = 0;
    }

    // [1. Comments ]: MSG's may be supported by driver layer. Comment handling performed by protocol.

    // [2. Set feed rate mode ]: G93 F word missing with G1,G2/3 active, implicitly or explicitly. Feed rate
    //   is not defined after switching between G93, G94 and G95.
    // NOTE: For jogging, ignore prior feed rate mode. Enforce G94 and check for required F word.
    if (gc_parser_flags.jog_motion) {

        if(!gc_block.words.f)
            FAIL(Status_GcodeUndefinedFeedRate);

        if (gc_block.modal.units_imperial)
            gc_block.values.f *= MM_PER_INCH;

    } else if(gc_block.modal.motion == MotionMode_SpindleSynchronized) {

        if (!gc_block.words.k) {
            gc_block.values.k = gc_state.distance_per_rev;
        } else {
            gc_block.words.k = Off;
            gc_block.values.k = gc_block.modal.units_imperial ? gc_block.values.ijk[K_VALUE] *= MM_PER_INCH : gc_block.values.ijk[K_VALUE];
        }

    } else if (gc_block.modal.feed_mode == FeedMode_InverseTime) { // = G93
        // NOTE: G38 can also operate in inverse time, but is undefined as an error. Missing F word check added here.
        if (axis_command == AxisCommand_MotionMode) {
            if (!(gc_block.modal.motion == MotionMode_None || gc_block.modal.motion == MotionMode_Seek)) {
                if (!gc_block.words.f)
                    FAIL(Status_GcodeUndefinedFeedRate); // [F word missing]
            }
        }
        // NOTE: It seems redundant to check for an F word to be passed after switching from G94 to G93. We would
        // accomplish the exact same thing if the feed rate value is always reset to zero and undefined after each
        // inverse time block, since the commands that use this value already perform undefined checks. This would
        // also allow other commands, following this switch, to execute and not error out needlessly. This code is
        // combined with the above feed rate mode and the below set feed rate error-checking.

        // [3. Set feed rate ]: F is negative (done.)
        // - In inverse time mode: Always implicitly zero the feed rate value before and after block completion.
        // NOTE: If in G93 mode or switched into it from G94, just keep F value as initialized zero or passed F word
        // value in the block. If no F word is passed with a motion command that requires a feed rate, this will error
        // out in the motion modes error-checking. However, if no F word is passed with NO motion command that requires
        // a feed rate, we simply move on and the state feed rate value gets updated to zero and remains undefined.

    } else if (gc_block.modal.feed_mode == FeedMode_UnitsPerMin || gc_block.modal.feed_mode == FeedMode_UnitsPerRev) {
          // if F word passed, ensure value is in mm/min or mm/rev depending on mode, otherwise push last state value.
        if (!gc_block.words.f) {
            if(gc_block.modal.feed_mode == gc_state.modal.feed_mode)
                gc_block.values.f = gc_state.feed_rate; // Push last state feed rate
        } else if (gc_block.modal.units_imperial)
            gc_block.values.f *= MM_PER_INCH;
    } // else, switching to G94 from G93, so don't push last state feed rate. Its undefined or the passed F word value.

    // bit_false(gc_block.words,bit(Word_F)); // NOTE: Single-meaning value word. Set at end of error-checking.

    // [4. Set spindle speed and address spindle ]: S or D is negative (done.)
    if(gc_block.words.$) {
        bool single_spindle_only = is_single_spindle_block(&gc_block, command_words);
        if(command_words.M7 || single_spindle_only) {
            if(gc_block.values.$ < (single_spindle_only ? 0 : -1))
                FAIL(single_spindle_only ? Status_NegativeValue : Status_GcodeValueOutOfRange);
#if N_SYS_SPINDLE > 1
            if(gc_block.values.$ < 0)
                sspindle = NULL;
            else {
                if(!spindle_is_enabled(gc_block.values.$))
                    FAIL(Status_GcodeValueOutOfRange);
                if(gc_state.modal.spindle[gc_block.values.$].hal == NULL)
                    gc_state.modal.spindle[gc_block.values.$].hal = spindle_get(gc_block.values.$);
                sspindle = &gc_state.modal.spindle[gc_block.values.$];
            }
#else
            if(gc_block.values.$ > 0)
                FAIL(Status_GcodeValueOutOfRange);
#endif
            gc_block.words.$ = Off;
        }
    }
#if N_SYS_SPINDLE > 1
    // For now, remove when downstream code can handle multiple spindles?
    else if(command_words.M7 || is_single_spindle_block(&gc_block, command_words))
        sspindle = &gc_state.modal.spindle[0];
#endif

    if(gc_block.modal.feed_mode == FeedMode_UnitsPerRev && (sspindle == NULL || !sspindle->hal->get_data))
        FAIL(Status_GcodeUnsupportedCommand); // [G95 not supported]

    if(command_words.G14) {
        if(gc_block.spindle_modal.rpm_mode == SpindleSpeedMode_CSS) {
            if(!sspindle->hal->cap.variable)
                FAIL(Status_GcodeUnsupportedCommand);
            if(!gc_block.words.s) // TODO: add check for S0?
                FAIL(Status_GcodeValueWordMissing);
    // see below!! gc_block.values.s *= (gc_block.modal.units_imperial ? MM_PER_INCH * 12.0f : 1000.0f); // convert surface speed to mm/min
            if(gc_block.words.d) {
                sspindle->hal->param->css.max_rpm = min(gc_block.values.d, sspindle->hal->rpm_max);
                gc_block.words.d = Off;
            } else
                sspindle->hal->param->css.max_rpm = sspindle->hal->rpm_max;
        } else if(sspindle->rpm_mode == SpindleSpeedMode_CSS) {
            if(sspindle->css) {
                sspindle->css = NULL;
                protocol_buffer_synchronize(); // Empty planner buffer to ensure we get RPM at end of last CSS motion
            }
            sspindle->rpm = sspindle->hal->param->rpm; // Is it correct to restore latest spindle RPM here?
        }
        sspindle->rpm_mode = gc_block.spindle_modal.rpm_mode;
    } else if(sspindle)
        gc_block.spindle_modal.rpm_mode = sspindle->rpm_mode;

    if((spindle_event = gc_block.words.s)) {
        if(sspindle->rpm_mode == SpindleSpeedMode_CSS) {
            // Unsure what to do about S values when in SpindleSpeedMode_CSS - ignore? For now use it to (re)calculate surface speed.
            // Reinsert commented out code above if this is removed!!
            gc_block.values.s *= (gc_block.modal.units_imperial ? MM_PER_INCH * 12.0f : 1000.0f); // convert surface speed to mm/min
            sspindle->hal->param->css.surface_speed = gc_block.values.s;
        }
    } else if(sspindle)
        gc_block.values.s = sspindle->rpm_mode == SpindleSpeedMode_RPM ? sspindle->rpm : sspindle->hal->param->css.max_rpm;

    // bit_false(gc_block.words,bit(Word_S)); // NOTE: Single-meaning value word. Set at end of error-checking.

    // [5. Select tool ]: If not supported then only tracks value. T is negative (done.) Not an integer (done).
    if(set_tool) { // M61
        if(!gc_block.words.q)
            FAIL(Status_GcodeValueWordMissing);
        if(!isintf(gc_block.values.q))
            FAIL(Status_GcodeCommandValueNotInteger);
        if((uint32_t)gc_block.values.q > (grbl.tool_table.n_tools ? grbl.tool_table.n_tools : MAX_TOOL_NUMBER))
            FAIL(Status_GcodeIllegalToolTableEntry);

        gc_block.values.t = (uint32_t)gc_block.values.q;
        gc_block.words.q = Off;
#if NGC_EXPRESSIONS_ENABLE
        if(hal.stream.file) {
            gc_state.tool_pending = 0; // force set tool
            if(grbl.tool_table.n_tools) {
                if(gc_state.g43_pending) {
                    gc_block.values.h = gc_state.g43_pending;
                    command_words.G8 = On;
                }
                gc_state.g43_pending = 0;
            }
        }
#endif
    } else if(!gc_block.words.t)
        gc_block.values.t = gc_state.tool_pending;

    if(command_words.M5 && port_command) {

        switch(port_command) {

            case IoMCode_OutputOnSynced:
            case IoMCode_OutputOffSynced:
            case IoMCode_OutputOnImmediate:
            case IoMCode_OutputOffImmediate:
                if(!gc_block.words.p)
                    FAIL(Status_GcodeValueWordMissing);
                if(gc_block.values.p < 0.0f)
                    FAIL(Status_NegativeValue);
                if((uint32_t)gc_block.values.p + 1 > hal.port.num_digital_out)
                    FAIL(Status_GcodeValueOutOfRange);
                gc_block.output_command.is_digital = true;
                gc_block.output_command.port = (uint8_t)gc_block.values.p;
                gc_block.output_command.value = port_command == 62 || port_command == 64 ? 1.0f : 0.0f;
                gc_block.words.p = Off;
                break;

            case IoMCode_WaitOnInput:
                if(!(gc_block.words.l || gc_block.words.q))
                    FAIL(Status_GcodeValueWordMissing);

                if(gc_block.words.p && gc_block.words.e)
                    FAIL(Status_ValueWordConflict);

                if(gc_block.values.l >= (uint8_t)WaitMode_Max)
                    FAIL(Status_GcodeValueOutOfRange);

                if((wait_mode_t)gc_block.values.l != WaitMode_Immediate && gc_block.values.q == 0.0f)
                    FAIL(Status_GcodeValueOutOfRange);

                if(gc_block.words.p) {
                    if(gc_block.values.p < 0.0f)
                        FAIL(Status_NegativeValue);
                    if((uint32_t)gc_block.values.p + 1 > hal.port.num_digital_in)
                        FAIL(Status_GcodeValueOutOfRange);

                    gc_block.output_command.is_digital = true;
                    gc_block.output_command.port = (uint8_t)gc_block.values.p;
                }

                if(gc_block.words.e) {
                    if((uint32_t)gc_block.values.e + 1 > hal.port.num_analog_in)
                        FAIL(Status_GcodeValueOutOfRange);
                    if((wait_mode_t)gc_block.values.l != WaitMode_Immediate)
                        FAIL(Status_GcodeValueOutOfRange);

                    gc_block.output_command.is_digital = false;
                    gc_block.output_command.port = (uint8_t)gc_block.values.e;
                }

                gc_block.words.e = gc_block.words.l = gc_block.words.p = gc_block.words.q = Off;
                break;

            case IoMCode_AnalogOutSynced:
            case IoMCode_AnalogOutImmediate:
                if(!(gc_block.words.e || gc_block.words.q))
                    FAIL(Status_GcodeValueWordMissing);
                if((uint32_t)gc_block.values.e + 1 > hal.port.num_analog_out)
                    FAIL(Status_GcodeRPMOutOfRange);
                gc_block.output_command.is_digital = false;
                gc_block.output_command.port = (uint8_t)gc_block.values.e;
                gc_block.output_command.value = gc_block.values.q;
                gc_block.words.e = gc_block.words.q = Off;
            break;
        }
    }

    // bit_false(gc_block.words,bit(Word_T)); // NOTE: Single-meaning value word. Set at end of error-checking.

    // [6. Change tool ]: N/A

    // [7. Spindle control ]:
    if(command_words.M7) {
        if(gc_block.spindle_modal.state.ccw) {
            // Check if spindle(s) support reversing direction
#if N_SYS_SPINDLE > 1
            if(sspindle == NULL) {
                uint_fast8_t idx = N_SYS_SPINDLE;
                do {
                    idx--;
                    if(gc_state.modal.spindle[idx].hal && !(gc_state.modal.spindle[idx].hal->cap.direction || gc_state.modal.spindle[idx].hal->cap.laser))
                        FAIL(Status_GcodeUnsupportedCommand);
                } while(idx);
            } else
#endif
            if(!(sspindle->hal->cap.direction || sspindle->hal->cap.laser))
                FAIL(Status_GcodeUnsupportedCommand);
        }
    } else if(sspindle)
        gc_block.spindle_modal.state = sspindle->state;

    // [8. Coolant control ]: N/A

    // [9. Override control ]:
    if (command_words.M9) {

        if(!gc_block.words.p)
            gc_block.values.p = 1.0f;
        else {
            if(gc_block.values.p < 0.0f)
                FAIL(Status_NegativeValue);
            gc_block.words.p = Off;
        }
        switch(gc_block.override_command) {

            case Override_FeedSpeedEnable:
                gc_block.modal.override_ctrl.feed_rate_disable = Off;
                gc_block.modal.override_ctrl.spindle_rpm_disable = Off;
                break;

            case Override_FeedSpeedDisable:
                gc_block.modal.override_ctrl.feed_rate_disable = On;
                gc_block.modal.override_ctrl.spindle_rpm_disable = On;
                break;

            case Override_FeedRate:
                gc_block.modal.override_ctrl.feed_rate_disable = gc_block.values.p == 0.0f;
                break;

            case Override_SpindleSpeed:
                gc_block.modal.override_ctrl.spindle_rpm_disable = gc_block.values.p == 0.0f;
                break;

            case Override_FeedHold:
                gc_block.modal.override_ctrl.feed_hold_disable = gc_block.values.p == 0.0f;
                break;

            case Override_Parking:
                if(settings.parking.flags.enable_override_control)
                    gc_block.modal.override_ctrl.parking_disable = gc_block.values.p == 0.0f;
                break;

            default:
                break;
        }
    }

    // [10. Dwell ]: P value missing. NOTE: See below.
    if (gc_block.non_modal_command == NonModal_Dwell) {
        if (!gc_block.words.p)
            FAIL(Status_GcodeValueWordMissing); // [P word missing]
        if(gc_block.values.p < 0.0f)
            FAIL(Status_NegativeValue);
        gc_block.words.p = Off;
    }

    // [11. Set active plane ]: N/A
    gc_get_plane_data(&plane, gc_block.modal.plane_select);

    // [12. Set length units ]: N/A
    // Pre-convert XYZ coordinate values to millimeters, if applicable.
    uint_fast8_t idx = N_AXIS;
    if (gc_block.modal.units_imperial) do { // Axes indices are consistent, so loop may be used.
        idx--;
#if N_AXIS > 3
        if (bit_istrue(axis_words.mask, bit(idx)) && bit_isfalse(settings.steppers.is_rotary.mask, bit(idx))) {
#else
        if (bit_istrue(axis_words.mask, bit(idx))) {
#endif
            gc_block.values.xyz[idx] *= MM_PER_INCH;
#if LATHE_UVW_OPTION
  #if N_AXIS > 3
            if(idx <= Z_AXIS)
  #endif
            gc_block.values.uvw[idx] *= MM_PER_INCH;
#endif
        }
    } while(idx);

    if (command_words.G15 && gc_state.modal.diameter_mode != gc_block.modal.diameter_mode) {
        gc_state.modal.diameter_mode = gc_block.modal.diameter_mode;
        system_add_rt_report(Report_LatheXMode);
    }

    if(gc_state.modal.diameter_mode && bit_istrue(axis_words.mask, bit(X_AXIS)))
        gc_block.values.xyz[X_AXIS] /= 2.0f;

    // Scale axis words if commanded
    if(axis_command == AxisCommand_Scaling) {

        if(gc_block.modal.scaling_active) {

            bool report_scaling = false;

            // TODO: precheck for 0.0f and fail if found?

            gc_block.modal.scaling_active = false;

#ifdef MACH3_SCALING
            // [G51 Errors]: No axis words. TODO: add support for P (scale all with same factor)?
            if (!axis_words.mask)
                FAIL(Status_GcodeNoAxisWords); // [No axis words]

            idx = N_AXIS;
            do {
                if(bit_istrue(axis_words.mask, bit(--idx))) {
                    report_scaling |= scale_factor.ijk[idx] != gc_block.values.xyz[idx];
                    scale_factor.ijk[idx] = gc_block.values.xyz[idx];
                    bit_false(axis_words.mask, bit(idx));
                    system_add_rt_report(Report_Scaling);
                }
                gc_block.modal.scaling_active = gc_block.modal.scaling_active || (scale_factor.xyz[idx] != 1.0f);
            } while(idx);

            gc_block.words.mask &= ~axis_words_mask.mask; // Remove axis words.
#else
            if (!(gc_block.words.p || ijk_words.mask))
                FAIL(Status_GcodeNoAxisWords); // [No axis words]

            idx = N_AXIS;
            do {
                if(bit_istrue(axis_words.mask, bit(--idx)))
                    scale_factor.xyz[idx] = gc_block.values.xyz[idx];
                else
                    scale_factor.xyz[idx] = gc_state.position[idx];
            } while(idx);

            gc_block.words.mask &= ~axis_words_mask.mask; // Remove axis words.

            idx = 3;
            do {
                idx--;
                if(gc_block.words.p) {
                    report_scaling |= scale_factor.ijk[idx] != gc_block.values.p;
                    scale_factor.ijk[idx] = gc_block.values.p;
                } else if(bit_istrue(ijk_words.mask, bit(idx))) {
                    report_scaling |= scale_factor.ijk[idx] != gc_block.values.ijk[idx];
                    scale_factor.ijk[idx] = gc_block.values.ijk[idx];
                }
                gc_block.modal.scaling_active = gc_block.modal.scaling_active || (scale_factor.ijk[idx] != 1.0f);
            } while(idx);

            if(gc_block.words.p)
                gc_block.words.p = Off;
            else
                gc_block.words.i = gc_block.words.j = gc_block.words.k = Off;
#endif
            report_scaling |= gc_state.modal.scaling_active != gc_block.modal.scaling_active;
            gc_state.modal.scaling_active = gc_block.modal.scaling_active;

            if(report_scaling)
                system_add_rt_report(Report_Scaling);

        } else
            set_scaling(1.0f);
    }

    // Scale axis words if scaling active
    if(gc_state.modal.scaling_active) {
        idx = N_AXIS;
        do {
            if(bit_istrue(axis_words.mask, bit(--idx))) {
                if(gc_block.modal.distance_incremental)
                     gc_block.values.xyz[idx] *= scale_factor.ijk[idx];
                else
                     gc_block.values.xyz[idx] = (gc_block.values.xyz[idx] - scale_factor.xyz[idx]) * scale_factor.ijk[idx] + scale_factor.xyz[idx];
#if LATHE_UVW_OPTION
  #if N_AXIS > 3
                if(idx <= Z_AXIS)
  #endif
                gc_block.values.uvw[idx] *= scale_factor.ijk[idx];
#endif
            }
        } while(idx);
    }

    // [13. Cutter radius compensation ]: G41/42 NOT SUPPORTED. Error, if enabled while G53 is active.
    // [G40 Errors]: G2/3 arc is programmed after a G40. The linear move after disabling is less than tool diameter.
    //   NOTE: Since cutter radius compensation is never enabled, these G40 errors don't apply. grblHAL supports G40
    //   only for the purpose to not error when G40 is sent with a g-code program header to setup the default modes.

    // [14. Tool length compensation ]: G43.1 and G49 are always supported, G43 and G43.2 if grbl.tool_table.n_tools > 0
    // [G43.1 Errors]: Motion command in same line.
    // [G43.2 Errors]: Tool number not in the tool table,
    if (command_words.G8) { // Indicates called in block.

#if TOOL_LENGTH_OFFSET_AXIS >= 0
        // NOTE: Although not explicitly stated so, G43.1 should be applied to only one valid
        // axis that is configured (in config.h). There should be an error if the configured axis
        // is absent or if any of the other axis words are present.
        if(gc_block.modal.tool_offset_mode == ToolLengthOffset_EnableDynamic) {
            if (axis_words.mask ^ bit(TOOL_LENGTH_OFFSET_AXIS))
                FAIL(Status_GcodeG43DynamicAxisError);
        }
#endif

        switch(gc_block.modal.tool_offset_mode) {

            case ToolLengthOffset_EnableDynamic:
                if(!axis_words.mask)
                    FAIL(Status_GcodeG43DynamicAxisError);
                break;

            case ToolLengthOffset_Enable:
                if(grbl.tool_table.n_tools) {
                    if(gc_block.words.h) {
                        if(gc_block.values.h > grbl.tool_table.n_tools)
                            FAIL(Status_GcodeIllegalToolTableEntry);
                        gc_block.words.h = Off;
                        if(gc_block.values.h == 0)
                            gc_block.values.h = gc_block.values.t;
                    } else
                        gc_block.values.h = gc_block.values.t;
                } else
                    FAIL(Status_GcodeUnsupportedCommand);
                break;

            case ToolLengthOffset_ApplyAdditional:
                if(grbl.tool_table.n_tools) {
                    if(gc_block.words.h) {
                        if(gc_block.values.h == 0 || gc_block.values.h > grbl.tool_table.n_tools)
                            FAIL(Status_GcodeIllegalToolTableEntry);
                        gc_block.words.h = Off;
                    } else
                        FAIL(Status_GcodeValueWordMissing);
                } else
                    FAIL(Status_GcodeUnsupportedCommand);
                break;

            default:
                break;
        }
    }

    // [15. Coordinate system selection ]: *N/A. Error, if cutter radius comp is active.
    // TODO: A read of the coordinate data may require a buffer sync when the cycle
    // is active. The read pauses the processor temporarily and may cause a rare crash.
    // NOTE: If NVS buffering is active then non-volatile storage reads/writes are buffered and updates
    // delayed until no cycle is active.

    if (command_words.G12) { // Check if called in block
        if (gc_state.modal.coord_system.id != gc_block.modal.coord_system.id && !settings_read_coord_data(gc_block.modal.coord_system.id, &gc_block.modal.coord_system.xyz))
            FAIL(Status_SettingReadFail);
    }

    // [16. Set path control mode ]: N/A. Only G61. G61.1 and G64 NOT SUPPORTED.
#if ENABLE_PATH_BLENDING
    if(command_words.G13) { // Check if called in block
        if(gc_block.modal.control == ControlMode_PathBlending) {
            gc_state.path_tolerance = gc_block.words.p ? gc_block.values.p : 0.0f;
            gc_state.cam_tolerance = gc_block.words.q ? gc_block.values.q : 0.0f;
            gc_block.words.p = gc_block.words.q = Off;
        } else
            gc_state.path_tolerance = gc_state.cam_tolerance = 0.0f;
    }
#endif

    // [17. Set distance mode ]: N/A. Only G91.1. G90.1 NOT SUPPORTED.
    // [18. Set retract mode ]: N/A.

    // [19. Remaining non-modal actions ]: Check go to predefined position, set G10, or set axis offsets.
    // NOTE: We need to separate the non-modal commands that are axis word-using (G10/G28/G30/G92), as these
    // commands all treat axis words differently. G10 as absolute offsets or computes current position as
    // the axis value, G92 similarly to G10 L20, and G28/30 as an intermediate target position that observes
    // all the current coordinate system and G92 offsets.
    switch (gc_block.non_modal_command) {

        case NonModal_SetCoordinateData:

            // [G10 Errors]: L missing and is not 2 or 20. P word missing. (Negative P value done.)
            // [G10 L2 Errors]: R word NOT SUPPORTED. P value not 0 to N_WorkCoordinateSystems (max 9). Axis words missing.
            // [G10 L20 Errors]: P must be 0 to N_WorkCoordinateSystems (max 9). Axis words missing.
            // [G10 L1, L10, L11 Errors]: P must be 0 to grbl.tool_table.n_tools. Axis words or R word missing.

            if (!(axis_words.mask || (gc_block.values.l != 20 && gc_block.words.r)))
                FAIL(Status_GcodeNoAxisWords); // [No axis words (or R word for tool offsets)]

            if (!(gc_block.words.p || gc_block.words.l))
                FAIL(Status_GcodeValueWordMissing); // [P/L word missing]

            if(gc_block.values.p < 0.0f)
                FAIL(Status_NegativeValue);

            uint8_t p_value;

            p_value = (uint8_t)truncf(gc_block.values.p); // Convert p value to int.

            switch(gc_block.values.l) {

                case 2:
                    if (gc_block.words.r)
                        FAIL(Status_GcodeUnsupportedCommand); // [G10 L2 R not supported]
                    // no break

                case 20:
                    if (p_value > N_WorkCoordinateSystems)
                        FAIL(Status_GcodeUnsupportedCoordSys); // [Greater than N sys]
                    // Determine coordinate system to change and try to load from non-volatile storage.
                    gc_block.values.coord_data.id = p_value == 0
                                                     ? gc_block.modal.coord_system.id       // Index P0 as the active coordinate system
                                                     : (coord_system_id_t)(p_value - 1);    // else adjust index to NVS coordinate data indexing.

                    if (!settings_read_coord_data(gc_block.values.coord_data.id, &gc_block.values.coord_data.xyz))
                        FAIL(Status_SettingReadFail); // [non-volatile storage read fail]

#if COMPATIBILITY_LEVEL <= 1
                    if(settings.offset_lock.mask && gc_block.values.coord_data.id >= CoordinateSystem_G59_1 && gc_block.values.coord_data.id <= CoordinateSystem_G59_3) {
                        if(bit_istrue(settings.offset_lock.mask, bit(gc_block.values.coord_data.id - CoordinateSystem_G59_1)))
                            FAIL(Status_GCodeCoordSystemLocked);
                    }
#endif

                    // Pre-calculate the coordinate data changes.
                    idx = N_AXIS;
                    do { // Axes indices are consistent, so loop may be used.
                        // Update axes defined only in block. Always in machine coordinates. Can change non-active system.
                        if (bit_istrue(axis_words.mask, bit(--idx))) {
                            if (gc_block.values.l == 20)
                                // L20: Update coordinate system axis at current position (with modifiers) with programmed value
                                // WPos = MPos - WCS - G92 - TLO  ->  WCS = MPos - G92 - TLO - WPos
                                gc_block.values.coord_data.xyz[idx] = gc_state.position[idx] - gc_block.values.xyz[idx] - gc_state.g92_coord_offset[idx] - gc_state.tool_length_offset[idx];
                            else // L2: Update coordinate system axis to programmed value.
                                gc_block.values.coord_data.xyz[idx] = gc_block.values.xyz[idx];
                        } // else, keep current stored value.
                    } while(idx);
                    break;

                case 1: case 10:
#if COMPATIBILITY_LEVEL <= 1
                case 11:
#endif
                    if(grbl.tool_table.n_tools) {
                        if(p_value == 0 || p_value > grbl.tool_table.n_tools)
                           FAIL(Status_GcodeIllegalToolTableEntry); // [Greater than max allowed tool number]

                        grbl.tool_table.tool[p_value].tool_id = (tool_id_t)p_value;

                        if(gc_block.words.r) {
                            grbl.tool_table.tool[p_value].radius = gc_block.values.r;
                            gc_block.words.r = Off;
                        }

#if COMPATIBILITY_LEVEL <= 1
                        float g59_3_offset[N_AXIS];
                        if(gc_block.values.l == 11 && !settings_read_coord_data(CoordinateSystem_G59_3, &g59_3_offset))
                            FAIL(Status_SettingReadFail);
#endif

                        if(gc_block.values.l == 1)
                            grbl.tool_table.read(p_value, &grbl.tool_table.tool[p_value]);

                        idx = N_AXIS;
                        do {
                            if(bit_istrue(axis_words.mask, bit(--idx))) {
                                if(gc_block.values.l == 1)
                                    grbl.tool_table.tool[p_value].offset[idx] = gc_block.values.xyz[idx];
                                else if(gc_block.values.l == 10)
                                    grbl.tool_table.tool[p_value].offset[idx] = gc_state.position[idx] - gc_state.modal.coord_system.xyz[idx] - gc_state.g92_coord_offset[idx] - gc_block.values.xyz[idx];
#if COMPATIBILITY_LEVEL <= 1
                                else if(gc_block.values.l == 11)
                                    grbl.tool_table.tool[p_value].offset[idx] = g59_3_offset[idx] - gc_block.values.xyz[idx];
#endif
    //                            if(gc_block.values.l != 1)
    //                                tool_table[p_value].offset[idx] -= gc_state.tool_length_offset[idx];
                            } else if(gc_block.values.l == 10 || gc_block.values.l == 11)
                                grbl.tool_table.tool[p_value].offset[idx] = gc_state.tool_length_offset[idx];

                            // else, keep current stored value.
                        } while(idx);

                        if(gc_block.values.l == 1)
                            grbl.tool_table.write(&grbl.tool_table.tool[p_value]);
                    } else
                        FAIL(Status_GcodeUnsupportedCommand);
                    break;

                default:
                    FAIL(Status_GcodeUnsupportedCommand); // [Unsupported L]
            }
            gc_block.words.l = gc_block.words.p = Off;
            break;

        case NonModal_SetCoordinateOffset:

            // [G92 Errors]: No axis words.
            if (!axis_words.mask)
                FAIL(Status_GcodeNoAxisWords); // [No axis words]

            // Update axes defined only in block. Offsets current system to defined value. Does not update when
            // active coordinate system is selected, but is still active unless G92.1 disables it.
            idx = N_AXIS;
            do { // Axes indices are consistent, so loop may be used.
                if (bit_istrue(axis_words.mask, bit(--idx))) {
            // WPos = MPos - WCS - G92 - TLO  ->  G92 = MPos - WCS - TLO - WPos
                    gc_block.values.xyz[idx] = gc_state.position[idx] - gc_block.modal.coord_system.xyz[idx] - gc_block.values.xyz[idx] - gc_state.tool_length_offset[idx];
                } else
                    gc_block.values.xyz[idx] = gc_state.g92_coord_offset[idx];
            } while(idx);
            break;
            
#if ENABLE_ACCELERATION_PROFILES
        case NonModal_SetAccelerationProfile:
            if(gc_block.words.e)
                FAIL(Status_GcodeUnsupportedCommand);

            if(gc_block.words.p && (gc_block.values.p < 1.0f || gc_block.values.p > 5.0f))
                FAIL(Status_GcodeValueOutOfRange);

            gc_state.modal.acceleration_factor = gc_get_accel_factor(gc_block.words.p ? (uint8_t)gc_block.values.p - 1 : 0);
            gc_block.words.p = Off;
            break;
#endif
        default:

            // At this point, the rest of the explicit axis commands treat the axis values as the traditional
            // target position with the coordinate system offsets, G92 offsets, absolute override, and distance
            // modes applied. This includes the motion mode commands. We can now pre-compute the target position.
            // NOTE: Tool offsets may be appended to these conversions when/if this feature is added.
            if((axis_words.mask || gc_block.modal.motion == MotionMode_CwArc || gc_block.modal.motion == MotionMode_CcwArc) && axis_command != AxisCommand_ToolLengthOffset) { // TLO block any axis command.
                idx = N_AXIS;
                do { // Axes indices are consistent, so loop may be used to save flash space.
                    if(bit_isfalse(axis_words.mask, bit(--idx)))
                        gc_block.values.xyz[idx] = gc_state.position[idx]; // No axis word in block. Keep same axis position.
                    else if(gc_block.non_modal_command != NonModal_AbsoluteOverride) {
                        // Update specified value according to distance mode or ignore if absolute override is active.
                        // NOTE: G53 is never active with G28/30 since they are in the same modal group.
                        // Apply coordinate offsets based on distance mode.
#if LATHE_UVW_OPTION
  #if N_AXIS > 3
                        if(idx <= Z_AXIS && bit_istrue(axis_words.mask, bit(idx)) && gc_block.values.uvw[idx] != 0.0f)
  #else
                        if(bit_istrue(axis_words.mask, bit(idx)) && gc_block.values.uvw[idx] != 0.0f)
  #endif
                            gc_block.values.xyz[idx] = gc_state.position[idx] + gc_block.values.uvw[idx];
                        else
#endif
                        if(gc_block.modal.distance_incremental)
                            gc_block.values.xyz[idx] += gc_state.position[idx];
                        else  // Absolute mode
                            gc_block.values.xyz[idx] += gc_get_block_offset(&gc_block, idx);
                    }
                } while(idx);
            }

            // Check remaining non-modal commands for errors.
            switch (gc_block.non_modal_command) {

                case NonModal_GoHome_0: // G28
                case NonModal_GoHome_1: // G30
                    // [G28/30 Errors]: Cutter compensation is enabled.
                    // Retrieve G28/30 go-home position data (in machine coordinates) from non-volatile storage

                    if (!settings_read_coord_data(gc_block.non_modal_command == NonModal_GoHome_0 ? CoordinateSystem_G28 : CoordinateSystem_G30, &gc_block.values.coord_data.xyz))
                        FAIL(Status_SettingReadFail);

                    if (axis_words.mask) {
                        // Move only the axes specified in secondary move.
                        idx = N_AXIS;
                        do {
                            if (bit_isfalse(axis_words.mask, bit(--idx)))
                                gc_block.values.coord_data.xyz[idx] = gc_state.position[idx];
                        } while(idx);
                    } else
                        axis_command = AxisCommand_None; // Set to none if no intermediate motion.
                    break;

                case NonModal_SetHome_0: // G28.1
                case NonModal_SetHome_1: // G30.1
                    // [G28.1/30.1 Errors]: Cutter compensation is enabled.
                    // NOTE: If axis words are passed here, they are interpreted as an implicit motion mode.
                    break;

                case NonModal_ResetCoordinateOffset:
                    // NOTE: If axis words are passed here, they are interpreted as an implicit motion mode.
                    break;

                case NonModal_AbsoluteOverride:
                    // [G53 Errors]: G0 and G1 are not active. Cutter compensation is enabled.
                    // NOTE: All explicit axis word commands are in this modal group. So no implicit check necessary.
                    if (!(gc_block.modal.motion == MotionMode_Seek || gc_block.modal.motion == MotionMode_Linear))
                        FAIL(Status_GcodeG53InvalidMotionMode); // [G53 G0/1 not active]
                    break;

                case NonModal_MacroCall:
                    if(!gc_block.words.p)
                        FAIL(Status_GcodeValueWordMissing); // [P word missing]
                    if(gc_block.values.p > 65535.0f)
                        FAIL(Status_GcodeValueOutOfRange); // [P word out of range]
#if NGC_PARAMETERS_ENABLE
                    if(!ngc_call_push(&gc_state + ngc_call_level()))
                        FAIL(Status_FlowControlStackOverflow); // [Call level too deep]
#endif
#if NGC_EXPRESSIONS_ENABLE
                    // TODO: add context for local storage?
                    {
                        uint_fast8_t idx = 1;

                        axis_words.mask = 0;
                        gc_block.words.p = Off;
                        gc_block.words.value >>= 1;

                        while(gc_block.words.value) {
                            if(gc_block.words.value & 0x1 && gc_value_ptr[idx].value) switch(gc_value_ptr[idx].type) {

                                case ValueType_Float:
                                    g65_words.value |= (1 << idx);
                                    ngc_param_set((ngc_param_id_t)idx, *(float *)gc_value_ptr[idx].value);
                                    break;

                                case ValueType_UInt32:
                                    g65_words.value |= (1 << idx);
                                    ngc_param_set((ngc_param_id_t)idx, (float)*(uint32_t *)gc_value_ptr[idx].value);
                                    break;

                                default:
                                    break;
                            }
                            idx++;
                            gc_block.words.value >>= 1;
                        }
                    }
#else
                    gc_block.words.p = Off;
#endif
                    break;
                default:
                    break;
            }
    } // end gc_block.non_modal_command

    // [20. Motion modes ]:
    if (gc_block.modal.motion == MotionMode_None) {

        // [G80 Errors]: Axis word are programmed while G80 is active.
        // NOTE: Even non-modal commands or TLO that use axis words will throw this strict error.
        if (axis_words.mask && axis_command != AxisCommand_NonModal) // [No axis words allowed]
            FAIL(Status_GcodeAxisWordsExist);

        gc_block.modal.retract_mode = CCRetractMode_Previous;

    // Check remaining motion modes, if axis word are implicit (exist and not used by G10/28/30/92), or
    // was explicitly commanded in the g-code block.
    } else if (axis_command == AxisCommand_MotionMode) {

        gc_parser_flags.motion_mode_changed = gc_block.modal.motion != gc_state.modal.motion;

        if (gc_block.modal.motion == MotionMode_Seek) {
            // [G0 Errors]: Axis letter not configured or without real value (done.)
            // Axis words are optional. If missing, set axis command flag to ignore execution.
            if (!axis_words.mask)
                axis_command = AxisCommand_None;

        // All remaining motion modes (all but G0 and G80), require a valid feed rate value. In units per mm mode,
        // the value must be positive. In inverse time mode, a positive value must be passed with each block.
        } else {

            if(!gc_block.modal.canned_cycle_active)
                gc_block.modal.retract_mode = CCRetractMode_Previous;

            // Initial(?) check for spindle running for moves in G96 mode
            if(gc_block.spindle_modal.rpm_mode == SpindleSpeedMode_CSS && (!gc_block.spindle_modal.state.on || gc_block.values.s == 0.0f))
                 FAIL(Status_GcodeSpindleNotRunning);

            // Check if feed rate is defined for the motion modes that require it.
            if(gc_block.modal.motion == MotionMode_SpindleSynchronized) {

                if(!sspindle->hal->get_data)
                    FAIL(Status_GcodeUnsupportedCommand); // [G33, G33.1]

                if(gc_block.values.k == 0.0f)
                    FAIL(Status_GcodeValueOutOfRange); // [No distance (pitch) given]

                // Ensure spindle speed is at 100% - any override will be disabled on execute.
                gc_parser_flags.spindle_force_sync = On;

            } else if(gc_block.modal.motion == MotionMode_Threading) {

                // Fail if cutter radius comp is active

                if(!sspindle->hal->get_data)
                    FAIL(Status_GcodeUnsupportedCommand); // [G76 not supported]

                if(gc_block.modal.plane_select != PlaneSelect_ZX)
                    FAIL(Status_GcodeIllegalPlane); // [Plane not ZX]

                if(axis_words.mask & ~(bit(X_AXIS)|bit(Z_AXIS)))
                    FAIL(Status_GcodeUnusedWords); // [Only X and Z axis words allowed]

                if(gc_block.words.r && gc_block.values.r < 1.0f)
                    FAIL(Status_GcodeValueOutOfRange);

                if(!axis_words.z || !(gc_block.words.i || gc_block.words.j || gc_block.words.k || gc_block.words.p))
                    FAIL(Status_GcodeValueWordMissing);

                if(gc_block.values.p < 0.0f || gc_block.values.ijk[J_VALUE] < 0.0f || gc_block.values.ijk[K_VALUE] < 0.0f)
                    FAIL(Status_NegativeValue);

                if(gc_block.values.ijk[I_VALUE] == 0.0f ||
                    gc_block.values.ijk[J_VALUE] == 0.0f ||
                     gc_block.values.ijk[K_VALUE] <= gc_block.values.ijk[J_VALUE] ||
                      (gc_block.words.l && (gc_taper_type)gc_block.values.l > Taper_Both))
                    FAIL(Status_GcodeValueOutOfRange);

                if(sspindle->rpm < sspindle->hal->rpm_min || sspindle->rpm > sspindle->hal->rpm_max)
                    FAIL(Status_GcodeRPMOutOfRange);

                if(gc_block.modal.motion != gc_state.modal.motion) {
                    memset(&thread, 0, sizeof(gc_thread_data));
                    thread.depth_degression = 1.0f;
                }

                thread.pitch = gc_block.values.p;
                thread.z_final = gc_block.values.xyz[Z_AXIS];
                thread.cut_direction = gc_block.values.ijk[I_VALUE] < 0.0f ? -1.0f : 1.0f;
                thread.peak = fabsf(gc_block.values.ijk[I_VALUE]);
                thread.initial_depth = gc_block.values.ijk[J_VALUE];
                thread.depth = gc_block.values.ijk[K_VALUE];

                if(gc_block.modal.units_imperial) {
                    thread.peak *= MM_PER_INCH;
                    thread.initial_depth *= MM_PER_INCH;
                    thread.depth *= MM_PER_INCH;
                }

                if(gc_block.modal.diameter_mode) {
                    thread.peak /= 2.0f;
                    thread.initial_depth /= 2.0f;
                    thread.depth /= 2.0f;
                }

                //scaling?

                if(axis_words.x) {
                    thread.main_taper_height = gc_block.values.xyz[X_AXIS] - gc_get_block_offset(&gc_block, X_AXIS);
                    gc_block.values.p = fabsf(thread.z_final - gc_state.position[Z_AXIS]);
                    thread.pitch = thread.pitch * hypot_f(thread.main_taper_height, gc_block.values.p) / gc_block.values.p;
                }

                if(gc_block.words.h)
                    thread.spring_passes = (uint_fast16_t)gc_block.values.h;

                if(gc_block.words.l)
                    thread.end_taper_type = (gc_taper_type)gc_block.values.l;

                if(gc_block.words.e)
                    thread.end_taper_length = gc_block.values.e;

                if(thread.end_taper_length <= 0.0f || thread.end_taper_type == Taper_None) {
                    thread.end_taper_length = 0.0f;
                    thread.end_taper_type = Taper_None;
                    // TODO: fail?
                }

                if(thread.end_taper_type != Taper_None && thread.end_taper_length > fabsf(thread.z_final - gc_state.position[Z_AXIS]) / 2.0f)
                    FAIL(Status_GcodeValueOutOfRange);

                if(gc_block.words.r)
                    thread.depth_degression = gc_block.values.r;

                if(gc_block.words.q)
                    thread.infeed_angle = gc_block.values.q;

                // Ensure spindle speed is at 100% - any override will be disabled on execute.
                gc_parser_flags.spindle_force_sync = On;

                gc_block.words.e = gc_block.words.h = gc_block.words.i = gc_block.words.j = gc_block.words.k = gc_block.words.l = gc_block.words.p = gc_block.words.q = gc_block.words.r = Off;

            } else if (gc_block.values.f == 0.0f)
                FAIL(Status_GcodeUndefinedFeedRate); // [Feed rate undefined]

            if (gc_block.modal.canned_cycle_active) {

                if(gc_parser_flags.canned_cycle_change) {

                    if(gc_state.modal.feed_mode == FeedMode_InverseTime)
                        FAIL(Status_InvalidStatement);

                    if(!gc_block.words.r)
                        FAIL(Status_GcodeValueWordMissing);

                    if(!(axis_words.mask & bit(plane.axis_linear)))
                        FAIL(Status_GcodeValueWordMissing);

                    gc_state.canned.dwell = 0.0f;
                    gc_state.canned.xyz[plane.axis_0] = 0.0f;
                    gc_state.canned.xyz[plane.axis_1] = 0.0f;
                    gc_state.canned.rapid_retract = On;
                    gc_state.canned.spindle_off = Off;
                }

                if(!gc_block.words.l)
                    gc_block.values.l = 1;
                else if(gc_block.values.l <= 0)
                    FAIL(Status_NonPositiveValue); // [L <= 0]

                if(gc_block.words.r)
                    gc_state.canned.retract_position = gc_block.values.r * (gc_block.modal.units_imperial ? MM_PER_INCH : 1.0f) +
                                                        (gc_block.modal.distance_incremental
                                                          ? gc_state.position[plane.axis_linear]
                                                          : gc_get_block_offset(&gc_block, plane.axis_linear));

                idx = N_AXIS;
                do {
                    if(bit_istrue(axis_words.mask, bit(--idx))) {
                        gc_state.canned.xyz[idx] = gc_block.values.xyz[idx];
                        if(idx != plane.axis_linear)
                            gc_state.canned.xyz[idx] -= gc_state.position[idx];
                        else if(gc_block.modal.distance_incremental)
                            gc_state.canned.xyz[idx] = gc_state.canned.retract_position + (gc_state.canned.xyz[idx] - gc_state.position[idx]);
                    }
                } while(idx);

                if(gc_state.canned.retract_position < gc_state.canned.xyz[plane.axis_linear])
                    FAIL(Status_GcodeInvalidRetractPosition);

                gc_block.words.r = gc_block.words.l = Off; // Remove single-meaning value words.

                switch (gc_block.modal.motion) {

                    case MotionMode_CannedCycle86:
                    case MotionMode_CannedCycle89:
                        gc_state.canned.spindle_off = gc_block.modal.motion == MotionMode_CannedCycle86;
                        gc_state.canned.rapid_retract = gc_block.modal.motion == MotionMode_CannedCycle86;
                        // no break

                    case MotionMode_CannedCycle82:
                        if(gc_block.words.p) {
                            if(gc_block.values.p < 0.0f)
                                FAIL(Status_NegativeValue);
                            gc_state.canned.dwell = gc_block.values.p;
                            gc_block.words.p = Off; // Remove single-meaning value word.
                        } else if(gc_parser_flags.canned_cycle_change)
                            FAIL(Status_GcodeValueWordMissing);
                        // no break

                    case MotionMode_CannedCycle85:
                    case MotionMode_CannedCycle81:
                        gc_state.canned.delta = - gc_state.canned.xyz[plane.axis_linear] + gc_state.canned.retract_position;
                        if(gc_block.modal.motion == MotionMode_CannedCycle85)
                            gc_state.canned.rapid_retract = Off;
                        break;

                    case MotionMode_DrillChipBreak:
                    case MotionMode_CannedCycle83:
                        if(gc_block.words.q) {
                            if(gc_block.values.q <= 0.0f)
                                FAIL(Status_NegativeValue); // [Q <= 0]
                            gc_state.canned.delta = gc_block.values.q * (gc_block.modal.units_imperial ? MM_PER_INCH : 1.0f);
                            gc_block.words.q = Off; // Remove single-meaning value word.
                        } else if(gc_parser_flags.canned_cycle_change)
                            FAIL(Status_GcodeValueWordMissing);
                        gc_state.canned.dwell = 0.25f;
                        break;

                    default:
                        break;

                } // end switch gc_state.canned.motion

            } else switch (gc_block.modal.motion) {

                case MotionMode_Linear:
                    // [G1 Errors]: Feed rate undefined. Axis letter not configured or without real value.
                    // Axis words are optional. If missing, set axis command flag to ignore execution.
                    if (!axis_words.mask)
                        axis_command = AxisCommand_None;
                    break;

                case MotionMode_CwArc:
                    gc_parser_flags.arc_is_clockwise = On;
                    // No break intentional.

                case MotionMode_CcwArc:
                    // [G2/3 Errors All-Modes]: Feed rate undefined.
                    // [G2/3 Radius-Mode Errors]: No axis words in selected plane. Target point is same as current.
                    // [G2/3 Offset-Mode Errors]: No axis words and/or offsets in selected plane. The radius to the current
                    //   point and the radius to the target point differs more than 0.002mm (EMC def. 0.5mm OR 0.005mm and 0.1% radius).
                    // [G2/3 Full-Circle-Mode Errors]: Axis words exist. No offsets programmed. P must be an integer.
                    // NOTE: Both radius and offsets are required for arc tracing and are pre-computed with the error-checking.
                    if (gc_block.words.r) { // Arc Radius Mode
                    if (!axis_words.mask)
                        FAIL(Status_GcodeNoAxisWords); // [No axis words]

                    if (!(axis_words.mask & (bit(plane.axis_0)|bit(plane.axis_1))))
                        FAIL(Status_GcodeNoAxisWordsInPlane); // [No axis words in plane]
                    }
                    if (gc_block.words.p) { // Number of turns
                        if(!isintf(gc_block.values.p))
                            FAIL(Status_GcodeCommandValueNotInteger); // [P word is not an integer]
                        gc_block.arc_turns = (uint32_t)truncf(gc_block.values.p);
                        if(gc_block.arc_turns == 0)
                            FAIL(Status_GcodeValueOutOfRange); // [P word is 0]
                        gc_block.words.p = Off;
                    } else
                        gc_block.arc_turns = 1;

                    // Calculate the change in position along each selected axis
                    float x, y;
                    x = gc_block.values.xyz[plane.axis_0] - gc_state.position[plane.axis_0]; // Delta x between current position and target
                    y = gc_block.values.xyz[plane.axis_1] - gc_state.position[plane.axis_1]; // Delta y between current position and target

                    if (gc_block.words.r) { // Arc Radius Mode

                        gc_block.words.r = Off;

                        if (isequal_position_vector(gc_state.position, gc_block.values.xyz))
                            FAIL(Status_GcodeInvalidTarget); // [Invalid target]

                        // Convert radius value to proper units.
                        if (gc_block.modal.units_imperial)
                            gc_block.values.r *= MM_PER_INCH;

                        if(gc_state.modal.scaling_active)
                            gc_block.values.r *= (scale_factor.ijk[plane.axis_0] > scale_factor.ijk[plane.axis_1]
                                                   ? scale_factor.ijk[plane.axis_0]
                                                   : scale_factor.ijk[plane.axis_1]);

                        /*  We need to calculate the center of the circle that has the designated radius and passes
                             through both the current position and the target position. This method calculates the following
                             set of equations where [x,y] is the vector from current to target position, d == magnitude of
                             that vector, h == hypotenuse of the triangle formed by the radius of the circle, the distance to
                             the center of the travel vector. A vector perpendicular to the travel vector [-y,x] is scaled to the
                             length of h [-y/d*h, x/d*h] and added to the center of the travel vector [x/2,y/2] to form the new point
                             [i,j] at [x/2-y/d*h, y/2+x/d*h] which will be the center of our arc.

                             d^2 == x^2 + y^2
                             h^2 == r^2 - (d/2)^2
                             i == x/2 - y/d*h
                             j == y/2 + x/d*h

                                                                                  O <- [i,j]
                                                                               -  |
                                                                     r      -     |
                                                                         -        |
                                                                      -           | h
                                                                   -              |
                                                     [0,0] ->  C -----------------+--------------- T  <- [x,y]
                                                               | <------ d/2 ---->|

                             C - Current position
                             T - Target position
                             O - center of circle that pass through both C and T
                             d - distance from C to T
                             r - designated radius
                             h - distance from center of CT to O

                             Expanding the equations:

                             d -> sqrt(x^2 + y^2)
                             h -> sqrt(4 * r^2 - x^2 - y^2)/2
                             i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2
                             j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2)) / sqrt(x^2 + y^2)) / 2

                             Which can be written:

                             i -> (x - (y * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2
                             j -> (y + (x * sqrt(4 * r^2 - x^2 - y^2))/sqrt(x^2 + y^2))/2

                             Which we for size and speed reasons optimize to:

                             h_x2_div_d = sqrt(4 * r^2 - x^2 - y^2)/sqrt(x^2 + y^2)
                             i = (x - (y * h_x2_div_d))/2
                             j = (y + (x * h_x2_div_d))/2
                         */

                        // First, use h_x2_div_d to compute 4*h^2 to check if it is negative or r is smaller
                        // than d. If so, the sqrt of a negative number is complex and error out.
                        float h_x2_div_d = 4.0f * gc_block.values.r * gc_block.values.r - x * x - y * y;

                        if (h_x2_div_d < 0.0f)
                            FAIL(Status_GcodeArcRadiusError); // [Arc radius error] TODO: this will fail due to limited float precision...

                        // Finish computing h_x2_div_d.
                        h_x2_div_d = -sqrtf(h_x2_div_d) / hypot_f(x, y); // == -(h * 2 / d)

                        // Invert the sign of h_x2_div_d if the circle is counter clockwise (see sketch below)
                        if (gc_block.modal.motion == MotionMode_CcwArc)
                            h_x2_div_d = -h_x2_div_d;

                        /* The counter clockwise circle lies to the left of the target direction. When offset is positive,
                           the left hand circle will be generated - when it is negative the right hand circle is generated.

                                                                               T  <-- Target position

                                                                               ^
                                    Clockwise circles with this center         |          Clockwise circles with this center will have
                                    will have > 180 deg of angular travel      |          < 180 deg of angular travel, which is a good thing!
                                                                     \         |          /
                        center of arc when h_x2_div_d is positive ->  x <----- | -----> x <- center of arc when h_x2_div_d is negative
                                                                               |
                                                                               |

                                                                               C  <-- Current position
                        */
                        // Negative R is g-code-alese for "I want a circle with more than 180 degrees of travel" (go figure!),
                        // even though it is advised against ever generating such circles in a single line of g-code. By
                        // inverting the sign of h_x2_div_d the center of the circles is placed on the opposite side of the line of
                        // travel and thus we get the unadvisably long arcs as prescribed.
                        if (gc_block.values.r < 0.0f) {
                            h_x2_div_d = -h_x2_div_d;
                            gc_block.values.r = -gc_block.values.r; // Finished with r. Set to positive for mc_arc
                        }
                        // Complete the operation by calculating the actual center of the arc
                        gc_block.values.ijk[plane.axis_0] = 0.5f * (x - (y * h_x2_div_d));
                        gc_block.values.ijk[plane.axis_1] = 0.5f * (y + (x * h_x2_div_d));

                    } else { // Arc Center Format Offset Mode

                        if (!(ijk_words.mask & (bit(plane.axis_0)|bit(plane.axis_1))))
                            FAIL(Status_GcodeNoOffsetsInPlane);// [No offsets in plane]

                        gc_block.words.i = gc_block.words.j = gc_block.words.k = Off;

                        // Convert IJK values to proper units.
                        if (gc_block.modal.units_imperial) {
                            idx = 3;
                            do { // Axes indices are consistent, so loop may be used to save flash space.
                                idx--;
                                if (ijk_words.mask & bit(idx))
                                    gc_block.values.ijk[idx] *= MM_PER_INCH;
                            } while(idx);
                        }

                        // Scale values if scaling active - NOTE: only incremental mode is supported
                        if(gc_state.modal.scaling_active) {
                            idx = 3;
                            do {
                                if (ijk_words.mask & bit(--idx))
                                    gc_block.values.ijk[idx] *= scale_factor.ijk[idx];
                            } while(idx);
                        }

                        // Arc radius from center to target
                        x -= gc_block.values.ijk[plane.axis_0]; // Delta x between circle center and target
                        y -= gc_block.values.ijk[plane.axis_1]; // Delta y between circle center and target
                        float target_r = hypot_f(x, y);

                        // Compute arc radius for mc_arc. Defined from current location to center.
                        gc_block.values.r = hypot_f(gc_block.values.ijk[plane.axis_0], gc_block.values.ijk[plane.axis_1]);

                        // Compute difference between current location and target radii for final error-checks.
                        float delta_r = fabsf(target_r - gc_block.values.r);
                        if (delta_r > 0.005f) {
                            if (delta_r > 0.5f)
                                FAIL(Status_GcodeInvalidTarget); // [Arc definition error] > 0.5mm
                            if (delta_r > (0.001f * gc_block.values.r))
                                FAIL(Status_GcodeInvalidTarget); // [Arc definition error] > 0.005mm AND 0.1% radius
                        }
                    }
                    break;

                case MotionMode_CubicSpline:
                    // [G5 Errors]: Feed rate undefined.
                    // [G5 Plane Errors]: The active plane is not G17.
                    // [G5 Offset Errors]: P and Q are not both specified.
                    // [G5 Offset Errors]: Just one of I or J are specified.
                    // [G5 Offset Errors]: I or J are unspecified in the first of a series of G5 commands.
                    // [G5 Axisword Errors]: An axis other than X or Y is specified.
                    if(gc_block.modal.plane_select != PlaneSelect_XY)
                        FAIL(Status_GcodeIllegalPlane); // [The active plane is not G17]

                    if (axis_words.mask & ~(bit(X_AXIS)|bit(Y_AXIS)))
                        FAIL(Status_GcodeAxisCommandConflict); // [An axis other than X or Y is specified]

                    if((gc_block.words.mask & pq_words.mask) != pq_words.mask)
                        FAIL(Status_GcodeValueWordMissing); // [P and Q are not both specified]

                    if(gc_parser_flags.motion_mode_changed && (gc_block.words.mask & ij_words.mask) != ij_words.mask)
                        FAIL(Status_GcodeValueWordMissing); // [I or J are unspecified in the first of a series of G5 commands]

                    if(!(gc_block.words.i || gc_block.words.j)) {
                        gc_block.values.ijk[I_VALUE] = - gc_block.values.p;
                        gc_block.values.ijk[J_VALUE] = - gc_block.values.q;
                    } else {
                        // Convert I and J values to proper units.
                        if (gc_block.modal.units_imperial) {
                            gc_block.values.ijk[I_VALUE] *= MM_PER_INCH;
                            gc_block.values.ijk[J_VALUE] *= MM_PER_INCH;
                        }
                        // Scale values if scaling active
                        if(gc_state.modal.scaling_active) {
                            gc_block.values.ijk[I_VALUE] *= scale_factor.ijk[X_AXIS];
                            gc_block.values.ijk[J_VALUE] *= scale_factor.ijk[Y_AXIS];
                        }
                    }
                    // Convert P and Q values to proper units.
                    if (gc_block.modal.units_imperial) {
                        gc_block.values.p *= MM_PER_INCH;
                        gc_block.values.q *= MM_PER_INCH;
                    }
                    // Scale values if scaling active
                    if(gc_state.modal.scaling_active) {
                        gc_block.values.p *= scale_factor.ijk[X_AXIS];
                        gc_block.values.q *= scale_factor.ijk[Y_AXIS];
                    }
                    gc_state.modal.spline_pq[X_AXIS] = gc_block.values.p;
                    gc_state.modal.spline_pq[Y_AXIS] = gc_block.values.q;
                    gc_block.words.p = gc_block.words.q = gc_block.words.i = gc_block.words.j = Off;
                    break;

                case MotionMode_QuadraticSpline:
                    // [G5.1 Errors]: Feed rate undefined.
                    // [G5.1 Plane Errors]: The active plane is not G17.
                    // [G5.1 Offset Errors]: Just one of I or J are specified.
                    // [G5.1 Offset Errors]: I or J are unspecified in the first of a series of G5 commands.
                    // [G5.1 Axisword Errors]: An axis other than X or Y is specified.
                    if(gc_block.modal.plane_select != PlaneSelect_XY)
                        FAIL(Status_GcodeIllegalPlane); // [The active plane is not G17]

                    if (axis_words.mask & ~(bit(X_AXIS)|bit(Y_AXIS)))
                        FAIL(Status_GcodeAxisCommandConflict); // [An axis other than X or Y is specified]

                    if((gc_block.words.mask & ij_words.mask) != ij_words.mask)
                        FAIL(Status_GcodeValueWordMissing); // [I or J are unspecified]

                    if(gc_block.values.ijk[I_VALUE] == 0.0f && gc_block.values.ijk[I_VALUE] == 0.0f)
                        FAIL(Status_GcodeValueOutOfRange); // [I or J are zero]

                    // Convert I and J values to proper units.
                    if (gc_block.modal.units_imperial) {
                        gc_block.values.ijk[I_VALUE] *= MM_PER_INCH;
                        gc_block.values.ijk[J_VALUE] *= MM_PER_INCH;
                    }
                    // Scale values if scaling active
                    if(gc_state.modal.scaling_active) {
                        gc_block.values.ijk[I_VALUE] *= scale_factor.ijk[X_AXIS];
                        gc_block.values.ijk[J_VALUE] *= scale_factor.ijk[Y_AXIS];
                    }
                    gc_block.words.i = gc_block.words.j = Off;
                    break;

                case MotionMode_ProbeTowardNoError:
                case MotionMode_ProbeAwayNoError:
                    gc_parser_flags.probe_is_no_error = On;
                    // No break intentional.

                case MotionMode_ProbeToward:
                case MotionMode_ProbeAway:
                    if(gc_block.modal.motion == MotionMode_ProbeAway || gc_block.modal.motion == MotionMode_ProbeAwayNoError)
                        gc_parser_flags.probe_is_away = On;
                    // [G38 Errors]: Target is same current. No axis words. Cutter compensation is enabled. Feed rate
                    //   is undefined. Probe is triggered. NOTE: Probe check moved to probe cycle. Instead of returning
                    //   an error, it issues an alarm to prevent further motion to the probe. It's also done there to
                    //   allow the planner buffer to empty and move off the probe trigger before another probing cycle.
                    if (!axis_words.mask)
                        FAIL(Status_GcodeNoAxisWords); // [No axis words]
                    if (isequal_position_vector(gc_state.position, gc_block.values.xyz))
                        FAIL(Status_GcodeInvalidTarget); // [Invalid target]
                    break;

                default:
                    break;

            } // end switch gc_block.modal.motion
        }
    }

    // [21. Program flow ]: No error checks required.

    // [0. Non-specific error-checks]: Complete unused value words check, i.e. IJK used when in arc
    // radius mode, or axis words that aren't used in the block.
    if (gc_parser_flags.jog_motion) // Jogging only uses the F feed rate and XYZ value words. N is valid, but S and T are invalid.
        gc_block.words.n = gc_block.words.f = Off;
    else
        gc_block.words.n = gc_block.words.f = gc_block.words.s = gc_block.words.t = Off;

    if (axis_command)
        gc_block.words.mask &= ~axis_words_mask.mask; // Remove axis words.

    if (gc_block.words.mask)
        FAIL(Status_GcodeUnusedWords); // [Unused words]

    /* -------------------------------------------------------------------------------------
     STEP 4: EXECUTE!!
     Assumes that all error-checking has been completed and no failure modes exist. We just
     need to update the state and execute the block according to the order-of-execution.
    */

    // Initialize planner data struct for motion blocks.
    plan_line_data_t plan_data;
    memset(&plan_data, 0, sizeof(plan_line_data_t)); // Zero plan_data struct
    plan_data.offset_id = gc_state.offset_id;
    plan_data.condition.target_validated = plan_data.condition.target_valid = sys.soft_limits.mask == 0;
#if ENABLE_ACCELERATION_PROFILES
    plan_data.acceleration_factor = gc_state.modal.acceleration_factor;
#endif

    // Intercept jog commands and complete error checking for valid jog commands and execute.
    // NOTE: G-code parser state is not updated, except the position to ensure sequential jog
    // targets are computed correctly. The final parser position after a jog is updated in
    // protocol_execute_realtime() when jogging completes or is canceled.
    if(gc_parser_flags.jog_motion) {

        // Only distance and unit modal commands and G53 absolute override command are allowed.
        // NOTE: Feed rate word and axis word checks have already been performed in STEP 3.
        if(command_words.mask & ~jog_groups.mask)
            FAIL(Status_InvalidJogCommand);

        if(!(gc_block.non_modal_command == NonModal_AbsoluteOverride || gc_block.non_modal_command == NonModal_NoAction))
            FAIL(Status_InvalidJogCommand);

#if N_SYS_SPINDLE > 1
        spindle_t *spindle = sspindle ? sspindle : gc_state.modal.spindle;
#else
        spindle_t *spindle = &gc_block.modal.spindle;
#endif

        // Initialize planner data to current spindle and coolant modal state.
        memcpy(&plan_data.spindle, spindle, sizeof(spindle_t));
        plan_data.condition.coolant = gc_state.modal.coolant;
        plan_data.condition.is_rpm_rate_adjusted = gc_state.is_rpm_rate_adjusted || (spindle->state.ccw && spindle->hal->cap.laser);

        if((status_code_t)(int_value = (uint_fast16_t)mc_jog_execute(&plan_data, &gc_block, gc_state.position)) == Status_OK)
            memcpy(gc_state.position, gc_block.values.xyz, sizeof(gc_state.position));

        return (status_code_t)int_value;
    }

    // If in laser mode, setup laser power based on current and past parser conditions.
    if(sspindle && sspindle->hal->cap.laser) {

        if(!motion_is_lasercut(gc_block.modal.motion))
            gc_parser_flags.laser_disable = On;

        // Any motion mode with axis words is allowed to be passed from a spindle speed update.
        // NOTE: G1 and G0 without axis words sets axis_command to none. G28/30 are intentionally omitted.
        // TODO: Check sync conditions for M3 enabled motions that don't enter the planner. (zero length).
        if(axis_words.mask && (axis_command == AxisCommand_MotionMode))
            gc_parser_flags.laser_is_motion = On;
        else if(sspindle->state.on && !sspindle->state.ccw) {
            // M3 constant power laser requires planner syncs to update the laser when changing between
            // a G1/2/3 motion mode state and vice versa when there is no motion in the line.
            if(motion_is_lasercut(gc_state.modal.motion)) {
                if(gc_parser_flags.laser_disable)
                    gc_parser_flags.spindle_force_sync = On; // Change from G1/2/3 motion mode.
            } else if(!gc_parser_flags.laser_disable) // When changing to a G1 motion mode without axis words from a non-G1/2/3 motion mode.
                gc_parser_flags.spindle_force_sync = On;
        }

        gc_state.is_rpm_rate_adjusted = sspindle->state.ccw && !gc_parser_flags.laser_disable;
    }

    // [0. Non-specific/common error-checks and miscellaneous setup]:
    // NOTE: If no line number is present, the value is zero.
    gc_state.line_number = gc_block.values.n;
    plan_data.line_number = gc_state.line_number; // Record data for planner use.

    bool check_mode = state_get() == STATE_CHECK_MODE;

    // [1. Comments feedback ]: Extracted in protocol.c if HAL entry point provided
    if(message && !check_mode && (plan_data.message = malloc(strlen(message) + 1)))
        strcpy(plan_data.message, message);

    // [2. Set feed rate mode ]:
    gc_state.modal.feed_mode = gc_block.modal.feed_mode;
    if (gc_state.modal.feed_mode == FeedMode_InverseTime)
        plan_data.condition.inverse_time = On; // Set condition flag for planner use.

    // [3. Set feed rate ]:
    gc_state.feed_rate = gc_block.values.f; // Always copy this value. See feed rate error-checking.
    plan_data.feed_rate = gc_state.feed_rate; // Record data for planner use.

    // [4. Set spindle speed ]:

#if N_SYS_SPINDLE > 1
  if(sspindle) {
#endif // N_SYS_SPINDLE > 1

    if(sspindle->rpm_mode == SpindleSpeedMode_CSS) {
        if(gc_block.modal.motion != MotionMode_None && gc_block.modal.motion != MotionMode_Seek) {
            sspindle->css = &sspindle->hal->param->css;
            sspindle->css->axis = plane.axis_1;
            sspindle->css->tool_offset = gc_get_offset(plane.axis_1, false);
            float pos = gc_state.position[plane.axis_1] - sspindle->css->tool_offset;
            gc_block.values.s = pos <= 0.0f ? sspindle->css->max_rpm : min(sspindle->css->max_rpm, sspindle->css->surface_speed / (pos * (float)(2.0f * M_PI)));
//??            gc_parser_flags.spindle_force_sync = On;
        } else {
            if(sspindle->css) {
                sspindle->css = NULL;
                protocol_buffer_synchronize(); // Empty planner buffer to ensure we get RPM at end of last CSS motion
            }
            gc_block.values.s = sspindle->rpm; //sspindle.hal->param->rpm; // Keep current RPM
        }
    }

    if(sspindle->rpm != gc_block.values.s || gc_parser_flags.spindle_force_sync) {
        if(sspindle->state.on && !gc_parser_flags.laser_is_motion) {
            sspindle->hal->param->rpm = gc_block.values.s;
            spindle_sync(sspindle->hal, sspindle->state, gc_parser_flags.laser_disable ? 0.0f : gc_block.values.s);
        }
        sspindle->rpm = gc_block.values.s; // Update spindle speed state.
    }

    // NOTE: Pass zero spindle speed for all restricted laser motions.
    plan_data.spindle.rpm = gc_parser_flags.laser_disable ? 0.0f : gc_block.values.s;

#if N_SYS_SPINDLE > 1
  }
#endif

    //
    // [5. Select tool ]: Only tracks tool value if ATC or manual tool change is not possible.
    if(gc_state.tool_pending != gc_block.values.t && !check_mode) {

        tool_data_t *pending_tool = tool_get_pending((gc_state.tool_pending = gc_block.values.t));

        // If M6 not available or M61 commanded set new tool immediately
        if(set_tool || settings.tool_change.mode == ToolChange_Ignore || !(hal.stream.suspend_read || hal.tool.change)) {

            tool_set(pending_tool);

            if(grbl.on_tool_selected) {

                spindle_state_t state = sspindle ? sspindle->state : (spindle_state_t){0};

                grbl.on_tool_selected(pending_tool);

                if(sspindle && state.value != sspindle->state.value) {
                    command_words.M7 = On;
                    gc_block.spindle_modal.state = sspindle->state;
                }
            }

            if(grbl.on_tool_changed)
                grbl.on_tool_changed(gc_state.tool);

            system_add_rt_report(Report_Tool);
        }

        // Prepare tool carousel when available
        if(hal.tool.select)
            hal.tool.select(pending_tool, !set_tool);
        else
            system_add_rt_report(Report_Tool);
    }

    // [5a. HAL pin I/O ]: M62 - M68. (Modal group M10)

    if(port_command) {

        switch(port_command) {

            case IoMCode_OutputOnSynced:
            case IoMCode_OutputOffSynced:
                add_output_command(&gc_block.output_command);
                break;

            case IoMCode_OutputOnImmediate:
            case IoMCode_OutputOffImmediate:
                hal.port.digital_out(gc_block.output_command.port, gc_block.output_command.value != 0.0f);
                break;

            case IoMCode_WaitOnInput:
                sys.var5399 = hal.port.wait_on_input((io_port_type_t)gc_block.output_command.is_digital, gc_block.output_command.port, (wait_mode_t)gc_block.values.l, gc_block.values.q);
                system_add_rt_report(Report_M66Result);
                break;

            case IoMCode_AnalogOutSynced:
                add_output_command(&gc_block.output_command);
                break;

            case IoMCode_AnalogOutImmediate:
                hal.port.analog_out(gc_block.output_command.port, gc_block.output_command.value);
                break;
        }
    }

    // [6. Change tool ]: Delegated to (possible) driver implementation
    if(command_words.M6 && !set_tool && !check_mode) {

        tool_data_t *pending_tool = tool_get_pending(gc_state.tool_pending);

        protocol_buffer_synchronize();

        if(plan_data.message) {
            gc_output_message(plan_data.message);
            plan_data.message = NULL;
        }

        if(pending_tool->tool_id != gc_state.tool->tool_id) {

            if(grbl.on_tool_selected) {

                spindle_state_t state = sspindle ? sspindle->state : (spindle_state_t){0};

                grbl.on_tool_selected(pending_tool);

                if(sspindle && state.value != sspindle->state.value) {
                    command_words.M7 = On;
                    gc_block.spindle_modal.state = sspindle->state;
                }
            }

            if(hal.tool.change) { // ATC
                if((int_value = (uint_fast16_t)hal.tool.change(&gc_state)) != Status_OK) {
#if NGC_EXPRESSIONS_ENABLE
                    if(int_value != Status_Unhandled)
#endif
                        FAIL((status_code_t)int_value);
                }
                system_add_rt_report(Report_Tool);
            } else { // Manual
                int_value = (uint_fast16_t)Status_OK;
                gc_state.tool_change = true;
                system_set_exec_state_flag(EXEC_TOOL_CHANGE);   // Set up program pause for manual tool change
                protocol_execute_realtime();                    // Execute...
            }
#if NGC_EXPRESSIONS_ENABLE
            if((status_code_t)int_value != Status_Unhandled)
                tool_set(pending_tool);
            else if(grbl.tool_table.n_tools && command_words.G8 && gc_block.modal.tool_offset_mode && ToolLengthOffset_Enable) {
                gc_state.g43_pending = gc_block.values.h;
                command_words.G8 = Off;
            }
#else
            tool_set(pending_tool);
#endif
            if(grbl.on_tool_changed && state_get() != STATE_TOOL_CHANGE)
                grbl.on_tool_changed(gc_state.tool);
        }
    }

    // [7. Spindle control ]:
    // Update spindle control and apply spindle speed when enabling it in this block.
    // NOTE: All spindle state changes are synced, even in laser mode. Also, plan_data,
    // rather than gc_state, is used to manage laser state for non-laser motions.
    if(command_words.M7) {

        bool spindle_ok;

#if N_SYS_SPINDLE > 1

        if(sspindle == NULL) {

            idx = N_SYS_SPINDLE;
            do {
                spindle_t *sys_spindle = &gc_state.modal.spindle[--idx];
                if(sys_spindle->hal) {

                    if((spindle_ok = sys_spindle->state.value != gc_block.spindle_modal.state.value)) {

                        if(grbl.on_spindle_programmed)
                            grbl.on_spindle_programmed(sys_spindle->hal, gc_block.spindle_modal.state, sys_spindle->rpm, sys_spindle->rpm_mode);

                        if((spindle_ok = spindle_sync(sys_spindle->hal, gc_block.spindle_modal.state, sys_spindle->rpm))) {
                            if((sys_spindle->state = sys_spindle->hal->param->state = gc_block.spindle_modal.state).on)
                                sspindle = sys_spindle;
                        }
                    }

                    if((spindle_ok ? false : (!spindle_ok || spindle_event)) && grbl.on_spindle_programmed)
                        grbl.on_spindle_programmed(sys_spindle->hal, sys_spindle->state, sys_spindle->rpm, sys_spindle->rpm_mode);
                }
            } while(idx);

            spindle_event = false;

        } else

#endif // N_SYS_SPINDLE > 1

        if((spindle_ok = sspindle->state.value != gc_block.spindle_modal.state.value)) {

            if(grbl.on_spindle_programmed)
                grbl.on_spindle_programmed(sspindle->hal, gc_block.spindle_modal.state, plan_data.spindle.rpm, sspindle->rpm_mode);

            if((spindle_ok = spindle_sync(sspindle->hal, gc_block.spindle_modal.state, plan_data.spindle.rpm)))
                sspindle->state = sspindle->hal->param->state = gc_block.spindle_modal.state;

            spindle_event = !spindle_ok;
        }

        if(spindle_event && grbl.on_spindle_programmed)
            grbl.on_spindle_programmed(sspindle->hal, sspindle->state, plan_data.spindle.rpm, sspindle->rpm_mode);
    }

    if(sspindle != NULL)
        gc_state.spindle = sspindle; // for now

    plan_data.spindle.hal = gc_state.spindle->hal;
    memcpy(&plan_data.spindle, gc_state.spindle, offsetof(spindle_t, rpm)); // Record data for planner use.

// TODO: Recheck spindle running in CCS mode (is_rpm_pos_adjusted == On)?

    plan_data.spindle.state = gc_state.spindle->state; // Set condition flag for planner use.
    plan_data.condition.is_rpm_rate_adjusted = gc_state.is_rpm_rate_adjusted;
    plan_data.condition.is_laser_ppi_mode = gc_state.is_rpm_rate_adjusted && gc_state.is_laser_ppi_mode;

#if NGC_PARAMETERS_ENABLE

    // [7a. Modal state actions ]:
    switch(gc_block.state_action) {

        case ModalState_Save:
        case ModalState_SaveAutoRestore:
            gc_state.modal.feed_rate = gc_state.feed_rate;
            if(!ngc_modal_state_save(&gc_state.modal, gc_block.state_action == ModalState_SaveAutoRestore))
                FAIL(Status_FlowControlOutOfMemory); // [Out of memory] TODO: allocate memory during validation? Static allocation?
            break;

        case ModalState_Invalidate:
            ngc_modal_state_invalidate();
            break;

        case ModalState_Restore:
            ngc_modal_state_restore();
            break;

        default:
            break;
    }

#endif // NGC_PARAMETERS_ENABLE

    // [8. Coolant control ]:
    if (gc_parser_flags.set_coolant && gc_state.modal.coolant.value != gc_block.modal.coolant.value) {
    // NOTE: Coolant M-codes are modal. Only one command per line is allowed. But, multiple states
    // can exist at the same time, while coolant disable clears all states.
        if(coolant_sync(gc_block.modal.coolant))
            gc_state.modal.coolant = gc_block.modal.coolant;
    }

    plan_data.condition.coolant = gc_state.modal.coolant; // Set condition flag for planner use.

    sys.override_delay.flags = 0;

    // [9. Override control ]:
    if(command_words.M9 && gc_state.modal.override_ctrl.value != gc_block.modal.override_ctrl.value) {

        gc_state.modal.override_ctrl = gc_block.modal.override_ctrl;

#if N_SYS_SPINDLE > 1
        if(sspindle == NULL) {
            uint_fast8_t idx = N_SYS_SPINDLE;
            do {
                set_spindle_override(&gc_state.modal.spindle[--idx], gc_state.modal.override_ctrl.spindle_rpm_disable);
            } while(idx);
        } else
#else
        set_spindle_override(sspindle, gc_state.modal.override_ctrl.spindle_rpm_disable);
#endif

        if(gc_state.modal.override_ctrl.feed_rate_disable)
            plan_feed_override(0, 0);

        mc_override_ctrl_update(gc_state.modal.override_ctrl); // NOTE: must be called last!
    }

    // [9a. User defined M commands ]:
    if(gc_block.user_mcode && !check_mode) {

        if(gc_block.user_mcode_sync)
            protocol_buffer_synchronize(); // Ensure user defined mcode is executed when specified in program.

        gc_block.words.mask = user_words.mask;
        gc_block.values.f = single_meaning_value.f;
        gc_block.values.o = single_meaning_value.o;
        gc_block.values.s = single_meaning_value.s;
        gc_block.values.t = single_meaning_value.t;
        grbl.user_mcode.execute(state_get(), &gc_block);
        gc_block.words.mask = 0;
    }

    // [10. Dwell ]:
    if (gc_block.non_modal_command == NonModal_Dwell)
        mc_dwell(gc_block.values.p);

    // [11. Set active plane ]:
    gc_state.modal.plane_select = gc_block.modal.plane_select;

    // [12. Set length units ]:
    gc_state.modal.units_imperial = gc_block.modal.units_imperial;

    // [13. Cutter radius compensation ]: G41/42 NOT SUPPORTED
    // gc_state.modal.cutter_comp = gc_block.modal.cutter_comp; // NOTE: Not needed since always disabled.

    // [14. Tool length compensation ]: G43, G43.1 and G49 supported. G43 supported when grbl.tool_table.n_tools > 0.
    // NOTE: If G43 were supported, its operation wouldn't be any different from G43.1 in terms
    // of execution. The error-checking step would simply load the offset value into the correct
    // axis of the block XYZ value array.
    if (command_words.G8) { // Indicates a change.

        bool tlo_changed = false;

        idx = N_AXIS;
        gc_state.modal.tool_offset_mode = gc_block.modal.tool_offset_mode;

        do {

            idx--;

            switch(gc_state.modal.tool_offset_mode) {

                case ToolLengthOffset_Cancel: // G49
                    tlo_changed |= gc_state.tool_length_offset[idx] != 0.0f;
                    gc_state.tool_length_offset[idx] = 0.0f;
                    break;

                case ToolLengthOffset_Enable: // G43
                    if (gc_state.tool_length_offset[idx] != grbl.tool_table.tool[gc_block.values.h].offset[idx]) {
                        tlo_changed = true;
                        gc_state.tool_length_offset[idx] = grbl.tool_table.tool[gc_block.values.h].offset[idx];
                    }
                    break;

                case ToolLengthOffset_ApplyAdditional: // G43.2
                    tlo_changed |= grbl.tool_table.tool[gc_block.values.h].offset[idx] != 0.0f;
                    gc_state.tool_length_offset[idx] += grbl.tool_table.tool[gc_block.values.h].offset[idx];
                    break;

                case ToolLengthOffset_EnableDynamic: // G43.1
                    if (bit_istrue(axis_words.mask, bit(idx)) && gc_state.tool_length_offset[idx] != gc_block.values.xyz[idx]) {
                        tlo_changed = true;
                        gc_state.tool_length_offset[idx] = gc_block.values.xyz[idx];
                    }
                    break;

                default:
                    break;
            }
        } while(idx);

        if(tlo_changed) {
            system_add_rt_report(Report_ToolOffset);
            system_flag_wco_change();
        }
    }

    // [15. Coordinate system selection ]:
    if (gc_state.modal.coord_system.id != gc_block.modal.coord_system.id) {
        memcpy(&gc_state.modal.coord_system, &gc_block.modal.coord_system, sizeof(gc_state.modal.coord_system));
        system_add_rt_report(Report_GWCO);
        system_flag_wco_change();
    }

    // [16. Set path control mode ]: G61.1/G64 NOT SUPPORTED
    // gc_state.modal.control = gc_block.modal.control; // NOTE: Always default.
#if ENABLE_PATH_BLENDING
    gc_state.modal.control = gc_block.modal.control;
#endif

    // [17. Set distance mode ]:
    gc_state.modal.distance_incremental = gc_block.modal.distance_incremental;

    // [18. Set retract mode ]:
    gc_state.modal.retract_mode = gc_block.modal.retract_mode;

    // [19. Go to predefined position, Set G10, or Set axis offsets ]:
    switch(gc_block.non_modal_command) {

        case NonModal_SetCoordinateData:
            if(gc_block.values.l == 2 || gc_block.values.l == 20) {
                settings_write_coord_data(gc_block.values.coord_data.id, &gc_block.values.coord_data.xyz);
                // Update system coordinate system if currently active.
                if (gc_state.modal.coord_system.id == gc_block.values.coord_data.id) {
                    memcpy(gc_state.modal.coord_system.xyz, gc_block.values.coord_data.xyz, sizeof(gc_state.modal.coord_system.xyz));
                    system_flag_wco_change();
                }
            }
            break;

        case NonModal_GoHome_0:
#if N_AXIS > 3
            {
                axes_signals_t wrap = { (axis_words.mask & settings.steppers.is_rotary.mask) & settings.steppers.rotary_wrap.mask };
                if(gc_state.modal.distance_incremental && wrap.mask) {
                    for(idx = Z_AXIS + 1; idx < N_AXIS; idx++) {
                        if(bit_istrue(wrap.mask, bit(idx)) && gc_block.values.xyz[idx] == gc_state.position[idx])
                            gc_block.rotary_wrap.mask |= bit(idx);
                    }
                }
            }
            // no break
#endif

        case NonModal_GoHome_1:
            // Move to intermediate position before going home. Obeys current coordinate system and offsets
            // and absolute and incremental modes.
            plan_data.condition.rapid_motion = On; // Set rapid motion condition flag.
            if(axis_command)
                mc_line(gc_block.values.xyz, &plan_data);
#if N_AXIS > 3
            if(gc_block.rotary_wrap.mask) {

                coord_system_t wrap_target;

                protocol_buffer_synchronize();
                memcpy(wrap_target.xyz, gc_block.values.coord_data.xyz, sizeof(coord_system_t));

                for(idx = Z_AXIS + 1; idx < N_AXIS; idx++) {
                    if(bit_istrue(gc_block.rotary_wrap.mask, bit(idx))) {
                        float position, delta;
                        if((wrap_target.xyz[idx] = fmodf(wrap_target.xyz[idx], 360.0f)) < 0.0f)
                            wrap_target.xyz[idx] = 360.0f + wrap_target.xyz[idx];
                        if((position = fmodf(gc_state.position[idx], 360.0f)) < 0.0)
                            position = 360.0f + position;
                        if((delta = position - wrap_target.xyz[idx]) < -180.0f)
                            position += 360.0f;
                        else if(delta > 180.0f)
                            position -= 360.0f;
                        sys.position[idx] = lroundf(position * settings.axis[idx].steps_per_mm);
                    }
                }

                sync_position();
                mc_line(wrap_target.xyz, &plan_data);
                protocol_buffer_synchronize();

                for(idx = Z_AXIS + 1; idx < N_AXIS; idx++) {
                    if(bit_istrue(gc_block.rotary_wrap.mask, bit(idx)))
                        sys.position[idx] = lroundf(gc_block.values.coord_data.xyz[idx] * settings.axis[idx].steps_per_mm);
                }

                sync_position();
            } else
#endif
            mc_line(gc_block.values.coord_data.xyz, &plan_data);
            memcpy(gc_state.position, gc_block.values.coord_data.xyz, sizeof(gc_state.position));
            set_scaling(1.0f);
            break;

        case NonModal_SetHome_0:
            settings_write_coord_data(CoordinateSystem_G28, &gc_state.position);
            break;

        case NonModal_SetHome_1:
            settings_write_coord_data(CoordinateSystem_G30, &gc_state.position);
            break;

        case NonModal_MacroCall:
            {
#if NGC_PARAMETERS_ENABLE
                ngc_named_param_set("_value", 0.0f);
                ngc_named_param_set("_value_returned", 0.0f);
#endif

                status_code_t status = grbl.on_macro_execute((macro_id_t)gc_block.values.p);

#if NGC_PARAMETERS_ENABLE
                if(status != Status_Handled)
                    ngc_call_pop();
#endif
                return status == Status_Unhandled ? Status_GcodeValueOutOfRange : (status == Status_Handled ? Status_OK : status);
            }
            break;

        case NonModal_SetCoordinateOffset: // G92
            gc_state.g92_coord_offset_applied = true; // TODO: check for all zero?
            memcpy(gc_state.g92_coord_offset, gc_block.values.xyz, sizeof(gc_state.g92_coord_offset));
            if(!settings.flags.g92_is_volatile)
                settings_write_coord_data(CoordinateSystem_G92, &gc_state.g92_coord_offset); // Save G92 offsets to non-volatile storage
            add_offset();
            break;

        case NonModal_ResetCoordinateOffset: // G92.1
            gc_state.g92_coord_offset_applied = false;
            clear_vector(gc_state.g92_coord_offset); // Disable G92 offsets by zeroing offset vector.
            if(!settings.flags.g92_is_volatile)
                settings_write_coord_data(CoordinateSystem_G92, &gc_state.g92_coord_offset); // Save G92 offsets to non-volatile storage
            add_offset();
            break;

        case NonModal_ClearCoordinateOffset: // G92.2
            gc_state.g92_coord_offset_applied = false;
            clear_vector(gc_state.g92_coord_offset); // Disable G92 offsets by zeroing offset vector.
            add_offset();
            break;

        case NonModal_RestoreCoordinateOffset: // G92.3
            gc_state.g92_coord_offset_applied = true; // TODO: check for all zero?
            settings_read_coord_data(CoordinateSystem_G92, &gc_state.g92_coord_offset); // Restore G92 offsets from non-volatile storage
            add_offset();
            break;

        default:
            break;
    }

    // [20. Motion modes ]:
    // NOTE: Commands G10,G28,G30,G92 lock out and prevent axis words from use in motion modes.
    // Enter motion modes only if there are axis words or a motion mode command word in the block.
    gc_state.modal.motion = gc_block.modal.motion;
    gc_state.modal.canned_cycle_active = gc_block.modal.canned_cycle_active;

    if(gc_state.modal.motion != MotionMode_None && axis_command == AxisCommand_MotionMode) {

        plan_data.output_commands = output_commands;
#if ENABLE_PATH_BLENDING
        plan_data.cam_tolerance = gc_state.cam_tolerance;
        plan_data.path_tolerance = gc_state.path_tolerance;
#endif
        output_commands = NULL;

        pos_update_t gc_update_pos = GCUpdatePos_Target;

        switch(gc_state.modal.motion) {

            case MotionMode_Linear:
                if(gc_state.modal.feed_mode == FeedMode_UnitsPerRev) {
                    plan_data.condition.units_per_rev = On;
                    plan_data.spindle.state.synchronized = settings.mode != Mode_Lathe || gc_block.values.xyz[Z_AXIS] != gc_state.position[Z_AXIS];
                //??    gc_state.distance_per_rev = plan_data.feed_rate;
                    // check initial feed rate - fail if zero?
                }
                mc_line(gc_block.values.xyz, &plan_data);
                break;

            case MotionMode_Seek:
                plan_data.condition.rapid_motion = On; // Set rapid motion condition flag.
                mc_line(gc_block.values.xyz, &plan_data);
                break;

            case MotionMode_CwArc:
            case MotionMode_CcwArc:
                if(gc_state.modal.feed_mode == FeedMode_UnitsPerRev)
                    plan_data.condition.units_per_rev = plan_data.spindle.state.synchronized = On;

                mc_arc(gc_block.values.xyz, &plan_data, gc_state.position, gc_block.values.ijk, gc_block.values.r,
                        plane, gc_parser_flags.arc_is_clockwise ? -gc_block.arc_turns : gc_block.arc_turns);
                break;

            case MotionMode_CubicSpline:
                {
                    point_2d_t cp1 = {
                        .x = gc_state.position[X_AXIS] + gc_block.values.ijk[X_AXIS],
                        .y = gc_state.position[Y_AXIS] + gc_block.values.ijk[Y_AXIS]
                    };
                    point_2d_t cp2 = {
                        .x = gc_block.values.xyz[X_AXIS] + gc_state.modal.spline_pq[X_AXIS],
                        .y = gc_block.values.xyz[Y_AXIS] + gc_state.modal.spline_pq[Y_AXIS]
                    };
                    mc_cubic_b_spline(gc_block.values.xyz, &plan_data, gc_state.position, cp1.values, cp2.values);
                }
                break;

            case MotionMode_QuadraticSpline:
                {
                    point_2d_t cp1 = {
                        .x = gc_state.position[X_AXIS] + (gc_block.values.ijk[X_AXIS] * 2.0f) / 3.0f,
                        .y = gc_state.position[Y_AXIS] + (gc_block.values.ijk[Y_AXIS] * 2.0f) / 3.0f
                    };
                    point_2d_t cp2 = {
                        .x = gc_block.values.xyz[X_AXIS] + ((gc_state.position[X_AXIS] + gc_block.values.ijk[X_AXIS] - gc_block.values.xyz[X_AXIS]) * 2.0f) / 3.0f,
                        .y = gc_block.values.xyz[Y_AXIS] + ((gc_state.position[Y_AXIS] + gc_block.values.ijk[Y_AXIS] - gc_block.values.xyz[Y_AXIS]) * 2.0f) / 3.0f
                    };
                    mc_cubic_b_spline(gc_block.values.xyz, &plan_data, gc_state.position, cp1.values, cp2.values);
                }
                break;

            case MotionMode_SpindleSynchronized:
                {
                    protocol_buffer_synchronize(); // Wait until any previous moves are finished.

                    gc_override_flags_t overrides = sys.override.control; // Save current override disable status.

                    status_code_t status = init_sync_motion(&plan_data, gc_block.values.k);
                    if(status != Status_OK)
                        FAIL(status);

                    plan_data.spindle.state.synchronized = On;

                    mc_line(gc_block.values.xyz, &plan_data);

                    protocol_buffer_synchronize();    // Wait until synchronized move is finished,
                    sys.override.control = overrides; // then restore previous override disable status.
                }
                break;

            case MotionMode_Threading:
                {
                    protocol_buffer_synchronize(); // Wait until any previous moves are finished.

                    gc_override_flags_t overrides = sys.override.control; // Save current override disable status.

                    status_code_t status = init_sync_motion(&plan_data, thread.pitch);
                    if(status != Status_OK)
                        FAIL(status);

                    mc_thread(&plan_data, gc_state.position, &thread, overrides.feed_hold_disable);

                    sys.override.control = overrides; // then restore previous override disable status.
                }
                break;

            case MotionMode_DrillChipBreak:
            case MotionMode_CannedCycle81:
            case MotionMode_CannedCycle82:
            case MotionMode_CannedCycle83:;
                plan_data.spindle.rpm = gc_block.values.s;
                gc_state.canned.retract_mode = gc_state.modal.retract_mode;
                mc_canned_drill(gc_state.modal.motion, gc_block.values.xyz, &plan_data, gc_state.position, plane, gc_block.values.l, &gc_state.canned);
                break;

            case MotionMode_ProbeToward:
            case MotionMode_ProbeTowardNoError:
            case MotionMode_ProbeAway:
            case MotionMode_ProbeAwayNoError:
                // NOTE: gc_block.values.xyz is returned from mc_probe_cycle with the updated position value. So
                // upon a successful probing cycle, the machine position and the returned value should be the same.
                plan_data.condition.no_feed_override = !settings.probe.allow_feed_override;
                gc_update_pos = (pos_update_t)mc_probe_cycle(gc_block.values.xyz, &plan_data, gc_parser_flags);
                break;

            default:
                break;
        }

        // Do not update position on cancel (already done in protocol_exec_rt_system)
        if(sys.cancel)
            gc_update_pos = GCUpdatePos_None;

        //  Clean out any remaining output commands (may linger on error)
        while(plan_data.output_commands) {
            output_command_t *next = plan_data.output_commands->next;
            free(plan_data.output_commands);
            plan_data.output_commands = next;
        }

        // As far as the parser is concerned, the position is now == target. In reality the
        // motion control system might still be processing the action and the real tool position
        // in any intermediate location.
        if (gc_update_pos == GCUpdatePos_Target)
            memcpy(gc_state.position, gc_block.values.xyz, sizeof(gc_state.position)); // gc_state.position[] = gc_block.values.xyz[]
        else if (gc_update_pos == GCUpdatePos_System)
            gc_sync_position(); // gc_state.position[] = sys.position
        // == GCUpdatePos_None
    }

    if(plan_data.message)
        gc_output_message(plan_data.message);

    // [21. Program flow ]:
    // M0,M1,M2,M30,M60: Perform non-running program flow actions. During a program pause, the buffer may
    // refill and can only be resumed by the cycle start run-time command.
    gc_state.modal.program_flow = gc_block.modal.program_flow;

    if(gc_state.modal.program_flow || sys.flags.single_block) {

        protocol_buffer_synchronize(); // Sync and finish all remaining buffered motions before moving on.

        if(gc_state.modal.program_flow == ProgramFlow_Return) {
            if(grbl.on_macro_return)
                grbl.on_macro_return();
        } else if(gc_state.modal.program_flow == ProgramFlow_Paused || gc_block.modal.program_flow == ProgramFlow_OptionalStop || gc_block.modal.program_flow == ProgramFlow_CompletedM60 || sys.flags.single_block) {
            if(!check_mode) {
                if(gc_block.modal.program_flow == ProgramFlow_CompletedM60 && hal.pallet_shuttle)
                    hal.pallet_shuttle();
                system_set_exec_state_flag(EXEC_FEED_HOLD); // Use feed hold for program pause.
                protocol_execute_realtime(); // Execute suspend.
            }
        } else { // == ProgramFlow_Completed
            // Upon program complete, only a subset of g-codes reset to certain defaults, according to
            // LinuxCNC's program end descriptions and testing. Only modal groups [G-code 1,2,3,5,7,12]
            // and [M-code 7,8,9] reset to [G1,G17,G90,G94,G40,G54,M5,M9,M48]. The remaining modal groups
            // [G-code 4,6,8,10,13,14,15] and [M-code 4,5,6] and the modal words [F,S,T,H] do not reset.

            if(!check_mode && gc_block.modal.program_flow == ProgramFlow_CompletedM30 && hal.pallet_shuttle)
                hal.pallet_shuttle();

            gc_state.file_run = false;
            gc_state.modal.motion = MotionMode_Linear;
            gc_block.modal.canned_cycle_active = false;
            gc_state.modal.plane_select = PlaneSelect_XY;
//            gc_state.modal.plane_select = settings.flags.lathe_mode ? PlaneSelect_ZX : PlaneSelect_XY;
            gc_state.modal.distance_incremental = false;
            gc_state.modal.feed_mode = FeedMode_UnitsPerMin;
// TODO: check           gc_state.distance_per_rev = 0.0f;
            // gc_state.modal.cutter_comp = CUTTER_COMP_DISABLE; // Not supported.
            if(gc_state.modal.coord_system.id != CoordinateSystem_G54) {
                gc_state.modal.coord_system.id = CoordinateSystem_G54;
                system_add_rt_report(Report_GWCO);
            }
            gc_state.modal.coolant = (coolant_state_t){0};
            gc_state.modal.override_ctrl.feed_rate_disable = Off;
            gc_state.modal.override_ctrl.spindle_rpm_disable = Off;
#if ENABLE_ACCELERATION_PROFILES
            gc_state.modal.acceleration_factor = gc_get_accel_factor(0);
#endif

#if N_SYS_SPINDLE > 1

            idx = N_SYS_SPINDLE;
            spindle_t *spindle;
            do {
                if((spindle = &gc_state.modal.spindle[--idx])->hal) {
                    spindle->css = NULL;
                    spindle->state = (spindle_state_t){0};
                    spindle->rpm_mode = SpindleSpeedMode_RPM; // NOTE: not compliant with linuxcnc (?);
                    spindle->hal->param->state.override_disable = Off;
                    if(settings.flags.restore_overrides)
                        spindle->hal->param->override_pct = DEFAULT_SPINDLE_RPM_OVERRIDE;
                }
            } while(idx);
#else
            gc_state.modal.spindle.css = NULL;
            gc_state.modal.spindle.state = (spindle_state_t){0};
            gc_state.modal.spindle.rpm_mode = SpindleSpeedMode_RPM; // NOTE: not compliant with linuxcnc (?)
            gc_state.modal.spindle.hal->param->state.override_disable = Off;
            if(settings.flags.restore_overrides)
                sspindle->hal->param->override_pct = DEFAULT_SPINDLE_RPM_OVERRIDE;
#endif

            if(settings.parking.flags.enabled)
                gc_state.modal.override_ctrl.parking_disable = settings.parking.flags.enable_override_control &&
                                                                settings.parking.flags.deactivate_upon_init;
            sys.override.control = gc_state.modal.override_ctrl;

            if(settings.flags.restore_overrides) {
                sys.override.feed_rate = DEFAULT_FEED_OVERRIDE;
                sys.override.rapid_rate = DEFAULT_RAPID_OVERRIDE;
            }

            // Execute coordinate change and spindle/coolant stop.
            if (!check_mode) {

                if (!(settings_read_coord_data(gc_state.modal.coord_system.id, &gc_state.modal.coord_system.xyz)))
                    FAIL(Status_SettingReadFail);

#if COMPATIBILITY_LEVEL <= 1
                float g92_offset_stored[N_AXIS];
                if(settings_read_coord_data(CoordinateSystem_G92, &g92_offset_stored) && !isequal_position_vector(g92_offset_stored, gc_state.g92_coord_offset))
                    settings_write_coord_data(CoordinateSystem_G92, &gc_state.g92_coord_offset); // Save G92 offsets to non-volatile storage
#endif

                system_flag_wco_change(); // Set to refresh immediately just in case something altered.

                spindle_all_off();
                hal.coolant.set_state(gc_state.modal.coolant);
                system_add_rt_report(Report_Spindle); // Set to report change
                system_add_rt_report(Report_Coolant); // immediately.
            }

            if(grbl.on_program_completed)
                grbl.on_program_completed(gc_state.modal.program_flow, check_mode);

            // Clear any pending output commands
            while(output_commands) {
                output_command_t *next = output_commands->next;
                free(output_commands);
                output_commands = next;
            }

#if NGC_PARAMETERS_ENABLE
            ngc_modal_state_invalidate();
#endif

            grbl.report.feedback_message(Message_ProgramEnd);
        }
        gc_state.modal.program_flow = ProgramFlow_Running; // Reset program flow.
    }

#if NGC_EXPRESSIONS_ENABLE
    if(ngc_param_count) do {
        ngc_param_count--;
        ngc_param_set(ngc_params[ngc_param_count].id, ngc_params[ngc_param_count].value);
    } while(ngc_param_count);
#endif

    // TODO: % to denote start of program.

    return Status_OK;
}
