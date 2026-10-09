// penta_axis_bc.c
//
// RTCP-Kinematics for BC-head/table configuration (2 rotary axis, one on the tool head, one on the table).
// Vectorchain: MCS -> B_Center -> C_Center -> RTCP_csys -> TCP
// Author: DerAndere (@DerAndere1)
//
// Copyright 2024 - 2026 DerAndere
//
// Based on a relicensed verion of LinuxCNC file maxkins.c and grblHAL core file rtcp_ac.c:
//
// Based on a relicensed verion of LinuxCNC file maxkins.c
// Author: Chris Radek <chris@timeguy.com>
//
// Copyright (c) 2007, 2022 Chris Radek
//
// Based on grblHAL file rtcp_ac.c
// Author: @NicoDetzler

/*** EXPERIMENTAL AND UNTESTED, MIGTH BE REMOVED ***/

#include "../grbl.h"

#if PENTA_AXIS_HT_BC

#if N_AXIS < 5 || AXIS3_LETTER != 'B' || AXIS4_LETTER != 'C'
#error Illegal axis configuration for PENTA_AXIS_HT_BC - should be XYZBC!
#endif

#include "../hal.h"
#include "../settings.h"
#include "../nvs_buffer.h"
#include "../planner.h"
#include "../motion_control.h"
#include "../protocol.h"
#include "interface.h"

#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

typedef struct {
    // currently only the Z component of the b_vector is accounted for. All other offsets are not yet supported and should be 0.
    point_3d_t b_vector;    // vector from gage line to B center when all axes are at MCS position 0.
    point_3d_t c_vector;    // C_center. Not yet supported, should be 0.
    float segment_length;
    float reserved[3];      // reserved for future use
} kinematics_settings_t;

#define RTCP_ARC_SAMPLES   64   // Pass 1
#define RTCP_MAX_SEGMENTS  1000 // safety upper limit for amount of segments Pass 2

static float rtcp_s_tab[RTCP_ARC_SAMPLES + 1];
static point_3d_t rtcp_mcs_tab[RTCP_ARC_SAMPLES + 1];
static float rtcp_len_tab[RTCP_ARC_SAMPLES + 1];
static uint8_t rtcp_mode;                   // flag to enable rtcp. 1 -> RTCP on, 0 -> RTCP off
static coord_data_t start_rtcp;             // actual tool center point displayed in RTCP-csys
static nvs_address_t nvs_address;
static user_mcode_ptrs_t user_mcode;
static kinematics_settings_t kinematics_settings;
static point_3d_t c_to_active_csys = {0};   // vector from C-Center to the RTCP-csys. It is set with M852 and depending on the actual B and C angles, and the G54/G55/G56...offsets

// mathematical helpers
static inline float deg2rad (float deg)
{
    return deg * RADDEG;
}

FLASHMEM static void rtcp_enable (void) //enables RTCP and displays the actual toolcenterpoint
{
    char buf[256];

    rtcp_mode = 1;
    hal.stream.write_all("[MSG:RTCP active.]" ASCII_EOL);
    snprintf(buf, sizeof(buf),
             "[MSG:RTCP_aktuell: X=%.3f, Y=%.3f, Z=%.3f, B=%.3f, C=%.3f]" ASCII_EOL,
             start_rtcp.x, start_rtcp.y, start_rtcp.z, start_rtcp.b ,start_rtcp.c);
    hal.stream.write_all(buf);
}

static void rtcp_disable (void) //disables RTCP
{
    rtcp_mode = 0;

    hal.stream.write_all("[MSG:RTCP deactivated.]" ASCII_EOL);
}

// -----------------------------------------------------------------------------
// Inverse kinematics transformation: RTCP-CSYS -> machine coordinates (MCS)
// target: TCP displayed in RTCP-CSYS
// B_deg, C_deg: axisangles in degree
// -----------------------------------------------------------------------------

static coord_data_t *rtcp_inverse (coord_data_t *target, coord_data_t *position) //inverse  kinematics transform any target point in the g_code into the RTCP-csys
{
    const float B = deg2rad(position->b);
    const float C = deg2rad(position->c);

    const float cosB = cosf(B);
    const float sinB = sinf(B);
    const float cosC = cosf(C);
    const float sinC = sinf(C);

    const float pivot_length = kinematics_settings.b_vector[Z_AXIS] + gc_state.modal.tool_length_offset[Z_AXIS];
  
    // B correction
    const float zb = pivot_length * cosB - kinematics_settings.b_vector[Z_AXIS];
    const float xb = pivot_length * sinB;

    // C correction
    const float xyr = hypot_f(x_trans, y_trans);
    const float xytheta = atan2f(y_trans, x_trans) - C;

    target->x = xyr * cos(xytheta) + xb, x_trans + xb;
    target->y = xyr * sin(xytheta), y_trans;
    target->z = z_trans + zb;
    target->b = position->b;
    target->c = position->c;

    return target;
}

// -----------------------------------------------------------------------------
// forward kinematics transformation: machinecoordinates (MCS) -> RTCP-csys
// -----------------------------------------------------------------------------

static coord_data_t *rtcp_forward (coord_data_t *target, coord_data_t *position) //calculates any machine position to RTCP-csys
{
    const float x_trans = position->x + gc_state.modal.tool_length_offset[X_AXIS];
    const float y_trans = position->y + gc_state.modal.tool_length_offset[Y_AXIS];
    const float z_trans = position->z;

    const float B = deg2rad(position->b);
    const float C = deg2rad(position->c);

    const float cosB = cosf(B);
    const float sinB = sinf(B);
    const float cosC = cosf(C);
    const float sinC = sinf(C);

    const float pivot_length = kinematics_settings.b_vector[Z_AXIS] + gc_state.modal.tool_length_offset[Z_AXIS];

    // B correction
    const float zb = pivot_length * cosB;
    const float xb = pivot_length * sinB;

    // C correction
    const float xyr = hypot_f(position->x, position->y);
    const float xytheta = atan2f(position->y, position->x) + C;

    target->x = xyr * cos(xytheta) - xb;
    target->y = xyr * sin(xytheta);
    target->z = position->z - zb + pivot_lengt;
    target->b = position->b;
    target->c = position->c;

    return target;
}

// -----------------------------------------------------------------------------
// actual machine position displayed in RTCP-csys (starting point , is set via M851)
// -----------------------------------------------------------------------------

coord_data_t *calc_pos_in_rtcp (coord_data_t *position, mpos_t *steps)
{
    uint_fast8_t idx = N_AXIS;
    coord_data_t cpos;

    do {
        idx--;
        cpos.values[idx] = (float)steps->values[idx] / settings.axis[idx].steps_per_mm;
    } while(idx);

    return rtcp_inverse(position, &cpos);
}

// -----------------------------------------------------------------------------
// calculation of c_to_active_csys (is set via M852)
// -----------------------------------------------------------------------------

static void rtcp_measure(void)
{
    float B = deg2rad(sys.position[B_AXIS] / settings.axis[B_AXIS].steps_per_mm);
    float C = deg2rad(sys.position[C_AXIS] / settings.axis[C_AXIS].steps_per_mm);

    float cosB = cosf(B);
    float sinB = sinf(B);
    float cosC = cosf(C);
    float sinC = sinf(C);

    float x_trans = gc_state.modal.g5x_offset.data.coord.x;
    float y_trans = gc_state.modal.g5x_offset.data.coord.y;
    float z_trans = gc_state.modal.g5x_offset.data.coord.z;

    const float pivot_length = kinematics_settings.b_vector[Z_AXIS] + gc_state.modal.tool_length_offset[Z_AXIS];

    // B correction
    const float zb = pivot_length * cosB;
    const float xb = pivot_length * sinB;

    // C correction
    const float xyr = hypot_f(position->x, position->y);
    const float xytheta = atan2f(position->y, position->x) + C;

    c_to_active_csys.x = xyr * cos(xytheta) - xb;
    c_to_active_csys.y = xyr * sin(xytheta);
    c_to_active_csys.z = position->z - zb + pivot_lengt;

    char buf[128];
    snprintf(buf, sizeof(buf),
             "[RTCP] c_to_active_csys: X=%.3f, Y=%.3f, Z=%.3f\n",
             c_to_active_csys.x,
             c_to_active_csys.y,
             c_to_active_csys.z);
    hal.stream.write_all(buf);
}

// Cartesian passthru

static coord_data_t *rtcp_inverse_cartesian (coord_data_t *target, coord_data_t *position)
{
    return position;
}

static coord_data_t *calc_pos_in_cartesian (coord_data_t *position, mpos_t *steps)
{
    uint_fast8_t idx = N_AXIS;

    do {
        idx--;
        position->values[idx] = (float)steps->values[idx] / settings.axis[idx].steps_per_mm;
    } while(idx);

    return position;
}

static coord_data_t *kinematics_passthru (coord_data_t *target, coord_data_t *position, plan_line_data_t *pl_data, bool init)
{
    static coord_data_t trsf;
    static uint_fast8_t iterations;

    if(init) {
        iterations = 2;
        memcpy(&trsf, target, sizeof(coord_data_t));
    }

    return iterations-- == 0 ? NULL : &trsf;
}

// -----------------------------------------------------------------------------
// RTCP-segmentation through radian parameter (2-pass, terminated)
//
// Pass 1: curve is segmented with fix amount of segments. Those segments are summed up.
// The total lengt devided by the rtcp_segment length gives an approximated value for the real amount of segments needed

// -----------------------------------------------------------------------------

static uint_fast16_t rtcp_segments (coord_data_t *end_rtcp, coord_data_t *delta, float *total_len)
{
    coord_data_t position, mcs;

    // --- Pass 1: segment curve rough, MCS-radians sum --------------------
    memcpy(&rtcp_mcs_tab[0], rtcp_inverse(&mcs, &start_rtcp), sizeof(float) * 3);
    rtcp_s_tab[0] = rtcp_len_tab[0] = 0.0f;

    for (int i = 1; i <= RTCP_ARC_SAMPLES; i++) {
        float s = (float)i / (float)RTCP_ARC_SAMPLES;

        uint_fast8_t idx = N_AXIS;

        do {
            idx--;
            position.values[idx]  = start_rtcp.values[idx] + s * delta->values[idx];
        } while(idx);

        rtcp_inverse(&mcs, &position);

        float dx = mcs.x - rtcp_mcs_tab[i - 1].x;
        float dy = mcs.y - rtcp_mcs_tab[i - 1].y;
        float dz = mcs.z - rtcp_mcs_tab[i - 1].z;

        rtcp_s_tab[i]   = s;
        memcpy(rtcp_mcs_tab[i].values, mcs.values, sizeof(float) * 3);
        rtcp_len_tab[i] = rtcp_len_tab[i - 1] + sqrtf(dx * dx + dy * dy + dz * dz);
    }

    *total_len = rtcp_len_tab[RTCP_ARC_SAMPLES];

    // --- calc amount of segments -----------------------------------------------
    uint_fast16_t n_seg = (uint_fast16_t)(*total_len / kinematics_settings.segment_length + 0.5f); //+0.5 to round up
    if (n_seg < 1)
        n_seg = 1;

    return n_seg > RTCP_MAX_SEGMENTS ? RTCP_MAX_SEGMENTS : n_seg;
}

static coord_data_t *kinematics_segment_line (coord_data_t *target, coord_data_t *position, plan_line_data_t *pl_data, bool init)
{
    static bool segment;
    static uint_fast16_t iterations, idx;
    static float total_len, step_len, target_len;
    static coord_data_t trsf, delta;

    if(init) {

        coord_data_t end_rtcp;
        // tolerance
        const float eps = 1e-6f;

        idx = N_AXIS;
        do {
            idx--;
            end_rtcp.values[idx]  = target->values[idx] - gc_state.modal.g5x_offset.data.coord.values[idx];
        } while(idx);

        idx = N_AXIS;
        do {
            idx--;
            delta.values[idx]  = end_rtcp.values[idx] - start_rtcp.values[idx];
        } while(idx);

        if(!(segment = fabsf(delta.b) >= eps || fabsf(delta.c) >= eps)) {
            iterations = 2;
            rtcp_forward(&trsf, &end_rtcp);
        } else {
            idx = 0;
            target_len = 0.0f;
            iterations = rtcp_segments(&end_rtcp, &delta, &total_len);
            step_len = total_len / (float)iterations;
            iterations++;
        }
        memcpy(&start_rtcp, &end_rtcp, sizeof(coord_data_t));

    } else if(segment) {

        //
        // Pass 2: segmentation with the calculated amount of segments
        //
        target_len += step_len;

        while (idx < RTCP_ARC_SAMPLES && rtcp_len_tab[idx] < target_len)
            idx++;

        float s_out;
        if (idx == 0)
            s_out = 0.0f;
        else if(iterations == 1)
            s_out = 1.0f; // exactly fit endpoint , probably no float drift
        else {
            float len0 = rtcp_len_tab[idx - 1];
            float len1 = rtcp_len_tab[idx];
            float frac = (len1 > len0) ? (target_len - len0) / (len1 - len0) : 0.0f;
            s_out = rtcp_s_tab[idx - 1] + frac * (rtcp_s_tab[idx] - rtcp_s_tab[idx - 1]);
        }

        coord_data_t out = {
            .x = start_rtcp.x + s_out * delta.x,
            .y = start_rtcp.y + s_out * delta.y,
            .z = start_rtcp.z + s_out * delta.z,
            .b = start_rtcp.b + s_out * delta.b,
            .c = start_rtcp.c + s_out * delta.c
        };

        rtcp_forward(&trsf, &out);
    }

    return iterations-- == 0 ? NULL : &trsf;
}

FLASHMEM static user_mcode_type_t mcode_check (user_mcode_t mcode)
{
    return mcode == 850 || mcode == 851 || mcode == 852
                     ? UserMCode_NoValueWords
                     : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Unsupported);
}

FLASHMEM static status_code_t mcode_validate (parser_block_t *gc_block)
{
    status_code_t state = Status_OK;

    switch((uint16_t)gc_block->user_mcode) {

        case 850:
        case 851:
        case 852:
            gc_block->user_mcode_sync = On;
            break;

        default:
            state = Status_Unhandled;
            break;
    }

    return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block) : state;
}

FLASHMEM static void mcode_execute (uint_fast16_t state, parser_block_t *gc_block)
{
    bool handled = true;

    switch((uint16_t)gc_block->user_mcode) {

         case 850:
             rtcp_disable();
             kinematics.transform_steps_to_cartesian = (transform_steps_to_cartesian_ptr)calc_pos_in_cartesian;
             kinematics.segment_line = (segment_line_ptr)kinematics_passthru;
             sync_position();
             break;

         case 851:
             kinematics.transform_steps_to_cartesian = (transform_steps_to_cartesian_ptr)calc_pos_in_rtcp;
             kinematics.segment_line = (segment_line_ptr)kinematics_segment_line;
             calc_pos_in_rtcp(&start_rtcp, (mpos_t *)sys.position);
             rtcp_enable();
             break;

         case 852:
             rtcp_measure();
             break;

         default:
            handled = false;
            break;
    }

    if(!handled && user_mcode.execute)
        user_mcode.execute(state, gc_block);
}

PROGMEM static const setting_group_detail_t rtcp_groups [] = {
    { Group_Root, Group_Kinematics, "Kinematics offsets" } //Here
};

PROGMEM static const setting_detail_t rtcp_settings[] = {
    { Setting_Kinematics0, Group_Kinematics, "RTCP B offset X", "mm", Format_Decimal, "#####0.000", "-1000", "1000", Setting_IsExtended, &kinematics_settings.b_vector.x },
    { Setting_Kinematics1, Group_Kinematics, "RTCP B offset Y", "mm", Format_Decimal, "#####0.000", "-1000", "1000", Setting_IsExtended, &kinematics_settings.b_vector.y },
    { Setting_Kinematics2, Group_Kinematics, "RTCP B offset Z", "mm", Format_Decimal, "#####0.000", "-1000", "1000", Setting_IsExtended, &kinematics_settings.b_vector.z },
    { Setting_Kinematics3, Group_Kinematics, "RTCP C offset X", "mm", Format_Decimal, "#####0.000", "-1000", "1000", Setting_IsExtended, &kinematics_settings.c_vector.x },
    { Setting_Kinematics4, Group_Kinematics, "RTCP C offset Y", "mm", Format_Decimal, "#####0.000", "-1000", "1000", Setting_IsExtended, &kinematics_settings.c_vector.y },
    { Setting_Kinematics5, Group_Kinematics, "RTCP C offset Z", "mm", Format_Decimal, "#####0.000", "-1000", "1000", Setting_IsExtended, &kinematics_settings.c_vector.z },
    { Setting_Kinematics6, Group_Kinematics, "RTCP segment length", "mm", Format_Decimal, "#####0.000", "0.010", "1.000", Setting_IsExtended, &kinematics_settings.segment_length },
/*
    { Setting_Kinematics7, Group_Kinematics, "RTCP reserved", "mm", Format_Decimal, "#####0.000", "-1000", "1000", Setting_IsExtended, &kinematics_settings.reserved[0] },
    { Setting_Kinematics8, Group_Kinematics, "RTCP reserved", "mm", Format_Decimal, "#####0.000", "-1000", "1000", Setting_IsExtended, &kinematics_settings.reserved[1] },
    { Setting_Kinematics9, Group_Kinematics, "RTCP reserved", "mm", Format_Decimal, "#####0.000", "-1000", "1000", Setting_IsExtended, &kinematics_settings.reserved[2] }
*/
};

/*
PROGMEM static const setting_descr_t rtcp_settings_descr[] = {

};
*/

FLASHMEM static void rtcp_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&kinematics_settings, sizeof(kinematics_settings_t), true);
}

FLASHMEM static void rtcp_settings_restore (void)
{
    static const kinematics_settings_t defaults = {     // offset values of the rotary centers and the segmentlength are saved here, can be changed via $640-$649
        .b_vector.x = 0.0f,                         // vector from gage line to B center when all axes are at MCS position 0.
        .b_vector.y = 0.0f,
        .b_vector.z = 100.0f,
        .c_vector.x = 0.0f,                         // Not yet supported
        .c_vector.y = 0.0f,
        .c_vector.z = 0.0f,
        .segment_length = 0.500f                    // rough segment length for the curve
    };

    memcpy(&kinematics_settings, &defaults, sizeof(kinematics_settings_t));

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&kinematics_settings, sizeof(kinematics_settings_t), true);
}

FLASHMEM static void rtcp_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&kinematics_settings, nvs_address, sizeof(kinematics_settings_t), true) != NVS_TransferResult_OK)
        rtcp_settings_restore();
}

FLASHMEM static uint_fast8_t get_axis_mask (uint_fast8_t idx)
{
    return bit(idx);
}

FLASHMEM static void set_target_pos (uint_fast8_t idx) // fn name?
{
    sys.position[idx] = 0;
}

FLASHMEM static void set_machine_positions (axes_signals_t cycle)
{
    limits_set_machine_positions(cycle, true);
}

FLASHMEM static bool homing_cycle_validate (axes_signals_t cycle)
{
    return true;
}

FLASHMEM static float homing_cycle_get_feedrate (axes_signals_t cycle, float feedrate, homing_mode_t mode)
{
    return feedrate;
}

// Initialize API pointers for PENTA_AXIS_HT kinematics
FLASHMEM void rtcp_init (void)
{
    static setting_details_t setting_details = {
        .groups = rtcp_groups,
        .n_groups = sizeof(rtcp_groups) / sizeof(setting_group_detail_t),
        .settings = rtcp_settings,
        .n_settings = sizeof(rtcp_settings) / sizeof(setting_detail_t),
//        .descriptions = rtcp_settings_descr,
//        .n_descriptions = sizeof(rtcp_settings_descr) / sizeof(setting_descr_t),
        .save = rtcp_settings_save,
        .load = rtcp_settings_load,
        .restore = rtcp_settings_restore
    };

    if((nvs_address = nvs_alloc(sizeof(kinematics_settings_t)))) {

        kinematics.transform_from_cartesian = (transform_from_cartesian_ptr)rtcp_inverse_cartesian; // called from homing routine - RTCP should be turned off during homing?
        kinematics.transform_steps_to_cartesian = (transform_steps_to_cartesian_ptr)calc_pos_in_cartesian;
        kinematics.segment_line = (segment_line_ptr)kinematics_passthru;

        kinematics.limits_set_target_pos = set_target_pos;
        kinematics.limits_get_axis_mask = get_axis_mask;
        kinematics.limits_set_machine_positions = set_machine_positions;
        kinematics.homing_cycle_validate = homing_cycle_validate;
        kinematics.homing_cycle_get_feedrate = homing_cycle_get_feedrate;

        settings_register(&setting_details);

        memcpy(&user_mcode, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));

        grbl.user_mcode.check = mcode_check;
        grbl.user_mcode.validate = mcode_validate;
        grbl.user_mcode.execute = mcode_execute;
    }
}

#endif // PENTA_AXIS_HT_BC
