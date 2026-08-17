// rtcp_ac.c
//
// RTCP-Kinematik for AC-table/table configuration (2 rotary axis, both on the table, C sits on A).
// Vectorchain: MCS -> A_Center -> C_Center -> RTCP_csys -> TCP
//
// Tool length is not included in the vector chain, because of the machine-build the tool axis is always perpendicular to the z-axie,
// tool length offset can be easily added as a sum ontop of z-value


/*** EXPERIMENTAL AND UNTESTED, MIGTH BE REMOVED ***/

// By @NicoDetzler - see https://github.com/grblHAL/core/issues/965

#include "../grbl.h"

#if RTCP_AC

#if N_AXIS < 5 || AXIS3_LETTER != 'A' || AXIS4_LETTER != 'C'
#error Illegal axis configuration for RTCP AC - should be XYZAC!
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
    point_3d_t a_vector;    // A_center
    point_3d_t c_vector;    // C_center
    float segment_length;
    float reserved[3];      // reserved for future use
} rtp_ac_settings_t;

#define RTCP_ARC_SAMPLES   64   // Pass 1
#define RTCP_MAX_SEGMENTS  1000 // safety upper limit for amount of segments Pass 2

static float rtcp_s_tab[RTCP_ARC_SAMPLES + 1];
static point_3d_t rtcp_mcs_tab[RTCP_ARC_SAMPLES + 1];
static float rtcp_len_tab[RTCP_ARC_SAMPLES + 1];
static uint8_t rtcp_mode;                   // flag to enable rtcp. 1 -> RTCP on, 0 -> RTCP off
static coord_data_t start_rtcp;             // actual tool center point displayed in RTCP-csys
static nvs_address_t nvs_address;
static user_mcode_ptrs_t user_mcode;
static rtp_ac_settings_t rtp_ac_settings;
static point_3d_t c_to_active_csys = {0};   // vector from C-Center to the RTCP-csys. It is set with M852 and depending on the actual A and C angles, and the G54/G55/G56...offsets

// mathematical helpers
static inline float deg2rad (float deg)
{
    return deg * RADDEG;
}

FLASHMEM static void rtcp_ac_enable (void) //enables RTCP and displays the actual toolcenterpoint
{
    char buf[256];

    rtcp_mode = 1;
    hal.stream.write_all("[MSG:RTCP aktiviert.]" ASCII_EOL);
    snprintf(buf, sizeof(buf),
             "[MSG:RTCP_aktuell: X=%.3f, Y=%.3f, Z=%.3f, A=%.3f, C=%.3f]" ASCII_EOL,
             start_rtcp.x, start_rtcp.y, start_rtcp.z, start_rtcp.a ,start_rtcp.c);
    hal.stream.write_all(buf);
}

static void rtcp_ac_disable (void) //disables RTCP
{
    rtcp_mode = 0;

    hal.stream.write_all("[MSG:RTCP deaktiviert.]" ASCII_EOL);
}

// -----------------------------------------------------------------------------
// Forward-Trafo: RTCP-CSYS -> machinecoordinates(MCS)
// target: TCP displayed in RTCP-CSYS
// A_deg, C_deg: axisangles in degree
// -----------------------------------------------------------------------------

static coord_data_t *rtcp_forward (coord_data_t *target, coord_data_t *position) //forward kinematic transforms any target point in the g_code into the RTCP-csys
{
    float A = deg2rad(position->a);
    float C = deg2rad(position->c);

    float cosA = cosf(A);
    float sinA = sinf(A);
    float cosC = cosf(C);
    float sinC = sinf(C);

    target->x =
        rtp_ac_settings.a_vector.x +
        (c_to_active_csys.x + position->x) * cosC -
        (c_to_active_csys.y + position->y) * sinC +
        rtp_ac_settings.c_vector.x;

    target->y =
        rtp_ac_settings.a_vector.y +
        ((c_to_active_csys.x + position->x) * sinC +
         (c_to_active_csys.y + position->y) * cosC +
         rtp_ac_settings.c_vector.y) * cosA -
        ((c_to_active_csys.z + position->z) + rtp_ac_settings.c_vector.z) * sinA;

    target->z =
        rtp_ac_settings.a_vector.z +
        ((c_to_active_csys.x + position->x) * sinC +
         (c_to_active_csys.y + position->y) * cosC +
         rtp_ac_settings.c_vector.y) * sinA +
        ((c_to_active_csys.z + position->z) + rtp_ac_settings.c_vector.z) * cosA;

    target->a = position->a;
    target->c = position->c;

    return target;
}

// -----------------------------------------------------------------------------
// Inverse-Trafo: machinecoordinates (MCS) -> RTCP-csys
// -----------------------------------------------------------------------------

static coord_data_t *rtcp_inverse (coord_data_t *target, coord_data_t *position) //calculates any machine position to RTCP-csys
{
    float x_trans = position->x;
    float y_trans = position->y;
    float z_trans = position->z;

    float A = deg2rad(position->a);
    float C = deg2rad(position->c);

    float cosA = cosf(A);
    float sinA = sinf(A);
    float cosC = cosf(C);
    float sinC = sinf(C);

    target->x =
        ((x_trans - rtp_ac_settings.a_vector.x -
          rtp_ac_settings.c_vector.x) * cosC) +
        ((((y_trans - rtp_ac_settings.a_vector.y) * cosA) +
          ((z_trans - rtp_ac_settings.a_vector.z) * sinA) -
          rtp_ac_settings.c_vector.y) * sinC) -
        c_to_active_csys.x;

    target->y =
        -((x_trans - rtp_ac_settings.a_vector.x -
           rtp_ac_settings.c_vector.x) * sinC) +
        ((((y_trans - rtp_ac_settings.a_vector.y) * cosA) +
          ((z_trans - rtp_ac_settings.a_vector.z) * sinA) -
          rtp_ac_settings.c_vector.y) * cosC) -
        c_to_active_csys.y;

    target->z =
        -((y_trans - rtp_ac_settings.a_vector.y) * sinA) +
        ((z_trans - rtp_ac_settings.a_vector.z) * cosA) -
        rtp_ac_settings.c_vector.z -
        c_to_active_csys.z;

    target->a = position->a;
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

static void rtcp_ac_measure(void)
{
    float A = deg2rad(sys.position[A_AXIS] / settings.axis[A_AXIS].steps_per_mm);
    float C = deg2rad(sys.position[C_AXIS] / settings.axis[C_AXIS].steps_per_mm);

    float cosA = cosf(A);
    float sinA = sinf(A);
    float cosC = cosf(C);
    float sinC = sinf(C);

    float x_trans = gc_state.modal.g5x_offset.data.coord.x;
    float y_trans = gc_state.modal.g5x_offset.data.coord.y;
    float z_trans = gc_state.modal.g5x_offset.data.coord.z;

    // Vorher wurde hier ein stets-Null-Vektor "target" abgezogen (kompletter
    // Copy&Paste-Rest aus rtcp_inverse ohne Wirkung) - entfernt, da wirkungslos.
    c_to_active_csys.x =
        ((x_trans - rtp_ac_settings.a_vector.x -
          rtp_ac_settings.c_vector.x) * cosC) +
        ((((y_trans - rtp_ac_settings.a_vector.y) * cosA) +
          ((z_trans - rtp_ac_settings.a_vector.z) * sinA) -
          rtp_ac_settings.c_vector.y) * sinC);

    c_to_active_csys.y =
        -((x_trans - rtp_ac_settings.a_vector.x -
           rtp_ac_settings.c_vector.x) * sinC) +
        ((((y_trans - rtp_ac_settings.a_vector.y) * cosA) +
          ((z_trans - rtp_ac_settings.a_vector.z) * sinA) -
          rtp_ac_settings.c_vector.y) * cosC);

    c_to_active_csys.z =
        -((y_trans - rtp_ac_settings.a_vector.y) * sinA) +
        ((z_trans - rtp_ac_settings.a_vector.z) * cosA) -
        rtp_ac_settings.c_vector.z;

    char buf[128];
    snprintf(buf, sizeof(buf),
             "[RTCP] c_to_active_csys: X=%.3f, Y=%.3f, Z=%.3f\n",
             c_to_active_csys.x,
             c_to_active_csys.y,
             c_to_active_csys.z);
    hal.stream.write_all(buf);
}

// Cartesian passthru

static coord_data_t *rtcp_forward_cartesian (coord_data_t *target, coord_data_t *position)
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
// The total lengt devided by the rtcp_segment length gives a approximated value for the real amount of segments needed

// -----------------------------------------------------------------------------

static uint_fast16_t rtcp_segments (coord_data_t *end_rtcp, coord_data_t *delta, float *total_len)
{
    coord_data_t position, mcs;

    // --- Pass 1: segment curve rough, MCS-radians sum --------------------
    memcpy(&rtcp_mcs_tab[0], rtcp_forward(&mcs, &start_rtcp), sizeof(float) * 3);
    rtcp_s_tab[0] = rtcp_len_tab[0] = 0.0f;

    for (int i = 1; i <= RTCP_ARC_SAMPLES; i++) {
        float s = (float)i / (float)RTCP_ARC_SAMPLES;

        uint_fast8_t idx = N_AXIS;

        do {
            idx--;
            position.values[idx]  = start_rtcp.values[idx] + s * delta->values[idx];
        } while(idx);

        rtcp_forward(&mcs, &position);

        float dx = mcs.x - rtcp_mcs_tab[i - 1].x;
        float dy = mcs.y - rtcp_mcs_tab[i - 1].y;
        float dz = mcs.z - rtcp_mcs_tab[i - 1].z;

        rtcp_s_tab[i]   = s;
        memcpy(rtcp_mcs_tab[i].values, mcs.values, sizeof(float) * 3);
        rtcp_len_tab[i] = rtcp_len_tab[i - 1] + sqrtf(dx * dx + dy * dy + dz * dz);
    }

    *total_len = rtcp_len_tab[RTCP_ARC_SAMPLES];

    // --- calc amount of segments -----------------------------------------------
    uint_fast16_t n_seg = (uint_fast16_t)(*total_len / rtp_ac_settings.segment_length + 0.5f); //+0.5 to round up
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

        if(!(segment = fabsf(delta.a) >= eps || fabsf(delta.c) >= eps)) {
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
            .a = start_rtcp.a + s_out * delta.a,
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
             rtcp_ac_disable();
             kinematics.transform_steps_to_cartesian = (transform_steps_to_cartesian_ptr)calc_pos_in_cartesian;
             kinematics.segment_line = (segment_line_ptr)kinematics_passthru;
             sync_position();
             break;

         case 851:
             kinematics.transform_steps_to_cartesian = (transform_steps_to_cartesian_ptr)calc_pos_in_rtcp;
             kinematics.segment_line = (segment_line_ptr)kinematics_segment_line;
             calc_pos_in_rtcp(&start_rtcp, (mpos_t *)sys.position);
             rtcp_ac_enable();
             break;

         case 852:
             rtcp_ac_measure();
             break;

         default:
            handled = false;
            break;
    }

    if(!handled && user_mcode.execute)
        user_mcode.execute(state, gc_block);
}

PROGMEM static const setting_group_detail_t rtcp_groups [] = {
    { Group_Root, Group_Kinematics, "Kinematik Versatzwerte" } //Here
};

PROGMEM static const setting_detail_t rtcp_settings[] = {
    { Setting_Kinematics0, Group_Kinematics, "RTCP A offset X", "mm", Format_Decimal, "#####0.000", "-1000", "1000", Setting_IsExtended, &rtp_ac_settings.a_vector.x },
    { Setting_Kinematics1, Group_Kinematics, "RTCP A offset Y", "mm", Format_Decimal, "#####0.000", "-1000", "1000", Setting_IsExtended, &rtp_ac_settings.a_vector.y },
    { Setting_Kinematics2, Group_Kinematics, "RTCP A offset Z", "mm", Format_Decimal, "#####0.000", "-1000", "1000", Setting_IsExtended, &rtp_ac_settings.a_vector.z },
    { Setting_Kinematics3, Group_Kinematics, "RTCP C offset X", "mm", Format_Decimal, "#####0.000", "-1000", "1000", Setting_IsExtended, &rtp_ac_settings.c_vector.x },
    { Setting_Kinematics4, Group_Kinematics, "RTCP C offset Y", "mm", Format_Decimal, "#####0.000", "-1000", "1000", Setting_IsExtended, &rtp_ac_settings.c_vector.y },
    { Setting_Kinematics5, Group_Kinematics, "RTCP C offset Z", "mm", Format_Decimal, "#####0.000", "-1000", "1000", Setting_IsExtended, &rtp_ac_settings.c_vector.z },
    { Setting_Kinematics6, Group_Kinematics, "RTCP segment length", "mm", Format_Decimal, "#####0.000", "0.010", "1.000", Setting_IsExtended, &rtp_ac_settings.segment_length },
/*
    { Setting_Kinematics7, Group_Kinematics, "RTCP reserved", "mm", Format_Decimal, "#####0.000", "-1000", "1000", Setting_IsExtended, &rtp_ac_settings.reserved[0] },
    { Setting_Kinematics8, Group_Kinematics, "RTCP reserved", "mm", Format_Decimal, "#####0.000", "-1000", "1000", Setting_IsExtended, &rtp_ac_settings.reserved[1] },
    { Setting_Kinematics9, Group_Kinematics, "RTCP reserved", "mm", Format_Decimal, "#####0.000", "-1000", "1000", Setting_IsExtended, &rtp_ac_settings.reserved[2] }
*/
};

/*
PROGMEM static const setting_descr_t rtcp_settings_descr[] = {

};
*/

FLASHMEM static void rtcp_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&rtp_ac_settings, sizeof(rtp_ac_settings_t), true);
}

FLASHMEM static void rtcp_settings_restore (void)
{
    static const rtp_ac_settings_t defaults = {     // offset values of the rotary centers and the segmentlength are saved here, can be changed via $640-$649
        .a_vector.x = 113.660f,                     // vector from MCS zero to the Center of A-Rotary (y and z value) x is the value where C-Rotary sits on top of A.
        .a_vector.y = 158.700f,
        .a_vector.z = -93.540f,
        .c_vector.x = 0.0f,                         // vector from A-rotary-center to C-rotary center (my A and C intersect, thats why x,y are 0)
        .c_vector.y = 0.0f,
        .c_vector.z = -93.0f,
        .segment_length = 0.500f                    // rough segment length for the curve
    };

    memcpy(&rtp_ac_settings, &defaults, sizeof(rtp_ac_settings_t));

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&rtp_ac_settings, sizeof(rtp_ac_settings_t), true);
}

FLASHMEM static void rtcp_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&rtp_ac_settings, nvs_address, sizeof(rtp_ac_settings_t), true) != NVS_TransferResult_OK)
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

// Initialize API pointers for RTCP_AC kinematics
FLASHMEM void rtcp_ac_init (void)
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

    if((nvs_address = nvs_alloc(sizeof(rtp_ac_settings_t)))) {

        kinematics.transform_from_cartesian = (transform_from_cartesian_ptr)rtcp_forward_cartesian; // called from homing routine - RTCP should be turned off during homing?
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

#endif // RTCP_AC
