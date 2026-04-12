/*
 * SHIM BETWEEN GRBLHAL AND THE CUTTER COMPENSATION CORE
 * Overall, this file serves as a bridge between grblHAL and the cutter compensation core, enabling them to work together seamlessly while keeping their internal implementations decoupled.
  
 * code is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef CUTTER_COMP_GRBLHAL_H
#define CUTTER_COMP_GRBLHAL_H
#include "config.h"

#ifdef __cplusplus
extern "C"
{
#endif

#if CUTTER_COMP_ENABLE

#include <stdio.h>

#include "cutter_comp.h"

    // forward declarations
    // these are implemented in grblhal's motion_control.c, but declared here so they can be called from the cc_emit_via_mc callback.
    bool mc_line(float *xyz, plan_line_data_t *pl_data);
    void mc_arc(float *xyz, plan_line_data_t *pl_data, float *position, float *ijk, float radius, plane_t plane, int32_t turns);
    void report_message(const char *msg, message_type_t type);
    void debug_printf(const char *fmt, ...);
    static plan_line_data_t cc_mc_active_plan_data = {0};
    static bool cc_mc_have_plan_data = false;
    static bool cc_mc_active = false;
    static bool cc_mc_should_pause_once = false;
    static float cc_mc_input_pos[N_AXIS] = {0};

    // in cutter_comp_grblhal.h
    static inline void cc_report_version(void)
    {
        report_message("Cutter compensation v" CUTTER_COMP_VERSION, Message_Info);
    }


    // Sync cc_mc_input_pos with the current parser position.
    // Must be called when enabling comp, and can be called any time cc_mc_input_pos
    // may be stale (e.g. after rapids that bypass cc_mc_line_in).
    static inline void cc_mc_sync_input_pos(const float *pos)
    {
        for (int i = 0; i < N_AXIS; ++i)
            cc_mc_input_pos[i] = pos[i];
    }

    static bool cc_mc_is_active(bool pause_on_active)
    {
        if(pause_on_active)
            cc_mc_should_pause_once = cc_mc_active;
        return cc_mc_active;
    }

    static void cc_message(cc_status_code_t msgcode, msg_type_t severity, uint32_t lineNum)
    {
        const char *msg = "Unknown";
        char formatted_msg[128];
        switch (msgcode)
        {
        case cc_status_OK:
            return;
        case cc_status_ArcLtToolRad:
            msg = "Arc radius less than tool radius";
            break;
        case cc_status_InvalidMove:
            msg = "Invalid move";
            break;
        case cc_status_MoveTooShort:
            msg = "Move too short to compensate";
            break;
        case cc_status_ArcRadiusInconsistent:
            msg = "Arc radius inconsistent";
            break;
        case cc_status_UnresolvedGap:
            msg = "Unresolved gap between moves";
            break;
        case cc_status_InputBufferOverflow:
            msg = "Cutter compensation input buffer overflow";
            break;
        case cc_status_OutputBufferOverflow:
            msg = "Cutter compensation output buffer overflow";
            break;
        }   

        if (lineNum != 0)
        {
            snprintf(formatted_msg, sizeof(formatted_msg), "CC:%s at line %lu", msg, (unsigned long)lineNum);
            report_message(formatted_msg, (message_type_t)severity);
            return;
        }

        report_message(msg, (message_type_t)severity);
    }


    // Creates a move2d struct from the given grblHAL cutter compensation data.
    static inline move2d cc_mc_to_move2d(gc_ccomp_t cc,
                                         float *xyz,
                                         plan_line_data_t *pl_data,
                                         float *position,
                                         float *ijk,
                                         float radius,
                                         int32_t turns,
                                         bool is_arc)
    {
        move2d mv = {0};
        mv.p_0 = cc_v2(cc_mc_input_pos[0], cc_mc_input_pos[1]);
        mv.p_1 = cc_v2(xyz[0], xyz[1]);
        mv.z_0 = cc_mc_input_pos[2];
        mv.z_1 = xyz[2];
        mv.feed = pl_data ? pl_data->feed_rate : 0.0f;
        mv.lineNum = pl_data ? pl_data->line_number : 0;
        mv.compMode = (uint8_t)cc_api_get_mode();
        mv.valid = true;

        if (is_arc)
        {
            mv.type = CC_MOT_ARC;
            if (position)
            {
                mv.p_0 = cc_v2(position[0], position[1]);
                mv.z_0 = position[2];
            }
            if (ijk)
            {
                mv.center = cc_v2(mv.p_0.x + ijk[0], mv.p_0.y + ijk[1]);
            }
            mv.radius = radius;
            mv.arcDir = (uint8_t)((turns >= 0) ? CC_ARC_CCW : CC_ARC_CW);
        }
        else
        {
            mv.type = (uint8_t)((pl_data && pl_data->condition.rapid_motion) ? CC_MOT_RAPID : CC_MOT_LINE);
        }

        cc_mc_input_pos[0] = xyz[0];
        cc_mc_input_pos[1] = xyz[1];
        cc_mc_input_pos[2] = xyz[2];
        return mv;
    }

    // replaces mc_line when cutter compensation is active. If compensation is not active, passes through to mc_line.
    cc_status_code_t cc_mc_line_in(gc_ccomp_t cc, float *xyz, plan_line_data_t *pl_data)
    {
        if((cc_mc_have_plan_data = pl_data != NULL))
            cc_mc_active_plan_data = *pl_data;

        if (cc.side == CComp_Off && cc_api_get_comp() == CC_COMP_OFF)
        {
            cc_mc_active = false;
            cc_mc_input_pos[0] = xyz[0];
            cc_mc_input_pos[1] = xyz[1];
            cc_mc_input_pos[2] = xyz[2];
            mc_line(xyz, pl_data);
            return cc_status_OK;
        }
        cc_mc_active = true;    

        comp_side side = cc.side == CComp_Left ? CC_COMP_LEFT : (cc.side == CComp_Right ? CC_COMP_RIGHT : CC_COMP_OFF);
        comp_side current_side = cc_api_get_comp();
        bool turning_off = current_side != CC_COMP_OFF && side == CC_COMP_OFF;
 
        if (side != current_side || (side != CC_COMP_OFF && cc_api_get_mode() == CC_CM_NONE))
            cc_api_set_comp(side);

        move2d mv = cc_mc_to_move2d(cc, xyz, pl_data, 0, 0, 0.0f, 0, false);

        if (side != CC_COMP_OFF && mv.compMode == CC_CM_IN)
        {
            bool inch = cc_api_get_units() == CC_UNITS_INCH;
            float r = inch ? cc.radius / 25.4f : cc.radius;
            const char *corner_mode = cc_api_get_corner_treatment_mode() == CC_CTM_CHAMFER ? "Chamfer" : "Roll";
            char msg[96];
            snprintf(msg, sizeof(msg), "CC_On R=%.4f %s Corner=%s", r, inch ? "in" : "mm", corner_mode);
            report_message(msg, Message_Info);
        }

        cc_status_code_t st = cc_api_process_move(&mv);
        if (st != cc_status_OK)
            return st;


        if (turning_off)
        {
            st = cc_api_process_move(0);
            if (st != cc_status_OK)
                return st;
            report_message("CC_Off", Message_Info);
        }


        return cc_status_OK;
    }

    // replaces mc_arc when cutter compensation is active. If compensation is not active, passes through to mc_arc.
    cc_status_code_t cc_mc_arc_in(gc_ccomp_t cc, float *xyz, plan_line_data_t *pl_data, float *position, float *ijk, float radius, plane_t plane, int32_t turns)
    {
        if((cc_mc_have_plan_data = pl_data != NULL))
            cc_mc_active_plan_data = *pl_data;

        if (cc.side == CComp_Off && cc_api_get_comp() == CC_COMP_OFF)
        {
            cc_mc_active = false;
            cc_mc_input_pos[0] = xyz[0];
            cc_mc_input_pos[1] = xyz[1];
            cc_mc_input_pos[2] = xyz[2];
            mc_arc(xyz, pl_data, position, ijk, radius, plane, turns);
            return cc_status_OK;
        }
        cc_mc_active = true;
        comp_side side = cc.side == CComp_Left ? CC_COMP_LEFT : (cc.side == CComp_Right ? CC_COMP_RIGHT : CC_COMP_OFF);
        comp_side current_side = cc_api_get_comp();
        bool turning_off = current_side != CC_COMP_OFF && side == CC_COMP_OFF;
        bool turned_off = false;


        if (side != current_side || (side != CC_COMP_OFF && cc_api_get_mode() == CC_CM_NONE))
            cc_api_set_comp(side);

        move2d mv = cc_mc_to_move2d(cc, xyz, pl_data, position, ijk, radius, turns, true);
        cc_status_code_t st = cc_api_process_move(&mv);

        if (st != cc_status_OK)
            return st;

        if (turning_off)
        {
            st = cc_api_process_move(0);
            if (st != cc_status_OK)
                return st;
            turned_off = cc_api_get_comp() == CC_COMP_OFF;
        }
        return cc_status_OK;
    }

    static inline void cc_emit_via_mc(const move2d *mv)
    {
        plan_line_data_t local_pl_data = {0};
        plan_line_data_t *pl_data = &local_pl_data;
        float xyz[N_AXIS] = {0};

        if (!mv || !mv->valid)
            return;

        if (cc_mc_have_plan_data)
            local_pl_data = cc_mc_active_plan_data;

        local_pl_data.feed_rate = mv->feed;
        local_pl_data.condition.rapid_motion = (mv->type == CC_MOT_RAPID) ? 1 : 0;

        if (mv->type == CC_MOT_LINE || mv->type == CC_MOT_RAPID)
        {
            xyz[0] = mv->p_1.x;
            xyz[1] = mv->p_1.y;
            xyz[2] = mv->z_1;
            mc_line(xyz, pl_data);
        }
        else if (mv->type == CC_MOT_ARC)
        {
            xyz[0] = mv->p_1.x;
            xyz[1] = mv->p_1.y;
            xyz[2] = mv->z_1;

            float position[N_AXIS] = {0};
            position[0] = mv->p_0.x;
            position[1] = mv->p_0.y;
            position[2] = mv->z_0;

            float ijk[3] = {0};
            ijk[0] = mv->center.x - mv->p_0.x;
            ijk[1] = mv->center.y - mv->p_0.y;
            ijk[2] = 0.0f;

            plane_t plane = {0};
            plane.axis_0 = 0;
            plane.axis_1 = 1;
            plane.axis_linear = 2;

            int32_t turns = (mv->arcDir == CC_ARC_CCW) ? 1 : -1;
            mc_arc(xyz, pl_data, position, ijk, mv->radius, plane, turns);
        }

        if(cc_mc_should_pause_once || sys.flags.single_block) {
            cc_mc_should_pause_once = false;
            protocol_buffer_synchronize();
            system_set_exec_state_flag(EXEC_FEED_HOLD);
            protocol_execute_realtime();
        }

    }
#endif
#ifdef __cplusplus
}
#endif
#endif