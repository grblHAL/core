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
    static plan_line_data_t *cc_mc_active_plan_data = 0;
    static float cc_mc_input_pos[N_AXIS] = {0};

#ifndef CC_DEBUG_TRACE
#define CC_DEBUG_TRACE 0  // Set to 0 to disable debug tracing
#endif

    // Sync cc_mc_input_pos with the current parser position.
    // Must be called when enabling comp, and can be called any time cc_mc_input_pos
    // may be stale (e.g. after rapids that bypass cc_mc_line_in).
    static inline void cc_mc_sync_input_pos(const float *pos)
    {
        for (int i = 0; i < N_AXIS; ++i)
            cc_mc_input_pos[i] = pos[i];
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
        case cc_status_ArcRadiusInconsistant:
            msg = "Arc radius inconsistant";
            break;
        case cc_status_FlippedArc:
            msg = "Flipped arc";
            break;
        case cc_status_CompInCrossing:
            msg = "Crossing detected on move into compensation";
            break;
        case cc_status_CompOutCrossing:
            msg = "Crossing detected on move out of compensation";
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
        case cc_status_GlobalSelfIntersection:
            msg = "Global self intersection detected in compensation moves";
            break;
        }   

        if (lineNum != 0)
        {
            snprintf(formatted_msg, sizeof(formatted_msg), "%s at line %lu", msg, (unsigned long)lineNum);
            report_message(formatted_msg, (message_type_t)severity);
            return;
        }

        report_message(msg, (message_type_t)severity);
    }


    static inline uint8_t cc_mc_comp_mode_from_input(gc_ccomp_t cc)
    {
        if (cc.side == CComp_Left || cc.side == CComp_Right)
            return (uint8_t)(cc.first_move ? CC_CM_IN : CC_CM_STEADY);

        if (cc_api_get_comp() != CC_COMP_OFF)
            return (uint8_t)CC_CM_OUT;

        return (uint8_t)CC_CM_NONE;
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
        mv.compMode = cc_mc_comp_mode_from_input(cc);
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
        cc_mc_active_plan_data = pl_data;

        if (cc.side == CComp_Off && cc_api_get_comp() == CC_COMP_OFF)
        {
            cc_mc_input_pos[0] = xyz[0];
            cc_mc_input_pos[1] = xyz[1];
            cc_mc_input_pos[2] = xyz[2];
            mc_line(xyz, pl_data);
            return cc_status_OK;
        }
#if CC_DEBUG_TRACE
        {
            char dbg[128];
            snprintf(dbg, sizeof(dbg), "CC_IN side=%d first=%d inp=(%.3f,%.3f,%.3f) tgt=(%.3f,%.3f,%.3f)",
                     cc.side, cc.first_move, cc_mc_input_pos[0], cc_mc_input_pos[1], cc_mc_input_pos[2],
                     xyz[0], xyz[1], xyz[2]);
            report_message(dbg, Message_Info);
        }
#endif
        comp_side side = CC_COMP_OFF;
        bool turning_off = false;
        bool turningOn = false;
        // convert from grbl-style comp mode to cc style.
        if (cc.side == CComp_Left)
        {
            side = CC_COMP_LEFT;
            if (cc_api_get_comp() == CC_COMP_OFF)
                turningOn = true;
        }
        else if (cc.side == CComp_Right)
        {
            side = CC_COMP_RIGHT;
            if (cc_api_get_comp() == CC_COMP_OFF)
                turningOn = true;
        }
        else if (cc_api_get_comp() != CC_COMP_OFF)
            turning_off = true;

        if (side != CC_COMP_OFF)
            cc_api_set_comp(side);// i should only do this if not already in comp, but just in case.

        move2d mv = cc_mc_to_move2d(cc, xyz, pl_data, 0, 0, 0.0f, 0, false);

        if (turningOn)
        {
            bool inch = cc_api_get_units() == CC_UNITS_INCH;
            float r = inch ? cc.radius / 25.4f : cc.radius;
            char msg[64];
            snprintf(msg, sizeof(msg), "CC_On R=%.4f %s", r, inch ? "in" : "mm");
            report_message(msg, Message_Info);
        }

        if (turning_off)
            cc_api_set_comp(CC_COMP_OFF);

        cc_status_code_t st = cc_api_process_move(&mv);
        if (st != cc_status_OK)
            return st;

        if (cc.side == CComp_Off)
        {
            st = cc_api_process_move(0);
            if (st != cc_status_OK)
                return st;

            cc_api_set_comp(CC_COMP_OFF);
            report_message("CC_Off", Message_Info);
        }

        return cc_status_OK;
    }

    // replaces mc_arc when cutter compensation is active. If compensation is not active, passes through to mc_arc.
    cc_status_code_t cc_mc_arc_in(gc_ccomp_t cc, float *xyz, plan_line_data_t *pl_data, float *position, float *ijk, float radius, plane_t plane, int32_t turns)
    {
        cc_mc_active_plan_data = pl_data;

        if (cc.side == CComp_Off && cc_api_get_comp() == CC_COMP_OFF)
        {
            cc_mc_input_pos[0] = xyz[0];
            cc_mc_input_pos[1] = xyz[1];
            cc_mc_input_pos[2] = xyz[2];
            mc_arc(xyz, pl_data, position, ijk, radius, plane, turns);
            return cc_status_OK;
        }

        comp_side side = CC_COMP_OFF;
        bool turning_off = false;
        if (cc.side == CComp_Left)
            side = CC_COMP_LEFT;
        else if (cc.side == CComp_Right)
            side = CC_COMP_RIGHT;
        else if (cc_api_get_comp() != CC_COMP_OFF)
            turning_off = true;
        if (side != CC_COMP_OFF)
            cc_api_set_comp(side);

        move2d mv = cc_mc_to_move2d(cc, xyz, pl_data, position, ijk, radius, turns, true);

        if (turning_off)
            cc_api_set_comp(CC_COMP_OFF);

        cc_status_code_t st = cc_api_process_move(&mv);
        if (st != cc_status_OK)
            return st;

        if (cc.side == CComp_Off) // this should never happen for arcs, but just in case
        {
            st = cc_api_process_move(0);
            if (st != cc_status_OK)
                return st;
            cc_api_set_comp(CC_COMP_OFF);
        }

        return cc_status_OK;
    }

    static inline void cc_emit_via_mc(const move2d *mv)
    {
        plan_line_data_t local_pl_data = {0};
        plan_line_data_t *pl_data = &local_pl_data;

        if (!mv || !mv->valid || mv->suppressOutput)
            return;

        if (cc_mc_active_plan_data)
            local_pl_data = *cc_mc_active_plan_data;

        local_pl_data.feed_rate = mv->feed;
        local_pl_data.condition.rapid_motion = (mv->type == CC_MOT_RAPID) ? 1 : 0;


#if CC_DEBUG_TRACE
        {
            char dbg[128];
            snprintf(dbg, sizeof(dbg), "CC_EMIT t=%d cm=%d p0=(%.3f,%.3f) p1=(%.3f,%.3f) z0=%.3f z1=%.3f",
                     mv->type, mv->compMode, mv->p_0.x, mv->p_0.y, mv->p_1.x, mv->p_1.y, mv->z_0, mv->z_1);
            report_message(dbg, Message_Info);
        }
#endif

        if (mv->type == CC_MOT_LINE || mv->type == CC_MOT_RAPID)
        {
            float xyz[N_AXIS] = {0};
            xyz[0] = mv->p_1.x;
            xyz[1] = mv->p_1.y;
            xyz[2] = mv->z_1;
            mc_line(xyz, pl_data);
        }
        else if (mv->type == CC_MOT_ARC)
        {
            float xyz[N_AXIS] = {0};
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
    }
#endif
#ifdef __cplusplus
}
#endif
#endif