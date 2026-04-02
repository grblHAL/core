

/*
 * cutter_comp.h
 * Jason Titcomb 2026
 * code is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CUTTER_COMP_H
#define CUTTER_COMP_H
#include "config.h"
#include <stdint.h>

#if CUTTER_COMP_ENABLE

#ifdef __cplusplus
extern "C" {
#endif

#define CC_IN_CAP 2

/*
 * Corner style selection:
 * CC_ENABLE_CORNER_TREATMENT 0 -> roll path (CC_CORNER_TREATMENT_MODE is ignored).
 * CC_ENABLE_CORNER_TREATMENT 1 and CC_CORNER_TREATMENT_MODE 0 -> roll.
 * CC_ENABLE_CORNER_TREATMENT 1 and CC_CORNER_TREATMENT_MODE != 0 -> chamfer-style treatment.
 */
#ifndef CC_ENABLE_CORNER_TREATMENT
#define CC_ENABLE_CORNER_TREATMENT 1
#endif

#ifndef CC_CORNER_TREATMENT_MODE
#define CC_CORNER_TREATMENT_MODE 0
#endif

#ifndef CC_INSERT_CAP
#if CC_ENABLE_CORNER_TREATMENT
#define CC_INSERT_CAP 3
#else
#define CC_INSERT_CAP 1
#endif
#endif

#ifndef CC_ENABLE_LOOKAHEAD
#define CC_ENABLE_LOOKAHEAD 1
#endif

#ifndef CC_LOOKAHEAD_CAP
#define CC_LOOKAHEAD_CAP 8
#endif

#ifndef CC_LOOKAHEAD_STEPS
#define CC_LOOKAHEAD_STEPS 4
#endif

#ifndef CC_LA_TARGET_BATCH_EMIT
#define CC_LA_TARGET_BATCH_EMIT 3
#endif

#ifndef CC_LA_TRIM_OVERLAP
#define CC_LA_TRIM_OVERLAP (CC_LOOKAHEAD_STEPS + 2)
#endif

#ifndef CC_LA_EMIT_HOLDBACK
#define CC_LA_EMIT_HOLDBACK CC_LA_TRIM_OVERLAP
#endif

#ifndef CC_LA_MIN_PENDING
#define CC_LA_MIN_PENDING (CC_LA_EMIT_HOLDBACK + CC_LA_TARGET_BATCH_EMIT)
#endif

#ifndef CC_OUT_CAP
#if CC_ENABLE_LOOKAHEAD
#define CC_OUT_CAP (CC_LOOKAHEAD_CAP + 1)
#else
#define CC_OUT_CAP (1 + CC_INSERT_CAP)
#endif
#endif


typedef struct
{
    float x;
    float y;
} vec2;


typedef enum {
    CC_UNITS_MM = 0,
    CC_UNITS_INCH = 1
} cc_units;

typedef enum
{
    CC_CM_NONE = 0,
    CC_CM_IN = 1,
    CC_CM_STEADY = 2,
    CC_CM_OUT = 3
} comp_mode;

typedef enum
{
    CC_MOT_EMPTY = 0,
    CC_MOT_RAPID = 1,
    CC_MOT_LINE = 2,
    CC_MOT_ARC = 3
} motion_type;

typedef enum
{
    CC_ARC_CW = 0,
    CC_ARC_CCW = 1
} arc_dir;

typedef enum
{
    CC_IT_NONE = 0,
    CC_IT_TANGENT = 1,
    CC_IT_INTERSECT = 2
} intersect_type;

typedef enum
{
    CC_COMP_OFF = 0,//G40
    CC_COMP_LEFT = 1,//G41
    CC_COMP_RIGHT = -1//G42
} comp_side;

typedef enum
{
    CC_CTM_ROLL = 0,
    CC_CTM_CHAMFER = 1
} cc_corner_treatment_mode;

typedef enum
{
    cc_status_OK = 0,
    cc_status_ArcRadiusInconsistant = 101,
    cc_status_InvalidMove = 102,
    cc_status_MoveTooShort = 103,
    cc_status_ArcLtToolRad = 104,
    cc_status_FlippedArc = 105,
    cc_status_CompInCrossing = 106,
    cc_status_CompOutCrossing = 107,
    cc_status_UnresolvedGap = 108,
    cc_status_InputBufferOverflow = 109,
    cc_status_OutputBufferOverflow = 110,
    cc_status_GlobalSelfIntersection = 111
} cc_status_code_t;

typedef enum {
    CC_MSG_PLAIN = 0,
    CC_MSG_INFO,
    CC_MSG_WARNING,
    CC_MSG_ERROR,
    CC_MSG_DEBUG
} msg_type_t;

typedef struct
{
    vec2 p_0;
    vec2 p_1;
    vec2 center;
    vec2 startDir;
    vec2 endDir;
    float radius;
    float feed;
    float z_0;
    float z_1;
    uint32_t lineNum;
    uint8_t type;
    uint8_t arcDir;
    uint8_t compMode;
    bool valid;
    bool suppressOutput;
} move2d;

vec2 cc_v2(float x, float y);

typedef void (*cc_msg_cb)( cc_status_code_t msg, msg_type_t severity, uint32_t lineNum);
typedef void (*emit_move_cb)(const move2d *move);

typedef enum
{
    CC_JT_NONE = 0,
    CC_JT_TRIM_TO_INTERSECTION,
    CC_JT_EXTEND_TO_INTERSECTION,
    CC_JT_ROLL_AROUND,
 } junction_type;

typedef struct
{
    junction_type jtype;
    vec2 p;
} junction;

typedef struct
{
    float a0;
    float a1;
    uint8_t dir;
} arc_angles;

typedef struct
{
    cc_status_code_t status;

    float toolR;
    int8_t toolSign;
    comp_side compSide;
    comp_mode compMode;
    uint8_t cornerTreatmentMode;
    uint32_t lastLineNum;
    cc_units units;
    float arcTol;
    float gapTol;
    float minOutputLen;

    int inHead;
    int inCount;
    int outHead;
    int outCount;
    bool stopErr;
    bool havePrevMove;

    move2d prevOff;
    move2d input_buffer[CC_IN_CAP];
    move2d output_buffer[CC_OUT_CAP];
#if CC_ENABLE_LOOKAHEAD
    move2d lookahead_buffer[CC_LOOKAHEAD_CAP];
    int lookahead_count;
#endif
} cc_context;
cc_units cc_api_get_units(void);

void cc_api_init(float radius, cc_units units, emit_move_cb emitCb, cc_msg_cb errCb);

// Process a move. If move is null, flushes any pending moves and reports any pending errors.
// Returns CC_OK if the move was processed and emitted successfully, or if flushing completed successfully.
// Returns an appropriate error code otherwise.
cc_status_code_t cc_api_process_move(const move2d *move);

// comp_side is CC_COMP_OFF=0, CC_COMP_LEFT=1, or CC_COMP_RIGHT=-1
void cc_api_set_comp(comp_side side);
comp_side cc_api_get_comp(void);

//Requires CC_ENABLE_CORNER_TREATMENT set to 1
void cc_api_set_corner_treatment_mode(cc_corner_treatment_mode mode);


#ifdef __cplusplus
}
#endif

#endif // CUTTER_COMP_H
#endif // CUTTER_COMP_ENABLE