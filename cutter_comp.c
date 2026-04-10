/*
 * cutter_comp.c
 * Jason Titcomb 2026
 *
 * Cutter compensation engine intended for grblHAL-style
 * integration.
 * 
 * code is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with grblHAL. If not, see <http://www.gnu.org/licenses/>.
 */

#include "config.h"

#ifndef CUTTER_COMP_ENABLE
#define CUTTER_COMP_ENABLE 0
#endif
#if CUTTER_COMP_ENABLE

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "cutter_comp.h"

#define CC_TOL 0.0001f
#define CC_ARC_TOL_MM 0.01f
#define CC_GAP_TOL_MM 0.01f
#define CC_ARC_TOL_IN 0.0004f
#define CC_GAP_TOL_IN 0.0004f
#define CC_MIN_OUTPUT_LEN_MM 0.001f
#define CC_MIN_OUTPUT_LEN_IN 0.00004f

#define CC_EPS 1e-7f
#define CC_PARALLEL_TOL 1e-3f
#define CC_BEVEL_VEC_TOL 1.0e-1f
#define CC_PI 3.14159265358979323846f
#define CC_TWO_PI 6.2831853071795864769f
#define CC_MAX_SWEEP_DEG 359.9f
#define CC_MIN_ARC_LEN 0.001f

static cc_context g_core_ctx;
static emit_move_cb g_core_emit_cb = (emit_move_cb)0;
static cc_msg_cb g_core_msg_cb = (cc_msg_cb)0;

cc_units cc_api_get_units(void)
{
    return g_core_ctx.units;
}

void cc_api_set_corner_treatment_mode(cc_corner_treatment_mode mode)
{
    g_core_ctx.cornerTreatmentMode = (uint8_t)mode;
}

static inline comp_side cc_effective_comp_side(const cc_context *ctx)
{
    if (ctx->toolSign >= 0)
        return ctx->compSide;

    if (ctx->compSide == CC_COMP_LEFT)
        return CC_COMP_RIGHT;

    if (ctx->compSide == CC_COMP_RIGHT)
        return CC_COMP_LEFT;

    return ctx->compSide;
}

static inline bool cc_comp_uses_left(const cc_context *ctx)
{
    return cc_effective_comp_side(ctx) == CC_COMP_LEFT;
}

static inline float cc_clamp(float x, float lo, float hi)
{
    return (x < lo) ? lo : ((x > hi) ? hi : x);
}

vec2 cc_v2(float x, float y)
{
    vec2 v;
    v.x = x;
    v.y = y;
    return v;
}

static inline vec2 cc_add(vec2 a, vec2 b)
{
    return cc_v2(a.x + b.x, a.y + b.y);
}

static inline vec2 cc_sub(vec2 a, vec2 b)
{
    return cc_v2(a.x - b.x, a.y - b.y);
}

static inline vec2 cc_scale(vec2 v, float s)
{
    return cc_v2(v.x * s, v.y * s);
}

static inline float cc_dot(vec2 a, vec2 b)
{
    return a.x * b.x + a.y * b.y;
}

static inline float cc_cross(vec2 a, vec2 b)
{
    return a.x * b.y - a.y * b.x;
}

static inline float cc_len(vec2 v)
{
    return sqrtf(cc_dot(v, v));
}

static inline bool cc_is_equalf(const float a, const float b)
{
    return fabsf(a - b) <= CC_EPS;
}

static inline bool cc_is_equalv(vec2 a, vec2 b)
{
    return cc_is_equalf(a.x, b.x) && cc_is_equalf(a.y, b.y);
}

static inline float cc_dist(vec2 a, vec2 b)
{
    return cc_len(cc_sub(a, b));
}

static inline vec2 cc_normalize(vec2 v)
{
    float l = cc_len(v);
    if (l < CC_TOL)
        return cc_v2(0.0f, 0.0f);
    return cc_v2(v.x / l, v.y / l);
}

static inline vec2 cc_left_normal(vec2 v)
{
    return cc_v2(-v.y, v.x);
}

static inline vec2 cc_right_normal(vec2 v)
{
    return cc_v2(v.y, -v.x);
}

static inline float cc_angle_norm(float a)
{
    a = fmodf(a, CC_TWO_PI);
    if (a < 0.0f)
        a += CC_TWO_PI;
    return a;
}

static inline float cc_sweep_ccw(float a0, float a1)
{
    float d;
    a0 = cc_angle_norm(a0);
    a1 = cc_angle_norm(a1);
    d = a1 - a0;
    if (d < 0.0f)
        d += CC_TWO_PI;
    return d;
}

static inline float cc_sweep_cw(float a0, float a1)
{
    return cc_sweep_ccw(a1, a0);
}

static inline bool cc_angle_on_sweep_ccw(float a0, float a1, float ap)
{
    a0 = cc_angle_norm(a0);
    a1 = cc_angle_norm(a1);
    ap = cc_angle_norm(ap);
    if (a0 <= a1)
        return (ap + CC_EPS >= a0) && (ap <= a1 + CC_EPS);
    return (ap >= a0 - CC_EPS) || (ap <= a1 + CC_EPS);
}

static inline bool cc_angle_on_sweep_cw(float a0, float a1, float ap)
{
    return cc_angle_on_sweep_ccw(a1, a0, ap);
}

static inline int cc_get_winding_dir(vec2 a, vec2 b)
{
    float z = cc_cross(a, b);
    if (z > CC_TOL)
        return 1;
    if (z < -CC_TOL)
        return -1;
    return 0;
}

static inline bool cc_is_near(vec2 a, vec2 b)
{
    vec2 d = cc_sub(a, b);
    return cc_dot(d, d) <= CC_TOL * CC_TOL;
}

static inline bool cc_is_line_like(const move2d *m)
{
    return m->type == CC_MOT_LINE || m->type == CC_MOT_RAPID;
}

static inline bool cc_has_rapid_move(const move2d *a, const move2d *b)
{
    return a->type == CC_MOT_RAPID || b->type == CC_MOT_RAPID;
}

static inline int cc_z_move_direction(float z0, float z1)
{
    if (cc_is_equalf(z0, z1))
        return 0;
    return (z1 > z0) ? 1 : -1;
}

static inline bool cc_should_replace_pending_z_target(const move2d *pending, const move2d *candidate)
{
    int pendingDir = cc_z_move_direction(pending->z_0, pending->z_1);
    int candidateDir = cc_z_move_direction(candidate->z_0, candidate->z_1);

    if (pendingDir != 0 && pendingDir == candidateDir)
        return fabsf(candidate->z_1) > fabsf(pending->z_1);

    return true;
}

static inline void cc_update_vectors(move2d *m)
{
    m->hasXY = !cc_is_equalv(m->p_1, m->p_0);
    m->hasZ = !cc_is_equalf(m->z_1, m->z_0);
    if (!m->hasXY)
    {
        m->startDir = cc_v2(0.0f, 0.0f);
        m->endDir = cc_v2(0.0f, 0.0f);
        return;
    }

    if (cc_is_line_like(m))
    {
        vec2 d = cc_sub(m->p_1, m->p_0);
        if (cc_dot(d, d) < CC_TOL * CC_TOL)
            return;

        vec2 u = cc_normalize(d);
        m->startDir = u;
        m->endDir = u;
        return;
    }

    if (m->type == CC_MOT_ARC)
    {
        vec2 rs = cc_normalize(cc_sub(m->p_0, m->center));
        vec2 re = cc_normalize(cc_sub(m->p_1, m->center));
        if (m->arcDir == CC_ARC_CCW)
        {
            m->startDir = cc_left_normal(rs);
            m->endDir = cc_left_normal(re);
        }
        else
        {
            m->startDir = cc_right_normal(rs);
            m->endDir = cc_right_normal(re);
        }
        return;
    }

    m->startDir = cc_v2(0.0f, 0.0f);
    m->endDir = cc_v2(0.0f, 0.0f);
}

static inline vec2 cc_roll_center(vec2 p_offset, vec2 dir, bool useLeft, float toolR)
{
    vec2 normal = useLeft ? cc_left_normal(dir) : cc_right_normal(dir);
    return cc_sub(p_offset, cc_scale(normal, toolR));
}

static inline bool cc_is_radius_consistent(const move2d *m)
{
    float r0 = cc_len(cc_sub(m->p_0, m->center));
    float r1 = cc_len(cc_sub(m->p_1, m->center));
    float tol = g_core_ctx.arcTol > 0 ? g_core_ctx.arcTol : CC_ARC_TOL_MM;
    return fabsf(r0 - r1) <= tol;
}

static inline float cc_wrap2pi(float a)
{
    a = fmodf(a, CC_TWO_PI);
    if (a < 0.0f)
        a += CC_TWO_PI;
    return a;
}

static inline float cc_arc_sweep_deg(const move2d *m)
{
    vec2 r0;
    vec2 r1;
    float sw;

    r0 = (m->arcDir == CC_ARC_CCW) ? cc_right_normal(m->startDir) : cc_left_normal(m->startDir);
    r1 = (m->arcDir == CC_ARC_CCW) ? cc_right_normal(m->endDir) : cc_left_normal(m->endDir);

    if (m->arcDir == CC_ARC_CCW)
    {
        sw = atan2f(cc_cross(r0, r1), cc_dot(r0, r1));
    }
    else
    {
        sw = atan2f(cc_cross(r1, r0), cc_dot(r1, r0));
    }

    if (sw < 0.0f)
        sw += CC_TWO_PI;

    return sw * (180.0f / CC_PI);
}

static inline float cc_line_t(const move2d *m, vec2 p)
{
    vec2 d = cc_sub(m->p_1, m->p_0);
    float l2 = cc_dot(d, d);
    if (l2 < 1e-12f)
        return 0.0f;
    return cc_dot(cc_sub(p, m->p_0), d) / l2;
}

static inline float cc_dist_from_start_along(const move2d *m, vec2 p)
{
    if (m->type == CC_MOT_LINE || m->type == CC_MOT_RAPID)
    {
        float t = cc_line_t(m, p);
        t = cc_clamp(t, 0.0f, 1.0f);
        return cc_len(cc_sub(m->p_1, m->p_0)) * t;
    }

    if (m->type == CC_MOT_ARC)
    {
        float a0 = atan2f(m->p_0.y - m->center.y, m->p_0.x - m->center.x);
        float ap = atan2f(p.y - m->center.y, p.x - m->center.x);
        float sw = (m->arcDir == CC_ARC_CCW) ? cc_sweep_ccw(a0, ap) : cc_sweep_cw(a0, ap);
        return fabsf(m->radius) * sw;
    }

    return 0.0f;
}

static inline bool cc_point_on_segment(vec2 a, vec2 b, vec2 p)
{
    vec2 ab = cc_sub(b, a);
    float lab2 = cc_dot(ab, ab);
    float t;
    float cross;

    if (lab2 < CC_TOL)
    {
        vec2 pa = cc_sub(p, a);
        return cc_dot(pa, pa) < CC_TOL * CC_TOL;
    }

    t = cc_dot(cc_sub(p, a), ab) / lab2;
    if (t < -CC_TOL || t > 1.0f + CC_TOL)
        return false;

    cross = cc_cross(cc_sub(p, a), ab);
    return cross * cross < CC_TOL * CC_TOL * lab2;
}

static inline arc_angles cc_precompute_arc_angles(const move2d *m)
{
    arc_angles aa;
    aa.a0 = atan2f(m->p_0.y - m->center.y, m->p_0.x - m->center.x);
    aa.a1 = atan2f(m->p_1.y - m->center.y, m->p_1.x - m->center.x);
    aa.dir = m->arcDir;
    return aa;
}

static inline bool cc_point_on_arc_cached(const move2d *a, vec2 p, const arc_angles *aa)
{
    float rp = cc_len(cc_sub(p, a->center));
    float ap;

    if (fabsf(rp - fabsf(a->radius)) > CC_TOL)
        return false;

    ap = atan2f(p.y - a->center.y, p.x - a->center.x);
    if (aa->dir == CC_ARC_CCW)
        return cc_angle_on_sweep_ccw(aa->a0, aa->a1, ap);
    return cc_angle_on_sweep_cw(aa->a0, aa->a1, ap);
}

static inline intersect_type cc_intersect_line_line(const move2d *ln1, const move2d *ln2, vec2 *ip, bool *tip)
{
    vec2 p = ln1->p_0;
    vec2 q = ln2->p_0;
    float lr = cc_dist(ln1->p_0, ln1->p_1);
    float ls = cc_dist(ln2->p_0, ln2->p_1);
    vec2 r = ln1->startDir;
    vec2 s = ln2->startDir;
    float den;
    float denTol;
    float t;
    float u;

    *ip = cc_v2(0.0f, 0.0f);
    if (lr < CC_TOL || ls < CC_TOL || cc_len(r) < CC_TOL || cc_len(s) < CC_TOL)
    {
        *tip = false;
    }

    den = cc_cross(r, s);
    denTol = CC_PARALLEL_TOL * cc_len(r) * cc_len(s);
    if (fabsf(den) <= denTol)
    {
        *tip = false;
        return CC_IT_NONE;
    }

    t = cc_cross(cc_sub(q, p), s) / den;
    u = cc_cross(cc_sub(q, p), r) / den;
    *ip = cc_add(p, cc_scale(r, t));
    *tip = (t >= -CC_TOL && t <= lr + CC_TOL && u >= -CC_TOL && u <= ls + CC_TOL);
    return CC_IT_INTERSECT;
}

static inline intersect_type cc_intersect_circle_circle(const move2d *a1, const move2d *a2, vec2 *p1, vec2 *p2, int *count)
{
    vec2 c0 = a1->center;
    vec2 c1 = a2->center;
    float r0 = fabsf(a1->radius);
    float r1 = fabsf(a2->radius);
    vec2 d = cc_sub(c1, c0);
    float distc = cc_len(d);
    float a;
    float h2;
    vec2 u;
    vec2 mid;
    float h;
    vec2 perp;

    *count = 0;

    if (distc < CC_TOL)
        return CC_IT_NONE;
    if (distc > r0 + r1 + CC_TOL)
        return CC_IT_NONE;
    if (distc < fabsf(r0 - r1) - CC_TOL)
        return CC_IT_NONE;

    a = (r0 * r0 - r1 * r1 + distc * distc) / (2.0f * distc);
    h2 = r0 * r0 - a * a;
    u = cc_scale(d, 1.0f / distc);
    mid = cc_add(c0, cc_scale(u, a));

    if (fabsf(h2) < CC_TOL)
    {
        *p1 = mid;
        *count = 1;
        return CC_IT_TANGENT;
    }

    h = sqrtf(fmaxf(0.0f, h2));
    perp = cc_left_normal(u);
    *p1 = cc_add(mid, cc_scale(perp, h));
    *p2 = cc_sub(mid, cc_scale(perp, h));
    *count = 2;
    return CC_IT_INTERSECT;
}

static inline intersect_type cc_intersect_line_circle(vec2 l1, vec2 l2, vec2 ctr, float r, vec2 *p1, vec2 *p2, int *count)
{
    vec2 d = cc_sub(l2, l1);
    float dd = cc_dot(d, d);
    vec2 f;
    float t0;
    vec2 q;
    vec2 qc;
    float dist2;
    float r2;
    float h2;
    float h;
    float invLen;
    vec2 u;
    const float eps = 1e-5f;
    const float eps2 = eps * eps;

    *count = 0;
    if (dd < 1e-20f)
        return CC_IT_NONE;

    f = cc_sub(l1, ctr);
    t0 = -cc_dot(f, d) / dd;
    q = cc_add(l1, cc_scale(d, t0));
    qc = cc_sub(q, ctr);
    dist2 = cc_dot(qc, qc);
    r2 = r * r;
    h2 = r2 - dist2;

    if (h2 < -eps2)
        return CC_IT_NONE;

    if (fabsf(h2) <= eps2)
    {
        *p1 = q;
        *count = 1;
        return CC_IT_TANGENT;
    }

    h = sqrtf(h2);
    invLen = 1.0f / sqrtf(dd);
    u = cc_scale(d, invLen);
    *p1 = cc_sub(q, cc_scale(u, h));
    *p2 = cc_add(q, cc_scale(u, h));
    *count = 2;
    return CC_IT_INTERSECT;
}

static inline void cc_report_msg(cc_context *ctx, cc_status_code_t msg,msg_type_t severity)
{
    ctx->stopErr = (severity == CC_MSG_ERROR);
    ctx->status = msg;

    if (!g_core_msg_cb)
        return;
    g_core_msg_cb(msg, severity, ctx->lastLineNum);
}

static inline bool cc_validate(cc_context *ctx, move2d *m)
{
    if (cc_is_line_like(m))
    {
        if (cc_len(cc_sub(m->p_1, m->p_0)) < CC_TOL)
        {
            m->valid = false;//VALIDATE FAIL
            return false;//VALIDATE FAIL
        }

        m->valid = true;
        return true;
    }

    if (m->type != CC_MOT_ARC)
        return m->valid;

    {
        bool degenerate = fabsf(m->radius) < CC_TOL;
        float sw = cc_arc_sweep_deg(m);
        bool sweep_ok = sw <= CC_MAX_SWEEP_DEG && sw >= CC_MIN_ARC_LEN;

        if (degenerate || !sweep_ok)
        {
            m->valid = false;
            cc_report_msg(ctx, cc_status_ArcLtToolRad, CC_MSG_ERROR);
            return m->valid;
        }

        {
            comp_side side = cc_effective_comp_side(ctx);
            bool inner_arc = (side == CC_COMP_LEFT && m->arcDir == CC_ARC_CCW) ||
                             (side == CC_COMP_RIGHT && m->arcDir == CC_ARC_CW);
            if (inner_arc && fabsf(m->radius) < ctx->toolR)
            {
                m->valid = false;
                cc_report_msg(ctx, cc_status_ArcLtToolRad, CC_MSG_ERROR);
                return m->valid;
            }
        }

        bool consistent = cc_is_radius_consistent(m);
        if (!consistent)
            cc_report_msg(ctx, cc_status_ArcRadiusInconsistent, CC_MSG_ERROR);
        if (!sweep_ok)
            cc_report_msg(ctx, cc_status_InvalidMove, CC_MSG_ERROR);

        return m->valid;
    }
}

static inline bool cc_motion_valid(const move2d *m)
{
    return m->valid && m->type != CC_MOT_EMPTY && m->hasXY;
}

static inline bool cc_point_on_finite_elem(const move2d *m, vec2 p)
{
    if (cc_is_line_like(m))
        return cc_point_on_segment(m->p_0, m->p_1, p);

    if (m->type == CC_MOT_ARC)
    {
        arc_angles aa = cc_precompute_arc_angles(m);
        return cc_point_on_arc_cached(m, p, &aa);
    }

    return false;
}

static inline int cc_intersect_carrier(const move2d *a, const move2d *b, vec2 pts[2])
{
    if (cc_is_line_like(a) && cc_is_line_like(b))
    {
        bool tip = false;
        intersect_type it = cc_intersect_line_line(a, b, &pts[0], &tip);
        return (it == CC_IT_NONE) ? 0 : 1;
    }

    if (a->type == CC_MOT_ARC && b->type == CC_MOT_ARC)
    {
        int count = 0;
        intersect_type it;
        if (cc_is_near(a->center, b->center))
            return 0;
        it = cc_intersect_circle_circle(a, b, &pts[0], &pts[1], &count);
        if (it == CC_IT_NONE)
            return 0;
        return count;
    }

    {
        const move2d *line = cc_is_line_like(a) ? a : b;
        const move2d *arc = (a->type == CC_MOT_ARC ? a : b);
        int count = 0;
        intersect_type it = cc_intersect_line_circle(line->p_0, line->p_1, arc->center, fabsf(arc->radius), &pts[0], &pts[1], &count);
        if (it == CC_IT_NONE)
            return 0;
        return count;
    }
}

static inline int cc_finite_intersection_points(const move2d *a, const move2d *b, vec2 pts[2])
{
    vec2 carrierPts[2];
    int carrierCount = cc_intersect_carrier(a, b, carrierPts);
    int finiteCount = 0;
    int i;

    for (i = 0; i < carrierCount; ++i)
    {
        vec2 p = carrierPts[i];
        if (!cc_point_on_finite_elem(a, p) || !cc_point_on_finite_elem(b, p))
            continue;
        if (finiteCount > 0 && cc_is_near(pts[0], p))
            continue;
        pts[finiteCount++] = p;
    }

    return finiteCount;
}

static inline bool cc_is_forward_extension_point(const move2d *a, const move2d *b, vec2 p)
{
    float fipDir1 = cc_dot(cc_sub(p, a->p_1), a->endDir);
    float fipDir2 = cc_dot(cc_sub(p, b->p_0), b->startDir);
    return fipDir1 > 0.0f && fipDir2 < 0.0f;
}

static inline bool cc_convex_from_winding(const cc_context *ctx, int winding)
{
    bool isLeft;
    if (winding == 0)
        return false;
    isLeft = cc_comp_uses_left(ctx);
    if (isLeft)
        return !(winding > 0);
    return winding > 0;
}

static inline bool cc_is_convex(const cc_context *ctx, const move2d *a, const move2d *b)
{
    int winding = cc_get_winding_dir(a->endDir, b->startDir);
    return cc_convex_from_winding(ctx, winding);
}

static inline bool cc_solve_junction(const cc_context *ctx, const move2d *a, const move2d *b, bool allowExtend, junction *outjunc)
{
    vec2 trimPts[2];
    vec2 carrierPts[2];
    int carrierCount = cc_intersect_carrier(a, b, carrierPts);
    int trimCount = 0;
    float bestTrimScore = 0.0f;
    float bestExtendScore = 0.0f;
    bool foundTrim = false;
    bool foundExtend = false;
    int i;

    outjunc->jtype = CC_JT_NONE;
    outjunc->p = cc_v2(0.0f, 0.0f);

    for (i = 0; i < carrierCount; ++i)
    {
        vec2 p = carrierPts[i];
        if (!cc_point_on_finite_elem(a, p) || !cc_point_on_finite_elem(b, p))
            continue;
        if (trimCount > 0 && cc_is_near(trimPts[0], p))
            continue;
        trimPts[trimCount++] = p;
    }

    for (i = 0; i < trimCount; ++i)
    {
        vec2 p = trimPts[i];
        float score = cc_dist_from_start_along(a, p) + cc_dist_from_start_along(b, p);
        if (!foundTrim || score < bestTrimScore)
        {
            outjunc->jtype = CC_JT_TRIM_TO_INTERSECTION;
            outjunc->p = p;
            bestTrimScore = score;
            foundTrim = true;
        }
    }

    for (i = 0; i < carrierCount; ++i)
    {
        vec2 p = carrierPts[i];
        if (allowExtend && cc_is_forward_extension_point(a, b, p))
        {
            float score = cc_dist(a->p_1, p) + cc_dist(b->p_0, p);
            if (!foundExtend || score < bestExtendScore)
            {
                outjunc->jtype = CC_JT_EXTEND_TO_INTERSECTION;
                outjunc->p = p;
                bestExtendScore = score;
                foundExtend = true;
            }
        }
    }

    if (foundTrim)
        return true;
    if (foundExtend)
        return true;
    if (cc_is_convex(ctx, a, b))
    {
        outjunc->jtype = CC_JT_ROLL_AROUND;
        return true;
    }

    outjunc->jtype = CC_JT_NONE;
    return false;
}

static inline bool cc_out_has_space(cc_context *ctx, int n)
{
    bool ok = (ctx->outCount + n) <= CC_OUT_CAP;
    if (!ok)
        cc_report_msg(ctx, cc_status_OutputBufferOverflow, CC_MSG_ERROR);
    return ok;
}

static inline move2d cc_pop_in(cc_context *ctx)
{
    move2d m = ctx->input_buffer[ctx->inHead];
    ctx->inHead = (ctx->inHead + 1) % CC_IN_CAP;
    ctx->inCount--;
    return m;
}

static inline void cc_push_out(cc_context *ctx, const move2d *m)
{
    if (!cc_out_has_space(ctx, 1))
        return;

    ctx->output_buffer[(ctx->outHead + ctx->outCount) % CC_OUT_CAP] = *m;
    ctx->outCount++;
}

static inline move2d cc_make_bevel(const move2d *a, const move2d *b)
{
    move2d m = {0};
    m.valid = true;
    m.type = CC_MOT_LINE;
    m.feed = (a->feed > 0.0f) ? a->feed : b->feed;
    m.compMode = CC_CM_STEADY;
    m.p_0 = a->p_1;
    m.p_1 = b->p_0;
    m.z_0 = b->z_0;
    m.z_1 = b->z_0;
    cc_update_vectors(&m);
    return m;
}

static inline bool cc_offset_line(const cc_context *ctx, move2d *m)
{
    vec2 v;
    float l;
    vec2 u;
    bool useLeft;
    vec2 n;
    vec2 off;

    v = cc_sub(m->p_1, m->p_0);
    l = cc_len(v);
    if (l < CC_TOL)
    {
        m->valid = false;
        return false;
    }

    u = cc_scale(v, 1.0f / l);
    useLeft = cc_comp_uses_left(ctx);
    n = useLeft ? cc_left_normal(u) : cc_right_normal(u);
    off = cc_scale(n, ctx->toolR);

    if (m->compMode == CC_CM_IN || m->compMode == CC_CM_OUT)
        off = cc_v2(0.0f, 0.0f);

    m->p_0 = cc_add(m->p_0, off);
    m->p_1 = cc_add(m->p_1, off);
    return true;
}

static inline bool cc_offset_arc(cc_context *ctx, move2d *m)
{
    float r0 = m->radius;
    float dr;
    bool ccw;
    bool left;
    float r1;
    vec2 v0;
    vec2 v1;
    float lv0;
    float lv1;

    if (fabs(r0) < CC_TOL)
        r0 = cc_len(cc_sub(m->p_0, m->center));
    if (fabs(r0) < CC_TOL)
        return false;

    dr = ctx->toolR;
    ccw = (m->arcDir == CC_ARC_CCW);
    left = cc_comp_uses_left(ctx);

    if (ccw)
        r1 = r0 + (left ? -dr : dr);
    else
        r1 = r0 + (left ? dr : -dr);

    if (r1 <= CC_TOL)
    {
        cc_report_msg(ctx, cc_status_ArcLtToolRad, CC_MSG_ERROR);
        m->valid = false;
        return false;
    }

    v0 = cc_sub(m->p_0, m->center);
    v1 = cc_sub(m->p_1, m->center);
    lv0 = cc_len(v0);
    lv1 = cc_len(v1);
    if (lv0 < CC_TOL || lv1 < CC_TOL)
    {
        m->valid = false;
        return false;
    }

    m->center = m->center;
    m->radius = r1;
    m->p_0 = cc_add(m->center, cc_scale(v0, r1 / lv0));
    m->p_1 = cc_add(m->center, cc_scale(v1, r1 / lv1));
    return true;
}

static inline bool cc_offset_move(cc_context *ctx, move2d *dst)
{
    if (dst->type == CC_MOT_LINE || dst->type == CC_MOT_RAPID)
        return cc_offset_line(ctx, dst);
    if (dst->type == CC_MOT_ARC)
        return cc_offset_arc(ctx, dst);
    return false;
}

static inline bool cc_trim_to(cc_context *ctx, move2d *a, move2d *b, vec2 tip)
{
    a->p_1 = tip;
    b->p_0 = tip;
    cc_update_vectors(a);
    cc_update_vectors(b);
    cc_validate(ctx, a);
    cc_validate(ctx, b);
    return a->valid && b->valid;
}

static inline bool cc_stage_out(cc_context *ctx, const move2d *m)
{
    if (!cc_out_has_space(ctx, 1))
        return false;
    cc_push_out(ctx, m);
    return true;
}

static inline bool cc_stage_flush(cc_context *ctx)
{
    (void)ctx;
    return true;
}

static inline bool cc_extend_to(cc_context *ctx, move2d *a, move2d *b, vec2 fip)
{
    float fipDir1 = cc_dot(cc_sub(fip, a->p_1), a->endDir);
    float fipDir2 = cc_dot(cc_sub(fip, b->p_0), b->startDir);

    if (fipDir1 > 0.0f && fipDir2 < 0.0f)
    {
        a->p_1 = fip;
        b->p_0 = fip;
        cc_update_vectors(a);
        cc_update_vectors(b);
        cc_validate(ctx, a);
        cc_validate(ctx, b);
    }

    return a->valid && b->valid;
}

static inline move2d cc_make_roll_arc(const cc_context *ctx, const move2d *a, const move2d *b)
{
    move2d roll = {0};
    bool useLeft = cc_comp_uses_left(ctx);
    vec2 v0;
    vec2 v1;
    float r0;
    float r1;
    float r;
    arc_dir preferredDir;

    roll.type = CC_MOT_ARC;
    roll.compMode = CC_CM_STEADY;
    roll.feed = (a->feed > 0.0f) ? a->feed : b->feed;
    roll.p_0 = a->p_1;
    roll.p_1 = b->p_0;
    roll.z_0 = b->z_0;
    roll.z_1 = b->z_0;    
    roll.center = cc_roll_center(a->p_1, a->endDir, useLeft, ctx->toolR);

    v0 = cc_sub(roll.p_0, roll.center);
    v1 = cc_sub(roll.p_1, roll.center);
    r0 = cc_len(v0);
    r1 = cc_len(v1);
    r = ctx->toolR;

    if (r0 >= CC_TOL && r1 >= CC_TOL)
        r = 0.5f * (r0 + r1);
    else if (r0 >= CC_TOL)
        r = r0;
    else if (r1 >= CC_TOL)
        r = r1;
        
    if (r0 >= CC_TOL)
        roll.p_0 = cc_add(roll.center, cc_scale(v0, r / r0));
    if (r1 >= CC_TOL)
        roll.p_1 = cc_add(roll.center, cc_scale(v1, r / r1));

    roll.radius = r;
    preferredDir = useLeft ? CC_ARC_CW : CC_ARC_CCW;
    roll.arcDir = (uint8_t)preferredDir;

    roll.valid = true;
    cc_update_vectors(&roll);
    return roll;
}

#if CC_ENABLE_CORNER_TREATMENT
static inline move2d cc_make_arc_extension_line_only(const cc_context *ctx, const move2d *arc, bool fromEnd)
{
    move2d ext = {0};
    vec2 anchor;
    vec2 dir;
    float extent;

    if (arc->type != CC_MOT_ARC)
        return ext;

    extent = ctx->toolR * 2.0f;
    anchor = fromEnd ? arc->p_1 : arc->p_0;
    dir = fromEnd ? arc->endDir : arc->startDir;

    if (cc_len(dir) < CC_TOL)
        return ext;

    ext.type = CC_MOT_LINE;
    ext.compMode = arc->compMode;
    ext.feed = arc->feed;
    ext.z_0 = fromEnd ? arc->z_1 : arc->z_0;
    ext.z_1 = ext.z_0;    
    ext.p_0 = anchor;
    ext.p_1 = cc_add(anchor, cc_scale(dir, extent));
    ext.startDir = dir;
    ext.endDir = dir;
    cc_update_vectors(&ext);
    cc_validate((cc_context *)ctx, &ext);

    return ext;
}

static inline int cc_make_corner_treatment(cc_context *ctx, move2d *a, move2d *b, move2d outmove[3])
{
    int outCountLocal = 0;
    move2d l1;
    move2d l2;
    move2d extA = {0};
    move2d extB = {0};
    bool haveExtA = false;
    bool haveExtB = false;
    bool aLineLike;
    bool bLineLike;
    float turnSign0;
    vec2 partCorner;
    vec2 vIn;
    vec2 vOut;
    vec2 bisector;
    vec2 chamferDir;
    vec2 offsetCap;
    move2d cap = {0};
    vec2 ipForL1;
    vec2 ipForL2;
    bool tip;
    intersect_type it;

    aLineLike = (a->type == CC_MOT_LINE || a->type == CC_MOT_RAPID);
    bLineLike = (b->type == CC_MOT_LINE || b->type == CC_MOT_RAPID);

    turnSign0 = cc_cross(cc_scale(a->endDir, -1.0f), b->startDir);
    if (fabsf(turnSign0) <= CC_BEVEL_VEC_TOL)
    {
        move2d bevel = cc_make_bevel(a, b);
        outmove[outCountLocal++] = bevel;
        return outCountLocal;
    }

    if (aLineLike)
    {
        l1 = *a;
    }
    else
    {
        extA = cc_make_arc_extension_line_only(ctx, a, true);
        if (!extA.valid)
            return 0;
        l1 = extA;
        haveExtA = true;
    }

    if (bLineLike)
    {
        l2 = *b;
    }
    else
    {
        extB = cc_make_arc_extension_line_only(ctx, b, false);
        if (!extB.valid)
            return 0;
        l2 = extB;
        haveExtB = true;
    }

    partCorner = cc_roll_center(a->p_1, a->endDir, cc_comp_uses_left(ctx), ctx->toolR);
    vIn = cc_normalize(cc_scale(l1.endDir, -1.0f));
    vOut = cc_normalize(l2.startDir);
    bisector = cc_normalize(cc_add(vIn, vOut));
    if (cc_len(bisector) < CC_TOL)
        return 0;

    chamferDir = cc_normalize(cc_left_normal(bisector));
    if (cc_len(chamferDir) < CC_TOL)
        return 0;

    offsetCap = cc_add(partCorner, cc_scale(bisector, -ctx->toolR));
    cap.type = CC_MOT_LINE;
    cap.compMode = CC_CM_STEADY;
    cap.feed = (a->feed > 0.0f) ? a->feed : b->feed;
    cap.z_0 = b->z_0;
    cap.z_1 = b->z_0;
    {
        float halfLen = 0.5f * (ctx->toolR + 2.0f);
        cap.p_0 = cc_sub(offsetCap, cc_scale(chamferDir, halfLen));
        cap.p_1 = cc_add(offsetCap, cc_scale(chamferDir, halfLen));
    }
    cc_update_vectors(&cap);

    ipForL1 = cc_v2(0.0f, 0.0f);
    ipForL2 = cc_v2(0.0f, 0.0f);
    tip = false;
    it = cc_intersect_line_line(&l1, &cap, &ipForL1, &tip);
    if (it == CC_IT_NONE)
        return 0;

    it = cc_intersect_line_line(&l2, &cap, &ipForL2, &tip);
    if (it == CC_IT_NONE)
        return 0;

    cap.p_0 = ipForL1;
    cap.p_1 = ipForL2;
    if (!cc_validate(ctx, &cap))
        return 0;

    if (aLineLike)
    {
        a->p_1 = ipForL1;
        cc_update_vectors(a);
        if (!cc_validate(ctx, a))
            return 0;
    }

    if (bLineLike)
    {
        b->p_0 = ipForL2;
        cc_update_vectors(b);
        if (!cc_validate(ctx, b))
            return 0;
    }

    if (haveExtA)
    {
        extA.p_1 = ipForL1;
        extA.z_0 = b->z_0;
        extA.z_1 = b->z_0;
        cc_update_vectors(&extA);
        if (!cc_validate(ctx, &extA))
            return 0;
        outmove[outCountLocal++] = extA;
    }

    outmove[outCountLocal++] = cap;

    if (haveExtB)
    {
        extB.p_0 = ipForL2;
        extB.p_1 = b->p_0;
        extB.z_0 = b->z_0;
        extB.z_1 = b->z_0;
        cc_update_vectors(&extB);
        if (!cc_validate(ctx, &extB))
            return 0;
        outmove[outCountLocal++] = extB;
    }

    return outCountLocal;
}
#endif

static inline bool cc_insert_roll_or_corner(cc_context *ctx, move2d *a, move2d *b, move2d inserts[CC_INSERT_CAP], int *insertCount)
{
    float gap = cc_len(cc_sub(b->p_0, a->p_1));
    int startCount = *insertCount;

#if !CC_ENABLE_CORNER_TREATMENT
    (void)startCount;
#endif

    float gapTol = ctx->gapTol > 0 ? ctx->gapTol : CC_GAP_TOL_MM;
    if (gap < gapTol)
    {
        return true;
    }

#if CC_ENABLE_CORNER_TREATMENT
    if ((cc_corner_treatment_mode)ctx->cornerTreatmentMode == CC_CTM_CHAMFER)
    {
        move2d cornerSegs[3];
        int cornerCount = cc_make_corner_treatment(ctx, a, b, cornerSegs);
        int i;

        for (i = 0; i < cornerCount && *insertCount < CC_INSERT_CAP; ++i)
            inserts[(*insertCount)++] = cornerSegs[i];

        return *insertCount > startCount;
    }
#endif

    if (*insertCount >= CC_INSERT_CAP)
        return false;

    move2d roll = cc_make_roll_arc(ctx, a, b);
    if (!cc_validate(ctx, &roll))
        return false;

    inserts[(*insertCount)++] = roll;
    return true;
}

static inline void cc_handle_line_line(cc_context *ctx, move2d *a, move2d *b, move2d inserts[CC_INSERT_CAP], int *insertCount)
{
    junction junction;
    float gap = cc_dist(b->p_0, a->p_1);
    float gapTol = ctx->gapTol > 0 ? ctx->gapTol : CC_GAP_TOL_MM;
    bool anyRapid = cc_has_rapid_move(a, b);
    bool allowExtend = anyRapid || (gap < gapTol);
    bool resolved = cc_solve_junction(ctx, a, b, allowExtend, &junction);

    if (junction.jtype == CC_JT_TRIM_TO_INTERSECTION)
    {
        cc_trim_to(ctx, a, b, junction.p);
        if (!a->valid)
        {
            b->p_0 = a->p_1;
            return;
        }
        if (!b->valid)
        {
            a->p_1 = b->p_0;
            return;
        }
        return;
    }

    if (junction.jtype == CC_JT_EXTEND_TO_INTERSECTION && cc_extend_to(ctx, a, b, junction.p))
        return;

    if (a->compMode == CC_CM_IN)
    {
        a->p_1 = b->p_0;
        cc_update_vectors(a);
        cc_validate(ctx, a);
        return;
    }

    if (b->compMode == CC_CM_OUT)
    {
        b->p_0 = a->p_1;
        cc_update_vectors(b);
        cc_validate(ctx, b);
        return;
    }

    if (!anyRapid && resolved && junction.jtype == CC_JT_ROLL_AROUND)
    {
        if (!cc_insert_roll_or_corner(ctx, a, b, inserts, insertCount))
            cc_report_msg(ctx, cc_status_UnresolvedGap, CC_MSG_ERROR);
        return;
    }

    inserts[(*insertCount)++] = cc_make_bevel(a, b);
}

static inline void cc_handle_arc_arc(cc_context *ctx, move2d *a, move2d *b, move2d inserts[CC_INSERT_CAP], int *insertCount)
{
    junction junction;
    float gap;
    bool resolved;

    if (cc_is_near(a->p_1, b->p_0) || cc_is_near(a->center, b->center))
        return;

    gap = cc_len(cc_sub(b->p_0, a->p_1));
    float gapTol = ctx->gapTol > 0 ? ctx->gapTol : CC_GAP_TOL_MM;
    resolved = cc_solve_junction(ctx, a, b, gap < gapTol, &junction);
    if(!resolved)
    {
        if (gap < gapTol)
        {
            a->p_1 = b->p_0;
            cc_update_vectors(a);
            cc_validate(ctx, a);
            return;
        }
    }

    if (junction.jtype == CC_JT_TRIM_TO_INTERSECTION && cc_trim_to(ctx, a, b, junction.p))
        return;

    if (junction.jtype == CC_JT_EXTEND_TO_INTERSECTION && cc_extend_to(ctx, a, b, junction.p))
        return;

    if (cc_insert_roll_or_corner(ctx, a, b, inserts, insertCount))
    {
        return;
    }

    cc_report_msg(ctx, cc_status_InvalidMove, CC_MSG_ERROR);

    inserts[(*insertCount)++] = cc_make_bevel(a, b);
}

static inline void cc_handle_arc_line(cc_context *ctx, move2d *a, move2d *b, move2d inserts[CC_INSERT_CAP], int *insertCount)
{
    junction junction;
    float gap;
    bool resolved;

    if (cc_is_near(a->p_1, b->p_0))
    {
        b->p_0 = a->p_1;
        cc_update_vectors(b);
        cc_validate(ctx, b);
        return;
    }

    gap = cc_len(cc_sub(b->p_0, a->p_1));
    float gapTol = ctx->gapTol > 0 ? ctx->gapTol : CC_GAP_TOL_MM;
    bool anyRapid = cc_has_rapid_move(a, b);
    resolved = cc_solve_junction(ctx, a, b, anyRapid || (gap < gapTol), &junction);

    if (junction.jtype == CC_JT_TRIM_TO_INTERSECTION && cc_trim_to(ctx, a, b, junction.p))
        return;

    if (junction.jtype == CC_JT_EXTEND_TO_INTERSECTION && cc_extend_to(ctx, a, b, junction.p))
        return;

    if (!anyRapid && resolved && junction.jtype == CC_JT_ROLL_AROUND)
    {
        if (!cc_insert_roll_or_corner(ctx, a, b, inserts, insertCount))
            cc_report_msg(ctx, cc_status_UnresolvedGap,true);
        return;
    }

    cc_report_msg(ctx, cc_status_InvalidMove, CC_MSG_ERROR);

    inserts[(*insertCount)++] = cc_make_bevel(a, b);
}

static inline void cc_apply_logic(cc_context *ctx, move2d *a, move2d *b, move2d inserts[CC_INSERT_CAP], int *insertCount)
{
    *insertCount = 0;

    if (cc_is_line_like(a) && cc_is_line_like(b))
    {
        cc_handle_line_line(ctx, a, b, inserts, insertCount);
        return;
    }

    if (a->type == CC_MOT_ARC && b->type == CC_MOT_ARC)
    {
        cc_handle_arc_arc(ctx, a, b, inserts, insertCount);
        return;
    }

    if ((a->type == CC_MOT_ARC && (b->type == CC_MOT_LINE || b->type == CC_MOT_RAPID)) ||
        ((a->type == CC_MOT_LINE || a->type == CC_MOT_RAPID) && b->type == CC_MOT_ARC))
    {
        cc_handle_arc_line(ctx, a, b, inserts, insertCount);
    }
}

static inline void cc_reset_state(cc_context *ctx)
{
    ctx->havePrevMove = false;
    ctx->havePendingZMove = false;
}

static inline bool cc_emit_pending_z_move_at(cc_context *ctx, const move2d *anchor)
{
    move2d zMove;

    if (!ctx->havePendingZMove)
        return true;

    zMove = ctx->pendingZMove;
    zMove.p_0 = anchor->p_1;
    zMove.p_1 = anchor->p_1;
    zMove.z_0 = anchor->z_1;
    zMove.hasXY = false;
    zMove.hasZ = !cc_is_equalf(zMove.z_1, zMove.z_0);

    if (!zMove.hasZ)
    {
        ctx->havePendingZMove = false;
        return true;
    }

    if (!cc_stage_out(ctx, &zMove))
        return false;

    ctx->havePendingZMove = false;
    return true;
}

static void cc_init_internal(cc_context *ctx, float toolRadius)
{
    cc_context zero = {0};
    *ctx = zero;
    ctx->status = cc_status_OK;
    ctx->toolSign = 1;
    ctx->compSide = CC_COMP_OFF;
    ctx->cornerTreatmentMode = (uint8_t)CC_CORNER_TREATMENT_MODE;
    ctx->toolR = (toolRadius < 0.0f) ? -toolRadius : toolRadius;
    ctx->toolSign = (toolRadius < 0.0f) ? -1 : 1;
    cc_reset_state(ctx);
}

void cc_set_comp(cc_context *ctx, comp_side side)
{
    comp_side prevSide = ctx->compSide;
    ctx->compSide = side;

    if (prevSide == CC_COMP_OFF && side != CC_COMP_OFF)
    {
        ctx->compMode = CC_CM_IN;
        return;
    }

    if (prevSide != CC_COMP_OFF && side == CC_COMP_OFF)
    {
        ctx->compMode = CC_CM_OUT;
        return;
    }

    if (side == CC_COMP_OFF)
    {
        ctx->compMode = CC_CM_NONE;
        return;
    }

    if (prevSide != side)
    {
        ctx->compMode = CC_CM_IN;
        return;
    }

    if (ctx->compMode == CC_CM_NONE || ctx->compMode == CC_CM_OUT)
    {
        ctx->compMode = CC_CM_IN;
        return;
    }

    if (ctx->compMode != CC_CM_IN)
        ctx->compMode = CC_CM_STEADY;
}

bool cc_push_in(cc_context *ctx, const move2d *m)
{
    if (ctx->inCount >= CC_IN_CAP)
        return false;
    ctx->input_buffer[(ctx->inHead + ctx->inCount) % CC_IN_CAP] = *m;
    ctx->inCount++;
    return true;
}

bool cc_process(cc_context *ctx)
{
    if (ctx->stopErr)
        return false;

     while (ctx->inCount > 0)
    {
        move2d curOff;
        move2d inserts[CC_INSERT_CAP];
        int insertCount = 0;

        if (!cc_out_has_space(ctx, 2 + CC_INSERT_CAP))
            return false;

        curOff = cc_pop_in(ctx);
        ctx->lastLineNum = curOff.lineNum;
        
        if (curOff.type == CC_MOT_EMPTY)
            continue;

        cc_update_vectors(&curOff);
        curOff.compMode = ctx->compMode;

        // Z-only move: no XY displacement, nothing to offset
        if (!curOff.hasXY && curOff.hasZ)
        {
            if (ctx->havePrevMove)
            {
                if (ctx->havePendingZMove)
                {
                    if (cc_should_replace_pending_z_target(&ctx->pendingZMove, &curOff))
                    {
                        ctx->pendingZMove.z_1 = curOff.z_1;
                        ctx->pendingZMove.lineNum = curOff.lineNum;
                        ctx->pendingZMove.feed = curOff.feed;
                        ctx->pendingZMove.type = curOff.type;
                        ctx->pendingZMove.hasZ = !cc_is_equalf(ctx->pendingZMove.z_1, ctx->pendingZMove.z_0);
                    }
                }
                else
                {
                    ctx->pendingZMove = curOff;
                    ctx->pendingZMove.p_0 = ctx->prevOff.p_1;
                    ctx->pendingZMove.p_1 = ctx->prevOff.p_1;
                    ctx->pendingZMove.z_0 = ctx->prevOff.z_1;
                    ctx->pendingZMove.hasXY = false;
                    ctx->pendingZMove.hasZ = !cc_is_equalf(ctx->pendingZMove.z_1, ctx->pendingZMove.z_0);
                    ctx->havePendingZMove = ctx->pendingZMove.hasZ;
                }
            }
            else
            {
                if (!cc_stage_out(ctx, &curOff))
                    return false;
            }
            continue;
        }

        if (!cc_validate(ctx, &curOff))
            return false;
        if (!cc_offset_move(ctx, &curOff))
            return false;


        if (!ctx->havePrevMove)
        {
            ctx->prevOff = curOff;
            ctx->havePrevMove = true;
            if (curOff.compMode == CC_CM_IN)
                ctx->compMode = CC_CM_STEADY;
            else if (curOff.compMode == CC_CM_OUT)
                ctx->compMode = CC_CM_NONE;
            continue;
        }

         if (ctx->prevOff.compMode == CC_CM_IN)
        {
            float originalLen = cc_len(cc_sub(ctx->prevOff.p_1, ctx->prevOff.p_0));
            if (originalLen <= ctx->toolR + CC_TOL)
            {
                cc_report_msg(ctx, cc_status_MoveTooShort, CC_MSG_ERROR);
                return false;
            }

            /* Modify the previous move so that the end is the start of the current move. */
            ctx->prevOff.p_1 = curOff.p_0;

            {
                float finalLen = cc_len(cc_sub(ctx->prevOff.p_1, ctx->prevOff.p_0));
                if (finalLen <= CC_TOL)
                {
                    cc_report_msg(ctx, cc_status_MoveTooShort, CC_MSG_ERROR);
                    return false;
                }
            }

            cc_update_vectors(&ctx->prevOff);
        }

        if (curOff.compMode == CC_CM_OUT)
        {
            float originalLen = cc_len(cc_sub(curOff.p_1, curOff.p_0));
            if (originalLen <= ctx->toolR + CC_TOL)
            {
                cc_report_msg(ctx, cc_status_MoveTooShort, CC_MSG_ERROR);
                return false;
            }

            /* Modify the G40 move so that the start is the end of the previous move. */
            curOff.p_0 = ctx->prevOff.p_1;

            {
                float finalLen = cc_len(cc_sub(curOff.p_1, curOff.p_0));
                if (finalLen <= CC_TOL)
                {
                    cc_report_msg(ctx, cc_status_MoveTooShort, CC_MSG_ERROR);
                    return false;
                }
            }

            cc_update_vectors(&curOff);
        }

 
        if (curOff.compMode == CC_CM_STEADY)
            cc_apply_logic(ctx, &ctx->prevOff, &curOff, inserts, &insertCount);

        cc_validate(ctx, &curOff);

        if (ctx->prevOff.valid)
        {
            int i;
            if (!cc_stage_out(ctx, &ctx->prevOff))
                return false;
            if (!cc_emit_pending_z_move_at(ctx, &ctx->prevOff))
                return false;
            for (i = 0; i < insertCount; ++i)
            {
                if (!cc_stage_out(ctx, &inserts[i]))
                    return false;
            }
        }

        if (curOff.compMode == CC_CM_IN)
            ctx->compMode = CC_CM_STEADY;
        else if (curOff.compMode == CC_CM_OUT)
            ctx->compMode = CC_CM_NONE;

        ctx->prevOff = curOff;
    }

    return true;
}

void cc_flush(cc_context *ctx)
{
    cc_process(ctx);
    if (ctx->havePrevMove)
    {
        if (!cc_stage_out(ctx, &ctx->prevOff))
            return;
        if (!cc_emit_pending_z_move_at(ctx, &ctx->prevOff))
            return;
        ctx->havePrevMove = false;
    }
    (void)cc_stage_flush(ctx);
}

bool cc_pop_out(cc_context *ctx, move2d *m)
{
    if (ctx->outCount == 0)
        return false;

    *m = ctx->output_buffer[ctx->outHead];
    ctx->outHead = (ctx->outHead + 1) % CC_OUT_CAP;
    ctx->outCount--;
    return true;
}

static inline void cc_core_drain(void)
{
    move2d out;
    if (!g_core_emit_cb)
        return;
    while (cc_pop_out(&g_core_ctx, &out))
    {
        if (!out.valid)
            continue;
        g_core_emit_cb(&out);
    }
}

// Usage:
// Call cc_api_init() whenever you need to reset the compensation core state, such as:
//   - When starting compensation (G41/G42)
//   - When aborting or stopping a job and before restarting
// Always provide the correct tool radius and callback pointers.
// Example:
//     cc_api_init(tool_radius, CC_UNITS_MM, emit_callback, error_callback);
void cc_api_init(float toolRadius, cc_units units, emit_move_cb emitCb, cc_msg_cb errCb)
{
    cc_init_internal(&g_core_ctx, toolRadius);
    g_core_ctx.units = units;

    // force to mm for grblhal.
    g_core_ctx.arcTol = CC_ARC_TOL_MM;
    g_core_ctx.gapTol = CC_GAP_TOL_MM;
    g_core_ctx.minOutputLen = CC_MIN_OUTPUT_LEN_MM;
    g_core_msg_cb = errCb;
    g_core_emit_cb = emitCb;
}

void cc_api_set_comp(comp_side side)
{
    cc_set_comp(&g_core_ctx, side);
}

comp_side cc_api_get_comp(void)
{
    // if tool rad is zero then comp is effectively off, even if the state is set to in or out
    if (g_core_ctx.toolR == 0.0f)
        return CC_COMP_OFF;

    return (comp_side)g_core_ctx.compSide;
}

comp_mode cc_api_get_mode(void)
{
    return g_core_ctx.compMode;
}

cc_corner_treatment_mode cc_api_get_corner_treatment_mode(void)
{
    return (cc_corner_treatment_mode)g_core_ctx.cornerTreatmentMode;
}

cc_status_code_t cc_api_process_move(const move2d *move)
{
     if (!move)
    {
        cc_flush(&g_core_ctx);
        if (g_core_ctx.stopErr)
            return g_core_ctx.status;
        cc_core_drain();
        return cc_status_OK;
    }

    if (!cc_push_in(&g_core_ctx, move))
        return cc_status_InputBufferOverflow;

    if (!cc_process(&g_core_ctx))
        return g_core_ctx.status;

    cc_core_drain();
    return cc_status_OK;
}
#endif