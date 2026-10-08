/* Groups (5.7): one path profile s from 0 to 1, member i aiming at start_i + delta_i * s, so every
 * member arrives on one interval and D6 puts every final rising edge on its last tick. */
#include <math.h>
#include "puldir_internal.h"

/* The longest member travel: past 2^24 pulses one ulp of s moves a member by more than a pulse
 * (6x its speed limit at 2^28). It also keeps delta * s inside int64 without a clamp. */
#define TRAVEL_MAX ((int64_t)1 << 24)

static PulDirGroup* groups[PULDIR_MAX_GROUPS];
static uint8_t ngroups;

static bool listed(const PulDirGroup* g)
{
    for (uint8_t k = 0; k < ngroups; k++)
        if (groups[k] == g)
            return true;
    return false;
}

/* D16: refuses a member list it cannot run, and a full table. The plan tick ignores an inactive,
 * still group, so only joining the table takes the lock. */
bool PulDirGroup_init(PulDirGroup* g, PulDir** axes, uint8_t count)
{
    PulDir* first = (axes != NULL && count > 0u) ? axes[0] : NULL;
    PulDirRefusal why = PULDIR_R_NONE;
    if (axes == NULL || count == 0u || count > PULDIR_MAX_AXES || !puldir_timing.set) {
        why = PULDIR_R_ARG;                     /* the path profile needs the plan interval */
    } else {
        for (uint8_t i = 0; i < count && why == PULDIR_R_NONE; i++) {
            if (axes[i] == NULL || axes[i]->slot >= PULDIR_MAX_AXES)
                why = PULDIR_R_ARG;             /* missing, or not registered */
            for (uint8_t j = 0; j < i; j++)
                if (axes[j] == axes[i])
                    why = PULDIR_R_ARG;         /* a member twice */
        }
    }
    const bool known = listed(g);
    if (why == PULDIR_R_NONE) {
        if (known && (g->active || g->moving))
            why = PULDIR_R_BUSY;
        else if (!known && ngroups >= PULDIR_MAX_GROUPS)
            why = PULDIR_R_FULL;
    }
    if (why != PULDIR_R_NONE) {
        if (first != NULL)
            puldir_refuse(first, "groupInit", why, 0.0f);
        return false;
    }
    for (uint8_t i = 0; i < count; i++)
        g->axes[i] = axes[i];
    g->count = count;
    g->active = g->moving = g->faulted = false;
    g->started = g->still = false;
    g->req = 0;
    for (uint8_t i = 0; i < PULDIR_MAX_AXES; i++) {
        g->start[i] = 0;
        g->delta[i] = 0;
    }
    (void)MotionProfile_init(&g->s, 0.0f, puldir_timing.dt);  /* set, so in range: cannot refuse */
    if (!known) {
        port_lock();
        groups[ngroups] = g;
        ngroups++;
        port_unlock();
    }
    return true;
}

/* 5.5: a member must be IDLE and at rest, not faulted, not in a group's move. */
static PulDirRefusal member_refusal(const PulDir* a)
{
    if (puldir_fault_latched(a))
        return PULDIR_R_FAULT;
    if (a->mode == PULDIR_MODE_GROUP)
        return PULDIR_R_BUSY;                   /* in this group's move or another's */
    if (a->mode != PULDIR_MODE_IDLE || a->moving)
        return PULDIR_R_NOT_IDLE;
    return PULDIR_R_NONE;
}

/* The path's limits in s per second (5.7): each member's own in pulses over its travel, the
 * tightest winning. A member without travel, or without jerk, does not constrain. */
static bool path_limits(const PulDirGroup* g, const int32_t* delta, MotionLimits* L)
{
    float l[4] = { INFINITY, INFINITY, INFINITY, INFINITY };
    bool any = false;
    for (uint8_t i = 0; i < g->count; i++) {
        if (delta[i] == 0)
            continue;
        const PulDir* a = g->axes[i];
        const float d = fabsf((float)delta[i]);
        any = true;
        for (int k = 0; k < 4; k++)
            if (k < 3 || a->lim[3] > 0.0f)
                l[k] = fminf(l[k], a->lim[k] * a->conv / d);
    }
    *L = (MotionLimits){ l[0], l[1], l[2], isfinite(l[3]) ? l[3] : 0.0f };
    return any;
}

/* Whole-pulse targets (D4), all built before the lock: the plan tick reads no inactive group and
 * only main leaves IDLE. Only the fault latch can change meanwhile, so it is checked again. */
bool PulDirGroup_moveTo(PulDirGroup* g, const float* targets)
{
    const uint8_t n = g->count;
    PulDir* bad = n > 0u ? g->axes[0] : NULL;
    PulDirRefusal why = n > 0u ? PULDIR_R_NONE : PULDIR_R_ARG;
    for (uint8_t i = 0; i < n && why == PULDIR_R_NONE; i++) {
        why = member_refusal(g->axes[i]);
        bad = g->axes[i];
    }
    if (why == PULDIR_R_NONE && g->active) {
        why = PULDIR_R_BUSY;
        bad = g->axes[0];
    }

    int32_t count[PULDIR_MAX_AXES], delta[PULDIR_MAX_AXES];
    if (why == PULDIR_R_NONE) {
        port_lock();                            /* at rest: the counts do not move */
        for (uint8_t i = 0; i < n; i++) {
            PulDirSnap p;
            puldir_pulse_read(g->axes[i]->slot, 0u, &p);
            count[i] = p.count;
        }
        port_unlock();
    }
    for (uint8_t i = 0; i < n && why == PULDIR_R_NONE; i++) {
        const PulDir* a = g->axes[i];
        const float x = roundf(targets[i] * a->conv);
        const bool fits = x >= -PULDIR_PULSES_MAX && x < PULDIR_PULSES_MAX;      /* false for NaN */
        const int64_t d = fits ? (int64_t)x - count[i] : 0;
        if (!fits || d > TRAVEL_MAX || d < -TRAVEL_MAX) {
            why = PULDIR_R_ARG;
            bad = g->axes[i];
        } else {
            delta[i] = (int32_t)d;
        }
    }

    MotionLimits L = { 1.0f, 1.0f, 1.0f, 0.0f };
    bool any = false;
    if (why == PULDIR_R_NONE) {
        bad = g->axes[0];                       /* a path refusal: reported on the first member */
        any = path_limits(g, delta, &L);
        /* Not active: the plan tick leaves s alone, and a halt leaves it in flight, so it is reset
         * first, at rest, with no frozen window and no history for the check; refused, it stays so,
         * unread. Invalid is an underflow: limits too small for the travel. Each member's own window
         * fits and the tightest jerk's member bounds the ratio, so only float rounding at the
         * 256-tap edge lands on the taps. */
        (void)MotionProfile_init(&g->s, 0.0f, puldir_timing.dt);   /* set, so in range: cannot refuse */
        const MotionLimitsCheck check = any ? MotionProfile_checkLimits(&g->s, L) : MOTION_LIMITS_OK;
        if (check == MOTION_LIMITS_INVALID)
            why = PULDIR_R_ARG;
        else if (check == MOTION_LIMITS_TAPS)
            why = PULDIR_R_TAPS;
    }
    if (why != PULDIR_R_NONE) {
        if (bad != NULL)
            puldir_refuse(bad, "groupMoveTo", why, 0.0f);
        return false;
    }

    /* Not active: the plan tick leaves s and these fields alone. */
    if (any) {
        (void)MotionProfile_setLimits(&g->s, L);   /* checked above as the generator checks */
        (void)MotionProfile_setTarget(&g->s, 1.0f);
    } else {
        (void)MotionProfile_init(&g->s, 1.0f, puldir_timing.dt); /* at rest on the end already */
    }
    for (uint8_t i = 0; i < n; i++) {
        g->start[i] = (uint64_t)(uint32_t)count[i] << 32;
        g->delta[i] = delta[i];
    }
    g->started = false;
    g->still = !any;
    g->req = 0;

    port_lock();
    for (uint8_t i = 0; i < n && why == PULDIR_R_NONE; i++)
        if (puldir_fault_latched(g->axes[i])) {
            why = PULDIR_R_FAULT;
            bad = g->axes[i];
        }
    if (why == PULDIR_R_NONE) {
        for (uint8_t i = 0; i < n; i++) {
            PulDir* a = g->axes[i];
            a->target = g->start[i] + ((uint64_t)(int64_t)delta[i] << 32);
            a->mode = PULDIR_MODE_GROUP;
            a->moving = true;
            a->group = g;
            a->arrived = false;                 /* a new course (E17) */
            a->settle = 0;
            a->vel_out = 0.0f;
            puldir_enable_wake(a);              /* 5.8: s waits for every member's enable delay */
        }
        g->faulted = false;
        g->moving = true;
        g->active = true;
    }
    port_unlock();
    if (why != PULDIR_R_NONE) {
        puldir_refuse(bad, "groupMoveTo", why, 0.0f);
        return false;
    }
    return true;
}

/* Ramped, along the line: the plan tick stops s at its decel. */
void PulDirGroup_stop(PulDirGroup* g)
{
    port_lock();
    if (g->active)
        g->req |= GROUP_REQ_STOP;
    port_unlock();
}

void PulDirGroup_halt(PulDirGroup* g)
{
    port_lock();
    puldir_group_halt(g, NULL);
    port_unlock();
}

void puldir_group_halt(PulDirGroup* g, PulDirInc* shadow)
{
    if (!g->active)
        return;
    for (uint8_t i = 0; i < g->count; i++) {
        PulDir* a = g->axes[i];
        PulDirSnap p;
        puldir_kill(a->slot);
        if (shadow != NULL)
            shadow[a->slot].inc = 0;
        puldir_pulse_read(a->slot, 0u, &p);
        a->target = (uint64_t)(uint32_t)p.count << 32;
        /* The member's generator sat at rest through the move; the plan tick puts
         * it where the pulses stopped. Motion is already over without it. */
        a->req |= REQ_HALT;
        a->mode = PULDIR_MODE_IDLE;
        a->vel_out = 0.0f;
        a->arrived = false;                     /* E17 */
        a->settle = 0;
        a->group = NULL;
    }
    g->active = false;                          /* moving clears once every member is at rest */
}

bool PulDirGroup_isMoving(PulDirGroup* g) { return g->moving; }
bool PulDirGroup_faulted(PulDirGroup* g)  { return g->faulted; }

/* ---- the plan tick's part ----------------------------------------------------------------- */

/* s in 32.32. s stays in [0, 1] (the trapezoid never passes its target, E2), and
 * |delta| <= 2^24, so delta * s_fix fits int64 with room to spare. */
static int64_t s_fix(float s)
{
    return (int64_t)(s * PULDIR_FIX);
}

/* The direction member i must show before s advances. */
static int8_t wanted(const PulDirGroup* g, uint8_t i)
{
    const int32_t d = g->delta[i];
    return d > 0 ? 1 : d < 0 ? -1 : g->axes[i]->dir_out;
}

/* D19 for the group: every member IDLE together, its generator on its target as
 * an axis's rests after landing, and its origin on the count. */
static void finish(PulDirGroup* g, const PulDirSnap* sn)
{
    for (uint8_t i = 0; i < g->count; i++) {
        PulDir* a = g->axes[i];
        const PulDirSnap* q = &sn[a->slot];
        const uint64_t base = (uint64_t)(uint32_t)q->count << 32;
        const int64_t T = (int64_t)(a->target - base);
        (void)MotionProfile_setPosition(&a->prof, (float)T * (1.0f / PULDIR_FIX));
        a->origin = q->count;
        a->target = base;                       /* the target, or the last integer a stop reached */
        a->vel_out = 0.0f;
        a->mode = PULDIR_MODE_IDLE;
        a->moving = false;
        a->group = NULL;
        a->events |= PULDIR_EV_LANDED;
    }
    g->active = false;
    g->moving = false;
}

static void run_group(PulDirGroup* g, const PulDirSnap* sn, PulDirInc* out)
{
    const uint8_t n = g->count;
    if (!g->active) {
        /* After a halt: moving until every member is at rest (its last pulse lowered). */
        if (g->moving) {
            bool rest = true;
            for (uint8_t i = 0; i < n; i++)
                rest = rest && !g->axes[i]->moving;
            if (rest)
                g->moving = false;
        }
        return;
    }
    if (g->still) {                             /* every travel zero: done, no pulse */
        for (uint8_t i = 0; i < n; i++)
            puldir_set_inc(g->axes[i], &out[g->axes[i]->slot], 0);
        finish(g, sn);
        return;
    }

    MotionProfile* s = &g->s;
    const uint8_t req = g->req;
    g->req = 0;
    /* Before s lands no member has arrived, so nothing else changes course (E17);
     * once landed, MotionProfile_stop ignores it and the move ends as planned. */
    if (req & GROUP_REQ_STOP)
        MotionProfile_stop(s);

    if (!g->started) {
        /* A reversal costs a member ticks (E14), so s waits for every member's direction and enable
         * delay (5.8), publishing directions alone. A stop before the start need not wait. */
        bool ready = true;
        for (uint8_t i = 0; i < n; i++)
            ready = ready && sn[g->axes[i]->slot].dir == wanted(g, i) && !puldir_enable_delaying(g->axes[i]);
        if (!ready && !s->stopping) {
            for (uint8_t i = 0; i < n; i++) {
                PulDir* a = g->axes[i];
                a->dir_out = wanted(g, i);
                out[a->slot].inc = 0;
                out[a->slot].dir = a->dir_out;
            }
            return;
        }
        g->started = true;
    }

    MotionProfile_advance(s);
    const int64_t sf = s_fix(s->pos);
    /* Arrival: s has landed with its output on its target (1.0f, or where a stop
     * ended). Every member's end equals its target on this same call. */
    const bool arrival = s->landed && s->pos == s->target;
    bool settled = true, rest = true;
    for (uint8_t i = 0; i < n; i++) {
        PulDir* a = g->axes[i];
        const PulDirSnap* q = &sn[a->slot];
        const int64_t d = g->delta[i];
        const uint64_t base = (uint64_t)(uint32_t)q->count << 32;
        const int64_t pb = (int64_t)(q->Pb - base);
        const int64_t end = (int64_t)(g->start[i] - base) + d * sf;
        if (!a->arrived) {
            /* As for an axis: landed, the target is final, fractional after a stop;
             * a stop in progress has none, so it reads the path. */
            if (s->landed)
                a->target = g->start[i] + (uint64_t)(d * s_fix(s->target));
            else if (s->stopping)
                a->target = base + (uint64_t)end;
        }
        const int64_t T = (int64_t)(a->target - base);
        a->vel_out = s->vel == 0.0f ? 0.0f : (float)d * s->vel;
        const bool done = puldir_steer(a, &out[a->slot], T, end, pb, arrival, !s->stopping);
        settled = settled && done;
        rest = rest && !q->high && q->inc == 0;
    }
    if (settled && s->done && rest)
        finish(g, sn);
}

void puldir_group_tick(const PulDirSnap* snap, PulDirInc* out)
{
    for (uint8_t k = 0; k < ngroups; k++)
        run_group(groups[k], snap, out);
}
