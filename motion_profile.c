/* Motion profile generator. Section 5.2 of docs/puldir-guidelines.md and
 * plan decisions D1, D13, D14. Float only, no static data. */
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "motion_profile.h"

#define DT_MIN 0.00025f    /* 2 kHz plan tick (D14) */
#define DT_MAX 0.01f

/* Jerk filter length for these limits; 0 when it would not fit the ring. */
static uint16_t window(MotionLimits L, float dt)
{
    if (L.jmax == 0.0f)
        return 1;
    const float n = ceilf((L.amax + L.dmax) / (L.jmax * dt));
    if (!(n <= (float)MOTION_PROFILE_MAX_TAPS))
        return 0;
    return (uint16_t)n;
}

static bool limits_valid(MotionLimits L)
{
    return isfinite(L.vmax) && isfinite(L.amax) && isfinite(L.dmax) && isfinite(L.jmax)
        && L.vmax > 0.0f && L.amax > 0.0f && L.dmax > 0.0f && L.jmax >= 0.0f;
}

static void clear_ring(MotionProfile* p)
{
    for (int i = 0; i < MOTION_PROFILE_MAX_TAPS; i++)
        p->taps[i] = 0.0f;
    p->head = 0;
    p->sum = 0.0f;
}

bool MotionProfile_init(MotionProfile* p, float pos, float dt)
{
    if (!isfinite(pos) || !(dt >= DT_MIN && dt <= DT_MAX))
        return false;
    p->dt = dt;
    p->pos = p->target = p->p_t = pos;
    p->vel = p->acc = p->v_t = p->speed = 0.0f;
    p->endless = p->stopping = false;
    p->landed = p->done = true;
    p->lim = (MotionLimits){ 1.0f, 1.0f, 1.0f, 0.0f };
    p->n = 1;
    p->drain = 0;
    clear_ring(p);
    return true;
}

bool MotionProfile_setLimits(MotionProfile* p, MotionLimits L)
{
    if (!limits_valid(L))
        return false;
    const uint16_t n = window(L, p->dt);
    if (n == 0)
        return false;
    if (p->done) {
        p->n = n;
        clear_ring(p);
    } else if (L.jmax != 0.0f && (L.amax + L.dmax) / (L.jmax * p->dt) > (float)p->n) {
        return false;       /* the window is frozen in flight and would not bound this jerk */
    }
    p->lim = L;
    return true;
}

bool MotionProfile_setTarget(MotionProfile* p, float target)
{
    (void)p;
    (void)target;
    return false;
}

bool MotionProfile_setEndless(MotionProfile* p, float speed)
{
    (void)p;
    (void)speed;
    return false;
}

void MotionProfile_stop(MotionProfile* p)
{
    (void)p;
}

void MotionProfile_halt(MotionProfile* p)
{
    clear_ring(p);
    p->v_t = p->vel = p->acc = 0.0f;
    p->p_t = p->target = p->pos;
    p->endless = p->stopping = false;
    p->landed = p->done = true;
    p->drain = 0;
    p->n = window(p->lim, p->dt);
}

bool MotionProfile_setPosition(MotionProfile* p, float pos)
{
    if (!p->done || !isfinite(pos))
        return false;
    p->pos = p->p_t = p->target = pos;
    return true;
}

void MotionProfile_advance(MotionProfile* p)
{
    (void)p;
}

float MotionProfile_stopDistance(const MotionProfile* p)
{
    return (p->p_t - p->pos) + p->v_t * fabsf(p->v_t) / (2.0f * p->lim.dmax);
}

void MotionProfile_rebase(MotionProfile* p, float shift, float target)
{
    (void)p;
    (void)shift;
    (void)target;
}

bool MotionProfile_seed(MotionProfile* p, float pos, float vel)
{
    (void)p;
    (void)pos;
    (void)vel;
    return false;
}

float MotionProfile_minJerk(float amax, float dmax, float dt)
{
    return (amax + dmax) / ((float)MOTION_PROFILE_MAX_TAPS * dt);
}
