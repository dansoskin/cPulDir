/* Motion profile generator. Section 5.2 of docs/puldir-guidelines.md and
 * plan decisions D1, D13, D14. Float only, no static data. */
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "motion_profile.h"

#ifdef __FAST_MATH__
#error "compensated sums need strict float evaluation"
#endif

#define DT_MIN 0.00025f    /* 2 kHz plan tick (D14) */
#define DT_MAX 0.01f

/* The acceleration span the window must cover: a reversal puts +accel and -accel (or the larger of
 * accel and decel, both ways) one window apart, so it is twice the larger, not accel + decel (E31). */
static float acc_span(float accel, float decel)
{
    return 2.0f * fmaxf(accel, decel);
}

/* Jerk filter length for these settings; 0 when it would not fit the ring. */
static uint16_t window(MotionSettings S, float dt)
{
    if (S.jerk == 0.0f)
        return 1;
    const float n = ceilf(acc_span(S.accel, S.decel) / (S.jerk * dt));
    if (!(n <= (float)MOTION_PROFILE_MAX_TAPS))
        return 0;
    return (uint16_t)n;
}

static bool settings_valid(MotionSettings S)
{
    return isfinite(S.speed) && isfinite(S.accel) && isfinite(S.decel) && isfinite(S.jerk)
        && S.speed > 0.0f && S.accel > 0.0f && S.decel > 0.0f && S.jerk >= 0.0f;
}

static void clear_ring(MotionProfile* p)
{
    for (int i = 0; i < MOTION_PROFILE_MAX_TAPS; i++)
        p->taps[i] = 0.0f;
    p->head = 0;
    p->sum = p->sum_c = p->wsum = p->wsum_c = 0.0f;
    p->run = p->n;                        /* a ring of zeros is one-signed */
    p->run_sign = 0;
    p->hist_acc = 0.0f;
    p->hist_left = 0;
}

/* The largest acceleration the ring may hold: the current settings' and any replaced within n calls. */
static float acc_history(const MotionProfile* p)
{
    return fmaxf(p->hist_acc, fmaxf(p->settings.accel, p->settings.decel));
}

bool MotionProfile_init(MotionProfile* p, float pos, float dt)
{
    if (!isfinite(pos) || !(dt >= DT_MIN && dt <= DT_MAX))
        return false;
    p->dt = dt;
    p->pos = p->target = p->p_t = pos;
    p->p_c = 0.0f;
    p->vel = p->acc = p->v_t = p->speed = 0.0f;
    p->endless = p->stopping = false;
    p->landed = p->done = true;
    p->settings = (MotionSettings){ 1.0f, 1.0f, 1.0f, 0.0f };
    p->n = 1;
    p->drain = 0;
    clear_ring(p);
    return true;
}

MotionSettingsCheck MotionProfile_checkSettings(const MotionProfile* p, MotionSettings S)
{
    if (!settings_valid(S))
        return MOTION_SETTINGS_INVALID;
    if (window(S, p->dt) == 0)
        return MOTION_SETTINGS_TAPS;
    /* In flight the window is frozen, and the ring may still hold motion under any settings in force
     * during its last n calls (E32): it must bound a step between the largest of those accelerations
     * and the largest the new settings allow. New ramps larger than the history make this the plain
     * frozen-window test, 2 max(accel, decel) over n. */
    if (!p->done && S.jerk != 0.0f) {
        /* On the landing call v_t still holds the landing step (E16) and the next call zeroes it, a
         * step the new ramps do not bound: it counts as an acceleration of its own. */
        const float a_land = p->landed ? fabsf(p->v_t) / p->dt : 0.0f;
        const float a_new = fmaxf(fmaxf(S.accel, S.decel), a_land);
        if ((a_new + fmaxf(a_new, acc_history(p))) / (S.jerk * p->dt) > (float)p->n)
            return MOTION_SETTINGS_WINDOW;
    }
    return MOTION_SETTINGS_OK;
}

bool MotionProfile_setSettings(MotionProfile* p, MotionSettings S)
{
    if (MotionProfile_checkSettings(p, S) != MOTION_SETTINGS_OK)
        return false;
    if (p->done) {
        p->n = window(S, p->dt);
        clear_ring(p);
    } else {                              /* the replaced settings' motion stays in the ring for n calls */
        p->hist_acc = acc_history(p);
        p->hist_left = p->n;              /* restarted by each change: a set replaced twice within a
                                             window is kept up to 2n - 1 calls, which only refuses more */
    }
    p->settings = S;
    return true;
}

bool MotionProfile_setTarget(MotionProfile* p, float target)
{
    if (!isfinite(target))
        return false;
    p->target = target;
    p->endless = p->stopping = p->landed = p->done = false;
    p->drain = 0;
    return true;
}

bool MotionProfile_setEndless(MotionProfile* p, float speed)
{
    if (!isfinite(speed))
        return false;
    p->speed = speed;
    p->endless = true;
    p->stopping = p->landed = p->done = false;
    p->drain = 0;
    return true;
}

void MotionProfile_stop(MotionProfile* p)
{
    /* Once landed the trapezoid is at rest and its target is final. */
    if (p->done || p->landed)
        return;
    p->stopping = true;
    p->endless = false;
}

void MotionProfile_halt(MotionProfile* p)
{
    p->n = window(p->settings, p->dt);
    clear_ring(p);
    p->v_t = p->vel = p->acc = 0.0f;
    p->p_t = p->target = p->pos;
    p->p_c = 0.0f;
    p->endless = p->stopping = false;
    p->landed = p->done = true;
    p->drain = 0;
}

bool MotionProfile_setPosition(MotionProfile* p, float pos)
{
    if (!p->done || !isfinite(pos))
        return false;
    p->pos = p->p_t = p->target = pos;
    p->p_c = 0.0f;
    return true;
}

/* *s += x with compensation *c. Needs strict float evaluation: never -ffast-math. */
static void kahan_add(float* s, float* c, float x)
{
    const float y = x - *c;
    const float t = *s + y;
    *c = (t - *s) - y;
    *s = t;
}

/* Move v toward vc by at most up when |v| grows and down when it shrinks.
 * Never crosses zero in one call: it lands on zero and reverses next call. */
static float toward(float v, float vc, float up, float down)
{
    const bool same_dir = (v > 0.0f) == (vc > 0.0f);
    if (vc != 0.0f && (v == 0.0f || (same_dir && fabsf(vc) > fabsf(v))))
        return copysignf(fminf(fabsf(v) + up, fabsf(vc)), vc);
    const float floor_ = (same_dir && v != 0.0f) ? fabsf(vc) : 0.0f;
    /* Never shed less than down to rounding: the ceiling's margin, dd^2/2v,
     * is below an ulp of v on long ramps, so a short shed is never recovered. */
    float m = fmaxf(fabsf(v) - down, floor_);
    if (m > floor_ && fabsf(v) - m < down)       /* exact on long ramps (Sterbenz) */
        m = nextafterf(m, 0.0f);
    return copysignf(m, v);
}

/* One step of the trapezoid. Returns the velocity used over this step. */
static float trapezoid(MotionProfile* p)
{
    const MotionSettings S = p->settings;
    const float dt = p->dt, dd = S.decel * dt;
    if (p->landed) {
        p->v_t = 0.0f;                    /* at rest from the call after landing */
        return 0.0f;
    }

    const float rem = p->target - p->p_t;
    if (!p->endless && !p->stopping) {
        /* Landing: the rest of the way in one step, within the step rule (D13). */
        const float u = rem / dt, v = p->v_t;
        const bool same = v == 0.0f || rem == 0.0f || (u > 0.0f) == (v > 0.0f);
        const bool fits = fabsf(u) <= fabsf(v) ? fabsf(v) - fabsf(u) <= dd
                                               : fabsf(u) - fabsf(v) <= S.accel * dt;
        if (fabsf(rem) <= dd * dt && same && fits) {
            p->p_t = p->target;           /* assigned rather than integrated */
            p->p_c = 0.0f;
            /* A new course on the next call starts from u: from 0 it could step by |u| too much. */
            p->v_t = u;
            p->landed = true;
            return u;
        }
    }

    float vc;
    if (p->stopping)
        vc = 0.0f;
    else if (p->endless)
        vc = copysignf(fminf(fabsf(p->speed), S.speed), p->speed);
    else /* the discrete ceiling: do not simplify to sqrtf(2*decel*|rem|) */
        vc = copysignf(fminf(S.speed, sqrtf(dd * dd + 2.0f * S.decel * fabsf(rem)) - dd), rem);

    p->v_t = toward(p->v_t, vc, S.accel * dt, dd);
    /* Plain rounding of p_t misses the decel*dt^2 landing window at large coordinates. */
    kahan_add(&p->p_t, &p->p_c, p->v_t * dt);
    if (p->stopping && p->v_t == 0.0f) {
        p->target = p->p_t;
        p->p_c = 0.0f;
        p->stopping = false;
        p->landed = true;
    }
    return p->v_t;
}

/* How far the filtered output trails the trapezoid: dt * sum ((n-1-m)/n) tap_m,
 * m = 0 newest. Deriving pos from p_t keeps it from drifting off the trapezoid. */
static float lag(const MotionProfile* p)
{
    return p->wsum * (p->dt / (float)p->n);
}

/* The sums from the ring itself: the oldest tap, at head, has weight 0. */
static void recompute(MotionProfile* p)
{
    float s = 0.0f, c = 0.0f, w = 0.0f, wc = 0.0f;
    uint16_t i = p->head;
    for (uint16_t k = 0; k < p->n; k++) {
        kahan_add(&s, &c, p->taps[i]);
        kahan_add(&w, &wc, (float)k * p->taps[i]);
        if (++i == p->n)
            i = 0;
    }
    p->sum = s;
    p->wsum = w;
    p->sum_c = p->wsum_c = 0.0f;
}

/* Push the trapezoid's velocity into the ring. Compensated: plain running sums
 * drift by tens of ulps between recomputes, and each recompute would step vel. */
static void push(MotionProfile* p, float u)
{
    kahan_add(&p->sum, &p->sum_c, u);
    kahan_add(&p->sum, &p->sum_c, -p->taps[p->head]);
    /* the new tap enters at weight n-1; every other tap loses one */
    kahan_add(&p->wsum, &p->wsum_c, (float)p->n * u);
    kahan_add(&p->wsum, &p->wsum_c, -p->sum);
    p->taps[p->head] = u;
    /* The run of newest taps sharing a sign, zeros extending it: the drain clamps the lag only
     * once the run fills the ring. */
    if (u != 0.0f) {
        const int8_t s = u > 0.0f ? 1 : -1;
        if (s != p->run_sign && p->run_sign != 0)
            p->run = 0;
        p->run_sign = s;
    }
    if (p->run < p->n)
        p->run++;
    if (++p->head == p->n) {              /* recompute: rounding must not walk the sums */
        p->head = 0;
        recompute(p);
    }
}

/* The ring has drained: exact rest at the target, where lag is already 0. */
static void finish(MotionProfile* p)
{
    p->n = window(p->settings, p->dt);
    clear_ring(p);
    p->vel = p->acc = 0.0f;
    p->pos = p->p_t = p->target;
    p->p_c = 0.0f;
    p->done = true;
    p->drain = 0;
}

void MotionProfile_advance(MotionProfile* p)
{
    if (p->done)
        return;
    if (p->hist_left > 0 && --p->hist_left == 0)
        p->hist_acc = 0.0f;               /* the ring has turned over since the settings changed */
    const bool was_landed = p->landed;
    const float u = trapezoid(p);         /* 0 once landed: the ring drains through the same push */
    const float vel0 = p->vel, w0 = p->wsum;
    if (was_landed)
        p->drain++;
    push(p, u);
    if (was_landed && p->drain >= p->n) {
        p->sum = p->sum_c = p->wsum = p->wsum_c = 0.0f;   /* every tap is zero: exactly at rest */
    } else if (was_landed && p->run >= p->n) {
        /* Draining a one-signed ring, the lag only shrinks toward zero; a rounding residue past
         * that would put the output a hair beyond its target. A ring of both signs (a jog, then a
         * target just behind) still carries the output on before it returns: it drains as it is. */
        const float w = fminf(fmaxf(p->wsum, fminf(w0, 0.0f)), fmaxf(w0, 0.0f));
        if (w != p->wsum) {
            p->wsum = w;
            p->wsum_c = 0.0f;
        }
    }
    p->vel = p->sum / (float)p->n;
    p->pos = p->p_t - lag(p);
    p->acc = (p->vel - vel0) / p->dt;
    /* Done one call after the drain, so acc comes to 0 within the jerk bound. */
    if (p->drain > p->n)
        finish(p);
}

float MotionProfile_stopDistance(const MotionProfile* p)
{
    const float v = p->landed ? 0.0f : p->v_t;   /* a landing step's u is not motion to stop */
    return lag(p) + v * fabsf(v) / (2.0f * p->settings.decel);
}

/* D2: shift, never re-sync; the output follows p_t, and the lag stays with it. */
void MotionProfile_rebase(MotionProfile* p, float shift, float target)
{
    if (p->landed) {                      /* at rest on its target: keep them equal */
        p->target -= shift;
        p->p_t = p->target;
        p->p_c = 0.0f;
    } else {
        kahan_add(&p->p_t, &p->p_c, -shift);
        if (p->endless || p->stopping)
            p->target -= shift;           /* no target of the caller's to take */
        else
            p->target = target;
    }
    p->pos = p->p_t - lag(p);
}

bool MotionProfile_seed(MotionProfile* p, float pos, float vel)
{
    if (!p->done || !isfinite(pos) || !isfinite(vel))
        return false;
    for (uint16_t i = 0; i < p->n; i++)
        p->taps[i] = vel;
    p->head = 0;
    p->run = p->n;                        /* one-signed throughout */
    p->run_sign = vel > 0.0f ? 1 : vel < 0.0f ? -1 : 0;
    recompute(p);
    /* The trapezoid runs ahead by the full ring's lag. fl(n*vel)/n and p_t - lag
     * may each be an ulp off, so the next output is vel and pos + vel*dt to an ulp. */
    p->p_t = pos + lag(p);
    p->p_c = 0.0f;
    p->pos = p->target = pos;
    p->v_t = p->vel = p->speed = vel;
    p->acc = 0.0f;
    p->endless = true;
    p->stopping = p->landed = p->done = false;
    p->drain = 0;
    return true;
}

float MotionProfile_minJerk(float accel, float decel, float dt)
{
    return acc_span(accel, decel) / ((float)MOTION_PROFILE_MAX_TAPS * dt);
}
