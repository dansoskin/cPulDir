/* The plan tick (guidelines 5.4): prediction, re-base, modes, publication.
 * Float lives here; the pulse tick sees only the published integers. */
#include "puldir_internal.h"

#define N   ((int64_t)PULDIR_TICKS_PER_PLAN)
#define M   (N * N)                             /* D6 margin, in 2^-32 pulse */
#define FIX 4294967296.0f                       /* one pulse in 32.32 */
#define SAT ((int64_t)1 << 56)                  /* 2^24 pulses (D2) */

static int64_t to_fix(float pulses) { return (int64_t)(pulses * FIX); }

/* Pulses as float, saturated to +-2^24 so the generator never sees more. */
static float to_pulses(int64_t q)
{
    if (q > SAT)
        q = SAT;
    else if (q < -SAT)
        q = -SAT;
    return (float)q * (1.0f / FIX);
}

/* The speed cap keeps cruise below PULDIR_INC_MAX; a speed one float rounding
 * under the cap can touch it. An internal invariant, never a limit on input. */
static int32_t sat(int64_t q)
{
    return q > PULDIR_INC_MAX ? PULDIR_INC_MAX : q < -PULDIR_INC_MAX ? -PULDIR_INC_MAX : (int32_t)q;
}

static int32_t div_trunc(int64_t d) { return sat(d / N); }

static int32_t div_away(int64_t d)
{
    int64_t q = d / N;
    if (q * N != d)
        q += d > 0 ? 1 : -1;
    return sat(q);
}

/* A non-zero inc carries its own sign as the direction; zero keeps the last one,
 * so a pause never writes the direction pin. */
static void publish(PulDir* a, PulDirInc* out, int32_t inc)
{
    if (inc != 0)
        a->dir_out = inc > 0 ? 1 : -1;
    out->inc = inc;
    out->dir = a->dir_out;
}

/* PROFILE: advance the generator and aim this interval's end at its position.
 * T, pb and end are 32.32 relative to base. */
static void run_profile(PulDir* a, PulDirInc* out, uint64_t base, int64_t pb, bool pul_high)
{
    MotionProfile* p = &a->prof;
    const bool was_done = p->done;
    MotionProfile_advance(p);
    if (p->landed)                              /* at rest in the trapezoid: its target is final */
        a->target = base + (uint64_t)to_fix(p->target);
    const int64_t T = (int64_t)(a->target - base);
    int64_t end = to_fix(p->pos);
    a->vel_out = p->vel;

    if (!p->done) {
        /* D6: hold the end M short of the target on the approach side until the
         * done interval, which then carries the final edge to its last tick. */
        const int64_t gap = T - end;
        if (gap < M && gap > -M && T != pb)
            end = T > pb ? T - M : T + M;
        publish(a, out, div_trunc(end - pb));
    } else if (!was_done) {
        publish(a, out, div_away(end - pb));    /* 5.4 step 5: cross the target's integer */
        a->settle = 2;
    } else {
        /* D19: the done interval runs after publication and its last pulse is
         * lowered on the first tick after it: two runs, then check the pin. */
        publish(a, out, 0);
        if (a->settle > 0)
            a->settle--;
        if (a->settle == 0 && !pul_high) {
            a->mode = PULDIR_MODE_IDLE;
            a->moving = false;
            a->events |= PULDIR_EV_LANDED;
        }
    }
}

static void run_axis(PulDir* a, PulDirInc* out, uint64_t Pb, int32_t count, bool pul_high)
{
    /* 1. Re-base on the whole pulses emitted. */
    const uint64_t base = (uint64_t)(uint32_t)count << 32;
    const int32_t shift = (int32_t)((uint32_t)count - (uint32_t)a->origin);
    a->origin = count;

    /* 2. Requests. Main writes req only under the lock, which masks this tick,
     * so reading and clearing it here needs none. */
    const uint16_t req = a->req;
    a->req = 0;
    if (req & REQ_LIMITS) {
        const float c = a->conv;
        const MotionLimits L = { a->lim[0] * c, a->lim[1] * c, a->lim[2] * c, a->lim[3] * c };
        (void)MotionProfile_setLimits(&a->prof, L);  /* validated the same way by apply_limits */
    }
    const float trel = to_pulses((int64_t)(a->target - base));
    if (req & REQ_TARGET) {
        (void)MotionProfile_setTarget(&a->prof, trel);   /* trel is finite */
        a->settle = 0;
    }

    /* 3. Shift the generator onto the new origin; the target is set relative to it. */
    MotionProfile_rebase(&a->prof, (float)shift, trel);

    switch (a->mode) {
    case PULDIR_MODE_PROFILE:
        run_profile(a, out, base, (int64_t)(Pb - base), pul_high);
        break;
    default:
        publish(a, out, 0);
        a->vel_out = 0.0f;
        break;
    }
}

void puldir_plan_tick(void)
{
    PulDirPulse s[PULDIR_MAX_AXES];
    bool high[PULDIR_MAX_AXES];
    const uint8_t n = puldir_naxes;

    port_lock();                                /* one consistent instant for every axis */
    /* Ticks the live increments still run before the next latch. At a boundary
     * (left == N) the set published below latches on the very next tick, so none. */
    const uint32_t ahead = puldir_left() % (uint32_t)N;
    for (uint8_t i = 0; i < n; i++) {
        puldir_pulse_read(i, &s[i]);
        high[i] = puldir_pul_high(i);
    }
    port_unlock();

    PulDirInc* out = puldir_shadow();
    for (uint8_t i = 0; i < n; i++) {
        const uint64_t Pb = s[i].P + (uint64_t)((int64_t)s[i].inc * ahead);   /* D5 prediction */
        run_axis(puldir_axes[i], &out[i], Pb, s[i].count, high[i]);
    }
    puldir_publish();
}
