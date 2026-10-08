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

/* PROFILE and STOPPING: aim this interval's end at the generator's position.
 * T, pb and end are 32.32 relative to base; live is the running increment. */
static void run_profile(PulDir* a, PulDirInc* out, uint64_t base, int64_t pb, int32_t live,
                        bool pul_high)
{
    MotionProfile* p = &a->prof;
    MotionProfile_advance(p);
    int64_t end = to_fix(p->pos);
    if (!a->arrived) {
        /* At rest the trapezoid's target is final, fractional after a stop (D4); a
         * jog or a stop in progress has none, so it reads the profile. */
        if (p->landed)
            a->target = base + (uint64_t)to_fix(p->target);
        else if (p->endless || p->stopping)
            a->target = base + (uint64_t)end;
    }
    const int64_t T = (int64_t)(a->target - base);
    a->vel_out = p->vel;

    if (!a->arrived && p->landed && end == T) {
        /* Arrival: round away from zero so the target's integer is crossed (5.4
         * step 5). D6 kept the last interval M short, so this one moves more than
         * N(N-1) and the final edge falls on its last tick. */
        a->arrived = true;
        a->approach = T > pb ? 1 : T < pb ? -1 : 0;
        a->settle = 2;
        publish(a, out, div_away(T - pb));
    } else if (!a->arrived) {
        /* D6: hold the end M short of the target on the approach side until
         * arrival. Only toward a target: a jog or a stop in progress has none. */
        const int64_t gap = T - end;
        if (!p->endless && !p->stopping && gap < M && gap > -M && T != pb)
            end = T > pb ? T - M : T + M;
        publish(a, out, div_trunc(end - pb));
    } else {
        /* Settling (D19): the final interval runs after publication and its last
         * pulse is lowered on the tick after it. If the final set latched late or
         * was overwritten, the boundary is still short of the target: steer again. */
        const bool short_of = (a->approach > 0 && pb < T) || (a->approach < 0 && pb > T);
        if (short_of) {
            publish(a, out, div_away(T - pb));
            a->settle = 2;
        } else {
            publish(a, out, 0);
            if (a->settle > 0)
                a->settle--;
        }
        /* The live increment too: a plan tick run twice in an interval can see
         * the zeros published before the final set has run out. */
        if (a->settle == 0 && p->done && !pul_high && live == 0) {
            a->target = base;                   /* the target, or the last integer a stop reached */
            a->mode = PULDIR_MODE_IDLE;
            a->moving = false;
            a->events |= PULDIR_EV_LANDED;
        }
    }
}

static void run_axis(PulDir* a, PulDirInc* out, uint64_t Pb, int32_t count, int32_t live,
                     bool pul_high)
{
    /* 1. Re-base on the whole pulses emitted. */
    const uint64_t base = (uint64_t)(uint32_t)count << 32;
    const int32_t shift = (int32_t)((uint32_t)count - (uint32_t)a->origin);
    const int64_t pb = (int64_t)(Pb - base);
    a->origin = count;

    /* 2. Shift the generator onto the new origin. A profile heading for a target
     * takes the axis's target relative to it. */
    const float trel = to_pulses((int64_t)(a->target - base));
    MotionProfile_rebase(&a->prof, (float)shift, trel);

    /* 3. Requests (main writes req under the lock, which masks this tick). Halt
     * first, so later commands start where the pulses stopped; a stop last. */
    const uint16_t req = a->req;
    a->req = 0;
    if (req & REQ_LIMITS) {
        const float c = a->conv;
        const MotionLimits L = { a->lim[0] * c, a->lim[1] * c, a->lim[2] * c, a->lim[3] * c };
        (void)MotionProfile_setLimits(&a->prof, L);  /* validated the same way by apply_limits */
    }
    if (req & REQ_HALT) {
        /* puldir_kill zeroed the increments, so pb is where the pulses stopped */
        MotionProfile_halt(&a->prof);
        (void)MotionProfile_setPosition(&a->prof, to_pulses(pb));
        a->arrived = false;
        a->settle = 0;
    }
    if (req & REQ_SETPOS)                       /* the datum is count, which is the origin */
        (void)MotionProfile_setPosition(&a->prof, 0.0f);
    if (req & (REQ_TARGET | REQ_ENDLESS)) {
        if (req & REQ_TARGET)
            (void)MotionProfile_setTarget(&a->prof, trel);
        else
            (void)MotionProfile_setEndless(&a->prof, a->req_arg[0]);
        a->arrived = false;                     /* so must every request that sets a new course */
        a->settle = 0;
    }
    if (req & REQ_STOP)
        MotionProfile_stop(&a->prof);

    switch (a->mode) {
    case PULDIR_MODE_PROFILE:
    case PULDIR_MODE_STOPPING:
        run_profile(a, out, base, pb, live, pul_high);
        break;
    case PULDIR_MODE_IDLE:
        publish(a, out, 0);
        a->vel_out = 0.0f;
        if (a->moving && !pul_high && live == 0)    /* after a halt: the last pulse is lowered */
            a->moving = false;
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
        run_axis(puldir_axes[i], &out[i], Pb, s[i].count, s[i].inc, high[i]);
    }
    puldir_publish();
}
