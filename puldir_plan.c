/* The plan tick (guidelines 5.4): prediction, re-base, modes, publication.
 * Float lives here; the pulse tick sees only the published integers. */
#include "puldir_internal.h"

#define N   ((int64_t)PULDIR_TICKS_PER_PLAN)
#define M   (N * N)                             /* D6 margin, in 2^-32 pulse */
#define FIX 4294967296.0f                       /* one pulse in 32.32 */
#define SAT ((int64_t)1 << 56)                  /* 2^24 pulses (D2) */
#define DT  (1.0f / (float)PULDIR_PLAN_HZ)      /* the profile's dt, the same expression as init's */
#define TIMEOUT_TICKS ((uint32_t)((uint64_t)PULDIR_STREAM_TIMEOUT_MS * PULDIR_TICK_HZ / 1000u))

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

void puldir_set_inc(PulDir* a, PulDirInc* out, int32_t inc)
{
    if (inc != 0)
        a->dir_out = inc > 0 ? 1 : -1;
    out->inc = inc;
    out->dir = a->dir_out;
}

bool puldir_steer(PulDir* a, PulDirInc* out, int64_t T, int64_t end, int64_t pb, bool arrival, bool hold)
{
    if (!a->arrived && arrival) {
        /* Arrival: round away from zero so the target's integer is crossed (5.4
         * step 5). D6 kept the last interval M short, so this one moves more than
         * N(N-1) and the final edge falls on its last tick. */
        int32_t inc = div_away(T - pb);
        /* No edge lies between pb and T (D3): a fraction back must not turn the pin. */
        const int64_t one = (int64_t)PULDIR_ONE;
        const bool no_edge = T < one && T > -one && pb < one && pb > -one;
        if (no_edge && inc != 0 && (inc > 0) != (a->dir_out > 0))
            inc = 0;
        a->arrived = true;
        a->approach = inc > 0 ? 1 : inc < 0 ? -1 : 0;
        a->settle = 2;
        puldir_set_inc(a, out, inc);
    } else if (!a->arrived) {
        /* D6: hold the end M short of the target on the approach side until
         * arrival. Only toward a target: a jog or a stop in progress has none. */
        const int64_t gap = T - end;
        if (hold && gap < M && gap > -M && T != pb)
            end = T > pb ? T - M : T + M;
        puldir_set_inc(a, out, div_trunc(end - pb));
    } else {
        /* Settling (D19): the final interval runs after publication and its last
         * pulse is lowered on the tick after it. If the final set latched late or
         * was overwritten, the boundary is still short of the target: steer again. */
        const bool short_of = (a->approach > 0 && pb < T) || (a->approach < 0 && pb > T);
        if (short_of) {
            puldir_set_inc(a, out, div_away(T - pb));
            a->settle = 2;
        } else {
            puldir_set_inc(a, out, 0);
            if (a->settle > 0)
                a->settle--;
        }
    }
    return a->arrived && a->settle == 0;
}

/* PROFILE and STOPPING: aim this interval's end at the generator's position.
 * T, pb and end are 32.32 relative to base; live is the running increment. */
static void run_profile(PulDir* a, PulDirInc* out, uint64_t base, int64_t pb, int32_t live,
                        bool pul_high)
{
    MotionProfile* p = &a->prof;
    MotionProfile_advance(p);
    const int64_t end = to_fix(p->pos);
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

    const bool settled = puldir_steer(a, out, T, end, pb, p->landed && end == T,
                                      !p->endless && !p->stopping);
    /* The live increment too: a plan tick run twice in an interval can see the
     * zeros published before the final set has run out. */
    if (settled && p->done && !pul_high && live == 0) {
        a->target = base;                       /* the target, or the last integer a stop reached */
        a->mode = PULDIR_MODE_IDLE;
        a->moving = false;
        a->events |= PULDIR_EV_LANDED;
    }
}

/* Leave STREAM for STOPPING (5.6): the idle generator is put in motion at the
 * stream's velocity through the boundary position, then stopped at its decel. */
static void leave_stream(PulDir* a, int64_t pb)
{
    /* The generator was halted by streamBegin and stays done: seed accepts. */
    (void)MotionProfile_seed(&a->prof, to_pulses(pb), a->v_cmd);
    MotionProfile_stop(&a->prof);
    a->stream_live = false;
    a->arrived = false;                         /* a new course (E17) */
    a->settle = 0;
}

/* STREAM: feedforward plus proportional catch-up toward the first-order hold,
 * evaluated at t_b, the boundary where the set published now takes over. pb is
 * P_b relative to count, so no large float is subtracted from another. */
static void run_stream(PulDir* a, PulDirInc* out, uint64_t base, int64_t pb, int32_t count,
                       uint32_t t_b)
{
    const float frac = (float)pb * (1.0f / FIX);
    if (a->sp_have) {
        /* sp_pos - P_b as (whole part - count) + (sp_pos's fraction) - frac: each
         * term is exact or small. PulDir_stream keeps |sp_pos| below 2^31. */
        const int32_t k = (int32_t)a->sp_pos;
        /* A setpoint stamped after the snapshot (a caller outside main context)
         * is taken as of now: its age is clamped at zero, never wrapped. */
        const int32_t ticks_old = (int32_t)(t_b - a->sp_tick);
        const float age = (float)(ticks_old > 0 ? ticks_old : 0) * (1.0f / (float)PULDIR_TICK_HZ);
        const float err = (float)((int64_t)k - (int64_t)count) + (a->sp_pos - (float)k)
                        + a->sp_vel * age - frac;
        a->stream_err = err;
        a->v_cmd = a->sp_vel + err * (1000.0f / (float)a->catchup_ms);
    }
    /* Specified behaviour (5.6), not input clamping: in STREAM the configured
     * speed limit bounds v_cmd, and it lies below the tick ceiling. */
    const float vmax = a->lim[0] * a->conv;
    if (a->v_cmd > vmax)
        a->v_cmd = vmax;
    else if (a->v_cmd < -vmax)
        a->v_cmd = -vmax;

    const int32_t inc = div_trunc(to_fix(a->v_cmd * DT));
    puldir_set_inc(a, out, inc);
    a->vel_out = a->v_cmd;
    a->target = base + (uint64_t)(pb + (int64_t)inc * N);   /* the aim, as a jog reads it */
    (void)MotionProfile_setPosition(&a->prof, frac);        /* the idle generator follows the pulses */
}

static void run_axis(PulDir* a, PulDirInc* out, uint64_t Pb, int32_t count, int32_t live,
                     bool pul_high, uint32_t now, uint32_t ahead)
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
     * first, so later commands start where the pulses stopped; then a stream
     * begins, so limits reach an idle generator; a stop last. */
    const uint16_t req = a->req;
    a->req = 0;
    if (req & REQ_HALT) {
        /* puldir_kill zeroed the increments, so pb is where the pulses stopped */
        MotionProfile_halt(&a->prof);
        (void)MotionProfile_setPosition(&a->prof, to_pulses(pb));
        a->arrived = false;
        a->settle = 0;
        a->stream_live = false;
    }
    if (req & REQ_STREAM_BEGIN) {
        /* The stream starts at the velocity the running set was aimed with (a
         * stream still live, ended and begun within one interval, keeps its own).
         * The generator stays idle, where the pulses are, until the stream ends. */
        if (!a->stream_live)
            a->v_cmd = a->prof.vel;
        MotionProfile_halt(&a->prof);
        (void)MotionProfile_setPosition(&a->prof, to_pulses(pb));
        a->stream_live = true;
        a->arrived = false;                     /* the generator's goal is gone (E17) */
        a->settle = 0;
    }
    if (req & REQ_LIMITS) {
        const float c = a->conv;
        const MotionLimits L = { a->lim[0] * c, a->lim[1] * c, a->lim[2] * c, a->lim[3] * c };
        (void)MotionProfile_setLimits(&a->prof, L);  /* validated the same way by apply_limits */
    }
    if (req & REQ_SETPOS)                       /* the datum is count, which is the origin */
        (void)MotionProfile_setPosition(&a->prof, 0.0f);
    /* streamEnd (its defaults applied just above) or stop left STREAM; so does the
     * timeout. A move accepted in STOPPING after it starts from the seeded motion. */
    if (a->stream_live && a->mode != PULDIR_MODE_STREAM) {
        leave_stream(a, pb);
    } else if (a->mode == PULDIR_MODE_STREAM && (int32_t)(now - a->sp_tick) >= (int32_t)TIMEOUT_TICKS) {
        a->mode = PULDIR_MODE_STOPPING;         /* at the axis's decel, without the defaults */
        a->stream_timed_out = true;
        a->events |= PULDIR_EV_STREAM_TIMEOUT;
        leave_stream(a, pb);
    }
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
    case PULDIR_MODE_STREAM:
        run_stream(a, out, base, pb, count, now + ahead);
        break;
    case PULDIR_MODE_IDLE:
        puldir_set_inc(a, out, 0);
        a->vel_out = 0.0f;
        if (a->moving && !pul_high && live == 0)    /* after a halt: the last pulse is lowered */
            a->moving = false;
        break;
    default:
        puldir_set_inc(a, out, 0);
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
    const uint32_t now = PulDir_ticks();        /* the tick count P was read at */
    for (uint8_t i = 0; i < n; i++) {
        puldir_pulse_read(i, &s[i]);
        high[i] = puldir_pul_high(i);
    }
    port_unlock();

    PulDirSnap snap[PULDIR_MAX_AXES];
    for (uint8_t i = 0; i < n; i++) {
        snap[i].Pb = s[i].P + (uint64_t)((int64_t)s[i].inc * ahead);   /* D5 prediction */
        snap[i].count = s[i].count;
        snap[i].live = s[i].inc;
        snap[i].dir = s[i].dir;
        snap[i].high = high[i];
    }

    PulDirInc* out = puldir_shadow();
    puldir_group_tick(snap, out);               /* the groups write their members' entries */
    for (uint8_t i = 0; i < n; i++) {
        PulDir* a = puldir_axes[i];
        if (a->mode != PULDIR_MODE_GROUP)
            run_axis(a, &out[i], snap[i].Pb, snap[i].count, snap[i].live, snap[i].high, now, ahead);
    }
    puldir_publish();
}
