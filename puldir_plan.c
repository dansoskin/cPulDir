/* The plan tick (guidelines 5.4): prediction, re-base, modes, publication.
 * Float lives here; the pulse tick sees only the published integers. */
#include <math.h>
#include "puldir_internal.h"

#define N   ((int64_t)puldir_timing.n)          /* fixed before the ticks start */
#define M   (N * N)                             /* D6 margin, in 2^-32 pulse */
#define SAT ((int64_t)1 << 56)                  /* 2^24 pulses (D2) */
#define TIMEOUT_TICKS  puldir_ms_ticks(PULDIR_STREAM_TIMEOUT_MS)

static int64_t to_fix(float pulses) { return (int64_t)(pulses * PULDIR_FIX); }

/* Pulses as float, saturated to +-2^24 so the generator never sees more. */
float puldir_to_pulses(int64_t q)
{
    if (q > SAT)
        q = SAT;
    else if (q < -SAT)
        q = -SAT;
    return (float)q * (1.0f / PULDIR_FIX);
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
        /* Arrival rounds away from zero to cross the target's integer (5.4 step 5); D6 kept the
         * last interval M short, so this one moves more than N(N-1): the edge is on its last tick. */
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
        /* Settling (D19): the final interval runs after publication. If its set latched late or was
         * overwritten, the boundary is still short of the target: steer again. */
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

/* The generator stops dead where the pulses are (pb, relative to the origin): a
 * new goal (E17), and a stream's idle generator stays put. */
void puldir_halt_generator(PulDir* a, int64_t pb)
{
    MotionProfile_halt(&a->prof);
    (void)MotionProfile_setPosition(&a->prof, puldir_to_pulses(pb));
    a->arrived = false;
    a->settle = 0;
    a->stream_live = false;
}

/* Aims this interval's end at the generator's position (T, pb, end: 32.32 from base). True at rest:
 * settled on the final position, the generator done, the last pulse lowered. */
bool puldir_run_generator(PulDir* a, PulDirInc* out, uint64_t base, int64_t pb, int32_t live,
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
    return settled && p->done && !pul_high && live == 0;
}

/* PROFILE and STOPPING. */
static void run_profile(PulDir* a, PulDirInc* out, uint64_t base, int64_t pb, int32_t live,
                        bool pul_high)
{
    if (puldir_run_generator(a, out, base, pb, live, pul_high)) {
        a->target = base;                       /* the target, or the last integer a stop reached */
        a->mode = PULDIR_MODE_IDLE;
        a->moving = false;
        a->events |= PULDIR_EV_LANDED;
    }
}

/* Limits in user units to the generator, and to lim[] only if it takes them. */
void puldir_take_limits(PulDir* a, const float l[4])
{
    const float c = a->conv;
    if (MotionProfile_setLimits(&a->prof, (MotionLimits){ l[0] * c, l[1] * c, l[2] * c, l[3] * c }))
        for (int k = 0; k < 4; k++)
            a->lim[k] = l[k];
}

/* Leave STREAM for STOPPING (5.6): the idle generator is put in motion at the
 * stream's velocity through the boundary position, then stopped at its decel. */
static void leave_stream(PulDir* a, int64_t pb)
{
    /* The generator was halted by streamBegin and stays done: seed accepts. */
    (void)MotionProfile_seed(&a->prof, puldir_to_pulses(pb), a->v_cmd);
    MotionProfile_stop(&a->prof);
    a->stream_live = false;
    a->arrived = false;                         /* a new course (E17) */
    a->settle = 0;
}

/* STREAM: feedforward plus catch-up toward the first-order hold at t_b, where this set takes over.
 * pb is relative to count, so no large float is subtracted from another. */
static void run_stream(PulDir* a, PulDirInc* out, uint64_t base, int64_t pb, int32_t count,
                       uint32_t t_b)
{
    const float frac = (float)pb * (1.0f / PULDIR_FIX);
    if (a->sp_have) {
        /* sp_pos - P_b as (whole part - count) + (sp_pos's fraction) - frac: each
         * term is exact or small. PulDir_stream keeps |sp_pos| below 2^31. */
        const int32_t k = (int32_t)a->sp_pos;
        /* A setpoint stamped after the snapshot (a caller outside main context)
         * is taken as of now: its age is clamped at zero, never wrapped. */
        const int32_t ticks_old = (int32_t)(t_b - a->sp_tick);
        const float age = (float)(ticks_old > 0 ? ticks_old : 0) * (1.0f / (float)puldir_timing.tick_hz);
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

    const int32_t inc = div_trunc(to_fix(a->v_cmd * puldir_timing.dt));
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
    const float trel = puldir_to_pulses((int64_t)(a->target - base));
    MotionProfile_rebase(&a->prof, (float)shift, trel);

    /* 3. Requests, read after service: a latch's halt this tick is applied now (E17). Halt first, so
     * commands start where the pulses stopped; a stream begin before the limits, so they reach an
     * idle generator; a stop last. */
    uint16_t req = a->req;
    a->req = 0;
    if (req & REQ_HALT)                         /* puldir_kill zeroed the increments: pb is where they stopped */
        puldir_halt_generator(a, pb);
    if (req & REQ_STREAM_BEGIN) {
        /* The stream starts at the velocity the running set was aimed with; a stream ended and
         * begun within one interval keeps its own. The generator idles where the pulses are. */
        if (!a->stream_live)
            a->v_cmd = a->prof.vel;
        MotionProfile_halt(&a->prof);
        (void)MotionProfile_setPosition(&a->prof, puldir_to_pulses(pb));
        a->stream_live = true;
        a->arrived = false;                     /* the generator's goal is gone (E17) */
        a->settle = 0;
    }
    if (req & REQ_LIMITS) {
        const float c = a->conv;
        const MotionLimits L = { a->lim[0] * c, a->lim[1] * c, a->lim[2] * c, a->lim[3] * c };
        /* apply_limits checked them as the generator does. A homing stop's restore
         * can meet a window frozen in flight: it waits for the generator to rest. */
        if (!MotionProfile_setLimits(&a->prof, L))
            a->req |= REQ_LIMITS;
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
            (void)MotionProfile_setEndless(&a->prof, a->req_speed);
        a->arrived = false;                     /* so must every request that sets a new course */
        a->settle = 0;
    }
    if (req & REQ_STOP)
        MotionProfile_stop(&a->prof);
    if ((req & REQ_HOME) && a->mode == PULDIR_MODE_HOMING)
        puldir_home_begin(a, now);              /* homeStart: toward the switch */

    /* 4. The enable delay (5.8): a drive enabled too recently publishes nothing and its generator
     * waits. The homing phase clock starts when the hold ends. */
    if (a->enable.wait && a->mode != PULDIR_MODE_IDLE) {
        puldir_set_inc(a, out, 0);
        a->vel_out = 0.0f;
        a->home.t0 = now;
        return;
    }

    switch (a->mode) {
    case PULDIR_MODE_PROFILE:
    case PULDIR_MODE_STOPPING:
        run_profile(a, out, base, pb, live, pul_high);
        break;
    case PULDIR_MODE_STREAM:
        run_stream(a, out, base, pb, count, now + ahead);
        break;
    case PULDIR_MODE_HOMING:
        puldir_home_run(a, out, base, pb, count, live, pul_high, now);
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

/* ---- enable delay, fault and release (5.8), for every axis, group members too ---- */

/* Before the groups and axes run. A latch halts under the lock, the axis or its whole group, and
 * reads the snapshots of what it stopped again; run_axis then reads the halt's requests. */
static void service(PulDir* a, bool flt, uint32_t t_pin, uint32_t now, uint32_t ahead,
                    PulDirSnap* snap, PulDirInc* out)
{
    puldir_enable_service(a, snap[a->slot].high, now);
    if (!puldir_fault_service(a, flt, t_pin, now))
        return;

    /* Latch: halt, and leave the enable pin alone (D12). */
    puldir_home_fault(a);
    a->enable.held = true;
    PulDirGroup* g = a->group;
    port_lock();
    if (a->mode == PULDIR_MODE_GROUP && g != NULL) {
        puldir_group_halt(g, out);
        g->faulted = true;
        for (uint8_t i = 0; i < g->count; i++) {
            g->axes[i]->enable.held = true;
            puldir_pulse_read(g->axes[i]->slot, ahead, &snap[g->axes[i]->slot]);
        }
    } else {
        puldir_halt_locked(a);
        puldir_pulse_read(a->slot, ahead, &snap[a->slot]);
    }
    port_unlock();
}

void puldir_plan_tick(void)
{
    if (!puldir_timing.set)
        return;                                 /* started with no valid timing, so with no axis */
    const uint8_t n = puldir_naxes;

    /* The pins first. A pin read can be preempted by pulse ticks, and one that ran
     * between the snapshot and the publish would make the prediction stale. */
    bool flt[PULDIR_MAX_AXES];
    const uint32_t t_pin = PulDir_ticks();
    for (uint8_t i = 0; i < n; i++)
        flt[i] = puldir_fault_sample(puldir_axes[i]);

    PulDirSnap snap[PULDIR_MAX_AXES];
    port_lock();                                /* one consistent instant for every axis */
    /* Ticks the live increments still run before the next latch. At a boundary
     * (left == N) the set published below latches on the very next tick, so none. */
    const uint32_t ahead = puldir_left() % (uint32_t)N;
    const uint32_t now = PulDir_ticks();        /* the tick count the positions were read at */
    for (uint8_t i = 0; i < n; i++)
        puldir_pulse_read(i, ahead, &snap[i]);  /* D5 prediction */
    port_unlock();

    PulDirInc* out = puldir_shadow();
    bool was[PULDIR_MAX_AXES];
    for (uint8_t i = 0; i < n; i++) {
        was[i] = puldir_axes[i]->moving;
        service(puldir_axes[i], flt[i], t_pin, now, ahead, snap, out);
    }
    puldir_group_tick(snap, out);               /* the groups write their members' entries */
    for (uint8_t i = 0; i < n; i++) {
        PulDir* a = puldir_axes[i];
        if (a->mode != PULDIR_MODE_GROUP)
            run_axis(a, &out[i], snap[i].Pb, snap[i].count, snap[i].inc, snap[i].high, now, ahead);
        puldir_enable_release(a, was[i], now);
    }
    puldir_publish(now + ahead);                /* due at the boundary the snapshot aimed at: the tick after the ahead ticks */
}
