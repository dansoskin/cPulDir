/* The plan tick (guidelines 5.4): prediction, re-base, modes, publication.
 * Float lives here; the pulse tick sees only the published integers. */
#include <math.h>
#include "puldir_internal.h"

#define N   ((int64_t)PULDIR_TICKS_PER_PLAN)
#define M   (N * N)                             /* D6 margin, in 2^-32 pulse */
#define FIX 4294967296.0f                       /* one pulse in 32.32 */
#define SAT ((int64_t)1 << 56)                  /* 2^24 pulses (D2) */
#define DT  (1.0f / (float)PULDIR_PLAN_HZ)      /* the profile's dt, the same expression as init's */
#define TIMEOUT_TICKS  PULDIR_MS_TICKS(PULDIR_STREAM_TIMEOUT_MS)
#define DEBOUNCE_TICKS PULDIR_MS_TICKS(PULDIR_FAULT_DEBOUNCE_MS)
#define CLEAR_TICKS    PULDIR_MS_TICKS(PULDIR_FAULT_CLEAR_MS)
#define DELAY_TICKS    PULDIR_MS_TICKS(PULDIR_HOMING_DELAY_MS)

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

/* The generator stops dead where the pulses are (pb, relative to the origin): a
 * new goal (E17), and a stream's idle generator stays put. */
static void halt_generator(PulDir* a, int64_t pb)
{
    MotionProfile_halt(&a->prof);
    (void)MotionProfile_setPosition(&a->prof, to_pulses(pb));
    a->arrived = false;
    a->settle = 0;
    a->stream_live = false;
}

/* PulDir_halt from inside the plan tick, after the re-base (E17): no rising edge
 * after this tick, the target where the pulses stopped, IDLE. isMoving clears in
 * IDLE once the pin is low. */
static void halt_now(PulDir* a, PulDirInc* out, uint64_t base)
{
    PulDirPulse s;
    port_lock();
    puldir_kill(a->slot);
    puldir_pulse_read(a->slot, &s);
    port_unlock();
    a->target = (uint64_t)(uint32_t)s.count << 32;
    a->mode = PULDIR_MODE_IDLE;
    a->vel_out = 0.0f;
    halt_generator(a, (int64_t)(s.P - base));
    puldir_set_inc(a, out, 0);
}

/* Aim this interval's end at the generator's position. T, pb and end are 32.32
 * relative to base; live is the running increment. Returns true at rest: settled
 * on the generator's final position, the generator done, the last pulse lowered. */
static bool run_generator(PulDir* a, PulDirInc* out, uint64_t base, int64_t pb, int32_t live,
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
    if (run_generator(a, out, base, pb, live, pul_high)) {
        a->target = base;                       /* the target, or the last integer a stop reached */
        a->mode = PULDIR_MODE_IDLE;
        a->moving = false;
        a->events |= PULDIR_EV_LANDED;
    }
}

/* ---- homing (5.8): the phases of cFlexyStepper's machine, from the plan tick ---- */

static void home_fail(PulDir* a, PulDirInc* out, uint64_t base)
{
    a->home_fail = a->home_state;
    a->home_state = PULDIR_HOME_ERROR;
    a->events |= PULDIR_EV_HOME_FAILED;
    halt_now(a, out, base);
}

/* A new goal for the generator (E17). */
static void home_course(PulDir* a)
{
    a->arrived = false;
    a->settle = 0;
}

/* DELAY3: the datum, as PulDir_setPosition sets it. The axis must be at rest:
 * no increment and the pin low, read again under the lock (puldir_set_datum). */
static bool home_datum(PulDir* a, PulDirInc* out)
{
    const int32_t c = (int32_t)roundf(a->home_zero * a->conv);   /* checked by homeStart */
    PulDirPulse s;
    port_lock();
    puldir_pulse_read(a->slot, &s);
    const bool rest = s.inc == 0 && !puldir_pul_high(a->slot);
    if (rest)
        puldir_set_datum(a->slot, c);
    port_unlock();
    if (!rest)
        return false;
    a->origin = c;
    a->target = (uint64_t)(uint32_t)c << 32;
    (void)MotionProfile_setPosition(&a->prof, 0.0f);   /* done: ADJUST ended at rest */
    a->homed = true;
    a->home_state = PULDIR_HOME_IDLE;
    a->mode = PULDIR_MODE_IDLE;
    a->moving = false;
    a->vel_out = 0.0f;
    a->events |= PULDIR_EV_HOMED;
    puldir_set_inc(a, out, 0);
    return true;
}

/* One plan interval of HOMING. The switch and the clocks are read before the
 * generator advances, so a switch seen now stops the aim published now. Every
 * phase that moves has home_timeout_ms; a timeout halts the axis (2.2). */
static void run_home(PulDir* a, PulDirInc* out, uint64_t base, int64_t pb, int32_t count,
                     int32_t live, bool pul_high, uint32_t now)
{
    MotionProfile* p = &a->prof;
    const uint8_t sw = *(const volatile uint8_t*)a->home_switch;
    const uint32_t in_phase = now - a->home_t0;
    const bool late = in_phase >= a->home_timeout_ticks;
    const float v = (float)a->home_dir * a->home_speed * a->conv;
    PulDirHomeState next = a->home_state;
    switch (a->home_state) {
    case PULDIR_HOME_MOVE_TOWARDS_LIMIT:
    case PULDIR_HOME_MOVE_AWAY_FROM_LIMIT: {
        const bool towards = a->home_state == PULDIR_HOME_MOVE_TOWARDS_LIMIT;
        if (sw == (towards ? 1u : 0u)) {
            MotionProfile_halt(p);              /* stops on the aim published last interval */
            home_course(a);
            next = towards ? PULDIR_HOME_DELAY1 : PULDIR_HOME_DELAY2;
        } else if (late) {
            home_fail(a, out, base);
            return;
        }
        break;
    }
    case PULDIR_HOME_DELAY1:
        if (in_phase >= DELAY_TICKS) {
            (void)MotionProfile_setEndless(p, -0.2f * v);   /* finite: checked by homeStart */
            home_course(a);
            next = PULDIR_HOME_MOVE_AWAY_FROM_LIMIT;
        }
        break;
    case PULDIR_HOME_DELAY2:
        if (in_phase >= DELAY_TICKS) {
            const int64_t t = (int64_t)count + (int64_t)roundf(a->home_adjust * a->conv);
            if (t < INT32_MIN || t > INT32_MAX) {
                home_fail(a, out, base);
                return;
            }
            /* The defaults, validated when set and fixed in pulses since; the
             * generator is done (halted a delay ago), so it takes them. */
            for (int k = 0; k < 4; k++)
                a->lim[k] = a->def[k];
            const float c = a->conv;
            (void)MotionProfile_setLimits(p, (MotionLimits){ a->def[0] * c, a->def[1] * c,
                                                             a->def[2] * c, a->def[3] * c });
            a->target = (uint64_t)t << 32;
            (void)MotionProfile_setTarget(p, to_pulses((int64_t)(a->target - base)));
            home_course(a);
            next = PULDIR_HOME_ADJUST_POSITION;
        }
        break;
    case PULDIR_HOME_ADJUST_POSITION:
        if (late) {
            home_fail(a, out, base);
            return;
        }
        break;
    case PULDIR_HOME_DELAY3:
        if (in_phase >= DELAY_TICKS && !pul_high && live == 0 && home_datum(a, out))
            return;
        break;
    default:
        break;
    }
    if (next != a->home_state) {
        a->home_state = next;
        a->home_t0 = now;
    }
    /* The adjust ends through the usual arrival path (E13, D19). */
    if (run_generator(a, out, base, pb, live, pul_high) && a->home_state == PULDIR_HOME_ADJUST_POSITION) {
        a->home_state = PULDIR_HOME_DELAY3;
        a->home_t0 = now;
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
                     bool pul_high, uint32_t now, uint32_t ahead, bool latch)
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
     * begins, so limits reach an idle generator; a stop last. A fault latched
     * this tick halts here and now (E17) and discards every course requested. */
    uint16_t req = a->req;
    a->req = 0;
    if (latch) {
        req &= (uint16_t)~(REQ_COURSE | REQ_HALT);
        halt_now(a, out, base);
    }
    if (req & REQ_HALT)                         /* puldir_kill zeroed the increments: pb is where they stopped */
        halt_generator(a, pb);
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
    if ((req & REQ_HOME) && a->mode == PULDIR_MODE_HOMING) {
        /* homeStart: run toward the switch; finite, homeStart checked the speed */
        (void)MotionProfile_setEndless(&a->prof, (float)a->home_dir * a->home_speed * a->conv);
        a->arrived = false;                     /* E17 */
        a->settle = 0;
        a->home_state = PULDIR_HOME_MOVE_TOWARDS_LIMIT;
        a->home_t0 = now;
    }

    /* 4. The enable delay (5.8): a moving mode on a drive enabled less than
     * enable_ms ago publishes nothing and leaves the generator where it is.
     * The homing phase clock starts when the hold ends. */
    if (a->ena_wait && a->mode != PULDIR_MODE_IDLE) {
        puldir_set_inc(a, out, 0);
        a->vel_out = 0.0f;
        a->home_t0 = now;
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
        run_home(a, out, base, pb, count, live, pul_high, now);
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

/* ---- fault, enable delay and release (5.8), for every axis, group members too ---- */

/* Before the groups and the axes run. flt is the fault input sampled at t_pin.
 * Returns true when the latch closes on an axis that is not in a group's move;
 * run_axis halts it after its re-base. A member's latch halts its group here,
 * under the lock, and refreshes every member's snapshot. */
static bool service(PulDir* a, bool flt, uint32_t t_pin, uint32_t now, uint32_t ahead,
                    PulDirSnap* snap, PulDirInc* out)
{
    if (a->ena_wait && now - a->enabled_at >= a->enable_ticks)
        a->ena_wait = false;

    /* The clear pulse ends: the latch lifts, and the debounce restarts from the
     * first sample taken after the pin is lowered, not this one, taken before it. */
    if (a->clearing && now - a->clr_t0 >= CLEAR_TICKS) {
        port_pin_write(a->clr, a->clr_inverted);
        a->clearing = false;
        a->fault = false;
        a->flt_seen = false;
        a->events |= PULDIR_EV_FAULT_CLEARED;
        return false;
    }

    if (!flt) {
        a->flt_seen = false;
        return false;
    }
    if (!a->flt_seen) {
        a->flt_seen = true;
        a->flt_since = t_pin;
        return false;
    }
    if (a->fault || t_pin - a->flt_since < DEBOUNCE_TICKS)
        return false;

    /* Latch: halt, and leave the enable pin alone (D12). */
    a->fault = true;
    a->fault_count++;
    a->last_fault_tick = t_pin;
    a->events |= PULDIR_EV_FAULT;
    a->homed = false;
    if (a->mode == PULDIR_MODE_HOMING) {
        a->home_fail = a->home_state;
        a->home_state = PULDIR_HOME_ERROR;
        a->events |= PULDIR_EV_HOME_FAILED;
    }
    PulDirGroup* g = a->group;
    if (a->mode != PULDIR_MODE_GROUP || g == NULL)
        return true;
    port_lock();
    puldir_group_halt(g, out);
    g->faulted = true;
    for (uint8_t i = 0; i < g->count; i++) {    /* the members' pulses stopped just now */
        const uint8_t k = g->axes[i]->slot;
        PulDirPulse s;
        puldir_pulse_read(k, &s);
        snap[k].Pb = s.P + (uint64_t)((int64_t)s.inc * ahead);
        snap[k].count = s.count;
        snap[k].live = s.inc;
        snap[k].dir = s.dir;
        snap[k].high = puldir_pul_high(k);
    }
    port_unlock();
    return false;
}

/* After the modes: with auto-release on, the drive is disabled release_ms after
 * the axis went IDLE at the end of a move, which is within one plan interval of
 * the last falling edge (D19). Never after a fault latch (D12). */
static void release(PulDir* a, bool was_moving, uint32_t now)
{
    if (was_moving && !a->moving && a->mode == PULDIR_MODE_IDLE && a->auto_release
        && a->has_ena && a->enabled) {
        a->release_due = true;
        a->idle_at = now;
    }
    if (!a->release_due)
        return;
    if (a->moving || a->mode != PULDIR_MODE_IDLE || a->fault) {
        a->release_due = false;
    } else if (now - a->idle_at >= a->release_ticks) {
        port_pin_write(a->ena, a->ena_inverted);
        a->enabled = false;
        a->ena_wait = false;
        a->release_due = false;
    }
}

void puldir_plan_tick(void)
{
    PulDirPulse s[PULDIR_MAX_AXES];
    bool high[PULDIR_MAX_AXES];
    const uint8_t n = puldir_naxes;

    /* The pins first. A pin read can be preempted by pulse ticks, and one that ran
     * between the snapshot and the publish would make the prediction stale. */
    bool flt[PULDIR_MAX_AXES];
    const uint32_t t_pin = PulDir_ticks();
    for (uint8_t i = 0; i < n; i++) {
        const PulDir* a = puldir_axes[i];
        flt[i] = a->has_flt && port_pin_read(a->flt) != a->flt_inverted;
    }

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
    bool latch[PULDIR_MAX_AXES], was[PULDIR_MAX_AXES];
    for (uint8_t i = 0; i < n; i++) {
        was[i] = puldir_axes[i]->moving;
        latch[i] = service(puldir_axes[i], flt[i], t_pin, now, ahead, snap, out);
    }
    puldir_group_tick(snap, out);               /* the groups write their members' entries */
    for (uint8_t i = 0; i < n; i++) {
        PulDir* a = puldir_axes[i];
        if (a->mode != PULDIR_MODE_GROUP)
            run_axis(a, &out[i], snap[i].Pb, snap[i].count, snap[i].live, snap[i].high, now, ahead,
                     latch[i]);
        release(a, was[i], now);
    }
    puldir_publish();
}
