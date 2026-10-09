/* Stream mode (5.6): the setpoint calls from main, the first-order hold from the plan tick. */
#include <math.h>
#include "puldir_internal.h"

#define TIMEOUT_TICKS puldir_ms_ticks(PULDIR_STREAM_TIMEOUT_MS)

/* From IDLE, PROFILE or STOPPING; a pending move or stop gives way. The plan tick takes the axis's
 * velocity as the stream's and starts the timeout, so the call itself changes no motion. */
bool PulDir_streamBegin(PulDir* a)
{
    port_lock();
    const PulDirRefusal why = puldir_motion_check(a);
    if (why == PULDIR_R_NONE) {
        a->req = (uint16_t)((a->req & ~REQ_MOTION) | REQ_STREAM_BEGIN);
        a->mode = PULDIR_MODE_STREAM;
        a->moving = true;
        puldir_enable_wake(a);
        a->stream.timed_out = false;
        a->stream.err = 0.0f;
        a->stream.sp_tick = PulDir_ticks(); /* the timeout runs from here until a setpoint */
        a->stream.have = false;
    }
    port_unlock();
    if (why != PULDIR_R_NONE) {
        puldir_refuse(a, "streamBegin", why, 0.0f);
        return false;
    }
    return true;
}

/* Main context only (5.9): the plan tick relies on sp_tick never being ahead of its reading. Past
 * 2^24 pulses from the datum the float setpoint is quantised before the library sees it. */
void PulDir_stream(PulDir* a, float position, float velocity)
{
    const float p = position * a->pulses_per_unit, v = velocity * a->pulses_per_unit;
    bool ok = isfinite(v) && p >= -PULDIR_PULSES_MAX && p < PULDIR_PULSES_MAX;   /* false for NaN */
    if (ok) {
        port_lock();
        ok = a->mode == PULDIR_MODE_STREAM;
        if (ok) {
            a->stream.sp_pos = p;
            a->stream.sp_vel = v;
            a->stream.sp_tick = PulDir_ticks();
            a->stream.have = true;
        }
        port_unlock();
    }
    if (!ok)
        puldir_refuse(a, "stream", PULDIR_R_DROPPED, 0.0f);
}

/* In STREAM only: the defaults first, then a stop from the stream's velocity
 * under them (the plan tick seeds the idle generator and stops it). */
void PulDir_streamEnd(PulDir* a)
{
    port_lock();
    if (a->mode == PULDIR_MODE_STREAM) {
        a->settings = a->defaults;            /* validated when set, at the one conversion */
        a->req |= REQ_SETTINGS;               /* the plan tick sees STOPPING and leaves */
        a->mode = PULDIR_MODE_STOPPING;
    }
    port_unlock();
}

bool  PulDir_inStream(PulDir* a)       { return a->mode == PULDIR_MODE_STREAM; }
bool  PulDir_streamTimedOut(PulDir* a) { return a->stream.timed_out; }
float PulDir_streamError(PulDir* a)    { return a->stream.err / a->pulses_per_unit; }

/* D11: at least five plan intervals (5.6), and not while streaming. */
bool PulDir_setStreamCatchup(PulDir* a, uint32_t ms)
{
    PulDirRefusal why = PULDIR_R_NONE;
    if ((uint64_t)ms * 1000u < 5u * (uint64_t)puldir_timing.plan_us) {
        why = PULDIR_R_ARG;
    } else {
        port_lock();
        if (a->mode == PULDIR_MODE_STREAM)
            why = PULDIR_R_MODE;
        else
            a->stream.catchup_ms = ms;
        port_unlock();
    }
    if (why != PULDIR_R_NONE) {
        puldir_refuse(a, "setStreamCatchup", why, 0.0f);
        return false;
    }
    return true;
}

void puldir_stream_defaults(PulDir* a)
{
    a->stream.catchup_ms = PULDIR_CATCHUP_MS;
}

/* ---- the plan tick's part ----------------------------------------------------------------- */

void puldir_stream_halt(PulDir* a)
{
    a->stream.live = false;
}

void puldir_stream_begin(PulDir* a, int64_t pb)
{
    /* The stream starts at the velocity the running set was aimed with; a stream ended and
     * begun within one interval keeps its own. */
    if (!a->stream.live)
        a->stream.v_cmd = a->profile.vel;
    MotionProfile_halt(&a->profile);
    (void)MotionProfile_setPosition(&a->profile, puldir_to_pulses(pb));
    a->stream.live = true;
    a->arrived = false;                         /* the generator's goal is gone (E17) */
    a->settle = 0;
}

/* Leave STREAM for STOPPING (5.6): the idle generator is put in motion at the
 * stream's velocity through the boundary position, then stopped at its decel. */
static void leave_stream(PulDir* a, int64_t pb)
{
    /* The generator was halted by streamBegin and stays done: seed accepts. */
    (void)MotionProfile_seed(&a->profile, puldir_to_pulses(pb), a->stream.v_cmd);
    MotionProfile_stop(&a->profile);
    a->stream.live = false;
    a->arrived = false;                         /* a new course (E17) */
    a->settle = 0;
}

/* A move accepted in STOPPING after the stream left starts from the seeded motion. */
void puldir_stream_check(PulDir* a, int64_t pb, uint32_t now)
{
    if (a->stream.live && a->mode != PULDIR_MODE_STREAM) {
        leave_stream(a, pb);
    } else if (a->mode == PULDIR_MODE_STREAM
               && (int32_t)(now - a->stream.sp_tick) >= (int32_t)TIMEOUT_TICKS) {
        a->mode = PULDIR_MODE_STOPPING;         /* at the axis's decel, without the defaults */
        a->stream.timed_out = true;
        a->events |= PULDIR_EV_STREAM_TIMEOUT;
        leave_stream(a, pb);
    }
}

/* Feedforward plus catch-up toward the first-order hold at t_b. pb is relative to count, so no
 * large float is subtracted from another. */
void puldir_stream_run(PulDir* a, PulDirInc* out, uint64_t base, int64_t pb, int32_t count, uint32_t t_b)
{
    PulDirStream* s = &a->stream;
    const float frac = (float)pb * (1.0f / PULDIR_FIX);
    if (s->have) {
        /* sp_pos - P_b as (whole part - count) + (sp_pos's fraction) - frac: each
         * term is exact or small. PulDir_stream keeps |sp_pos| below 2^31. */
        const int32_t k = (int32_t)s->sp_pos;
        /* A setpoint stamped after the snapshot (a caller outside main context)
         * is taken as of now: its age is clamped at zero, never wrapped. */
        const int32_t ticks_old = (int32_t)(t_b - s->sp_tick);
        const float age = (float)(ticks_old > 0 ? ticks_old : 0) * (1.0f / (float)puldir_timing.tick_hz);
        const float err = (float)((int64_t)k - (int64_t)count) + (s->sp_pos - (float)k)
                        + s->sp_vel * age - frac;
        s->err = err;
        s->v_cmd = s->sp_vel + err * (1000.0f / (float)s->catchup_ms);
    }
    /* Specified behaviour (5.6), not input clamping: in STREAM the speed setting
     * bounds v_cmd, and it lies below the tick ceiling. */
    const float top = a->settings.speed * a->pulses_per_unit;
    if (s->v_cmd > top)
        s->v_cmd = top;
    else if (s->v_cmd < -top)
        s->v_cmd = -top;

    const int32_t inc = puldir_div_trunc(puldir_to_fix(s->v_cmd * puldir_timing.dt));
    puldir_set_inc(a, out, inc);
    a->velocity = s->v_cmd;
    a->target = base + (uint64_t)(pb + (int64_t)inc * PULDIR_TICKS_PER_PLAN);   /* the aim, as a jog reads it */
    (void)MotionProfile_setPosition(&a->profile, frac);        /* the idle generator follows the pulses */
}
