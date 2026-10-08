/* Main context: setup, commands, getters. */
#include <math.h>
#include <string.h>
#include "puldir_internal.h"

static bool started;                    /* PulDir_start has run */

bool PulDir_init(PulDir* a, const char* name)
{
    size_t len = 0;
    while (name != NULL && len < sizeof a->name && name[len] != '\0')
        len++;
    if (name == NULL || len == sizeof a->name)
        return false;                   /* nothing to log through: the struct is not ours yet */
    memset(a, 0, sizeof *a);
    memcpy(a->name, name, len);
    a->conv = 1.0f;
    const float d[4] = { 200.0f, 200.0f, 200.0f, 0.0f };
    for (int k = 0; k < 4; k++)
        a->def[k] = a->lim[k] = d[k];
    a->catchup_ms = PULDIR_CATCHUP_MS;
    a->log_on = true;
    a->slot = PULDIR_MAX_AXES;
    a->dir_out = 1;                     /* the pulse tick's entries start at +1 */
    a->mode = PULDIR_MODE_IDLE;
    a->home_state = PULDIR_HOME_IDLE;
    a->refusal = PULDIR_R_NONE;
    /* The plan interval is a constant inside MotionProfile_init's range and
     * the limits are finite and positive, so neither call can refuse. */
    (void)MotionProfile_init(&a->prof, 0.0f, PULDIR_DT);
    (void)MotionProfile_setLimits(&a->prof, (MotionLimits){ d[0], d[1], d[2], d[3] });
    return true;
}

void PulDir_attachPins(PulDir* a, puldir_pin_t pulse, puldir_pin_t dir,
                       bool pulse_active_low, bool dir_inverted)
{
    a->pul = pulse;
    a->dir = dir;
    a->pul_active_low = pulse_active_low;
    a->dir_inverted = dir_inverted;
    a->has_pins = true;
    port_pin_init_output(pulse, pulse_active_low);        /* inactive */
    port_pin_init_output(dir, !dir_inverted);             /* the level for +1 */
}

/* The enable, fault and clear pins: setup, before PulDir_start. The outputs are
 * parked inactive; the drive is disabled until a command or PulDir_enable. */
void PulDir_attachEnable(PulDir* a, puldir_pin_t p, bool inverted)
{
    a->ena = p;
    a->ena_inverted = inverted;
    a->has_ena = true;
    a->enabled = false;
    port_pin_init_output(p, inverted);
}

void PulDir_attachFault(PulDir* a, puldir_pin_t p, bool inverted)
{
    a->flt = p;
    a->flt_inverted = inverted;
    a->has_flt = true;
    port_pin_init_input(p, false);
}

void PulDir_attachFaultClear(PulDir* a, puldir_pin_t p, bool inverted)
{
    a->clr = p;
    a->clr_inverted = inverted;
    a->has_clr = true;
    port_pin_init_output(p, inverted);
}

/* A duration the plan tick can compare by unsigned subtraction. */
static bool ms_fits(uint32_t ms)
{
    return (uint64_t)ms * PULDIR_TICK_HZ / 1000u <= PULDIR_TICKS_MAX;
}

bool PulDir_setDelays(PulDir* a, uint32_t enable_ms, uint32_t release_ms)
{
    if (!ms_fits(enable_ms) || !ms_fits(release_ms)) {
        puldir_refuse(a, "setDelays", PULDIR_R_ARG, 0.0f);
        return false;
    }
    port_lock();
    a->enable_ticks = PULDIR_MS_TICKS(enable_ms);
    a->release_ticks = PULDIR_MS_TICKS(release_ms);
    port_unlock();
    return true;
}

void PulDir_setAutoRelease(PulDir* a, bool on)
{
    port_lock();
    a->auto_release = on;
    if (!on)
        a->release_due = false;
    port_unlock();
}

void puldir_wake(PulDir* a)
{
    a->release_due = false;
    a->held = false;
    a->ena_off = false;
    if (a->has_ena && !a->enabled) {
        port_pin_write(a->ena, !a->ena_inverted);
        a->enabled = true;
        a->enabled_at = PulDir_ticks();
        a->ena_wait = true;                 /* the plan tick lifts it once the enable delay has passed */
    }
}

bool PulDir_register(PulDir* a)
{
    PulDirRefusal why = PULDIR_R_NONE;
    if (!a->has_pins)
        why = PULDIR_R_NO_PIN;
    else if (a->slot != PULDIR_MAX_AXES || started)
        why = PULDIR_R_BUSY;            /* already registered, or the ticks are running */
    else if (puldir_naxes >= PULDIR_MAX_AXES)
        why = PULDIR_R_FULL;
    if (why != PULDIR_R_NONE) {
        puldir_refuse(a, "register", why, 0.0f);
        return false;
    }
    puldir_pulse_add(a);
    return true;
}

void PulDir_start(void)
{
    started = true;
    port_start(puldir_pulse_tick, puldir_plan_tick);
}

/* ---- limits ---------------------------------------------------------------- */

/* The generator's window() test, on the same float products as apply_limits. */
static bool taps_fit(float acc, float dec, float j)
{
    return ceilf((acc + dec) / (j * PULDIR_DT)) <= (float)MOTION_PROFILE_MAX_TAPS;
}

/* The smallest jerk in user units that fits: minJerk's quotient and the
 * conversion each round, so step to the exact float boundary. */
static float jerk_floor(float acc, float dec, float conv)
{
    float j = MotionProfile_minJerk(acc, dec, PULDIR_DT) / conv;
    while (!taps_fit(acc, dec, j * conv))
        j = nextafterf(j, INFINITY);
    for (float d = nextafterf(j, 0.0f); taps_fit(acc, dec, d * conv); d = nextafterf(d, 0.0f))
        j = d;
    return j;
}

/* GROUP: the path profile owns the limits until the move ends (5.7). HOMING: the
 * run sets them from the plan tick. */
static bool limits_busy(PulDirMode m)
{
    return m == PULDIR_MODE_GROUP || m == PULDIR_MODE_HOMING;
}

/* Checks four limits (user units) as MotionProfile_setLimits will see them, then
 * stores them, and def too, under the lock that reads the frozen window (D17). */
static bool apply_limits(PulDir* a, const float lim[4], bool defaults, const char* call)
{
    const float conv = a->conv;
    const float v = lim[0] * conv, acc = lim[1] * conv, dec = lim[2] * conv, j = lim[3] * conv;
    const float ratio = j > 0.0f ? (acc + dec) / (j * PULDIR_DT) : 0.0f;    /* generator's window() */
    PulDirRefusal why = PULDIR_R_NONE;
    float value = 0.0f;
    if (!(isfinite(v) && isfinite(acc) && isfinite(dec) && isfinite(j))
        || !(v > 0.0f && acc > 0.0f && dec > 0.0f && j >= 0.0f)) {
        why = PULDIR_R_ARG;
    } else if (!(lim[0] < (float)(PULDIR_TICK_HZ / 2u) / conv)) {
        why = PULDIR_R_RATE;
        value = (float)(PULDIR_TICK_HZ / 2u) / conv;
    } else if (j > 0.0f && !taps_fit(acc, dec, j)) {
        why = PULDIR_R_TAPS;
        value = jerk_floor(acc, dec, conv);
    } else {
        port_lock();
        if (limits_busy(a->mode)) {
            why = PULDIR_R_MODE;
        } else if (a->prof.done || j == 0.0f || ratio <= (float)a->prof.n) {
            memcpy(a->lim, lim, sizeof a->lim);
            if (defaults)
                memcpy(a->def, lim, sizeof a->def);
            a->req |= REQ_LIMITS;
        } else {
            why = PULDIR_R_WINDOW;          /* the window is frozen in flight (5.2) */
        }
        port_unlock();
    }
    if (why != PULDIR_R_NONE) {
        puldir_refuse(a, call, why, value);
        return false;
    }
    return true;
}

/* The copy and the mode at one instant: a copy taken in HOMING could be stale
 * by the time the run ends, so it is refused here, not by apply_limits. */
static bool set_limit(PulDir* a, int k, float x, const char* call)
{
    float l[4];
    port_lock();
    memcpy(l, a->lim, sizeof l);
    const bool busy = limits_busy(a->mode);
    port_unlock();
    if (busy) {
        puldir_refuse(a, call, PULDIR_R_MODE, 0.0f);
        return false;
    }
    l[k] = x;
    return apply_limits(a, l, false, call);
}

/* Limits and defaults stay fixed in pulses, as in cFlexyStepper: the user-unit
 * values are rescaled, so the generator's limits are untouched. */
bool PulDir_setConversion(PulDir* a, float pulses_per_unit)
{
    const float c = pulses_per_unit;
    PulDirRefusal why = PULDIR_R_NONE;
    float lim[4], def[4];
    port_lock();                                /* the plan tick writes lim, but never in IDLE */
    const bool idle = a->mode == PULDIR_MODE_IDLE && !a->moving;
    memcpy(lim, a->lim, sizeof lim);
    memcpy(def, a->def, sizeof def);
    port_unlock();
    if (!(isfinite(c) && c > 0.0f)) {
        why = PULDIR_R_ARG;
    } else if (!idle) {
        why = PULDIR_R_NOT_IDLE;
    } else {
        for (int k = 0; k < 4; k++) {
            lim[k] = lim[k] * a->conv / c;
            def[k] = def[k] * a->conv / c;
        }
        port_lock();                            /* only main context leaves IDLE */
        a->conv = c;
        memcpy(a->lim, lim, sizeof lim);
        memcpy(a->def, def, sizeof def);
        port_unlock();
    }
    if (why != PULDIR_R_NONE) {
        puldir_refuse(a, "setConversion", why, 0.0f);
        return false;
    }
    return true;
}

bool PulDir_setDefaults(PulDir* a, float speed, float accel, float decel, float jerk)
{
    const float d[4] = { speed, accel, decel, jerk };
    return apply_limits(a, d, true, "setDefaults");
}

bool PulDir_restoreDefaults(PulDir* a)
{
    float d[4];
    port_lock();
    memcpy(d, a->def, sizeof d);
    port_unlock();
    return apply_limits(a, d, false, "restoreDefaults");
}

bool  PulDir_setSpeed(PulDir* a, float x)        { return set_limit(a, 0, x, "setSpeed"); }
bool  PulDir_setAcceleration(PulDir* a, float x) { return set_limit(a, 1, x, "setAcceleration"); }
bool  PulDir_setDeceleration(PulDir* a, float x) { return set_limit(a, 2, x, "setDeceleration"); }
bool  PulDir_setJerk(PulDir* a, float x)         { return set_limit(a, 3, x, "setJerk"); }
float PulDir_speed(PulDir* a)                    { return a->lim[0]; }
float PulDir_acceleration(PulDir* a)             { return a->lim[1]; }
float PulDir_deceleration(PulDir* a)             { return a->lim[2]; }
float PulDir_jerk(PulDir* a)                     { return a->lim[3]; }

/* ---- motion ---------------------------------------------------------------- */

static bool motion_refused(PulDir* a, const char* call)
{
    const PulDirMode m = a->mode;
    PulDirRefusal why = PULDIR_R_NONE;
    if (a->fault)
        why = PULDIR_R_FAULT;
    else if (m == PULDIR_MODE_STREAM || m == PULDIR_MODE_GROUP || m == PULDIR_MODE_HOMING)
        why = PULDIR_R_MODE;
    if (why == PULDIR_R_NONE)
        return false;
    puldir_refuse(a, call, why, 0.0f);
    return true;
}

/* Under the lock: what the caller may read back at once is set here (D17). The
 * fault latch is read again: the plan tick may have latched since the check. */
static PulDirRefusal take_target(PulDir* a, uint64_t target)
{
    if (a->fault)
        return PULDIR_R_FAULT;
    a->target = target;
    a->req = (uint16_t)((a->req & ~REQ_MOTION) | REQ_TARGET);
    a->mode = PULDIR_MODE_PROFILE;
    a->moving = true;
    puldir_wake(a);
    return PULDIR_R_NONE;
}

/* Targets are whole pulses (D4). */
bool PulDir_moveTo(PulDir* a, float position)
{
    if (motion_refused(a, "moveTo"))
        return false;
    const float x = roundf(position * a->conv);
    PulDirRefusal why = PULDIR_R_ARG;
    if (x >= -PULDIR_PULSES_MAX && x < PULDIR_PULSES_MAX) {
        port_lock();
        why = take_target(a, (uint64_t)(int64_t)x << 32);
        port_unlock();
    }
    if (why != PULDIR_R_NONE) {
        puldir_refuse(a, "moveTo", why, 0.0f);
        return false;
    }
    return true;
}

bool PulDir_moveBy(PulDir* a, float distance)
{
    if (motion_refused(a, "moveBy"))
        return false;
    const float dx = roundf(distance * a->conv);
    PulDirRefusal why = PULDIR_R_ARG;
    if (dx >= -2.0f * PULDIR_PULSES_MAX && dx <= 2.0f * PULDIR_PULSES_MAX) {   /* finite, and the sum fits */
        port_lock();
        const int64_t whole = (int32_t)(uint32_t)((a->target + (PULDIR_ONE >> 1)) >> 32);
        const int64_t t = whole + (int64_t)dx;
        if (t >= INT32_MIN && t <= INT32_MAX)
            why = take_target(a, (uint64_t)t << 32);
        port_unlock();
    }
    if (why != PULDIR_R_NONE) {
        puldir_refuse(a, "moveBy", why, 0.0f);
        return false;
    }
    return true;
}

/* D7: run at speed until told otherwise; the count carries on (2.2). jog(0) is a stop, refused
 * wherever any jog is (5.5), so a jog never ends a STREAM. */
bool PulDir_jog(PulDir* a, float speed)
{
    if (motion_refused(a, "jog"))
        return false;
    if (speed == 0.0f) {
        PulDir_stop(a);
        return true;
    }
    const float cap = (float)(PULDIR_TICK_HZ / 2u) / a->conv;
    PulDirRefusal why = PULDIR_R_NONE;
    if (!isfinite(speed))
        why = PULDIR_R_ARG;
    else if (!(fabsf(speed) < cap))
        why = PULDIR_R_RATE;
    if (why != PULDIR_R_NONE) {
        puldir_refuse(a, "jog", why, why == PULDIR_R_RATE ? cap : 0.0f);
        return false;
    }
    const float v = speed * a->conv;
    port_lock();
    if (a->fault) {
        why = PULDIR_R_FAULT;
    } else {
        a->req_speed = v;
        a->req = (uint16_t)((a->req & ~REQ_MOTION) | REQ_ENDLESS);
        a->mode = PULDIR_MODE_PROFILE;
        a->moving = true;
        puldir_wake(a);
    }
    port_unlock();
    if (why != PULDIR_R_NONE) {
        puldir_refuse(a, "jog", why, 0.0f);
        return false;
    }
    return true;
}

/* Under the lock, a homing run ended from main, unhomed: the limits homeStart
 * found go back. The plan tick hands them to the generator once its window allows. */
static void home_unwind(PulDir* a)
{
    a->home_state = PULDIR_HOME_IDLE;
    memcpy(a->lim, a->home_lim, sizeof a->lim);
    a->req |= REQ_LIMITS;
}

/* Ramped from the motion as commanded: a pending target or jog applies first. A group member
 * refuses it (5.7); in HOMING it ends the run unhomed and stops at the axis's decel. */
void PulDir_stop(PulDir* a)
{
    port_lock();
    const bool group = a->mode == PULDIR_MODE_GROUP;
    if (!group && a->mode != PULDIR_MODE_IDLE) {
        if (a->mode == PULDIR_MODE_HOMING) {
            home_unwind(a);
            a->req &= (uint16_t)~REQ_HOME;  /* not yet begun: there is nothing to stop */
        }
        a->req |= REQ_STOP;
        a->mode = PULDIR_MODE_STOPPING;
    }
    port_unlock();
    if (group)
        puldir_refuse(a, "stop", PULDIR_R_MODE, 0.0f);
}

/* Immediate (D17): no rising edge after this tick, the target where the pulses stopped. A member
 * halts its whole group (5.5); a homing run ends. isMoving clears once the pin is low. */
void PulDir_halt(PulDir* a)
{
    port_lock();
    if (a->mode == PULDIR_MODE_GROUP && a->group != NULL) {
        puldir_group_halt(a->group, NULL);
        port_unlock();
        return;
    }
    if (a->slot < PULDIR_MAX_AXES) {
        PulDirPulse s;
        puldir_kill(a->slot);
        puldir_pulse_read(a->slot, &s);
        a->target = (uint64_t)(uint32_t)s.count << 32;
    }
    if (a->mode == PULDIR_MODE_HOMING)
        home_unwind(a);
    a->req = (uint16_t)((a->req & ~REQ_COURSE) | REQ_HALT);
    a->mode = PULDIR_MODE_IDLE;
    a->vel_out = 0.0f;
    port_unlock();
}

/* 5.8: enabling arms the enable delay; disabling halts first, and a pulse still high is not cut
 * short: the plan tick lowers the enable pin after it. Refused without an enable pin. */
bool PulDir_enable(PulDir* a, bool on)
{
    if (!a->has_ena) {
        puldir_refuse(a, "enable", PULDIR_R_NO_PIN, 0.0f);
        return false;
    }
    if (!on && (a->moving || a->mode != PULDIR_MODE_IDLE))
        PulDir_halt(a);
    port_lock();
    if (on) {
        puldir_wake(a);
    } else {
        const bool pulse_high = a->slot < PULDIR_MAX_AXES && puldir_pul_high(a->slot);
        if (pulse_high)
            a->ena_off = true;
        else
            port_pin_write(a->ena, a->ena_inverted);
        a->enabled = false;
        a->ena_wait = false;
        a->release_due = false;
    }
    port_unlock();
    return true;
}

/* ---- stream (5.6) ---------------------------------------------------------- */

/* From IDLE, PROFILE or STOPPING; a pending move or stop gives way. The plan tick takes the axis's
 * velocity as the stream's and starts the timeout, so the call itself changes no motion. */
bool PulDir_streamBegin(PulDir* a)
{
    PulDirRefusal why = PULDIR_R_NONE;
    port_lock();
    const PulDirMode m = a->mode;
    if (a->fault) {
        why = PULDIR_R_FAULT;
    } else if (m == PULDIR_MODE_STREAM || m == PULDIR_MODE_GROUP || m == PULDIR_MODE_HOMING) {
        why = PULDIR_R_MODE;
    } else {
        a->req = (uint16_t)((a->req & ~REQ_MOTION) | REQ_STREAM_BEGIN);
        a->mode = PULDIR_MODE_STREAM;
        a->moving = true;
        puldir_wake(a);
        a->stream_timed_out = false;
        a->stream_err = 0.0f;
        a->sp_tick = PulDir_ticks();        /* the timeout runs from here until a setpoint */
        a->sp_have = false;
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
    const float p = position * a->conv, v = velocity * a->conv;
    bool ok = isfinite(v) && p >= -PULDIR_PULSES_MAX && p < PULDIR_PULSES_MAX;   /* false for NaN */
    if (ok) {
        port_lock();
        ok = a->mode == PULDIR_MODE_STREAM;
        if (ok) {
            a->sp_pos = p;
            a->sp_vel = v;
            a->sp_tick = PulDir_ticks();
            a->sp_have = true;
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
        memcpy(a->lim, a->def, sizeof a->lim);   /* validated when set; fixed in pulses since */
        a->req |= REQ_LIMITS;                 /* the plan tick sees STOPPING and leaves */
        a->mode = PULDIR_MODE_STOPPING;
    }
    port_unlock();
}

bool  PulDir_inStream(PulDir* a)       { return a->mode == PULDIR_MODE_STREAM; }
bool  PulDir_streamTimedOut(PulDir* a) { return a->stream_timed_out; }
float PulDir_streamError(PulDir* a)    { return a->stream_err / a->conv; }

/* D11: at least five plan intervals (5.6), and not while streaming. */
bool PulDir_setStreamCatchup(PulDir* a, uint32_t ms)
{
    PulDirRefusal why = PULDIR_R_NONE;
    if ((uint64_t)ms * PULDIR_PLAN_HZ < 5000u) {
        why = PULDIR_R_ARG;
    } else {
        port_lock();
        if (a->mode == PULDIR_MODE_STREAM)
            why = PULDIR_R_MODE;
        else
            a->catchup_ms = ms;
        port_unlock();
    }
    if (why != PULDIR_R_NONE) {
        puldir_refuse(a, "setStreamCatchup", why, 0.0f);
        return false;
    }
    return true;
}

/* ---- homing and fault (5.8) -------------------------------------------------- */

/* The homing values, checked at the conversion they will run under: the speed
 * below the tick ceiling, the adjust and the datum whole pulses inside int32. */
static PulDirRefusal home_values(const PulDir* a, float speed, float adjust, float zero, float* cap)
{
    const float c = a->conv;
    const float z = roundf(zero * c), d = roundf(adjust * c);
    *cap = (float)(PULDIR_TICK_HZ / 2u) / c;
    if (!(speed < *cap))
        return PULDIR_R_RATE;
    if (!(z >= -PULDIR_PULSES_MAX && z < PULDIR_PULSES_MAX && d > -PULDIR_PULSES_MAX && d < PULDIR_PULSES_MAX))
        return PULDIR_R_ARG;
    return PULDIR_R_NONE;
}

/* D16: direction +-1, a finite positive speed under the ceiling, a switch, a
 * timeout of at least one tick and at most half the counter; refused while homing. */
bool PulDir_setHoming(PulDir* a, int direction, float speed, float adjust,
                      const uint8_t* switch_ptr, float zero_pos, uint32_t phase_timeout_ms)
{
    PulDirRefusal why = PULDIR_R_NONE;
    float cap = 0.0f;
    if ((direction != 1 && direction != -1) || !(isfinite(speed) && speed > 0.0f)
        || switch_ptr == NULL || PULDIR_MS_TICKS(phase_timeout_ms) == 0u || !ms_fits(phase_timeout_ms)
        || !isfinite(adjust) || !isfinite(zero_pos))
        why = PULDIR_R_ARG;
    else
        why = home_values(a, speed, adjust, zero_pos, &cap);
    if (why == PULDIR_R_NONE) {
        port_lock();
        if (a->mode == PULDIR_MODE_HOMING) {
            why = PULDIR_R_BUSY;
        } else {
            a->home_dir = (int8_t)direction;
            a->home_speed = speed;
            a->home_adjust = adjust;
            a->home_zero = zero_pos;
            a->home_switch = switch_ptr;
            a->home_timeout_ticks = PULDIR_MS_TICKS(phase_timeout_ms);
        }
        port_unlock();
    }
    if (why != PULDIR_R_NONE) {
        puldir_refuse(a, "setHoming", why, why == PULDIR_R_RATE ? cap : 0.0f);
        return false;
    }
    return true;
}

/* 5.5: IDLE and at rest, not faulted, configured. The values are checked again because the
 * conversion may have changed since setHoming; the plan tick runs the phases (5.8). */
bool PulDir_homeStart(PulDir* a)
{
    PulDirRefusal why = PULDIR_R_NONE;
    float cap = 0.0f;
    if (a->home_dir == 0)
        why = PULDIR_R_NO_HOMING;
    else
        why = home_values(a, a->home_speed, a->home_adjust, a->home_zero, &cap);
    if (why == PULDIR_R_NONE) {
        port_lock();
        if (a->fault) {
            why = PULDIR_R_FAULT;
        } else if (a->mode != PULDIR_MODE_IDLE) {
            why = PULDIR_R_MODE;
        } else if (a->moving) {
            why = PULDIR_R_NOT_IDLE;
        } else {
            a->homed = false;
            memcpy(a->home_lim, a->lim, sizeof a->home_lim);
            a->req |= REQ_HOME;
            a->mode = PULDIR_MODE_HOMING;
            a->moving = true;
            a->home_state = PULDIR_HOME_MOVE_TOWARDS_LIMIT;
            puldir_wake(a);
        }
        port_unlock();
    }
    if (why != PULDIR_R_NONE) {
        puldir_refuse(a, "homeStart", why, why == PULDIR_R_RATE ? cap : 0.0f);
        return false;
    }
    return true;
}

/* Halts a run in progress; the homing state goes IDLE either way (it clears ERROR). */
void PulDir_homeStop(PulDir* a)
{
    if (a->mode == PULDIR_MODE_HOMING)
        PulDir_halt(a);
    port_lock();
    a->home_state = PULDIR_HOME_IDLE;
    port_unlock();
}

PulDirHomeState PulDir_homeState(PulDir* a) { return a->home_state; }
bool PulDir_isHomed(PulDir* a)              { return a->homed; }

/* Raises the clear pin and returns; the plan tick lowers it after
 * PULDIR_FAULT_CLEAR_MS and lifts the latch (5.8). Nothing blocks. */
bool PulDir_clearFault(PulDir* a)
{
    PulDirRefusal why = PULDIR_R_NONE;
    if (!a->has_clr) {
        why = PULDIR_R_NO_PIN;
    } else {
        port_lock();
        if (a->clearing) {
            why = PULDIR_R_BUSY;
        } else {
            port_pin_write(a->clr, !a->clr_inverted);
            a->clearing = true;
            a->clr_t0 = PulDir_ticks();
        }
        port_unlock();
    }
    if (why != PULDIR_R_NONE) {
        puldir_refuse(a, "clearFault", why, 0.0f);
        return false;
    }
    return true;
}

/* ---- state ----------------------------------------------------------------- */

static int32_t emitted(const PulDir* a)
{
    if (a->slot >= PULDIR_MAX_AXES)
        return 0;
    PulDirPulse s;
    port_lock();
    puldir_pulse_read(a->slot, &s);
    port_unlock();
    return s.count;
}

/* D18: the pulses the drive has been told, not the profile. */
float PulDir_position(PulDir* a) { return (float)emitted(a) / a->conv; }
float PulDir_velocity(PulDir* a) { return a->vel_out / a->conv; }

float PulDir_target(PulDir* a)
{
    port_lock();
    const uint64_t t = a->target;
    port_unlock();
    const int32_t whole = (int32_t)(uint32_t)(t >> 32);
    return ((float)whole + (float)(uint32_t)t * (1.0f / PULDIR_FIX)) / a->conv;
}

/* D18: the datum, in IDLE only. The pulse tick's count, position and edge move
 * together under the lock; the plan tick moves the generator (REQ_SETPOS). */
bool PulDir_setPosition(PulDir* a, float pos)
{
    const float x = roundf(pos * a->conv);
    PulDirRefusal why = PULDIR_R_NONE;
    if (!(x >= -PULDIR_PULSES_MAX && x < PULDIR_PULSES_MAX)) {
        why = PULDIR_R_ARG;
    } else {
        const int32_t c = (int32_t)x;
        const bool reg = a->slot < PULDIR_MAX_AXES;
        port_lock();
        PulDirPulse s = { 0, 0, 0, 1 };
        if (reg)
            puldir_pulse_read(a->slot, &s);
        /* IDLE and not moving imply no increment left and the pin low, which
         * puldir_set_datum needs; checked anyway, it is two loads. */
        if (a->mode != PULDIR_MODE_IDLE || a->moving || s.inc != 0 || (reg && puldir_pul_high(a->slot))) {
            why = PULDIR_R_NOT_IDLE;
        } else {
            if (reg)
                puldir_set_datum(a->slot, c);
            a->target = (uint64_t)(uint32_t)c << 32;
            a->homed = false;
            a->req |= REQ_SETPOS;
        }
        port_unlock();
    }
    if (why != PULDIR_R_NONE) {
        puldir_refuse(a, "setPosition", why, 0.0f);
        return false;
    }
    return true;
}

bool       PulDir_isMoving(PulDir* a) { return a->moving; }
PulDirMode PulDir_mode(PulDir* a)     { return a->mode; }

PulDirStatus PulDir_status(PulDir* a)
{
    if (a->fault)
        return PULDIR_FAULT;
    if (a->moving)
        return PULDIR_MOVING;
    return (a->enabled || !a->has_ena) ? PULDIR_ENABLED : PULDIR_IDLE;
}
