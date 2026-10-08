/* Main context: setup, commands, getters. */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "puldir_internal.h"

/* The rate cap in user units: speeds at or above it are refused. */
static float rate_cap(float conv)
{
    return (float)(puldir_timing.tick_hz / 2u) / conv;
}

/* Init's refusal: no axis holds an event yet, so the line goes out now. */
static bool init_refused(const char* name, const char* why)
{
    char b[PULDIR_LINE];
    snprintf(b, sizeof b, "[%s] init refused: %s", name, why);
    puldir_log(NULL, b);
    return false;
}

bool PulDir_init(PulDir* a, const char* name, float pulses_per_unit)
{
    const float c = pulses_per_unit;
    size_t len = 0;
    while (name != NULL && len < sizeof a->name && name[len] != '\0')
        len++;
    if (name == NULL || len == sizeof a->name)
        return init_refused("?", "name NULL or longer than 15 characters");
    if (!(isfinite(c) && c > 0.0f))
        return init_refused(name, "conversion not finite and positive");
    if (!puldir_timing_take())
        return init_refused(name, puldir_timing.pulse_us == 0u || puldir_timing.plan_us == 0u
                                  ? "timers not attached"
                                  : "plan period not 2 to 4096 whole pulse periods");
    memset(a, 0, sizeof *a);
    memcpy(a->name, name, len);
    a->conv = c;
    /* The starting limits are 200 pulses/s, 200 pulses/s^2, no jerk: under the rate cap at any
     * timing, whatever the conversion. */
    const float d[4] = { 200.0f, 200.0f, 200.0f, 0.0f };
    for (int k = 0; k < 4; k++)
        a->def[k] = a->lim[k] = d[k] / c;
    a->catchup_ms = PULDIR_CATCHUP_MS;
    a->log_on = true;
    a->slot = PULDIR_MAX_AXES;
    a->dir_out = 1;                     /* the pulse tick's entries start at +1 */
    a->mode = PULDIR_MODE_IDLE;
    a->home_state = PULDIR_HOME_IDLE;
    a->refusal = PULDIR_R_NONE;
    /* puldir_timing_plan kept dt inside MotionProfile_init's range and the
     * limits are finite and positive, so neither call can refuse. */
    (void)MotionProfile_init(&a->prof, 0.0f, puldir_timing.dt);
    (void)MotionProfile_setLimits(&a->prof, (MotionLimits){ d[0], d[1], d[2], d[3] });

    char b[PULDIR_LINE];
    snprintf(b, sizeof b, "[%s] max speed %.4f units/s, resolution %.4f units/pulse (pulse %lu us, plan %lu us)",
             a->name, (double)rate_cap(c), (double)(1.0f / c), (unsigned long)puldir_timing.pulse_us,
             (unsigned long)puldir_timing.plan_us);
    puldir_log(a, b);
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
    port_pin_write(pulse, pulse_active_low);              /* parked inactive, before the ticks run */
    port_pin_write(dir, !dir_inverted);                   /* the level for +1 */
}

bool PulDir_register(PulDir* a)
{
    PulDirRefusal why = PULDIR_R_NONE;
    if (!a->has_pins)
        why = PULDIR_R_NO_PIN;
    else if (a->slot != PULDIR_MAX_AXES || puldir_timing.started)
        why = PULDIR_R_BUSY;            /* already registered (or never initialised), or the ticks run */
    else if (puldir_naxes >= PULDIR_MAX_AXES)
        why = PULDIR_R_FULL;
    if (why != PULDIR_R_NONE) {
        puldir_refuse(a, "register", why, 0.0f);
        return false;
    }
    puldir_pulse_add(a);
    return true;
}

/* ---- limits ---------------------------------------------------------------- */

/* GROUP: the path profile owns the limits until the move ends (5.7). HOMING: the
 * run sets them from the plan tick. */
static bool limits_busy(PulDirMode m)
{
    return m == PULDIR_MODE_GROUP || m == PULDIR_MODE_HOMING;
}

/* Checks four limits (user units) as MotionProfile_setLimits will see them, the rate cap being the
 * axis's own, then stores them, and def too (D17). The check reads the profile's frozen window and
 * the acceleration its ring may still hold (E32), so it runs under the lock: a few compares, a ceilf
 * and two divisions, about 100 cycles. */
static bool apply_limits(PulDir* a, const float lim[4], bool defaults, const char* call)
{
    const float conv = a->conv, cap = rate_cap(conv);
    const MotionLimits L = { lim[0] * conv, lim[1] * conv, lim[2] * conv, lim[3] * conv };
    PulDirRefusal why = PULDIR_R_NONE;
    float value = 0.0f;
    port_lock();
    const MotionLimitsCheck check = MotionProfile_checkLimits(&a->prof, L);
    if (check == MOTION_LIMITS_INVALID)
        why = PULDIR_R_ARG;
    else if (!(lim[0] < cap))
        why = PULDIR_R_RATE;
    else if (check == MOTION_LIMITS_TAPS)
        why = PULDIR_R_TAPS;
    else if (limits_busy(a->mode))
        why = PULDIR_R_MODE;
    else if (check == MOTION_LIMITS_WINDOW)
        why = PULDIR_R_WINDOW;              /* the window is frozen in flight (5.2) */
    else {
        memcpy(a->lim, lim, sizeof a->lim);
        if (defaults)
            memcpy(a->def, lim, sizeof a->def);
        a->req |= REQ_LIMITS;
    }
    port_unlock();
    if (why == PULDIR_R_RATE)
        value = cap;
    else if (why == PULDIR_R_TAPS)          /* the log rounds the floor up to fit */
        value = MotionProfile_minJerk(L.amax, L.dmax, puldir_timing.dt) / conv;
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

/* Under the lock, for every motion command: the fault latch and the mode at one instant, so a
 * latch the plan tick closes between a check and the store cannot slip through (D17). */
static PulDirRefusal motion_check(const PulDir* a)
{
    const PulDirMode m = a->mode;
    if (a->slot == PULDIR_MAX_AXES)
        return PULDIR_R_UNREGISTERED;   /* the ticks cannot run it: start needs registration */
    if (a->fault.latched)
        return PULDIR_R_FAULT;
    if (m == PULDIR_MODE_STREAM || m == PULDIR_MODE_GROUP || m == PULDIR_MODE_HOMING)
        return PULDIR_R_MODE;
    return PULDIR_R_NONE;
}

/* Under the lock: what the caller may read back at once is set here (D17). */
static PulDirRefusal take_target(PulDir* a, uint64_t target)
{
    const PulDirRefusal why = motion_check(a);
    if (why != PULDIR_R_NONE)
        return why;
    a->target = target;
    a->req = (uint16_t)((a->req & ~REQ_MOTION) | REQ_TARGET);
    a->mode = PULDIR_MODE_PROFILE;
    a->moving = true;
    puldir_enable_wake(a);
    return PULDIR_R_NONE;
}

/* Targets are whole pulses (D4). */
bool PulDir_moveTo(PulDir* a, float position)
{
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
 * wherever any jog is (5.5), so a jog never ends a STREAM: PulDir_stop's two stores, under the
 * same check. */
bool PulDir_jog(PulDir* a, float speed)
{
    const float cap = rate_cap(a->conv);
    PulDirRefusal why = PULDIR_R_NONE;
    if (!isfinite(speed)) {
        why = PULDIR_R_ARG;
    } else if (!(fabsf(speed) < cap)) {
        why = PULDIR_R_RATE;
    } else {
        port_lock();
        why = motion_check(a);
        if (why == PULDIR_R_NONE && speed == 0.0f) {
            if (a->mode != PULDIR_MODE_IDLE) {
                a->req |= REQ_STOP;
                a->mode = PULDIR_MODE_STOPPING;
            }
        } else if (why == PULDIR_R_NONE) {
            a->req_speed = speed * a->conv;
            a->req = (uint16_t)((a->req & ~REQ_MOTION) | REQ_ENDLESS);
            a->mode = PULDIR_MODE_PROFILE;
            a->moving = true;
            puldir_enable_wake(a);
        }
        port_unlock();
    }
    if (why != PULDIR_R_NONE) {
        puldir_refuse(a, "jog", why, why == PULDIR_R_RATE ? cap : 0.0f);
        return false;
    }
    return true;
}

/* Under the lock, a homing run ending unhomed: the limits homeStart found go
 * back. The plan tick hands them to the generator once its window allows. */
static void home_restore(PulDir* a)
{
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
            a->home_state = PULDIR_HOME_IDLE;
            home_restore(a);
            a->req &= (uint16_t)~REQ_HOME;  /* not yet begun: there is nothing to stop */
        }
        a->req |= REQ_STOP;
        a->mode = PULDIR_MODE_STOPPING;
    }
    port_unlock();
    if (group)
        puldir_refuse(a, "stop", PULDIR_R_MODE, 0.0f);
}

void puldir_halt_locked(PulDir* a)
{
    if (a->slot < PULDIR_MAX_AXES) {
        PulDirSnap s;
        puldir_kill(a->slot);
        puldir_pulse_read(a->slot, 0u, &s);
        a->target = (uint64_t)(uint32_t)s.count << 32;
    }
    if (a->mode == PULDIR_MODE_HOMING)
        home_restore(a);
    a->req = (uint16_t)((a->req & ~REQ_COURSE) | REQ_HALT);
    a->mode = PULDIR_MODE_IDLE;
    a->vel_out = 0.0f;
}

/* Immediate (D17): no rising edge after this tick, the target where the pulses stopped. A member
 * halts its whole group (5.5); a homing run ends. isMoving clears once the pin is low. */
void PulDir_halt(PulDir* a)
{
    port_lock();
    if (a->mode == PULDIR_MODE_GROUP && a->group != NULL) {
        puldir_group_halt(a->group, NULL);
    } else {
        const bool homing = a->mode == PULDIR_MODE_HOMING;
        puldir_halt_locked(a);
        if (homing)
            a->home_state = PULDIR_HOME_IDLE;
    }
    port_unlock();
}

/* ---- stream (5.6) ---------------------------------------------------------- */

/* From IDLE, PROFILE or STOPPING; a pending move or stop gives way. The plan tick takes the axis's
 * velocity as the stream's and starts the timeout, so the call itself changes no motion. */
bool PulDir_streamBegin(PulDir* a)
{
    port_lock();
    const PulDirRefusal why = motion_check(a);
    if (why == PULDIR_R_NONE) {
        a->req = (uint16_t)((a->req & ~REQ_MOTION) | REQ_STREAM_BEGIN);
        a->mode = PULDIR_MODE_STREAM;
        a->moving = true;
        puldir_enable_wake(a);
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
        memcpy(a->lim, a->def, sizeof a->lim);   /* validated when set, at the one conversion */
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
    if ((uint64_t)ms * 1000u < 5u * (uint64_t)puldir_timing.plan_us) {
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

/* ---- homing (5.8) ------------------------------------------------------------ */

/* The homing values, checked at the conversion they will run under: the speed
 * below the tick ceiling, the adjust and the datum whole pulses inside int32. */
static PulDirRefusal home_values(const PulDir* a, float speed, float adjust, float zero, float* cap)
{
    const float c = a->conv;
    const float z = roundf(zero * c), d = roundf(adjust * c);
    *cap = rate_cap(c);
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
        || switch_ptr == NULL || puldir_ms_ticks(phase_timeout_ms) == 0u || !puldir_ms_fits(phase_timeout_ms)
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
            a->home_timeout_ticks = puldir_ms_ticks(phase_timeout_ms);
        }
        port_unlock();
    }
    if (why != PULDIR_R_NONE) {
        puldir_refuse(a, "setHoming", why, why == PULDIR_R_RATE ? cap : 0.0f);
        return false;
    }
    return true;
}

/* 5.5: IDLE and at rest, not faulted, configured. setHoming checked the values at the conversion
 * and timing they run under, both fixed since init; the plan tick runs the phases (5.8). */
bool PulDir_homeStart(PulDir* a)
{
    PulDirRefusal why = a->home_dir == 0 ? PULDIR_R_NO_HOMING : PULDIR_R_NONE;
    if (why == PULDIR_R_NONE) {
        port_lock();
        if (a->slot == PULDIR_MAX_AXES) {
            why = PULDIR_R_UNREGISTERED;    /* as motion_check: the ticks cannot run it */
        } else if (a->fault.latched) {
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
            puldir_enable_wake(a);
        }
        port_unlock();
    }
    if (why != PULDIR_R_NONE) {
        puldir_refuse(a, "homeStart", why, 0.0f);
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

/* ---- state ----------------------------------------------------------------- */

static int32_t emitted(const PulDir* a)
{
    if (a->slot >= PULDIR_MAX_AXES)             /* unregistered: the datum, which registration takes over */
        return (int32_t)(uint32_t)(a->target >> 32);
    PulDirSnap s;
    port_lock();
    puldir_pulse_read(a->slot, 0u, &s);
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
        PulDirSnap s = { 0, 0, 0, 1, false };
        if (reg)
            puldir_pulse_read(a->slot, 0u, &s);
        /* IDLE and not moving imply no increment left and the pin low, which
         * puldir_set_datum needs; checked anyway, it is two loads. */
        if (a->mode != PULDIR_MODE_IDLE || a->moving || s.inc != 0 || s.high) {
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
    if (a->fault.latched)
        return PULDIR_FAULT;
    if (a->moving)
        return PULDIR_MOVING;
    return (a->enable.enabled || !a->enable.attached) ? PULDIR_ENABLED : PULDIR_IDLE;
}
