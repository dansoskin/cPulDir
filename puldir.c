/* Main context: setup, commands, getters. */
#include <math.h>
#include <string.h>
#include "puldir_internal.h"

#define DT (1.0f / (float)PULDIR_PLAN_HZ)      /* the profile's dt, the same expression as init's */
#define PULSES_MAX 2147483648.0f                /* whole-pulse targets lie in [-2^31, 2^31) */

static bool started;                    /* PulDir_start has run */

void PulDir_init(PulDir* a, const char* name)
{
    memset(a, 0, sizeof *a);
    size_t i = 0;
    if (name)
        for (; i < sizeof a->name - 1u && name[i] != '\0'; i++)
            a->name[i] = name[i];
    a->name[i] = '\0';
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
    (void)MotionProfile_init(&a->prof, 0.0f, 1.0f / (float)PULDIR_PLAN_HZ);
    (void)MotionProfile_setLimits(&a->prof, (MotionLimits){ d[0], d[1], d[2], d[3] });
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
    return ceilf((acc + dec) / (j * DT)) <= (float)MOTION_PROFILE_MAX_TAPS;
}

/* The smallest jerk in user units that fits: minJerk's quotient and the
 * conversion each round, so step to the exact float boundary. */
static float jerk_floor(float acc, float dec, float conv)
{
    float j = MotionProfile_minJerk(acc, dec, DT) / conv;
    while (!taps_fit(acc, dec, j * conv))
        j = nextafterf(j, INFINITY);
    for (float d = nextafterf(j, 0.0f); taps_fit(acc, dec, d * conv); d = nextafterf(d, 0.0f))
        j = d;
    return j;
}

/* Validate four limits (user units) at conv exactly as MotionProfile_setLimits
 * will see them, so the plan tick's call cannot refuse; then hand them over
 * (D17). The frozen-window test reads n and done under the same lock as the
 * stores, so the plan tick cannot move the generator in between. */
static bool apply_limits(PulDir* a, const float lim[4], const char* call)
{
    const float conv = a->conv;
    const float v = lim[0] * conv, acc = lim[1] * conv, dec = lim[2] * conv, j = lim[3] * conv;
    const float ratio = j > 0.0f ? (acc + dec) / (j * DT) : 0.0f;    /* generator's window() */
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
        if (a->prof.done || j == 0.0f || ratio <= (float)a->prof.n) {
            for (int k = 0; k < 4; k++)
                a->lim[k] = lim[k];
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

static bool set_limit(PulDir* a, int k, float x, const char* call)
{
    float l[4];
    memcpy(l, a->lim, sizeof l);
    l[k] = x;
    return apply_limits(a, l, call);
}

/* Limits and defaults stay fixed in pulses, as in cFlexyStepper: the user-unit
 * values are rescaled, so the generator's limits are untouched and there is
 * nothing to check but the argument. */
bool PulDir_setConversion(PulDir* a, float pulses_per_unit)
{
    const float c = pulses_per_unit;
    PulDirRefusal why = PULDIR_R_NONE;
    if (!(isfinite(c) && c > 0.0f)) {
        why = PULDIR_R_ARG;
    } else {
        float lim[4], def[4];
        for (int k = 0; k < 4; k++) {           /* read in main context, where they are written */
            lim[k] = a->lim[k] * a->conv / c;
            def[k] = a->def[k] * a->conv / c;
        }
        port_lock();
        if (a->mode == PULDIR_MODE_IDLE && !a->moving) {
            a->conv = c;
            memcpy(a->lim, lim, sizeof lim);
            memcpy(a->def, def, sizeof def);
        } else {
            why = PULDIR_R_NOT_IDLE;
        }
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
    if (!apply_limits(a, d, "setDefaults"))
        return false;
    memcpy(a->def, d, sizeof d);
    return true;
}

bool PulDir_restoreDefaults(PulDir* a)
{
    return apply_limits(a, a->def, "restoreDefaults");
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

/* The motion requests: a newer one replaces any older one still pending, so
 * the plan tick applies only the latest (a halt is kept: it comes first). */
#define REQ_MOTION (REQ_TARGET | REQ_ENDLESS | REQ_STOP)

/* Under the lock: what the caller may read back at once is set here (D17). */
static void take_target(PulDir* a, uint64_t target)
{
    a->target = target;
    a->req = (uint16_t)((a->req & ~REQ_MOTION) | REQ_TARGET);
    a->mode = PULDIR_MODE_PROFILE;
    a->moving = true;
}

/* Targets are whole pulses (D4). */
bool PulDir_moveTo(PulDir* a, float position)
{
    if (motion_refused(a, "moveTo"))
        return false;
    const float x = roundf(position * a->conv);
    if (!(x >= -PULSES_MAX && x < PULSES_MAX)) {
        puldir_refuse(a, "moveTo", PULDIR_R_ARG, 0.0f);
        return false;
    }
    port_lock();
    take_target(a, (uint64_t)(int64_t)x << 32);
    port_unlock();
    return true;
}

bool PulDir_moveBy(PulDir* a, float distance)
{
    if (motion_refused(a, "moveBy"))
        return false;
    const float dx = roundf(distance * a->conv);
    bool ok = dx >= -2.0f * PULSES_MAX && dx <= 2.0f * PULSES_MAX;   /* finite, and the sum fits */
    if (ok) {
        port_lock();
        const int64_t whole = (int32_t)(uint32_t)((a->target + (PULDIR_ONE >> 1)) >> 32);
        const int64_t t = whole + (int64_t)dx;
        ok = t >= INT32_MIN && t <= INT32_MAX;
        if (ok)
            take_target(a, (uint64_t)t << 32);
        port_unlock();
    }
    if (!ok)
        puldir_refuse(a, "moveBy", PULDIR_R_ARG, 0.0f);
    return ok;
}

/* D7: run at speed until told otherwise. The count carries on (2.2). jog(0) is
 * a stop, but refused wherever any jog is (5.5), so STREAM is left by streamEnd
 * or stop, never by a jog. */
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
    a->req_arg[0] = v;
    a->req = (uint16_t)((a->req & ~REQ_MOTION) | REQ_ENDLESS);
    a->mode = PULDIR_MODE_PROFILE;
    a->moving = true;
    port_unlock();
    return true;
}

/* Ramped: the plan tick stops the generator from where it is. A pending target
 * or jog is applied first, so the stop is from the motion as commanded. A group
 * member refuses it (5.7): PulDirGroup_stop stops the group along its line. */
void PulDir_stop(PulDir* a)
{
    port_lock();
    const bool group = a->mode == PULDIR_MODE_GROUP;
    if (!group && a->mode != PULDIR_MODE_IDLE) {
        a->req |= REQ_STOP;
        a->mode = PULDIR_MODE_STOPPING;
    }
    port_unlock();
    if (group)
        puldir_refuse(a, "stop", PULDIR_R_MODE, 0.0f);
}

/* Immediate (D17): no rising edge after the tick this is called in. The target
 * is where the pulses stopped; isMoving clears once the plan tick sees the pin low.
 * A group member's halt halts the whole group (5.5). */
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
    a->req = (uint16_t)((a->req & ~(REQ_MOTION | REQ_STREAM_BEGIN)) | REQ_HALT);
    a->mode = PULDIR_MODE_IDLE;
    a->vel_out = 0.0f;
    port_unlock();
}

/* ---- stream (5.6) ---------------------------------------------------------- */

/* From IDLE, PROFILE or STOPPING. The plan tick takes the velocity the axis has
 * as the stream's, halts the generator and starts the timeout: the call itself
 * changes nothing in the motion. A pending move or stop gives way to it. */
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

/* One setpoint: where the axis should be now, and how fast it moves there. One
 * lock and four stores; the plan tick does the arithmetic. Outside STREAM it is
 * dropped and never starts a stream.
 *
 * Precision: sp_pos is a float in pulses and carries about seven significant
 * digits. Beyond 2^24 pulses (16.8 million) from the datum the setpoint itself
 * is quantised to a whole pulse or more, before the library sees it; the plan
 * tick's error arithmetic is exact relative to it, but cannot restore what the
 * float has lost.
 *
 * Main context only (5.9): the plan tick relies on sp_tick never being ahead of its own reading. */
void PulDir_stream(PulDir* a, float position, float velocity)
{
    const float p = position * a->conv, v = velocity * a->conv;
    bool ok = isfinite(v) && p >= -PULSES_MAX && p < PULSES_MAX;   /* false for NaN */
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
    return ((float)whole + (float)(uint32_t)t * (1.0f / 4294967296.0f)) / a->conv;
}

/* D18: the datum, in IDLE only. The pulse tick's count, position and edge move
 * together under the lock; the plan tick moves the generator (REQ_SETPOS). */
bool PulDir_setPosition(PulDir* a, float pos)
{
    const float x = roundf(pos * a->conv);
    PulDirRefusal why = PULDIR_R_NONE;
    if (!(x >= -PULSES_MAX && x < PULSES_MAX)) {
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
