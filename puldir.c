/* Main context: setup, commands, getters. */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "puldir_internal.h"

/* The rate cap in user units: speeds at or above it are refused. */
float puldir_rate_cap(float pulses_per_unit)
{
    return (float)(puldir_timing.tick_hz / 2u) / pulses_per_unit;
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
    memset(a, 0, sizeof *a);              /* every module at rest: drive off, no latch, homing IDLE */
    memcpy(a->name, name, len);
    a->pulses_per_unit = c;
    /* The starting settings are 200 pulses/s, 200 pulses/s^2, no jerk: under the rate cap at any
     * timing, whatever the conversion. */
    const MotionSettings d = { 200.0f, 200.0f, 200.0f, 0.0f };
    a->defaults = a->settings = (MotionSettings){ d.speed / c, d.accel / c, d.decel / c, d.jerk / c };
    puldir_stream_defaults(a);
    a->log_on = true;
    a->slot = PULDIR_MAX_AXES;
    a->dir_out = 1;                     /* the pulse tick's entries start at +1 */
    a->mode = PULDIR_MODE_IDLE;
    a->refusal = PULDIR_R_NONE;
    /* puldir_timing_plan kept dt inside MotionProfile_init's range and the
     * settings are finite and positive, so neither call can refuse. */
    (void)MotionProfile_init(&a->profile, 0.0f, puldir_timing.dt);
    (void)MotionProfile_setSettings(&a->profile, d);

    char b[PULDIR_LINE];
    snprintf(b, sizeof b, "[%s] max speed %.4f units/s, resolution %.4f units/pulse (pulse %lu us, plan %lu us)",
             a->name, (double)puldir_rate_cap(c), (double)(1.0f / c),
             (unsigned long)puldir_timing.pulse_us, (unsigned long)puldir_timing.plan_us);
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

/* ---- settings -------------------------------------------------------------- */

MotionSettings puldir_in_pulses(const PulDir* a, MotionSettings s)
{
    const float c = a->pulses_per_unit;
    return (MotionSettings){ s.speed * c, s.accel * c, s.decel * c, s.jerk * c };
}

/* GROUP: the path profile owns the settings until the move ends (5.7). HOMING: the
 * run sets them from the plan tick. */
static bool settings_busy(PulDirMode m)
{
    return m == PULDIR_MODE_GROUP || m == PULDIR_MODE_HOMING;
}

/* Checks the settings (user units) as MotionProfile_setSettings will see them, the rate cap being
 * the axis's own, then stores them, and the defaults too (D17). The check reads the profile's frozen
 * window and the acceleration its ring may still hold (E32), so it runs under the lock: a few
 * compares, a ceilf and two divisions, about 100 cycles. */
static bool apply_settings(PulDir* a, MotionSettings s, bool defaults, const char* call)
{
    const float cap = puldir_rate_cap(a->pulses_per_unit);
    const MotionSettings S = puldir_in_pulses(a, s);
    PulDirRefusal why = PULDIR_R_NONE;
    float value = 0.0f;
    port_lock();
    const MotionSettingsCheck check = MotionProfile_checkSettings(&a->profile, S);
    if (check == MOTION_SETTINGS_INVALID)
        why = PULDIR_R_ARG;
    else if (!(s.speed < cap))
        why = PULDIR_R_RATE;
    else if (check == MOTION_SETTINGS_TAPS)
        why = PULDIR_R_TAPS;
    else if (settings_busy(a->mode))
        why = PULDIR_R_MODE;
    else if (check == MOTION_SETTINGS_WINDOW)
        why = PULDIR_R_WINDOW;              /* the window is frozen in flight (5.2) */
    else {
        a->settings = s;
        if (defaults)
            a->defaults = s;
        a->req |= REQ_SETTINGS;
    }
    port_unlock();
    if (why == PULDIR_R_RATE)
        value = cap;
    else if (why == PULDIR_R_TAPS)          /* the log rounds the floor up to fit */
        value = MotionProfile_minJerk(S.accel, S.decel, puldir_timing.dt) / a->pulses_per_unit;
    if (why != PULDIR_R_NONE) {
        puldir_refuse(a, call, why, value);
        return false;
    }
    return true;
}

/* For a one-setting setter: the current settings into *s and the mode, at one instant. A copy taken
 * in HOMING could be stale by the time the run ends, so it is refused here, not by apply_settings. */
static bool current_settings(PulDir* a, MotionSettings* s, const char* call)
{
    port_lock();
    *s = a->settings;
    const bool busy = settings_busy(a->mode);
    port_unlock();
    if (busy)
        puldir_refuse(a, call, PULDIR_R_MODE, 0.0f);
    return !busy;
}

bool PulDir_setDefaults(PulDir* a, float speed, float accel, float decel, float jerk)
{
    return apply_settings(a, (MotionSettings){ speed, accel, decel, jerk }, true, "setDefaults");
}

bool PulDir_restoreDefaults(PulDir* a)
{
    port_lock();
    const MotionSettings d = a->defaults;
    port_unlock();
    return apply_settings(a, d, false, "restoreDefaults");
}

bool PulDir_setSpeed(PulDir* a, float x)
{
    MotionSettings s;
    if (!current_settings(a, &s, "setSpeed"))
        return false;
    s.speed = x;
    return apply_settings(a, s, false, "setSpeed");
}

bool PulDir_setAcceleration(PulDir* a, float x)
{
    MotionSettings s;
    if (!current_settings(a, &s, "setAcceleration"))
        return false;
    s.accel = x;
    return apply_settings(a, s, false, "setAcceleration");
}

bool PulDir_setDeceleration(PulDir* a, float x)
{
    MotionSettings s;
    if (!current_settings(a, &s, "setDeceleration"))
        return false;
    s.decel = x;
    return apply_settings(a, s, false, "setDeceleration");
}

bool PulDir_setJerk(PulDir* a, float x)
{
    MotionSettings s;
    if (!current_settings(a, &s, "setJerk"))
        return false;
    s.jerk = x;
    return apply_settings(a, s, false, "setJerk");
}

float PulDir_speed(PulDir* a)                    { return a->settings.speed; }
float PulDir_acceleration(PulDir* a)             { return a->settings.accel; }
float PulDir_deceleration(PulDir* a)             { return a->settings.decel; }
float PulDir_jerk(PulDir* a)                     { return a->settings.jerk; }

/* ---- motion ---------------------------------------------------------------- */

/* Under the lock, for every motion command: the fault latch and the mode at one instant, so a
 * latch the plan tick closes between a check and the store cannot slip through (D17). */
PulDirRefusal puldir_motion_check(const PulDir* a)
{
    const PulDirMode m = a->mode;
    if (a->slot == PULDIR_MAX_AXES)
        return PULDIR_R_UNREGISTERED;   /* the ticks cannot run it: start needs registration */
    if (puldir_fault_latched(a))
        return PULDIR_R_FAULT;
    if (m == PULDIR_MODE_STREAM || m == PULDIR_MODE_GROUP || m == PULDIR_MODE_HOMING)
        return PULDIR_R_MODE;
    return PULDIR_R_NONE;
}

/* Under the lock: what the caller may read back at once is set here (D17). */
static PulDirRefusal take_target(PulDir* a, uint64_t target)
{
    const PulDirRefusal why = puldir_motion_check(a);
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
    const float x = roundf(position * a->pulses_per_unit);
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
    const float dx = roundf(distance * a->pulses_per_unit);
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
    const float cap = puldir_rate_cap(a->pulses_per_unit);
    PulDirRefusal why = PULDIR_R_NONE;
    if (!isfinite(speed)) {
        why = PULDIR_R_ARG;
    } else if (!(fabsf(speed) < cap)) {
        why = PULDIR_R_RATE;
    } else {
        port_lock();
        why = puldir_motion_check(a);
        if (why == PULDIR_R_NONE && speed == 0.0f) {
            if (a->mode != PULDIR_MODE_IDLE) {
                a->req |= REQ_STOP;
                a->mode = PULDIR_MODE_STOPPING;
            }
        } else if (why == PULDIR_R_NONE) {
            a->req_speed = speed * a->pulses_per_unit;
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

/* Ramped from the motion as commanded: a pending target or jog applies first. A group member
 * refuses it (5.7); in HOMING it ends the run unhomed and stops at the axis's decel. */
void PulDir_stop(PulDir* a)
{
    port_lock();
    const bool group = a->mode == PULDIR_MODE_GROUP;
    if (!group && a->mode != PULDIR_MODE_IDLE) {
        if (a->mode == PULDIR_MODE_HOMING) {
            puldir_home_cancel(a);
            puldir_home_restore(a);
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
        puldir_home_restore(a);
    a->req = (uint16_t)((a->req & ~REQ_COURSE) | REQ_HALT);
    a->mode = PULDIR_MODE_IDLE;
    a->velocity = 0.0f;
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
            puldir_home_cancel(a);
    }
    port_unlock();
}

/* ---- state ----------------------------------------------------------------- */

/* Lock-free: the firmware reads it on every main-loop pass (E42). */
static int32_t emitted(const PulDir* a)
{
    if (a->slot >= PULDIR_MAX_AXES)             /* unregistered: the datum, which registration takes over */
        return (int32_t)(uint32_t)(a->target >> 32);
    return puldir_pulse_count(a->slot);
}

/* D18: the pulses the drive has been told, not the profile. */
float PulDir_position(PulDir* a) { return (float)emitted(a) / a->pulses_per_unit; }
float PulDir_velocity(PulDir* a) { return a->velocity / a->pulses_per_unit; }

float PulDir_target(PulDir* a)
{
    port_lock();
    const uint64_t t = a->target;
    port_unlock();
    const int32_t whole = (int32_t)(uint32_t)(t >> 32);
    return ((float)whole + (float)(uint32_t)t * (1.0f / PULDIR_FIX)) / a->pulses_per_unit;
}

/* D18: the datum, in IDLE only. The pulse tick's count, position and edge move
 * together under the lock; the plan tick moves the generator (REQ_SETPOS). */
bool PulDir_setPosition(PulDir* a, float pos)
{
    const float x = roundf(pos * a->pulses_per_unit);
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
            a->req |= REQ_SETPOS;           /* homed stays: a datum set by hand is the caller's call (E43) */
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
    if (puldir_fault_latched(a))
        return PULDIR_FAULT;
    if (a->moving)
        return PULDIR_MOVING;
    return (a->enable.enabled || !a->enable.attached) ? PULDIR_ENABLED : PULDIR_IDLE;
}
