/* Homing (5.8): setup and commands from main, cFlexyStepper's phases from the plan tick. */
#include <math.h>
#include "puldir_internal.h"

#define DELAY_TICKS puldir_ms_ticks(PULDIR_HOMING_DELAY_MS)

/* The homing values, checked at the conversion they will run under: the speed
 * below the tick ceiling, the adjust and the datum whole pulses inside int32. */
static PulDirRefusal home_values(const PulDir* a, float speed, float adjust, float zero, float* cap)
{
    const float c = a->pulses_per_unit;
    const float z = roundf(zero * c), d = roundf(adjust * c);
    *cap = puldir_rate_cap(c);
    if (!(speed < *cap))
        return PULDIR_R_RATE;
    if (!(z >= -PULDIR_PULSES_MAX && z < PULDIR_PULSES_MAX && d > -PULDIR_PULSES_MAX && d < PULDIR_PULSES_MAX))
        return PULDIR_R_ARG;
    return PULDIR_R_NONE;
}

/* D16: direction +-1, a finite positive speed under the ceiling, a switch, a
 * timeout of at least one tick and at most half the counter; refused while homing. */
/* The homing offset alone: the distance moved after the switch releases, before the
 * datum is declared. Accepted in every mode, homing included -- a run in progress takes
 * it when it reaches its adjust, so a change after that applies from the next run. Under
 * the same checks as setHoming's adjust; refused before setHoming. */
bool PulDir_setHomingOffset(PulDir* a, float adjust)
{
    PulDirHoming* h = &a->home;
    PulDirRefusal why = PULDIR_R_NONE;
    float cap = 0.0f;
    if (h->switch_ptr == NULL)
        why = PULDIR_R_NO_HOMING;
    else if (!isfinite(adjust))
        why = PULDIR_R_ARG;
    else
        why = home_values(a, h->speed, adjust, h->zero, &cap);
    if (why != PULDIR_R_NONE) {
        puldir_refuse(a, "setHomingOffset", why, 0.0f);
        return false;
    }
    port_lock();
    h->adjust = adjust;
    port_unlock();
    return true;
}

float PulDir_homingOffset(PulDir* a) { return a->home.adjust; }

/* Make the spot the axis stands on its home. A homing run ends its adjust at the datum,
 * zero_pos, so a spot `here - zero` further on is that much more adjust: the offset grows
 * by it, and the next run lands here. The position is declared zero_pos now, so the frame
 * already agrees. Needs a homed axis -- only that relates the count to the switch -- and
 * an idle one. Nothing changes on a refusal. */
bool PulDir_setHomeHere(PulDir* a)
{
    PulDirHoming* h = &a->home;
    PulDirRefusal why = PULDIR_R_NONE;
    float cap = 0.0f;
    float adjust = 0.0f;
    if (h->switch_ptr == NULL)
        why = PULDIR_R_NO_HOMING;
    else if (!PulDir_isHomed(a))
        why = PULDIR_R_NOT_HOMED;
    else if (a->mode != PULDIR_MODE_IDLE)
        why = PULDIR_R_NOT_IDLE;
    else {
        adjust = h->adjust + (PulDir_position(a) - h->zero);
        why = home_values(a, h->speed, adjust, h->zero, &cap);
    }
    if (why != PULDIR_R_NONE) {
        puldir_refuse(a, "setHomeHere", why, 0.0f);
        return false;
    }
    if (!PulDir_setPosition(a, h->zero))   /* it refuses and logs if it moved meanwhile */
        return false;
    port_lock();
    h->adjust = adjust;
    port_unlock();
    return true;
}

bool PulDir_setHoming(PulDir* a, int direction, float speed, float adjust,
                      const uint8_t* switch_ptr, float zero_pos, uint32_t phase_timeout_ms)
{
    PulDirHoming* h = &a->home;
    PulDirRefusal why = PULDIR_R_NONE;
    float cap = 0.0f;
    if ((direction != 1 && direction != -1) || !(isfinite(speed) && speed > 0.0f) || switch_ptr == NULL
        || puldir_ms_ticks(phase_timeout_ms) == 0u || !puldir_ms_fits(phase_timeout_ms)
        || !isfinite(adjust) || !isfinite(zero_pos))
        why = PULDIR_R_ARG;
    else
        why = home_values(a, speed, adjust, zero_pos, &cap);
    if (why == PULDIR_R_NONE) {
        port_lock();
        if (a->mode == PULDIR_MODE_HOMING) {
            why = PULDIR_R_BUSY;
        } else {
            h->dir = (int8_t)direction;
            h->speed = speed;
            h->adjust = adjust;
            h->zero = zero_pos;
            h->switch_ptr = switch_ptr;
            h->timeout_ticks = puldir_ms_ticks(phase_timeout_ms);
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
    PulDirHoming* h = &a->home;
    PulDirRefusal why = h->dir == 0 ? PULDIR_R_NO_HOMING : PULDIR_R_NONE;
    if (why == PULDIR_R_NONE) {
        port_lock();
        if (a->slot == PULDIR_MAX_AXES) {
            why = PULDIR_R_UNREGISTERED;    /* as puldir_motion_check: the ticks cannot run it */
        } else if (puldir_fault_latched(a)) {
            why = PULDIR_R_FAULT;
        } else if (a->mode != PULDIR_MODE_IDLE) {
            why = PULDIR_R_MODE;
        } else if (a->moving) {
            why = PULDIR_R_NOT_IDLE;
        } else {
            h->homed = false;
            h->saved = a->settings;
            a->req |= REQ_HOME;
            a->mode = PULDIR_MODE_HOMING;
            a->moving = true;
            h->state = PULDIR_HOME_MOVE_TOWARDS_LIMIT;
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
    puldir_home_cancel(a);
    port_unlock();
}

PulDirHomeState PulDir_homeState(PulDir* a) { return a->home.state; }
bool PulDir_isHomed(PulDir* a)              { return a->home.homed; }

void puldir_home_restore(PulDir* a)
{
    a->settings = a->home.saved;
    a->req |= REQ_SETTINGS;
}

void puldir_home_cancel(PulDir* a)
{
    a->home.state = PULDIR_HOME_IDLE;
}

/* ---- the plan tick's part ----------------------------------------------------------------- */

void puldir_home_fault(PulDir* a)
{
    PulDirHoming* h = &a->home;
    h->homed = false;
    if (a->mode == PULDIR_MODE_HOMING) {
        h->fail = h->state;
        h->state = PULDIR_HOME_ERROR;
        a->events |= PULDIR_EV_HOME_FAILED;
    }
}

/* A phase timed out: the run ends unhomed, the pulses stop now. The next plan tick
 * applies the halt's REQ_HALT and REQ_SETTINGS (the settings homeStart found) to the generator. */
static void home_fail(PulDir* a, PulDirInc* out, int64_t pb)
{
    a->home.fail = a->home.state;
    a->home.state = PULDIR_HOME_ERROR;
    a->events |= PULDIR_EV_HOME_FAILED;
    port_lock();
    puldir_halt_locked(a);
    port_unlock();
    /* The requests it left apply next tick; the generator stops and takes its settings back now. */
    puldir_halt_generator(a, pb);
    puldir_take_settings(a, a->home.saved);
    puldir_set_inc(a, out, 0);
}

/* A new goal for the generator (E17). */
static void home_course(PulDir* a)
{
    a->arrived = false;
    a->settle = 0;
}

/* A moving phase: the homing speed toward the switch, or 0.2x back, as the speed setting, as
 * cFlexyStepper's jog did. The generator is at rest, so only the speed changes. */
static void home_jog(PulDir* a, bool away)
{
    const PulDirHoming* h = &a->home;
    const float speed = away ? 0.2f * h->speed : h->speed;
    MotionSettings s = a->settings;
    s.speed = speed;
    puldir_take_settings(a, s);
    const float dir = away ? -(float)h->dir : (float)h->dir;
    (void)MotionProfile_setEndless(&a->profile, dir * speed * a->pulses_per_unit);   /* finite: homeStart checked it */
    home_course(a);
}

void puldir_home_hold(PulDir* a, uint32_t now)
{
    a->home.t0 = now;
}

void puldir_home_begin(PulDir* a, uint32_t now)
{
    home_jog(a, false);
    a->home.state = PULDIR_HOME_MOVE_TOWARDS_LIMIT;
    a->home.t0 = now;
}

/* DELAY3: the datum, as PulDir_setPosition sets it. The axis must be at rest:
 * no increment and the pin low, read again under the lock (puldir_set_datum). */
static bool home_datum(PulDir* a, PulDirInc* out)
{
    const int32_t c = (int32_t)roundf(a->home.zero * a->pulses_per_unit);   /* checked by homeStart */
    PulDirSnap s;
    port_lock();
    puldir_pulse_read(a->slot, 0u, &s);
    const bool rest = s.inc == 0 && !s.high;
    if (rest)
        puldir_set_datum(a->slot, c);
    port_unlock();
    if (!rest)
        return false;
    a->origin = c;
    a->target = (uint64_t)(uint32_t)c << 32;
    (void)MotionProfile_setPosition(&a->profile, 0.0f);   /* done: ADJUST ended at rest */
    a->home.homed = true;
    a->home.state = PULDIR_HOME_IDLE;
    a->mode = PULDIR_MODE_IDLE;
    a->moving = false;
    a->velocity = 0.0f;
    a->events |= PULDIR_EV_HOMED;
    puldir_set_inc(a, out, 0);
    return true;
}

/* The switch is read before the generator advances, so a switch seen now stops the aim published
 * now. A phase timeout halts the axis (2.2). */
void puldir_home_run(PulDir* a, PulDirInc* out, uint64_t base, int64_t pb, int32_t count,
                     int32_t live, bool pul_high, uint32_t now)
{
    PulDirHoming* h = &a->home;
    MotionProfile* p = &a->profile;
    const uint8_t sw = *(const volatile uint8_t*)h->switch_ptr;
    const uint32_t in_phase = now - h->t0;
    const bool late = in_phase >= h->timeout_ticks;
    PulDirHomeState next = h->state;
    switch (h->state) {
    case PULDIR_HOME_MOVE_TOWARDS_LIMIT:
    case PULDIR_HOME_MOVE_AWAY_FROM_LIMIT: {
        const bool towards = h->state == PULDIR_HOME_MOVE_TOWARDS_LIMIT;
        if (sw == (towards ? 1u : 0u)) {
            MotionProfile_halt(p);              /* stops on the aim published last interval */
            home_course(a);
            next = towards ? PULDIR_HOME_DELAY1 : PULDIR_HOME_DELAY2;
        } else if (late) {
            home_fail(a, out, pb);
            return;
        }
        break;
    }
    case PULDIR_HOME_DELAY1:
        if (in_phase >= DELAY_TICKS) {
            home_jog(a, true);
            next = PULDIR_HOME_MOVE_AWAY_FROM_LIMIT;
        }
        break;
    case PULDIR_HOME_DELAY2:
        if (in_phase >= DELAY_TICKS) {
            const int64_t t = (int64_t)count + (int64_t)roundf(h->adjust * a->pulses_per_unit);
            if (t < INT32_MIN || t > INT32_MAX) {
                home_fail(a, out, pb);
                return;
            }
            puldir_take_settings(a, a->defaults); /* the defaults, for the adjust */
            a->target = (uint64_t)t << 32;
            (void)MotionProfile_setTarget(p, puldir_to_pulses((int64_t)(a->target - base)));
            home_course(a);
            next = PULDIR_HOME_ADJUST_POSITION;
        }
        break;
    case PULDIR_HOME_ADJUST_POSITION:
        if (late) {
            home_fail(a, out, pb);
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
    if (next != h->state) {
        h->state = next;
        h->t0 = now;
    }
    /* The adjust ends through the usual arrival path (E13, D19). */
    if (puldir_run_generator(a, out, base, pb, live, pul_high) && h->state == PULDIR_HOME_ADJUST_POSITION) {
        h->state = PULDIR_HOME_DELAY3;
        h->t0 = now;
    }
}
