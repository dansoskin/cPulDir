/* Enable and release (5.8): setup and commands from main, the delays from the plan tick. */
#include "puldir_internal.h"

/* Setup, before PulDir_start: the pin is parked inactive; the drive is disabled until a command
 * or PulDir_enable. */
void PulDir_attachEnable(PulDir* a, puldir_pin_t p, bool inverted)
{
    PulDirEnable* e = &a->enable;
    e->pin = p;
    e->inverted = inverted;
    e->attached = true;
    e->enabled = false;
    port_pin_write(p, inverted);
}

bool PulDir_setDelays(PulDir* a, uint32_t enable_ms, uint32_t release_ms)
{
    if (!puldir_ms_fits(enable_ms) || !puldir_ms_fits(release_ms)) {
        puldir_refuse(a, "setDelays", PULDIR_R_ARG, 0.0f);
        return false;
    }
    port_lock();
    a->enable.enable_ticks = puldir_ms_ticks(enable_ms);
    a->enable.release_ticks = puldir_ms_ticks(release_ms);
    port_unlock();
    return true;
}

void PulDir_setAutoRelease(PulDir* a, bool on)
{
    port_lock();
    a->enable.auto_release = on;
    if (!on)
        a->enable.release_due = false;
    port_unlock();
}

void puldir_enable_wake(PulDir* a)
{
    PulDirEnable* e = &a->enable;
    e->release_due = false;
    e->fault_hold = false;
    e->off_due = false;
    if (e->attached && !e->enabled) {
        port_pin_write(e->pin, !e->inverted);
        e->enabled = true;
        e->enabled_at = PulDir_ticks();
        e->wait = true;                     /* the plan tick lifts it once the enable delay has passed */
    }
}

/* 5.8: enabling arms the enable delay; disabling halts first, and a pulse still high is not cut
 * short: the plan tick lowers the enable pin after it. Refused without an enable pin. */
bool PulDir_enable(PulDir* a, bool on)
{
    PulDirEnable* e = &a->enable;
    if (!e->attached) {
        puldir_refuse(a, "enable", PULDIR_R_NO_PIN, 0.0f);
        return false;
    }
    if (!on && (a->moving || a->mode != PULDIR_MODE_IDLE))
        PulDir_halt(a);
    port_lock();
    if (on) {
        puldir_enable_wake(a);
    } else {
        PulDirSnap s = { 0, 0, 0, 1, false };
        if (a->slot < PULDIR_MAX_AXES)
            puldir_pulse_read(a->slot, 0u, &s);
        if (s.high)
            e->off_due = true;
        else
            port_pin_write(e->pin, e->inverted);
        e->enabled = false;
        e->wait = false;
        e->release_due = false;
    }
    port_unlock();
    return true;
}

/* ---- the plan tick's part ----------------------------------------------------------------- */

void puldir_enable_service(PulDir* a, bool pul_high, uint32_t now)
{
    PulDirEnable* e = &a->enable;
    if (e->wait && now - e->enabled_at >= e->enable_ticks)
        e->wait = false;
    if (e->off_due && !pul_high) {          /* enable(false): the last pulse is lowered */
        port_pin_write(e->pin, e->inverted);
        e->off_due = false;
    }
}

/* release_ticks after the axis went IDLE at the end of a move (D19). Never on an axis a fault
 * latch halted, or its group's members, until its next accepted command (D12). */
void puldir_enable_release(PulDir* a, bool was_moving, uint32_t now)
{
    PulDirEnable* e = &a->enable;
    if (was_moving && !a->moving && a->mode == PULDIR_MODE_IDLE && e->auto_release
        && e->attached && e->enabled) {
        e->release_due = true;
        e->idle_at = now;
    }
    if (!e->release_due)
        return;
    if (a->moving || a->mode != PULDIR_MODE_IDLE || e->fault_hold) {
        e->release_due = false;
    } else if (now - e->idle_at >= e->release_ticks) {
        port_pin_write(e->pin, e->inverted);
        e->enabled = false;
        e->wait = false;
        e->release_due = false;
    }
}

bool puldir_enable_delaying(const PulDir* a)
{
    return a->enable.wait;
}

void puldir_enable_set_fault_hold(PulDir* a)
{
    a->enable.fault_hold = true;
}
