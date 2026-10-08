/* The drive-signal calls (5.8): the lock, the refusals and ms to ticks around drive/'s enable
 * and fault modules, from main context. */
#include "puldir_internal.h"

/* Setup, before PulDir_start: the drive is disabled until a command or PulDir_enable. */
void PulDir_attachEnable(PulDir* a, puldir_pin_t p, bool inverted)
{
    puldir_enable_attach(&a->enable, p, inverted);
}

bool PulDir_setDelays(PulDir* a, uint32_t enable_ms, uint32_t release_ms)
{
    if (!puldir_ms_fits(enable_ms) || !puldir_ms_fits(release_ms)) {
        puldir_refuse(a, "setDelays", PULDIR_R_ARG, 0.0f);
        return false;
    }
    const uint32_t on = puldir_ms_ticks(enable_ms), off = puldir_ms_ticks(release_ms);
    port_lock();
    puldir_enable_set_delays(&a->enable, on, off);
    port_unlock();
    return true;
}

void PulDir_setAutoRelease(PulDir* a, bool on)
{
    port_lock();
    puldir_enable_set_auto_release(&a->enable, on);
    port_unlock();
}

/* 5.8: enabling arms the enable delay; disabling halts first, and a pulse still high is not cut
 * short: the plan tick lowers the enable pin after it. Refused without an enable pin. */
bool PulDir_enable(PulDir* a, bool on)
{
    if (!a->enable.attached) {
        puldir_refuse(a, "enable", PULDIR_R_NO_PIN, 0.0f);
        return false;
    }
    if (!on && (a->moving || a->mode != PULDIR_MODE_IDLE))
        PulDir_halt(a);
    port_lock();
    if (on) {
        puldir_enable_wake(&a->enable, PulDir_ticks());
    } else {
        PulDirSnap s = { 0, 0, 0, 1, false };
        if (a->slot < PULDIR_MAX_AXES)
            puldir_pulse_read(a->slot, 0u, &s);
        puldir_enable_off(&a->enable, s.high);
    }
    port_unlock();
    return true;
}

/* Setup, before PulDir_start, after PulDir_init has fixed the timing. */
void PulDir_attachFault(PulDir* a, puldir_pin_t p, bool inverted)
{
    puldir_fault_attach(&a->fault, p, inverted, puldir_ms_ticks(PULDIR_FAULT_DEBOUNCE_MS));
}

void PulDir_attachFaultClear(PulDir* a, puldir_pin_t p, bool inverted)
{
    puldir_fault_attach_clear(&a->fault, p, inverted, puldir_ms_ticks(PULDIR_FAULT_CLEAR_MS));
}

/* Raises the clear pin and returns; the plan tick lowers it after
 * PULDIR_FAULT_CLEAR_MS and lifts the latch (5.8). Nothing blocks. */
bool PulDir_clearFault(PulDir* a)
{
    PulDirRefusal why = PULDIR_R_NONE;
    if (!a->fault.clr_attached) {
        why = PULDIR_R_NO_PIN;
    } else {
        port_lock();
        if (!puldir_fault_clear(&a->fault, PulDir_ticks()))
            why = PULDIR_R_BUSY;
        port_unlock();
    }
    if (why != PULDIR_R_NONE) {
        puldir_refuse(a, "clearFault", why, 0.0f);
        return false;
    }
    return true;
}
