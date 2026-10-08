/* The fault input and its clear pulse (5.8): setup and the clear from main, the debounce and the
 * latch from the plan tick. The halt a latch causes is the plan tick's own. */
#include "puldir_internal.h"

#define DEBOUNCE_TICKS puldir_ms_ticks(PULDIR_FAULT_DEBOUNCE_MS)
#define CLEAR_TICKS    puldir_ms_ticks(PULDIR_FAULT_CLEAR_MS)

/* Setup, before PulDir_start. */
void PulDir_attachFault(PulDir* a, puldir_pin_t p, bool inverted)
{
    a->fault.pin = p;
    a->fault.inverted = inverted;
    a->fault.attached = true;
}

/* Setup, before PulDir_start: the output is parked inactive. */
void PulDir_attachFaultClear(PulDir* a, puldir_pin_t p, bool inverted)
{
    a->fault.clr_pin = p;
    a->fault.clr_inverted = inverted;
    a->fault.clr_attached = true;
    port_pin_write(p, inverted);
}

/* Raises the clear pin and returns; the plan tick lowers it after
 * PULDIR_FAULT_CLEAR_MS and lifts the latch (5.8). Nothing blocks. */
bool PulDir_clearFault(PulDir* a)
{
    PulDirFault* f = &a->fault;
    PulDirRefusal why = PULDIR_R_NONE;
    if (!f->clr_attached) {
        why = PULDIR_R_NO_PIN;
    } else {
        port_lock();
        if (f->clearing) {
            why = PULDIR_R_BUSY;
        } else {
            port_pin_write(f->clr_pin, !f->clr_inverted);
            f->clearing = true;
            f->clr_t0 = PulDir_ticks();
        }
        port_unlock();
    }
    if (why != PULDIR_R_NONE) {
        puldir_refuse(a, "clearFault", why, 0.0f);
        return false;
    }
    return true;
}

/* ---- the plan tick's part ----------------------------------------------------------------- */

bool puldir_fault_sample(const PulDir* a)
{
    return a->fault.attached && port_pin_read(a->fault.pin) != a->fault.inverted;
}

bool puldir_fault_service(PulDir* a, bool active, uint32_t t_pin, uint32_t now)
{
    PulDirFault* f = &a->fault;
    /* The clear pulse ends. This sample was taken before the pin was lowered, so the latch holds
     * until a later one reads inactive; a drive still in alarm stays latched, uncounted. */
    if (f->clearing && now - f->clr_t0 >= CLEAR_TICKS) {
        port_pin_write(f->clr_pin, f->clr_inverted);
        f->clearing = false;
        f->clr_armed = f->latched;
        return false;
    }

    if (!active) {
        f->seen = false;
        if (f->clr_armed) {
            f->clr_armed = false;
            f->latched = false;
            a->events |= PULDIR_EV_FAULT_CLEARED;
        }
        return false;
    }
    if (!f->seen) {
        f->seen = true;
        f->since = t_pin;
        return false;
    }
    if (f->latched || t_pin - f->since < DEBOUNCE_TICKS)
        return false;

    f->latched = true;
    f->count++;
    f->last_tick = t_pin;
    a->events |= PULDIR_EV_FAULT;
    return true;
}
