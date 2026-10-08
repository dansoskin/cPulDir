/* The fault input and its clear pulse (5.8): the caller holds any lock and halts what a latch stops. */
#include "puldir_fault.h"
#include "puldir_port.h"

void puldir_fault_attach(PulDirFault* f, puldir_pin_t pin, bool inverted, uint32_t debounce_ticks)
{
    f->pin = pin;
    f->inverted = inverted;
    f->attached = true;
    f->debounce_ticks = debounce_ticks;
}

void puldir_fault_attach_clear(PulDirFault* f, puldir_pin_t pin, bool inverted, uint32_t clear_ticks)
{
    f->clr_pin = pin;
    f->clr_inverted = inverted;
    f->clr_attached = true;
    f->clear_ticks = clear_ticks;
    port_pin_write(pin, inverted);
}

bool puldir_fault_clear(PulDirFault* f, uint32_t now)
{
    if (f->clearing)
        return false;
    port_pin_write(f->clr_pin, !f->clr_inverted);
    f->clearing = true;
    f->clr_t0 = now;
    return true;
}

bool puldir_fault_latched(const PulDirFault* f)
{
    return f->latched;
}

bool puldir_fault_sample(const PulDirFault* f)
{
    return f->attached && port_pin_read(f->pin) != f->inverted;
}

PulDirLatch puldir_fault_service(PulDirFault* f, bool active, uint32_t t_pin, uint32_t now)
{
    /* The clear pulse ends. This sample was taken before the pin was lowered, so the latch holds
     * until a later one reads inactive; a drive still in alarm stays latched, uncounted. */
    if (f->clearing && now - f->clr_t0 >= f->clear_ticks) {
        port_pin_write(f->clr_pin, f->clr_inverted);
        f->clearing = false;
        f->clr_armed = f->latched;
        return PULDIR_LATCH_SAME;
    }

    if (!active) {
        f->seen = false;
        if (!f->clr_armed)
            return PULDIR_LATCH_SAME;
        f->clr_armed = false;
        f->latched = false;
        return PULDIR_LATCH_DROPPED;
    }
    if (!f->seen) {
        f->seen = true;
        f->since = t_pin;
        return PULDIR_LATCH_SAME;
    }
    if (f->latched || t_pin - f->since < f->debounce_ticks)
        return PULDIR_LATCH_SAME;

    f->latched = true;
    f->count++;
    f->last_tick = t_pin;
    return PULDIR_LATCH_CLOSED;
}
