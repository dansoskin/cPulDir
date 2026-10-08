/* Enable and release (5.8): the pin writes and the two delays; the caller holds any lock. */
#include "puldir_enable.h"
#include "puldir_port.h"

void puldir_enable_attach(PulDirEnable* e, puldir_pin_t pin, bool inverted)
{
    e->pin = pin;
    e->inverted = inverted;
    e->attached = true;
    e->enabled = false;
    port_pin_write(pin, inverted);
}

void puldir_enable_set_delays(PulDirEnable* e, uint32_t enable_ticks, uint32_t release_ticks)
{
    e->enable_ticks = enable_ticks;
    e->release_ticks = release_ticks;
}

void puldir_enable_set_auto_release(PulDirEnable* e, bool on)
{
    e->auto_release = on;
    if (!on)
        e->release_due = false;
}

void puldir_enable_wake(PulDirEnable* e, uint32_t now)
{
    e->release_due = false;
    e->held = false;
    e->off_due = false;
    if (e->attached && !e->enabled) {
        port_pin_write(e->pin, !e->inverted);
        e->enabled = true;
        e->enabled_at = now;
        e->wait = true;                     /* a service lifts it once the enable delay has passed */
    }
}

/* A pulse still high is not cut short: the service lowers the enable pin after it. */
void puldir_enable_off(PulDirEnable* e, bool pul_high)
{
    if (pul_high)
        e->off_due = true;
    else
        port_pin_write(e->pin, e->inverted);
    e->enabled = false;
    e->wait = false;
    e->release_due = false;
}

void puldir_enable_service(PulDirEnable* e, bool pul_high, uint32_t now)
{
    if (e->wait && now - e->enabled_at >= e->enable_ticks)
        e->wait = false;
    if (e->off_due && !pul_high) {          /* disabled: the last pulse is lowered */
        port_pin_write(e->pin, e->inverted);
        e->off_due = false;
    }
}

/* release_ticks after the axis went idle at the end of a move (D19). Never while a fault
 * latch holds it, until its next wake (D12). */
void puldir_enable_release(PulDirEnable* e, bool was_moving, bool idle, uint32_t now)
{
    if (was_moving && idle && e->auto_release && e->attached && e->enabled) {
        e->release_due = true;
        e->idle_at = now;
    }
    if (!e->release_due)
        return;
    if (!idle || e->held) {
        e->release_due = false;
    } else if (now - e->idle_at >= e->release_ticks) {
        port_pin_write(e->pin, e->inverted);
        e->enabled = false;
        e->wait = false;
        e->release_due = false;
    }
}

bool puldir_enable_delaying(const PulDirEnable* e)
{
    return e->wait;
}

void puldir_enable_hold(PulDirEnable* e)
{
    e->held = true;
}
