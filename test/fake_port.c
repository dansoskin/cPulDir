/* The fake port: pins are an array, the ticks are called by the test. It
 * checks the concurrency rules of 5.9 that a host can check. */
#include <assert.h>
#include <stddef.h>
#include "fake_port.h"
#include "puldir.h"

uint32_t fake_plan_phase = PULDIR_TICKS_PER_PLAN / 2u;
uint32_t fake_preempt;

static bool level[FAKE_PINS];
static bool motion[FAKE_PINS];
static fake_write_fn on_write;
static void (*pulse_cb)(bool overrun);
static void (*plan_cb)(void);
static uint32_t since_start;            /* pulse ticks run since port_start */
static int tick_depth;                  /* tick callbacks in progress (a pulse tick may preempt the plan tick) */
static int lock_depth;
static bool main_locked;                /* the lock was taken outside every tick */

puldir_pin_t fake_pin(uint8_t id)
{
    assert(id < FAKE_PINS);
    puldir_pin_t p = { id };
    return p;
}

bool fake_level(uint8_t id)                  { assert(id < FAKE_PINS); return level[id]; }
void fake_set_input(uint8_t id, bool lvl)    { assert(id < FAKE_PINS); level[id] = lvl; }
void fake_mark_motion_pin(uint8_t id)        { assert(id < FAKE_PINS); motion[id] = true; }
void fake_on_write(fake_write_fn fn)         { on_write = fn; }

void port_pin_init_output(puldir_pin_t p, bool lvl) { assert(p.id < FAKE_PINS); level[p.id] = lvl; }
void port_pin_init_input(puldir_pin_t p, bool pullup) { assert(p.id < FAKE_PINS); (void)pullup; }

void port_pin_write(puldir_pin_t p, bool lvl)
{
    assert(p.id < FAKE_PINS);
    /* Main context may write only the enable and fault-clear pins (D17). */
    assert(!(lock_depth > 0 && main_locked && motion[p.id]));
    level[p.id] = lvl;
    if (on_write)
        on_write(PulDir_ticks(), p.id, lvl);
}

bool port_pin_read(puldir_pin_t p)
{
    assert(p.id < FAKE_PINS);
    const bool lvl = level[p.id];
    /* A pulse tick that came due while the lock is held would fire on unlock;
     * the fake only preempts unlocked reads. */
    if (lock_depth == 0)
        for (uint32_t i = 0; i < fake_preempt; i++)
            fake_pulse(false);
    return lvl;
}

void port_start(void (*pulse_tick)(bool overrun), void (*plan_tick)(void))
{
    pulse_cb = pulse_tick;
    plan_cb = plan_tick;
    since_start = 0;
}

void port_lock(void)
{
    assert(lock_depth == 0);
    lock_depth++;
    main_locked = tick_depth == 0;
}

void port_unlock(void)
{
    assert(lock_depth == 1);
    lock_depth--;
}

void fake_pulse(bool overrun)
{
    assert(pulse_cb != NULL);
    assert(lock_depth == 0);            /* the lock masks this interrupt */
    tick_depth++;
    pulse_cb(overrun);
    tick_depth--;
    since_start++;
}

void fake_plan(void)
{
    assert(plan_cb != NULL);
    assert(lock_depth == 0);
    assert(tick_depth == 0);            /* the plan tick never preempts the pulse tick */
    tick_depth++;
    plan_cb();
    tick_depth--;
}

void fake_run(uint32_t ticks)
{
    for (uint32_t i = 0; i < ticks; i++) {
        fake_pulse(false);
        if (since_start % PULDIR_TICKS_PER_PLAN == fake_plan_phase)
            fake_plan();
    }
}
