/* The drive modules alone: built from drive/'s two files and the fake port with no puldir core,
 * which proves they stand alone. One tick is 10 us here. */
#ifdef NDEBUG
#error "the tests call the code under test inside assert(): build without NDEBUG"
#endif
#include <assert.h>
#include "fake_port.h"
#include "drive/puldir_enable.h"
#include "drive/puldir_fault.h"

#define MS 100u

/* The fake port's three calls into the core, which this build does not link. */
uint32_t PulDir_ticks(void) { return 0u; }
bool puldir_timing_pulse(uint32_t us) { (void)us; return true; }
bool puldir_timing_plan(uint32_t us)  { (void)us; return true; }

static PulDirLatch step(PulDirFault* f, uint32_t t)
{
    return puldir_fault_service(f, puldir_fault_sample(f), t, t);
}

static void fault(void)
{
    PulDirFault f = { 0 };
    puldir_fault_attach(&f, fake_pin(1), false, 2u * MS);
    puldir_fault_attach_clear(&f, fake_pin(2), false, 10u * MS);
    assert(!fake_level(2));

    /* A 1.9 ms glitch does not latch; 2 ms does. */
    uint32_t t = 0;
    fake_set_input(1, true);
    for (; t < 19u * MS / 10u; t++)
        assert(step(&f, t) == PULDIR_LATCH_SAME);
    fake_set_input(1, false);
    assert(step(&f, t++) == PULDIR_LATCH_SAME && !puldir_fault_latched(&f));
    fake_set_input(1, true);
    const uint32_t on = t;
    for (; step(&f, t) == PULDIR_LATCH_SAME; t++)
        assert(t - on < 2u * MS);
    assert(t - on == 2u * MS && puldir_fault_latched(&f) && f.count == 1u && f.last_tick == t);

    /* The clear pulse ends after 10 ms; the latch drops only once the input reads inactive. */
    const uint32_t c0 = ++t;
    assert(puldir_fault_clear(&f, c0) && fake_level(2) && !puldir_fault_clear(&f, c0));
    for (; t - c0 < 10u * MS; t++)
        assert(step(&f, t) == PULDIR_LATCH_SAME && fake_level(2));
    assert(step(&f, t++) == PULDIR_LATCH_SAME && !fake_level(2) && puldir_fault_latched(&f));
    assert(step(&f, t++) == PULDIR_LATCH_SAME && puldir_fault_latched(&f));   /* still in alarm */
    fake_set_input(1, false);
    assert(step(&f, t) == PULDIR_LATCH_DROPPED && !puldir_fault_latched(&f) && f.count == 1u);
}

static void enable(void)
{
    PulDirEnable e = { 0 };
    puldir_enable_attach(&e, fake_pin(3), true);
    assert(fake_level(3) && !e.enabled);
    puldir_enable_set_delays(&e, 50u * MS, 200u * MS);
    puldir_enable_set_auto_release(&e, true);

    /* The enable delay holds the motion for 50 ms. */
    puldir_enable_wake(&e, 1000u);
    assert(!fake_level(3) && e.enabled && puldir_enable_delaying(&e));
    puldir_enable_service(&e, false, 1000u + 50u * MS - 1u);
    assert(puldir_enable_delaying(&e));
    puldir_enable_service(&e, false, 1000u + 50u * MS);
    assert(!puldir_enable_delaying(&e));

    /* The release delay: 200 ms after the move ended. */
    puldir_enable_release(&e, true, true, 20000u);
    puldir_enable_release(&e, false, true, 20000u + 200u * MS - 1u);
    assert(e.enabled && !fake_level(3));
    puldir_enable_release(&e, false, true, 20000u + 200u * MS);
    assert(!e.enabled && fake_level(3));

    /* A hold after a fault suppresses the release. */
    puldir_enable_wake(&e, 50000u);
    puldir_enable_hold(&e);
    puldir_enable_release(&e, true, true, 50000u);
    puldir_enable_release(&e, false, true, 50000u + 200u * MS);
    assert(e.enabled && !fake_level(3));
}

int main(void)
{
    fault();
    enable();
    return 0;
}
