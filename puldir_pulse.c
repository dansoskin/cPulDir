/* The pulse tick (guidelines 5.3): integer only, edges at integer crossings
 * (D3), increment sets double-buffered and latched at boundaries (D5). */
#include "puldir_internal.h"

PulDir* puldir_axes[PULDIR_MAX_AXES];
uint8_t puldir_naxes;

/* The pulse tick's own state per slot, compact for the hot loop. Other
 * contexts read it only through the accessors below, under the lock. */
typedef struct {
    uint64_t past;              /* 32.32 position minus edge(); unsigned for defined wrap */
    uint32_t down;              /* 0 moving up, all ones moving down */
    uint32_t count;             /* edges emitted; int32_t to everyone else */
    int32_t  inc;               /* live increment */
    int8_t   dir;               /* live direction */
    int8_t   dir_now;           /* the direction the pin shows */
    bool     pul_high;
    bool     pul_active_low, dir_inverted;
    puldir_pin_t pul, dir_pin;
} Axis;

static Axis ax[PULDIR_MAX_AXES];
static PulDirInc sets[2][PULDIR_MAX_AXES];
static volatile uint8_t pub;            /* index of the last published set */
static volatile uint8_t seq, latched;   /* publish count; publish count last latched */
static volatile uint32_t due;           /* the tick the published set is due at: its boundary */
static volatile uint32_t ticks = PULDIR_TICK_START;
static volatile uint32_t left;          /* ticks left in this interval, ticks_per_plan at a boundary */
static volatile uint32_t overruns;
static volatile uint32_t late_sets;     /* sets latched after their boundary: published too late */

/* ---- timing: the port's attach calls set it, the first PulDir_init fixes it ---- */

PulDirTiming puldir_timing;

static bool timing_open(void) { return !puldir_timing.set && !puldir_timing.started; }

bool puldir_timing_pulse(uint32_t pulse_us)
{
    if (!timing_open() || pulse_us < 2u || pulse_us > 1000u)
        return false;
    puldir_timing.pulse_us = pulse_us;
    return true;
}

bool puldir_timing_plan(uint32_t plan_us)
{
    if (!timing_open() || plan_us < 250u || plan_us > 10000u)  /* the generator's dt range; a 16-bit ARR */
        return false;
    puldir_timing.plan_us = plan_us;
    return true;
}

bool puldir_timing_take(void)
{
    PulDirTiming* t = &puldir_timing;
    if (!timing_open() || t->pulse_us == 0u || t->plan_us % t->pulse_us != 0u)
        return t->set;
    const uint32_t ticks_per_plan = t->plan_us / t->pulse_us;
    if (ticks_per_plan < 2u || ticks_per_plan > 4096u)
        return false;
    t->tick_hz = PULDIR_TIMER_HZ / t->pulse_us;
    t->ticks_per_plan = ticks_per_plan;
    t->dt = (float)t->plan_us / (float)PULDIR_TIMER_HZ;   /* a division rounds to the nearest float */
    left = ticks_per_plan;
    t->set = true;
    return true;
}

uint32_t puldir_ms_ticks(uint32_t ms)
{
    return (uint32_t)((uint64_t)ms * puldir_timing.tick_hz / 1000u);
}

/* A duration the plan tick can compare by unsigned subtraction. */
bool puldir_ms_fits(uint32_t ms)
{
    return (uint64_t)ms * puldir_timing.tick_hz / 1000u <= PULDIR_TICKS_MAX;
}

/* Fixes the timing even if no axis took it; the plan tick then does nothing. */
void PulDir_start(void)
{
    (void)puldir_timing_take();
    puldir_timing.started = true;
    port_start(puldir_pulse_tick, puldir_plan_tick);
}

/* The next edge, at count + dir_now (D3). Moving down it is one unit higher,
 * so the edge is reached when past turns >= 0 up, < 0 down: a sign test. */
static uint64_t edge(const Axis* a)
{
    return ((uint64_t)(a->count + (uint32_t)(int32_t)a->dir_now) << 32) + (a->dir_now < 0);
}

static uint64_t pos(const Axis* a) { return a->past + edge(a); }

static void aim(Axis* a, int8_t d)
{
    const uint64_t P = pos(a);
    a->dir_now = d;
    a->down = d > 0 ? 0u : 0xFFFFFFFFu;
    a->past = P - edge(a);
}

void puldir_pulse_tick(bool overrun)
{
    if (overrun)
        overruns++;
    const uint32_t n = puldir_naxes;
    const uint32_t ticks_per_plan = puldir_timing.ticks_per_plan;
    /* A boundary latches the whole set; so does any tick at or past the set's due tick, so a set
     * published late runs the rest of its interval rather than a whole one later. */
    if (latched != seq && (left == ticks_per_plan || (int32_t)(ticks - due) >= 0)) {
        const PulDirInc* s = sets[pub];
        for (uint32_t i = 0; i < n; i++) {
            ax[i].inc = s[i].inc;
            ax[i].dir = s[i].dir;
        }
        if ((int32_t)(ticks - due) > 0)
            late_sets++;
        latched = seq;
    }
    for (uint32_t i = 0; i < n; i++) {
        Axis* a = &ax[i];
        bool lowered = false;
        if (a->pul_high) {
            port_pin_write(a->pul, a->pul_active_low);
            a->pul_high = false;
            lowered = true;
        }
        const int8_t d = a->dir;
        if (d != a->dir_now) {           /* never on a tick with a pulse edge */
            if (!lowered) {
                port_pin_write(a->dir_pin, (d > 0) != a->dir_inverted);
                aim(a, d);
            }
            continue;
        }
        a->past += (uint64_t)(int64_t)a->inc;
        if ((int32_t)((uint32_t)(a->past >> 32) ^ a->down) >= 0) {
            port_pin_write(a->pul, !a->pul_active_low);
            a->pul_high = true;
            a->count += (uint32_t)(int32_t)d;
            a->past -= (uint64_t)(int64_t)d << 32;     /* the edge moves on by one pulse */
        }
    }
    ticks++;
    if (--left == 0)
        left = ticks_per_plan;
}

/* Before PulDir_start: take the next slot at rest, inc 0, dir +1, at the datum. */
void puldir_pulse_add(PulDir* p)
{
    const uint8_t i = puldir_naxes;
    const PulDirInc rest = { 0, 1 };
    sets[0][i] = sets[1][i] = rest;
    Axis* a = &ax[i];
    a->inc = 0;
    a->dir = a->dir_now = 1;
    a->down = 0;
    a->pul_high = false;
    /* The target is the datum PulDir_setPosition may have set before registration: whole in IDLE. */
    puldir_set_datum(i, (int32_t)(uint32_t)(p->target >> 32));
    a->pul = p->pul;
    a->dir_pin = p->dir;
    a->pul_active_low = p->pul_active_low;
    a->dir_inverted = p->dir_inverted;
    p->slot = i;
    puldir_axes[i] = p;
    puldir_naxes = (uint8_t)(i + 1u);
}

PulDirInc* puldir_shadow(void) { return sets[pub ^ 1u]; }

void puldir_publish(uint32_t due_tick)
{
    due = due_tick;
#if defined(__GNUC__)
    __asm__ volatile("" ::: "memory");  /* the shadow's stores and due land before the flip */
#endif
    pub ^= 1u;
    seq++;
}

void puldir_kill(uint8_t slot)
{
    ax[slot].inc = 0;
    sets[pub][slot].inc = 0;
}

uint32_t puldir_left(void) { return left; }

void puldir_pulse_read(uint8_t slot, uint32_t ahead, PulDirSnap* out)
{
    const Axis* a = &ax[slot];
    out->Pb = pos(a) + (uint64_t)((int64_t)a->inc * ahead);
    out->count = (int32_t)a->count;
    out->inc = a->inc;
    out->dir = a->dir_now;
    out->high = a->pul_high;
}

void puldir_set_datum(uint8_t slot, int32_t count)
{
    Axis* a = &ax[slot];
    a->count = (uint32_t)count;
    a->past = ((uint64_t)a->count << 32) - edge(a);
}

uint32_t PulDir_ticks(void)    { return ticks; }
uint32_t PulDir_overruns(void) { return overruns; }
uint32_t PulDir_lateSets(void) { return late_sets; }
