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
static volatile uint32_t ticks = PULDIR_TICK_START;
static volatile uint32_t left = PULDIR_TICKS_PER_PLAN;   /* ticks left in this interval */
static volatile uint32_t overruns;

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
    if (left == PULDIR_TICKS_PER_PLAN && latched != seq) {   /* a boundary: latch the whole set */
        const PulDirInc* s = sets[pub];
        for (uint32_t i = 0; i < n; i++) {
            ax[i].inc = s[i].inc;
            ax[i].dir = s[i].dir;
        }
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
        left = PULDIR_TICKS_PER_PLAN;
}

/* Before PulDir_start: take the next slot at rest, inc 0, dir +1. */
void puldir_pulse_add(PulDir* p)
{
    const uint8_t i = puldir_naxes;
    const PulDirInc rest = { 0, 1 };
    sets[0][i] = sets[1][i] = rest;
    Axis* a = &ax[i];
    a->count = 0;
    a->inc = 0;
    a->dir = a->dir_now = 1;
    a->down = 0;
    a->past = 0 - edge(a);              /* at position 0 */
    a->pul_high = false;
    a->pul = p->pul;
    a->dir_pin = p->dir;
    a->pul_active_low = p->pul_active_low;
    a->dir_inverted = p->dir_inverted;
    p->slot = i;
    puldir_axes[i] = p;
    puldir_naxes = (uint8_t)(i + 1u);
}

PulDirInc* puldir_shadow(void) { return sets[pub ^ 1u]; }

void puldir_publish(void)
{
#if defined(__GNUC__)
    __asm__ volatile("" ::: "memory");  /* the shadow's stores land before the flip */
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

void puldir_pulse_read(uint8_t slot, PulDirPulse* out)
{
    const Axis* a = &ax[slot];
    out->P = pos(a);
    out->count = (int32_t)a->count;
    out->inc = a->inc;
    out->dir = a->dir_now;
}

bool puldir_pul_high(uint8_t slot) { return ax[slot].pul_high; }

void puldir_set_datum(uint8_t slot, int32_t count)
{
    Axis* a = &ax[slot];
    a->count = (uint32_t)count;
    a->past = ((uint64_t)a->count << 32) - edge(a);
}

uint32_t PulDir_ticks(void)    { return ticks; }
uint32_t PulDir_overruns(void) { return overruns; }
