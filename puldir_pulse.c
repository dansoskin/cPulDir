/* The pulse tick (guidelines 5.3): integer only. Each axis accumulates its
 * live increment into a 32.32 position and raises its pulse pin when the
 * position reaches the next edge in the direction of travel (D3). Increment
 * sets are double-buffered and latched only at interval boundaries (D5). */
#include "puldir_internal.h"

PulDir* puldir_axes[PULDIR_MAX_AXES];
uint8_t puldir_naxes;

static PulDirInc sets[2][PULDIR_MAX_AXES];
static PulDirInc live[PULDIR_MAX_AXES];
static volatile uint8_t pub;            /* index of the last published set */
static volatile uint8_t seq, latched;   /* publish count; publish count last latched */
static volatile uint32_t ticks = PULDIR_TICK_START;
static volatile uint32_t left = PULDIR_TICKS_PER_PLAN;   /* ticks left in this interval */
static volatile uint32_t overruns;

static void write_pul(PulDir* a, bool active) { port_pin_write(a->pul, active != a->pul_active_low); }
static void write_dir(PulDir* a, int8_t d)    { port_pin_write(a->dir, (d > 0) != a->dir_inverted); }

void puldir_pulse_tick(bool overrun)
{
    if (overrun)
        overruns++;
    if (left == PULDIR_TICKS_PER_PLAN && latched != seq) {   /* a boundary: latch the whole set */
        const PulDirInc* s = sets[pub];
        for (uint8_t i = 0; i < puldir_naxes; i++)
            live[i] = s[i];
        latched = seq;
    }
    for (uint8_t i = 0; i < puldir_naxes; i++) {
        PulDir* a = puldir_axes[i];
        bool edge = false;
        if (a->pul_high) {
            write_pul(a, false);
            a->pul_high = false;
            edge = true;
        }
        const int8_t d = live[i].dir;
        if (d != a->dir_now) {           /* never on a tick with a pulse edge */
            if (!edge) {
                write_dir(a, d);
                a->dir_now = d;
                a->edge = ((uint64_t)(uint32_t)(a->count + d)) << 32;
            }
            continue;
        }
        a->P += (uint64_t)(int64_t)live[i].inc;
        const int64_t past = (int64_t)(a->P - a->edge);
        if (d > 0 ? past >= 0 : past <= 0) {
            write_pul(a, true);
            a->pul_high = true;
            a->count += d;
            a->edge += (uint64_t)(int64_t)d << 32;
        }
    }
    ticks++;
    if (--left == 0)
        left = PULDIR_TICKS_PER_PLAN;
}

/* Before PulDir_start: join the table with live and published entries at
 * inc 0, dir +1, matching dir_now and the parked direction pin. The entries
 * are written before the axis is counted in. */
void puldir_pulse_add(PulDir* a)
{
    const uint8_t i = puldir_naxes;
    const PulDirInc rest = { 0, 1 };
    live[i] = sets[0][i] = sets[1][i] = rest;
    a->slot = i;
    puldir_axes[i] = a;
    puldir_naxes = (uint8_t)(i + 1u);
}

/* The plan tick is the only publisher, so the two stores of publish cannot
 * interleave with another publish, and the pulse tick reads pub only at a boundary. */
PulDirInc* puldir_shadow(void)     { return sets[pub ^ 1u]; }
void       puldir_publish(void)    { pub ^= 1u; seq++; }

void puldir_kill(uint8_t slot)
{
    live[slot].inc = 0;
    sets[pub][slot].inc = 0;
}

uint32_t puldir_left(void)              { return left; }
int32_t  puldir_live_inc(uint8_t slot)  { return live[slot].inc; }

uint32_t PulDir_ticks(void)    { return ticks; }
uint32_t PulDir_overruns(void) { return overruns; }
