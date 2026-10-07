/* Axis tests (guidelines section 8, tests 10 to 24). One case per process
 * (D9): `test_puldir list` prints the names, `test_puldir <name>` runs one. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "fake_port.h"
#include "puldir_internal.h"

#define N PULDIR_TICKS_PER_PLAN

/* ---- the pin-write log ---------------------------------------------------- */

typedef struct { uint32_t tick; uint8_t id; bool level; } Write;
#define MAX_WRITES 200000u
static Write wlog[MAX_WRITES];
static size_t nw;

static void on_write(uint32_t tick, uint8_t id, bool level)
{
    assert(nw < MAX_WRITES);
    wlog[nw].tick = tick;
    wlog[nw].id = id;
    wlog[nw].level = level;
    nw++;
}

/* ---- axes on fake pins: pulse pin 2i, direction pin 2i+1 ------------------- */

static PulDir ax[PULDIR_MAX_AXES];

static void add_axis(uint8_t i, bool pul_active_low, bool dir_inverted)
{
    char name[8];
    snprintf(name, sizeof name, "a%u", (unsigned)i);
    PulDir_init(&ax[i], name);
    fake_mark_motion_pin((uint8_t)(2u * i));
    fake_mark_motion_pin((uint8_t)(2u * i + 1u));
    PulDir_attachPins(&ax[i], fake_pin((uint8_t)(2u * i)), fake_pin((uint8_t)(2u * i + 1u)),
                      pul_active_low, dir_inverted);
    assert(PulDir_register(&ax[i]));
    assert(ax[i].slot == i);
}

static void start(void)
{
    fake_on_write(on_write);
    PulDir_start();
}

static void setup(uint8_t k)
{
    for (uint8_t i = 0; i < k; i++)
        add_axis(i, false, false);
    start();
}

/* ---- driving increments by hand -------------------------------------------- */

static uint32_t left(void)
{
    port_lock();
    const uint32_t l = puldir_left();
    port_unlock();
    return l;
}

static int32_t live_inc(uint8_t slot)
{
    port_lock();
    const int32_t inc = puldir_live_inc(slot);
    port_unlock();
    return inc;
}

/* Write every registered axis's shadow entry, then publish. */
static void publish_all(const int32_t* inc, const int8_t* dir)
{
    PulDirInc* s = puldir_shadow();
    for (uint8_t i = 0; i < puldir_naxes; i++) {
        s[i].inc = inc[i];
        s[i].dir = dir[i];
    }
    puldir_publish();
}

static void publish1(int32_t inc, int8_t dir) { publish_all(&inc, &dir); }

static void pulse(uint32_t n)
{
    for (uint32_t i = 0; i < n; i++)
        fake_pulse(false);
}

/* Run until the next tick is a boundary (none if it already is). */
static void to_boundary(void)
{
    while (left() != N)
        fake_pulse(false);
}

/* Ticks of the writes to pin id at level, from log index from; returns how many. */
static size_t writes(size_t from, uint8_t id, bool level, uint32_t* out, size_t max)
{
    size_t k = 0;
    for (size_t i = from; i < nw; i++)
        if (wlog[i].id == id && wlog[i].level == level) {
            assert(k < max);
            out[k++] = wlog[i].tick;
        }
    return k;
}

static bool wrote_at(uint8_t id, uint32_t tick)
{
    for (size_t i = 0; i < nw; i++)
        if (wlog[i].id == id && wlog[i].tick == tick)
            return true;
    return false;
}

static uint32_t rise[20000], fall[20000];

/* ---- setup ------------------------------------------------------------------- */

static void t_setup(void)
{
    PulDir a;
    memset(&a, 0xA5, sizeof a);           /* init must not depend on a zeroed struct */
    PulDir_init(&a, "a name longer than sixteen");
    assert(strlen(a.name) == 15 && strncmp(a.name, "a name longer t", 15) == 0);
    assert(a.conv == 1.0f);
    for (int i = 0; i < 4; i++) {
        const float want = i < 3 ? 200.0f : 0.0f;
        assert(a.def[i] == want && a.lim[i] == want);
    }
    assert(a.catchup_ms == PULDIR_CATCHUP_MS);
    assert(a.log_on);
    assert(a.slot == PULDIR_MAX_AXES);
    assert(a.dir_now == 1 && a.count == 0 && a.P == 0 && a.edge == PULDIR_ONE);
    assert(!a.has_pins && !a.pul_high && a.mode == PULDIR_MODE_IDLE && a.events == 0);
    assert(a.prof.dt == 1.0f / (float)PULDIR_PLAN_HZ && a.prof.done && a.prof.pos == 0.0f);
    assert(a.prof.lim.vmax == 200.0f && a.prof.lim.amax == 200.0f &&
           a.prof.lim.dmax == 200.0f && a.prof.lim.jmax == 0.0f);

    PulDir_init(&a, NULL);
    assert(a.name[0] == '\0');

    /* register refuses without pins, and changes nothing */
    assert(!PulDir_register(&a));
    assert(a.slot == PULDIR_MAX_AXES && puldir_naxes == 0);
    assert((a.events & PULDIR_EV_REFUSED) && a.refusal == PULDIR_R_NO_PIN);

    /* fill the table; a second register of the same axis is refused */
    for (uint8_t i = 0; i < PULDIR_MAX_AXES; i++)
        add_axis(i, false, false);
    assert(puldir_naxes == PULDIR_MAX_AXES);
    for (uint8_t i = 0; i < PULDIR_MAX_AXES; i++)
        assert(puldir_axes[i] == &ax[i]);
    ax[0].events = 0;
    assert(!PulDir_register(&ax[0]));
    assert(ax[0].slot == 0 && puldir_naxes == PULDIR_MAX_AXES);
    assert((ax[0].events & PULDIR_EV_REFUSED) && ax[0].refusal == PULDIR_R_BUSY);

    /* the table is full */
    PulDir_attachPins(&a, fake_pin(60), fake_pin(61), false, false);
    a.events = 0;
    assert(!PulDir_register(&a));
    assert(a.slot == PULDIR_MAX_AXES && puldir_naxes == PULDIR_MAX_AXES);
    assert((a.events & PULDIR_EV_REFUSED) && a.refusal == PULDIR_R_FULL);

    /* live and published entries start at dir +1, inc 0: no pulse, no direction write */
    start();
    pulse(3u * N);
    assert(nw == 0);
    for (uint8_t i = 0; i < PULDIR_MAX_AXES; i++)
        assert(ax[i].count == 0 && live_inc(i) == 0);
}

/* ---- pulse rate --------------------------------------------------------------- */

static void t_pulse_rate(void)
{
    setup(1);
    publish1(1 << 30, 1);                 /* a quarter pulse per tick */
    to_boundary();
    assert(nw == 0);
    const uint32_t t0 = PulDir_ticks();
    pulse(400);
    pulse(1);                             /* lowers the last pulse; the next rises at 403 */
    const size_t r = writes(0, 0, true, rise, 20000);
    const size_t f = writes(0, 0, false, fall, 20000);
    assert(r == 100 && f == 100);
    for (size_t i = 0; i < r; i++) {
        assert(rise[i] - t0 == 3u + 4u * i);  /* the edge to count k is the tick the position reaches k */
        assert(fall[i] == rise[i] + 1u);
    }
    assert(ax[0].count == 100);
}

/* ---- test 14: direction setup and hold ------------------------------------------- */

static void publish2(int32_t inc0, int8_t dir0, int32_t inc1, int8_t dir1)
{
    const int32_t inc[2] = { inc0, inc1 };
    const int8_t dir[2] = { dir0, dir1 };
    publish_all(inc, dir);
}

/* Axis i moves up; reverse it at a quarter pulse per tick with the publish
 * made just before the boundary tick b. Returns the tick of the direction write. */
static uint32_t reverse_at(uint8_t i, uint32_t b)
{
    assert(PulDir_ticks() == b && left() == N);
    const uint8_t pul = (uint8_t)(2u * i), dir = (uint8_t)(2u * i + 1u);
    const size_t from = nw;
    if (i == 0)
        publish2(-(1 << 30), -1, 0, ax[1].dir_now);
    else
        publish2(0, ax[0].dir_now, -(1 << 30), -1);
    pulse(2u * N);
    uint32_t td[4];
    assert(writes(from, dir, false, td, 4) == 1);
    assert(!wrote_at(pul, td[0]));        /* never on a tick with a pulse edge */
    const size_t r = writes(from, pul, true, rise, 20000);
    assert(r > 0);
    assert(rise[0] - td[0] >= 1u && rise[0] - td[0] < 2u * N);   /* setup: at least one tick */
    assert(ax[i].dir_now == -1 && !fake_level(dir));
    return td[0];
}

static void t_dir_change(void)
{
    setup(2);
    const uint64_t four = 4u * PULDIR_ONE;

    /* axis 0: up just short of 4 pulses in one interval, so its third edge is
     * long past when the boundary comes: the direction moves on the boundary tick */
    publish2((int32_t)(four / N), 1, 0, 1);
    to_boundary();
    pulse(N);
    assert(ax[0].count == 3 && !ax[0].pul_high);
    uint32_t b = PulDir_ticks();
    assert(reverse_at(0, b) == b);

    /* axis 1: up just over 4 pulses, so its fourth edge rises on the interval's
     * last tick and the boundary tick lowers it: the direction moves one tick later */
    publish2(0, -1, (int32_t)((four + N - 1u) / N), 1);
    to_boundary();
    pulse(N);
    assert(ax[1].count == 4 && ax[1].pul_high);
    b = PulDir_ticks();
    const size_t from = nw;
    assert(reverse_at(1, b) == b + 1u);
    assert(writes(from, 2, false, fall, 20000) >= 1 && fall[0] == b);   /* the lowering was on b */
}

/* ---- test 14: polarity -------------------------------------------------------------- */

static void t_polarity(void)
{
    add_axis(0, false, false);
    add_axis(1, true, true);
    /* parked: pulse inactive, direction at the level for +1 */
    assert(!fake_level(0) && fake_level(2));
    assert(fake_level(1) && !fake_level(3));
    start();

    const int8_t up[2] = { 1, 1 }, down[2] = { -1, -1 };
    const int32_t inc_up[2] = { 1 << 30, 1 << 30 }, inc_down[2] = { -(1 << 30), -(1 << 30) };
    for (int leg = 0; leg < 2; leg++) {
        publish_all(leg ? inc_down : inc_up, leg ? down : up);
        to_boundary();
        for (uint32_t i = 0; i < 4u * N; i++) {
            fake_pulse(false);
            assert(fake_level(2) == !fake_level(0));
            assert(fake_level(3) == !fake_level(1));
        }
        assert(fake_level(1) == !leg);
    }
    fake_pulse(false);                    /* lowers a pulse raised on the last tick */
    assert(ax[0].count == ax[1].count);

    /* the active-low axis: each pulse is a low level for one tick */
    const size_t r = writes(0, 2, false, rise, 20000);
    const size_t f = writes(0, 2, true, fall, 20000);
    assert(r > 10 && r == f);
    for (size_t i = 0; i < r; i++)
        assert(fall[i] == rise[i] + 1u);
}

/* ---- test 16: latch ------------------------------------------------------------------ */

/* Run one tick; return true if the live increments changed (all four at once). */
static bool tick_and_check(const int32_t* before_set, const int32_t* after_set, bool* latched)
{
    const bool boundary = left() == N;
    fake_pulse(false);
    int changed = 0;
    for (uint8_t i = 0; i < 4; i++) {
        const int32_t v = live_inc(i);
        assert(v == (*latched ? after_set[i] : before_set[i]) || (!*latched && v == after_set[i]));
        changed += !*latched && v == after_set[i];
    }
    assert(changed == 0 || changed == 4);   /* every axis on the same tick */
    if (changed) {
        assert(boundary);
        *latched = true;
    }
    return changed != 0;
}

static void t_latch(void)
{
    setup(4);
    const int8_t dir[4] = { 1, 1, 1, 1 };
    int32_t cur[4] = { 0, 0, 0, 0 }, next[4];
    /* the four writes start at every tick of the interval, so a pulse tick
     * falls at every point of the writes and publish lands at every phase */
    for (uint32_t phase = 0; phase < N; phase++) {
        for (uint8_t i = 0; i < 4; i++)
            next[i] = (int32_t)(1000u * (phase + 1u) + i);
        to_boundary();
        pulse(phase);
        bool latched = false;
        PulDirInc* s = puldir_shadow();
        for (uint8_t i = 0; i < 4; i++) {
            s[i].inc = next[i];
            s[i].dir = dir[i];
            assert(!tick_and_check(cur, next, &latched));   /* not before publish */
        }
        const bool boundary_next = left() == N;
        puldir_publish();
        if (boundary_next) {                 /* published one tick before a boundary */
            assert(tick_and_check(cur, next, &latched));
        } else {
            while (left() != N)
                assert(!tick_and_check(cur, next, &latched));
            assert(tick_and_check(cur, next, &latched));
        }
        memcpy(cur, next, sizeof cur);
    }
}

/* ---- test 15, increment half --------------------------------------------------------- */

static void check_max_rate(uint8_t pul, int32_t count)
{
    const size_t r = writes(0, pul, true, rise, 20000);
    const size_t f = writes(0, pul, false, fall, 20000);
    assert(r == f);
    for (size_t i = 0; i < r; i++) {
        assert(fall[i] == rise[i] + 1u);              /* high for one tick */
        if (i > 0)
            assert(rise[i] - rise[i - 1] >= 2u);      /* low for at least one tick */
    }
    assert(count == (int32_t)r || count == -(int32_t)r);
    assert(r == 4999 || r == 5000);
}

static void t_inc_max(void)
{
    /* axis 0 up and axis 1 down from rest, for 10,000 ticks (whole intervals) */
    setup(2);
    const uint32_t T = (10000u + N - 1u) / N * N;
    assert(T == 10000u);                  /* the 4,999-or-5,000 count assumes it */
    publish2(PULDIR_INC_MAX, 1, -PULDIR_INC_MAX, -1);
    to_boundary();
    pulse(T);
    publish2(0, 1, 0, -1);
    assert(left() == N);
    pulse(1);                             /* the boundary: lowers a pulse raised on the last tick */
    check_max_rate(0, ax[0].count);
    check_max_rate(2, ax[1].count);
    assert(ax[0].count > 0 && ax[1].count < 0);
}

/* ---- D3: hysteretic edges ---------------------------------------------------------------- */

/* One interval (N ticks) at inc; every edge must come on the first tick the
 * position reaches the next edge in the direction of travel. */
static size_t interval(int32_t inc, int8_t dir)
{
    publish1(inc, dir);
    to_boundary();
    size_t edges = 0;
    for (uint32_t i = 0; i < N; i++) {
        const int32_t c = ax[0].count;
        const int64_t before = (int64_t)ax[0].P;
        fake_pulse(false);
        const int64_t after = (int64_t)ax[0].P;
        if (ax[0].count != c) {
            assert(ax[0].count == c + dir);
            const int64_t at = (int64_t)ax[0].count << 32;
            assert(dir > 0 ? (before < at && after >= at) : (before > at && after <= at));
            edges++;
        }
    }
    return edges;
}

static void t_hysteresis(void)
{
    setup(1);
    const int64_t one = (int64_t)PULDIR_ONE;
    /* up to 10.25 in one interval */
    const int32_t inc_up = (int32_t)((41 * one / 4) / N);
    assert(inc_up <= PULDIR_INC_MAX);
    assert(interval(inc_up, 1) == 10);
    assert(ax[0].count == 10 && (int64_t)ax[0].P > 10 * one);

    /* down 1.5: through 10.0 with no edge, one edge at 9.0. The first tick of a
     * reversal writes the direction and does not move. */
    const int32_t inc_down = (int32_t)((3 * one / 2) / (N - 1));
    assert(interval(-inc_down, -1) == 1);
    assert(ax[0].count == 9 && (int64_t)ax[0].P < 9 * one && (int64_t)ax[0].P > 8 * one);

    /* dither across 9.0 and 9.5, between 8.75 and 9.75, for at least 1,000 ticks: no edges */
    const size_t from = nw;
    const int32_t inc_d = (int32_t)(one / (N - 1));
    const uint32_t cycles = (1000u + 2u * N - 1u) / (2u * N);
    for (uint32_t k = 0; k < cycles; k++) {
        assert(interval(inc_d, 1) == 0);
        assert((int64_t)ax[0].P > 9 * one + one / 2);
        assert(interval(-inc_d, -1) == 0);
        assert((int64_t)ax[0].P < 9 * one);
    }
    assert(ax[0].count == 9);
    for (size_t i = from; i < nw; i++)
        assert(wlog[i].id != 0);              /* direction writes only */
}

/* ---- overrun ------------------------------------------------------------------------------- */

static void t_overrun(void)
{
    setup(1);
    fake_pulse(false);
    assert(PulDir_overruns() == 0);
    fake_pulse(true);
    fake_pulse(true);
    assert(PulDir_overruns() == 2);
}

/* ---- the case table ------------------------------------------------------------------------ */

static const struct { const char* name; void (*fn)(void); } cases[] = {
    { "setup", t_setup },
    { "pulse_rate", t_pulse_rate },
    { "dir_change", t_dir_change },
    { "polarity", t_polarity },
    { "latch", t_latch },
    { "inc_max", t_inc_max },
    { "hysteresis", t_hysteresis },
    { "overrun", t_overrun },
};

int main(int argc, char** argv)
{
    const size_t n = sizeof cases / sizeof cases[0];
    if (argc == 2 && strcmp(argv[1], "list") == 0) {
        for (size_t i = 0; i < n; i++)
            printf("%s\n", cases[i].name);
        return 0;
    }
    if (argc == 2)
        for (size_t i = 0; i < n; i++)
            if (strcmp(argv[1], cases[i].name) == 0) {
                cases[i].fn();
                printf("ok %s\n", cases[i].name);
                return 0;
            }
    fprintf(stderr, "usage: %s list | <case>\n", argv[0]);
    return 2;
}
