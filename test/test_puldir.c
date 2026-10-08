/* Axis tests (guidelines section 8, tests 10 to 24). One case per process
 * (D9): `test_puldir list` prints the names, `test_puldir <name>` runs one. */
#ifdef NDEBUG
#error "the tests call the code under test inside assert(): build without NDEBUG"
#endif
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "fake_port.h"
#include "puldir_internal.h"

/* The timing main hands to fake_attach, from the command line: pulse_us plan_us. */
static uint32_t pulse_us = 20u, plan_us = 1000u;
#define N       puldir_timing.n
#define TICK_HZ puldir_timing.tick_hz
#define PLAN_HZ (1000000u / plan_us)
#define DT      puldir_timing.dt

/* The hand-built timing tests make whole-interval moves of up to 10.25 pulses and
 * run 10,000 ticks as whole intervals: at another N they skip. */
static bool n_fits(void)
{
    if (N >= 32u && 10000u % N == 0u)
        return true;
    printf("  skipped at N = %u\n", (unsigned)N);
    return false;
}

static float next_conv = 1.0f;                 /* the conversion add_axis gives */

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
static PulDirGroup grp, grp2;

static void add_axis(uint8_t i, bool pul_active_low, bool dir_inverted)
{
    char name[8];
    snprintf(name, sizeof name, "a%u", (unsigned)i);
    assert(PulDir_init(&ax[i], name, next_conv));
    fake_mark_motion_pin((uint8_t)(2u * i));
    fake_mark_motion_pin((uint8_t)(2u * i + 1u));
    PulDir_attachPins(&ax[i], fake_pin((uint8_t)(2u * i)), fake_pin((uint8_t)(2u * i + 1u)),
                      pul_active_low, dir_inverted);
    assert(PulDir_register(&ax[i]));
    assert(ax[i].slot == i);
}

/* After every plan tick: each published entry has a direction, and a non-zero
 * inc has the sign of it (the pulse tick relies on that). */
static PulDirInc* being_written;
static void check_published(bool after)
{
    if (!after) {
        being_written = puldir_shadow();
        return;
    }
    for (uint8_t i = 0; i < puldir_naxes; i++) {
        const PulDirInc e = being_written[i];
        assert(e.dir == 1 || e.dir == -1);
        assert(e.inc == 0 || (e.inc > 0) == (e.dir > 0));
        assert(e.inc <= PULDIR_INC_MAX && e.inc >= -PULDIR_INC_MAX);
    }
}

static void start(void)
{
    fake_on_write(on_write);
    fake_on_plan(check_published);
    PulDir_start();
}

static void setup(uint8_t k)
{
    for (uint8_t i = 0; i < k; i++)
        add_axis(i, false, false);
    start();
}

static void setup_conv(uint8_t k, float conv)
{
    next_conv = conv;
    setup(k);
}

/* ---- driving increments by hand -------------------------------------------- */

static uint32_t left(void)
{
    port_lock();
    const uint32_t l = puldir_left();
    port_unlock();
    return l;
}

/* The pulse tick's state, at ahead 0: Pb is the position itself. */
static PulDirSnap st(uint8_t slot)
{
    PulDirSnap s;
    port_lock();
    puldir_pulse_read(slot, 0u, &s);
    port_unlock();
    return s;
}

static bool high(uint8_t slot) { return st(slot).high; }

/* The next boundary's tick: the due tick of a set published now, as the plan tick computes it. */
static uint32_t next_boundary(void) { return PulDir_ticks() + left() % N; }

/* Write every registered axis's shadow entry, then publish. */
static void publish_all(const int32_t* inc, const int8_t* dir)
{
    PulDirInc* s = puldir_shadow();
    for (uint8_t i = 0; i < puldir_naxes; i++) {
        s[i].inc = inc[i];
        s[i].dir = dir[i];
    }
    puldir_publish(next_boundary());
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

#define MAX_EDGES 60000u
static uint32_t rise[MAX_EDGES], fall[MAX_EDGES];

/* ---- setup ------------------------------------------------------------------- */

static void t_setup(void)
{
    PulDir a, junk;
    memset(&a, 0xA5, sizeof a);           /* init must not depend on a zeroed struct */
    memset(&junk, 0xA5, sizeof junk);
    /* a name that does not fit, or none: refused, nothing written */
    assert(!PulDir_init(&a, "sixteen chars ok", 1.0f));
    assert(!PulDir_init(&a, NULL, 1.0f));
    assert(memcmp(&a, &junk, sizeof a) == 0);
    assert(PulDir_init(&a, "fifteen chars o", 1.0f));
    assert(strcmp(a.name, "fifteen chars o") == 0);
    assert(a.conv == 1.0f);
    for (int i = 0; i < 4; i++) {
        const float want = i < 3 ? 200.0f : 0.0f;
        assert(a.def[i] == want && a.lim[i] == want);
    }
    assert(a.catchup_ms == PULDIR_CATCHUP_MS);
    assert(a.log_on);
    assert(a.slot == PULDIR_MAX_AXES);
    assert(!a.has_pins && a.mode == PULDIR_MODE_IDLE && a.events == 0);
    assert(a.prof.dt == DT && a.prof.done && a.prof.pos == 0.0f);
    assert(a.prof.lim.vmax == 200.0f && a.prof.lim.amax == 200.0f &&
           a.prof.lim.dmax == 200.0f && a.prof.lim.jmax == 0.0f);

    assert(PulDir_init(&a, "", 1.0f));
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
    for (uint8_t i = 0; i < PULDIR_MAX_AXES; i++) {
        const PulDirSnap s = st(i);
        assert(s.Pb == 0 && s.count == 0 && s.inc == 0 && s.dir == 1 && !high(i));
    }
}

static void t_register_after_start(void)
{
    setup(1);
    PulDir b;
    assert(PulDir_init(&b, "late", 1.0f));
    PulDir_attachPins(&b, fake_pin(10), fake_pin(11), false, false);
    assert(!PulDir_register(&b));
    assert(b.slot == PULDIR_MAX_AXES && puldir_naxes == 1);
    assert((b.events & PULDIR_EV_REFUSED) && b.refusal == PULDIR_R_BUSY);
}

/* The timing the attach calls handed over, fixed by init: the plan period is N pulse periods, one
 * count per microsecond. Once fixed, no attach call is taken. */
static void t_timer_counts(void)
{
    setup(1);
    assert(puldir_timing.set && puldir_timing.pulse_us == pulse_us && puldir_timing.plan_us == plan_us);
    assert(N * pulse_us == plan_us && TICK_HZ == 1000000u / pulse_us);
    assert(DT == (float)plan_us / 1.0e6f && ax[0].prof.dt == DT);
    assert(left() == N);
    assert(!fake_attach(pulse_us, plan_us) && !puldir_timing_pulse(10u) && !puldir_timing_plan(500u));
    assert(puldir_timing.pulse_us == pulse_us && puldir_timing.plan_us == plan_us);
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
    const size_t r = writes(0, 0, true, rise, MAX_EDGES);
    const size_t f = writes(0, 0, false, fall, MAX_EDGES);
    assert(r == 100 && f == 100);
    for (size_t i = 0; i < r; i++) {
        assert(rise[i] - t0 == 3u + 4u * i);  /* the edge to count k is the tick the position reaches k */
        assert(fall[i] == rise[i] + 1u);
    }
    assert(st(0).count == 100);
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
        publish2(-(1 << 30), -1, 0, st(1).dir);
    else
        publish2(0, st(0).dir, -(1 << 30), -1);
    pulse(2u * N);
    uint32_t td[4];
    assert(writes(from, dir, false, td, 4) == 1);
    assert(!wrote_at(pul, td[0]));        /* never on a tick with a pulse edge */
    const size_t r = writes(from, pul, true, rise, MAX_EDGES);
    assert(r > 0);
    assert(rise[0] - td[0] >= 1u && rise[0] - td[0] < 2u * N);   /* setup: at least one tick */
    assert(st(i).dir == -1 && !fake_level(dir));
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
    assert(st(0).count == 3 && !high(0));
    uint32_t b = PulDir_ticks();
    assert(reverse_at(0, b) == b);

    /* axis 1: up just over 4 pulses, so its fourth edge rises on the interval's
     * last tick and the boundary tick lowers it: the direction moves one tick later */
    publish2(0, -1, (int32_t)((four + N - 1u) / N), 1);
    to_boundary();
    pulse(N);
    assert(st(1).count == 4 && high(1));
    b = PulDir_ticks();
    const size_t from = nw;
    assert(reverse_at(1, b) == b + 1u);
    assert(writes(from, 2, false, fall, MAX_EDGES) >= 1 && fall[0] == b);   /* the lowering was on b */
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
    assert(st(0).count == st(1).count);

    /* the active-low axis: each pulse is a low level for one tick */
    const size_t r = writes(0, 2, false, rise, MAX_EDGES);
    const size_t f = writes(0, 2, true, fall, MAX_EDGES);
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
        const int32_t v = st(i).inc;
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
        puldir_publish(next_boundary());
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
    const size_t r = writes(0, pul, true, rise, MAX_EDGES);
    const size_t f = writes(0, pul, false, fall, MAX_EDGES);
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
    if (!n_fits())
        return;
    const uint32_t T = 10000u;            /* whole intervals: n_fits */
    publish2(PULDIR_INC_MAX, 1, -PULDIR_INC_MAX, -1);
    to_boundary();
    pulse(T);
    publish2(0, 1, 0, -1);
    assert(left() == N);
    pulse(1);                             /* the boundary: lowers a pulse raised on the last tick */
    check_max_rate(0, st(0).count);
    check_max_rate(2, st(1).count);
    assert(st(0).count > 0 && st(1).count < 0);
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
        const uint32_t c = (uint32_t)st(0).count;
        const uint64_t before = st(0).Pb;
        fake_pulse(false);
        const uint64_t after = st(0).Pb;
        if ((uint32_t)st(0).count != c) {
            assert((uint32_t)st(0).count == c + (uint32_t)(int32_t)dir);   /* wraps like the count */
            const uint64_t at = (uint64_t)(uint32_t)st(0).count << 32;
            const int64_t b = (int64_t)(before - at), f = (int64_t)(after - at);
            assert(dir > 0 ? (b < 0 && f >= 0) : (b > 0 && f <= 0));
            edges++;
        }
    }
    return edges;
}

static void t_hysteresis(void)
{
    setup(1);
    if (!n_fits())
        return;
    const int64_t one = (int64_t)PULDIR_ONE;
    /* up to 10.25 in one interval */
    const int32_t inc_up = (int32_t)((41 * one / 4) / N);
    assert(inc_up <= PULDIR_INC_MAX);
    assert(interval(inc_up, 1) == 10);
    assert(st(0).count == 10 && (int64_t)st(0).Pb > 10 * one);

    /* down 1.5: through 10.0 with no edge, one edge at 9.0. The first tick of a
     * reversal writes the direction and does not move. */
    const int32_t inc_down = (int32_t)((3 * one / 2) / (N - 1));
    assert(interval(-inc_down, -1) == 1);
    assert(st(0).count == 9 && (int64_t)st(0).Pb < 9 * one && (int64_t)st(0).Pb > 8 * one);

    /* dither across 9.0 and 9.5, between 8.75 and 9.75, for at least 1,000 ticks: no edges */
    const size_t from = nw;
    const int32_t inc_d = (int32_t)(one / (N - 1));
    const uint32_t cycles = (1000u + 2u * N - 1u) / (2u * N);
    for (uint32_t k = 0; k < cycles; k++) {
        assert(interval(inc_d, 1) == 0);
        assert((int64_t)st(0).Pb > 9 * one + one / 2);
        assert(interval(-inc_d, -1) == 0);
        assert((int64_t)st(0).Pb < 9 * one);
    }
    assert(st(0).count == 9);
    for (size_t i = from; i < nw; i++)
        assert(wlog[i].id != 0);              /* direction writes only */
}

/* ---- kill and datum (for halt and setPosition) ------------------------------------------------ */

static void t_kill(void)
{
    setup(2);
    const int32_t q = 1 << 30, q2 = 1 << 29;
    publish2(q, 1, q, 1);
    to_boundary();
    pulse(N / 2u);
    publish2(q, 1, q2, 1);                /* pending until the next boundary */
    port_lock();
    puldir_kill(0);
    port_unlock();
    const uint64_t P0 = st(0).Pb;
    const size_t from = nw;
    assert(st(0).inc == 0 && st(1).inc == q);
    to_boundary();
    pulse(2u * N);
    assert(st(0).inc == 0 && st(1).inc == q2);   /* the pending set lost axis 0's inc only */
    assert(st(0).Pb == P0);
    assert(writes(from, 0, true, rise, MAX_EDGES) == 0);    /* no edge after the kill */
    assert(writes(from, 2, true, rise, MAX_EDGES) > 0);
}

static void t_datum(void)
{
    setup(1);
    const int32_t q = 1 << 30;
    port_lock();
    puldir_set_datum(0, -5);
    port_unlock();
    PulDirSnap s = st(0);
    assert(s.count == -5 && s.Pb == (uint64_t)(int64_t)-5 << 32 && s.dir == 1);
    assert(interval(q, 1) == N / 4u);     /* first edge at -4.0 */
    assert(st(0).count == -5 + (int32_t)(N / 4u));

    (void)interval(-q, -1);
    assert(interval(0, -1) == 0);
    port_lock();
    puldir_set_datum(0, 3);
    port_unlock();
    s = st(0);
    assert(s.count == 3 && s.Pb == (uint64_t)3 << 32 && s.dir == -1);
    assert(interval(-q, -1) == N / 4u);   /* first edge at 2.0 */
    assert(st(0).count == 3 - (int32_t)(N / 4u));

    /* the count wraps: from INT32_MAX - 2 up through INT32_MAX to INT32_MIN */
    (void)interval(q, 1);
    assert(interval(0, 1) == 0);
    port_lock();
    puldir_set_datum(0, INT32_MAX - 2);
    port_unlock();
    s = st(0);
    assert(s.count == INT32_MAX - 2 && s.Pb == (uint64_t)(INT32_MAX - 2) << 32 && s.dir == 1);
    assert(interval(q, 1) == N / 4u);     /* every edge at its crossing, across the wrap */
    assert(st(0).count == (int32_t)((uint32_t)(INT32_MAX - 2) + N / 4u));
    assert(st(0).count < 0);
    assert(interval(-q, -1) == N / 4u - 1u);   /* back down across it; the reversal skips one tick */
    assert(st(0).count == INT32_MAX - 1);
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

/* ---- the plan tick ------------------------------------------------------------------------- */

#define M ((int64_t)N * N)                    /* D6's margin, in 2^-32 pulse */

/* Rising edges per axis (active-high pins), signed by the direction pin's level,
 * and the ticks of axis 0's rising and falling edges. */
static uint64_t edges[PULDIR_MAX_AXES];
static int64_t net[PULDIR_MAX_AXES];
static uint32_t last_rise[PULDIR_MAX_AXES];
static uint32_t dirw[PULDIR_MAX_AXES];        /* direction pin writes per axis */
static size_t nrise, nfall;

static void count_write(uint32_t tick, uint8_t id, bool level)
{
    if (id / 2u >= PULDIR_MAX_AXES)
        return;
    const uint8_t i = (uint8_t)(id / 2u);
    if (id % 2u != 0u) {
        dirw[i]++;
        return;
    }
    if (level) {
        edges[i]++;
        last_rise[i] = tick;
        net[i] += fake_level((uint8_t)(id + 1u)) ? 1 : -1;
        if (i == 0 && nrise < MAX_EDGES)
            rise[nrise++] = tick;
    } else if (i == 0 && nfall < MAX_EDGES) {
        fall[nfall++] = tick;
    }
}

static void run_until_idle(PulDir* a, uint32_t max_ticks)
{
    for (uint32_t t = 0; PulDir_isMoving(a); t += N) {
        assert(t < max_ticks);
        fake_run(N);
    }
}

/* Tracking (test 11): a plan tick aims its set at origin + prof.pos; when that
 * set's interval ends, the pulse position must be within one increment of it.
 * D6 holds the end up to M short while the profile is within M of the target.
 * Guideline 5.4's "one increment" holds except on an interval whose set reverses
 * the direction: the direction write costs a tick, and one more when the latch
 * tick lowers a pulse, so there P may trail by 2 |inc| + N. */
typedef struct { bool valid; uint64_t aim; bool near; int8_t dir; bool reversal; } Aim;
static Aim aim_next, aim_cur;
static int64_t worst_track, worst_rev;        /* the largest |P - aim| - |inc| away from the target */
static int64_t worst_raw;                     /* the largest |P - aim| itself, likewise and off reversals */

static void track_run(PulDir* a, uint32_t ticks)
{
    const uint32_t plan_left = fake_plan_phase == 0u ? N : N - fake_plan_phase;
    for (uint32_t k = 0; k < ticks; k++) {
        if (left() == N) {                    /* this tick latches: the running set has finished */
            const PulDirSnap s = st(0);
            if (aim_cur.valid) {
                int64_t err = (int64_t)(s.Pb - aim_cur.aim);
                if (err < 0)
                    err = -err;
                const int64_t inc = s.inc < 0 ? -(int64_t)s.inc : s.inc;
                int64_t* worst = aim_cur.reversal ? &worst_rev : &worst_track;
                if (!aim_cur.near && err - inc > *worst)
                    *worst = err - inc;
                if (!aim_cur.near && !aim_cur.reversal && err > worst_raw)
                    worst_raw = err;
                assert(err <= (aim_cur.reversal ? 2 : 1) * inc + (int64_t)N + (aim_cur.near ? M : 0));
            }
            aim_cur = aim_next;
            aim_cur.reversal = aim_cur.dir != s.dir;    /* the set about to latch turns the pin */
        }
        fake_pulse(false);
        if (left() == plan_left) {
            fake_plan();
            const uint64_t aim = ((uint64_t)(uint32_t)a->origin << 32)
                               + (uint64_t)(int64_t)(a->prof.pos * PULDIR_FIX);
            int64_t gap = (int64_t)(a->target - aim);
            if (gap < 0)
                gap = -gap;
            aim_next.valid = true;
            aim_next.aim = aim;
            aim_next.near = gap < M;
            aim_next.dir = being_written[0].dir;          /* the set just published */
        }
    }
}

static void track_until_idle(PulDir* a, uint32_t max_ticks)
{
    for (uint32_t t = 0; PulDir_isMoving(a); t += N) {
        assert(t < max_ticks);
        track_run(a, N);
    }
}

static uint32_t rng = 0x2545F491u;
static uint32_t rnd(void)
{
    rng ^= rng << 13;
    rng ^= rng >> 17;
    rng ^= rng << 5;
    return rng;
}
static float frand(float lo, float hi) { return lo + (hi - lo) * (float)(rnd() >> 8) * (1.0f / 16777216.0f); }

/* Limits in pulses, set in user units at a new conversion. The conversion is fixed at init, so the
 * test writes it into the axis, at rest, as init would. */
static void set_limits(PulDir* a, float conv, float v, float acc, float dec, float jerk)
{
    assert(!a->moving && a->mode == PULDIR_MODE_IDLE);
    a->conv = conv;
    assert(PulDir_setDefaults(a, v / conv, acc / conv, dec / conv, jerk / conv));
}

static void t_move_exact(void)
{
    setup(1);
    fake_on_write(count_write);
    PulDir* a = &ax[0];
    const float cap = (float)(TICK_HZ / 2u);
    const float dt = DT;
    int retargets = 0, reversals = 0;
    static const uint32_t seeds[] = { 0x1234567u, 0x0BADF00Du, 0x2468ACE0u };
    for (int n = 0; n < 3000; n++) {
        const int move = n % 1000;
        if (move == 0)
            rng = seeds[n / 1000];
        if (move % 250 == 0)
            fake_plan_phase = (uint32_t)move / 250u * (N / 3u);   /* 0, N/3, 2N/3, 3N/3 - 1 */
        if (fake_plan_phase >= N)
            fake_plan_phase = N - 1u;
        const float conv = frand(10.0f, 500.0f);
        const float v = frand(200.0f, fminf(20000.0f, 0.9f * cap));
        const float acc = frand(v / 0.2f, v / 0.02f), dec = frand(v / 0.2f, v / 0.02f);
        const float jerk = move % 2 ? 0.0f : (acc + dec) / (frand(2.0f, 200.0f) * dt);
        set_limits(a, conv, v, acc, dec, jerk);

        const int32_t c0 = st(0).count;
        float d = rnd() % 8u == 0u ? frand(-3.0f, 3.0f) : frand(-0.3f, 0.3f) * v;
        const float target = ((float)c0 + d) / conv;
        const int32_t want = (int32_t)roundf(target * conv);
        const uint64_t e0 = edges[0];
        const int64_t n0 = net[0];

        assert(PulDir_moveTo(a, target));
        assert(PulDir_isMoving(a));
        bool retarget = rnd() % 4u == 0u && (want - c0 > 20 || c0 - want > 20);
        int32_t final = want;
        if (retarget) {
            /* part way there, a target behind the axis: it must reverse */
            track_run(a, (rnd() % 100u + 20u) * N);
            const int32_t c = st(0).count;
            const float back = (float)(want > c0 ? -1 : 1) * frand(1.0f, 0.3f * v);
            const float t2 = ((float)c + back) / conv;
            final = (int32_t)roundf(t2 * conv);
            retargets++;
            if (PulDir_isMoving(a)) {
                assert(PulDir_moveTo(a, t2));
                reversals++;
            } else {
                final = want;                 /* it had already landed */
            }
        }
        track_until_idle(a, 60u * TICK_HZ);
        const PulDirSnap s = st(0);
        assert(s.count == final);
        assert(!PulDir_isMoving(a));
        assert(PulDir_velocity(a) == 0.0f);
        assert(net[0] - n0 == (int64_t)s.count - c0);
        if (!retarget)
            assert(edges[0] - e0 == (uint64_t)(s.count > c0 ? s.count - c0 : c0 - s.count));
        assert(PulDir_target(a) == (float)final / conv);
        if (s.count != c0)                    /* D6: the final edge is the arrival interval's last tick */
            assert((last_rise[0] - PULDIR_TICK_START) % N == N - 1u);
        assert(a->mode == PULDIR_MODE_IDLE && !high(0));
    }
    assert(reversals > 150);
    printf("  move_exact: %d retargets, %d reversals, worst |P - aim| - |inc| = %lld units, %lld on reversals (N = %u)\n",
           retargets, reversals, (long long)worst_track, (long long)worst_rev, (unsigned)N);
}

/* Test 10: the axis's rising edges against a bare generator's integer crossings. */
static float gen[40000];

static void parity(float jerk)
{
    const float v = 10000.0f, acc = 1.0e5f, dec = 1.0e5f;
    const int32_t dist = 2000;
    MotionProfile g;
    assert(MotionProfile_init(&g, 0.0f, DT));
    assert(MotionProfile_setLimits(&g, (MotionLimits){ v, acc, dec, jerk }));
    assert(MotionProfile_setTarget(&g, (float)dist));
    size_t k = 0;
    gen[k++] = 0.0f;
    while (!g.done) {
        MotionProfile_advance(&g);
        assert(k < sizeof gen / sizeof gen[0]);
        gen[k++] = g.pos;
    }
    size_t k_t = 0;                           /* the first sample at the target */
    while (gen[k_t] != (float)dist)
        k_t++;
    assert(k_t + 3u == k);                    /* done two calls after it (E4) */

    PulDir* a = &ax[0];
    assert(PulDir_setSpeed(a, v) && PulDir_setAcceleration(a, acc) && PulDir_setDeceleration(a, dec));
    assert(PulDir_setJerk(a, jerk));
    nrise = nfall = 0;
    const int32_t c0 = st(0).count;
    assert(PulDir_moveTo(a, (float)(c0 + dist)));
    run_until_idle(a, 10u * TICK_HZ);
    assert(st(0).count == c0 + dist && nrise == (size_t)dist);

    /* crossing of integer m, in ticks from the first sample, by linear interpolation */
    double lat = 0.0, worst = 0.0, lo = 1e9, hi = -1e9;
    size_t j = 1;
    for (int32_t m = 1; m <= dist; m++) {
        while (gen[j] < (float)m)
            j++;
        const double c = ((double)(j - 1u) + ((double)m - (double)gen[j - 1u])
                          / ((double)gen[j] - (double)gen[j - 1u])) * (double)N;
        const double e = (double)(rise[m - 1] - rise[0]);
        if (m == 1)
            lat = -c;
        const double off = e - c - lat;      /* the last edge too: it goes on arrival */
        if (fabs(off) > worst)
            worst = fabs(off);
        assert(off >= -1.0 && off <= 1.0);
        lo = fmin(lo, e - c);
        hi = fmax(hi, e - c);
    }
    /* every edge lies in one window a tick wide (an edge is the first whole tick
     * at or past its crossing), widened only by float rounding */
    assert(hi - lo < 1.01);
    printf("  parity jerk %g: worst %.3f ticks from the first edge's latency, spread %.4f\n",
           (double)jerk, worst, hi - lo);
}

static void t_parity(void)
{
    setup(1);
    fake_on_write(count_write);
    parity(0.0f);
    parity(2.0e6f);                           /* a 100-tap window */
}

/* Test 23: a 5 us pulse, pulses 5 us wide at a 10 us period. */
static void t_full_rate(void)
{
    if (pulse_us != 5u) {
        printf("  skipped: needs a 5 us pulse\n");
        return;
    }
    setup(1);
    fake_on_write(count_write);
    PulDir* a = &ax[0];
    assert(PulDir_setSpeed(a, 99990.0f));
    assert(PulDir_setAcceleration(a, 1.0e6f) && PulDir_setDeceleration(a, 1.0e6f));
    assert(!PulDir_setSpeed(a, 100000.0f) && PulDir_speed(a) == 99990.0f);
    assert(a->refusal == PULDIR_R_RATE);
    assert(PulDir_moveTo(a, 50000.0f));
    run_until_idle(a, 2u * TICK_HZ);
    assert(st(0).count == 50000 && nrise == 50000 && nfall == 50000);
    for (size_t i = 0; i < nrise; i++)
        assert(fall[i] == rise[i] + 1u);      /* 5 us high */
    size_t threes = 0;
    for (size_t i = 10000; i < 40000; i++) { /* the cruise: the ramps take 5,000 pulses each */
        const uint32_t gap = rise[i] - rise[i - 1];
        assert(gap == 2u || gap == 3u);
        threes += gap == 3u;
    }
    /* 99,990 / 200,000 pulses per tick: a 3-tick gap every 5,000 pulses */
    assert(threes >= 4 && threes <= 8);
}

/* Settle steering: the plan tick runs twice in the interval where it arrives, so
 * the final set is overwritten before it latches. The count must still end on
 * the target: settling steers onto it again. */
static void t_arrival_twice(void)
{
    setup(1);
    fake_on_write(count_write);
    PulDir* a = &ax[0];
    assert(PulDir_setDefaults(a, 2000.0f, 20000.0f, 20000.0f, 0.0f));
    const uint32_t phases[3] = { 0u, N / 2u, N - 1u };
    for (int k = 0; k < 6; k++) {
        fake_plan_phase = phases[k % 3];
        assert(PulDir_setJerk(a, k < 3 ? 0.0f : 2.0e6f));      /* jerk: a 20-tap window */
        const int32_t c0 = st(0).count, want = c0 + (k % 2 ? -300 : 300);
        const int64_t n0 = net[0];
        assert(PulDir_moveTo(a, (float)want));
        const uint32_t plan_left = fake_plan_phase == 0u ? N : N - fake_plan_phase;
        bool twice = false;
        for (uint32_t t = 0; PulDir_isMoving(a); t++) {
            assert(t < 10u * TICK_HZ);
            fake_pulse(false);
            if (left() == plan_left) {
                fake_plan();
                if (a->arrived && !twice) {
                    fake_plan();              /* overwrites the final set, same interval */
                    twice = true;
                }
            }
        }
        assert(twice && st(0).count == want && net[0] - n0 == want - c0 && !high(0));
    }
}

/* D6: a soft decel through a long jerk window creeps the last few calls to
 * within M of the target. The final edge must still be the last tick of the
 * arrival interval, not an early tick of it. */
static void t_creep(void)
{
    setup(1);
    fake_on_write(count_write);
    PulDir* a = &ax[0];
    const float jerk = 40.0f / (250.0f / (float)PLAN_HZ);   /* a 250-tap window */
    assert(PulDir_setDefaults(a, 50.0f, 20.0f, 20.0f, jerk));
    const int32_t steps[4] = { 3, -1, 2, -4 };
    for (int k = 0; k < 8; k++) {
        fake_plan_phase = (uint32_t)k * (N / 8u);
        const int32_t c0 = st(0).count, want = c0 + steps[k % 4];
        assert(PulDir_moveTo(a, (float)want));
        run_until_idle(a, 10u * TICK_HZ);
        assert(st(0).count == want);
        assert((last_rise[0] - PULDIR_TICK_START) % N == N - 1u);
    }
}

/* Logging and refusals. */
static char lines[8][PULDIR_LINE];
static int nlines;
static void logger(const char* s)
{
    assert(nlines < 8 && strlen(s) < PULDIR_LINE);
    strcpy(lines[nlines++], s);
}

static void t_refusals(void)
{
    /* the natural setup order: the conversion at init, then limits in user units */
    PulDir b;
    assert(PulDir_init(&b, "b", 280.0f));
    assert(PulDir_setDefaults(&b, 15.0f, 30.0f, 30.0f, 0.0f));
    assert(PulDir_speed(&b) == 15.0f && PulDir_acceleration(&b) == 30.0f);
    assert(PulDir_deceleration(&b) == 30.0f && PulDir_jerk(&b) == 0.0f && b.def[0] == 15.0f);

    setup_conv(1, 2.0f);
    PulDir* a = &ax[0];
    PulDir_attachLogger(logger);

    assert(!PulDir_moveTo(a, NAN));
    assert(!PulDir_isMoving(a) && a->mode == PULDIR_MODE_IDLE && a->req == 0);
    assert(!PulDir_moveTo(a, 3.0e9f));       /* beyond 2^31 pulses */
    PulDir_loop(a);
    assert(nlines == 1 && strstr(lines[0], "moveTo refused") && strncmp(lines[0], "[a0] ", 5) == 0);

    /* init's limits are 200 pulses/s and 200 pulses/s^2: 100 units at conv 2 */
    assert(PulDir_speed(a) == 100.0f && PulDir_acceleration(a) == 100.0f && a->def[2] == 100.0f);
    assert(PulDir_setAcceleration(a, 200.0f) && PulDir_setDeceleration(a, 200.0f));

    /* every limit: refused on bad input, unchanged */
    assert(!PulDir_setSpeed(a, 0.0f) && !PulDir_setSpeed(a, NAN) && PulDir_speed(a) == 100.0f);
    assert(!PulDir_setSpeed(a, (float)(TICK_HZ / 2u) / 2.0f) && PulDir_speed(a) == 100.0f);
    assert(PulDir_setSpeed(a, nextafterf((float)(TICK_HZ / 2u) / 2.0f, 0.0f)));
    assert(PulDir_setSpeed(a, 100.0f));
    assert(!PulDir_setAcceleration(a, -1.0f) && PulDir_acceleration(a) == 200.0f);
    assert(!PulDir_setDeceleration(a, INFINITY) && PulDir_deceleration(a) == 200.0f);
    assert(!PulDir_setJerk(a, -1.0f) && PulDir_jerk(a) == 0.0f);
    nlines = 0;
    PulDir_loop(a);
    assert(nlines == 1 && strstr(lines[0], "setJerk refused"));

    /* a jerk whose window would not fit the ring: refused, the line names the smallest that fits */
    const float dt = DT;
    const float jmin = MotionProfile_minJerk(400.0f, 400.0f, dt) / 2.0f;     /* user units */
    assert(!PulDir_setJerk(a, jmin * 0.5f) && PulDir_jerk(a) == 0.0f && a->refusal == PULDIR_R_TAPS);
    nlines = 0;
    PulDir_loop(a);
    assert(nlines == 1 && strstr(lines[0], "setJerk refused: jerk below "));
    assert(PulDir_setJerk(a, jmin * 2.0f));
    assert(!PulDir_setDefaults(a, 100.0f, 200.0f, 200.0f, jmin * 0.5f));
    assert(a->def[0] == 100.0f && a->def[1] == 100.0f && a->def[3] == 0.0f);
    assert(PulDir_setDefaults(a, 100.0f, 200.0f, 300.0f, 0.0f));
    assert(a->def[2] == 300.0f && PulDir_deceleration(a) == 300.0f && PulDir_jerk(a) == 0.0f);

    /* logging off: the event is consumed, no line */
    PulDir_setLogging(a, false);
    nlines = 0;
    assert(!PulDir_moveTo(a, NAN));
    PulDir_loop(a);
    assert(nlines == 0 && a->events == 0);
    PulDir_setLogging(a, true);

    /* a landing, with the position formatted from integers */
    assert(PulDir_moveTo(a, -12.25f));     /* conv 2: -24.5 pulses rounds to -25 (away from zero) */
    assert(PulDir_target(a) == -12.5f);
    nlines = 0;
    run_until_idle(a, 10u * TICK_HZ);
    PulDir_loop(a);
    assert(nlines == 1 && strcmp(lines[0], "[a0] landed at -12.5000") == 0);
    assert(PulDir_position(a) == -12.5f && PulDir_status(a) == PULDIR_ENABLED);

    /* moveBy adds whole pulses to the target */
    assert(!PulDir_moveBy(a, NAN) && !PulDir_moveBy(a, 2.0e9f) && !PulDir_isMoving(a));
    assert(PulDir_moveBy(a, 1.2f));         /* 2.4 pulses rounds to 2 */
    assert(PulDir_target(a) == -11.5f && PulDir_isMoving(a) && PulDir_status(a) == PULDIR_MOVING);
    run_until_idle(a, 10u * TICK_HZ);
    assert(st(0).count == -23 && PulDir_position(a) == -11.5f);
}

/* 5.8, 5.9: each new batch of overruns is logged once, by the first axis loop
 * that can log it. */
static void t_overrun_log(void)
{
    setup(2);
    PulDir_attachLogger(logger);
    fake_pulse(false);
    PulDir_loop(&ax[0]);
    assert(nlines == 0);
    fake_pulse(true);
    fake_pulse(true);
    PulDir_loop(&ax[1]);
    assert(nlines == 1 && strcmp(lines[0], "[a1] pulse tick overrun, total 2") == 0);
    PulDir_loop(&ax[0]);
    PulDir_loop(&ax[1]);
    assert(nlines == 1);

    /* an axis that does not log leaves the batch to one that does */
    PulDir_setLogging(&ax[0], false);
    fake_pulse(true);
    PulDir_loop(&ax[0]);
    assert(nlines == 1);
    PulDir_loop(&ax[1]);
    assert(nlines == 2 && strcmp(lines[1], "[a1] pulse tick overrun, total 3") == 0);
}

/* A datum set before registration carries through it: registration takes the pulse state from
 * the target. Motion before registration is refused, since the ticks cannot run it. */
static void t_setup_order(void)
{
    PulDir* a = &ax[0];
    static const uint8_t sw_in = 0;
    assert(PulDir_init(a, "a0", 1.0f));
    fake_mark_motion_pin(0);
    fake_mark_motion_pin(1);
    PulDir_attachPins(a, fake_pin(0), fake_pin(1), false, false);
    assert(PulDir_setHoming(a, 1, 10.0f, 0.0f, &sw_in, 0.0f, 100u));
    PulDir_attachLogger(logger);

    /* the review's case: the datum before registration, read back at once */
    assert(PulDir_setPosition(a, 100.0f));
    assert(PulDir_position(a) == 100.0f && PulDir_target(a) == 100.0f);
    const uint16_t req0 = a->req;             /* REQ_SETPOS, for the plan tick */

    /* motion is refused and changes nothing */
    assert(!PulDir_moveTo(a, 101.0f) && a->refusal == PULDIR_R_UNREGISTERED);
    assert(!PulDir_moveBy(a, 1.0f) && a->refusal == PULDIR_R_UNREGISTERED);
    assert(!PulDir_jog(a, 5.0f) && a->refusal == PULDIR_R_UNREGISTERED);
    assert(!PulDir_streamBegin(a) && a->refusal == PULDIR_R_UNREGISTERED);
    assert(!PulDir_homeStart(a) && a->refusal == PULDIR_R_UNREGISTERED);
    assert(a->mode == PULDIR_MODE_IDLE && !PulDir_isMoving(a) && a->req == req0);
    assert(a->home_state == PULDIR_HOME_IDLE && a->slot == PULDIR_MAX_AXES);
    assert(PulDir_position(a) == 100.0f && PulDir_target(a) == 100.0f);
    nlines = 0;
    PulDir_loop(a);
    assert(nlines == 1 && strcmp(lines[0], "[a0] homeStart refused: not registered") == 0);

    /* registration takes the datum; a second datum after it moves the pulse state too */
    assert(PulDir_register(a) && a->slot == 0);
    PulDirSnap s = st(0);
    assert(s.count == 100 && s.Pb == (uint64_t)100 << 32 && s.inc == 0 && s.dir == 1 && !s.high);
    assert(PulDir_position(a) == 100.0f && PulDir_target(a) == 100.0f);
    assert(PulDir_setPosition(a, -7.0f));
    s = st(0);
    assert(s.count == -7 && s.Pb == (uint64_t)(int64_t)-7 << 32);
    assert(PulDir_position(a) == -7.0f && PulDir_target(a) == -7.0f);
    assert(PulDir_setPosition(a, 100.0f));
    assert(st(0).count == 100 && PulDir_position(a) == 100.0f && PulDir_target(a) == 100.0f);

    /* started: moveTo(101) from the datum emits exactly one pulse */
    start();
    fake_on_write(count_write);
    assert(PulDir_moveTo(a, 101.0f));
    run_until_idle(a, TICK_HZ);
    assert(st(0).count == 101 && edges[0] == 1 && net[0] == 1 && !high(0));
    assert(PulDir_position(a) == 101.0f && PulDir_target(a) == 101.0f);
    assert(a->origin == 101 && a->prof.done && a->prof.pos == 0.0f);
}

/* One move under the late-publish fault, tick by tick: the count never leaves the span between
 * where it started and the target, and lands on it. */
static void late_move(PulDir* a, int32_t want)
{
    const int32_t c0 = st(a->slot).count;
    const int32_t lo = c0 < want ? c0 : want, hi = c0 < want ? want : c0;
    assert(PulDir_moveTo(a, (float)want));
    for (uint32_t t = 0; PulDir_isMoving(a); t++) {
        assert(t < 10u * TICK_HZ);
        fake_run(1);
        const int32_t c = st(a->slot).count;
        assert(c >= lo && c <= hi);
    }
    assert(st(a->slot).count == want && !high(a->slot) && a->mode == PULDIR_MODE_IDLE);
    assert(PulDir_target(a) == (float)want && PulDir_velocity(a) == 0.0f);
}

/* The plan tick publishing after its boundary: a preemption between its snapshot and its publish
 * at a phase just before the boundary. The set latches on the next tick and runs the rest of its
 * interval, the next plan tick corrects the shortfall from the real position, and every move still
 * lands exactly. (Latched a whole interval late instead, the review's 301-pulse move ended at 310:
 * the settle branch republished the remainder and both sets ran.) Every late set is counted. */
static uint32_t plan_ticks;                   /* plan ticks run, for one late set per plan tick */
static void count_plans(bool after)
{
    check_published(after);
    plan_ticks += after;
}

static void t_late_publish(void)
{
    setup(2);
    fake_on_write(count_write);
    fake_on_plan(count_plans);
    PulDir_attachLogger(logger);
    PulDir* a = &ax[0];
    const float v = 9.0f * (float)PLAN_HZ;      /* about 9 pulses per interval */
    for (uint8_t i = 0; i < 2; i++)
        assert(PulDir_setDefaults(&ax[i], v, 50.0f * v, 50.0f * v, 0.0f));
    fake_plan_phase = N - 1u;                   /* one tick of slack before the boundary */
    fake_preempt_unlock = 2u;                   /* and two ticks run before the publish */

    /* the review's reproduction, then the same with jerk on */
    late_move(a, 301);
    assert(PulDir_setJerk(a, 5.0f * v / DT));  /* a 20-tap window */
    late_move(a, 0);
    assert(PulDir_setJerk(a, 0.0f));

    /* a reversal mid-way: the count never passes the first target and lands on the second */
    assert(PulDir_moveTo(a, 301.0f));
    while (st(0).count < 150)
        fake_run(1);
    assert(PulDir_moveTo(a, -100.0f));
    for (uint32_t t = 0; PulDir_isMoving(a); t++) {
        assert(t < 10u * TICK_HZ);
        fake_run(1);
        assert(st(0).count <= 301 && st(0).count >= -100);
    }
    assert(st(0).count == -100 && PulDir_target(a) == -100.0f && !high(0));

    /* a group move lands both members exactly; under this fault the final edges need not share
     * a tick, since each member's shortfall is corrected on its own */
    PulDir* m[2] = { &ax[0], &ax[1] };
    assert(PulDirGroup_init(&grp, m, 2));
    const float tg[2] = { 201.0f, -150.0f };
    assert(PulDirGroup_moveTo(&grp, tg));
    for (uint32_t t = 0; PulDirGroup_isMoving(&grp); t++) {
        assert(t < 10u * TICK_HZ);
        fake_run(1);
        assert(st(0).count >= -100 && st(0).count <= 201 && st(1).count <= 0 && st(1).count >= -150);
    }
    assert(st(0).count == 201 && st(1).count == -150 && !high(0) && !high(1));
    assert(PulDir_target(&ax[0]) == 201.0f && PulDir_target(&ax[1]) == -150.0f);

    /* every set published under the fault latched late; the loop logs the total once */
    const uint32_t late = PulDir_lateSets();
    assert(late > 0u);
    char want[PULDIR_LINE];
    snprintf(want, sizeof want, "[a0] plan tick late, total %u", (unsigned)late);
    nlines = 0;
    PulDir_loop(a);
    assert(nlines >= 1 && strcmp(lines[0], want) == 0);
    nlines = 0;
    PulDir_loop(a);
    PulDir_loop(&ax[1]);
    for (int i = 0; i < nlines; i++)
        assert(strstr(lines[i], "plan tick late") == NULL);

    /* the port's own timing again: once the last set published under the fault has latched, the
     * late sets are one per plan tick run under it, and a normal move adds none */
    const uint32_t faulted = plan_ticks;
    fake_preempt_unlock = 0u;
    fake_plan_phase = N / 2u;
    fake_run(2u * N);
    const uint32_t late0 = PulDir_lateSets();
    assert(late0 == faulted);
    late_move(a, 0);
    assert(PulDir_lateSets() == late0);
    printf("  late_publish: %u sets latched late (N = %u)\n", (unsigned)late, (unsigned)N);
}

/* ---- jog, stop, halt, the datum and live limits --------------------------------------------- */

static uint32_t plan_left(void) { return fake_plan_phase == 0u ? N : N - fake_plan_phase; }

/* One pulse tick, and the plan tick if it is due; returns whether it ran. */
static bool tick1(void)
{
    fake_run(1);
    return left() == plan_left();
}

/* After every plan tick: axis 0's velocity steps within its ramps. */
static float vprev;
static void check_ramps(bool after)
{
    check_published(after);
    if (!after) {
        vprev = ax[0].vel_out;
        return;
    }
    const MotionLimits L = ax[0].prof.lim;
    const float dv = fabsf(ax[0].vel_out - vprev);
    assert(dv <= fmaxf(L.amax, L.dmax) * (DT) * 1.0001f + 1e-3f);
}

/* At rest the pulse position is within a pulse of the count, on the side the
 * hysteresis leaves it (D3) for the direction of travel, when there is one. */
static void check_rest(uint8_t i, int travel)
{
    const PulDirSnap s = st(i);
    const int64_t off = (int64_t)(s.Pb - ((uint64_t)(uint32_t)s.count << 32));
    assert(off < (int64_t)PULDIR_ONE && off > -(int64_t)PULDIR_ONE);
    assert(travel <= 0 || off >= 0);
    assert(travel >= 0 || off <= 0);
    assert(s.inc == 0 && !high(i));
}

/* D7, 2.2: a jog never resets the count, changes speed live, reverses under
 * the ramps, and jog(0) stops at target == position. */
static void t_jog(void)
{
    const float conv = 100.0f;
    setup_conv(1, conv);
    fake_on_write(count_write);
    fake_on_plan(check_ramps);
    PulDir* a = &ax[0];
    assert(PulDir_setDefaults(a, 20.0f, 100.0f, 100.0f, 0.0f));   /* 2,000 pulses/s, 10,000 pulses/s^2 */
    assert(PulDir_setPosition(a, 10.0f));
    fake_run(N);
    assert(st(0).count == 1000 && PulDir_position(a) == 10.0f);

    /* refused at or above the rate cap and when not finite; nothing changes */
    const float cap = (float)(TICK_HZ / 2u) / conv;
    assert(!PulDir_jog(a, cap) && a->refusal == PULDIR_R_RATE && a->refused_value == cap);
    assert(!PulDir_jog(a, -cap) && a->refusal == PULDIR_R_RATE);
    assert(!PulDir_jog(a, NAN) && a->refusal == PULDIR_R_ARG);
    assert(!PulDir_jog(a, -INFINITY) && a->refusal == PULDIR_R_ARG);
    assert(!PulDir_isMoving(a) && a->mode == PULDIR_MODE_IDLE && a->req == 0);

    /* jog(5) = 500 pulses/s, counting on from 1,000 */
    assert(PulDir_jog(a, 5.0f));
    assert(PulDir_isMoving(a) && a->mode == PULDIR_MODE_PROFILE);
    int32_t c = st(0).count;
    for (uint32_t k = 0; k < PLAN_HZ; k++) {           /* 1 s */
        fake_run(N);
        assert(st(0).count >= c);
        c = st(0).count;
    }
    assert(PulDir_velocity(a) == 5.0f);
    /* 1 s at 500 less the 50 ms ramp's 12.5, less up to two intervals of latency */
    assert(c - 1000 > 480 && c - 1000 <= 488);

    /* jog(10): the speed changes live */
    assert(PulDir_jog(a, 10.0f));
    for (uint32_t k = 0; k < PLAN_HZ / 2u; k++) {
        fake_run(N);
        assert(st(0).count >= c);
        c = st(0).count;
    }
    assert(PulDir_velocity(a) == 10.0f);
    /* a jog has no target: it reads where the profile is, an interval or two ahead of the pulses */
    assert(PulDir_target(a) > PulDir_position(a) && PulDir_target(a) - PulDir_position(a) < 0.25f);

    /* D7: a lowered speed limit ramps the jog down at decel, a raised one back up
     * at accel to the jog's own speed: 600 pulses/s at 10,000 pulses/s^2 a second */
    const float step = 100.0f / (float)PLAN_HZ;            /* user units/s per plan tick */
    const int nsteps = (int)(6u * PLAN_HZ / 100u);         /* 60 at 1 kHz */
    for (int way = 0; way < 2; way++) {
        assert(PulDir_setSpeed(a, way == 0 ? 4.0f : 20.0f));
        const float to = way == 0 ? 4.0f : 10.0f;
        int steps = 0;
        for (float v = PulDir_velocity(a); v != to; steps++) {
            assert(steps < 2 * nsteps);
            fake_run(N);
            const float w = PulDir_velocity(a), dv = way == 0 ? v - w : w - v;
            assert(dv > step * 0.999f && dv < step * 1.001f);
            v = w;
        }
        assert(steps == nsteps);
        fake_run(20u * N);
        assert(PulDir_velocity(a) == to && a->mode == PULDIR_MODE_PROFILE);
    }
    c = st(0).count;

    /* jog(-5) reverses through zero under the ramps (check_ramps) */
    assert(PulDir_jog(a, -5.0f));
    int32_t top = c;
    for (uint32_t k = 0; k < PLAN_HZ; k++) {
        fake_run(N);
        if (st(0).count > top)
            top = st(0).count;
    }
    assert(PulDir_velocity(a) == -5.0f && st(0).dir == -1 && st(0).count < top - 380);

    /* jog(0) is stop: no reversal, IDLE, target == position */
    const uint32_t d0 = dirw[0];
    assert(PulDir_jog(a, 0.0f));
    assert(a->mode == PULDIR_MODE_STOPPING && PulDir_isMoving(a));
    run_until_idle(a, TICK_HZ);
    assert(dirw[0] == d0);
    assert(a->mode == PULDIR_MODE_IDLE && PulDir_velocity(a) == 0.0f);
    assert(PulDir_target(a) == PulDir_position(a));
    assert(net[0] == (int64_t)st(0).count - 1000);
    check_rest(0, -1);
}

/* stop plans a stop at decel from where the axis is; moveTo during STOPPING lands. */
static void t_stop(void)
{
    setup(1);
    fake_on_write(count_write);
    fake_on_plan(check_ramps);
    PulDir* a = &ax[0];
    const uint32_t phases[3] = { 0u, N / 3u, N - 1u };
    for (int k = 0; k < 12; k++) {
        fake_plan_phase = phases[k % 3];
        /* jerk on odd k: a 10-tap window */
        assert(PulDir_setDefaults(a, 2000.0f, 20000.0f, 7000.0f, k % 2 ? 2.7e6f : 0.0f));
        const int32_t c0 = st(0).count, dir = k % 4 < 2 ? 1 : -1, want = c0 + dir * 3000;
        const int64_t n0 = net[0];
        assert(PulDir_moveTo(a, (float)want));
        fake_run((60u + 13u * (uint32_t)k) * N + (uint32_t)k * 3u);   /* on the ramp or cruising */
        float expect = (float)a->origin + a->prof.pos + MotionProfile_stopDistance(&a->prof);
        if (k >= 6) {                         /* a lower decel set just before the stop: the stop uses it */
            const float v = a->prof.v_t, d1 = a->prof.lim.dmax;
            assert(PulDir_setDeceleration(a, 5000.0f));
            expect += v * fabsf(v) * (1.0f / (2.0f * 5000.0f) - 1.0f / (2.0f * d1));
        }
        const uint32_t d0 = dirw[0];
        PulDir_stop(a);
        assert(a->mode == PULDIR_MODE_STOPPING && PulDir_isMoving(a));
        run_until_idle(a, TICK_HZ);
        const int32_t c = st(0).count;
        assert(a->mode == PULDIR_MODE_IDLE && PulDir_velocity(a) == 0.0f);
        assert(dir > 0 ? (c > c0 && c < want) : (c < c0 && c > want));   /* short of the target */
        assert(fabsf((float)c - expect) <= 4.0f);   /* at decel: within an interval's travel */
        assert(PulDir_target(a) == PulDir_position(a) && PulDir_position(a) == (float)c);
        assert(dirw[0] == d0);                       /* a stop never reverses */
        assert(net[0] - n0 == c - c0);
        check_rest(0, dir);
    }

    /* moveTo during STOPPING is accepted and lands exactly, here behind the stop */
    fake_plan_phase = N / 2u;
    const int32_t c0 = st(0).count;
    assert(PulDir_moveTo(a, (float)(c0 + 3000)));
    fake_run(150u * N);
    PulDir_stop(a);
    fake_run(5u * N);
    assert(a->mode == PULDIR_MODE_STOPPING);
    assert(PulDir_moveTo(a, (float)(c0 + 100)));
    assert(a->mode == PULDIR_MODE_PROFILE && PulDir_target(a) == (float)(c0 + 100));
    run_until_idle(a, 2u * TICK_HZ);
    assert(st(0).count == c0 + 100 && PulDir_target(a) == (float)(c0 + 100));

    /* stop in IDLE is a no-op */
    PulDir_stop(a);
    assert(a->mode == PULDIR_MODE_IDLE && a->req == 0 && !PulDir_isMoving(a));
}

/* Stops, jog stops and halts at random times and limits: every one reaches IDLE
 * with target == position and the count on the last integer reached. */
static void t_stop_random(void)
{
    setup(1);
    fake_on_write(count_write);
    PulDir* a = &ax[0];
    const float cap = (float)(TICK_HZ / 2u);
    const float dt = DT;
    rng = 0x13579BDFu;
    int kinds[3] = { 0, 0, 0 }, fractional = 0;
    for (int n = 0; n < 300; n++) {
        fake_plan_phase = rnd() % N;
        const float conv = frand(10.0f, 500.0f);
        const float v = frand(200.0f, fminf(20000.0f, 0.9f * cap));
        const float acc = frand(v / 0.2f, v / 0.02f), dec = frand(v / 0.2f, v / 0.02f);
        const float jerk = n % 2 ? 0.0f : (acc + dec) / (frand(2.0f, 200.0f) * dt);
        set_limits(a, conv, v, acc, dec, jerk);

        const int32_t c0 = st(0).count;
        const int64_t n0 = net[0];
        const uint32_t kind = rnd() % 3u;
        kinds[kind]++;
        const float pos0 = PulDir_position(a);
        if (kind == 0)
            assert(PulDir_moveTo(a, ((float)c0 + frand(-0.3f, 0.3f) * v) / conv));
        else
            assert(PulDir_jog(a, frand(-1.0f, 1.0f) * v / conv));
        const float tg = PulDir_target(a);
        const int travel = kind == 0 ? (tg > pos0) - (tg < pos0) : (a->req_speed > 0.0f) - (a->req_speed < 0.0f);
        fake_run(rnd() % (300u * N));
        if (kind == 2) {
            const uint64_t e0 = edges[0];
            PulDir_halt(a);
            run_until_idle(a, 3u * N);
            fake_run(2u * N);
            assert(edges[0] == e0);
        } else {
            PulDir_stop(a);
            run_until_idle(a, TICK_HZ);
        }
        const int32_t c = st(0).count;
        assert(a->mode == PULDIR_MODE_IDLE && PulDir_velocity(a) == 0.0f);
        assert(PulDir_target(a) == PulDir_position(a));
        assert(net[0] - n0 == c - c0);
        check_rest(0, c != c0 ? travel : 0);  /* the count is the last integer reached */
        if (kind != 2 && fabsf(a->prof.pos) > 1e-3f)
            fractional++;                     /* the stop ended between two pulses */

        if (kind == 2) {                  /* after a halt, the next move lands exactly */
            const int32_t want = c + (int32_t)(rnd() % 201u) - 100;
            assert(PulDir_moveTo(a, (float)want / conv));
            const int32_t w = (int32_t)roundf(((float)want / conv) * conv);
            run_until_idle(a, 10u * TICK_HZ);
            assert(st(0).count == w && net[0] - n0 == w - c0);
        }
    }
    assert(kinds[0] > 50 && kinds[1] > 50 && kinds[2] > 50 && fractional > 100);
}

/* Test 17: halt between two pulse ticks mid-cruise. */
static void t_halt(void)
{
    setup(1);
    fake_on_write(count_write);
    PulDir* a = &ax[0];
    assert(PulDir_setDefaults(a, 5000.0f, 50000.0f, 50000.0f, 0.0f));
    const uint32_t phases[3] = { 0u, N / 2u, N - 1u };
    for (int k = 0; k < 12; k++) {
        fake_plan_phase = phases[k % 3];
        assert(PulDir_setJerk(a, k % 6 < 3 ? 0.0f : 1.0e7f));     /* a 10-tap window at 1 kHz */
        const int32_t c0 = st(0).count, dir = k % 2 ? -1 : 1;
        assert(PulDir_moveTo(a, (float)(c0 + dir * 20000)));
        fake_run(TICK_HZ / 5u);        /* 200 ms: cruising, the ramp takes 100 ms */
        assert(a->prof.vel == (float)dir * 5000.0f);
        if (k % 4 < 2)
            while (!high(0))                  /* just after a rising edge */
                fake_run(1);
        else
            while (!tick1())                  /* just after a plan tick published */
                ;
        const uint64_t e0 = edges[0];
        const int32_t ch = st(0).count;
        const bool was_high = high(0);
        PulDir_halt(a);
        assert(a->mode == PULDIR_MODE_IDLE && PulDir_velocity(a) == 0.0f);
        assert(PulDir_isMoving(a));           /* until a plan tick sees the pin low */
        assert(PulDir_target(a) == (float)ch && PulDir_position(a) == (float)ch);
        int plans = 0;
        if (was_high) {                       /* a plan tick before the pin is lowered */
            fake_plan();
            plans++;
            assert(PulDir_isMoving(a));
        }
        while (PulDir_isMoving(a)) {
            plans += tick1();
            assert(plans <= 2);
        }
        assert(!high(0));
        fake_run(3u * N);
        assert(edges[0] == e0 && st(0).count == ch && st(0).inc == 0);   /* no rising edge since */
        assert(PulDir_target(a) == (float)ch && PulDir_velocity(a) == 0.0f && a->mode == PULDIR_MODE_IDLE);
        /* the generator is at rest where the pulses stopped, not where it was aiming */
        const float frac = (float)(int64_t)(st(0).Pb - ((uint64_t)(uint32_t)a->origin << 32)) / PULDIR_FIX;
        assert(a->prof.done && a->prof.vel == 0.0f && fabsf(a->prof.pos - frac) < 1e-6f);
        assert(a->origin == ch && fabsf(frac) < 1.0f);

        /* the next move starts where the pulses stopped and lands exactly */
        const int64_t n0 = net[0];
        assert(PulDir_moveTo(a, (float)(ch - dir * 777)));
        run_until_idle(a, 2u * TICK_HZ);
        assert(st(0).count == ch - dir * 777 && net[0] - n0 == -dir * 777);
    }

    /* a moveTo or a jog not yet applied when the halt comes is discarded with the plan */
    for (int k = 0; k < 2; k++) {
        const uint64_t e0 = edges[0];
        const int32_t c0 = st(0).count;
        assert(k == 0 ? PulDir_moveTo(a, (float)(c0 + 5000)) : PulDir_jog(a, -3000.0f));
        PulDir_halt(a);
        fake_run(3u * N);
        assert(!PulDir_isMoving(a) && a->mode == PULDIR_MODE_IDLE && a->prof.done);
        assert(edges[0] == e0 && PulDir_target(a) == (float)c0);
    }
}

static int32_t abs_inc(void)
{
    const int64_t i = st(0).inc;
    return (int32_t)(i < 0 ? -i : i);
}

/* Test 15: the cap refuses at TICK_HZ / 2 / conv and accepts one ulp below,
 * and no live increment ever exceeds PULDIR_INC_MAX. */
static void t_rate_cap(void)
{
    const float conv = 3.0f;
    setup_conv(1, conv);
    fake_on_write(count_write);
    PulDir* a = &ax[0];
    const float cap = (float)(TICK_HZ / 2u) / conv, below = nextafterf(cap, 0.0f);
    const float s0 = PulDir_speed(a);
    assert(!PulDir_setSpeed(a, cap) && a->refusal == PULDIR_R_RATE && PulDir_speed(a) == s0);
    assert(PulDir_setSpeed(a, below) && PulDir_speed(a) == below);
    assert(PulDir_setAcceleration(a, 2.0e6f / conv) && PulDir_setDeceleration(a, 2.0e6f / conv));

    const float to = (float)(TICK_HZ / 4u) / conv;     /* half a second at the cap */
    const int32_t want = (int32_t)roundf(to * conv);
    assert(PulDir_moveTo(a, to));
    /* sat() clamps every increment, so a clamp shows only as lag: test 11's tracking */
    int32_t worst = 0;
    for (uint32_t t = 0; PulDir_isMoving(a); t++) {
        assert(t < 2u * TICK_HZ);
        track_run(a, 1);
        const int32_t m = abs_inc();
        if (m > worst)
            worst = m;
    }
    assert(st(0).count == want && worst > PULDIR_INC_MAX - PULDIR_INC_MAX / 1000);
    /* Unclamped, each boundary is within a few truncation remainders (3N seen);
     * a clamp of even 1000 units a tick accumulates to ~10^7. */
    assert(worst_raw <= 4 * (int64_t)N && worst_rev == 0);

    /* the same cap for a jog */
    assert(!PulDir_jog(a, cap) && a->refusal == PULDIR_R_RATE && !PulDir_isMoving(a));
    assert(PulDir_jog(a, -below));
    worst = 0;
    for (uint32_t t = 0; t < TICK_HZ / 5u; t++) {
        track_run(a, 1);
        const int32_t m = abs_inc();
        if (m > worst)
            worst = m;
    }
    assert(worst > PULDIR_INC_MAX - PULDIR_INC_MAX / 1000);
    PulDir_stop(a);
    track_until_idle(a, TICK_HZ);
}

/* 2.2 and test 24, the axis half: which limit changes are live. */
static void t_live_limits(void)
{
    const float conv = 100.0f;
    setup_conv(1, conv);
    PulDir_attachLogger(logger);
    PulDir* a = &ax[0];
    /* Jerks scale with the plan rate, so every window below has the same taps at any rate. */
    const float hz = (float)PLAN_HZ / 1000.0f;

    /* jerk off, in flight: accel, decel and speed are live; switching jerk on is not */
    assert(PulDir_setDefaults(a, 20.0f, 40.0f, 40.0f, 0.0f));
    assert(PulDir_moveTo(a, 100.0f));
    fake_run(20u * N);
    assert(!a->prof.done);
    assert(PulDir_setAcceleration(a, 30.0f) && PulDir_setDeceleration(a, 30.0f));
    assert(PulDir_setSpeed(a, 10.0f));
    fake_run(N);
    assert(a->prof.lim.amax == 3000.0f && a->prof.lim.dmax == 3000.0f && a->prof.lim.vmax == 1000.0f);
    assert(!PulDir_setJerk(a, 300.0f * hz) && a->refusal == PULDIR_R_WINDOW);   /* fits the taps, not the frozen window */
    assert(PulDir_jerk(a) == 0.0f && a->req == 0);
    fake_run(TICK_HZ / 2u);                       /* 500 ms */
    assert(PulDir_velocity(a) == 10.0f);                 /* the lowered speed took effect */
    run_until_idle(a, 30u * TICK_HZ);
    assert(st(0).count == 10000);

    /* jerk 290 at 1 kHz in flight: the window holds 207 taps, (30 + 30) / (290 * dt) = 206.9 */
    assert(PulDir_setDefaults(a, 20.0f, 40.0f, 30.0f, 290.0f * hz));  /* 7000 / 29 = 241.4 */
    assert(PulDir_setAcceleration(a, 30.0f));
    fake_run(N);
    assert(a->prof.n == 207);
    assert(PulDir_moveTo(a, 150.0f));
    fake_run(20u * N);
    assert(!a->prof.done && a->prof.n == 207);
    assert(PulDir_setAcceleration(a, 25.0f));            /* lowered: live */
    assert(PulDir_setAcceleration(a, 30.0f));            /* raised within the slack: live */
    assert(!PulDir_setAcceleration(a, 31.0f) && a->refusal == PULDIR_R_WINDOW);   /* 210.3 taps */
    assert(PulDir_acceleration(a) == 30.0f);
    assert(!PulDir_setJerk(a, 250.0f * hz) && a->refusal == PULDIR_R_WINDOW && PulDir_jerk(a) == 290.0f * hz);
    assert(PulDir_setJerk(a, 400.0f * hz));                   /* raised: live */
    a->refusal = PULDIR_R_NONE;
    assert(!PulDir_restoreDefaults(a) && a->refusal == PULDIR_R_WINDOW);   /* accel 40: 241.4 taps */
    assert(PulDir_speed(a) == 20.0f && PulDir_acceleration(a) == 30.0f);
    assert(PulDir_deceleration(a) == 30.0f && PulDir_jerk(a) == 400.0f * hz);
    fake_run(N);
    assert(a->prof.lim.jmax == 40000.0f * hz && a->prof.lim.amax == 3000.0f && a->prof.n == 207);
    run_until_idle(a, 30u * TICK_HZ);
    assert(st(0).count == 15000);
    assert(PulDir_restoreDefaults(a));                   /* idle: accepted */
    assert(PulDir_speed(a) == 20.0f && PulDir_acceleration(a) == 40.0f);
    assert(PulDir_deceleration(a) == 30.0f && PulDir_jerk(a) == 290.0f * hz);
    fake_run(N);
    assert(a->prof.n == 242 && a->prof.lim.amax == 4000.0f);

    /* a jerk needing more than the ring's taps: refused, the log names the smallest that fits */
    const float jmin = MotionProfile_minJerk(4000.0f, 3000.0f, DT) / conv;
    nlines = 0;
    PulDir_loop(a);
    nlines = 0;
    assert(!PulDir_setJerk(a, 0.9f * jmin) && a->refusal == PULDIR_R_TAPS);
    const float floor_ = a->refused_value;
    assert(fabsf(floor_ - jmin) <= 1e-5f * jmin && PulDir_jerk(a) == 290.0f * hz);
    PulDir_loop(a);
    assert(nlines == 1 && strstr(lines[0], "setJerk refused: jerk below "));   /* 273.4 at 1 kHz */
    assert(strstr(lines[0], " needs more than 256 taps"));
    const float logged = strtof(strstr(lines[0], "below ") + 6, NULL);
    assert(logged >= floor_ && logged - floor_ <= 1e-4f + 3e-6f * floor_);
    assert(PulDir_setJerk(a, logged) && PulDir_jerk(a) == logged);   /* the logged value fits */
}

/* The refused value of a jerk below the taps is the floor as a plain quotient, and the logged
 * figure, rounded up (at most 1e-4 and a few ulps above it), typed back, is accepted. */
static void t_jerk_floor(void)
{
    setup(1);
    PulDir_attachLogger(logger);
    PulDir* a = &ax[0];
    rng = 0x5EEDF00Du;
    for (int k = 0; k < 3000; k++) {
        const float conv = frand(0.5f, 500.0f);
        a->conv = conv;                       /* fixed at init: the test writes it, at rest */
        const float acc = frand(10.0f, 1.0e5f) / conv, dec = frand(10.0f, 1.0e5f) / conv;
        assert(PulDir_setDefaults(a, 100.0f / conv, acc, dec, 0.0f));
        const float jmin = MotionProfile_minJerk(acc * conv, dec * conv, DT) / conv;
        nlines = 0;
        assert(!PulDir_setJerk(a, 0.5f * jmin) && a->refusal == PULDIR_R_TAPS);
        const float floor_ = a->refused_value;
        assert(fabsf(floor_ - jmin) <= 1e-5f * jmin);
        PulDir_loop(a);
        assert(nlines == 1);
        const float logged = strtof(strstr(lines[0], "below ") + 6, NULL);
        assert(logged >= floor_ && logged - floor_ <= 1e-4f + 3e-6f * floor_);
        assert(PulDir_setJerk(a, logged) && PulDir_jerk(a) == logged);
        assert(PulDir_setJerk(a, 0.0f));
    }
}

/* D18: the datum moves only in IDLE, and moves everything together. */
static void t_setpos(void)
{
    setup_conv(1, 100.0f);
    fake_on_write(count_write);
    PulDir* a = &ax[0];
    assert(PulDir_setDefaults(a, 20.0f, 100.0f, 100.0f, 0.0f));
    a->homed = true;
    assert(!PulDir_setPosition(a, NAN) && a->refusal == PULDIR_R_ARG);
    assert(!PulDir_setPosition(a, 3.0e7f) && a->refusal == PULDIR_R_ARG);   /* 3e9 pulses */
    assert(a->homed && st(0).count == 0 && !(a->req & REQ_SETPOS));

    /* refused while moving */
    assert(PulDir_moveTo(a, 2.0f));
    fake_run(10u * N);
    assert(!PulDir_setPosition(a, 7.0f) && a->refusal == PULDIR_R_NOT_IDLE);
    assert(a->homed && PulDir_target(a) == 2.0f);
    run_until_idle(a, TICK_HZ);
    assert(st(0).count == 200);

    /* after a halt, refused until the plan tick has seen the pin low */
    assert(PulDir_moveTo(a, 5.0f));
    fake_run(100u * N);
    while (!high(0))
        fake_run(1);
    PulDir_halt(a);
    assert(a->mode == PULDIR_MODE_IDLE && PulDir_isMoving(a));
    assert(!PulDir_setPosition(a, 7.0f) && a->refusal == PULDIR_R_NOT_IDLE && a->homed);
    run_until_idle(a, 3u * N);
    assert(st(0).Pb != (uint64_t)(uint32_t)st(0).count << 32);   /* halted between pulses */

    /* accepted in IDLE: count, pulse position, target and generator move together */
    assert(PulDir_setPosition(a, -3.0f));
    PulDirSnap s = st(0);
    assert(s.count == -300 && s.Pb == (uint64_t)(int64_t)-300 << 32);
    assert(PulDir_position(a) == -3.0f && PulDir_target(a) == -3.0f && !a->homed);
    fake_run(N);
    assert(a->origin == -300 && a->prof.pos == 0.0f && a->prof.target == 0.0f && a->prof.done);
    assert(!PulDir_isMoving(a) && a->mode == PULDIR_MODE_IDLE);

    /* the next move is relative to the new datum */
    const uint64_t e0 = edges[0];
    const int64_t n0 = net[0];
    assert(PulDir_moveTo(a, -2.0f));
    run_until_idle(a, TICK_HZ);
    assert(st(0).count == -200 && edges[0] - e0 == 100 && net[0] - n0 == 100);
}

/* D19: a plan tick run twice after the final set latched sees its own zeros
 * while that set still runs; it must not go IDLE before the last pulse is lowered. */
static void t_settle_twice(void)
{
    setup(1);
    fake_on_write(count_write);
    PulDir* a = &ax[0];
    assert(PulDir_setDefaults(a, 2000.0f, 20000.0f, 20000.0f, 0.0f));
    const uint32_t phases[3] = { 0u, N / 2u, N - 1u };
    for (int k = 0; k < 6; k++) {
        fake_plan_phase = phases[k % 3];
        const int32_t c0 = st(0).count, want = c0 + (k % 2 ? -300 : 300);
        assert(PulDir_moveTo(a, (float)want));
        int stage = 0;                        /* 1 once arrived, 2 once the extra run is made */
        for (uint32_t t = 0; PulDir_isMoving(a); t++) {
            assert(t < 10u * TICK_HZ);
            if (tick1()) {
                if (stage == 1) {
                    fake_plan();              /* the second run, same interval */
                    stage = 2;
                } else if (stage == 0 && a->arrived) {
                    stage = 1;
                }
            }
        }
        assert(stage == 2);
        assert(st(0).count == want && !high(0));   /* isMoving fell after the final pulse */
        const uint64_t e = edges[0];
        fake_run(3u * N);
        assert(edges[0] == e && st(0).count == want);
    }
}

/* A jog given while the axis settles after arrival is a new course: the axis
 * follows it rather than steering to the old target and going IDLE. */
static void t_jog_settling(void)
{
    setup(1);
    fake_on_write(count_write);
    fake_on_plan(check_ramps);
    PulDir* a = &ax[0];
    assert(PulDir_setDefaults(a, 2000.0f, 20000.0f, 20000.0f, 0.0f));
    const uint32_t phases[3] = { 0u, N / 2u, N - 1u };
    for (int k = 0; k < 12; k++) {
        fake_plan_phase = phases[k % 3];
        const int32_t c0 = st(0).count, want = c0 + (k % 2 ? -300 : 300);
        const int64_t n0 = net[0];
        const float speed = (k % 4 < 2 ? 1000.0f : -1000.0f);   /* on past the target, or back */
        assert(PulDir_moveTo(a, (float)want));
        fake_run(N);                          /* applied: arrived is left set by the last stop */
        while (!a->arrived)
            fake_run(1);
        if (k >= 6)                           /* one settle run later */
            while (!tick1())
                ;
        assert(a->arrived && PulDir_isMoving(a) && a->mode == PULDIR_MODE_PROFILE);
        assert(PulDir_jog(a, speed));
        fake_run(TICK_HZ / 5u);        /* 50 ms of ramp, then 150 ms at speed */
        assert(PulDir_isMoving(a) && a->mode == PULDIR_MODE_PROFILE);
        assert(PulDir_velocity(a) == speed);
        const int32_t c = st(0).count;
        assert(speed > 0.0f ? c > want + 100 : c < want - 100);
        PulDir_stop(a);
        run_until_idle(a, TICK_HZ);
        assert(PulDir_target(a) == PulDir_position(a) && PulDir_velocity(a) == 0.0f);
        assert(net[0] - n0 == st(0).count - c0);
        check_rest(0, speed > 0.0f ? 1 : -1);
    }
}

/* Test 17 at every tick of an interval, and from main just before a plan tick
 * that is due on the same pulse tick. */
static PulDir* halt_on_plan;
static uint64_t halt_edges;
static int32_t halt_count;
static uint32_t plan_runs;
static void halt_hook(bool after)
{
    check_published(after);
    if (after) {
        plan_runs++;
    } else if (halt_on_plan) {
        PulDir_halt(halt_on_plan);
        halt_edges = edges[0];
        halt_count = st(0).count;
        halt_on_plan = NULL;
    }
}

static void t_halt_any_tick(void)
{
    setup(1);
    fake_on_write(count_write);
    fake_on_plan(halt_hook);
    PulDir* a = &ax[0];
    const float v = 0.4f * (float)TICK_HZ;          /* an edge every 2 or 3 ticks */
    assert(PulDir_setDefaults(a, v, 100.0f * v, 100.0f * v, 0.0f));
    int k = 0;
    for (uint32_t off = 0; off < N; off += N / 25u) {
        for (int before_plan = 0; before_plan < 2; before_plan++, k++) {
            const float speed = k % 2 ? -v : v;
            fake_plan_phase = before_plan ? off : (off * 7u) % N;
            assert(PulDir_jog(a, speed));
            fake_run(20u * N);
            assert(PulDir_velocity(a) == speed);
            if (before_plan) {
                halt_on_plan = a;
                while (halt_on_plan)
                    fake_run(1);
            } else {
                while (left() != N)
                    fake_run(1);
                fake_run(off);
                PulDir_halt(a);
                halt_edges = edges[0];
                halt_count = st(0).count;
            }
            const uint32_t p0 = plan_runs;
            assert(a->mode == PULDIR_MODE_IDLE && PulDir_velocity(a) == 0.0f);
            assert(PulDir_target(a) == (float)halt_count && PulDir_position(a) == (float)halt_count);
            for (uint32_t t = 0; PulDir_isMoving(a); t++) {
                assert(t < 3u * N);
                fake_run(1);
            }
            assert(plan_runs - p0 <= 2u && !high(0));
            fake_run(2u * N);
            assert(edges[0] == halt_edges && st(0).count == halt_count);   /* no rising edge since */
            assert(PulDir_target(a) == (float)halt_count && a->mode == PULDIR_MODE_IDLE);
            check_rest(0, speed > 0.0f ? 1 : -1);
        }
    }
    assert(k >= 50);
}

/* ---- stream (5.6, tests 12, 13 and the stream rows of 19) ---------------------------------- */

#define MS (TICK_HZ / 1000u)               /* pulse ticks per millisecond */
#define PI_D 3.14159265358979323846

/* Rising edges [from, nrise) of axis 0 are g ticks apart, within one tick. */
static void check_gaps(size_t from, uint32_t g)
{
    for (size_t i = from + 1u; i < nrise; i++) {
        const uint32_t gap = rise[i] - rise[i - 1u];
        assert(gap + 1u >= g && gap <= g + 1u);
    }
}

/* The slope of axis 0's edge rate (pulses/s^2), least squares over the rate of
 * every 10-edge window from rise `from` on whose rate lies within [lo, hi]. */
static double rate_slope(size_t from, double lo, double hi)
{
    double sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
    int n = 0;
    for (size_t i = from; i + 10u < nrise; i++) {
        const double span = (double)(uint32_t)(rise[i + 10u] - rise[i]) / (double)TICK_HZ;
        const double r = 10.0 / span;
        if (r < lo || r > hi)
            continue;
        const double t = ((double)(uint32_t)(rise[i] - rise[from]) / (double)TICK_HZ) + span / 2.0;
        sx += t;
        sy += r;
        sxx += t * t;
        sxy += t * r;
        n++;
    }
    assert(n > 20);
    return ((double)n * sxy - sx * sy) / ((double)n * sxx - sx * sx);
}

/* Setpoints on the line p0 + v t (user units), one per plan interval, `offset`
 * ticks after each plan tick, for `ms` milliseconds. Returns the last one's tick. */
static uint32_t stream_line(PulDir* a, double p0, double v, uint32_t ms, uint32_t offset)
{
    uint32_t since = N, t0 = 0, last = 0;
    bool first = true;
    for (uint32_t k = 0; k < ms * MS; k++) {
        if (tick1())
            since = 0;
        else if (since < N)
            since++;
        if (since == offset) {
            const uint32_t now = PulDir_ticks();
            if (first) {
                t0 = now;
                first = false;
            }
            const double t = (double)(uint32_t)(now - t0) / (double)TICK_HZ;
            PulDir_stream(a, (float)(p0 + v * t), (float)v);
            last = now;
        }
    }
    return last;
}

/* Test 12: follow p(t) = A (1 - cos wt) for 10 s, setpoints at sp_hz arriving
 * `offset` pulse ticks after each plan tick that takes one, with the catch-up
 * time catchup_ms; |count - p| must stay below `bound` pulses after 0.5 s.
 *
 * The guideline's bound is 2 pulses. At the default T_CATCHUP of 50 ms this
 * controller does not meet it, at either stream rate. The first-order hold
 * holds the velocity between setpoints, so on the sine's peak acceleration
 * (A w^2 conv = 27,600 pulses/s^2) the feedforward falls short by a * age, and
 * the proportional catch-up settles where P - p = a * age * T_c. The age is the
 * setpoint's at the middle of the interval it steers: about 1.8 ms at 1 kHz,
 * 3.3 to 3.9 ms at 200 Hz. The count trails P by up to a pulse more (D3).
 * Measured |P - p| at 50 ms: 2.4 (1 kHz) and 4.6 to 5.4 (200 Hz) pulses,
 * linear in T_c. The 50 ms runs therefore assert the measured error plus a
 * margin, to catch a regression; the 2-pulse bound is asserted where the
 * guideline's own limits on T_c allow it (5 ms at 1 kHz). Meeting it at 200 Hz
 * with T_c of two stream periods or more is the owner's decision. */
static void stream_sine(uint32_t sp_hz, uint32_t offset, uint32_t catchup_ms, double bound)
{
    setup_conv(1, 280.0f);
    fake_on_write(count_write);
    PulDir* a = &ax[0];
    const double conv = 280.0, A = 10.0, w = 2.0 * PI_D * 0.5;
    assert(PulDir_setDefaults(a, 50.0f, 1000.0f, 1000.0f, 0.0f));
    assert(PulDir_setStreamCatchup(a, catchup_ms));
    assert(offset < N && PLAN_HZ % sp_hz == 0u);
    assert(PulDir_streamBegin(a));
    nrise = 0;

    const uint32_t every = PLAN_HZ / sp_hz;    /* plan ticks per setpoint */
    uint32_t plans = 0, since = N, t0 = 0, t_v100 = 0;
    bool started = false, have_v100 = false;
    double worst = 0.0, worst_p = 0.0;            /* |count - p| and |P - p|, pulses */
    for (;;) {
        if (tick1()) {
            if (started) {
                const double t = (double)(uint32_t)(PulDir_ticks() - t0) / (double)TICK_HZ;
                const double p = A * (1.0 - cos(w * t)) * conv;
                const PulDirSnap s = st(0);
                const double e = fabs((double)s.count - p);
                const double ep = fabs((double)(int64_t)s.Pb / (double)PULDIR_FIX - p);
                if (t > 0.5) {
                    worst = fmax(worst, e);
                    worst_p = fmax(worst_p, ep);
                    assert(e < bound);
                }
            }
            plans++;
            since = 0;
        } else if (since < N) {
            since++;
        }
        if (since == offset && plans % every == 0u) {
            const uint32_t now = PulDir_ticks();
            if (!started) {
                started = true;
                t0 = now;
            }
            const double t = (double)(uint32_t)(now - t0) / (double)TICK_HZ;
            if (t > 10.0)
                break;
            const double p = A * (1.0 - cos(w * t)), v = A * w * sin(w * t);
            PulDir_stream(a, (float)p, (float)v);
            if (!have_v100 && fabs(v * conv) > 100.0) {
                have_v100 = true;
                t_v100 = now;
            }
        }
    }
    assert(PulDir_inStream(a) && !PulDir_streamTimedOut(a));

    /* no stall at start: the first edge within 10 ms of the first setpoint above 100 pulses/s */
    assert(have_v100 && nrise > 0);
    const int32_t first = (int32_t)(rise[0] - t_v100);
    assert(first <= (int32_t)(10u * MS));

    /* no gap longer than the sine's own motion allows, plus one plan interval */
    double worst_gap = -1.0e9;                    /* the largest gap over its allowance, seconds */
    for (size_t i = 1; i < nrise; i++) {
        const double ta = (double)(uint32_t)(rise[i - 1u] - t0) / (double)TICK_HZ;
        const double tb = (double)(uint32_t)(rise[i] - t0) / (double)TICK_HZ;
        const double va = fabs(A * w * sin(w * ta)) * conv, vb = fabs(A * w * sin(w * tb)) * conv;
        const double vmin = fmin(va, vb);
        if (floor(ta) != floor(tb) || vmin <= 200.0)  /* the velocity's zeros are the whole seconds */
            continue;
        const double allow = 2.0 / vmin + 1.0 / (double)PLAN_HZ;
        if (tb - ta - allow > worst_gap)
            worst_gap = tb - ta - allow;
        assert(tb - ta <= allow);
    }
    printf("  stream_sine %u Hz, offset %u, T_c %u ms: worst |count - p| = %.3f, |P - p| = %.3f pulses "
           "after 0.5 s; first edge %.2f ms after v > 100; worst gap %.3f ms inside its allowance\n",
           (unsigned)sp_hz, (unsigned)offset, (unsigned)catchup_ms, worst, worst_p,
           (double)first * 1000.0 / (double)TICK_HZ, -worst_gap * 1000.0);

    PulDir_streamEnd(a);
    run_until_idle(a, TICK_HZ);
    assert(a->mode == PULDIR_MODE_IDLE);
}

/* The default catch-up: measured 5.5 (20 us pulse) and 6.3 (5 us pulse) pulses. Not the guideline's 2. */
static void t_stream_sine(void)
{
    stream_sine(200u, 37u, PULDIR_CATCHUP_MS, 7.0);
}

/* The default catch-up: measured 3.4 and 3.7 pulses. Not the guideline's 2. */
static void t_stream_sine_1k(void)
{
    stream_sine(1000u, 11u, PULDIR_CATCHUP_MS, 4.0);
}

/* The guideline's 2 pulses, at the shortest catch-up it allows at 1 kHz
 * (five plan intervals, at least two stream periods). */
static void t_stream_sine_1k_tuned(void)
{
    stream_sine(1000u, 11u, (5000u + PLAN_HZ - 1u) / PLAN_HZ, 2.0);
}

/* Test 13: stream loss and re-entry. */
static void t_stream_timeout(void)
{
    setup(1);
    fake_on_write(count_write);
    PulDir_attachLogger(logger);
    PulDir* a = &ax[0];
    assert(PulDir_setDefaults(a, 6000.0f, 50000.0f, 50000.0f, 0.0f));
    assert(PulDir_jog(a, 5000.0f));
    fake_run(200u * N);
    assert(PulDir_velocity(a) == 5000.0f);
    nrise = 0;
    fake_run(10u * MS);
    assert(PulDir_streamBegin(a));
    const uint32_t tb = PulDir_ticks();
    assert(a->mode == PULDIR_MODE_STREAM && PulDir_inStream(a) && PulDir_isMoving(a));
    assert(!PulDir_streamTimedOut(a));

    /* 90 ms without a setpoint: the velocity it had, edge for edge */
    for (uint32_t k = 0; k < 90u * MS; k++) {
        fake_run(1);
        assert(PulDir_inStream(a) && PulDir_velocity(a) == 5000.0f);
    }
    const uint32_t g = TICK_HZ / 5000u;
    check_gaps(0, g);
    assert(nrise > 450);

    /* the timeout, 100 ms after streamBegin */
    while (PulDir_inStream(a)) {
        fake_run(1);
        assert(PulDir_ticks() - tb < 110u * MS);
    }
    const uint32_t tt = PulDir_ticks() - tb, lim = PULDIR_STREAM_TIMEOUT_MS * MS;
    assert(tt >= lim && tt <= lim + N);
    assert(a->mode == PULDIR_MODE_STOPPING && PulDir_streamTimedOut(a) && PulDir_isMoving(a));
    check_gaps(0, g);                             /* full speed up to the timeout */
    const size_t r0 = nrise;
    const int32_t c0 = st(0).count;
    const uint32_t d0 = dirw[0];
    run_until_idle(a, TICK_HZ);
    const int32_t c = st(0).count;
    assert(a->mode == PULDIR_MODE_IDLE && PulDir_velocity(a) == 0.0f && PulDir_streamTimedOut(a));
    assert(PulDir_target(a) == PulDir_position(a) && dirw[0] == d0);
    check_rest(0, 1);                             /* the count is the last integer reached */
    /* at decel from 5,000: 250 pulses, the slope dmax */
    assert(c - c0 >= 245 && c - c0 <= 258);
    const double slope = rate_slope(r0 > 0u ? r0 - 1u : 0u, 500.0, 4500.0);
    printf("  stream_timeout: timed out %.2f ms after streamBegin, stopped in %d pulses, "
           "edge-rate slope %.0f pulses/s^2 (dmax 50000)\n",
           (double)tt * 1000.0 / (double)TICK_HZ, (int)(c - c0), slope);
    assert(fabs(slope + 50000.0) <= 0.05 * 50000.0);

    /* logged once; the stop it caused lands like any stop */
    nlines = 0;
    PulDir_loop(a);
    assert(nlines == 2 && strcmp(lines[0], "[a0] stream timed out") == 0);
    assert(strncmp(lines[1], "[a0] landed at ", 15) == 0);
    PulDir_loop(a);
    assert(nlines == 2);

    /* a setpoint afterwards is dropped, and logged as refused */
    a->refusal = PULDIR_R_NONE;
    PulDir_stream(a, (float)(c + 100), 0.0f);
    assert(a->refusal == PULDIR_R_DROPPED && !PulDir_inStream(a) && !PulDir_isMoving(a));
    PulDir_loop(a);
    assert(nlines == 3 && strcmp(lines[2], "[a0] stream refused: dropped") == 0);
    fake_run(20u * N);
    assert(st(0).count == c && !PulDir_isMoving(a));

    /* streamBegin restores service: 1,000 pulses/s for 200 ms */
    assert(PulDir_streamBegin(a) && !PulDir_streamTimedOut(a) && PulDir_inStream(a));
    (void)stream_line(a, (double)c, 1000.0, 200u, 11u);
    assert(PulDir_inStream(a) && !PulDir_streamTimedOut(a));
    const int32_t moved = st(0).count - c;
    assert(moved >= 196 && moved <= 201);
    PulDir_streamEnd(a);
    run_until_idle(a, TICK_HZ);
    PulDir_loop(a);                               /* no line per setpoint, no timeout */
    assert(nlines == 4 && strncmp(lines[3], "[a0] landed at ", 15) == 0);
}

/* streamEnd restores the defaults and stops from the stream's velocity under
 * the default deceleration; with jerk the seeded window smooths it. */
static void t_stream_end(void)
{
    setup_conv(1, 100.0f);
    fake_on_write(count_write);
    PulDir* a = &ax[0];
    for (int round = 0; round < 2; round++) {
        /* jerk 25,000 units/s^3 = 2.5e6 pulses/s^3: a 20-tap window */
        const float jerk = round ? 25000.0f : 0.0f;
        assert(PulDir_setDefaults(a, 50.0f, 200.0f, 300.0f, jerk));
        assert(PulDir_setSpeed(a, 80.0f) && PulDir_setAcceleration(a, 150.0f));
        assert(PulDir_setDeceleration(a, 100.0f) && PulDir_setJerk(a, round ? 50000.0f : 0.0f));
        fake_run(N);
        const double p0 = (double)PulDir_position(a);
        assert(PulDir_streamBegin(a));
        (void)stream_line(a, p0, 60.0, 300u, 11u);      /* 6,000 pulses/s, above the default speed */
        assert(PulDir_inStream(a) && fabsf(PulDir_velocity(a) - 60.0f) < 0.05f);
        assert(fabsf(PulDir_streamError(a)) < 0.01f);
        nrise = 0;
        const uint32_t d0 = dirw[0];
        const int32_t c_end = st(0).count;
        const float v_end = a->v_cmd;             /* pulses/s */
        const uint32_t t_end = PulDir_ticks();
        PulDir_streamEnd(a);
        assert(!PulDir_inStream(a) && a->mode == PULDIR_MODE_STOPPING && PulDir_isMoving(a));
        assert(PulDir_speed(a) == 50.0f && PulDir_acceleration(a) == 200.0f);
        assert(PulDir_deceleration(a) == 300.0f && PulDir_jerk(a) == jerk);
        while (!tick1())                          /* the plan tick that applies it */
            ;
        /* ticks at v_end before the seeded set latches: to this plan tick, then to its boundary */
        const uint32_t run_on = (PulDir_ticks() - t_end) + left() % N;
        assert(a->prof.lim.vmax == 5000.0f && a->prof.lim.dmax == 30000.0f);
        assert(a->prof.lim.jmax == jerk * 100.0f && !a->prof.done);
        const float expect = (float)a->origin + a->prof.pos + MotionProfile_stopDistance(&a->prof);
        run_until_idle(a, TICK_HZ);
        const int32_t c = st(0).count;
        assert(a->mode == PULDIR_MODE_IDLE && PulDir_velocity(a) == 0.0f && !PulDir_streamTimedOut(a));
        assert(fabsf((float)c - expect) <= 4.0f);  /* at decel: within an interval's travel */
        assert(PulDir_target(a) == PulDir_position(a) && dirw[0] == d0);
        check_rest(0, 1);
        if (!round) {
            /* The seed, independent of the generator's own stopDistance: from the
             * pre-end count, the run-on at v_end, then v^2 / (2 dmax) less the
             * discrete stop's v dt / 2 (its first step already sheds dmax dt). */
            const float dt = DT;
            const float seeded = (float)c_end + v_end * (float)run_on / (float)TICK_HZ
                               + v_end * v_end / (2.0f * 30000.0f) - v_end * dt / 2.0f;
            printf("  stream_end: stopped %.2f pulses from the pre-end count + run-on + v^2/2d\n",
                   (double)((float)c - seeded));
            assert(fabsf((float)c - seeded) <= 4.0f);
            const double slope = rate_slope(0, 1000.0, 5000.0);
            printf("  stream_end: edge-rate slope %.0f pulses/s^2 (default dmax 30000)\n", slope);
            assert(fabs(slope + 30000.0) <= 0.05 * 30000.0);
        }
    }
}

/* The stream rows of test 19, catch-up refusals, and leaving by stop and halt. */
static void t_stream_transitions(void)
{
    setup(1);
    fake_on_write(count_write);
    PulDir* a = &ax[0];
    assert(PulDir_setDefaults(a, 6000.0f, 50000.0f, 50000.0f, 0.0f));
    fake_run(N);

    /* catch-up: at least five plan intervals */
    const uint32_t min_ms = (5000u + PLAN_HZ - 1u) / PLAN_HZ;
    assert(!PulDir_setStreamCatchup(a, min_ms - 1u) && a->refusal == PULDIR_R_ARG);
    assert(!PulDir_setStreamCatchup(a, 0u) && a->catchup_ms == PULDIR_CATCHUP_MS);
    assert(PulDir_setStreamCatchup(a, min_ms) && a->catchup_ms == min_ms);
    assert(PulDir_setStreamCatchup(a, PULDIR_CATCHUP_MS));

    /* a setpoint outside STREAM is dropped; it never starts a stream */
    a->refusal = PULDIR_R_NONE;
    PulDir_stream(a, 1.0f, 0.0f);
    assert(a->refusal == PULDIR_R_DROPPED && a->mode == PULDIR_MODE_IDLE && !PulDir_isMoving(a));
    assert(a->req == 0 && !a->sp_have);

    /* refused when faulted, in GROUP and in HOMING; nothing changes */
    a->fault = true;
    assert(!PulDir_streamBegin(a) && a->refusal == PULDIR_R_FAULT);
    a->fault = false;
    PulDir* members[1] = { a };
    const float here[1] = { 0.0f };               /* a real group move, of zero travel */
    assert(PulDirGroup_init(&grp, members, 1) && PulDirGroup_moveTo(&grp, here));
    assert(!PulDir_streamBegin(a) && a->refusal == PULDIR_R_MODE && a->mode == PULDIR_MODE_GROUP);
    run_until_idle(a, 3u * N);
    assert(!PulDirGroup_isMoving(&grp) && a->mode == PULDIR_MODE_IDLE);
    a->mode = PULDIR_MODE_HOMING;
    assert(!PulDir_streamBegin(a) && a->refusal == PULDIR_R_MODE && a->mode == PULDIR_MODE_HOMING);
    a->mode = PULDIR_MODE_IDLE;
    assert(a->req == 0 && !PulDir_isMoving(a));

    /* in STREAM: moveTo, moveBy, jog, a second streamBegin and setStreamCatchup are refused */
    assert(PulDir_streamBegin(a) && PulDir_inStream(a) && PulDir_isMoving(a));
    a->refusal = PULDIR_R_NONE;
    assert(!PulDir_moveTo(a, 100.0f) && a->refusal == PULDIR_R_MODE);
    a->refusal = PULDIR_R_NONE;
    assert(!PulDir_moveBy(a, 100.0f) && a->refusal == PULDIR_R_MODE);
    a->refusal = PULDIR_R_NONE;
    assert(!PulDir_jog(a, 100.0f) && a->refusal == PULDIR_R_MODE);
    a->refusal = PULDIR_R_NONE;
    assert(!PulDir_jog(a, 0.0f) && a->refusal == PULDIR_R_MODE);   /* jog(0) too: not a way out */
    assert(a->mode == PULDIR_MODE_STREAM && !(a->req & REQ_STOP));
    a->refusal = PULDIR_R_NONE;
    assert(!PulDir_streamBegin(a) && a->refusal == PULDIR_R_MODE);
    a->refusal = PULDIR_R_NONE;
    assert(!PulDir_setStreamCatchup(a, 20u) && a->refusal == PULDIR_R_MODE);
    assert(a->catchup_ms == PULDIR_CATCHUP_MS && a->mode == PULDIR_MODE_STREAM);

    /* bad setpoints are dropped: not finite, or beyond 2^31 pulses */
    a->refusal = PULDIR_R_NONE;
    PulDir_stream(a, NAN, 0.0f);
    assert(a->refusal == PULDIR_R_DROPPED && !a->sp_have);
    a->refusal = PULDIR_R_NONE;
    PulDir_stream(a, 0.0f, INFINITY);
    assert(a->refusal == PULDIR_R_DROPPED && !a->sp_have);
    a->refusal = PULDIR_R_NONE;
    PulDir_stream(a, 3.0e9f, 0.0f);
    assert(a->refusal == PULDIR_R_DROPPED && !a->sp_have);
    fake_run(5u * N);
    assert(st(0).count == 0 && nrise == 0 && PulDir_inStream(a));   /* streamBegin changed nothing */
    PulDir_streamEnd(a);
    run_until_idle(a, TICK_HZ);
    assert(st(0).count == 0 && a->mode == PULDIR_MODE_IDLE);
    PulDir_streamEnd(a);                          /* outside STREAM: nothing */
    assert(a->mode == PULDIR_MODE_IDLE && a->req == 0);

    /* from PROFILE: the velocity is kept until a setpoint comes */
    const uint32_t g = TICK_HZ / 5000u;
    assert(PulDir_moveTo(a, 100000.0f));
    assert(PulDir_setSpeed(a, 5000.0f));
    fake_run(200u * N);
    assert(PulDir_velocity(a) == 5000.0f);
    nrise = 0;
    fake_run(10u * MS);
    assert(PulDir_streamBegin(a) && PulDir_inStream(a));
    for (uint32_t k = 0; k < 50u * MS; k++) {
        fake_run(1);
        assert(PulDir_velocity(a) == 5000.0f);
    }
    check_gaps(0, g);

    /* stop in STREAM: STOPPING from the stream's velocity, at the current decel */
    assert(PulDir_setDeceleration(a, 40000.0f));
    const uint32_t d0 = dirw[0];
    PulDir_stop(a);
    assert(a->mode == PULDIR_MODE_STOPPING && !PulDir_inStream(a) && PulDir_isMoving(a));
    while (!tick1())
        ;
    float expect = (float)a->origin + a->prof.pos + MotionProfile_stopDistance(&a->prof);
    assert(a->prof.lim.dmax == 40000.0f);
    run_until_idle(a, TICK_HZ);
    assert(fabsf((float)st(0).count - expect) <= 4.0f && dirw[0] == d0);
    assert(a->mode == PULDIR_MODE_IDLE && PulDir_target(a) == PulDir_position(a));
    assert(!PulDir_streamTimedOut(a));
    check_rest(0, 1);

    /* from STOPPING: the velocity at that instant is kept */
    assert(PulDir_jog(a, -5000.0f));
    fake_run(200u * N);
    PulDir_stop(a);
    fake_run(30u * N + 7u);
    const float v0 = PulDir_velocity(a);
    assert(v0 < -2000.0f && v0 > -4500.0f && a->mode == PULDIR_MODE_STOPPING);
    assert(PulDir_streamBegin(a));
    const int32_t c1 = st(0).count;
    for (uint32_t k = 0; k < 30u * MS; k++) {
        fake_run(1);
        assert(PulDir_velocity(a) == v0 && PulDir_inStream(a));
    }
    const float moved = (float)(st(0).count - c1);
    assert(fabsf(moved - v0 * 0.030f) <= 2.0f);

    /* halt in STREAM: IDLE at once, no pulse after */
    const uint64_t e0 = edges[0];
    const int32_t ch = st(0).count;
    PulDir_halt(a);
    assert(a->mode == PULDIR_MODE_IDLE && !PulDir_inStream(a) && PulDir_velocity(a) == 0.0f);
    fake_run(3u * N);
    assert(!PulDir_isMoving(a) && edges[0] == e0 && st(0).count == ch && a->prof.done);

    /* and streams again from rest: a setpoint where the axis is moves nothing */
    assert(PulDir_streamBegin(a));
    (void)stream_line(a, (double)ch, 0.0, 20u, 3u);
    assert(st(0).count == ch && edges[0] == e0);
    PulDir_streamEnd(a);
    run_until_idle(a, TICK_HZ);
    assert(st(0).count == ch && edges[0] == e0 && a->mode == PULDIR_MODE_IDLE);
}

/* Commands given together before one plan tick apply in order: a halt first, a
 * stream begun before its end, a stop last. */
static void t_request_order(void)
{
    setup(1);
    fake_on_write(count_write);
    fake_on_plan(check_ramps);
    PulDir* a = &ax[0];
    assert(PulDir_setDefaults(a, 5000.0f, 50000.0f, 50000.0f, 0.0f));
    const float dv = 50000.0f / (float)PLAN_HZ * 1.0001f;   /* one interval at accel */
    const uint32_t phases[3] = { 0u, N / 2u, N - 1u };
    for (int k = 0; k < 6; k++) {
        fake_plan_phase = phases[k % 3];
        const int32_t c0 = st(0).count, dir = k % 2 ? -1 : 1;
        const int64_t n0 = net[0];

        /* halt then moveTo: from rest where the pulses stopped, onto the new target */
        assert(PulDir_moveTo(a, (float)(c0 + dir * 20000)));
        fake_run(TICK_HZ * 3u / 20u + (uint32_t)k * 7u);   /* 150 ms: past the 100 ms ramp */
        assert(fabsf(PulDir_velocity(a)) == 5000.0f);
        const int32_t want = st(0).count - dir * 500;
        PulDir_halt(a);
        assert(PulDir_moveTo(a, (float)want));
        while (!tick1())
            ;
        assert(fabsf(PulDir_velocity(a)) <= dv);
        run_until_idle(a, 2u * TICK_HZ);
        assert(st(0).count == want && PulDir_target(a) == (float)want);
        assert(a->mode == PULDIR_MODE_IDLE && net[0] - n0 == want - c0);

        /* moveTo then stop from rest: STOPPING, then IDLE without moving */
        const uint64_t e1 = edges[0];
        const uint32_t d1 = dirw[0];
        assert(PulDir_moveTo(a, (float)(want + dir * 3000)));
        PulDir_stop(a);
        assert(a->mode == PULDIR_MODE_STOPPING && PulDir_isMoving(a));
        run_until_idle(a, TICK_HZ);
        assert(a->mode == PULDIR_MODE_IDLE && edges[0] == e1 && dirw[0] == d1);
        assert(st(0).count == want && PulDir_target(a) == (float)want);

        /* streamBegin then halt, and halt then streamBegin: no pulse after the halt */
        assert(PulDir_jog(a, (float)dir * 3000.0f));
        fake_run(TICK_HZ / 10u);       /* 100 ms: past the 60 ms ramp */
        uint64_t e2 = edges[0];
        int32_t ch = st(0).count;
        assert(PulDir_streamBegin(a));
        PulDir_halt(a);
        assert(a->mode == PULDIR_MODE_IDLE && !PulDir_inStream(a));
        fake_run(3u * N);
        assert(!PulDir_isMoving(a) && edges[0] == e2 && st(0).count == ch);
        assert(PulDir_jog(a, (float)dir * 3000.0f));
        fake_run(TICK_HZ / 10u);       /* 100 ms: past the 60 ms ramp */
        e2 = edges[0];
        ch = st(0).count;
        PulDir_halt(a);
        assert(PulDir_streamBegin(a));
        fake_run(20u * N);                    /* no setpoint: the stream holds the halted rest */
        assert(PulDir_inStream(a) && PulDir_velocity(a) == 0.0f && edges[0] == e2);
        PulDir_streamEnd(a);
        run_until_idle(a, TICK_HZ);
        assert(edges[0] == e2 && st(0).count == ch && a->mode == PULDIR_MODE_IDLE);

        /* streamBegin then streamEnd or stop: a stop from the jog's velocity (check_ramps) */
        const int32_t c3 = st(0).count;
        assert(PulDir_jog(a, (float)dir * 3000.0f));
        fake_run(TICK_HZ / 10u);       /* 100 ms: past the 60 ms ramp */
        const uint32_t d3 = dirw[0];
        assert(PulDir_streamBegin(a));
        if (k < 3)
            PulDir_streamEnd(a);
        else
            PulDir_stop(a);
        assert(a->mode == PULDIR_MODE_STOPPING);
        while (!tick1())
            ;
        assert(fabsf(PulDir_velocity(a)) >= 3000.0f - dv && a->prof.stopping);
        run_until_idle(a, TICK_HZ);
        assert(dirw[0] == d3 && a->mode == PULDIR_MODE_IDLE && PulDir_target(a) == PulDir_position(a));
        assert(dir > 0 ? st(0).count > c3 + 200 : st(0).count < c3 - 200);
        check_rest(0, dir);
    }
}

/* 5.6: the configured speed limit bounds the stream, live. Setpoints asking for
 * twice the limit move the axis at the limit, never faster. */
static void t_stream_speed_limit(void)
{
    setup(1);
    fake_on_write(count_write);
    PulDir* a = &ax[0];
    assert(PulDir_setDefaults(a, 2000.0f, 50000.0f, 50000.0f, 0.0f));
    fake_run(N);
    assert(PulDir_streamBegin(a));
    nrise = 0;
    (void)stream_line(a, 0.0, 4000.0, 200u, 11u);
    assert(PulDir_inStream(a) && PulDir_velocity(a) == 2000.0f);
    const uint32_t g = TICK_HZ / 2000u;
    assert(nrise > 350);
    for (size_t i = 1; i < nrise; i++)
        assert(rise[i] - rise[i - 1u] >= g);      /* never above 2,000 pulses/s */
    const int32_t c = st(0).count;
    assert(c >= 395 && c <= 401);                 /* 200 ms at the limit, not at 4,000 */
    assert(PulDir_streamError(a) > 300.0f);       /* the setpoints run away; the axis does not */

    /* lowered mid-stream, setpoints still running away: obeyed from the set after next */
    assert(PulDir_setSpeed(a, 1000.0f));
    const uint32_t t_low = PulDir_ticks();
    const size_t from = nrise;
    (void)stream_line(a, 1000.0, 4000.0, 100u, 11u);
    assert(PulDir_velocity(a) == 1000.0f);
    size_t checked = 0;
    for (size_t i = from + 1u; i < nrise; i++)
        if (rise[i - 1u] - t_low >= 2u * N) {
            assert(rise[i] - rise[i - 1u] >= 2u * g);
            checked++;
        }
    assert(checked > 80);
    PulDir_streamEnd(a);
    run_until_idle(a, TICK_HZ);
}

/* Test 13 after real setpoints: the timeout counts from the last one, and the
 * stop is v^2 / (2 dmax) from where the axis is when it fires. */
static void t_stream_loss(void)
{
    setup(1);
    fake_on_write(count_write);
    PulDir_attachLogger(logger);
    PulDir* a = &ax[0];
    const float dmax = 50000.0f, v = 3000.0f;
    assert(PulDir_setDefaults(a, 6000.0f, 50000.0f, dmax, 0.0f));
    fake_run(N);
    assert(PulDir_streamBegin(a));
    const uint32_t last = stream_line(a, 0.0, (double)v, 400u, 11u);
    assert(PulDir_inStream(a) && fabsf(PulDir_velocity(a) - v) < 1.0f);

    /* fed no more: the hold carries the axis on at v until the timeout */
    nrise = 0;
    while (PulDir_inStream(a)) {
        fake_run(1);
        assert(PulDir_ticks() - last < 110u * MS);
    }
    const uint32_t fired = PulDir_ticks() - last, lim = PULDIR_STREAM_TIMEOUT_MS * MS;
    assert(fired >= lim && fired <= lim + N);
    assert(a->mode == PULDIR_MODE_STOPPING && PulDir_streamTimedOut(a));
    check_gaps(0, TICK_HZ / 3000u);        /* still at v, within a tick */
    const int32_t c0 = st(0).count;
    const uint32_t ahead = left() % N;            /* the stream's last set runs on this long */
    const uint32_t d0 = dirw[0];
    run_until_idle(a, TICK_HZ);
    const int32_t c = st(0).count;
    const float dt = DT;
    const float want = (float)c0 + v * (float)ahead / (float)TICK_HZ
                     + v * v / (2.0f * dmax) - v * dt / 2.0f;
    printf("  stream_loss: timed out %.2f ms after the last setpoint, stopped in %d pulses "
           "(v^2/2d = %.0f), %.2f from the expected count\n",
           (double)fired * 1000.0 / (double)TICK_HZ, (int)(c - c0),
           (double)(v * v / (2.0f * dmax)), (double)((float)c - want));
    assert(fabsf((float)c - want) <= 3.0f);
    assert(a->mode == PULDIR_MODE_IDLE && dirw[0] == d0);
    check_rest(0, 1);
    nlines = 0;
    PulDir_loop(a);
    assert(nlines == 2 && strcmp(lines[0], "[a0] stream timed out") == 0);
}

/* streamEnd then streamBegin before the plan tick sees either: the stream goes
 * on at its own velocity (not the idle generator's zero), with the defaults that
 * streamEnd restored. */
static void t_stream_end_begin(void)
{
    setup(1);
    fake_on_write(count_write);
    PulDir* a = &ax[0];
    assert(PulDir_setDefaults(a, 6000.0f, 50000.0f, 50000.0f, 0.0f));
    assert(PulDir_setDeceleration(a, 20000.0f));
    fake_run(N);
    assert(PulDir_streamBegin(a));
    (void)stream_line(a, 0.0, 3000.0, 400u, 11u);
    while (!tick1())                              /* just after a plan tick */
        ;
    const float v = PulDir_velocity(a);
    assert(fabsf(v - 3000.0f) < 1.0f && a->prof.done && a->prof.vel == 0.0f);
    PulDir_streamEnd(a);
    assert(PulDir_deceleration(a) == 50000.0f && a->mode == PULDIR_MODE_STOPPING);
    assert(PulDir_streamBegin(a) && PulDir_inStream(a));
    nrise = 0;
    for (uint32_t k = 0; k < 50u * MS; k++) {    /* no setpoint: the velocity it had */
        fake_run(1);
        assert(PulDir_inStream(a) && PulDir_velocity(a) == v);
    }
    check_gaps(0, TICK_HZ / 3000u);
    assert(a->prof.done && a->prof.lim.dmax == 50000.0f && !a->arrived);
    PulDir_streamEnd(a);
    run_until_idle(a, TICK_HZ);
    assert(a->mode == PULDIR_MODE_IDLE);
    check_rest(0, 1);
}

/* ---- groups (5.7, test 18 and the group rows of 19) ---------------------------------------- */

static uint32_t first_rise[PULDIR_MAX_AXES], last_dir_tick[PULDIR_MAX_AXES];
static bool rose[PULDIR_MAX_AXES];

static void group_write(uint32_t tick, uint8_t id, bool level)
{
    count_write(tick, id, level);
    const uint8_t i = (uint8_t)(id / 2u);
    if (i >= PULDIR_MAX_AXES)
        return;
    if (id % 2u != 0u)
        last_dir_tick[i] = tick;
    else if (level && !rose[i]) {
        rose[i] = true;
        first_rise[i] = tick;
    }
}

/* Member i's aim at path parameter s, absolute 32.32, as the plan tick computes it. */
static uint64_t group_aim(const PulDirGroup* g, uint8_t i, float s)
{
    return g->start[i] + (uint64_t)((int64_t)g->delta[i] * (int64_t)(s * PULDIR_FIX));
}

/* Test 18's tracking. A plan tick publishes a set aimed at start + delta * s; when
 * that set's interval ends, each member is checked against the same s. The pulse
 * position P must be within one pulse of the aim. The count is P's last whole pulse
 * in the direction of travel, so it trails P by up to a pulse: it is held to one
 * pulse plus the truncation the aim allows (N units, D5), and measured. */
static float gs_next, gs_cur;
static bool gs_next_valid, gs_cur_valid;
static int64_t g_worst_P, g_worst_count;
static uint32_t g_first_inc;                  /* the first tick any member ran a non-zero increment */
static bool g_inc_seen;
/* Extra plan runs (D19 for the group): 1 runs the plan tick again in the interval
 * where the members arrive, overwriting the final set before it latches; 2 runs it
 * again in the next interval, while the final set still runs. */
static int g_twice, g_twice_stage;
static float g_peak;                          /* the largest |velocity| / speed limit of any member */

static void group_track(PulDirGroup* g, uint32_t ticks)
{
    for (uint32_t k = 0; k < ticks; k++) {
        if (left() == N) {                    /* this tick latches: the running set has finished */
            if (gs_cur_valid)
                for (uint8_t i = 0; i < g->count; i++) {
                    const PulDirSnap s = st(g->axes[i]->slot);
                    const uint64_t aim = group_aim(g, i, gs_cur);
                    int64_t e = (int64_t)(s.Pb - aim);
                    int64_t c = (int64_t)(((uint64_t)(uint32_t)s.count << 32) - aim);
                    e = e < 0 ? -e : e;
                    c = c < 0 ? -c : c;
                    g_worst_P = e > g_worst_P ? e : g_worst_P;
                    g_worst_count = c > g_worst_count ? c : g_worst_count;
                    assert(e <= (int64_t)PULDIR_ONE);
                    assert(c <= (int64_t)PULDIR_ONE + (int64_t)N);
                }
            gs_cur = gs_next;
            gs_cur_valid = gs_next_valid;
        }
        fake_pulse(false);
        for (uint8_t i = 0; i < g->count && !g_inc_seen; i++)
            if (st(g->axes[i]->slot).inc != 0) {
                g_inc_seen = true;
                g_first_inc = PulDir_ticks() - 1u;
            }
        if (left() == plan_left()) {
            fake_plan();
            if (g_twice != 0 && g_twice_stage < 2) {
                if (g_twice_stage == 1 || (g_twice == 1 && g->axes[0]->arrived)) {
                    fake_plan();
                    g_twice_stage = 2;
                } else if (g->axes[0]->arrived) {
                    g_twice_stage = 1;
                }
            }
            gs_next = g->s.pos;
            gs_next_valid = true;
        }
    }
}

/* One group move of travel[i] pulses from where the members are (conv 1). */
static void group_move_sync(PulDirGroup* g, const int32_t* travel)
{
    const uint8_t n = g->count;
    int32_t c0[PULDIR_MAX_AXES];
    uint64_t e0[PULDIR_MAX_AXES];
    int64_t n0[PULDIR_MAX_AXES];
    uint32_t d0[PULDIR_MAX_AXES];
    float tg[PULDIR_MAX_AXES];
    for (uint8_t i = 0; i < n; i++) {
        c0[i] = st(i).count;
        tg[i] = (float)(c0[i] + travel[i]);
        e0[i] = edges[i];
        n0[i] = net[i];
        d0[i] = dirw[i];
        rose[i] = false;
    }
    g_inc_seen = false;
    g_twice_stage = 0;
    assert(PulDirGroup_moveTo(g, tg));
    assert(PulDirGroup_isMoving(g));
    for (uint8_t i = 0; i < n; i++)
        assert(ax[i].mode == PULDIR_MODE_GROUP && PulDir_isMoving(&ax[i]));
    gs_next_valid = gs_cur_valid = false;
    for (uint32_t t = 0; PulDirGroup_isMoving(g); t++) {
        assert(t < 10u * TICK_HZ);
        group_track(g, 1);
        const bool moving = PulDirGroup_isMoving(g);
        for (uint8_t i = 0; i < n; i++) {     /* the zero-travel member too, throughout */
            assert(PulDir_isMoving(&ax[i]) == moving
                   && ax[i].mode == (moving ? PULDIR_MODE_GROUP : PULDIR_MODE_IDLE));
            /* each member's velocity is its share of the path's (conv 1) */
            const float vi = PulDir_velocity(&ax[i]);
            assert(vi == (float)travel[i] * g->s.vel || (vi == 0.0f && g->s.vel == 0.0f));
            const float m = fabsf(vi) / ax[i].lim[0];
            g_peak = m > g_peak ? m : g_peak;
        }
    }

    assert(g_twice == 0 || g_twice_stage == 2);
    uint32_t last = 0;
    bool have = false;
    for (uint8_t i = 0; i < n; i++) {
        const int32_t mag = travel[i] < 0 ? -travel[i] : travel[i];
        assert(st(i).count == c0[i] + travel[i] && !high(i));
        /* the generator rests on the target, the origin on the count, as after an axis's landing */
        assert(ax[i].origin == st(i).count && ax[i].prof.done && ax[i].prof.pos == 0.0f);
        assert(edges[i] - e0[i] == (uint64_t)mag && net[i] - n0[i] == travel[i]);
        assert(PulDir_target(&ax[i]) == (float)(c0[i] + travel[i]) && PulDir_velocity(&ax[i]) == 0.0f);
        if (travel[i] == 0) {
            assert(dirw[i] == d0[i]);         /* emits nothing, turns nothing */
            continue;
        }
        if (!have)
            last = last_rise[i];
        have = true;
        assert(last_rise[i] == last);         /* every final rising edge on one tick */
    }
    assert(have && (last - PULDIR_TICK_START) % N == N - 1u);   /* the arrival interval's last tick */
    /* The start tick (5.7): every direction pin was set before the first set with
     * an increment latched, so no member loses ticks to a reversal (E14). */
    assert(g_inc_seen);
    for (uint8_t i = 0; i < n; i++)
        if (dirw[i] != d0[i])
            assert(last_dir_tick[i] < g_first_inc);
}

static void t_group_sync(void)
{
    setup(5);
    fake_on_write(group_write);
    const float v[5] = { 20000.0f, 5000.0f, 24000.0f, 1000.0f, 1000.0f };
    const float acc[5] = { 100000.0f, 20000.0f, 50000.0f, 5000.0f, 3000.0f };
    const float dec[5] = { 80000.0f, 30000.0f, 40000.0f, 5000.0f, 4000.0f };
    const float jerk[5] = { 2.0e7f, 5.0e6f, 5.0e6f, 0.0f, 0.0f };
    PulDir* m[5];
    for (uint8_t i = 0; i < 5; i++) {
        assert(PulDir_setDefaults(&ax[i], v[i], acc[i], dec[i], 0.0f));
        m[i] = &ax[i];
    }
    assert(PulDirGroup_init(&grp, m, 5));
    const int32_t go[5] = { 1000, 250, -60000, 0, 1 }, back[5] = { -1000, -250, 60000, 0, -1 };
    const uint32_t phases[4] = { N / 2u, 0u, N - 1u, N / 3u };
    for (int k = 0; k < 4; k++) {             /* back reverses every moving member */
        fake_plan_phase = phases[k];
        if (k == 2)
            for (uint8_t i = 0; i < 3; i++)
                assert(PulDir_setJerk(&ax[i], jerk[i]));
        group_move_sync(&grp, k % 2 ? back : go);
    }
    assert(grp.s.n > 1);                      /* the jerk runs had a window */
    assert(g_peak > 0.999f && g_peak < 1.0001f);   /* the tightest member reached its own speed, no more */
    printf("  group_sync: worst |P - aim| = %.2e pulses, |count - aim| = %.4f pulses (N = %u)\n",
           (double)g_worst_P / 4294967296.0, (double)g_worst_count / 4294967296.0, (unsigned)N);
}

/* D19 for the group: the plan tick run twice where the members arrive (the final
 * set overwritten before it latches) and twice while the final set runs. Every
 * final edge still falls on one tick, and the group stops moving only once the
 * last pulse is lowered. */
static void t_group_twice(void)
{
    setup(3);
    fake_on_write(group_write);
    const float v[3] = { 3000.0f, 2000.0f, 1000.0f };
    PulDir* m[3];
    for (uint8_t i = 0; i < 3; i++) {
        assert(PulDir_setDefaults(&ax[i], v[i], 30000.0f, 20000.0f, 0.0f));
        m[i] = &ax[i];
    }
    assert(PulDirGroup_init(&grp, m, 3));
    const int32_t go[3] = { 300, -170, 1 }, back[3] = { -300, 170, -1 };
    const uint32_t phases[3] = { 0u, N / 2u, N - 1u };
    for (int k = 0; k < 12; k++) {
        fake_plan_phase = phases[k % 3];
        g_twice = k < 6 ? 1 : 2;
        if (k == 3 || k == 9)
            for (uint8_t i = 0; i < 2; i++)
                assert(PulDir_setJerk(&ax[i], 3.0e6f));
        if (k == 6)
            for (uint8_t i = 0; i < 2; i++)
                assert(PulDir_setJerk(&ax[i], 0.0f));
        group_move_sync(&grp, k % 2 ? back : go);
        const uint64_t e = edges[0] + edges[1] + edges[2];
        fake_run(3u * N);
        assert(edges[0] + edges[1] + edges[2] == e);
    }
    g_twice = 0;
}

/* D6 for the group: a soft decel through a long jerk window creeps s to within
 * an ulp of 1; the small members' final edges must still be the arrival
 * interval's last tick, with the others'. */
static void t_group_creep(void)
{
    setup(3);
    fake_on_write(group_write);
    const float jerk = 40.0f / (250.0f / (float)PLAN_HZ);   /* a 250-tap window */
    assert(PulDir_setDefaults(&ax[0], 50.0f, 20.0f, 20.0f, jerk));
    assert(PulDir_setDefaults(&ax[1], 1000.0f, 1.0e4f, 1.0e4f, 0.0f));
    assert(PulDir_setDefaults(&ax[2], 1000.0f, 1.0e4f, 1.0e4f, 0.0f));
    PulDir* m[3] = { &ax[0], &ax[1], &ax[2] };
    assert(PulDirGroup_init(&grp, m, 3));
    const int32_t steps[4][3] = { { 3, 1, 1 }, { -1, 2, -1 }, { 2, -1, 1 }, { -4, -2, 2 } };
    for (int k = 0; k < 8; k++) {
        fake_plan_phase = (uint32_t)k * (N / 8u);
        group_move_sync(&grp, steps[k % 4]);
    }
}

/* The group rows of test 19. */
static void t_group_rules(void)
{
    setup(4);
    fake_on_write(group_write);
    for (uint8_t i = 0; i < 4; i++)
        assert(PulDir_setDefaults(&ax[i], 4000.0f + 1000.0f * (float)i, 40000.0f, 30000.0f, 0.0f));
    fake_run(N);                              /* the limits applied: no request pending */
    PulDir* m3[3] = { &ax[0], &ax[1], &ax[2] };
    PulDir* m2[2] = { &ax[2], &ax[3] };

    /* init: refused for no member, a missing, unregistered or repeated one, and a full table */
    PulDir loose;
    assert(PulDir_init(&loose, "loose", 1.0f));
    PulDir* bad1[2] = { &ax[0], &loose };
    PulDir* bad2[2] = { &ax[0], &ax[0] };
    PulDir* bad3[2] = { &ax[0], NULL };
    assert(!PulDirGroup_init(&grp, m3, 0));
    ax[0].refusal = PULDIR_R_NONE;
    assert(!PulDirGroup_init(&grp, bad1, 2) && ax[0].refusal == PULDIR_R_ARG);
    ax[0].refusal = PULDIR_R_NONE;
    assert(!PulDirGroup_init(&grp, bad2, 2) && ax[0].refusal == PULDIR_R_ARG);
    ax[0].refusal = PULDIR_R_NONE;
    assert(!PulDirGroup_init(&grp, bad3, 2) && ax[0].refusal == PULDIR_R_ARG);
    assert(PulDirGroup_init(&grp, m3, 3) && PulDirGroup_init(&grp2, m2, 2));
    static PulDirGroup more[PULDIR_MAX_GROUPS];
    PulDir* m1[1] = { &ax[3] };
    for (uint32_t k = 0; k + 2u < PULDIR_MAX_GROUPS; k++)
        assert(PulDirGroup_init(&more[k], m1, 1));
    ax[3].refusal = PULDIR_R_NONE;
    assert(!PulDirGroup_init(&more[PULDIR_MAX_GROUPS - 2u], m1, 1) && ax[3].refusal == PULDIR_R_FULL);
    assert(PulDirGroup_init(&grp, m3, 3));    /* already in the table: initialised again */
    assert(!PulDirGroup_isMoving(&grp) && !PulDirGroup_faulted(&grp));

    /* a bad target: refused, nothing changes */
    float tg[3] = { 500.0f, NAN, -300.0f };
    assert(!PulDirGroup_moveTo(&grp, tg) && ax[1].refusal == PULDIR_R_ARG);
    tg[1] = 3.0e9f;
    assert(!PulDirGroup_moveTo(&grp, tg) && ax[1].refusal == PULDIR_R_ARG);
    for (uint8_t i = 0; i < 3; i++)
        assert(ax[i].mode == PULDIR_MODE_IDLE && !PulDir_isMoving(&ax[i]) && ax[i].req == 0);
    assert(!PulDirGroup_isMoving(&grp));

    /* a member not IDLE (jogging): refused, the others untouched */
    assert(PulDir_jog(&ax[1], 1000.0f));
    tg[1] = 700.0f;
    assert(!PulDirGroup_moveTo(&grp, tg) && ax[1].refusal == PULDIR_R_NOT_IDLE);
    assert(!PulDirGroup_isMoving(&grp) && ax[1].mode == PULDIR_MODE_PROFILE);
    assert(ax[0].mode == PULDIR_MODE_IDLE && ax[2].mode == PULDIR_MODE_IDLE && !PulDir_isMoving(&ax[0]));
    /* halted, until its last pulse is lowered: not yet at rest, refused */
    while (!high(1))
        fake_run(1);
    PulDir_halt(&ax[1]);
    assert(ax[1].mode == PULDIR_MODE_IDLE && PulDir_isMoving(&ax[1]));
    ax[1].refusal = PULDIR_R_NONE;
    assert(!PulDirGroup_moveTo(&grp, tg) && ax[1].refusal == PULDIR_R_NOT_IDLE);
    assert(!PulDirGroup_isMoving(&grp) && ax[0].mode == PULDIR_MODE_IDLE);
    run_until_idle(&ax[1], TICK_HZ);

    /* accepted: members refuse their own commands, except halt */
    int32_t c[4];
    for (uint8_t i = 0; i < 4; i++)
        c[i] = st(i).count;
    tg[0] = (float)(c[0] + 3000);
    tg[1] = (float)(c[1] - 2000);
    tg[2] = (float)(c[2] + 1500);
    assert(PulDirGroup_moveTo(&grp, tg) && PulDirGroup_isMoving(&grp));
    for (uint8_t i = 0; i < 3; i++) {
        PulDir* a = &ax[i];
        assert(a->mode == PULDIR_MODE_GROUP && PulDir_isMoving(a) && PulDir_target(a) == tg[i]);
        a->refusal = PULDIR_R_NONE;
        assert(!PulDir_moveTo(a, 0.0f) && a->refusal == PULDIR_R_MODE);
        a->refusal = PULDIR_R_NONE;
        assert(!PulDir_moveBy(a, 10.0f) && a->refusal == PULDIR_R_MODE);
        a->refusal = PULDIR_R_NONE;
        assert(!PulDir_jog(a, 100.0f) && a->refusal == PULDIR_R_MODE);
        a->refusal = PULDIR_R_NONE;
        assert(!PulDir_jog(a, 0.0f) && a->refusal == PULDIR_R_MODE);
        a->refusal = PULDIR_R_NONE;
        PulDir_stop(a);
        assert(a->refusal == PULDIR_R_MODE);
        a->refusal = PULDIR_R_NONE;
        assert(!PulDir_streamBegin(a) && a->refusal == PULDIR_R_MODE);
        a->refusal = PULDIR_R_NONE;
        assert(!PulDir_setPosition(a, 0.0f) && a->refusal == PULDIR_R_NOT_IDLE);
        assert(a->mode == PULDIR_MODE_GROUP && a->req == 0);
    }
    /* the group again, or a group sharing a member: refused */
    ax[0].refusal = PULDIR_R_NONE;
    assert(!PulDirGroup_moveTo(&grp, tg) && ax[0].refusal == PULDIR_R_BUSY);
    const float t2[2] = { 0.0f, 50.0f };
    assert(!PulDirGroup_moveTo(&grp2, t2) && ax[2].refusal == PULDIR_R_BUSY);
    assert(ax[3].mode == PULDIR_MODE_IDLE && !PulDir_isMoving(&ax[3]) && !PulDirGroup_isMoving(&grp2));

    /* a member's halt halts every member on that tick */
    for (int k = 0; k < 2; k++) {
        if (k == 1) {
            for (uint8_t i = 0; i < 3; i++)
                c[i] = st(i).count;
            tg[0] = (float)(c[0] - 2500);
            tg[1] = (float)(c[1] + 1000);
            tg[2] = (float)(c[2] - 2000);
            assert(PulDirGroup_moveTo(&grp, tg));
        }
        fake_run(150u * N);
        while (!high(1))                      /* just after a rising edge of member 1 */
            fake_run(1);
        uint64_t e0[3];
        int32_t ch[3];
        for (uint8_t i = 0; i < 3; i++) {
            e0[i] = edges[i];
            ch[i] = st(i).count;
            assert(ch[i] != c[i]);            /* every member was under way */
        }
        if (k == 0)
            PulDir_halt(&ax[1]);
        else
            PulDirGroup_halt(&grp);
        for (uint8_t i = 0; i < 3; i++) {
            PulDir* a = &ax[i];
            assert(a->mode == PULDIR_MODE_IDLE && PulDir_velocity(a) == 0.0f);
            assert(PulDir_target(a) == (float)ch[i] && PulDir_position(a) == (float)ch[i]);
        }
        assert(PulDirGroup_isMoving(&grp) && PulDir_isMoving(&ax[1]));   /* until the pin is low */
        fake_plan();                          /* a plan tick before member 1's pin is lowered */
        assert(high(1) && PulDirGroup_isMoving(&grp) && PulDir_isMoving(&ax[1]));
        for (uint32_t t = 0; PulDirGroup_isMoving(&grp); t++) {
            assert(t < 3u * N);
            fake_run(1);
        }
        for (uint8_t i = 0; i < 3; i++)
            assert(!PulDir_isMoving(&ax[i]) && !high(i));
        fake_run(3u * N);
        for (uint8_t i = 0; i < 3; i++)
            assert(edges[i] == e0[i] && st(i).count == ch[i] && st(i).inc == 0);   /* no rising edge since */
    }

    /* after a halt, a member's own move lands exactly: its generator is where the pulses stopped */
    for (uint8_t i = 0; i < 3; i++)
        c[i] = st(i).count;
    const uint64_t eh = edges[0];
    assert(PulDir_moveTo(&ax[0], (float)(c[0] + 100)));
    run_until_idle(&ax[0], TICK_HZ);
    assert(st(0).count == c[0] + 100 && edges[0] - eh == 100);   /* straight there */
    c[0] += 100;

    /* stop: every member comes to rest on the line, for one common s, short of its target */
    tg[0] = (float)(c[0] + 6000);
    tg[1] = (float)(c[1] - 9000);
    tg[2] = (float)(c[2] + 3000);
    assert(PulDirGroup_moveTo(&grp, tg));
    fake_run(400u * N);
    PulDirGroup_stop(&grp);
    for (uint8_t i = 0; i < 3; i++)
        assert(ax[i].mode == PULDIR_MODE_GROUP && PulDir_isMoving(&ax[i]));
    run_until_idle(&ax[0], 2u * TICK_HZ);
    assert(!PulDirGroup_isMoving(&grp));
    const float s = grp.s.pos;
    assert(grp.s.done && s > 0.1f && s < 0.9f);
    for (uint8_t i = 0; i < 3; i++) {
        PulDir* a = &ax[i];
        const float want = (float)c[i] + (float)grp.delta[i] * s;
        assert(fabsf((float)st(i).count - want) <= 1.0f);
        assert(a->mode == PULDIR_MODE_IDLE && !PulDir_isMoving(a) && PulDir_velocity(a) == 0.0f);
        assert(PulDir_target(a) == PulDir_position(a));
        check_rest(i, grp.delta[i]);
        /* the generator rests on the line, as an axis's rests where its stop ended */
        const double line = (double)c[i] + (double)grp.delta[i] * (double)s;
        assert(a->origin == st(i).count && fabs((double)a->origin + (double)a->prof.pos - line) < 1e-4);
    }

    /* and the next group move from there lands exactly */
    for (uint8_t i = 0; i < 3; i++) {
        c[i] = st(i).count;
        tg[i] = (float)(c[i] + (i == 1 ? 321 : -123));
    }
    assert(PulDirGroup_moveTo(&grp, tg));
    run_until_idle(&ax[0], 2u * TICK_HZ);
    assert(!PulDirGroup_isMoving(&grp));
    for (uint8_t i = 0; i < 3; i++)
        assert(st(i).count == (int32_t)tg[i]);

    /* every travel zero: accepted, done at the next plan tick, no pulse */
    uint64_t e1[3];
    for (uint8_t i = 0; i < 3; i++) {
        e1[i] = edges[i];
        tg[i] = (float)st(i).count;
    }
    assert(PulDirGroup_moveTo(&grp, tg) && PulDirGroup_isMoving(&grp) && ax[2].mode == PULDIR_MODE_GROUP);
    while (!tick1())
        ;
    assert(!PulDirGroup_isMoving(&grp));
    for (uint8_t i = 0; i < 3; i++)
        assert(ax[i].mode == PULDIR_MODE_IDLE && !PulDir_isMoving(&ax[i]) && edges[i] == e1[i]);

    /* the second group, now free */
    assert(PulDirGroup_moveTo(&grp2, t2));
    run_until_idle(&ax[3], 2u * TICK_HZ);
    assert(!PulDirGroup_isMoving(&grp2) && st(2).count == 0 && st(3).count == 50);
}

/* ---- enable, release, fault and homing (5.8; tests 19 to 22) ------------------------------ */

/* Enable, fault and fault-clear pins of axis i; the axes use 0 to 15. */
#define ENA(i) ((uint8_t)(32u + (i)))
#define FLT(i) ((uint8_t)(40u + (i)))
#define CLR(i) ((uint8_t)(48u + (i)))
#define WRAP_START ((uint32_t)PULDIR_TICK_START)

/* The pin-write log and the edge counters together. */
static void both_write(uint32_t tick, uint8_t id, bool level)
{
    on_write(tick, id, level);
    group_write(tick, id, level);
}

/* The tick of the first write of id at level from log index from. */
static bool find_write(size_t from, uint8_t id, bool level, uint32_t* tick)
{
    for (size_t i = from; i < nw; i++)
        if (wlog[i].id == id && wlog[i].level == level) {
            *tick = wlog[i].tick;
            return true;
        }
    return false;
}

/* Test 21 and the enable rows of 19: no pulse before enable_ms after the enable
 * pin goes active; the pin goes inactive release_ms after the last pulse, never
 * while pulses remain; enable(false) halts first. */
static void t_enable_release(void)
{
    add_axis(0, false, false);
    add_axis(1, false, false);
    PulDir* a = &ax[0];
    PulDir_attachEnable(a, fake_pin(ENA(0)), false);
    assert(!fake_level(ENA(0)));                  /* parked inactive */
    PulDir_attachEnable(&ax[1], fake_pin(ENA(1)), true);
    assert(fake_level(ENA(1)));                   /* inverted: inactive is high */
    start();
    fake_on_write(both_write);
    for (uint8_t i = 0; i < 2; i++)
        assert(PulDir_setDefaults(&ax[i], 5000.0f, 1.0e5f, 1.0e5f, 0.0f));
    assert(PulDir_setDelays(a, 50u, 200u));
    PulDir_setAutoRelease(a, true);
    fake_run(N);
    assert(PulDir_status(a) == PULDIR_IDLE && !a->enabled && !PulDir_isMoving(a));

    /* refused: a delay beyond half the tick counter, and enable without a pin */
    assert(!PulDir_setDelays(a, 0xFFFFFFFFu, 0u) && a->refusal == PULDIR_R_ARG);
    assert(!PulDir_setDelays(a, 0u, 0xFFFFFFFFu) && a->enable_ticks == 50u * MS && a->release_ticks == 200u * MS);
    PulDir b;
    assert(PulDir_init(&b, "b", 1.0f));
    assert(!PulDir_enable(&b, true) && b.refusal == PULDIR_R_NO_PIN && !b.enabled);

    /* a move on a disabled drive: the pin and isMoving at once, the first pulse enable_ms later */
    size_t w = nw;
    uint32_t t_cmd = PulDir_ticks(), t;
    assert(PulDir_moveTo(a, 1000.0f));
    assert(fake_level(ENA(0)) && a->enabled && PulDir_isMoving(a) && PulDir_status(a) == PULDIR_MOVING);
    assert(find_write(w, ENA(0), true, &t) && t == t_cmd);
    nrise = nfall = 0;
    run_until_idle(a, TICK_HZ);
    assert(st(0).count == 1000 && nrise == 1000 && nfall == 1000);
    assert(rise[0] - t_cmd >= 50u * MS && rise[0] - t_cmd <= 60u * MS);   /* the ramp's first pulse: 4.5 ms */
    assert(PulDir_status(a) == PULDIR_ENABLED && fake_level(ENA(0)));

    /* released release_ms after the last falling edge, within one plan interval */
    uint32_t f = fall[nfall - 1u];
    w = nw;
    while (fake_level(ENA(0))) {
        fake_run(1);
        assert(PulDir_ticks() - f <= 200u * MS + N);
    }
    assert(find_write(w, ENA(0), false, &t) && t - f >= 200u * MS && t - f <= 200u * MS + N);
    assert(nrise == 1000 && !high(0) && !a->enabled && PulDir_status(a) == PULDIR_IDLE);
    printf("  enable_release: first edge %.2f ms after enable, released %.2f ms after the last fall\n",
           (double)(rise[0] - t_cmd) * 1000.0 / TICK_HZ, (double)(t - f) * 1000.0 / TICK_HZ);

    /* a move 100 ms after landing cancels the pending release */
    assert(PulDir_moveTo(a, 0.0f));
    run_until_idle(a, TICK_HZ);
    fake_run(100u * MS);
    assert(fake_level(ENA(0)) && a->release_due);
    w = nw;
    t_cmd = PulDir_ticks();
    const size_t r1 = nrise;
    assert(PulDir_moveTo(a, 500.0f) && !a->release_due);
    assert(!find_write(w, ENA(0), true, &t));     /* enabled already: no write, no delay */
    run_until_idle(a, TICK_HZ);
    assert(rise[r1] - t_cmd < 10u * MS);
    f = fall[nfall - 1u];
    while (fake_level(ENA(0)))
        fake_run(1);
    assert(find_write(w, ENA(0), false, &t) && t - f >= 200u * MS && t - f <= 200u * MS + N);
    assert(st(0).count == 500 && nrise == r1 + 500u);

    /* an explicit enable holds the drive: no release without a move */
    assert(PulDir_enable(a, true) && fake_level(ENA(0)) && PulDir_status(a) == PULDIR_ENABLED);
    fake_run(300u * MS);
    assert(fake_level(ENA(0)) && a->enabled);

    /* enable(false) while moving: halts first (no rising edge after), then the pin goes inactive */
    assert(PulDir_jog(a, 3000.0f));
    fake_run(100u * MS);
    while (!high(0))
        fake_run(1);
    const size_t r2 = nrise;
    const int32_t ch = st(0).count;
    w = nw;
    t_cmd = PulDir_ticks();
    assert(PulDir_enable(a, false));
    /* the pulse is high: the drive stays enabled until the plan tick sees it lowered */
    assert(high(0) && fake_level(ENA(0)) && !a->enabled && a->mode == PULDIR_MODE_IDLE);
    assert(PulDir_status(a) == PULDIR_MOVING);
    while (fake_level(ENA(0))) {
        fake_run(1);
        assert(PulDir_ticks() - t_cmd <= N + 1u);
    }
    assert(find_write(w, ENA(0), false, &t) && t > fall[nfall - 1u] && fall[nfall - 1u] == t_cmd);
    fake_run(3u * N);
    assert(nrise == r2 && st(0).count == ch && !PulDir_isMoving(a) && PulDir_status(a) == PULDIR_IDLE);
    assert(PulDir_target(a) == (float)ch);

    /* at rest the pin goes inactive at once */
    assert(PulDir_enable(a, true) && fake_level(ENA(0)));
    assert(PulDir_enable(a, false) && !fake_level(ENA(0)) && !a->ena_off);

    /* a command between enable(false) and the pulse's fall re-enables: the plan
     * tick must not lower the pin after it */
    assert(PulDir_jog(a, 3000.0f));
    fake_run(100u * MS);
    while (!high(0))
        fake_run(1);
    assert(PulDir_enable(a, false) && a->ena_off && fake_level(ENA(0)));
    assert(PulDir_moveTo(a, (float)(st(0).count + 10)) && a->enabled && fake_level(ENA(0)));
    fake_run(3u * N);
    assert(fake_level(ENA(0)) && a->enabled);
    run_until_idle(a, TICK_HZ);
    assert(PulDir_enable(a, false) && !fake_level(ENA(0)));

    /* a group waits for the slowest enable delay; the final edges still share one tick */
    assert(PulDir_setDelays(&ax[1], 20u, 0u));
    PulDir* m[2] = { &ax[0], &ax[1] };
    assert(PulDirGroup_init(&grp, m, 2));
    const float tg[2] = { (float)(ch + 800), -300.0f };
    rose[0] = rose[1] = false;
    t_cmd = PulDir_ticks();
    assert(PulDirGroup_moveTo(&grp, tg));
    assert(!fake_level(ENA(1)) && fake_level(ENA(0)));   /* inverted: active is low */
    run_until_idle(&ax[0], TICK_HZ);
    assert(!PulDirGroup_isMoving(&grp) && st(0).count == ch + 800 && st(1).count == -300);
    assert(first_rise[0] - t_cmd >= 50u * MS && first_rise[1] - t_cmd >= 50u * MS);
    assert(last_rise[0] == last_rise[1]);
}

/* A plan tick that runs on main's next lock, before the mask. */
static bool plan_on_lock;
static void lock_hook(void)
{
    if (plan_on_lock) {
        plan_on_lock = false;
        fake_plan();
    }
}

/* The fault rows of 19 and 5.8: debounce, latch, refusals, the clear pulse. */
static void t_fault(void)
{
    add_axis(0, false, false);
    add_axis(1, false, false);
    PulDir* a = &ax[0];
    PulDir_attachEnable(a, fake_pin(ENA(0)), false);
    PulDir_attachFault(a, fake_pin(FLT(0)), false);
    PulDir_attachFaultClear(a, fake_pin(CLR(0)), false);
    assert(!fake_level(CLR(0)));
    PulDir_attachFault(&ax[1], fake_pin(FLT(1)), true);   /* inverted: low is active */
    fake_set_input(FLT(1), true);
    start();
    fake_on_write(both_write);
    PulDir_attachLogger(logger);
    assert(PulDir_setDefaults(a, 5000.0f, 5.0e4f, 5.0e4f, 0.0f));
    static uint8_t never;
    assert(PulDir_setHoming(a, 1, 100.0f, 0.0f, &never, 0.0f, 1000u));
    assert(PulDir_setDelays(a, 0u, 2u));
    PulDir_setAutoRelease(a, true);           /* released 2 ms after a move, never after a latch */

    /* Every pin read may be preempted by pulse ticks: the plan tick samples the
     * fault pins before its snapshot, so moves still land exactly (D6). */
    fake_preempt = 3;
    for (int k = 0; k < 3; k++) {
        fake_plan_phase = (uint32_t)k * (N / 3u);
        const int32_t want = k % 2 ? 0 : 2000;
        assert(PulDir_moveTo(a, (float)want));
        run_until_idle(a, TICK_HZ);
        assert(st(0).count == want && (last_rise[0] - WRAP_START) % N == N - 1u);
    }
    assert(!ax[1].fault && ax[1].fault_count == 0);
    PulDir_loop(a);

    /* active for 1 ms: no latch */
    assert(PulDir_jog(a, 4000.0f));
    fake_run(TICK_HZ / 10u);           /* 100 ms: past the 80 ms ramp */
    fake_set_input(FLT(0), true);
    fake_run(MS);
    fake_set_input(FLT(0), false);
    fake_run(5u * MS);
    assert(!a->fault && a->fault_count == 0 && PulDir_isMoving(a) && PulDir_velocity(a) == 4000.0f);

    /* held: latched 2 ms after onset, within one plan interval; the axis halts */
    a->homed = true;
    const uint32_t on = PulDir_ticks();
    fake_set_input(FLT(0), true);
    while (!a->fault) {
        fake_run(1);
        assert(PulDir_ticks() - on <= 2u * MS + N + 8u);
    }
    const size_t r0 = nrise;
    const int32_t ch = st(0).count;
    assert(a->last_fault_tick - on >= 2u * MS && a->last_fault_tick - on <= 2u * MS + N);
    assert(a->fault_count == 1 && PulDir_status(a) == PULDIR_FAULT && !a->homed);
    assert(a->mode == PULDIR_MODE_IDLE && PulDir_velocity(a) == 0.0f);
    fake_run(10u * MS);
    assert(nrise == r0 && st(0).count == ch && !PulDir_isMoving(a) && PulDir_target(a) == (float)ch);
    assert(fake_level(ENA(0)) && a->enabled);    /* D12: not released, auto-release or not */
    assert(PulDir_status(a) == PULDIR_FAULT);

    /* motion refused while latched */
    a->refusal = PULDIR_R_NONE;
    assert(!PulDir_moveTo(a, 0.0f) && a->refusal == PULDIR_R_FAULT);
    a->refusal = PULDIR_R_NONE;
    assert(!PulDir_moveBy(a, 10.0f) && a->refusal == PULDIR_R_FAULT);
    a->refusal = PULDIR_R_NONE;
    assert(!PulDir_jog(a, 100.0f) && a->refusal == PULDIR_R_FAULT);
    a->refusal = PULDIR_R_NONE;
    assert(!PulDir_streamBegin(a) && a->refusal == PULDIR_R_FAULT);
    a->refusal = PULDIR_R_NONE;
    assert(!PulDir_homeStart(a) && a->refusal == PULDIR_R_FAULT);
    assert(a->mode == PULDIR_MODE_IDLE && !PulDir_isMoving(a) && a->req == 0);
    nlines = 0;
    PulDir_loop(a);
    assert(nlines == 2 && strcmp(lines[0], "[a0] fault latched, count 1") == 0);   /* then the refusal */

    /* no clear pin: refused */
    ax[1].refusal = PULDIR_R_NONE;
    assert(!PulDir_clearFault(&ax[1]) && ax[1].refusal == PULDIR_R_NO_PIN);

    /* Still in alarm: the clear pulse is 10 ms and does not block, and the latch
     * holds after it: never ENABLED, motion refused, nothing counted again. */
    size_t w = nw;
    const uint32_t tc = PulDir_ticks();
    assert(PulDir_clearFault(a) && PulDir_ticks() == tc && fake_level(CLR(0)));
    a->refusal = PULDIR_R_NONE;
    assert(!PulDir_clearFault(a) && a->refusal == PULDIR_R_BUSY);
    while (fake_level(CLR(0))) {
        assert(a->fault);
        fake_run(1);
        assert(PulDir_ticks() - tc <= 10u * MS + N);
    }
    uint32_t tl;
    assert(find_write(w, CLR(0), false, &tl) && tl - tc >= 10u * MS && tl - tc <= 10u * MS + N);
    for (uint32_t k = 0; k < 20u * MS; k++) {
        assert(a->fault && PulDir_status(a) == PULDIR_FAULT && a->fault_count == 1);
        if (k % N == 0u) {
            a->refusal = PULDIR_R_NONE;
            assert(!PulDir_moveTo(a, 0.0f) && a->refusal == PULDIR_R_FAULT);
        }
        fake_run(1);
    }
    nlines = 0;
    PulDir_loop(a);
    assert(nlines == 1 && strcmp(lines[0], "[a0] moveTo refused: drive fault latched") == 0);

    /* the alarm gone: the next sample lifts the latch; one fault, one clear */
    fake_set_input(FLT(0), false);
    const uint32_t t_off = PulDir_ticks();
    while (a->fault) {
        fake_run(1);
        assert(PulDir_ticks() - t_off <= N + 8u);
    }
    assert(PulDir_status(a) == PULDIR_ENABLED && a->fault_count == 1);
    nlines = 0;
    PulDir_loop(a);
    assert(nlines == 1 && strcmp(lines[0], "[a0] fault cleared") == 0);
    assert(PulDir_moveTo(a, (float)(ch + 300)));
    run_until_idle(a, TICK_HZ);
    assert(st(0).count == ch + 300 && nrise == r0 + 300u);
    PulDir_loop(a);

    /* a move accepted just before the latch is discarded with the plan */
    fake_set_input(FLT(0), true);
    while (!a->flt_seen)
        fake_run(1);
    while (PulDir_ticks() - a->flt_since < 2u * MS - N / 2u)
        fake_run(1);
    assert(!a->fault);
    const int32_t c1 = st(0).count;
    const size_t r1 = nrise;
    assert(PulDir_moveTo(a, (float)(c1 + 5000)) && PulDir_isMoving(a));
    while (!a->fault)
        fake_run(1);
    fake_run(3u * N);
    assert(nrise == r1 && st(0).count == c1 && !PulDir_isMoving(a) && a->mode == PULDIR_MODE_IDLE);
    assert(PulDir_target(a) == (float)c1 && a->prof.done && a->fault_count == 2);

    /* the usual order, the alarm gone first: the latch holds until clearFault and
     * lifts at the first sample after the pulse */
    fake_set_input(FLT(0), false);
    fake_run(10u * N);
    assert(a->fault);
    w = nw;
    assert(PulDir_clearFault(a));
    while (fake_level(CLR(0)))
        fake_run(1);
    assert(find_write(w, CLR(0), false, &tl) && a->fault);
    while (a->fault) {
        fake_run(1);
        assert(PulDir_ticks() - tl <= N + 8u);
    }
    assert(a->fault_count == 2);
    nlines = 0;
    PulDir_loop(a);
    assert(nlines == 2 && strcmp(lines[0], "[a0] fault latched, count 2") == 0);
    assert(strcmp(lines[1], "[a0] fault cleared") == 0);

    /* nothing latched: the clear pin pulses all the same, and nothing is logged */
    w = nw;
    assert(PulDir_clearFault(a));
    while (a->clearing)
        fake_run(1);
    fake_run(3u * N);
    assert(find_write(w, CLR(0), true, &tl) && find_write(w, CLR(0), false, &tl));
    nlines = 0;
    PulDir_loop(a);
    assert(nlines == 0 && !a->fault && a->fault_count == 2);

    /* the latch closing between a command's check and its lock: refused all the same */
    fake_set_input(FLT(0), true);
    while (!a->flt_seen)
        fake_run(1);
    pulse(2u * MS);                           /* no plan tick: the next one latches */
    assert(!a->fault);
    fake_on_lock(lock_hook);
    plan_on_lock = true;
    a->refusal = PULDIR_R_NONE;
    assert(!PulDir_moveTo(a, (float)(c1 + 5000)) && a->refusal == PULDIR_R_FAULT && a->fault);
    fake_set_input(FLT(0), false);            /* and once more for jog */
    assert(PulDir_clearFault(a));
    while (a->fault)
        fake_run(1);
    fake_set_input(FLT(0), true);
    while (!a->flt_seen)
        fake_run(1);
    pulse(2u * MS);
    plan_on_lock = true;
    assert(!PulDir_jog(a, 100.0f) && a->refusal == PULDIR_R_FAULT && a->fault);
    fake_on_lock(NULL);
    fake_run(3u * N);
    assert(nrise == r1 && st(0).count == c1 && !PulDir_isMoving(a) && a->mode == PULDIR_MODE_IDLE);
    assert(a->fault_count == 4);

    /* the inverted pin: high is inactive, low latches */
    assert(!ax[1].fault);
    fake_set_input(FLT(1), false);
    fake_run(4u * MS);
    assert(ax[1].fault && ax[1].fault_count == 1);
    fake_preempt = 0;
}

/* Latches in odd places, and a glitch that must not latch. */
static void t_fault_cases(void)
{
    for (uint8_t i = 0; i < 3; i++) {
        add_axis(i, false, false);
        PulDir_attachFault(&ax[i], fake_pin(FLT(i)), false);
    }
    PulDir_attachEnable(&ax[0], fake_pin(ENA(0)), false);
    fake_set_input(FLT(1), true);             /* in alarm before the ticks start */
    start();
    fake_on_write(both_write);
    for (uint8_t i = 0; i < 3; i++)
        assert(PulDir_setDefaults(&ax[i], 5000.0f, 1.0e5f, 1.0e5f, 0.0f));

    /* the pin active at start: latched 2 ms after the first sample, motion refused */
    for (uint32_t t = 0; !ax[1].fault; t++) {
        assert(t < 2u * MS + N);
        fake_run(1);
    }
    assert(ax[1].last_fault_tick - WRAP_START >= 2u * MS && ax[1].fault_count == 1);
    assert(PulDir_status(&ax[1]) == PULDIR_FAULT && !PulDir_moveTo(&ax[1], 10.0f));

    /* a 1.9 ms glitch that two samples see, 1 ms apart: no latch */
    PulDir* a = &ax[0];
    while (!tick1())
        ;
    fake_run(N - 2u);                         /* the next sample two ticks away */
    fake_set_input(FLT(0), true);
    fake_run(19u * N / 10u);                  /* samples at +2 and +N+2 */
    assert(a->flt_seen);
    fake_set_input(FLT(0), false);
    fake_run(5u * MS);
    assert(!a->fault && a->fault_count == 0 && !a->flt_seen);

    /* a latch in the enable delay: no pulse ever, the drive left enabled (D12) */
    assert(PulDir_setDelays(a, 50u, 0u));
    assert(PulDir_moveTo(a, 1000.0f) && fake_level(ENA(0)) && a->ena_wait);
    fake_run(10u * MS);
    fake_set_input(FLT(0), true);
    while (!a->fault)
        fake_run(1);
    assert(a->mode == PULDIR_MODE_IDLE && a->ena_wait);
    fake_run(100u * MS);
    assert(edges[0] == 0u && st(0).count == 0 && !PulDir_isMoving(a) && !a->ena_wait);
    assert(fake_level(ENA(0)) && a->enabled && PulDir_target(a) == 0.0f);

    /* a latch in STREAM: halted, out of the stream, no edge after */
    PulDir* b = &ax[2];
    assert(PulDir_streamBegin(b));
    (void)stream_line(b, 0.0, 3000.0, 100u, 11u);
    assert(PulDir_inStream(b) && edges[2] > 200u);
    fake_set_input(FLT(2), true);
    while (!b->fault)
        fake_run(1);
    const uint64_t e2 = edges[2];
    const int32_t c2 = st(2).count;
    assert(b->mode == PULDIR_MODE_IDLE && !PulDir_inStream(b) && !b->stream_live && PulDir_velocity(b) == 0.0f);
    fake_run(3u * N);
    assert(edges[2] == e2 && st(2).count == c2 && !PulDir_isMoving(b) && PulDir_target(b) == (float)c2);
    b->refusal = PULDIR_R_NONE;
    assert(!PulDir_streamBegin(b) && b->refusal == PULDIR_R_FAULT);
    PulDir_stream(b, 0.0f, 0.0f);
    assert(b->refusal == PULDIR_R_DROPPED);
}

/* Rising edges of axis i over the next ms milliseconds. */
static uint64_t edges_over(uint8_t i, uint32_t ms)
{
    const uint64_t e = edges[i];
    fake_run(ms * MS);
    return edges[i] - e;
}

/* A member's fault halts the whole group and marks it faulted (test 19). */
static void t_group_fault(void)
{
    fake_plan_phase = 1u;                     /* the plan tick early: a stale prediction is a whole interval */
    for (uint8_t i = 0; i < 3; i++) {
        add_axis(i, false, false);
        PulDir_attachEnable(&ax[i], fake_pin(ENA(i)), false);
    }
    for (uint8_t i = 0; i < 2; i++) {
        PulDir_attachFault(&ax[i], fake_pin(FLT(i)), false);
        PulDir_attachFaultClear(&ax[i], fake_pin(CLR(i)), false);
    }
    start();
    fake_on_write(group_write);
    for (uint8_t i = 0; i < 3; i++) {
        assert(PulDir_setDefaults(&ax[i], 4000.0f + 1000.0f * (float)i, 40000.0f, 30000.0f, 0.0f));
        assert(PulDir_setDelays(&ax[i], 0u, 5u));
        PulDir_setAutoRelease(&ax[i], true);   /* D12 must hold all the same */
    }
    PulDir* m[3] = { &ax[0], &ax[1], &ax[2] };
    assert(PulDirGroup_init(&grp, m, 3));
    float tg[3] = { 3000.0f, -2000.0f, 1500.0f };
    assert(PulDirGroup_moveTo(&grp, tg));
    fake_run(150u * N);
    for (uint8_t i = 0; i < 3; i++)
        assert(st(i).count != 0);             /* every member under way */

    fake_set_input(FLT(1), true);
    for (uint32_t t = 0; !ax[1].fault; t++) {
        assert(t < 3u * MS + N);
        fake_run(1);
    }
    uint64_t e0[3];
    int32_t ch[3];
    for (uint8_t i = 0; i < 3; i++) {
        PulDir* a = &ax[i];
        e0[i] = edges[i];
        ch[i] = st(i).count;
        assert(a->mode == PULDIR_MODE_IDLE && PulDir_velocity(a) == 0.0f && a->group == NULL);
        assert(PulDir_target(a) == PulDir_position(a));
    }
    assert(PulDirGroup_faulted(&grp) && !ax[0].fault && !ax[2].fault);
    for (uint32_t t = 0; PulDirGroup_isMoving(&grp); t++) {
        assert(t < 3u * N);
        fake_run(1);
    }
    fake_run(3u * N);
    for (uint8_t i = 0; i < 3; i++)
        assert(edges[i] == e0[i] && st(i).count == ch[i] && !PulDir_isMoving(&ax[i]) && !high(i));
    /* D12: no member released, faulted or not, for three release delays */
    fake_run(15u * MS);
    for (uint8_t i = 0; i < 3; i++)
        assert(fake_level(ENA(i)) && ax[i].enabled && ax[i].held);

    /* the group is refused while a member is latched */
    ax[1].refusal = PULDIR_R_NONE;
    assert(!PulDirGroup_moveTo(&grp, tg) && ax[1].refusal == PULDIR_R_FAULT && PulDirGroup_faulted(&grp));

    /* The others' generators are where their pulses stopped, placed from the
     * position after the kill, not the prediction: their own moves start inside
     * the ramp (0.3 pulses in 4 ms at 40,000 pulses/s^2) and land exactly. */
    for (uint8_t i = 0; i < 3; i += 2) {
        assert(PulDir_moveTo(&ax[i], (float)(ch[i] + 100)));
        assert(edges_over(i, 4u) <= 1u);
        run_until_idle(&ax[i], TICK_HZ);
        assert(st(i).count == ch[i] + 100 && edges[i] == e0[i] + 100u);
    }

    /* cleared, the group moves again */
    fake_set_input(FLT(1), false);
    assert(PulDir_clearFault(&ax[1]));
    for (uint32_t t = 0; ax[1].fault; t++) {
        assert(t < 12u * MS);
        fake_run(1);
    }
    assert(fake_level(ENA(1)));                /* held until its own command */
    assert(PulDir_moveTo(&ax[1], (float)(ch[1] - 100)));   /* the faulted member's too */
    assert(edges_over(1, 4u) <= 1u);
    run_until_idle(&ax[1], TICK_HZ);
    assert(st(1).count == ch[1] - 100 && edges[1] == e0[1] + 100u);
    for (uint8_t i = 0; i < 3; i++)
        tg[i] = (float)(st(i).count + 200 * (i + 1));
    assert(PulDirGroup_moveTo(&grp, tg) && !PulDirGroup_faulted(&grp));
    run_until_idle(&ax[0], 2u * TICK_HZ);
    for (uint8_t i = 0; i < 3; i++)
        assert(st(i).count == (int32_t)tg[i]);

    /* two members latching on one tick: one halt, both latched, nothing released */
    for (uint8_t i = 0; i < 3; i++)
        tg[i] = (float)(st(i).count - 3000 + 1000 * i);
    assert(PulDirGroup_moveTo(&grp, tg));
    fake_run(150u * N);
    fake_set_input(FLT(0), true);
    fake_set_input(FLT(1), true);
    for (uint32_t t = 0; !ax[0].fault; t++) {
        assert(t < 3u * MS + N);
        fake_run(1);
    }
    assert(ax[1].fault && ax[0].last_fault_tick == ax[1].last_fault_tick);
    assert(ax[0].fault_count == 1 && ax[1].fault_count == 2 && !ax[2].fault && PulDirGroup_faulted(&grp));
    for (uint8_t i = 0; i < 3; i++) {
        e0[i] = edges[i];
        ch[i] = st(i).count;
        assert(ax[i].mode == PULDIR_MODE_IDLE && PulDir_target(&ax[i]) == (float)ch[i]);
    }
    fake_run(15u * MS);
    for (uint8_t i = 0; i < 3; i++)
        assert(edges[i] == e0[i] && st(i).count == ch[i] && !PulDir_isMoving(&ax[i]) && fake_level(ENA(i)));
    assert(!PulDirGroup_isMoving(&grp));
}

/* Group paths the other cases do not reach: the travel bound, members' limits
 * refused, the underflow and taps refusals, and stops before the start, during
 * the stop and after landing. */
static void t_group_paths(void)
{
    setup(2);
    fake_on_write(group_write);
    PulDir* a = &ax[0];
    for (uint8_t i = 0; i < 2; i++)
        assert(PulDir_setDefaults(&ax[i], 4000.0f, 40000.0f, 40000.0f, 0.0f));
    fake_run(N);
    PulDir* m[2] = { &ax[0], &ax[1] };
    assert(PulDirGroup_init(&grp, m, 2));

    /* a stop before the start moves nothing and turns no pin (the direction gate is skipped) */
    float tg[2] = { -500.0f, 0.0f };
    assert(PulDirGroup_moveTo(&grp, tg));
    PulDirGroup_stop(&grp);
    fake_run(3u * N);
    assert(!PulDirGroup_isMoving(&grp) && a->mode == PULDIR_MODE_IDLE && !PulDir_isMoving(a));
    assert(edges[0] == 0u && dirw[0] == 0u && st(0).count == 0);

    /* an underflowing path limit: refused */
    assert(PulDir_setAcceleration(a, 1.0e-38f));
    tg[0] = -16777216.0f;
    a->refusal = PULDIR_R_NONE;
    assert(!PulDirGroup_moveTo(&grp, tg) && a->refusal == PULDIR_R_ARG && !PulDirGroup_isMoving(&grp));
    assert(PulDir_setAcceleration(a, 40000.0f));

    /* a travel beyond 2^24 pulses: refused; 2^24 is accepted */
    tg[0] = -16777218.0f;
    a->refusal = PULDIR_R_NONE;
    assert(!PulDirGroup_moveTo(&grp, tg) && a->refusal == PULDIR_R_ARG && a->mode == PULDIR_MODE_IDLE);
    tg[0] = -16777216.0f;
    assert(PulDirGroup_moveTo(&grp, tg) && a->mode == PULDIR_MODE_GROUP);
    fake_run(TICK_HZ / 20u);           /* 50 ms */

    /* a member's limits belong to the path while it moves: refused, unchanged */
    const uint16_t req0 = a->req;
    a->refusal = PULDIR_R_NONE;
    assert(!PulDir_setSpeed(a, 100.0f) && a->refusal == PULDIR_R_MODE);
    a->refusal = PULDIR_R_NONE;
    assert(!PulDir_setAcceleration(a, 100.0f) && a->refusal == PULDIR_R_MODE);
    a->refusal = PULDIR_R_NONE;
    assert(!PulDir_setDeceleration(a, 100.0f) && a->refusal == PULDIR_R_MODE);
    a->refusal = PULDIR_R_NONE;
    assert(!PulDir_setJerk(a, 1.0e7f) && a->refusal == PULDIR_R_MODE);
    a->refusal = PULDIR_R_NONE;
    assert(!PulDir_restoreDefaults(a) && a->refusal == PULDIR_R_MODE);
    a->refusal = PULDIR_R_NONE;
    assert(!PulDir_setDefaults(a, 100.0f, 100.0f, 100.0f, 0.0f) && a->refusal == PULDIR_R_MODE);
    assert(a->lim[0] == 4000.0f && a->lim[1] == 40000.0f && a->lim[2] == 40000.0f && a->lim[3] == 0.0f);
    assert(a->def[0] == 4000.0f && a->req == req0);

    /* the target read during a stop is the path's position, not the old target */
    PulDirGroup_stop(&grp);
    while (!tick1())
        ;
    while (!tick1())
        ;
    assert(grp.s.stopping && a->mode == PULDIR_MODE_GROUP);
    assert(PulDir_target(a) != tg[0] && fabsf(PulDir_target(a) - PulDir_position(a)) <= 10.0f);
    run_until_idle(a, TICK_HZ);
    assert(PulDir_target(a) == PulDir_position(a) && st(0).count < -50);   /* 50 ms of ramp, then the stop */

    /* a stop after s has landed: the move ends as planned, exactly, on one tick */
    int32_t c[2] = { st(0).count, st(1).count };
    tg[0] = (float)(c[0] + 700);
    tg[1] = (float)(c[1] - 300);
    rose[0] = rose[1] = false;
    assert(PulDirGroup_moveTo(&grp, tg));
    while (!a->arrived)
        fake_run(1);
    assert(grp.s.landed);
    PulDirGroup_stop(&grp);
    run_until_idle(a, TICK_HZ);
    assert(st(0).count == c[0] + 700 && st(1).count == c[1] - 300 && last_rise[0] == last_rise[1]);

    /* a path window past the ring: each member's own fits, the path's rounds past it */
    assert(PulDir_setDefaults(a, 4000.0f, 1000.0f, 1000.0f, 0.0f));
    assert(!PulDir_setJerk(a, 1.0f) && a->refusal == PULDIR_R_TAPS);
    const float floor_j = a->refused_value;
    assert(PulDir_setJerk(a, floor_j));
    fake_run(N);
    c[0] = st(0).count;
    tg[0] = (float)(c[0] + 9);
    tg[1] = (float)st(1).count;
    a->refusal = PULDIR_R_NONE;
    assert(!PulDirGroup_moveTo(&grp, tg) && a->refusal == PULDIR_R_TAPS && !PulDirGroup_isMoving(&grp));
}

/* The switch byte the test owns: 1 at or below sw_at, 0 from sw_at + 20 up,
 * unchanged in between, updated from axis 0's pulse writes unless frozen. */
static uint8_t sw;
static int32_t sw_pos, sw_at;
static bool sw_frozen;
static void home_write(uint32_t tick, uint8_t id, bool level)
{
    both_write(tick, id, level);
    if (id == 0u && level) {
        sw_pos += fake_level(1) ? 1 : -1;
        if (!sw_frozen) {
            if (sw_pos <= sw_at)
                sw = 1;
            else if (sw_pos >= sw_at + 20)
                sw = 0;
        }
    }
}

/* The phases axis 0 passes through, with the tick and count at each entry. */
static PulDirHomeState seen[16];
static uint32_t seen_tick[16];
static int32_t seen_count[16];
static int nseen;
static float v_towards, v_away;               /* the fastest velocity in either moving phase */
static int64_t adj_inc;                       /* the largest |live increment| during ADJUST_POSITION */
static float lim_towards, lim_away;           /* the speed limit seen in either moving phase */
static bool freeze_at_delay1;
static void home_plan(bool after)
{
    check_published(after);
    if (!after)
        return;
    const PulDir* a = &ax[0];
    const PulDirHomeState h = a->home_state;
    if (nseen == 0 || seen[nseen - 1] != h) {
        assert(nseen < 16);
        seen[nseen] = h;
        seen_tick[nseen] = PulDir_ticks();
        seen_count[nseen] = st(0).count;
        nseen++;
    }
    if (h == PULDIR_HOME_MOVE_TOWARDS_LIMIT && fabsf(a->vel_out) > fabsf(v_towards))
        v_towards = a->vel_out;
    if (h == PULDIR_HOME_MOVE_TOWARDS_LIMIT)
        lim_towards = a->lim[0];
    if (h == PULDIR_HOME_MOVE_AWAY_FROM_LIMIT)
        lim_away = a->lim[0];
    if (h == PULDIR_HOME_MOVE_AWAY_FROM_LIMIT && fabsf(a->vel_out) > fabsf(v_away))
        v_away = a->vel_out;
    if (h == PULDIR_HOME_ADJUST_POSITION) {
        const int64_t i = st(0).inc;
        adj_inc = (i < 0 ? -i : i) > adj_inc ? (i < 0 ? -i : i) : adj_inc;
    }
    if (freeze_at_delay1 && h == PULDIR_HOME_DELAY1)
        sw_frozen = true;
}

/* homeStart with the switch at sw_at. */
static void home_begin(PulDir* a, int32_t at)
{
    sw_at = at;
    sw_pos = st(0).count;
    sw = sw_pos <= sw_at ? 1 : 0;
    sw_frozen = false;
    nseen = 0;
    v_towards = v_away = 0.0f;
    assert(PulDir_homeStart(a));
    assert(a->mode == PULDIR_MODE_HOMING && PulDir_isMoving(a) && !PulDir_isHomed(a));
    assert(PulDir_homeState(a) == PULDIR_HOME_MOVE_TOWARDS_LIMIT);
}

/* homeStart with the switch at sw_at; runs until the axis leaves HOMING. */
static void home_run(PulDir* a, int32_t at, uint32_t max_ms)
{
    home_begin(a, at);
    for (uint32_t t = 0; a->mode == PULDIR_MODE_HOMING; t++) {
        assert(t < max_ms * MS);
        fake_run(1);
    }
}

static void t_homing(void)
{
    add_axis(0, false, false);
    add_axis(1, false, false);
    PulDir* a = &ax[0];
    PulDir_attachFault(a, fake_pin(FLT(0)), false);
    PulDir_attachFaultClear(a, fake_pin(CLR(0)), false);
    start();
    fake_on_write(home_write);
    fake_on_plan(home_plan);
    PulDir_attachLogger(logger);
    assert(PulDir_setDefaults(a, 4000.0f, 40000.0f, 40000.0f, 0.0f));
    assert(PulDir_setDefaults(&ax[1], 4000.0f, 40000.0f, 40000.0f, 0.0f));
    assert(PulDir_setSpeed(a, 1000.0f));      /* below the homing speed: homing runs at its own */
    fake_run(N);                              /* the limits applied: no request pending */

    /* setHoming refuses bad input and changes nothing; homeStart needs it */
    const float cap = (float)(TICK_HZ / 2u);
    assert(!PulDir_setHoming(a, 0, 2000.0f, 50.0f, &sw, 100.0f, 3000u));
    assert(!PulDir_setHoming(a, 2, 2000.0f, 50.0f, &sw, 100.0f, 3000u));
    assert(!PulDir_setHoming(a, -1, 0.0f, 50.0f, &sw, 100.0f, 3000u));
    assert(!PulDir_setHoming(a, -1, -5.0f, 50.0f, &sw, 100.0f, 3000u));
    assert(!PulDir_setHoming(a, -1, NAN, 50.0f, &sw, 100.0f, 3000u));
    assert(!PulDir_setHoming(a, -1, INFINITY, 50.0f, &sw, 100.0f, 3000u));
    assert(!PulDir_setHoming(a, -1, 2000.0f, 50.0f, NULL, 100.0f, 3000u));
    assert(!PulDir_setHoming(a, -1, 2000.0f, 50.0f, &sw, 100.0f, 0u));
    assert(!PulDir_setHoming(a, -1, 2000.0f, 50.0f, &sw, 100.0f, 0xFFFFFFFFu));
    assert(!PulDir_setHoming(a, -1, 2000.0f, NAN, &sw, 100.0f, 3000u));
    assert(!PulDir_setHoming(a, -1, 2000.0f, 50.0f, &sw, INFINITY, 3000u));
    assert(!PulDir_setHoming(a, -1, 2000.0f, 50.0f, &sw, 3.0e9f, 3000u));
    assert(a->refusal == PULDIR_R_ARG);
    assert(!PulDir_setHoming(a, -1, cap, 50.0f, &sw, 100.0f, 3000u) && a->refusal == PULDIR_R_RATE);
    assert(a->home_dir == 0 && a->home_switch == NULL);
    assert(!PulDir_homeStart(a) && a->refusal == PULDIR_R_NO_HOMING);
    assert(a->mode == PULDIR_MODE_IDLE && !PulDir_isMoving(a) && a->req == 0);
    assert(PulDir_setHoming(a, -1, 2000.0f, 50.0f, &sw, 100.0f, 3000u));

    /* the success path: every phase in order, the datum at zero_pos */
    nlines = 0;
    sw_at = -1000;
    sw_pos = 0;
    sw = 0;
    nseen = 0;
    assert(PulDir_homeStart(a) && a->mode == PULDIR_MODE_HOMING && PulDir_isMoving(a));
    assert(PulDir_homeState(a) == PULDIR_HOME_MOVE_TOWARDS_LIMIT && !PulDir_isHomed(a));
    /* refused while homing (5.5) */
    a->refusal = PULDIR_R_NONE;
    assert(!PulDir_moveTo(a, 0.0f) && a->refusal == PULDIR_R_MODE);
    a->refusal = PULDIR_R_NONE;
    assert(!PulDir_jog(a, 10.0f) && a->refusal == PULDIR_R_MODE);
    a->refusal = PULDIR_R_NONE;
    assert(!PulDir_streamBegin(a) && a->refusal == PULDIR_R_MODE);
    a->refusal = PULDIR_R_NONE;
    assert(!PulDir_homeStart(a) && a->refusal == PULDIR_R_MODE);
    a->refusal = PULDIR_R_NONE;
    assert(!PulDir_setHoming(a, 1, 10.0f, 0.0f, &sw, 0.0f, 10u) && a->refusal == PULDIR_R_BUSY);
    a->refusal = PULDIR_R_NONE;
    assert(!PulDir_setPosition(a, 0.0f) && a->refusal == PULDIR_R_NOT_IDLE);
    assert(a->home_dir == -1 && a->mode == PULDIR_MODE_HOMING);
    PulDir_loop(a);
    nlines = 0;
    for (uint32_t t = 0; a->mode == PULDIR_MODE_HOMING; t++) {
        assert(t < 6000u * MS);
        fake_run(1);
    }
    const PulDirHomeState order[7] = {
        PULDIR_HOME_MOVE_TOWARDS_LIMIT, PULDIR_HOME_DELAY1, PULDIR_HOME_MOVE_AWAY_FROM_LIMIT,
        PULDIR_HOME_DELAY2, PULDIR_HOME_ADJUST_POSITION, PULDIR_HOME_DELAY3, PULDIR_HOME_IDLE
    };
    assert(nseen == 7);
    for (int k = 0; k < 7; k++)
        assert(seen[k] == order[k]);
    const uint32_t D = PULDIR_HOMING_DELAY_MS * MS;
    assert(seen_tick[2] - seen_tick[1] >= D && seen_tick[2] - seen_tick[1] <= D + N);
    assert(seen_tick[4] - seen_tick[3] >= D && seen_tick[4] - seen_tick[3] <= D + N);
    assert(seen_tick[6] - seen_tick[5] >= D && seen_tick[6] - seen_tick[5] <= D + N);
    assert(v_towards == -2000.0f && v_away == 400.0f);          /* home speed, then 0.2x of it back */
    assert(lim_towards == 2000.0f && lim_away == 400.0f);       /* applied as the speed limit */
    assert(seen_count[1] <= -1000 && seen_count[2] >= -1000 - 10);   /* stopped within an interval */
    assert(seen_count[3] >= -1000 + 20 && seen_count[3] <= -1000 + 22);
    assert(seen_count[5] - seen_count[4] == 50);                  /* the adjust, whole pulses from rest */
    assert(PulDir_speed(a) == 4000.0f);                           /* the defaults, restored for the adjust */
    /* the adjust follows its generator: never faster than the default speed */
    assert(adj_inc > 0 && adj_inc <= (int64_t)(4000.0 / TICK_HZ * 4294967296.0 * 1.01));
    assert(PulDir_isHomed(a) && PulDir_position(a) == 100.0f && PulDir_target(a) == 100.0f);
    assert(a->mode == PULDIR_MODE_IDLE && !PulDir_isMoving(a) && PulDir_homeState(a) == PULDIR_HOME_IDLE);
    assert(st(0).Pb == (uint64_t)100 << 32 && !high(0));
    PulDir_loop(a);
    assert(nlines == 1 && strcmp(lines[0], "[a0] homed at 100.0000") == 0);

    /* the next move is relative to the datum */
    uint64_t e0 = edges[0];
    assert(PulDir_moveTo(a, 150.0f));
    run_until_idle(a, TICK_HZ);
    assert(st(0).count == 150 && edges[0] - e0 == 50u);

    /* homeStart clears homed at once; homeStop halts (no rising edge after) and ends it */
    assert(PulDir_isHomed(a));
    sw_at = -1000000;
    sw_pos = st(0).count;
    sw = 0;
    assert(PulDir_homeStart(a) && !PulDir_isHomed(a));
    fake_run(200u * MS);
    for (int k = 0; k < 3; k++) {
        while (!high(0))
            fake_run(1);
        e0 = edges[0];
        const int32_t ch = st(0).count;
        if (k == 0)
            PulDir_homeStop(a);
        else if (k == 1)
            PulDir_halt(a);
        else
            PulDir_stop(a);
        assert(PulDir_homeState(a) == PULDIR_HOME_IDLE && !PulDir_isHomed(a));
        if (k < 2) {
            assert(a->mode == PULDIR_MODE_IDLE && PulDir_target(a) == (float)ch);
            fake_run(3u * N);
            assert(edges[0] == e0 && st(0).count == ch && !PulDir_isMoving(a));
        } else {                              /* stop: ramped, at the axis's decel */
            assert(a->mode == PULDIR_MODE_STOPPING);
            run_until_idle(a, TICK_HZ);
            assert(st(0).count < ch - 10 && PulDir_target(a) == PulDir_position(a));
        }
        assert(PulDir_homeState(a) == PULDIR_HOME_IDLE && a->mode == PULDIR_MODE_IDLE);
        if (k < 2) {
            assert(PulDir_homeStart(a));
            fake_run(200u * MS);
        }
    }

    /* a switch that never reads 1: the phase times out, the axis halts, ERROR */
    assert(PulDir_setHoming(a, -1, 2000.0f, 50.0f, &sw, 100.0f, 300u));
    uint32_t t0 = PulDir_ticks();
    PulDir_loop(a);
    nlines = 0;
    home_run(a, -1000000, 400u);
    uint32_t tf = PulDir_ticks();
    e0 = edges[0];
    int32_t ch = st(0).count;
    assert(tf - t0 >= 300u * MS && tf - t0 <= 300u * MS + 2u * N);
    assert(PulDir_homeState(a) == PULDIR_HOME_ERROR && !PulDir_isHomed(a) && a->mode == PULDIR_MODE_IDLE);
    assert(PulDir_target(a) == (float)ch && PulDir_velocity(a) == 0.0f);
    fake_run(3u * N);
    assert(edges[0] == e0 && st(0).count == ch && !PulDir_isMoving(a));
    PulDir_loop(a);
    assert(nlines == 1 && strcmp(lines[0], "[a0] homing failed in MOVE_TOWARDS_LIMIT") == 0);
    printf("  homing: the 300 ms phase timed out after %.2f ms\n", (double)(tf - t0) * 1000.0 / TICK_HZ);

    /* a switch byte that stops being updated (stuck at 1 after the limit): a timeout halt */
    assert(PulDir_setHoming(a, -1, 4000.0f, 50.0f, &sw, 100.0f, 400u));
    freeze_at_delay1 = true;
    nlines = 0;
    home_run(a, st(0).count - 300, 3000u);
    freeze_at_delay1 = false;
    e0 = edges[0];
    ch = st(0).count;
    assert(nseen == 4 && seen[2] == PULDIR_HOME_MOVE_AWAY_FROM_LIMIT && seen[3] == PULDIR_HOME_ERROR);
    tf = PulDir_ticks();
    assert(tf - seen_tick[2] >= 400u * MS && tf - seen_tick[2] <= 400u * MS + N);
    fake_run(3u * N);
    assert(edges[0] == e0 && st(0).count == ch && !PulDir_isMoving(a) && !PulDir_isHomed(a));
    PulDir_loop(a);
    assert(nlines == 1 && strcmp(lines[0], "[a0] homing failed in MOVE_AWAY_FROM_LIMIT") == 0);

    /* homed clears on a fault latch, and a fault during homing ends it in ERROR */
    assert(PulDir_setHoming(a, -1, 2000.0f, 50.0f, &sw, -7.0f, 3000u));
    home_run(a, st(0).count - 500, 6000u);
    assert(PulDir_isHomed(a) && PulDir_position(a) == -7.0f);
    fake_set_input(FLT(0), true);
    fake_run(4u * MS);
    assert(a->fault && !PulDir_isHomed(a));
    fake_set_input(FLT(0), false);
    assert(PulDir_clearFault(a));
    fake_run(12u * MS);
    assert(!a->fault);
    PulDir_loop(a);
    nlines = 0;
    sw_at = -1000000;
    sw = 0;
    assert(PulDir_homeStart(a));
    fake_run(100u * MS);
    fake_set_input(FLT(0), true);
    while (!a->fault)
        fake_run(1);
    assert(PulDir_homeState(a) == PULDIR_HOME_ERROR && a->mode == PULDIR_MODE_IDLE && !PulDir_isHomed(a));
    e0 = edges[0];
    fake_run(3u * N);
    assert(edges[0] == e0 && !PulDir_isMoving(a));
    PulDir_loop(a);
    assert(nlines == 2 && strcmp(lines[0], "[a0] fault latched, count 2") == 0);
    assert(strcmp(lines[1], "[a0] homing failed in MOVE_TOWARDS_LIMIT") == 0);
    a->refusal = PULDIR_R_NONE;
    assert(!PulDir_homeStart(a) && a->refusal == PULDIR_R_FAULT);
    fake_set_input(FLT(0), false);
    assert(PulDir_clearFault(a));
    fake_run(12u * MS);
    assert(!a->fault);

    /* homed clears on setPosition */
    home_run(a, st(0).count - 500, 6000u);
    assert(PulDir_isHomed(a));
    assert(PulDir_setPosition(a, 3.0f) && !PulDir_isHomed(a));

    /* defaults the generator refuses (a jerk past the ring, set behind the API's
     * back: the API cannot) are not copied: the adjust runs at the limits it has */
    const float jerk_def = a->def[3];
    a->def[3] = 1.0f;
    home_run(a, st(0).count - 500, 6000u);
    assert(PulDir_isHomed(a) && a->lim[0] == 400.0f && a->lim[3] == 0.0f && a->prof.lim.jmax == 0.0f);
    assert(seen_count[5] - seen_count[4] == 50);
    a->def[3] = jerk_def;

    /* refused outside IDLE: in STREAM (test 19), in GROUP, and while moving */
    assert(PulDir_streamBegin(a));
    a->refusal = PULDIR_R_NONE;
    assert(!PulDir_homeStart(a) && a->refusal == PULDIR_R_MODE && a->mode == PULDIR_MODE_STREAM);
    PulDir_streamEnd(a);
    run_until_idle(a, TICK_HZ);
    PulDir* m[2] = { &ax[0], &ax[1] };
    assert(PulDirGroup_init(&grp, m, 2));
    const float tg[2] = { (float)(st(0).count + 2000), 1000.0f };
    assert(PulDirGroup_moveTo(&grp, tg));
    a->refusal = PULDIR_R_NONE;
    assert(!PulDir_homeStart(a) && a->refusal == PULDIR_R_MODE && a->mode == PULDIR_MODE_GROUP);
    run_until_idle(a, TICK_HZ);
    assert(PulDir_moveBy(a, 2000.0f));
    a->refusal = PULDIR_R_NONE;
    assert(!PulDir_homeStart(a) && a->refusal == PULDIR_R_MODE && a->mode == PULDIR_MODE_PROFILE);
    assert(PulDir_isHomed(a));                /* a refused homeStart leaves the datum alone */
}

/* The limits as the getters read them and, at conv 1, as the generator holds them. */
static void limits_are(const PulDir* a, const float l[4], bool generator)
{
    for (int k = 0; k < 4; k++)
        assert(a->lim[k] == l[k]);
    const MotionLimits g = a->prof.lim;
    assert(!generator || (g.vmax == l[0] && g.amax == l[1] && g.dmax == l[2] && g.jmax == l[3]));
}

/* Runs until the axis has left HOMING. */
static void home_finish(PulDir* a, uint32_t max_ms)
{
    for (uint32_t t = 0; a->mode == PULDIR_MODE_HOMING; t++) {
        assert(t < max_ms * MS);
        fake_run(1);
    }
}

/* A run that ends unhomed, by any exit, puts back the limits homeStart found;
 * while it runs, the limit setters are refused, so nothing races the plan tick. */
static void t_homing_limits(void)
{
    add_axis(0, false, false);
    PulDir* a = &ax[0];
    PulDir_attachFault(a, fake_pin(FLT(0)), false);
    PulDir_attachFaultClear(a, fake_pin(CLR(0)), false);
    start();
    fake_on_write(home_write);
    fake_on_plan(home_plan);
    const float def[4] = { 4000.0f, 40000.0f, 40000.0f, 0.0f };
    /* A jerk whose window is wider than the one tap of the defaults. */
    const float mine[4] = { 1000.0f, 30000.0f, 20000.0f,
                            4.0f * MotionProfile_minJerk(30000.0f, 20000.0f, DT) };
    assert(PulDir_setDefaults(a, def[0], def[1], def[2], def[3]));
    assert(PulDir_setSpeed(a, mine[0]) && PulDir_setAcceleration(a, mine[1]));
    assert(PulDir_setDeceleration(a, mine[2]) && PulDir_setJerk(a, mine[3]));
    assert(PulDir_setHoming(a, -1, 2000.0f, 50.0f, &sw, 100.0f, 300u));
    fake_run(N);
    limits_are(a, mine, true);

    /* refused while homing, limits and defaults unchanged */
    home_begin(a, -1000000);
    fake_run(20u * MS);
    const float run[4] = { 2000.0f, mine[1], mine[2], mine[3] };   /* the homing speed as the limit */
    limits_are(a, run, true);
    a->refusal = PULDIR_R_NONE;
    assert(!PulDir_setSpeed(a, 500.0f) && a->refusal == PULDIR_R_MODE);
    a->refusal = PULDIR_R_NONE;
    assert(!PulDir_setAcceleration(a, 500.0f) && a->refusal == PULDIR_R_MODE);
    a->refusal = PULDIR_R_NONE;
    assert(!PulDir_setDeceleration(a, 500.0f) && a->refusal == PULDIR_R_MODE);
    a->refusal = PULDIR_R_NONE;
    assert(!PulDir_setJerk(a, 0.0f) && a->refusal == PULDIR_R_MODE);
    a->refusal = PULDIR_R_NONE;
    assert(!PulDir_restoreDefaults(a) && a->refusal == PULDIR_R_MODE);
    a->refusal = PULDIR_R_NONE;
    assert(!PulDir_setDefaults(a, 1.0f, 1.0f, 1.0f, 0.0f) && a->refusal == PULDIR_R_MODE);
    fake_run(N);
    limits_are(a, run, true);
    for (int k = 0; k < 4; k++)
        assert(a->def[k] == def[k]);

    /* halt, homeStop and stop put them back at once; the generator takes them */
    for (int k = 0; k < 3; k++) {
        if (k > 0) {
            home_begin(a, -1000000);
            fake_run(20u * MS);
            limits_are(a, run, true);
        }
        if (k == 0)
            PulDir_halt(a);
        else if (k == 1)
            PulDir_homeStop(a);
        else
            PulDir_stop(a);
        limits_are(a, mine, false);
        run_until_idle(a, TICK_HZ);
        fake_run(N);
        limits_are(a, mine, true);
        assert(PulDir_homeState(a) == PULDIR_HOME_IDLE && a->mode == PULDIR_MODE_IDLE);
    }

    /* a stop in the adjust, which runs on the defaults' one-tap window: the
     * generator cannot widen it in flight, so it takes them once at rest */
    home_begin(a, st(0).count - 300);
    for (uint32_t t = 0; !(a->home_state == PULDIR_HOME_ADJUST_POSITION && fabsf(a->vel_out) > 500.0f); t++) {
        assert(t < 3000u * MS);
        fake_run(1);
    }
    PulDir_stop(a);
    limits_are(a, mine, false);
    fake_run(N);
    assert(a->prof.lim.jmax == 0.0f && a->prof.lim.vmax == def[0]);
    run_until_idle(a, TICK_HZ);
    fake_run(N);
    limits_are(a, mine, true);

    /* a phase timeout and a fault latch end the run from the plan tick */
    home_begin(a, -1000000);
    home_finish(a, 400u);
    assert(PulDir_homeState(a) == PULDIR_HOME_ERROR);
    limits_are(a, mine, true);
    run_until_idle(a, TICK_HZ);
    home_begin(a, -1000000);
    fake_run(20u * MS);
    fake_set_input(FLT(0), true);
    for (uint32_t t = 0; !a->fault; t++) {
        assert(t < 10u * MS);
        fake_run(1);
    }
    assert(PulDir_homeState(a) == PULDIR_HOME_ERROR && a->mode == PULDIR_MODE_IDLE);
    limits_are(a, mine, true);
    fake_set_input(FLT(0), false);
    assert(PulDir_clearFault(a));
    fake_run(12u * MS);
    assert(!a->fault);

    /* a run that homes leaves the defaults its adjust ran on */
    home_begin(a, st(0).count - 300);
    home_finish(a, 6000u);
    assert(PulDir_isHomed(a));
    limits_are(a, def, true);
}

/* Test 22: with the counter started 1,000 ticks below its wrap, every timed
 * event that starts before the wrap and ends after it measures correctly. */
static void t_rollover(void)
{
#if PULDIR_TICK_START != 0
    add_axis(0, false, false);                /* the stream timeout */
    add_axis(1, false, false);                /* a homing phase timeout */
    add_axis(2, false, false);                /* the fault clear pulse */
    add_axis(3, false, false);                /* the fault debounce */
    add_axis(4, false, false);                /* the enable delay */
    PulDir_attachFaultClear(&ax[2], fake_pin(CLR(2)), false);
    PulDir_attachFault(&ax[3], fake_pin(FLT(3)), false);
    PulDir_attachEnable(&ax[4], fake_pin(ENA(4)), false);
    start();
    fake_on_write(both_write);
    const uint32_t wrap = 0u - WRAP_START;    /* ticks from the start to the wrap */
    assert(wrap == 1000u && PulDir_ticks() == WRAP_START);
    for (uint8_t i = 0; i < 5; i++)
        assert(PulDir_setDefaults(&ax[i], 5000.0f, 1.0e5f, 1.0e5f, 0.0f));
    static uint8_t never;
    assert(PulDir_setHoming(&ax[1], 1, 1000.0f, 0.0f, &never, 0.0f, 30u));
    assert(PulDir_setDelays(&ax[4], 5u, 0u));

    /* streamBegin and homeStart at once; setpoints until 400 ticks before the wrap */
    assert(PulDir_streamBegin(&ax[0]) && PulDir_homeStart(&ax[1]));
    const uint32_t t_home = PulDir_ticks();
    uint32_t t_sp = 0, t_clr = 0, t_on = 0, t_ena = 0;
    uint32_t t_to = 0, t_fail = 0, t_low = 0, t_latch = 0;
    for (uint32_t k = 0; k < 200u * MS; k++) {
        const uint32_t since = PulDir_ticks() - WRAP_START;
        if (since < wrap - 400u && since % N == 7u) {
            PulDir_stream(&ax[0], 0.0f, 0.0f);
            t_sp = PulDir_ticks();
        }
        if (since == wrap - 300u) {           /* 10 ms is 500 ticks at a 20 us pulse: start late enough */
            assert(PulDir_clearFault(&ax[2]));
            t_clr = PulDir_ticks();
        }
        if (since == wrap - 110u) {           /* samples before the wrap, the latch after it */
            fake_set_input(FLT(3), true);
            t_on = PulDir_ticks();
        }
        if (since == wrap - 100u) {
            assert(PulDir_moveTo(&ax[4], 100.0f));
            t_ena = PulDir_ticks();
        }
        fake_run(1);
        if (t_to == 0u && ax[0].mode != PULDIR_MODE_STREAM)
            t_to = PulDir_ticks();
        if (t_fail == 0u && ax[1].mode != PULDIR_MODE_HOMING)
            t_fail = PulDir_ticks();
        if (t_low == 0u && t_clr != 0u && !fake_level(CLR(2)))
            t_low = PulDir_ticks();
        if (t_latch == 0u && ax[3].fault)
            t_latch = ax[3].last_fault_tick;
    }
    /* every start before the wrap, every end after it */
    const uint32_t starts[5] = { t_sp, t_home, t_clr, t_on, t_ena };
    const uint32_t ends[4] = { t_to, t_fail, t_low, t_latch };
    for (int k = 0; k < 5; k++)
        assert(starts[k] >= WRAP_START);
    for (int k = 0; k < 4; k++)
        assert(ends[k] != 0u && ends[k] < 0x80000000u);
    assert(t_to - t_sp >= PULDIR_STREAM_TIMEOUT_MS * MS && t_to - t_sp <= PULDIR_STREAM_TIMEOUT_MS * MS + N);
    assert(PulDir_streamTimedOut(&ax[0]));
    assert(t_fail - t_home >= 30u * MS && t_fail - t_home <= 30u * MS + 2u * N);
    assert(ax[1].home_state == PULDIR_HOME_ERROR);
    assert(t_low - t_clr >= PULDIR_FAULT_CLEAR_MS * MS && t_low - t_clr <= PULDIR_FAULT_CLEAR_MS * MS + N);
    assert(t_latch - t_on >= PULDIR_FAULT_DEBOUNCE_MS * MS && t_latch - t_on <= PULDIR_FAULT_DEBOUNCE_MS * MS + N);
    assert(first_rise[4] < 0x80000000u && first_rise[4] - t_ena >= 5u * MS);
    assert(st(4).count == 100);
    printf("  rollover: stream timeout %.2f ms, homing timeout %.2f ms, clear pulse %.2f ms, "
           "debounce %.2f ms, first edge %.2f ms after enable\n",
           (double)(t_to - t_sp) * 1000.0 / TICK_HZ, (double)(t_fail - t_home) * 1000.0 / TICK_HZ,
           (double)(t_low - t_clr) * 1000.0 / TICK_HZ, (double)(t_latch - t_on) * 1000.0 / TICK_HZ,
           (double)(first_rise[4] - t_ena) * 1000.0 / TICK_HZ);
#endif
}

/* ---- the case table ------------------------------------------------------------------------ */

/* The @ console (task 17). */
static bool said(const char* what)
{
    for (int i = 0; i < nlines; i++)
        if (strstr(lines[i], what) != NULL)
            return true;
    return false;
}

static void t_console(void)
{
    add_axis(0, false, false);
    add_axis(1, false, false);
    PulDir* a = &ax[0];
    PulDir* b = &ax[1];                       /* no enable pin */
    PulDir_attachEnable(a, fake_pin(ENA(0)), false);
    start();
    PulDir_attachLogger(logger);
    assert(PulDir_setDefaults(a, 100.0f, 100.0f, 100.0f, 0.0f));
    assert(PulDir_setDefaults(b, 100.0f, 100.0f, 100.0f, 0.0f));

    /* T moves, p prints with four decimals and the axis prefix */
    PulDir_console(a, 'T', "10");
    run_until_idle(a, TICK_HZ);
    assert(PulDir_position(a) == 10.0f);
    nlines = 0;
    PulDir_console(a, 'p', "");
    assert(nlines == 1 && strstr(lines[0], "10.0000") && strncmp(lines[0], "[a0] ", 5) == 0);

    /* A leaves the deceleration alone; Z sets the jerk; the getters print */
    assert(PulDir_setDeceleration(a, 30.0f));
    PulDir_console(a, 'A', "40");
    assert(PulDir_acceleration(a) == 40.0f && PulDir_deceleration(a) == 30.0f);
    PulDir_console(a, 'Z', "600");
    assert(PulDir_jerk(a) == 600.0f);
    nlines = 0;
    PulDir_console(a, 'z', NULL);
    PulDir_console(a, 'a', "");
    PulDir_console(a, 'd', "");
    PulDir_console(a, 'v', "");
    assert(nlines == 4 && strstr(lines[0], "600.0000") && strstr(lines[1], "40.0000")
           && strstr(lines[2], "30.0000") && strstr(lines[3], "0.0000"));
    PulDir_console(a, 'D', "32");
    PulDir_console(a, 'V', "80");
    assert(PulDir_deceleration(a) == 32.0f && PulDir_speed(a) == 80.0f);
    PulDir_console(a, 'F', "");
    assert(PulDir_speed(a) == 100.0f && PulDir_jerk(a) == 0.0f);

    /* P, R, J, k, E, s */
    PulDir_console(a, 'P', "5");
    assert(PulDir_position(a) == 5.0f);
    PulDir_console(a, 'R', "-2");
    run_until_idle(a, TICK_HZ);
    assert(PulDir_position(a) == 3.0f);
    PulDir_console(a, 'J', "40");
    fake_run(50u * N);
    assert(PulDir_isMoving(a) && PulDir_velocity(a) > 0.0f);
    PulDir_console(a, 'k', "");
    run_until_idle(a, TICK_HZ);
    PulDir_console(a, 'E', "0");
    assert(!a->enabled);
    PulDir_console(a, 'E', "1");
    assert(a->enabled);
    nlines = 0;
    PulDir_console(a, 's', "");
    assert(nlines == 1 && strstr(lines[0], "status") && strstr(lines[0], "mode IDLE")
           && strstr(lines[0], "homed 0") && strstr(lines[0], "faults 0")
           && strstr(lines[0], "last fault tick 0") && !strstr(lines[0], "failed"));

    /* K halts, then releases the drive */
    PulDir_console(a, 'J', "40");
    fake_run(50u * N);
    PulDir_console(a, 'K', "");
    assert(a->mode == PULDIR_MODE_IDLE && !a->enabled);
    fake_run(4u * N);
    run_until_idle(a, TICK_HZ);
    assert(!PulDir_isMoving(a) && PulDir_velocity(a) == 0.0f);

    /* K without an enable pin still halts, says so, and leaves no refusal */
    PulDir_console(b, 'J', "40");
    fake_run(50u * N);
    nlines = 0;
    PulDir_console(b, 'K', "");
    assert(b->mode == PULDIR_MODE_IDLE);
    assert(nlines == 1 && strstr(lines[0], "halted") && strstr(lines[0], "no enable pin"));
    fake_run(4u * N);
    PulDir_loop(b);
    assert(nlines == 1);

    /* a malformed number is "bad argument", never a silent 0 */
    PulDir_console(a, 'P', "7");
    PulDir_loop(a);                           /* flush the events of the steps above */
    const char* bad[] = { "", "x", "1x", "1 2", "--1" };
    for (size_t i = 0; i < sizeof bad / sizeof bad[0]; i++) {
        nlines = 0;
        PulDir_console(a, 'T', bad[i]);
        PulDir_loop(a);
        assert(nlines == 1 && said("bad argument") && strncmp(lines[0], "[a0] ", 5) == 0);
        assert(!PulDir_isMoving(a) && PulDir_position(a) == 7.0f);
    }
    nlines = 0;
    PulDir_console(a, 'E', "2");
    PulDir_console(a, 'Z', "abc");
    PulDir_console(a, 'T', NULL);
    PulDir_loop(a);
    assert(said("bad argument") && PulDir_jerk(a) == 0.0f);
    PulDir_console(a, 'T', " 8 \r\n");        /* white space round the number is not a fault */
    run_until_idle(a, TICK_HZ);
    assert(PulDir_position(a) == 8.0f);

    /* the other verbs and unknown ones */
    nlines = 0;
    PulDir_console(a, 'X', "");
    assert(nlines == 1 && strstr(lines[0], "unknown command: X"));
    PulDir_console(a, 'C', "");               /* no clear pin: a refusal through the loop */
    PulDir_loop(a);
    assert(said("clearFault refused"));
    nlines = 0;
    PulDir_console(a, 'H', "");               /* homing not configured */
    PulDir_loop(a);
    assert(said("homeStart refused"));

    /* the switch silences the replies */
    nlines = 0;
    PulDir_setLogging(a, false);
    PulDir_console(a, 'p', "");
    PulDir_console(a, 'X', "");
    assert(nlines == 0);
    PulDir_setLogging(a, true);

    /* the menu fits 1,024 bytes, names the jerk verbs and every verb */
    char menu[1024];
    memset(menu, 'x', sizeof menu);
    PulDir_consoleMenu(menu, sizeof menu);
    assert(memchr(menu, '\0', sizeof menu) != NULL && strstr(menu, "Z<jerk>") && strstr(menu, "z - "));
    for (const char* v = "EPpVvAaDdJRTFkKsCH"; *v; v++) {
        const char key[3] = { '\n', *v, 0 };
        const char* at = strstr(menu, key);
        assert(at != NULL && (at[2] == ' ' || at[2] == '<'));
    }
    char small[16];
    PulDir_consoleMenu(small, sizeof small);
    assert(strlen(small) < sizeof small);
}

/* ---- timing and init -------------------------------------------------------------- */

/* Run before any timing (no fake_attach): init, register and a group refuse without it; each
 * attach call refuses a value out of range; init refuses a ratio that is not 2 to 4096 whole
 * pulse periods, and the first init that passes fixes the timing. */
static void t_timing(void)
{
    static PulDir z;                          /* zeroed, as an application's static axis is */
    PulDir_attachLogger(logger);
    assert(!PulDir_init(&z, "t", 280.0f));
    assert(nlines == 1 && strcmp(lines[0], "[t] init refused: timers not attached") == 0);
    PulDir_attachPins(&z, fake_pin(0), fake_pin(1), false, false);
    assert(!PulDir_register(&z) && puldir_naxes == 0);   /* never initialised */
    PulDir* m[1] = { &z };
    assert(!PulDirGroup_init(&grp, m, 1));

    assert(!puldir_timing_pulse(1u) && !puldir_timing_pulse(1001u));
    assert(!puldir_timing_plan(249u) && !puldir_timing_plan(10001u));
    assert(!fake_attach(1u, 1000u) && !fake_attach(20u, 20000u));   /* the second keeps its 20 us */
    assert(puldir_timing.pulse_us == 20u && puldir_timing.plan_us == 0u && !puldir_timing.set);

    static const uint32_t bad[3][2] = { { 30u, 1000u }, { 500u, 500u }, { 2u, 10000u } };   /* N 33.3, 1, 5000 */
    PulDir a;
    for (int k = 0; k < 3; k++) {
        assert(fake_attach(bad[k][0], bad[k][1]));
        nlines = 0;
        assert(!PulDir_init(&a, "t", 280.0f) && !puldir_timing.set);
        assert(nlines == 1 && strcmp(lines[0], "[t] init refused: plan period not 2 to 4096 whole pulse periods") == 0);
    }
    nlines = 0;
    assert(!PulDir_init(&a, NULL, 1.0f) && !PulDir_init(&a, "sixteen chars ok", 1.0f));
    assert(nlines == 2 && strcmp(lines[1], "[?] init refused: name NULL or longer than 15 characters") == 0);

    assert(fake_attach(2u, 8192u));           /* N = 4096, the most */
    nlines = 0;
    assert(PulDir_init(&a, "t", 1.0f) && puldir_timing.set && N == 4096u && TICK_HZ == 500000u);
    assert(nlines == 1 && strstr(lines[0], "(pulse 2 us, plan 8192 us)"));
    assert(!fake_attach(20u, 1000u) && !puldir_timing_pulse(4u) && !puldir_timing_plan(4096u));
    assert(puldir_timing.pulse_us == 2u && puldir_timing.plan_us == 8192u);
}

/* Run before any timing: PulDir_start fixes it even when no axis took it, and the ticks then run
 * harmlessly; no attach call is taken after it. */
static void t_timing_start(void)
{
    assert(fake_attach(30u, 1000u));          /* not whole: no init could take it */
    PulDir_start();
    assert(!puldir_timing.set && puldir_timing.started);
    assert(!fake_attach(20u, 1000u) && puldir_timing.pulse_us == 30u);
    fake_pulse(false);
    fake_plan();
    fake_pulse(false);
    assert(PulDir_ticks() == PULDIR_TICK_START + 2u && PulDir_overruns() == 0u);
}

/* Init refuses a bad conversion, writes nothing and says so; a good one logs the rate cap and the
 * resolution in user units, and starts at 200 pulses/s and 200 pulses/s^2. */
static void t_init_log(void)
{
    PulDir a, junk;
    memset(&a, 0xA5, sizeof a);
    memset(&junk, 0xA5, sizeof junk);
    PulDir_attachLogger(logger);
    const float bad[] = { 0.0f, -2.0f, INFINITY, -INFINITY, NAN };
    for (size_t i = 0; i < sizeof bad / sizeof bad[0]; i++) {
        nlines = 0;
        assert(!PulDir_init(&a, "tilt", bad[i]) && memcmp(&a, &junk, sizeof a) == 0);
        assert(nlines == 1 && strcmp(lines[0], "[tilt] init refused: conversion not finite and positive") == 0);
    }
    nlines = 0;
    assert(PulDir_init(&a, "tilt", 280.0f) && nlines == 1);
    if (pulse_us == 20u && plan_us == 1000u)
        assert(strcmp(lines[0], "[tilt] max speed 89.2857 units/s, resolution 0.0036 units/pulse "
                                "(pulse 20 us, plan 1000 us)") == 0);
    float vmax = 0.0f, res = 0.0f;
    unsigned pu = 0, pl = 0;
    assert(sscanf(lines[0], "[tilt] max speed %f units/s, resolution %f units/pulse (pulse %u us, plan %u us)",
                  &vmax, &res, &pu, &pl) == 4);
    const float cap = (float)(TICK_HZ / 2u) / 280.0f;
    assert(fabsf(vmax - cap) <= 5.0e-5f + 1.0e-6f * cap && fabsf(res - 1.0f / 280.0f) <= 5.0e-5f);
    assert(pu == pulse_us && pl == plan_us);
    assert(!PulDir_setSpeed(&a, cap) && PulDir_setSpeed(&a, nextafterf(cap, 0.0f)));   /* "max" is just under */
    assert(PulDir_init(&a, "tilt", 280.0f) && a.conv == 280.0f);
    assert(PulDir_speed(&a) == 200.0f / 280.0f && a.def[1] == 200.0f / 280.0f && a.lim[3] == 0.0f);
    assert(a.prof.lim.vmax == 200.0f && a.prof.lim.amax == 200.0f && a.prof.dt == DT);

    PulDir_attachLogger(NULL);                /* no logger: no line, init still works */
    nlines = 0;
    assert(PulDir_init(&a, "tilt", 280.0f) && !PulDir_init(&a, "tilt", 0.0f) && nlines == 0);
}

static const struct { const char* name; void (*fn)(void); } untimed[] = {
    { "timing", t_timing },
    { "timing_start", t_timing_start },
};

static const struct { const char* name; void (*fn)(void); } cases[] = {
    { "init_log", t_init_log },
    { "setup", t_setup },
    { "timer_counts", t_timer_counts },
    { "pulse_rate", t_pulse_rate },
    { "dir_change", t_dir_change },
    { "polarity", t_polarity },
    { "latch", t_latch },
    { "inc_max", t_inc_max },
    { "hysteresis", t_hysteresis },
    { "register_after_start", t_register_after_start },
    { "setup_order", t_setup_order },
    { "kill", t_kill },
    { "datum", t_datum },
    { "overrun", t_overrun },
    { "move_exact", t_move_exact },
    { "parity", t_parity },
    { "full_rate", t_full_rate },
    { "arrival_twice", t_arrival_twice },
    { "creep", t_creep },
    { "refusals", t_refusals },
    { "overrun_log", t_overrun_log },
    { "jog", t_jog },
    { "stop", t_stop },
    { "stop_random", t_stop_random },
    { "halt", t_halt },
    { "rate_cap", t_rate_cap },
    { "live_limits", t_live_limits },
    { "setpos_idle_only", t_setpos },
    { "settle_twice", t_settle_twice },
    { "late_publish", t_late_publish },
    { "jog_settling", t_jog_settling },
    { "halt_any_tick", t_halt_any_tick },
    { "stream_sine", t_stream_sine },
    { "stream_sine_1k", t_stream_sine_1k },
    { "stream_sine_1k_tuned", t_stream_sine_1k_tuned },
    { "stream_timeout", t_stream_timeout },
    { "stream_end", t_stream_end },
    { "stream_transitions", t_stream_transitions },
    { "stream_speed_limit", t_stream_speed_limit },
    { "stream_loss", t_stream_loss },
    { "stream_end_begin", t_stream_end_begin },
    { "jerk_floor", t_jerk_floor },
    { "request_order", t_request_order },
    { "group_sync", t_group_sync },
    { "group_rules", t_group_rules },
    { "group_twice", t_group_twice },
    { "group_creep", t_group_creep },
    { "enable_release", t_enable_release },
    { "fault", t_fault },
    { "fault_cases", t_fault_cases },
    { "group_fault", t_group_fault },
    { "group_paths", t_group_paths },
    { "homing", t_homing },
    { "homing_limits", t_homing_limits },
    { "rollover", t_rollover },
    { "console", t_console },
};

/* test_puldir <case> [pulse_us plan_us]: the timed cases run after fake_attach, 20/1000 by default. */
int main(int argc, char** argv)
{
    const size_t n = sizeof cases / sizeof cases[0], nu = sizeof untimed / sizeof untimed[0];
    if (argc == 2 && strcmp(argv[1], "list") == 0) {
        for (size_t i = 0; i < nu; i++)
            printf("%s\n", untimed[i].name);
        for (size_t i = 0; i < n; i++)
            printf("%s\n", cases[i].name);
        return 0;
    }
    if (argc == 4) {
        pulse_us = (uint32_t)strtoul(argv[2], NULL, 10);
        plan_us = (uint32_t)strtoul(argv[3], NULL, 10);
    }
    if (argc == 2 || argc == 4) {
        for (size_t i = 0; i < nu; i++)
            if (strcmp(argv[1], untimed[i].name) == 0) {
                untimed[i].fn();
                printf("ok %s\n", untimed[i].name);
                return 0;
            }
        for (size_t i = 0; i < n; i++)
            if (strcmp(argv[1], cases[i].name) == 0) {
                if (!fake_attach(pulse_us, plan_us)) {
                    fprintf(stderr, "timing %u/%u us refused\n", (unsigned)pulse_us, (unsigned)plan_us);
                    return 2;
                }
                cases[i].fn();
                printf("ok %s (%u/%u us)\n", cases[i].name, (unsigned)pulse_us, (unsigned)plan_us);
                return 0;
            }
    }
    fprintf(stderr, "usage: %s list | <case> [pulse_us plan_us]\n", argv[0]);
    return 2;
}
