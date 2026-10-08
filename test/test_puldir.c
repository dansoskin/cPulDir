/* Axis tests (guidelines section 8, tests 10 to 24). One case per process
 * (D9): `test_puldir list` prints the names, `test_puldir <name>` runs one. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "fake_port.h"
#include "puldir_internal.h"

#define N PULDIR_TICKS_PER_PLAN
/* The timing tests build whole-interval moves of up to 10.25 pulses and run
 * 10,000 ticks as whole intervals: they assume these. */
typedef char check_n[(N >= 32u && 10000u % N == 0u) ? 1 : -1];

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

/* ---- driving increments by hand -------------------------------------------- */

static uint32_t left(void)
{
    port_lock();
    const uint32_t l = puldir_left();
    port_unlock();
    return l;
}

static PulDirPulse st(uint8_t slot)
{
    PulDirPulse s;
    port_lock();
    puldir_pulse_read(slot, &s);
    port_unlock();
    return s;
}

static bool high(uint8_t slot)
{
    port_lock();
    const bool h = puldir_pul_high(slot);
    port_unlock();
    return h;
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

#define MAX_EDGES 60000u
static uint32_t rise[MAX_EDGES], fall[MAX_EDGES];

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
    assert(!a.has_pins && a.mode == PULDIR_MODE_IDLE && a.events == 0);
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
    for (uint8_t i = 0; i < PULDIR_MAX_AXES; i++) {
        const PulDirPulse s = st(i);
        assert(s.P == 0 && s.count == 0 && s.inc == 0 && s.dir == 1 && !high(i));
    }
}

static void t_register_after_start(void)
{
    setup(1);
    PulDir b;
    PulDir_init(&b, "late");
    PulDir_attachPins(&b, fake_pin(10), fake_pin(11), false, false);
    assert(!PulDir_register(&b));
    assert(b.slot == PULDIR_MAX_AXES && puldir_naxes == 1);
    assert((b.events & PULDIR_EV_REFUSED) && b.refusal == PULDIR_R_BUSY);
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
    const uint32_t T = 10000u;            /* whole intervals: check_n */
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
        const uint64_t before = st(0).P;
        fake_pulse(false);
        const uint64_t after = st(0).P;
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
    const int64_t one = (int64_t)PULDIR_ONE;
    /* up to 10.25 in one interval */
    const int32_t inc_up = (int32_t)((41 * one / 4) / N);
    assert(inc_up <= PULDIR_INC_MAX);
    assert(interval(inc_up, 1) == 10);
    assert(st(0).count == 10 && (int64_t)st(0).P > 10 * one);

    /* down 1.5: through 10.0 with no edge, one edge at 9.0. The first tick of a
     * reversal writes the direction and does not move. */
    const int32_t inc_down = (int32_t)((3 * one / 2) / (N - 1));
    assert(interval(-inc_down, -1) == 1);
    assert(st(0).count == 9 && (int64_t)st(0).P < 9 * one && (int64_t)st(0).P > 8 * one);

    /* dither across 9.0 and 9.5, between 8.75 and 9.75, for at least 1,000 ticks: no edges */
    const size_t from = nw;
    const int32_t inc_d = (int32_t)(one / (N - 1));
    const uint32_t cycles = (1000u + 2u * N - 1u) / (2u * N);
    for (uint32_t k = 0; k < cycles; k++) {
        assert(interval(inc_d, 1) == 0);
        assert((int64_t)st(0).P > 9 * one + one / 2);
        assert(interval(-inc_d, -1) == 0);
        assert((int64_t)st(0).P < 9 * one);
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
    const uint64_t P0 = st(0).P;
    const size_t from = nw;
    assert(st(0).inc == 0 && st(1).inc == q);
    to_boundary();
    pulse(2u * N);
    assert(st(0).inc == 0 && st(1).inc == q2);   /* the pending set lost axis 0's inc only */
    assert(st(0).P == P0);
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
    PulDirPulse s = st(0);
    assert(s.count == -5 && s.P == (uint64_t)(int64_t)-5 << 32 && s.dir == 1);
    assert(interval(q, 1) == N / 4u);     /* first edge at -4.0 */
    assert(st(0).count == -5 + (int32_t)(N / 4u));

    (void)interval(-q, -1);
    assert(interval(0, -1) == 0);
    port_lock();
    puldir_set_datum(0, 3);
    port_unlock();
    s = st(0);
    assert(s.count == 3 && s.P == (uint64_t)3 << 32 && s.dir == -1);
    assert(interval(-q, -1) == N / 4u);   /* first edge at 2.0 */
    assert(st(0).count == 3 - (int32_t)(N / 4u));

    /* the count wraps: from INT32_MAX - 2 up through INT32_MAX to INT32_MIN */
    (void)interval(q, 1);
    assert(interval(0, 1) == 0);
    port_lock();
    puldir_set_datum(0, INT32_MAX - 2);
    port_unlock();
    s = st(0);
    assert(s.count == INT32_MAX - 2 && s.P == (uint64_t)(INT32_MAX - 2) << 32 && s.dir == 1);
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

#define FIX 4294967296.0f
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

static void track_run(PulDir* a, uint32_t ticks)
{
    const uint32_t plan_left = fake_plan_phase == 0u ? N : N - fake_plan_phase;
    for (uint32_t k = 0; k < ticks; k++) {
        if (left() == N) {                    /* this tick latches: the running set has finished */
            const PulDirPulse s = st(0);
            if (aim_cur.valid) {
                int64_t err = (int64_t)(s.P - aim_cur.aim);
                if (err < 0)
                    err = -err;
                const int64_t inc = s.inc < 0 ? -(int64_t)s.inc : s.inc;
                int64_t* worst = aim_cur.reversal ? &worst_rev : &worst_track;
                if (!aim_cur.near && err - inc > *worst)
                    *worst = err - inc;
                assert(err <= (aim_cur.reversal ? 2 : 1) * inc + (int64_t)N + (aim_cur.near ? M : 0));
            }
            aim_cur = aim_next;
            aim_cur.reversal = aim_cur.dir != s.dir;    /* the set about to latch turns the pin */
        }
        fake_pulse(false);
        if (left() == plan_left) {
            fake_plan();
            const uint64_t aim = ((uint64_t)(uint32_t)a->origin << 32)
                               + (uint64_t)(int64_t)(a->prof.pos * FIX);
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

/* Limits in pulses, set in user units after the conversion, in the natural order. */
static void set_limits(PulDir* a, float conv, float v, float acc, float dec, float jerk)
{
    assert(PulDir_setConversion(a, conv));
    assert(PulDir_setDefaults(a, v / conv, acc / conv, dec / conv, jerk / conv));
}

static void t_move_exact(void)
{
    setup(1);
    fake_on_write(count_write);
    PulDir* a = &ax[0];
    const float cap = (float)(PULDIR_TICK_HZ / 2u);
    const float dt = 1.0f / (float)PULDIR_PLAN_HZ;
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
        track_until_idle(a, 60u * PULDIR_TICK_HZ);
        const PulDirPulse s = st(0);
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
    assert(MotionProfile_init(&g, 0.0f, 1.0f / (float)PULDIR_PLAN_HZ));
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
    run_until_idle(a, 10u * PULDIR_TICK_HZ);
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

/* Test 23: at 200 kHz, 5 us pulses at a 10 us period. */
static void t_full_rate(void)
{
#if PULDIR_TICK_HZ == 200000u
    setup(1);
    fake_on_write(count_write);
    PulDir* a = &ax[0];
    assert(PulDir_setSpeed(a, 99990.0f));
    assert(PulDir_setAcceleration(a, 1.0e6f) && PulDir_setDeceleration(a, 1.0e6f));
    assert(!PulDir_setSpeed(a, 100000.0f) && PulDir_speed(a) == 99990.0f);
    assert(a->refusal == PULDIR_R_RATE);
    assert(PulDir_moveTo(a, 50000.0f));
    run_until_idle(a, 2u * PULDIR_TICK_HZ);
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
#endif
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
            assert(t < 10u * PULDIR_TICK_HZ);
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
    const float jerk = 40.0f / (250.0f / (float)PULDIR_PLAN_HZ);   /* a 250-tap window */
    assert(PulDir_setDefaults(a, 50.0f, 20.0f, 20.0f, jerk));
    const int32_t steps[4] = { 3, -1, 2, -4 };
    for (int k = 0; k < 8; k++) {
        fake_plan_phase = (uint32_t)k * (N / 8u);
        const int32_t c0 = st(0).count, want = c0 + steps[k % 4];
        assert(PulDir_moveTo(a, (float)want));
        run_until_idle(a, 10u * PULDIR_TICK_HZ);
        assert(st(0).count == want);
        assert((last_rise[0] - PULDIR_TICK_START) % N == N - 1u);
    }
}

/* Logging and refusals. */
static char lines[8][96];
static int nlines;
static void logger(const char* s)
{
    assert(nlines < 8 && strlen(s) < 96);
    strcpy(lines[nlines++], s);
}

static void t_refusals(void)
{
    /* the natural setup order: the conversion first, then limits in user units */
    PulDir b;
    PulDir_init(&b, "b");
    assert(PulDir_setConversion(&b, 280.0f));
    assert(PulDir_setDefaults(&b, 15.0f, 30.0f, 30.0f, 0.0f));
    assert(PulDir_speed(&b) == 15.0f && PulDir_acceleration(&b) == 30.0f);
    assert(PulDir_deceleration(&b) == 30.0f && PulDir_jerk(&b) == 0.0f && b.def[0] == 15.0f);

    setup(1);
    PulDir* a = &ax[0];
    PulDir_attachLogger(logger);

    assert(!PulDir_moveTo(a, NAN));
    assert(!PulDir_isMoving(a) && a->mode == PULDIR_MODE_IDLE && a->req == 0);
    assert(!PulDir_moveTo(a, 3.0e9f));       /* beyond 2^31 pulses */
    PulDir_loop(a);
    assert(nlines == 1 && strstr(lines[0], "moveTo refused") && strncmp(lines[0], "[a0] ", 5) == 0);

    /* setConversion checks only its argument; limits and defaults stay fixed in pulses */
    assert(!PulDir_setConversion(a, 0.0f) && !PulDir_setConversion(a, -2.0f));
    assert(!PulDir_setConversion(a, INFINITY) && !PulDir_setConversion(a, NAN));
    assert(a->conv == 1.0f && a->refusal == PULDIR_R_ARG);
    assert(PulDir_setConversion(a, 1000.0f));     /* 200 pulses/s stays 200 pulses/s */
    assert(PulDir_speed(a) == 0.2f && PulDir_jerk(a) == 0.0f && a->def[1] == 0.2f);
    assert(PulDir_setConversion(a, 2.0f));
    assert(PulDir_speed(a) == 100.0f && PulDir_acceleration(a) == 100.0f && a->def[2] == 100.0f);
    assert(PulDir_setAcceleration(a, 200.0f) && PulDir_setDeceleration(a, 200.0f));

    /* every limit: refused on bad input, unchanged */
    assert(!PulDir_setSpeed(a, 0.0f) && !PulDir_setSpeed(a, NAN) && PulDir_speed(a) == 100.0f);
    assert(!PulDir_setSpeed(a, (float)(PULDIR_TICK_HZ / 2u) / 2.0f) && PulDir_speed(a) == 100.0f);
    assert(PulDir_setSpeed(a, nextafterf((float)(PULDIR_TICK_HZ / 2u) / 2.0f, 0.0f)));
    assert(PulDir_setSpeed(a, 100.0f));
    assert(!PulDir_setAcceleration(a, -1.0f) && PulDir_acceleration(a) == 200.0f);
    assert(!PulDir_setDeceleration(a, INFINITY) && PulDir_deceleration(a) == 200.0f);
    assert(!PulDir_setJerk(a, -1.0f) && PulDir_jerk(a) == 0.0f);
    nlines = 0;
    PulDir_loop(a);
    assert(nlines == 1 && strstr(lines[0], "setJerk refused"));

    /* a jerk whose window would not fit the ring: refused, the line names the smallest that fits */
    const float dt = 1.0f / (float)PULDIR_PLAN_HZ;
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
    assert(!PulDir_setConversion(a, 3.0f) && a->refusal == PULDIR_R_NOT_IDLE && a->conv == 2.0f);
    PulDir_loop(a);
    assert(nlines == 1 && strcmp(lines[0], "[a0] setConversion refused: not idle") == 0);
    nlines = 0;
    run_until_idle(a, 10u * PULDIR_TICK_HZ);
    PulDir_loop(a);
    assert(nlines == 1 && strcmp(lines[0], "[a0] landed at -12.5000") == 0);
    assert(PulDir_position(a) == -12.5f && PulDir_status(a) == PULDIR_ENABLED);

    /* moveBy adds whole pulses to the target */
    assert(!PulDir_moveBy(a, NAN) && !PulDir_moveBy(a, 2.0e9f) && !PulDir_isMoving(a));
    assert(PulDir_moveBy(a, 1.2f));         /* 2.4 pulses rounds to 2 */
    assert(PulDir_target(a) == -11.5f && PulDir_isMoving(a) && PulDir_status(a) == PULDIR_MOVING);
    run_until_idle(a, 10u * PULDIR_TICK_HZ);
    assert(st(0).count == -23 && PulDir_position(a) == -11.5f);
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
    assert(dv <= fmaxf(L.amax, L.dmax) * (1.0f / (float)PULDIR_PLAN_HZ) * 1.0001f + 1e-3f);
}

/* At rest the pulse position is within a pulse of the count, on the side the
 * hysteresis leaves it (D3) for the direction of travel, when there is one. */
static void check_rest(uint8_t i, int travel)
{
    const PulDirPulse s = st(i);
    const int64_t off = (int64_t)(s.P - ((uint64_t)(uint32_t)s.count << 32));
    assert(off < (int64_t)PULDIR_ONE && off > -(int64_t)PULDIR_ONE);
    assert(travel <= 0 || off >= 0);
    assert(travel >= 0 || off <= 0);
    assert(s.inc == 0 && !high(i));
}

/* D7, 2.2: a jog never resets the count, changes speed live, reverses under
 * the ramps, and jog(0) stops at target == position. */
static void t_jog(void)
{
    setup(1);
    fake_on_write(count_write);
    fake_on_plan(check_ramps);
    PulDir* a = &ax[0];
    const float conv = 100.0f;
    assert(PulDir_setConversion(a, conv));
    assert(PulDir_setDefaults(a, 20.0f, 100.0f, 100.0f, 0.0f));   /* 2,000 pulses/s, 10,000 pulses/s^2 */
    assert(PulDir_setPosition(a, 10.0f));
    fake_run(N);
    assert(st(0).count == 1000 && PulDir_position(a) == 10.0f);

    /* refused at or above the rate cap and when not finite; nothing changes */
    const float cap = (float)(PULDIR_TICK_HZ / 2u) / conv;
    assert(!PulDir_jog(a, cap) && a->refusal == PULDIR_R_RATE && a->refused_value == cap);
    assert(!PulDir_jog(a, -cap) && a->refusal == PULDIR_R_RATE);
    assert(!PulDir_jog(a, NAN) && a->refusal == PULDIR_R_ARG);
    assert(!PulDir_jog(a, -INFINITY) && a->refusal == PULDIR_R_ARG);
    assert(!PulDir_isMoving(a) && a->mode == PULDIR_MODE_IDLE && a->req == 0);

    /* jog(5) = 500 pulses/s, counting on from 1,000 */
    assert(PulDir_jog(a, 5.0f));
    assert(PulDir_isMoving(a) && a->mode == PULDIR_MODE_PROFILE);
    int32_t c = st(0).count;
    for (uint32_t k = 0; k < PULDIR_PLAN_HZ; k++) {           /* 1 s */
        fake_run(N);
        assert(st(0).count >= c);
        c = st(0).count;
    }
    assert(PulDir_velocity(a) == 5.0f);
    /* 1 s at 500 less the 50 ms ramp's 12.5, less up to two intervals of latency */
    assert(c - 1000 > 480 && c - 1000 <= 488);

    /* jog(10): the speed changes live */
    assert(PulDir_jog(a, 10.0f));
    for (uint32_t k = 0; k < PULDIR_PLAN_HZ / 2u; k++) {
        fake_run(N);
        assert(st(0).count >= c);
        c = st(0).count;
    }
    assert(PulDir_velocity(a) == 10.0f);
    /* a jog has no target: it reads where the profile is, an interval or two ahead of the pulses */
    assert(PulDir_target(a) > PulDir_position(a) && PulDir_target(a) - PulDir_position(a) < 0.25f);

    /* jog(-5) reverses through zero under the ramps (check_ramps) */
    assert(PulDir_jog(a, -5.0f));
    int32_t top = c;
    for (uint32_t k = 0; k < PULDIR_PLAN_HZ; k++) {
        fake_run(N);
        if (st(0).count > top)
            top = st(0).count;
    }
    assert(PulDir_velocity(a) == -5.0f && st(0).dir == -1 && st(0).count < top - 380);

    /* jog(0) is stop: no reversal, IDLE, target == position */
    const uint32_t d0 = dirw[0];
    assert(PulDir_jog(a, 0.0f));
    assert(a->mode == PULDIR_MODE_STOPPING && PulDir_isMoving(a));
    run_until_idle(a, PULDIR_TICK_HZ);
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
        run_until_idle(a, PULDIR_TICK_HZ);
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
    run_until_idle(a, 2u * PULDIR_TICK_HZ);
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
    const float cap = (float)(PULDIR_TICK_HZ / 2u);
    const float dt = 1.0f / (float)PULDIR_PLAN_HZ;
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
        const int travel = kind == 0 ? (tg > pos0) - (tg < pos0) : (a->req_arg[0] > 0.0f) - (a->req_arg[0] < 0.0f);
        fake_run(rnd() % (300u * N));
        if (kind == 2) {
            const uint64_t e0 = edges[0];
            PulDir_halt(a);
            run_until_idle(a, 3u * N);
            fake_run(2u * N);
            assert(edges[0] == e0);
        } else {
            PulDir_stop(a);
            run_until_idle(a, PULDIR_TICK_HZ);
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
            run_until_idle(a, 10u * PULDIR_TICK_HZ);
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
        assert(PulDir_setJerk(a, k % 6 < 3 ? 0.0f : 1.0e7f));     /* a 10-tap window */
        const int32_t c0 = st(0).count, dir = k % 2 ? -1 : 1;
        assert(PulDir_moveTo(a, (float)(c0 + dir * 20000)));
        fake_run(200u * N);                   /* cruising: the ramp takes 100 intervals */
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
        const float frac = (float)(int64_t)(st(0).P - ((uint64_t)(uint32_t)a->origin << 32)) / FIX;
        assert(a->prof.done && a->prof.vel == 0.0f && fabsf(a->prof.pos - frac) < 1e-6f);
        assert(a->origin == ch && fabsf(frac) < 1.0f);

        /* the next move starts where the pulses stopped and lands exactly */
        const int64_t n0 = net[0];
        assert(PulDir_moveTo(a, (float)(ch - dir * 777)));
        run_until_idle(a, 2u * PULDIR_TICK_HZ);
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
    setup(1);
    fake_on_write(count_write);
    PulDir* a = &ax[0];
    const float conv = 3.0f;
    assert(PulDir_setConversion(a, conv));
    const float cap = (float)(PULDIR_TICK_HZ / 2u) / conv, below = nextafterf(cap, 0.0f);
    const float s0 = PulDir_speed(a);
    assert(!PulDir_setSpeed(a, cap) && a->refusal == PULDIR_R_RATE && PulDir_speed(a) == s0);
    assert(PulDir_setSpeed(a, below) && PulDir_speed(a) == below);
    assert(PulDir_setAcceleration(a, 2.0e6f / conv) && PulDir_setDeceleration(a, 2.0e6f / conv));

    const float to = (float)(PULDIR_TICK_HZ / 4u) / conv;     /* half a second at the cap */
    const int32_t want = (int32_t)roundf(to * conv);
    assert(PulDir_moveTo(a, to));
    int32_t worst = 0;
    for (uint32_t t = 0; PulDir_isMoving(a); t++) {
        assert(t < 2u * PULDIR_TICK_HZ);
        fake_run(1);
        const int32_t m = abs_inc();
        assert(m <= PULDIR_INC_MAX);
        if (m > worst)
            worst = m;
    }
    assert(st(0).count == want && worst > PULDIR_INC_MAX - PULDIR_INC_MAX / 1000);

    /* the same cap for a jog */
    assert(!PulDir_jog(a, cap) && a->refusal == PULDIR_R_RATE && !PulDir_isMoving(a));
    assert(PulDir_jog(a, -below));
    worst = 0;
    for (uint32_t t = 0; t < PULDIR_TICK_HZ / 5u; t++) {
        fake_run(1);
        const int32_t m = abs_inc();
        assert(m <= PULDIR_INC_MAX);
        if (m > worst)
            worst = m;
    }
    assert(worst > PULDIR_INC_MAX - PULDIR_INC_MAX / 1000);
    PulDir_stop(a);
    for (uint32_t t = 0; PulDir_isMoving(a); t++) {
        assert(t < PULDIR_TICK_HZ);
        fake_run(1);
        assert(abs_inc() <= PULDIR_INC_MAX);
    }
}

/* 2.2 and test 24, the axis half: which limit changes are live. */
static void t_live_limits(void)
{
    setup(1);
    PulDir_attachLogger(logger);
    PulDir* a = &ax[0];
    const float conv = 100.0f;
    assert(PulDir_setConversion(a, conv));

    /* jerk off, in flight: accel, decel and speed are live; switching jerk on is not */
    assert(PulDir_setDefaults(a, 20.0f, 40.0f, 40.0f, 0.0f));
    assert(PulDir_moveTo(a, 100.0f));
    fake_run(20u * N);
    assert(!a->prof.done);
    assert(PulDir_setAcceleration(a, 30.0f) && PulDir_setDeceleration(a, 30.0f));
    assert(PulDir_setSpeed(a, 10.0f));
    fake_run(N);
    assert(a->prof.lim.amax == 3000.0f && a->prof.lim.dmax == 3000.0f && a->prof.lim.vmax == 1000.0f);
    assert(!PulDir_setJerk(a, 300.0f) && a->refusal == PULDIR_R_WINDOW);   /* fits the taps, not the frozen window */
    assert(PulDir_jerk(a) == 0.0f && a->req == 0);
    fake_run(500u * N);
    assert(PulDir_velocity(a) == 10.0f);                 /* the lowered speed took effect */
    run_until_idle(a, 30u * PULDIR_TICK_HZ);
    assert(st(0).count == 10000);

    /* jerk 290 in flight: the window holds 207 taps, (30 + 30) / (290 * dt) = 206.9 */
    assert(PulDir_setDefaults(a, 20.0f, 40.0f, 30.0f, 290.0f));  /* 7000 / 29 = 241.4 */
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
    assert(!PulDir_setJerk(a, 250.0f) && a->refusal == PULDIR_R_WINDOW && PulDir_jerk(a) == 290.0f);
    assert(PulDir_setJerk(a, 400.0f));                   /* raised: live */
    a->refusal = PULDIR_R_NONE;
    assert(!PulDir_restoreDefaults(a) && a->refusal == PULDIR_R_WINDOW);   /* accel 40: 241.4 taps */
    assert(PulDir_speed(a) == 20.0f && PulDir_acceleration(a) == 30.0f);
    assert(PulDir_deceleration(a) == 30.0f && PulDir_jerk(a) == 400.0f);
    fake_run(N);
    assert(a->prof.lim.jmax == 40000.0f && a->prof.lim.amax == 3000.0f && a->prof.n == 207);
    run_until_idle(a, 30u * PULDIR_TICK_HZ);
    assert(st(0).count == 15000);
    assert(PulDir_restoreDefaults(a));                   /* idle: accepted */
    assert(PulDir_speed(a) == 20.0f && PulDir_acceleration(a) == 40.0f);
    assert(PulDir_deceleration(a) == 30.0f && PulDir_jerk(a) == 290.0f);
    fake_run(N);
    assert(a->prof.n == 242 && a->prof.lim.amax == 4000.0f);

    /* a jerk needing more than the ring's taps: refused, the log names the smallest that fits */
    const float jmin = MotionProfile_minJerk(4000.0f, 3000.0f, 1.0f / (float)PULDIR_PLAN_HZ) / conv;
    nlines = 0;
    PulDir_loop(a);
    nlines = 0;
    assert(!PulDir_setJerk(a, 0.9f * jmin) && a->refusal == PULDIR_R_TAPS && a->refused_value == jmin);
    assert(PulDir_jerk(a) == 290.0f);
    PulDir_loop(a);
    assert(nlines == 1 && strstr(lines[0], "setJerk refused: jerk below 273.4"));
    assert(strstr(lines[0], " needs more than 256 taps"));
    assert(PulDir_setJerk(a, 1.001f * jmin));           /* just above the smallest that fits */
}

/* D18: the datum moves only in IDLE, and moves everything together. */
static void t_setpos(void)
{
    setup(1);
    fake_on_write(count_write);
    PulDir* a = &ax[0];
    assert(PulDir_setConversion(a, 100.0f));
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
    run_until_idle(a, PULDIR_TICK_HZ);
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
    assert(st(0).P != (uint64_t)(uint32_t)st(0).count << 32);   /* halted between pulses */

    /* accepted in IDLE: count, pulse position, target and generator move together */
    assert(PulDir_setPosition(a, -3.0f));
    PulDirPulse s = st(0);
    assert(s.count == -300 && s.P == (uint64_t)(int64_t)-300 << 32);
    assert(PulDir_position(a) == -3.0f && PulDir_target(a) == -3.0f && !a->homed);
    fake_run(N);
    assert(a->origin == -300 && a->prof.pos == 0.0f && a->prof.target == 0.0f && a->prof.done);
    assert(!PulDir_isMoving(a) && a->mode == PULDIR_MODE_IDLE);

    /* the next move is relative to the new datum */
    const uint64_t e0 = edges[0];
    const int64_t n0 = net[0];
    assert(PulDir_moveTo(a, -2.0f));
    run_until_idle(a, PULDIR_TICK_HZ);
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
            assert(t < 10u * PULDIR_TICK_HZ);
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
        fake_run(200u * N);                   /* 50 ms of ramp, then 150 ms at speed */
        assert(PulDir_isMoving(a) && a->mode == PULDIR_MODE_PROFILE);
        assert(PulDir_velocity(a) == speed);
        const int32_t c = st(0).count;
        assert(speed > 0.0f ? c > want + 100 : c < want - 100);
        PulDir_stop(a);
        run_until_idle(a, PULDIR_TICK_HZ);
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
    const float v = 0.4f * (float)PULDIR_TICK_HZ;          /* an edge every 2 or 3 ticks */
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

/* ---- the case table ------------------------------------------------------------------------ */

static const struct { const char* name; void (*fn)(void); } cases[] = {
    { "setup", t_setup },
    { "pulse_rate", t_pulse_rate },
    { "dir_change", t_dir_change },
    { "polarity", t_polarity },
    { "latch", t_latch },
    { "inc_max", t_inc_max },
    { "hysteresis", t_hysteresis },
    { "register_after_start", t_register_after_start },
    { "kill", t_kill },
    { "datum", t_datum },
    { "overrun", t_overrun },
    { "move_exact", t_move_exact },
    { "parity", t_parity },
    { "full_rate", t_full_rate },
    { "arrival_twice", t_arrival_twice },
    { "creep", t_creep },
    { "refusals", t_refusals },
    { "jog", t_jog },
    { "stop", t_stop },
    { "stop_random", t_stop_random },
    { "halt", t_halt },
    { "rate_cap", t_rate_cap },
    { "live_limits", t_live_limits },
    { "setpos_idle_only", t_setpos },
    { "settle_twice", t_settle_twice },
    { "jog_settling", t_jog_settling },
    { "halt_any_tick", t_halt_any_tick },
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
