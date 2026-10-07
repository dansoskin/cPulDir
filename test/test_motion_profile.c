/* Generator tests (guidelines section 8, tests 1-9), each printing "ok <name>".
 * Every advance goes through step(), which checks the limits of 5.2. */
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motion_profile.h"

/* After every advance: velocity, acceleration and (with jerk) jerk bounds.
 * The tolerance absorbs float rounding, not design slack. */
static int g_vmax_relaxed;   /* set by tests that lower vmax or retarget infeasibly */

static void check_step(const MotionProfile* p, float vel0, float acc0)
{
    const float tol = 1e-4f;
    const MotionLimits L = p->lim;
    const float dv = p->vel - vel0;
    const bool speeding_up = fabsf(p->vel) > fabsf(vel0);
    float bound = speeding_up ? L.amax : L.dmax;

    /* A zero crossing in one call is a shrink then a growth. */
    if ((p->vel > 0.0f && vel0 < 0.0f) || (p->vel < 0.0f && vel0 > 0.0f))
        bound = fmaxf(L.amax, L.dmax);
    /* The new velocity is rounded to its own ulp: at |v| = 32 that alone is 1.9e-6. */
    const float ulp = fmaxf(fabsf(p->vel), fabsf(vel0)) * FLT_EPSILON;
    if (!g_vmax_relaxed)
        assert(fabsf(p->vel) <= L.vmax * (1.0f + tol));
    assert(fabsf(dv) <= bound * p->dt * (1.0f + tol) + 1e-6f + ulp);
    /* acc is a difference of velocities over dt, so each carries about two
     * velocity ulps over dt, and the ring's taps reach vmax even when vel is small. */
    const float tap_ulp = fmaxf(ulp, L.vmax * FLT_EPSILON);
    if (L.jmax > 0.0f)
        assert(fabsf(p->acc - acc0) <= L.jmax * p->dt * (1.0f + tol) + 4.0f * tap_ulp / p->dt);
}

static void step(MotionProfile* p)
{
    const float v0 = p->vel, a0 = p->acc;
    MotionProfile_advance(p);
    check_step(p, v0, a0);
}

static bool limits_equal(MotionLimits a, MotionLimits b)
{
    return memcmp(&a, &b, sizeof a) == 0;
}

/* Field by field: a whole-struct memcmp would compare indeterminate padding. */
static bool profiles_equal(const MotionProfile* a, const MotionProfile* b)
{
    return a->dt == b->dt && a->pos == b->pos && a->vel == b->vel && a->acc == b->acc
        && a->target == b->target && a->endless == b->endless && a->stopping == b->stopping
        && a->landed == b->landed && a->done == b->done && limits_equal(a->lim, b->lim)
        && a->p_t == b->p_t && a->p_c == b->p_c && a->v_t == b->v_t && a->speed == b->speed
        && memcmp(a->taps, b->taps, sizeof a->taps) == 0
        && a->n == b->n && a->head == b->head && a->drain == b->drain && a->sum == b->sum
        && a->sum_c == b->sum_c && a->wsum == b->wsum && a->wsum_c == b->wsum_c;
}

static void test_setup(void)
{
    MotionProfile p;

    assert(MotionProfile_init(&p, 3.0f, 0.001f));
    assert(p.done && p.landed);
    assert(p.pos == 3.0f && p.target == 3.0f);
    assert(p.vel == 0.0f && p.acc == 0.0f);
    assert(p.lim.vmax == 1.0f && p.lim.amax == 1.0f && p.lim.dmax == 1.0f && p.lim.jmax == 0.0f);
    assert(p.n == 1);

    /* init refuses and leaves the profile alone */
    {
        MotionProfile q = p;
        assert(!MotionProfile_init(&q, 0.0f, 0.0001f));
        assert(!MotionProfile_init(&q, 0.0f, 0.011f));
        assert(!MotionProfile_init(&q, 0.0f, NAN));
        assert(!MotionProfile_init(&q, NAN, 0.001f));
        assert(!MotionProfile_init(&q, INFINITY, 0.001f));
        assert(profiles_equal(&q, &p));
    }

    /* setLimits refusals change nothing */
    {
        const MotionLimits before = p.lim;
        const MotionLimits bad[] = {
            { NAN, 1, 1, 0 }, { 1, NAN, 1, 0 }, { 1, 1, NAN, 0 }, { 1, 1, 1, NAN },
            { INFINITY, 1, 1, 0 }, { 1, INFINITY, 1, 0 }, { 1, 1, INFINITY, 0 }, { 1, 1, 1, INFINITY },
            { -1, 1, 1, 0 }, { 1, -1, 1, 0 }, { 1, 1, -1, 0 }, { 1, 1, 1, -1 },
            { 0, 1, 1, 0 }, { 1, 0, 1, 0 }, { 1, 1, 0, 0 },
        };
        for (unsigned i = 0; i < sizeof bad / sizeof bad[0]; i++) {
            assert(!MotionProfile_setLimits(&p, bad[i]));
            assert(limits_equal(p.lim, before));
        }
    }

    {
        const MotionLimits trap = { 15, 30, 30, 0 }, jerk = { 15, 30, 30, 300 }, soft = { 15, 30, 30, 1 };
        assert(MotionProfile_setLimits(&p, trap));
        assert(limits_equal(p.lim, trap) && p.n == 1);
        assert(MotionProfile_setLimits(&p, jerk));
        assert(limits_equal(p.lim, jerk) && p.n == 200);     /* ceil(60 / 0.3) */
        assert(!MotionProfile_setLimits(&p, soft));          /* 60,000 taps */
        assert(limits_equal(p.lim, jerk) && p.n == 200);
        /* 60 / (256 * 0.001f) in float is one ulp under 234.375 */
        assert(fabsf(MotionProfile_minJerk(30, 30, 0.001f) - 234.375f) <= 234.375f * 1e-6f);
    }

    assert(MotionProfile_setPosition(&p, 5.0f));
    assert(p.pos == 5.0f && p.target == 5.0f && p.p_t == 5.0f);
    assert(!MotionProfile_setPosition(&p, NAN));
    assert(p.pos == 5.0f);

    {
        const MotionProfile before = p;
        assert(!MotionProfile_setTarget(&p, NAN));
        assert(!MotionProfile_setTarget(&p, INFINITY));
        assert(profiles_equal(&before, &p));
    }

    {
        MotionProfile q;
        assert(MotionProfile_init(&q, 2.0f, 0.001f));
        MotionProfile_halt(&q);
        assert(q.done && q.landed && q.pos == 2.0f && q.target == 2.0f);
        assert(q.vel == 0.0f && q.acc == 0.0f);
        step(&q);                                            /* done: nothing moves */
        assert(q.done && q.pos == 2.0f);
        assert(MotionProfile_stopDistance(&q) == 0.0f);
    }
    assert(MotionProfile_stopDistance(&p) == 0.0f);

    printf("ok setup\n");
}

typedef struct { int calls; float peak; int accel_calls, decel_calls; } Run;

static Run run_to_done(MotionProfile* p, int max_calls)
{
    Run r = {0, 0.0f, 0, 0};
    while (!p->done) {
        const float v0 = fabsf(p->vel);
        step(p);
        r.calls++;
        if (fabsf(p->vel) > r.peak) r.peak = fabsf(p->vel);
        if (fabsf(p->vel) > v0) r.accel_calls++;
        if (fabsf(p->vel) < v0) r.decel_calls++;
        assert(r.calls < max_calls);
    }
    return r;
}

static MotionProfile fresh(float pos, float dt, MotionLimits L)
{
    MotionProfile p;
    assert(MotionProfile_init(&p, pos, dt));
    assert(MotionProfile_setLimits(&p, L));
    return p;
}

/* Calls beyond the analytic trapezoid: the drained call and the done call, plus the
 * discrete ceiling's tail (v' = sqrt(dd^2 + v^2) - dd sheds under dd), about ln(vmax/(dmax*dt))/2. */
static float extra_calls(MotionLimits L, float dt)
{
    return 2.0f + 0.5f * logf(L.vmax / (L.dmax * dt));
}

/* Test 2: against the analytic trapezoid (D8): 0.5 s up, 0.1 s cruise, 0.5 s down. */
static void test_trapezoid(void)
{
    const MotionLimits L = { 15, 30, 30, 0 };
    MotionProfile p = fresh(0.0f, 0.001f, L);
    assert(MotionProfile_setTarget(&p, 9.0f));
    const Run r = run_to_done(&p, 5000);
    assert(fabsf((float)r.calls - (1100.0f + extra_calls(L, 0.001f))) <= 2.0f);
    assert(r.peak == 15.0f);
    assert(p.vel == 0.0f && p.acc == 0.0f);
    assert(p.pos == 9.0f && p.target == 9.0f);

    /* The negative move mirrors it exactly. */
    MotionProfile q = fresh(0.0f, 0.001f, L);
    assert(MotionProfile_setTarget(&q, -9.0f));
    const Run s = run_to_done(&q, 5000);
    assert(s.calls == r.calls && s.peak == r.peak);
    assert(q.pos == -9.0f && q.vel == 0.0f);

    printf("ok trapezoid\n");
}

/* Test 3: up ramp 0.75 s, down ramp 0.1667 s. */
static void test_asymmetric(void)
{
    MotionProfile p = fresh(0.0f, 0.001f, (MotionLimits){ 15, 20, 90, 0 });
    assert(MotionProfile_setTarget(&p, 30.0f));
    const Run r = run_to_done(&p, 10000);
    const float ratio = (float)r.accel_calls / (float)r.decel_calls;
    assert(fabsf(ratio - 4.5f) <= 4.5f * 0.02f);
    assert(p.pos == 30.0f);
    printf("ok asymmetric\n");
}

static uint32_t lcg = 12345u;
static float frand(float lo, float hi)
{
    lcg = lcg * 1664525u + 1013904223u;
    return lo + (hi - lo) * (float)(lcg >> 8) / 16777216.0f;
}

static void run_until_vel(MotionProfile* p, float vel, int max_calls)
{
    for (int i = 0; p->vel != vel; i++) {
        assert(i < max_calls);
        step(p);
    }
}

/* Test 5: a target set from rest is never passed and is landed on exactly.
 * n moves, start within +-start_span, distance within +-dist_span (>= 0.001). */
static void random_moves(int n, float start_span, float dist_span)
{
    for (int i = 0; i < n; i++) {
        const MotionLimits L = { frand(1, 50), frand(1, 200), frand(1, 200), 0 };
        const float start = frand(-start_span, start_span);
        float d = frand(-dist_span, dist_span);
        if (fabsf(d) < 0.001f)
            d = copysignf(0.001f, d);
        MotionProfile p = fresh(start, 0.001f, L);
        assert(MotionProfile_setTarget(&p, start + d));
        const float sense = p.target - p.pos;
        for (int calls = 0; !p.done; calls++) {
            assert(calls < 10000000);    /* 5000 units at vmax 1, plus ramps */
            step(&p);
            assert((p.target - p.pos) * sense >= 0.0f);
        }
        assert(p.pos == p.target && p.vel == 0.0f);
    }
}

static void test_random_moves(void)
{
    random_moves(1000, 100.0f, 200.0f);
    printf("ok random_moves\n");
}

/* Large coordinates, as a virtual master axis in degrees: thousands of units
 * with small dmax, where uncompensated rounding of p_t passed targets. */
static void test_random_moves_large(void)
{
    random_moves(1000, 5000.0f, 5000.0f);
    printf("ok random_moves_large\n");
}

static int sign(float v) { return (v > 0.0f) - (v < 0.0f); }

/* Test 5: a target behind the axis costs exactly one reversal. */
static void test_retarget_behind(void)
{
    MotionProfile p = fresh(0.0f, 0.001f, (MotionLimits){ 15, 30, 30, 0 });
    assert(MotionProfile_setTarget(&p, 100.0f));
    run_until_vel(&p, 15.0f, 10000);
    assert(MotionProfile_setTarget(&p, p.pos - 5.0f));
    int last = sign(p.vel), changes = 0;
    for (int calls = 0; !p.done; calls++) {
        assert(calls < 10000);
        step(&p);
        const int s = sign(p.vel);
        if (s != 0 && s != last) {
            changes++;
            last = s;
        }
    }
    assert(changes == 1);
    assert(p.pos == p.target);
    printf("ok retarget_behind\n");
}

/* Test 5: a raised vmax applies at once, at amax. */
static void test_speed_raise(void)
{
    MotionProfile p = fresh(0.0f, 0.001f, (MotionLimits){ 10, 30, 30, 0 });
    assert(MotionProfile_setTarget(&p, 1000.0f));
    run_until_vel(&p, 10.0f, 10000);
    assert(MotionProfile_setLimits(&p, (MotionLimits){ 20, 30, 30, 0 }));
    const float dv = 30.0f * 0.001f;
    for (int calls = 0; p.vel != 20.0f; calls++) {
        assert(calls < 10000);
        const float v0 = p.vel;
        step(&p);
        if (p.vel != 20.0f)
            assert(fabsf((p.vel - v0) - dv) <= dv * 1e-4f);
        else                                 /* the last step only closes the gap */
            assert(p.vel - v0 > 0.0f && p.vel - v0 <= dv * (1.0f + 1e-4f));
    }
    printf("ok speed_raise\n");
}

/* Test 9: a target inside the stopping distance is passed, then reached. */
static void test_infeasible(void)
{
    MotionProfile p = fresh(0.0f, 0.001f, (MotionLimits){ 15, 30, 30, 0 });
    assert(MotionProfile_setTarget(&p, 100.0f));
    run_until_vel(&p, 15.0f, 10000);
    assert(MotionProfile_setTarget(&p, p.pos + 1.0f));
    float furthest = -INFINITY;
    for (int calls = 0; !p.done; calls++) {
        assert(calls < 10000);
        step(&p);
        furthest = fmaxf(furthest, p.pos - p.target);
    }
    assert(furthest >= 2.7f);            /* stopping distance 3.75, target 1 ahead */
    assert(p.pos == p.target);
    printf("ok infeasible\n");
}

/* n calls; asserts the position never jumps more than vmax*dt in one call.
 * Returns whether vel equalled want on any call. */
static bool run_endless(MotionProfile* p, int n, float want)
{
    bool reached = false;
    for (int i = 0; i < n; i++) {
        const float pos0 = p->pos;
        step(p);
        assert(fabsf(p->pos - pos0) <= p->lim.vmax * p->dt * (1.0f + 1e-4f));
        reached |= p->vel == want;
    }
    return reached;
}

/* Test 7: endless speed changes and a stop, position continuous throughout. */
static void test_endless(void)
{
    MotionProfile p = fresh(0.0f, 0.001f, (MotionLimits){ 20, 30, 60, 0 });
    assert(MotionProfile_setEndless(&p, 10.0f));
    assert(!p.done && p.endless);
    assert(run_endless(&p, 1000, 10.0f));
    assert(MotionProfile_setEndless(&p, 15.0f));
    assert(run_endless(&p, 1000, 15.0f));
    assert(MotionProfile_setEndless(&p, -15.0f));
    assert(run_endless(&p, 2000, -15.0f));
    MotionProfile_stop(&p);
    for (int calls = 0; !p.done; calls++) {
        assert(calls < 10000);
        run_endless(&p, 1, 0.0f);
    }
    assert(p.vel == 0.0f && p.pos == p.target && !p.endless);

    /* A speed above vmax runs at vmax (D7). */
    assert(MotionProfile_setEndless(&p, 30.0f));
    run_endless(&p, 2000, 20.0f);
    assert(p.vel == 20.0f);

    /* Refusal changes nothing. */
    const MotionProfile before = p;
    assert(!MotionProfile_setEndless(&p, NAN));
    assert(!MotionProfile_setEndless(&p, INFINITY));
    assert(profiles_equal(&before, &p));
    printf("ok endless\n");
}

/* Stop from the state p is in; the profile must come to rest stopDistance() ahead. */
static void stop_where_predicted(MotionProfile* p)
{
    const float d = MotionProfile_stopDistance(p), at = p->pos;
    MotionProfile_stop(p);
    run_to_done(p, 10000);
    assert(fabsf(p->pos - (at + d)) <= p->lim.vmax * p->dt);
    assert(p->target == p->pos && p->vel == 0.0f);
}

/* Test 6: stop at cruise and halfway up the ramp. */
static void test_stop(void)
{
    const MotionLimits L = { 15, 30, 30, 0 };
    MotionProfile p = fresh(0.0f, 0.001f, L);
    assert(MotionProfile_setTarget(&p, 1000.0f));
    run_until_vel(&p, 15.0f, 10000);
    stop_where_predicted(&p);

    MotionProfile q = fresh(0.0f, 0.001f, L);
    assert(MotionProfile_setTarget(&q, 1000.0f));
    for (int calls = 0; q.vel < 7.5f; calls++) {
        assert(calls < 10000);
        step(&q);
    }
    stop_where_predicted(&q);

    /* stop on a done profile is a no-op */
    const MotionProfile before = q;
    MotionProfile_stop(&q);
    assert(profiles_equal(&before, &q));
    printf("ok stop\n");
}

/* Test 6: halt is immediate and final. */
static void test_halt(void)
{
    MotionProfile p = fresh(0.0f, 0.001f, (MotionLimits){ 15, 30, 30, 0 });
    assert(MotionProfile_setTarget(&p, 1000.0f));
    run_until_vel(&p, 15.0f, 10000);
    MotionProfile_halt(&p);
    assert(p.vel == 0.0f && p.acc == 0.0f && p.done && p.target == p.pos);
    const MotionProfile before = p;
    MotionProfile_advance(&p);
    assert(profiles_equal(&before, &p));
    printf("ok halt\n");
}

/* Test 8: a lowered vmax is followed down at exactly dmax, then held. */
static void test_lowered_vmax(void)
{
    MotionProfile p = fresh(0.0f, 0.001f, (MotionLimits){ 15, 30, 30, 0 });
    assert(MotionProfile_setTarget(&p, 1000.0f));
    run_until_vel(&p, 15.0f, 10000);
    g_vmax_relaxed = 1;
    assert(MotionProfile_setLimits(&p, (MotionLimits){ 5, 30, 30, 0 }));
    const float dv = 30.0f * 0.001f;
    for (int calls = 0; p.vel != 5.0f; calls++) {
        assert(calls < 10000);
        const float v0 = p.vel;
        step(&p);
        if (p.vel != 5.0f)
            assert(fabsf((v0 - p.vel) - dv) <= dv * 1e-4f);
        else                                 /* the last step only closes the gap */
            assert(v0 - p.vel > 0.0f && v0 - p.vel <= dv * (1.0f + 1e-4f));
    }
    g_vmax_relaxed = 0;
    for (int i = 0; i < 100; i++) {
        step(&p);
        assert(p.vel == 5.0f);
    }
    printf("ok lowered_vmax\n");
}

/* Test 8: a lowered amax applies from the next call. */
static void test_lowered_amax(void)
{
    MotionProfile p = fresh(0.0f, 0.001f, (MotionLimits){ 15, 30, 30, 0 });
    assert(MotionProfile_setTarget(&p, 1000.0f));
    for (int i = 0; i < 100; i++)
        step(&p);
    assert(MotionProfile_setLimits(&p, (MotionLimits){ 15, 10, 30, 0 }));
    {
        const float v0 = p.vel;
        step(&p);
        assert(fabsf((p.vel - v0) - 10.0f * 0.001f) <= 10.0f * 0.001f * 1e-4f);
    }
    for (int calls = 0; p.vel != 15.0f; calls++) {
        assert(calls < 10000);
        const float v0 = p.vel;
        step(&p);
        assert(p.vel - v0 <= 10.0f * 0.001f * (1.0f + 1e-4f));
    }
    printf("ok lowered_amax\n");
}

/* Test 4: with jerk, the move is the trapezoid's plus the window, and lands exactly. */
static void test_scurve(void)
{
    const MotionLimits L = { 15, 30, 30, 300 };
    MotionProfile p = fresh(0.0f, 0.001f, L);
    assert(p.n == 200);
    assert(MotionProfile_setTarget(&p, 9.0f));
    int calls = 0, landed_call = -1;
    float peak = 0.0f;
    while (!p.done) {
        step(&p);
        calls++;
        assert(calls < 5000);
        if (p.landed && landed_call < 0)
            landed_call = calls;
        peak = fmaxf(peak, fabsf(p.vel));
        assert(p.pos <= 9.0f);
        assert(p.acc <= L.amax * (1.0f + 1e-4f) && p.acc >= -L.dmax * (1.0f + 1e-4f));
    }
    assert(calls - landed_call == 201);     /* n calls to drain, one more for acc to reach 0 */
    assert(p.pos == 9.0f && p.vel == 0.0f && p.acc == 0.0f);
    assert(peak <= 15.0f);
    /* The trapezoid's calls (1100 + extra_calls), plus the window's n - 1 calls of motion. */
    assert(fabsf((float)calls - (1100.0f + 200.0f + extra_calls(L, 0.001f) - 1.0f)) <= 2.0f);
    printf("ok scurve\n");
}

/* Test 4: with jmax = 0 the output is the trapezoid itself, bit for bit. */
static void test_trapezoid_bit_for_bit(void)
{
    MotionProfile p = fresh(0.0f, 0.001f, (MotionLimits){ 15, 30, 30, 0 });
    assert(MotionProfile_setTarget(&p, 9.0f));
    for (int calls = 0; !p.done; calls++) {
        assert(calls < 5000);
        const bool was_landed = p.landed;
        const float rem = p.target - p.p_t;
        step(&p);
        assert(p.pos == p.p_t);
        if (p.landed && !was_landed)
            assert(p.vel == rem / p.dt);      /* the landing call's velocity */
        else
            assert(p.vel == p.v_t);
    }
    printf("ok trapezoid_bit_for_bit\n");
}

/* Test 7: the running sum does not drift over ten million calls. */
static void test_sum_drift(void)
{
    MotionProfile p = fresh(0.0f, 0.001f, (MotionLimits){ 20, 30, 30, 300 });
    assert(MotionProfile_setEndless(&p, 12.345f));
    for (long i = 0; i < 10000000L; i++)
        step(&p);
    const float ulp = nextafterf(12.345f, 20.0f) - 12.345f;
    assert(fabsf(p.vel - 12.345f) <= 200.0f * ulp);
    printf("ok sum_drift\n");
}

/* Test 6 with jerk: stop at cruise and halfway up the ramp, where acc is not zero. */
static void test_stop_jerk(void)
{
    const MotionLimits L = { 15, 30, 30, 300 };
    MotionProfile p = fresh(0.0f, 0.001f, L);
    assert(MotionProfile_setTarget(&p, 1000.0f));
    run_until_vel(&p, 15.0f, 10000);
    stop_where_predicted(&p);

    MotionProfile q = fresh(0.0f, 0.001f, L);
    assert(MotionProfile_setTarget(&q, 1000.0f));
    for (int calls = 0; q.vel < 7.5f; calls++) {
        assert(calls < 10000);
        step(&q);
    }
    assert(q.acc != 0.0f);
    stop_where_predicted(&q);
    printf("ok stop_jerk\n");
}

/* Test 1: the same move at dt 1 ms and 5 ms differs only by the discretisation. */
static void test_dt_independence(void)
{
    const MotionLimits L = { 15, 30, 30, 0 };
    MotionProfile fine = fresh(0.0f, 0.001f, L), coarse = fresh(0.0f, 0.005f, L);
    assert(MotionProfile_setTarget(&fine, 9.0f));
    assert(MotionProfile_setTarget(&coarse, 9.0f));
    const Run f = run_to_done(&fine, 10000), c = run_to_done(&coarse, 10000);
    assert(fine.pos == 9.0f && coarse.pos == 9.0f);
    /* Each run's known tail grows as dt*ln(1/dt); compare what remains. */
    const float t_fine = ((float)f.calls - extra_calls(L, 0.001f)) * 0.001f;
    const float t_coarse = ((float)c.calls - extra_calls(L, 0.005f)) * 0.005f;
    assert(fabsf(t_fine - t_coarse) <= 0.005f);
    assert(fabsf(f.peak - c.peak) <= 30.0f * 0.005f);
    printf("ok dt_independence\n");
}

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);   /* keep the ok lines when an assert aborts */
    test_setup();
    test_trapezoid();
    test_asymmetric();
    test_random_moves();
    test_random_moves_large();
    test_retarget_behind();
    test_speed_raise();
    test_infeasible();
    test_endless();
    test_stop();
    test_halt();
    test_lowered_vmax();
    test_lowered_amax();
    test_dt_independence();
    test_trapezoid_bit_for_bit();
    test_scurve();
    test_stop_jerk();
    test_sum_drift();
    return 0;
}
