/* Generator tests (guidelines section 8, tests 1-9), each printing "ok <name>".
 * Every advance goes through step(), which checks the bounds of 5.2. */
#ifdef NDEBUG
#error "the tests call the code under test inside assert(): build without NDEBUG"
#endif
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motion_profile.h"

/* After every advance: velocity, acceleration and (with jerk) jerk bounds.
 * The tolerance absorbs float rounding, not design slack. */
static int g_speed_relaxed;   /* set by tests that lower speed or retarget infeasibly */
static float g_accel_carried; /* a lowered accel or decel the jerk ring still carries; 0 when none */

static void check_step(const MotionProfile* p, float vel0, float acc0)
{
    const float tol = 1e-4f;
    const MotionSettings S = p->settings;
    const float dv = p->vel - vel0;
    const bool speeding_up = fabsf(p->vel) > fabsf(vel0);
    float bound = fmaxf(speeding_up ? S.accel : S.decel, g_accel_carried);

    /* A zero crossing in one call is a shrink then a growth. */
    if ((p->vel > 0.0f && vel0 < 0.0f) || (p->vel < 0.0f && vel0 > 0.0f))
        bound = fmaxf(S.accel, S.decel);
    /* The new velocity is rounded to its own ulp: at |v| = 32 that alone is 1.9e-6. */
    const float ulp = fmaxf(fabsf(p->vel), fabsf(vel0)) * FLT_EPSILON;
    /* Through the filter, vel = sum/n adds the sum's rounding (an ulp of n*v, over n)
     * and the division's to both vel and vel0: about three ulps in all. */
    const float vel_ulps = p->window > 1 ? 3.0f : 1.0f;
    if (!g_speed_relaxed)
        assert(fabsf(p->vel) <= S.speed * (1.0f + tol));
    assert(fabsf(dv) <= bound * p->dt * (1.0f + tol) + 1e-6f + vel_ulps * ulp);
    /* acc is a difference of velocities over dt, so each carries about two
     * velocity ulps over dt, and the ring's taps reach speed even when vel is small. */
    const float tap_ulp = fmaxf(ulp, S.speed * FLT_EPSILON);
    if (S.jerk > 0.0f)
        assert(fabsf(p->acc - acc0) <= S.jerk * p->dt * (1.0f + tol) + 4.0f * tap_ulp / p->dt);
}

static void step(MotionProfile* p)
{
    const float v0 = p->vel, a0 = p->acc;
    MotionProfile_advance(p);
    check_step(p, v0, a0);
}

static bool settings_equal(MotionSettings a, MotionSettings b)
{
    return memcmp(&a, &b, sizeof a) == 0;
}

/* Field by field: a whole-struct memcmp would compare indeterminate padding. */
static bool profiles_equal(const MotionProfile* a, const MotionProfile* b)
{
    return a->dt == b->dt && a->pos == b->pos && a->vel == b->vel && a->acc == b->acc
        && a->target == b->target && a->endless == b->endless && a->stopping == b->stopping
        && a->landed == b->landed && a->done == b->done && settings_equal(a->settings, b->settings)
        && a->p_t == b->p_t && a->p_c == b->p_c && a->v_t == b->v_t && a->endless_speed == b->endless_speed
        && memcmp(a->taps, b->taps, sizeof a->taps) == 0
        && a->window == b->window && a->head == b->head && a->drain == b->drain && a->sum == b->sum
        && a->sum_c == b->sum_c && a->wsum == b->wsum && a->wsum_c == b->wsum_c
        && a->run == b->run && a->run_sign == b->run_sign
        && a->hist_acc == b->hist_acc && a->hist_left == b->hist_left;
}

static void test_setup(void)
{
    MotionProfile p;

    assert(MotionProfile_init(&p, 3.0f, 0.001f));
    assert(p.done && p.landed);
    assert(p.pos == 3.0f && p.target == 3.0f);
    assert(p.vel == 0.0f && p.acc == 0.0f);
    assert(p.settings.speed == 1.0f && p.settings.accel == 1.0f && p.settings.decel == 1.0f && p.settings.jerk == 0.0f);
    assert(p.window == 1);

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

    /* setSettings refusals change nothing */
    {
        const MotionSettings before = p.settings;
        const MotionSettings bad[] = {
            { NAN, 1, 1, 0 }, { 1, NAN, 1, 0 }, { 1, 1, NAN, 0 }, { 1, 1, 1, NAN },
            { INFINITY, 1, 1, 0 }, { 1, INFINITY, 1, 0 }, { 1, 1, INFINITY, 0 }, { 1, 1, 1, INFINITY },
            { -1, 1, 1, 0 }, { 1, -1, 1, 0 }, { 1, 1, -1, 0 }, { 1, 1, 1, -1 },
            { 0, 1, 1, 0 }, { 1, 0, 1, 0 }, { 1, 1, 0, 0 },
        };
        for (unsigned i = 0; i < sizeof bad / sizeof bad[0]; i++) {
            assert(!MotionProfile_setSettings(&p, bad[i]));
            assert(settings_equal(p.settings, before));
        }
    }

    {
        const MotionSettings trap = { 15, 30, 30, 0 }, jerk = { 15, 30, 30, 300 }, soft = { 15, 30, 30, 1 };
        assert(MotionProfile_setSettings(&p, trap));
        assert(settings_equal(p.settings, trap) && p.window == 1);
        assert(MotionProfile_setSettings(&p, jerk));
        assert(settings_equal(p.settings, jerk) && p.window == 200);     /* ceil(60 / 0.3) */
        assert(!MotionProfile_setSettings(&p, soft));          /* 60,000 taps */
        assert(settings_equal(p.settings, jerk) && p.window == 200);
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

static MotionProfile fresh(float pos, float dt, MotionSettings S)
{
    MotionProfile p;
    assert(MotionProfile_init(&p, pos, dt));
    assert(MotionProfile_setSettings(&p, S));
    return p;
}

/* Calls beyond the analytic trapezoid: the drained call and the done call, plus the
 * discrete ceiling's tail (v' = sqrt(dd^2 + v^2) - dd sheds under dd), about ln(speed/(decel*dt))/2. */
static float extra_calls(MotionSettings S, float dt)
{
    return 2.0f + 0.5f * logf(S.speed / (S.decel * dt));
}

/* Test 2: against the analytic trapezoid (D8): 0.5 s up, 0.1 s cruise, 0.5 s down. */
static void test_trapezoid(void)
{
    const MotionSettings S = { 15, 30, 30, 0 };
    MotionProfile p = fresh(0.0f, 0.001f, S);
    assert(MotionProfile_setTarget(&p, 9.0f));
    const Run r = run_to_done(&p, 5000);
    assert(fabsf((float)r.calls - (1100.0f + extra_calls(S, 0.001f))) <= 2.0f);
    assert(r.peak == 15.0f);
    assert(p.vel == 0.0f && p.acc == 0.0f);
    assert(p.pos == 9.0f && p.target == 9.0f);

    /* The negative move mirrors it exactly. */
    MotionProfile q = fresh(0.0f, 0.001f, S);
    assert(MotionProfile_setTarget(&q, -9.0f));
    const Run s = run_to_done(&q, 5000);
    assert(s.calls == r.calls && s.peak == r.peak);
    assert(q.pos == -9.0f && q.vel == 0.0f);

    printf("ok trapezoid\n");
}

/* Test 3: up ramp 0.75 s, down ramp 0.1667 s. */
static void test_asymmetric(void)
{
    MotionProfile p = fresh(0.0f, 0.001f, (MotionSettings){ 15, 20, 90, 0 });
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

/* Test 5: n moves from rest, start within +-start_span, distance within +-dist_span,
 * never passing and landing exactly; jerk_scale > 0 sets jerk 1.01-20x the smallest. */
static void random_moves(int n, float start_span, float dist_span, float jerk_scale)
{
    for (int i = 0; i < n; i++) {
        MotionSettings S = { frand(1, 50), frand(1, 200), frand(1, 200), 0 };
        if (jerk_scale > 0.0f)
            S.jerk = jerk_scale * MotionProfile_minJerk(S.accel, S.decel, 0.001f) * frand(1.01f, 20);
        float start = frand(-start_span, start_span);
        float d = frand(-dist_span, dist_span);
        if (fabsf(d) < 0.001f)
            d = copysignf(0.001f, d);
        if (i % 10 == 0)                 /* target 0, where an ulp of pos is finest */
            start = -d;
        MotionProfile p = fresh(start, 0.001f, S);
        assert(MotionProfile_setTarget(&p, i % 10 == 0 ? 0.0f : start + d));
        const float sense = p.target - p.pos;
        for (int calls = 0; !p.done; calls++) {
            assert(calls < 10000000);    /* 5000 units at speed 1, plus ramps */
            step(&p);
            assert((p.target - p.pos) * sense >= 0.0f);
        }
        assert(p.pos == p.target && p.vel == 0.0f);
    }
}

static void test_random_moves(float jerk_scale)
{
    random_moves(1000, 100.0f, 200.0f, jerk_scale);
    printf("ok random_moves%s\n", jerk_scale > 0.0f ? " (jerk)" : "");
}

/* Large coordinates, as a virtual master axis in degrees: thousands of units
 * with small decel, where uncompensated rounding of p_t passed targets. */
static void test_random_moves_large(float jerk_scale)
{
    random_moves(300, 5000.0f, 5000.0f, jerk_scale);
    printf("ok random_moves_large%s\n", jerk_scale > 0.0f ? " (jerk)" : "");
}

static int sign(float v) { return (v > 0.0f) - (v < 0.0f); }

/* Test 5: a target behind the axis costs exactly one reversal. */
static void test_retarget_behind(float jerk)
{
    MotionProfile p = fresh(0.0f, 0.001f, (MotionSettings){ 15, 30, 30, jerk });
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
    printf("ok retarget_behind%s\n", jerk > 0.0f ? " (jerk)" : "");
}

/* Test 5: a raised speed applies at once, at accel. */
static void test_speed_raise(void)
{
    MotionProfile p = fresh(0.0f, 0.001f, (MotionSettings){ 10, 30, 30, 0 });
    assert(MotionProfile_setTarget(&p, 1000.0f));
    run_until_vel(&p, 10.0f, 10000);
    assert(MotionProfile_setSettings(&p, (MotionSettings){ 20, 30, 30, 0 }));
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

/* Test 9: a target inside the stopping distance is passed, then reached:
 * one reversal, forward to back, and the return does not pass it. */
static void test_infeasible(float jerk)
{
    MotionProfile p = fresh(0.0f, 0.001f, (MotionSettings){ 15, 30, 30, jerk });
    assert(MotionProfile_setTarget(&p, 100.0f));
    run_until_vel(&p, 15.0f, 10000);
    assert(MotionProfile_setTarget(&p, p.pos + 1.0f));
    float furthest = -INFINITY;
    int last = 1, changes = 0;
    for (int calls = 0; !p.done; calls++) {
        assert(calls < 10000);
        step(&p);
        furthest = fmaxf(furthest, p.pos - p.target);
        const int s = sign(p.vel);
        if (s != 0 && s != last) {
            changes++;
            last = s;
        }
        if (last < 0)
            assert(p.pos >= p.target);
    }
    assert(changes == 1 && last < 0);
    assert(furthest >= 2.7f);            /* stopping distance 3.75, target 1 ahead */
    assert(p.pos == p.target);
    printf("ok infeasible%s\n", jerk > 0.0f ? " (jerk)" : "");
}

/* n calls; asserts the position never jumps more than speed*dt in one call.
 * Returns whether vel equalled want on any call. */
static bool run_endless(MotionProfile* p, int n, float want)
{
    bool reached = false;
    for (int i = 0; i < n; i++) {
        const float pos0 = p->pos;
        step(p);
        assert(fabsf(p->pos - pos0) <= p->settings.speed * p->dt * (1.0f + 1e-4f));
        reached |= p->vel == want;
    }
    return reached;
}

/* Test 7: endless speed changes and a stop, position continuous throughout. */
static void test_endless(void)
{
    MotionProfile p = fresh(0.0f, 0.001f, (MotionSettings){ 20, 30, 60, 0 });
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

    /* An endless speed above the speed setting runs at the setting (D7). */
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
    assert(fabsf(p->pos - (at + d)) <= p->settings.speed * p->dt);
    assert(p->target == p->pos && p->vel == 0.0f);
}

/* Test 6: stop at cruise and halfway up the ramp, where acc is not zero. */
static void test_stop(float jerk)
{
    const MotionSettings S = { 15, 30, 30, jerk };
    MotionProfile p = fresh(0.0f, 0.001f, S);
    assert(MotionProfile_setTarget(&p, 1000.0f));
    run_until_vel(&p, 15.0f, 10000);
    stop_where_predicted(&p);

    MotionProfile q = fresh(0.0f, 0.001f, S);
    assert(MotionProfile_setTarget(&q, 1000.0f));
    for (int calls = 0; q.vel < 7.5f; calls++) {
        assert(calls < 10000);
        step(&q);
    }
    assert(q.acc != 0.0f);
    stop_where_predicted(&q);

    /* stop on a done profile is a no-op */
    const MotionProfile before = q;
    MotionProfile_stop(&q);
    assert(profiles_equal(&before, &q));
    printf("ok stop%s\n", jerk > 0.0f ? " (jerk)" : "");
}

/* Test 6: halt is immediate and final. */
static void test_halt(void)
{
    MotionProfile p = fresh(0.0f, 0.001f, (MotionSettings){ 15, 30, 30, 0 });
    assert(MotionProfile_setTarget(&p, 1000.0f));
    run_until_vel(&p, 15.0f, 10000);
    MotionProfile_halt(&p);
    assert(p.vel == 0.0f && p.acc == 0.0f && p.done && p.target == p.pos);
    const MotionProfile before = p;
    MotionProfile_advance(&p);
    assert(profiles_equal(&before, &p));
    printf("ok halt\n");
}

/* Test 8: a lowered speed is followed down at exactly decel (with jerk, smoothly
 * under the jerk bound the checker asserts), then held. */
static void test_lowered_speed(float jerk)
{
    MotionProfile p = fresh(0.0f, 0.001f, (MotionSettings){ 15, 30, 30, jerk });
    assert(MotionProfile_setTarget(&p, 1000.0f));
    run_until_vel(&p, 15.0f, 10000);
    g_speed_relaxed = 1;
    assert(MotionProfile_setSettings(&p, (MotionSettings){ 5, 30, 30, jerk }));
    const float dv = 30.0f * 0.001f;
    for (int calls = 0; p.vel != 5.0f; calls++) {
        assert(calls < 10000);
        const float v0 = p.vel;
        step(&p);
        assert(p.vel <= v0);
        if (jerk > 0.0f)
            continue;
        if (p.vel != 5.0f)
            assert(fabsf((v0 - p.vel) - dv) <= dv * 1e-4f);
        else                                 /* the last step only closes the gap */
            assert(v0 - p.vel > 0.0f && v0 - p.vel <= dv * (1.0f + 1e-4f));
    }
    g_speed_relaxed = 0;
    for (int i = 0; i < 100; i++) {
        step(&p);
        assert(p.vel == 5.0f);
    }
    printf("ok lowered_speed%s\n", jerk > 0.0f ? " (jerk)" : "");
}

/* Test 8: a lowered accel applies from the next call. With jerk the ring still
 * carries the old ramp, so acc comes inside the new bound within one window. */
static void test_lowered_accel(float jerk)
{
    MotionProfile p = fresh(0.0f, 0.001f, (MotionSettings){ 15, 30, 30, jerk });
    assert(MotionProfile_setTarget(&p, 1000.0f));
    for (int i = 0; i < 100; i++)
        step(&p);
    assert(MotionProfile_setSettings(&p, (MotionSettings){ 15, 10, 30, jerk }));
    if (jerk > 0.0f) {
        g_accel_carried = 30.0f;
        int calls = 0;     /* the checker's acc rounding: two velocity ulps over dt */
        for (; p.acc > 10.0f * (1.0f + 1e-4f) + 2.0f * 15.0f * FLT_EPSILON / p.dt; calls++) {
            assert(calls <= p.window);
            step(&p);
        }
        assert(calls > 0);                   /* it was outside the new bound */
        g_accel_carried = 0.0f;
    } else {
        const float v0 = p.vel;
        step(&p);
        assert(fabsf((p.vel - v0) - 10.0f * 0.001f) <= 10.0f * 0.001f * 1e-4f);
    }
    for (int calls = 0; p.vel != 15.0f; calls++) {
        assert(calls < 10000);
        const float v0 = p.vel;
        step(&p);
        assert(p.vel - v0 <= 10.0f * 0.001f * (1.0f + 1e-4f) + 1e-6f + p.vel * FLT_EPSILON);
    }
    printf("ok lowered_accel%s\n", jerk > 0.0f ? " (jerk)" : "");
}

/* Test 4: with jerk, the move is the trapezoid's plus the window, and lands exactly. */
static void test_scurve(void)
{
    const MotionSettings S = { 15, 30, 30, 300 };
    MotionProfile p = fresh(0.0f, 0.001f, S);
    assert(p.window == 200);
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
        assert(p.acc <= S.accel * (1.0f + 1e-4f) && p.acc >= -S.decel * (1.0f + 1e-4f));
    }
    assert(calls - landed_call == 201);     /* n calls to drain, one more for acc to reach 0 */
    assert(p.pos == 9.0f && p.vel == 0.0f && p.acc == 0.0f);
    assert(peak <= 15.0f);
    /* The trapezoid's calls (1100 + extra_calls), plus the window's n - 1 calls of motion. */
    assert(fabsf((float)calls - (1100.0f + 200.0f + extra_calls(S, 0.001f) - 1.0f)) <= 2.0f);
    printf("ok scurve\n");
}

/* Test 4: with jerk = 0 the output is the trapezoid itself, bit for bit. */
static void test_trapezoid_bit_for_bit(void)
{
    MotionProfile p = fresh(0.0f, 0.001f, (MotionSettings){ 15, 30, 30, 0 });
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
    MotionProfile p = fresh(0.0f, 0.001f, (MotionSettings){ 20, 30, 30, 300 });
    assert(MotionProfile_setEndless(&p, 12.345f));
    for (long i = 0; i < 10000000L; i++)
        step(&p);
    const float ulp = nextafterf(12.345f, 20.0f) - 12.345f;
    assert(fabsf(p.vel - 12.345f) <= 200.0f * ulp);
    printf("ok sum_drift\n");
}

/* Test 1: the same move at dt 1 ms and 5 ms differs only by the discretisation. */
static void test_dt_independence(float jerk)
{
    const MotionSettings S = { 15, 30, 30, jerk };
    MotionProfile fine = fresh(0.0f, 0.001f, S), coarse = fresh(0.0f, 0.005f, S);
    /* The window adds n - 1 calls of motion: the done call is in extra_calls. */
    const float windows = fabsf((float)(fine.window - 1) * 0.001f - (float)(coarse.window - 1) * 0.005f);
    assert(MotionProfile_setTarget(&fine, 9.0f));
    assert(MotionProfile_setTarget(&coarse, 9.0f));
    const Run f = run_to_done(&fine, 10000), c = run_to_done(&coarse, 10000);
    assert(fine.pos == 9.0f && coarse.pos == 9.0f);
    /* Each run's known tail grows as dt*ln(1/dt); compare what remains. */
    const float t_fine = ((float)f.calls - extra_calls(S, 0.001f)) * 0.001f;
    const float t_coarse = ((float)c.calls - extra_calls(S, 0.005f)) * 0.005f;
    assert(fabsf(t_fine - t_coarse) <= 0.005f + windows);
    assert(fabsf(f.peak - c.peak) <= 30.0f * 0.005f);
    printf("ok dt_independence%s\n", jerk > 0.0f ? " (jerk)" : "");
}

/* Test 24: in flight the window is frozen; settings are accepted only while it
 * still bounds the jerk. */
static void test_frozen_window(void)
{
    const float dt = 0.001f;
    MotionProfile p = fresh(0.0f, dt, (MotionSettings){ 15, 30, 30, 290 });
    assert(p.window == 207);                       /* ceil(60 / 0.29) */
    assert(MotionProfile_setTarget(&p, 100.0f));
    for (int i = 0; i < 800; i++)             /* to cruise: 500 calls of ramp, 207 of window */
        step(&p);

    const struct { MotionSettings S; bool ok; } calls[] = {
        { { 15, 20, 30, 290 }, true },        /* lower accel */
        { { 15, 30, 20, 290 }, true },        /* lower decel */
        { { 15, 30, 30, 600 }, true },        /* raise jerk */
        { { 15, 30.01f, 30, 290 }, true },    /* inside the slack: 2 * 30.01 / 0.29 = 206.97 taps */
        { { 15, 30.02f, 30, 290 }, false },   /* just beyond it: 207.03 */
        { { 15, 30, 30, 250 }, false },       /* lower jerk: 240 */
        { { 15, 30, 30, 0 }, true },          /* jerk off; the window stays */
    };
    for (unsigned i = 0; i < sizeof calls / sizeof calls[0]; i++) {
        const MotionProfile before = p;
        assert(MotionProfile_setSettings(&p, calls[i].S) == calls[i].ok);
        if (calls[i].ok)
            assert(settings_equal(p.settings, calls[i].S) && p.window == 207);
        else
            assert(profiles_equal(&before, &p));
        for (int k = 0; k < 50; k++) {
            step(&p);
            assert(p.window == 207);
        }
    }

    /* Run to done. The window still bounds the jerk with the jerk setting now 0. */
    const float jerk = 2.0f * 30.0f / (207.0f * dt);
    for (int calls_ = 0; !p.done; calls_++) {
        assert(calls_ < 10000);
        const float a0 = p.acc;
        step(&p);
        if (!p.done) {
            assert(p.window == 207);
            assert(fabsf(p.acc - a0) <= jerk * dt * (1.0f + 1e-4f) + 4.0f * 15.0f * FLT_EPSILON / dt);
        }
    }
    assert(p.pos == 100.0f && p.window == 1);

    /* Done: the refused calls are accepted and n is recomputed. */
    assert(MotionProfile_setSettings(&p, (MotionSettings){ 15, 30.1f, 30, 290 }) && p.window == 208);
    assert(MotionProfile_setSettings(&p, (MotionSettings){ 15, 30, 30, 250 }) && p.window == 240);
    assert(MotionProfile_setTarget(&p, 60.0f));      /* a move on the new window */
    for (int calls_ = 0; !p.done; calls_++) {
        assert(calls_ < 10000);
        step(&p);
        assert(p.window == 240 || p.done);
    }
    assert(p.pos == 60.0f);
    {
        const MotionProfile before = p;       /* 300 taps: refused even when done */
        assert(!MotionProfile_setSettings(&p, (MotionSettings){ 15, 30, 30, 200 }));
        assert(profiles_equal(&before, &p));
    }

    /* Switching jerk on is refused in flight and accepted once done. */
    MotionProfile q = fresh(0.0f, dt, (MotionSettings){ 15, 30, 30, 0 });
    assert(MotionProfile_setTarget(&q, 5.0f));
    for (int i = 0; i < 100; i++)
        step(&q);
    {
        const MotionProfile before = q;
        assert(!MotionProfile_setSettings(&q, (MotionSettings){ 15, 30, 30, 300 }));
        assert(profiles_equal(&before, &q));
    }
    run_to_done(&q, 10000);
    assert(MotionProfile_setSettings(&q, (MotionSettings){ 15, 30, 30, 300 }) && q.window == 200);
    printf("ok frozen_window\n");
}

/* D1, D2: a profile re-based by whole units every 10 calls follows the same
 * path as one that is not, a whole number of units lower. */
static void test_rebase(void)
{
    const MotionSettings S = { 15, 30, 30, 300 };
    MotionProfile a = fresh(0.0f, 0.001f, S), b = fresh(0.0f, 0.001f, S);
    assert(MotionProfile_setTarget(&a, 50.0f));
    assert(MotionProfile_setTarget(&b, 50.0f));
    int k = 0;
    for (int calls = 1; !a.done || !b.done; calls++) {
        assert(calls < 10000);
        if (!a.done)
            step(&a);
        if (!b.done)
            step(&b);
        if (calls % 10 == 0) {
            k++;
            MotionProfile_rebase(&b, 1.0f, 50.0f - (float)k);
        }
        assert(fabsf((b.pos + (float)k) - a.pos) <= 1e-4f);
    }
    assert(a.pos == 50.0f && a.vel == 0.0f);
    assert(b.pos == 50.0f - (float)k && b.target == b.pos && b.vel == 0.0f);

    /* Landed, the target moves with the frame and the argument is ignored. */
    MotionProfile_rebase(&b, 2.0f, 999.0f);
    assert(b.pos == 48.0f - (float)k && b.target == b.pos && b.done);

    /* Endless, likewise; the speed is untouched. */
    MotionProfile e = fresh(0.0f, 0.001f, S);
    assert(MotionProfile_setEndless(&e, 5.0f));
    for (int i = 0; i < 300; i++)
        step(&e);
    const float pos0 = e.pos, target0 = e.target;
    MotionProfile_rebase(&e, 1.0f, 999.0f);
    assert(fabsf(e.pos - (pos0 - 1.0f)) <= 1e-6f && e.target == target0 - 1.0f);
    assert(e.endless && e.endless_speed == 5.0f);

    /* In flight, the caller's target is taken, not target - shift. */
    MotionProfile c = fresh(0.0f, 0.001f, S);
    assert(MotionProfile_setTarget(&c, 50.0f));
    for (int i = 0; i < 300; i++)
        step(&c);
    MotionProfile_rebase(&c, 3.0f, 20.0f);
    assert(c.target == 20.0f);
    run_to_done(&c, 10000);
    assert(c.pos == 20.0f && c.vel == 0.0f);
    printf("ok rebase\n");
}

/* A new target while the ring drains goes on from the ring as it stands; the
 * same target again replays the rest of the drain. */
static void test_retarget_draining(void)
{
    const MotionSettings S = { 15, 30, 30, 300 };
    MotionProfile x = fresh(0.0f, 0.001f, S);
    assert(MotionProfile_setTarget(&x, 9.0f));
    while (!(x.landed && x.drain == 100))
        step(&x);
    MotionProfile y = x;
    assert(MotionProfile_setTarget(&y, 9.0f));
    while (!x.done) {
        step(&x);
        step(&y);
        assert(fabsf(y.vel - x.vel) <= 1e-5f && fabsf(y.pos - x.pos) <= 1e-5f);
    }
    run_to_done(&y, 10000);
    assert(y.pos == 9.0f);

    /* Elsewhere, and endless then stopped: continuous, and lands exactly. */
    MotionProfile z = fresh(0.0f, 0.001f, S);
    assert(MotionProfile_setTarget(&z, 9.0f));
    while (!(z.landed && z.drain == 150))
        step(&z);
    MotionProfile w = z;
    assert(MotionProfile_setTarget(&z, 4.0f));
    run_to_done(&z, 10000);
    assert(z.pos == 4.0f);
    assert(MotionProfile_setEndless(&w, -3.0f));
    for (int i = 0; i < 1000; i++) {
        const float pos0 = w.pos;
        step(&w);
        assert(fabsf(w.pos - pos0) <= S.speed * w.dt);
    }
    MotionProfile_stop(&w);
    run_to_done(&w, 10000);
    assert(w.vel == 0.0f && w.pos == w.target);
    printf("ok retarget_draining\n");
}

/* A new course on the call after landing starts from the landing step's velocity,
 * not from rest: reversing there keeps every bound (the axis does it on arrival). */
static void test_course_after_landing(float jerk)
{
    const MotionSettings S = { 15, 30, 30, jerk };
    int sizable = 0;
    for (int k = 0; k < 40; k++) {
        const float to = 9.0f + 0.0371f * (float)k;
        MotionProfile p = fresh(0.0f, 0.001f, S);
        assert(MotionProfile_setTarget(&p, to));
        while (!p.landed)
            step(&p);
        const float u = p.taps[(p.head + p.window - 1u) % p.window];   /* the landing step */
        if (u > 0.5f * S.decel * p.dt)
            sizable++;
        /* landed: nothing left to stop but the ring's lag, though v_t still holds u */
        assert(fabsf(MotionProfile_stopDistance(&p) - (p.p_t - p.pos)) <= 1e-5f);
        if (jerk == 0.0f)
            assert(MotionProfile_stopDistance(&p) == 0.0f && p.v_t == u);
        MotionProfile q = p, r = p;
        assert(MotionProfile_setEndless(&p, k % 2 ? -15.0f : 15.0f));
        assert(MotionProfile_setTarget(&q, to - 1.0f));
        assert(MotionProfile_setTarget(&r, to + 1.0f));
        for (int i = 0; i < 300; i++) {
            step(&p);
            step(&q);
            step(&r);
        }
        MotionProfile_stop(&p);
        run_to_done(&p, 10000);
        run_to_done(&q, 10000);
        run_to_done(&r, 10000);
        assert(q.pos == to - 1.0f && r.pos == to + 1.0f);
    }
    assert(sizable >= 10);
    printf("ok course_after_landing\n");
}

/* A target at the position during a slow jog, so the ring lands with taps of both signs: the old
 * positive taps carry the output on past the target before it returns. Through the drain pos must
 * stay the integral of vel (E5) to rounding; a one-signed clamp there would hold pos still while
 * vel is not zero, an unfiltered stop. */
static void test_drain_mixed(void)
{
    const MotionSettings S = { 15, 30, 30, 300 };
    MotionProfile p = fresh(0.0f, 0.001f, S);
    assert(MotionProfile_setEndless(&p, 0.2f));
    run_until_vel(&p, 0.2f, 10000);
    for (int i = 0; i < 300; i++)
        step(&p);
    assert(MotionProfile_setTarget(&p, p.pos));
    int positive = 0;
    float worst = 0.0f, furthest = 0.0f;
    bool was_landed = false;
    for (int calls = 0; !p.done; calls++) {
        assert(calls < 10000);
        const float pos0 = p.pos;
        step(&p);
        const float err = fabsf((p.pos - pos0) - p.vel * p.dt);
        worst = fmaxf(worst, err);
        assert(err <= 1e-6f);                 /* measured 1e-8; the clamp on this ring gave 5e-4 */
        furthest = fmaxf(furthest, p.pos - p.target);
        if (p.landed && !was_landed) {        /* the ring as it lands: the taps are velocities */
            was_landed = true;
            for (uint16_t k = 0; k < p.window; k++)
                positive += p.taps[k] > 0.0f;
        }
    }
    assert(positive >= 100 && positive < p.window);   /* both signs, mostly the jog's */
    assert(furthest > 0.005f);                /* it went on past the target before returning */
    assert(p.pos == p.target && p.vel == 0.0f);
    printf("ok drain_mixed (%d of %u taps positive at landing, worst |dpos - vel dt| %.2e)\n",
           positive, (unsigned)p.window, (double)worst);
}

/* A reversal with accel well above decel (review, E31): the first ramp's +accel taps are still in
 * the ring when the reversal's -accel enters, one window apart, so the window must cover
 * 2 max(accel, decel), not accel + decel. The hold is sized so the two ramps meet exactly. */
static void test_reversal_asymmetric(void)
{
    const MotionSettings S = { 15, 90, 20, 720 };   /* 2 * 90 / 0.72 = 250 taps */
    MotionProfile p = fresh(0.0f, 0.001f, S);
    assert(MotionProfile_setEndless(&p, 1.0f));
    int up = 0;
    while (p.v_t != 1.0f) {                   /* the trapezoid's own ramp, through the step checker */
        step(&p);
        up++;
    }
    const int down = (int)ceilf(1.0f / (S.decel * p.dt));   /* 50 calls back to rest */
    for (int i = 0; i < (int)p.window - up - down; i++)
        step(&p);
    assert(MotionProfile_setEndless(&p, -1.0f));
    float peak = 0.0f;
    for (int i = 0; i < 2 * (int)p.window; i++) {
        const float a0 = p.acc;
        step(&p);
        peak = fmaxf(peak, fabsf(p.acc - a0) / p.dt);
    }
    assert(peak >= 0.9f * S.jerk);            /* the ramps did meet: the bound was exercised */
    assert(p.vel == -1.0f);
    printf("ok reversal_asymmetric (peak jerk %.1f of %.0f, n %u)\n", (double)peak, (double)S.jerk,
           (unsigned)p.window);
}

/* A live settings change meets a ring still holding motion under the old settings (review, E32): the
 * frozen window must bound a step from the largest acceleration retained to the largest the new
 * settings allow. Lowering accel and decel under the same jerk always fits. */
static void test_live_settings_history(void)
{
    const MotionSettings S = { 15, 30, 30, 300 };
    MotionProfile p = fresh(0.0f, 0.001f, S);
    assert(MotionProfile_setEndless(&p, 15.0f));
    for (int i = 0; i < 250; i++)
        step(&p);
    assert(p.acc > 29.0f);                    /* the ring holds the accel 30 ramp */
    {
        /* the reviewer's case: on the frozen window alone it was accepted, and the next jerk was 144 */
        const MotionProfile before = p;
        const MotionSettings soft = { 15, 1, 1, 10 };
        assert(MotionProfile_checkSettings(&p, soft) == MOTION_SETTINGS_WINDOW);
        assert(!MotionProfile_setSettings(&p, soft));
        assert(!MotionProfile_setSettings(&p, (MotionSettings){ 15, 1, 1, 150 }));   /* (1 + 30) / 0.15 = 206.7 taps */
        assert(profiles_equal(&before, &p));
    }
    /* a jerk the history allows, (1 + 30) / 0.16 = 193.8 taps: the bound holds through the turnover */
    assert(MotionProfile_setSettings(&p, (MotionSettings){ 15, 1, 1, 160 }));
    assert(p.hist_acc == 30.0f && p.hist_left == 200);
    g_accel_carried = 30.0f;
    float peak = 0.0f;
    for (int i = 0; i < 2 * (int)p.window; i++) {
        const float a0 = p.acc;
        step(&p);
        peak = fmaxf(peak, fabsf(p.acc - a0) / p.dt);
    }
    g_accel_carried = 0.0f;
    assert(peak >= 100.0f && p.hist_left == 0 && p.hist_acc == 0.0f);   /* the retained ramp did leave */

    /* lowering accel and decel alone; then every field at once, both ways, and beyond the slack */
    MotionProfile q = fresh(0.0f, 0.001f, S);
    assert(MotionProfile_setEndless(&q, 15.0f));
    for (int i = 0; i < 250; i++)
        step(&q);
    assert(MotionProfile_setSettings(&q, (MotionSettings){ 15, 1, 1, 300 }));
    assert(MotionProfile_setSettings(&q, (MotionSettings){ 10, 20, 25, 400 }));   /* (25 + 30) / 0.4 = 137.5 */
    g_accel_carried = 30.0f;
    for (int i = 0; i < 100; i++)
        step(&q);
    assert(!MotionProfile_setSettings(&q, (MotionSettings){ 20, 40, 35, 390 }));  /* (40 + 40) / 0.39 = 205 */
    assert(MotionProfile_setSettings(&q, (MotionSettings){ 20, 30, 30, 300 }));   /* (30 + 30) / 0.3 = 200 */
    for (int i = 0; i < 400; i++)
        step(&q);
    g_accel_carried = 0.0f;
    assert(q.hist_left == 0 && fabsf(q.vel - 15.0f) <= 1e-3f);   /* at the jog speed again */
    MotionProfile_stop(&q);
    run_to_done(&q, 10000);
    printf("ok live_settings_history (peak jerk %.1f of 160 through the turnover)\n", (double)peak);
}

/* A settings change on the landing call (review): v_t still holds the landing step (E16) and the next
 * call zeroes it, a step the new settings do not bound. Refused there; accepted one call later. */
static void test_settings_on_landing(void)
{
    const MotionSettings S = { 15, 30, 30, 300 }, soft = { 15, 1, 1, 160 };
    MotionProfile p = fresh(0.0f, 0.001f, S);
    assert(MotionProfile_setTarget(&p, 0.53725f));
    while (!p.landed)
        step(&p);
    assert(fabsf(p.v_t) >= 0.025f);           /* a landing step near decel dt = 0.03 */
    {
        const MotionProfile before = p;
        assert(MotionProfile_checkSettings(&p, soft) == MOTION_SETTINGS_WINDOW);   /* (30 + 30) / 0.16 = 375 taps */
        assert(!MotionProfile_setSettings(&p, soft) && profiles_equal(&before, &p));
    }
    step(&p);                                 /* the first drain call: v_t is 0 */
    assert(p.v_t == 0.0f && MotionProfile_setSettings(&p, soft));   /* (1 + 30) / 0.16 = 194 */
    g_accel_carried = 30.0f;                  /* the ring sheds the old ramp under the new settings */
    run_to_done(&p, 10000);
    g_accel_carried = 0.0f;
    assert(p.pos == 0.53725f && p.vel == 0.0f);
    printf("ok settings_on_landing\n");
}

/* D1: seeding the idle generator into motion, as the axis leaves a stream. */
static void test_seed(void)
{
    const float dt = 0.001f;
    MotionProfile p = fresh(0.0f, dt, (MotionSettings){ 20, 30, 30, 300 });
    assert(MotionProfile_seed(&p, 2.0f, 7.0f));
    assert(p.pos == 2.0f && p.vel == 7.0f && p.endless && !p.done);
    step(&p);
    assert(p.vel == 7.0f && p.acc == 0.0f);
    {
        const float want = 2.0f + 7.0f * dt;
        assert(fabsf(p.pos - want) <= nextafterf(want, INFINITY) - want);
    }
    const float d = MotionProfile_stopDistance(&p);
    MotionProfile_stop(&p);
    run_to_done(&p, 10000);
    assert(fabsf(p.pos - (2.0f + 7.0f * dt + d)) <= 20.0f * dt);

    /* Refused in motion, and for a non-finite pos or vel; nothing changes. */
    MotionProfile q = fresh(0.0f, dt, (MotionSettings){ 20, 30, 30, 300 });
    assert(MotionProfile_setTarget(&q, 10.0f));
    step(&q);
    {
        const MotionProfile before = q;
        assert(!MotionProfile_seed(&q, 0.0f, 1.0f));
        assert(profiles_equal(&before, &q));
    }
    {
        const MotionProfile before = p;
        assert(!MotionProfile_seed(&p, NAN, 1.0f));
        assert(!MotionProfile_seed(&p, 1.0f, INFINITY));
        assert(profiles_equal(&before, &p));
    }
    printf("ok seed\n");
}

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);   /* keep the ok lines when an assert aborts */
    test_setup();
    test_trapezoid();
    test_asymmetric();
    test_speed_raise();
    test_endless();
    test_halt();
    test_trapezoid_bit_for_bit();
    test_scurve();
    test_sum_drift();
    /* Each with the trapezoid alone, then through the jerk filter. */
    for (int j = 0; j < 2; j++) {
        const float jerk = j ? 300.0f : 0.0f;
        test_random_moves((float)j);
        test_random_moves_large((float)j);
        test_retarget_behind(jerk);
        test_infeasible(jerk);
        test_stop(jerk);
        test_lowered_speed(jerk);
        test_lowered_accel(jerk);
        test_dt_independence(jerk);
        test_course_after_landing(jerk);
    }
    test_frozen_window();
    test_rebase();
    test_retarget_draining();
    test_drain_mixed();
    test_reversal_asymmetric();
    test_live_settings_history();
    test_settings_on_landing();
    test_seed();
    return 0;
}
