/* Generator tests (guidelines section 8, tests 1-9). One executable; each
 * test prints "ok <name>". Every advance goes through step(), which checks
 * the limits of 5.2, so every test checks them for free. */
#include <assert.h>
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
    if (!g_vmax_relaxed)
        assert(fabsf(p->vel) <= L.vmax * (1.0f + tol));
    assert(fabsf(dv) <= bound * p->dt * (1.0f + tol) + 1e-6f);
    if (L.jmax > 0.0f)
        assert(fabsf(p->acc - acc0) <= L.jmax * p->dt * (1.0f + tol) + 1e-3f);
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
        assert(memcmp(&q, &p, sizeof q) == 0);
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

int main(void)
{
    test_setup();
    return 0;
}
