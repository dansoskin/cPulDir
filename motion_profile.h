/* Motion profile generator: an acceleration-limited online trapezoid through a moving-average
 * jerk filter. Pure maths, no statics, unit-agnostic (guidelines 5.2). */
#ifndef MOTION_PROFILE_H
#define MOTION_PROFILE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MOTION_PROFILE_MAX_TAPS
#define MOTION_PROFILE_MAX_TAPS 256       /* raise for softer jerk */
#endif

typedef struct {
    float vmax, amax, dmax, jmax;         /* jmax 0 = unlimited (trapezoid) */
} MotionLimits;

typedef struct {
    float dt;                             /* seconds per advance, fixed at init */
    float pos, vel, acc;                  /* the output: filtered, caller's units */
    float target;
    bool  endless;                        /* run at a speed, no target */
    bool  stopping;                       /* stop as soon as the ramps allow */
    bool  landed;                         /* the trapezoid is at rest; the ring may still drain */
    bool  done;                           /* at target with vel == 0.0f exactly */
    MotionLimits lim;
    /* private */
    float    p_t, p_c, v_t;               /* the trapezoid's own position, its Kahan compensation, velocity */
    float    speed;                       /* endless speed, signed */
    float    taps[MOTION_PROFILE_MAX_TAPS];   /* the trapezoid's velocities; zeros once it has landed */
    uint16_t n, head;                     /* window length; next slot to overwrite */
    uint16_t drain;                       /* calls since landing, up to n + 1 */
    uint16_t run;                         /* the newest taps sharing a sign, zeros included, at most n */
    int8_t   run_sign;                    /* that sign; 0 while the run holds only zeros */
    float    hist_acc;                    /* the largest amax or dmax of limits replaced in flight whose
                                             motion the ring may still hold (E32) */
    uint16_t hist_left;                   /* calls until the ring has turned over since; 0: no history */
    float    sum, sum_c;                  /* running sum of the ring, its Kahan compensation */
    float    wsum, wsum_c;                /* the ring weighted n-1 (newest) down to 0; pos = p_t - wsum*dt/n */
} MotionProfile;

typedef enum { MOTION_LIMITS_OK, MOTION_LIMITS_INVALID, MOTION_LIMITS_TAPS, MOTION_LIMITS_WINDOW } MotionLimitsCheck;

bool  MotionProfile_init(MotionProfile*, float pos, float dt);
/* What setLimits would answer for this profile. In flight its window is frozen and must also bound
 * the acceleration its ring may still hold from the limits being replaced (E32). */
MotionLimitsCheck MotionProfile_checkLimits(const MotionProfile*, MotionLimits);
bool  MotionProfile_setLimits(MotionProfile*, MotionLimits);
bool  MotionProfile_setTarget(MotionProfile*, float target);
bool  MotionProfile_setEndless(MotionProfile*, float speed);
void  MotionProfile_stop(MotionProfile*);
void  MotionProfile_halt(MotionProfile*);
bool  MotionProfile_setPosition(MotionProfile*, float pos);
void  MotionProfile_advance(MotionProfile*);
float MotionProfile_stopDistance(const MotionProfile*);
/* For the axis (D1): shift the origin by a whole number, set the target relative to it. */
void  MotionProfile_rebase(MotionProfile*, float shift, float target);
/* For the axis (D1): from rest, put the profile in motion at vel through pos, ring full of vel. */
bool  MotionProfile_seed(MotionProfile*, float pos, float vel);
/* The smallest jerk whose window fits MOTION_PROFILE_MAX_TAPS, as the plain quotient: it can be
 * one ulp under the exact boundary, so a refusal message prints it rounded up. */
float MotionProfile_minJerk(float amax, float dmax, float dt);

#ifdef __cplusplus
}
#endif
#endif
