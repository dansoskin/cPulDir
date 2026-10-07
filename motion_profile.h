/* Motion profile generator: an acceleration-limited online trapezoid whose
 * velocity passes through a moving-average jerk filter. Pure maths, no
 * statics, unit-agnostic. Section 5.2 of docs/puldir-guidelines.md. */
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
    float    p_t, v_t;                    /* the trapezoid's own position and velocity */
    float    speed;                       /* endless speed, signed */
    float    taps[MOTION_PROFILE_MAX_TAPS];
    uint16_t n, head;                     /* window length; next slot to overwrite */
    uint16_t drain;                       /* calls since landing, up to n */
    float    sum;
} MotionProfile;

bool  MotionProfile_init(MotionProfile*, float pos, float dt);
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
/* The smallest jerk whose window fits MOTION_PROFILE_MAX_TAPS, for refusal messages. */
float MotionProfile_minJerk(float amax, float dmax, float dt);

#ifdef __cplusplus
}
#endif
#endif
