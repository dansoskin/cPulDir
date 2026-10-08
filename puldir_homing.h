/* Homing (5.8): cFlexyStepper's phases, run by the plan tick. PulDir embeds the struct; the
 * functions below it are the library's own, seen through puldir_internal.h (D20). */
#ifndef PULDIR_HOMING_H
#define PULDIR_HOMING_H
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    PULDIR_HOME_IDLE, PULDIR_HOME_MOVE_TOWARDS_LIMIT, PULDIR_HOME_DELAY1,
    PULDIR_HOME_MOVE_AWAY_FROM_LIMIT, PULDIR_HOME_DELAY2,
    PULDIR_HOME_ADJUST_POSITION, PULDIR_HOME_DELAY3, PULDIR_HOME_ERROR
} PulDirHomeState;

typedef struct {
    int8_t   dir;                        /* +1, -1, 0 = not configured */
    float    speed, adjust, zero;        /* user units */
    const uint8_t* switch_ptr;
    uint32_t timeout_ticks;              /* setHoming's phase timeout */
    volatile PulDirHomeState state;
    PulDirHomeState fail;                /* the phase the last failed run was in */
    uint32_t t0;                         /* the tick the current phase began */
    float    saved_lim[4];               /* the limits at homeStart: a run that ends unhomed restores them */
    volatile bool homed;
} PulDirHoming;
#endif

#if defined(PULDIR_INTERNAL_FUNCTIONS) && !defined(PULDIR_HOMING_FUNCTIONS)
#define PULDIR_HOMING_FUNCTIONS
/* Under the lock, from main or the plan tick, a run ending unhomed: the limits homeStart found go
 * back. The plan tick hands them to the generator once its window allows. */
void puldir_home_restore(PulDir*);
/* Main context, under the lock: a stop or halt ended the run, and the homing state goes IDLE. */
void puldir_home_cancel(PulDir*);
/* Main context, under the lock: setPosition moved the datum, so the axis is no longer homed. */
void puldir_home_lose_datum(PulDir*);
/* Plan tick: the enable delay holds the axis, so the phase clock starts when the hold ends. */
void puldir_home_hold(PulDir*, uint32_t now);
/* Plan tick, on a fault latch: the datum is lost, and a run in progress fails. */
void puldir_home_fault(PulDir*);
/* Plan tick, homeStart's request in HOMING: the run sets off toward the switch. */
void puldir_home_begin(PulDir*, uint32_t now);
/* Plan tick: one interval of HOMING (base, pb: 32.32, as run_axis has them). */
void puldir_home_run(PulDir*, PulDirInc* out, uint64_t base, int64_t pb, int32_t count,
                     int32_t live, bool pul_high, uint32_t now);
#endif
