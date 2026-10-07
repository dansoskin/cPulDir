/* Shared between the library's own files only (D20). */
#ifndef PULDIR_INTERNAL_H
#define PULDIR_INTERNAL_H
#include "puldir.h"

#define PULDIR_ONE     ((uint64_t)1 << 32)    /* one pulse in 32.32 */
#define PULDIR_INC_MAX 0x7FFFFFFF             /* just under half a pulse per tick */

typedef struct { int32_t inc; int8_t dir; } PulDirInc;   /* D3: dir is +1 or -1 */

/* event bits */
enum {
    PULDIR_EV_REFUSED = 1u << 0, PULDIR_EV_LANDED = 1u << 1, PULDIR_EV_STREAM_TIMEOUT = 1u << 2,
    PULDIR_EV_HOMED = 1u << 3, PULDIR_EV_HOME_FAILED = 1u << 4, PULDIR_EV_FAULT = 1u << 5,
    PULDIR_EV_FAULT_CLEARED = 1u << 6, PULDIR_EV_DROPPED = 1u << 7
};

extern PulDir* puldir_axes[PULDIR_MAX_AXES];
extern uint8_t puldir_naxes;

/* pulse tick (puldir_pulse.c) */
void       puldir_pulse_tick(bool overrun);
PulDirInc* puldir_shadow(void);               /* the unpublished set; the plan tick writes it */
void       puldir_publish(void);              /* one store; latched at the next boundary (D5) */
void       puldir_kill(uint8_t slot);         /* under the lock: zero live and published inc */
uint32_t   puldir_left(void);                 /* under the lock: ticks left in this interval */
int32_t    puldir_live_inc(uint8_t slot);     /* under the lock */

/* plan tick (puldir_plan.c) */
void puldir_plan_tick(void);

/* events (puldir_loop.c) */
void puldir_refuse(PulDir*, const char* call, PulDirRefusal why, float value);
#endif
