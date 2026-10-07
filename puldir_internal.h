/* Shared between the library's own files only (D20). */
#ifndef PULDIR_INTERNAL_H
#define PULDIR_INTERNAL_H
#include "puldir.h"

#define PULDIR_ONE     ((uint64_t)1 << 32)    /* one pulse in 32.32 */
#define PULDIR_INC_MAX 0x7FFFFFFF             /* just under half a pulse per tick */

typedef struct { int32_t inc; int8_t dir; } PulDirInc;   /* D3: dir is +1 or -1 */

/* The pulse tick's state for one slot, read under the lock. */
typedef struct {
    uint64_t P;                               /* 32.32 pulse position */
    int32_t  count;                           /* edges emitted, signed (D18) */
    int32_t  inc;                             /* live increment */
    int8_t   dir;                             /* the direction the pin shows */
} PulDirPulse;

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
void       puldir_pulse_add(PulDir*);         /* PulDir_register: take the next slot, entries at inc 0, dir +1 */
PulDirInc* puldir_shadow(void);               /* the unpublished set; the plan tick writes it */
/* One store; latched at the next boundary (D5). Every entry with a non-zero
 * inc must have dir equal to its sign: the plan tick guarantees it. */
void       puldir_publish(void);
void       puldir_kill(uint8_t slot);         /* under the lock: zero live and published inc */
uint32_t   puldir_left(void);                 /* under the lock: ticks left in this interval */
void       puldir_pulse_read(uint8_t slot, PulDirPulse* out);   /* under the lock */
bool       puldir_pul_high(uint8_t slot);     /* under the lock */
void       puldir_set_datum(uint8_t slot, int32_t count);       /* under the lock, idle: count, P, edge */

/* plan tick (puldir_plan.c) */
void puldir_plan_tick(void);

/* events (puldir_loop.c) */
void puldir_refuse(PulDir*, const char* call, PulDirRefusal why, float value);
#endif
