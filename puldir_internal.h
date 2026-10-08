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
    int8_t   dir;                             /* the pin's current direction, not the last published
                                                 one: the plan tick keeps that per axis (dir_out) */
} PulDirPulse;

/* main -> plan requests (D17): set under the lock, applied by the next plan tick */
enum {
    REQ_LIMITS = 1u << 0, REQ_TARGET = 1u << 1, REQ_ENDLESS = 1u << 2, REQ_STOP = 1u << 3,
    REQ_HALT = 1u << 4, REQ_SETPOS = 1u << 5, REQ_STREAM_BEGIN = 1u << 6,
    REQ_HOME = 1u << 8
};
/* The motion requests: a newer one replaces any older one still pending, so
 * the plan tick applies only the latest (a halt is kept: it comes first). */
#define REQ_MOTION (REQ_TARGET | REQ_ENDLESS | REQ_STOP)
/* Every request a halt discards. */
#define REQ_COURSE (REQ_MOTION | REQ_STREAM_BEGIN | REQ_HOME)

/* A duration in pulse ticks; every time is compared by unsigned subtraction (5.9). */
#define PULDIR_MS_TICKS(ms) ((uint32_t)((uint64_t)(ms) * PULDIR_TICK_HZ / 1000u))
/* The longest delay or timeout a setter accepts, in ticks: half the counter. */
#define PULDIR_TICKS_MAX 0x7FFFFFFFu

/* main -> plan requests on a group */
enum { GROUP_REQ_STOP = 1u << 0 };

/* One axis at the plan tick's instant, read under one lock with every other. */
typedef struct {
    uint64_t Pb;                              /* the pulse position predicted at the next boundary (D5, E11) */
    int32_t  count;                           /* edges emitted */
    int32_t  live;                            /* the running increment */
    int8_t   dir;                             /* the direction the pin shows */
    bool     high;                            /* the pulse pin is high */
} PulDirSnap;

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
/* Under the lock: ticks left in this interval, N when the next tick is the
 * boundary that latches. The live inc runs left % N more ticks. */
uint32_t   puldir_left(void);
void       puldir_pulse_read(uint8_t slot, PulDirPulse* out);   /* under the lock */
bool       puldir_pul_high(uint8_t slot);     /* under the lock */
/* Under the lock: count, P and edge to a whole pulse. The caller guarantees the
 * axis is at rest, its live inc 0 and its pulse pin low. */
void       puldir_set_datum(uint8_t slot, int32_t count);

/* plan tick (puldir_plan.c) */
void puldir_plan_tick(void);
/* Write one entry of the set being built. A non-zero inc carries its sign as the
 * direction; zero keeps the last one, so a pause never turns the pin. */
void puldir_set_inc(PulDir*, PulDirInc* out, int32_t inc);
/* One interval of an axis or a group member heading for T. T, end (this
 * interval's aim) and pb are 32.32 relative to the axis's whole-pulse base.
 * Before arrival: D6 holds end M short of T when hold. arrival: the generator has
 * landed with end on T; the final increment is published, rounded away (E13).
 * After: settle steering. Returns true once settling has elapsed. */
bool puldir_steer(PulDir*, PulDirInc* out, int64_t T, int64_t end, int64_t pb, bool arrival, bool hold);

/* groups (puldir_group.c) */
void puldir_group_tick(const PulDirSnap* snap, PulDirInc* out);   /* every group, before the axes */
/* Under the lock, from main or from inside the plan tick: no rising edge on any
 * member after this tick, every member IDLE, the move abandoned. shadow is the set
 * the plan tick is writing, whose members' entries are zeroed too; NULL from main. */
void puldir_group_halt(PulDirGroup*, PulDirInc* shadow);

/* Main context, under the lock, for every accepted motion command (5.8): enables a
 * disabled drive (the plan tick then holds the motion for enable_ms) and cancels
 * a pending auto-release. A few stores and one pin write. */
void puldir_wake(PulDir*);

/* events (puldir_loop.c) */
void puldir_refuse(PulDir*, const char* call, PulDirRefusal why, float value);
#endif
