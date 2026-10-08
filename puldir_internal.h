/* Shared between the library's own files only (D20). */
#ifndef PULDIR_INTERNAL_H
#define PULDIR_INTERNAL_H
#include "puldir.h"

#define PULDIR_ONE     ((uint64_t)1 << 32)    /* one pulse in 32.32 */
#define PULDIR_FIX     4294967296.0f          /* the same, as a float scale */
#define PULDIR_PULSES_MAX 2147483648.0f       /* whole-pulse targets lie in [-2^31, 2^31) */
#define PULDIR_INC_MAX 0x7FFFFFFF             /* just under half a pulse per tick */

typedef struct { int32_t inc; int8_t dir; } PulDirInc;   /* D3: dir is +1 or -1 */

/* One axis's pulse state, read under the lock: the plan tick takes every axis's at one instant. */
typedef struct {
    uint64_t Pb;                              /* the 32.32 pulse position, predicted ahead ticks on (D5, E11) */
    int32_t  count;                           /* edges emitted, signed (D18) */
    int32_t  inc;                             /* the live increment */
    int8_t   dir;                             /* the direction the pin shows, not the last published
                                                 one: the plan tick keeps that per axis (dir_out) */
    bool     high;                            /* the pulse pin is high */
} PulDirSnap;

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

/* The timing, from the port's attach calls (puldir_pulse.c writes it, everyone else reads it).
 * Fixed from the first PulDir_init or PulDir_start on, so the ticks see constants. */
typedef struct {
    uint32_t pulse_us, plan_us;               /* 0 until the port hands them over */
    uint32_t tick_hz;                         /* 1e6 / pulse_us */
    uint32_t n;                               /* pulse ticks per plan interval, plan_us / pulse_us */
    float    dt;                              /* the plan interval in s: every profile's dt */
    bool     set;                             /* the pair passed PulDir_init's check */
    bool     started;                         /* PulDir_start has run */
} PulDirTiming;
extern PulDirTiming puldir_timing;
bool puldir_timing_take(void);                /* PulDir_init: checks the pair and fixes it; true once set */

/* A duration in pulse ticks; every time is compared by unsigned subtraction (5.9). */
uint32_t puldir_ms_ticks(uint32_t ms);
/* The longest delay or timeout a setter accepts, in ticks: half the counter. */
#define PULDIR_TICKS_MAX 0x7FFFFFFFu

/* main -> plan requests on a group */
enum { GROUP_REQ_STOP = 1u << 0 };

/* event bits */
enum {
    PULDIR_EV_REFUSED = 1u << 0, PULDIR_EV_LANDED = 1u << 1, PULDIR_EV_STREAM_TIMEOUT = 1u << 2,
    PULDIR_EV_HOMED = 1u << 3, PULDIR_EV_HOME_FAILED = 1u << 4, PULDIR_EV_FAULT = 1u << 5,
    PULDIR_EV_FAULT_CLEARED = 1u << 6
};

extern PulDir* puldir_axes[PULDIR_MAX_AXES];
extern uint8_t puldir_naxes;

/* pulse tick (puldir_pulse.c) */
void       puldir_pulse_tick(bool overrun);
void       puldir_pulse_add(PulDir*);         /* PulDir_register: take the next slot, entries at inc 0, dir +1 */
PulDirInc* puldir_shadow(void);               /* the unpublished set; the plan tick writes it */
/* One store; latched at the next boundary (D5), or on the next tick if due_tick, that boundary, has
 * passed. Every entry with a non-zero inc must have dir equal to its sign: the plan tick guarantees it. */
void       puldir_publish(uint32_t due_tick);
void       puldir_kill(uint8_t slot);         /* under the lock: zero live and published inc */
/* Under the lock: ticks left in this interval, N when the next tick is the
 * boundary that latches. The live inc runs left % N more ticks. */
uint32_t   puldir_left(void);
/* Under the lock: the slot's state, its position predicted ahead ticks on (0: the position itself). */
void       puldir_pulse_read(uint8_t slot, uint32_t ahead, PulDirSnap* out);
/* Under the lock: count, P and edge to a whole pulse. The caller guarantees the
 * axis is at rest, its live inc 0 and its pulse pin low. */
void       puldir_set_datum(uint8_t slot, int32_t count);

/* plan tick (puldir_plan.c) */
void puldir_plan_tick(void);
/* Write one entry of the set being built. A non-zero inc carries its sign as the
 * direction; zero keeps the last one, so a pause never turns the pin. */
void puldir_set_inc(PulDir*, PulDirInc* out, int32_t inc);
/* One interval toward T for an axis or a member; T, end and pb are 32.32 from the whole-pulse
 * base. D6 holds end M short when hold; arrival rounds away (E13). True once settled. */
bool puldir_steer(PulDir*, PulDirInc* out, int64_t T, int64_t end, int64_t pb, bool arrival, bool hold);

/* groups (puldir_group.c) */
void puldir_group_tick(const PulDirSnap* snap, PulDirInc* out);   /* every group, before the axes */
/* Under the lock, from main (shadow NULL) or the plan tick (shadow, the set it writes, zeroed too):
 * no rising edge on any member after this tick, every member IDLE, the move abandoned. */
void puldir_group_halt(PulDirGroup*, PulDirInc* shadow);

/* Main context, under the lock, for every accepted motion command (5.8): enables a disabled drive,
 * whose motion the plan tick then holds for the enable delay, and cancels a pending release. */
void puldir_wake(PulDir*);
/* Under the lock, from main or the plan tick, for an axis outside a group's move: no rising edge
 * after this tick, the target where the pulses stopped, IDLE, REQ_HALT for the generator. A homing
 * run gets back the limits homeStart saved; its homing state is the caller's to set. */
void puldir_halt_locked(PulDir*);

/* events (puldir_loop.c) */
void puldir_refuse(PulDir*, const char* call, PulDirRefusal why, float value);

/* Reply lines: four-decimal integer formatting, no float formatter. Each put
 * returns the new length; a line holds PULDIR_LINE - 1 characters. */
#define PULDIR_LINE 128u
size_t puldir_put(char* b, size_t at, const char* s);
size_t puldir_put_uint(char* b, size_t at, uint64_t v, int min_digits);
size_t puldir_put_num(char* b, size_t at, float v, bool up);
size_t puldir_head(char* b, const PulDir* a);             /* "[name] " */
const char* puldir_phase(PulDirHomeState h);
void   puldir_log(const PulDir* a, const char* line);     /* through the logger, if the axis logs; NULL: no axis yet */
#endif
