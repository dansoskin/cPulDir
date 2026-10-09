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
    REQ_SETTINGS = 1u << 0, REQ_TARGET = 1u << 1, REQ_ENDLESS = 1u << 2, REQ_STOP = 1u << 3,
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
#define PULDIR_N ((int64_t)puldir_timing.n)   /* pulse ticks per plan interval, as the 32.32 maths wants it */
bool puldir_timing_take(void);                /* PulDir_init: checks the pair and fixes it; true once set */

/* A duration in pulse ticks; every time is compared by unsigned subtraction (5.9). */
uint32_t puldir_ms_ticks(uint32_t ms);
bool     puldir_ms_fits(uint32_t ms);         /* a setter's delay or timeout is at most PULDIR_TICKS_MAX */
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
int64_t puldir_to_fix(float pulses);          /* float pulses to 32.32 */
float puldir_to_pulses(int64_t q);            /* 32.32 to float pulses, saturated to +-2^24 (D2) */
int32_t puldir_div_trunc(int64_t d);          /* a 32.32 distance over one interval, as an increment */
/* Settings in user units to the generator, and to the axis's own only if it takes them. */
void puldir_take_settings(PulDir*, MotionSettings);
/* The generator stops dead where the pulses are (pb, relative to the origin): a
 * new goal (E17), and a stream's idle generator stays put. */
void puldir_halt_generator(PulDir*, int64_t pb);
/* Aims this interval's end at the generator's position (T, pb, end: 32.32 from base). True at rest:
 * settled on the final position, the generator done, the last pulse lowered. */
bool puldir_run_generator(PulDir*, PulDirInc* out, uint64_t base, int64_t pb, int32_t live,
                          bool pul_high);

/* main context (puldir.c) */
float puldir_rate_cap(float conv);            /* in user units: speeds at or above it are refused */
MotionSettings puldir_in_pulses(const PulDir*, MotionSettings);   /* user units times the conversion */
/* Under the lock, for every motion command: the fault latch and the mode at one instant (D17). */
PulDirRefusal puldir_motion_check(const PulDir*);
/* Under the lock, from main or the plan tick, for an axis outside a group's move: no rising edge
 * after this tick, the target where the pulses stopped, IDLE, REQ_HALT for the generator. A homing
 * run gets back the settings homeStart saved; its homing state is the caller's to set. */
void puldir_halt_locked(PulDir*);

/* events (puldir_loop.c) */
void puldir_refuse(PulDir*, const char* call, PulDirRefusal why, float value);

/* Log and reply lines, each "[name] " and the text, by snprintf in main context only (E33).
 * A line holds PULDIR_LINE - 1 characters; a longer one is cut short. */
#define PULDIR_LINE 128u
const char* puldir_phase(PulDirHomeState h);
void   puldir_log(const PulDir* a, const char* line);     /* through the logger, if the axis logs; NULL: no axis yet */

/* The modules' functions: each header's second half, kept out of puldir.h's view. */
#define PULDIR_INTERNAL_FUNCTIONS
#include "puldir_enable.h"
#include "puldir_fault.h"
#include "puldir_homing.h"
#include "puldir_stream.h"
#include "puldir_group.h"
#endif
