/* puldir: several PUL/DIR drives from one microcontroller, every pulse from an
 * interrupt. docs/puldir-guidelines.md is the design. */
#ifndef PULDIR_H
#define PULDIR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "motion_profile.h"
#include "puldir_port.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef PULDIR_MAX_AXES
#define PULDIR_MAX_AXES 8u
#endif
#ifndef PULDIR_MAX_GROUPS
#define PULDIR_MAX_GROUPS 4u
#endif
#ifndef PULDIR_CATCHUP_MS
#define PULDIR_CATCHUP_MS 50u
#endif
#ifndef PULDIR_STREAM_TIMEOUT_MS
#define PULDIR_STREAM_TIMEOUT_MS 100u
#endif
#ifndef PULDIR_FAULT_DEBOUNCE_MS
#define PULDIR_FAULT_DEBOUNCE_MS 2u
#endif
#ifndef PULDIR_FAULT_CLEAR_MS
#define PULDIR_FAULT_CLEAR_MS 10u
#endif
#ifndef PULDIR_HOMING_DELAY_MS
#define PULDIR_HOMING_DELAY_MS 1000u
#endif
#ifndef PULDIR_TICK_START
#define PULDIR_TICK_START 0u                /* the rollover test starts near the wrap (D9) */
#endif

#define PULDIR_TICKS_PER_PLAN (PULDIR_TICK_HZ / PULDIR_PLAN_HZ)
typedef char puldir_check_ratio[(PULDIR_TICK_HZ % PULDIR_PLAN_HZ) == 0u ? 1 : -1];
typedef char puldir_check_plan[(PULDIR_PLAN_HZ >= 1000u && PULDIR_PLAN_HZ <= 4000u) ? 1 : -1];
typedef char puldir_check_n[(PULDIR_TICKS_PER_PLAN >= 2u && PULDIR_TICKS_PER_PLAN <= 4096u) ? 1 : -1];

typedef enum { PULDIR_IDLE, PULDIR_ENABLED, PULDIR_MOVING, PULDIR_FAULT } PulDirStatus;

typedef enum {
    PULDIR_MODE_IDLE, PULDIR_MODE_PROFILE, PULDIR_MODE_STOPPING,
    PULDIR_MODE_STREAM, PULDIR_MODE_GROUP, PULDIR_MODE_HOMING
} PulDirMode;

typedef enum {
    PULDIR_HOME_IDLE, PULDIR_HOME_MOVE_TOWARDS_LIMIT, PULDIR_HOME_DELAY1,
    PULDIR_HOME_MOVE_AWAY_FROM_LIMIT, PULDIR_HOME_DELAY2,
    PULDIR_HOME_ADJUST_POSITION, PULDIR_HOME_DELAY3, PULDIR_HOME_ERROR
} PulDirHomeState;

/* Why a call was refused; PulDir_loop turns it into one log line. */
typedef enum {
    PULDIR_R_NONE, PULDIR_R_FAULT, PULDIR_R_MODE, PULDIR_R_ARG, PULDIR_R_RATE,
    PULDIR_R_WINDOW, PULDIR_R_TAPS, PULDIR_R_NOT_IDLE, PULDIR_R_NO_PIN,
    PULDIR_R_NO_HOMING, PULDIR_R_DROPPED, PULDIR_R_BUSY, PULDIR_R_FULL
} PulDirRefusal;

struct PulDirGroup;

typedef struct PulDir {
    /* setup, main context */
    char  name[16];
    puldir_pin_t pul, dir, ena, flt, clr;
    bool  pul_active_low, dir_inverted, ena_inverted, flt_inverted, clr_inverted;
    bool  has_pins, has_ena, has_flt, has_clr;
    float conv;                          /* pulses per user unit, > 0 */
    float def[4];                        /* default speed, accel, decel, jerk, user units */
    float lim[4];                        /* current speed, accel, decel, jerk, user units */
    uint32_t catchup_ms;
    uint32_t enable_ticks, release_ticks; /* setDelays' delays, in pulse ticks */
    bool  auto_release, log_on;
    int8_t home_dir;                     /* +1, -1, 0 = not configured */
    float home_speed, home_adjust, home_zero;
    float home_lim[4];                   /* the limits at homeStart: a run that ends unhomed restores them */
    const uint8_t* home_switch;
    uint32_t home_timeout_ticks;         /* setHoming's phase timeout */
    uint8_t slot;                        /* index in the axis table, PULDIR_MAX_AXES if unregistered */

    /* main -> plan requests (D17), written under the lock */
    volatile uint16_t req;
    float req_speed;                     /* REQ_ENDLESS: the jog speed, pulses/s */

    /* plan tick state */
    volatile PulDirMode mode;
    MotionProfile prof;                  /* pulses, relative to origin */
    uint64_t target;                     /* absolute, 32.32 pulses */
    int32_t  origin;
    volatile bool moving, enabled, homed, fault;
    uint32_t enabled_at;                 /* tick of the enable edge */
    bool     ena_wait;                   /* the enable delay has not yet passed since enabled_at: hold */
    bool     release_due;                /* auto-release: the axis went IDLE at idle_at */
    uint32_t idle_at;
    bool     held;                       /* halted by a fault latch: no auto-release until a command (D12) */
    bool     ena_off;                    /* enable(false) with the pulse pin high: the plan tick lowers
                                            the enable pin once the pulse pin is low */
    volatile float vel_out;              /* pulses/s, exactly 0.0f when stopped */
    uint8_t  settle;                     /* plan runs left before IDLE (D19) */
    int8_t   dir_out;                    /* the last published direction, +1 or -1 */
    bool     arrived;                    /* the final interval is published; settling */
    int8_t   approach;                   /* the side the final interval came from: +1, -1, 0 */

    /* stream */
    float sp_pos, sp_vel;                /* pulses, pulses/s; the last setpoint */
    uint32_t sp_tick;                    /* its arrival tick, or streamBegin's before the first */
    bool  sp_have;                       /* a setpoint has arrived since streamBegin */
    bool  stream_live;                   /* plan tick: the generator is idle under the stream */
    volatile bool stream_timed_out;
    float v_cmd;                         /* pulses/s */
    volatile float stream_err;           /* pulses: p_hold - P at the boundary */

    /* homing and fault */
    volatile PulDirHomeState home_state;
    PulDirHomeState home_fail;           /* the phase the last failed run was in */
    uint32_t home_t0;                    /* the tick the current phase began */
    uint32_t flt_since;                  /* the first sample of the current active run */
    uint32_t fault_count, last_fault_tick, clr_t0;
    bool flt_seen, clearing;
    bool clr_armed;                      /* a clear pulse ended on a latch: the first sample that
                                            reads the input inactive lifts it */

    struct PulDirGroup* group;           /* the group whose move this axis is in (GROUP mode) */

    /* the pulse tick's state lives in puldir_pulse.c, by slot */

    /* events for PulDir_loop */
    volatile uint16_t events;
    PulDirRefusal refusal;
    const char* refused_call;
    float refused_value;
} PulDir;

typedef struct PulDirGroup {
    PulDir* axes[PULDIR_MAX_AXES];
    uint8_t count;
    MotionProfile s;                     /* the path parameter, 0 to 1 */
    uint64_t start[PULDIR_MAX_AXES];     /* 32.32 */
    int32_t  delta[PULDIR_MAX_AXES];     /* whole pulses (D4) */
    volatile bool moving, faulted;
    volatile bool active;                /* the plan tick runs s: from an accepted move to its end or a halt */
    volatile uint8_t req;                /* main -> plan requests, under the lock */
    bool started;                        /* every member's direction was set; s advances */
    bool still;                          /* every delta is zero */
} PulDirGroup;

/* library-wide */
void     PulDir_attachLogger(void (*fn)(const char*));
void     PulDir_start(void);
uint32_t PulDir_ticks(void);
uint32_t PulDir_overruns(void);
void     PulDir_consoleMenu(char* buf, size_t n);

/* axis setup, before PulDir_start() */
/* Refuses a NULL name or one longer than 15 characters, and then writes nothing. */
bool  PulDir_init(PulDir*, const char* name);
void  PulDir_attachPins(PulDir*, puldir_pin_t pulse, puldir_pin_t dir,
                        bool pulse_active_low, bool dir_inverted);
void  PulDir_attachEnable(PulDir*, puldir_pin_t, bool inverted);
void  PulDir_attachFault(PulDir*, puldir_pin_t, bool inverted);
void  PulDir_attachFaultClear(PulDir*, puldir_pin_t, bool inverted);
/* IDLE only. Limits and defaults are held in pulses, so call setDefaults after it. */
bool  PulDir_setConversion(PulDir*, float pulses_per_unit);
bool  PulDir_setDefaults(PulDir*, float speed, float accel, float decel, float jerk);
bool  PulDir_restoreDefaults(PulDir*);
/* D16: refuses a delay past half the tick counter's range (about 3 h at 200 kHz). */
bool  PulDir_setDelays(PulDir*, uint32_t enable_ms, uint32_t release_ms);
void  PulDir_setAutoRelease(PulDir*, bool);
/* D16: refuses bad values, a speed at the tick ceiling and a call while homing. The timeout covers
 * each moving phase, up to half the counter; the delays between phases are fixed. */
bool  PulDir_setHoming(PulDir*, int direction, float speed, float adjust,
                       const uint8_t* switch_ptr, float zero_pos, uint32_t phase_timeout_ms);
bool  PulDir_setStreamCatchup(PulDir*, uint32_t ms);
bool  PulDir_register(PulDir*);

/* limits. These, setDefaults and restoreDefaults are refused (R_MODE) in GROUP and HOMING. */
bool  PulDir_setSpeed(PulDir*, float);         float PulDir_speed(PulDir*);
bool  PulDir_setAcceleration(PulDir*, float);  float PulDir_acceleration(PulDir*);
bool  PulDir_setDeceleration(PulDir*, float);  float PulDir_deceleration(PulDir*);
bool  PulDir_setJerk(PulDir*, float);          float PulDir_jerk(PulDir*);

/* motion */
bool  PulDir_moveTo(PulDir*, float position);
bool  PulDir_moveBy(PulDir*, float distance);
bool  PulDir_jog(PulDir*, float speed);
void  PulDir_stop(PulDir*);
void  PulDir_halt(PulDir*);
bool  PulDir_enable(PulDir*, bool);              /* refused without an enable pin; false halts first */

/* stream */
bool  PulDir_streamBegin(PulDir*);
void  PulDir_stream(PulDir*, float position, float velocity);
void  PulDir_streamEnd(PulDir*);
bool  PulDir_inStream(PulDir*);
bool  PulDir_streamTimedOut(PulDir*);
float PulDir_streamError(PulDir*);

/* homing */
/* A run that ends unhomed (halt, stop, homeStop, timeout, fault) puts back the
 * limits in force at homeStart; one that homes leaves the defaults. */
bool  PulDir_homeStart(PulDir*);
void  PulDir_homeStop(PulDir*);                  /* halts a run; the homing state goes IDLE */
PulDirHomeState PulDir_homeState(PulDir*);
bool  PulDir_isHomed(PulDir*);

/* state */
float PulDir_position(PulDir*);
float PulDir_velocity(PulDir*);
float PulDir_target(PulDir*);
bool  PulDir_setPosition(PulDir*, float);
bool  PulDir_isMoving(PulDir*);
PulDirMode   PulDir_mode(PulDir*);
PulDirStatus PulDir_status(PulDir*);
bool  PulDir_clearFault(PulDir*);
void  PulDir_setLogging(PulDir*, bool);
void  PulDir_loop(PulDir*);
void  PulDir_console(PulDir*, char cmd, const char* arg);

/* group */
/* D16: refuses. The group's table keeps a pointer to it: give it static storage. */
bool  PulDirGroup_init(PulDirGroup*, PulDir** axes, uint8_t count);
/* Refuses a member not IDLE or faulted, and a member travel beyond 2^24 pulses.
 * While the move runs, members refuse their limit setters (R_MODE). */
bool  PulDirGroup_moveTo(PulDirGroup*, const float* targets);
/* Ramped along the line. After a stop mid-path each member ends on its last whole
 * pulse, within one pulse of the line; the final edges need not share a tick. */
void  PulDirGroup_stop(PulDirGroup*);
void  PulDirGroup_halt(PulDirGroup*);
bool  PulDirGroup_isMoving(PulDirGroup*);
bool  PulDirGroup_faulted(PulDirGroup*);

#ifdef __cplusplus
}
#endif
#endif
