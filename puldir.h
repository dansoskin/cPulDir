/* puldir: several PUL/DIR drives from one microcontroller, every pulse from an
 * interrupt. docs/puldir-guidelines.md is the design. */
#ifndef PULDIR_H
#define PULDIR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "motion_profile.h"
#include "puldir_port.h"
#include "puldir_enable.h"
#include "puldir_fault.h"
#include "puldir_homing.h"
#include "puldir_stream.h"

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

typedef enum { PULDIR_IDLE, PULDIR_ENABLED, PULDIR_MOVING, PULDIR_FAULT } PulDirStatus;

typedef enum {
    PULDIR_MODE_IDLE, PULDIR_MODE_PROFILE, PULDIR_MODE_STOPPING,
    PULDIR_MODE_STREAM, PULDIR_MODE_GROUP, PULDIR_MODE_HOMING
} PulDirMode;

/* Why a call was refused; PulDir_loop turns it into one log line. */
typedef enum {
    PULDIR_R_NONE, PULDIR_R_FAULT, PULDIR_R_MODE, PULDIR_R_ARG, PULDIR_R_RATE,
    PULDIR_R_WINDOW, PULDIR_R_TAPS, PULDIR_R_NOT_IDLE, PULDIR_R_NO_PIN,
    PULDIR_R_NO_HOMING, PULDIR_R_DROPPED, PULDIR_R_BUSY, PULDIR_R_FULL, PULDIR_R_UNREGISTERED
} PulDirRefusal;

struct PulDirGroup;

typedef struct PulDir {
    /* setup, main context */
    char  name[16];
    puldir_pin_t pul, dir;
    bool  pul_active_low, dir_inverted;
    bool  has_pins;
    float conv;                          /* pulses per user unit, > 0 */
    float def[4];                        /* default speed, accel, decel, jerk, user units */
    float lim[4];                        /* current speed, accel, decel, jerk, user units */
    bool  log_on;
    uint8_t slot;                        /* index in the axis table, PULDIR_MAX_AXES if unregistered */

    /* main -> plan requests (D17), written under the lock */
    volatile uint16_t req;
    float req_speed;                     /* REQ_ENDLESS: the jog speed, pulses/s */

    /* plan tick state */
    volatile PulDirMode mode;
    MotionProfile prof;                  /* pulses, relative to origin */
    uint64_t target;                     /* absolute, 32.32 pulses */
    int32_t  origin;
    volatile bool moving;
    volatile float vel_out;              /* pulses/s, exactly 0.0f when stopped */
    uint8_t  settle;                     /* plan runs left before IDLE (D19) */
    int8_t   dir_out;                    /* the last published direction, +1 or -1 */
    bool     arrived;                    /* the final interval is published; settling */
    int8_t   approach;                   /* the side the final interval came from: +1, -1, 0 */
    struct PulDirGroup* group;           /* the group whose move this axis is in (GROUP mode) */

    /* the modules, each with its own file */
    PulDirEnable enable;
    PulDirFault  fault;
    PulDirHoming home;
    PulDirStream stream;

    /* the pulse tick's state lives in puldir_pulse.c, by slot */

    /* events for PulDir_loop */
    volatile uint16_t events;
    PulDirRefusal refusal;
    const char* refused_call;
    float refused_value;                 /* R_RATE: the cap. R_TAPS: the jerk floor as a plain quotient; the
                                            log line rounds it up, and that printed value is the one that fits */
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
uint32_t PulDir_lateSets(void);          /* sets that latched after their boundary: the plan tick published too late */
void     PulDir_consoleMenu(char* buf, size_t n);

/* axis setup, before PulDir_start(), after both timers are attached */
/* Refuses a NULL name or one longer than 15 characters, a conversion not finite and positive, and
 * timing unset or not a whole 2 to 4096 pulse periods per plan period; then writes nothing. Logs one
 * line either way: the refusal, or the axis's top speed and resolution. */
bool  PulDir_init(PulDir*, const char* name, float pulses_per_unit);
void  PulDir_attachPins(PulDir*, puldir_pin_t pulse, puldir_pin_t dir,
                        bool pulse_active_low, bool dir_inverted);
void  PulDir_attachEnable(PulDir*, puldir_pin_t, bool inverted);
void  PulDir_attachFault(PulDir*, puldir_pin_t, bool inverted);
void  PulDir_attachFaultClear(PulDir*, puldir_pin_t, bool inverted);
bool  PulDir_setDefaults(PulDir*, float speed, float accel, float decel, float jerk);
bool  PulDir_restoreDefaults(PulDir*);
/* D16: refuses a delay past half the tick counter's range (about 3 h at a 5 us pulse). */
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
