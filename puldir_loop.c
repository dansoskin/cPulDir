/* Events and the log: PulDir_loop turns an axis's event bits into log lines. */
#include <float.h>
#include <math.h>
#include <stdio.h>
#include "puldir_internal.h"

static void (*logger)(const char*);
static uint32_t overruns_logged;        /* the overrun total last logged, by any axis */
static uint32_t late_logged;            /* the late-set total last logged, likewise */

void PulDir_attachLogger(void (*fn)(const char*))
{
    logger = fn;
}

void puldir_log(const PulDir* a, const char* line)
{
    if ((a == NULL || a->log_on) && logger != NULL)
        logger(line);
}

void PulDir_setLogging(PulDir* a, bool on)
{
    a->log_on = on;
}

/* Main context. The event bits are shared with the plan tick, so they are
 * set under the lock; PulDir_loop reports the refusal. */
void puldir_refuse(PulDir* a, const char* call, PulDirRefusal why, float value)
{
    port_lock();
    a->refusal = why;
    a->refused_call = call;
    a->refused_value = value;
    a->events |= PULDIR_EV_REFUSED;
    port_unlock();
}

static const char* reason(PulDirRefusal why)
{
    switch (why) {
    case PULDIR_R_FAULT:      return "drive fault latched";
    case PULDIR_R_MODE:       return "not accepted in this mode";
    case PULDIR_R_ARG:        return "bad argument";
    case PULDIR_R_RATE:       return "speed at or above the rate cap ";
    case PULDIR_R_WINDOW:     return "the jerk window is frozen while moving";
    case PULDIR_R_NOT_IDLE:   return "not idle";
    case PULDIR_R_NO_PIN:     return "no pin attached";
    case PULDIR_R_NO_HOMING:  return "homing not configured";
    case PULDIR_R_DROPPED:    return "dropped";
    case PULDIR_R_BUSY:       return "busy";
    case PULDIR_R_FULL:       return "axis table full";
    case PULDIR_R_UNREGISTERED: return "not registered";
    default:                  return "refused";
    }
}

const char* puldir_phase(PulDirHomeState h)
{
    static const char* const name[] = {
        "IDLE", "MOVE_TOWARDS_LIMIT", "DELAY1", "MOVE_AWAY_FROM_LIMIT",
        "DELAY2", "ADJUST_POSITION", "DELAY3", "ERROR"
    };
    return (unsigned)h < sizeof name / sizeof name[0] ? name[h] : "?";
}

/* The jerk floor is a plain quotient, one ulp under the boundary; rounded up with margin, the
 * figure typed back fits through the conversion, this scaling and the read-back. */
static float floor_up(float v)
{
    return ceilf(v * 1.0e4f * (1.0f + 16.0f * FLT_EPSILON)) / 1.0e4f;
}

void PulDir_loop(PulDir* a)
{
    port_lock();
    const uint16_t ev = a->events;
    a->events = 0;
    const PulDirRefusal why = a->refusal;
    const char* call = a->refused_call;
    const float value = a->refused_value;
    const uint32_t faults = a->fault.count;
    const PulDirHomeState failed_in = a->home.fail;
    port_unlock();
    if (!a->log_on || logger == NULL)
        return;

    /* The bits keep no order: the drive's state first, then what it caused. */
    char b[PULDIR_LINE];
    const char* n = a->name;
    const uint32_t overruns = PulDir_overruns();    /* one aligned word: no lock */
    if (overruns != overruns_logged) {
        overruns_logged = overruns;
        snprintf(b, sizeof b, "[%s] pulse tick overrun, total %lu", n, (unsigned long)overruns);
        logger(b);
    }
    const uint32_t late = PulDir_lateSets();
    if (late != late_logged) {
        late_logged = late;
        snprintf(b, sizeof b, "[%s] plan tick late, total %lu", n, (unsigned long)late);
        logger(b);
    }
    if (ev & PULDIR_EV_FAULT) {
        snprintf(b, sizeof b, "[%s] fault latched, count %lu", n, (unsigned long)faults);
        logger(b);
    }
    if (ev & PULDIR_EV_FAULT_CLEARED) {
        snprintf(b, sizeof b, "[%s] fault cleared", n);
        logger(b);
    }
    if (ev & PULDIR_EV_HOME_FAILED) {
        snprintf(b, sizeof b, "[%s] homing failed in %s", n, puldir_phase(failed_in));
        logger(b);
    }
    if (ev & PULDIR_EV_REFUSED) {
        const char* c = call != NULL ? call : "call";
        if (why == PULDIR_R_TAPS)               /* a floor: typed back, it must fit */
            snprintf(b, sizeof b, "[%s] %s refused: jerk below %.4f needs more than %d taps", n, c,
                     (double)floor_up(value), (int)MOTION_PROFILE_MAX_TAPS);
        else if (why == PULDIR_R_RATE)
            snprintf(b, sizeof b, "[%s] %s refused: %s%.4f", n, c, reason(why), (double)value);
        else
            snprintf(b, sizeof b, "[%s] %s refused: %s", n, c, reason(why));
        logger(b);
    }
    if (ev & PULDIR_EV_STREAM_TIMEOUT) {
        snprintf(b, sizeof b, "[%s] stream timed out", n);
        logger(b);
    }
    if (ev & PULDIR_EV_LANDED) {
        snprintf(b, sizeof b, "[%s] landed at %.4f", n, (double)PulDir_position(a));
        logger(b);
    }
    if (ev & PULDIR_EV_HOMED) {
        snprintf(b, sizeof b, "[%s] homed at %.4f", n, (double)PulDir_position(a));
        logger(b);
    }
}
