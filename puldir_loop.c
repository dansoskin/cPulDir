/* Events and the log: PulDir_loop turns an axis's event bits into log lines. */
#include <float.h>
#include <math.h>
#include "puldir_internal.h"

static void (*logger)(const char*);
static uint32_t overruns_logged;        /* the overrun total last logged, by any axis */

void PulDir_attachLogger(void (*fn)(const char*))
{
    logger = fn;
}

void puldir_log(const PulDir* a, const char* line)
{
    if (a->log_on && logger != NULL)
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

/* ---- lines from integers: no double ever reaches a formatter --------------- */

size_t puldir_put(char* b, size_t at, const char* s)
{
    while (*s != '\0' && at < PULDIR_LINE - 1u)
        b[at++] = *s++;
    b[at] = '\0';
    return at;
}

size_t puldir_put_uint(char* b, size_t at, uint64_t v, int min_digits)
{
    char d[21];
    int k = 0;
    do {
        d[k++] = (char)('0' + (int)(v % 10u));
        v /= 10u;
    } while (v != 0u || k < min_digits);
    char s[2] = { 0, 0 };
    while (k > 0) {
        s[0] = d[--k];
        at = puldir_put(b, at, s);
    }
    return at;
}

/* A signed decimal with four places, from integer arithmetic; up rounds a
 * positive v so the printed figure, read back as a float, is never below it. */
size_t puldir_put_num(char* b, size_t at, float v, bool up)
{
    if (isnan(v))
        return puldir_put(b, at, "nan");
    if (v < 0.0f) {
        at = puldir_put(b, at, "-");
        v = -v;
    }
    if (!(v < 1.0e14f))                         /* also inf: past what four places can show */
        return puldir_put(b, at, "1e14+");
    const uint64_t q = up ? (uint64_t)ceilf(v * (1.0f + 4.0f * FLT_EPSILON) * 10000.0f)
                          : (uint64_t)(v * 10000.0f + 0.5f);
    at = puldir_put_uint(b, at, q / 10000u, 1);
    at = puldir_put(b, at, ".");
    return puldir_put_uint(b, at, q % 10000u, 4);
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
    default:                  return "refused";
    }
}

size_t puldir_head(char* b, const PulDir* a)
{
    size_t at = puldir_put(b, 0, "[");
    at = puldir_put(b, at, a->name);
    return puldir_put(b, at, "] ");
}

const char* puldir_phase(PulDirHomeState h)
{
    static const char* const name[] = {
        "IDLE", "MOVE_TOWARDS_LIMIT", "DELAY1", "MOVE_AWAY_FROM_LIMIT",
        "DELAY2", "ADJUST_POSITION", "DELAY3", "ERROR"
    };
    return (unsigned)h < sizeof name / sizeof name[0] ? name[h] : "?";
}

void PulDir_loop(PulDir* a)
{
    port_lock();
    const uint16_t ev = a->events;
    a->events = 0;
    const PulDirRefusal why = a->refusal;
    const char* call = a->refused_call;
    const float value = a->refused_value;
    const uint32_t faults = a->fault_count;
    const PulDirHomeState failed_in = a->home_fail;
    port_unlock();
    if (!a->log_on || logger == NULL)
        return;

    /* The bits keep no order: the drive's state first, then what it caused. */
    char b[PULDIR_LINE];
    const uint32_t overruns = PulDir_overruns();    /* one aligned word: no lock */
    if (overruns != overruns_logged) {
        overruns_logged = overruns;
        size_t at = puldir_head(b, a);
        at = puldir_put(b, at, "pulse tick overrun, total ");
        (void)puldir_put_uint(b, at, overruns, 1);
        logger(b);
    }
    if (ev & PULDIR_EV_FAULT) {
        size_t at = puldir_head(b, a);
        at = puldir_put(b, at, "fault latched, count ");
        (void)puldir_put_uint(b, at, faults, 1);
        logger(b);
    }
    if (ev & PULDIR_EV_FAULT_CLEARED) {
        (void)puldir_put(b, puldir_head(b, a), "fault cleared");
        logger(b);
    }
    if (ev & PULDIR_EV_HOME_FAILED) {
        size_t at = puldir_head(b, a);
        at = puldir_put(b, at, "homing failed in ");
        (void)puldir_put(b, at, puldir_phase(failed_in));
        logger(b);
    }
    if (ev & PULDIR_EV_REFUSED) {
        size_t at = puldir_head(b, a);
        at = puldir_put(b, at, call != NULL ? call : "call");
        at = puldir_put(b, at, " refused: ");
        if (why == PULDIR_R_TAPS) {
            at = puldir_put(b, at, "jerk below ");
            at = puldir_put_num(b, at, value, true);   /* a floor: typed back, it must fit */
            at = puldir_put(b, at, " needs more than ");
            at = puldir_put_uint(b, at, MOTION_PROFILE_MAX_TAPS, 1);
            (void)puldir_put(b, at, " taps");
        } else {
            at = puldir_put(b, at, reason(why));
            if (why == PULDIR_R_RATE)
                (void)puldir_put_num(b, at, value, false);
        }
        logger(b);
    }
    if (ev & PULDIR_EV_STREAM_TIMEOUT) {
        (void)puldir_put(b, puldir_head(b, a), "stream timed out");
        logger(b);
    }
    if (ev & PULDIR_EV_LANDED) {
        size_t at = puldir_head(b, a);
        at = puldir_put(b, at, "landed at ");
        (void)puldir_put_num(b, at, PulDir_position(a), false);
        logger(b);
    }
    if (ev & PULDIR_EV_HOMED) {
        size_t at = puldir_head(b, a);
        at = puldir_put(b, at, "homed at ");
        (void)puldir_put_num(b, at, PulDir_position(a), false);
        logger(b);
    }
}
