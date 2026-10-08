/* The @ console: cFlexyStepper_decode.c verb for verb, plus jerk, from main context. Replies go
 * through the logger as PulDir_loop's lines do, and honour the axis's logging switch. */
#include <stdio.h>
#include <stdlib.h>
#include "puldir_internal.h"

/* Cut short to fit n. */
void PulDir_consoleMenu(char* buf, size_t n)
{
    static const char menu[] =
        "Stepper commands:\n"
        "E<0|1> - Enable/disable motor\n"
        "P<position> - Set current position\n"
        "p - Get current position\n"
        "V<speed> - Set speed\n"
        "v - Get current velocity\n"
        "A<acceleration> - Set acceleration\n"
        "a - Get current acceleration\n"
        "D<deceleration> - Set deceleration\n"
        "d - Get current deceleration\n"
        "Z<jerk> - Set jerk (0 = no jerk limit)\n"
        "z - Get current jerk\n"
        "J<speed> - Jog at speed\n"
        "R<distance> - Move relative to current position\n"
        "T<position> - Move to absolute position\n"
        "F - Restore default speed, acceleration, deceleration and jerk\n"
        "k - Stop at target position\n"
        "K - Halt now and release motor\n"
        "s - Get status, mode, homed, home state and fault count\n"
        "C - Clear fault\n"
        "H - Start homing state machine\n";
    snprintf(buf, n, "%s", menu);
}

/* The whole argument is one number; white space around it is not part of it. */
static bool number(const char* arg, float* out)
{
    if (arg == NULL)
        return false;
    char* end = NULL;
    const float v = strtof(arg, &end);
    if (end == arg)
        return false;
    while (*end == ' ' || *end == '\t' || *end == '\r' || *end == '\n')
        end++;
    if (*end != '\0')
        return false;
    *out = v;
    return true;
}

static void say(const PulDir* a, const char* text)
{
    char b[PULDIR_LINE];
    snprintf(b, sizeof b, "[%s] %s", a->name, text);
    puldir_log(a, b);
}

static void say_num(const PulDir* a, const char* label, float v)
{
    char b[PULDIR_LINE];
    snprintf(b, sizeof b, "[%s] %s%.4f", a->name, label, (double)v);
    puldir_log(a, b);
}

static const char* mode_name(PulDirMode m)
{
    static const char* const name[] = { "IDLE", "PROFILE", "STOPPING", "STREAM", "GROUP", "HOMING" };
    return (unsigned)m < sizeof name / sizeof name[0] ? name[m] : "?";
}

static const char* status_name(PulDirStatus s)
{
    static const char* const name[] = { "IDLE", "ENABLED", "MOVING", "FAULT" };
    return (unsigned)s < sizeof name / sizeof name[0] ? name[s] : "?";
}

static void status(PulDir* a)
{
    port_lock();
    const uint32_t faults = a->fault.count, last = a->fault.last_tick;
    const PulDirHomeState hs = a->home_state, failed = a->home_fail;
    port_unlock();
    char fail[40] = "";
    if (hs == PULDIR_HOME_ERROR)
        snprintf(fail, sizeof fail, " (failed in %s)", puldir_phase(failed));
    char b[PULDIR_LINE];
    snprintf(b, sizeof b, "[%s] status %s, mode %s, homed %d, home %s%s, faults %lu, last fault tick %lu",
             a->name, status_name(PulDir_status(a)), mode_name(PulDir_mode(a)), PulDir_isHomed(a) ? 1 : 0,
             puldir_phase(hs), fail, (unsigned long)faults, (unsigned long)last);
    puldir_log(a, b);
}

static void halt_and_release(PulDir* a)
{
    PulDir_halt(a);
    if (a->enable.attached)
        (void)PulDir_enable(a, false);
    else
        say(a, "halted, no enable pin to release");
}

void PulDir_console(PulDir* a, char cmd, const char* arg)
{
    float x = 0.0f;
    switch (cmd) {
    case 'p': say_num(a, "position ", PulDir_position(a)); return;
    case 'v': say_num(a, "velocity ", PulDir_velocity(a)); return;
    case 'a': say_num(a, "acceleration ", PulDir_acceleration(a)); return;
    case 'd': say_num(a, "deceleration ", PulDir_deceleration(a)); return;
    case 'z': say_num(a, "jerk ", PulDir_jerk(a)); return;
    case 'F': (void)PulDir_restoreDefaults(a); return;
    case 'k': PulDir_stop(a); return;
    case 'K': halt_and_release(a); return;
    case 's': status(a); return;
    case 'C': (void)PulDir_clearFault(a); return;
    case 'H': (void)PulDir_homeStart(a); return;
    case 'E': case 'P': case 'V': case 'A': case 'D': case 'Z':
    case 'J': case 'R': case 'T':
        break;
    default: {
        char text[] = "unknown command: ?";
        text[sizeof text - 2u] = cmd;
        say(a, text);
        return;
    }
    }

    if (!number(arg, &x) || (cmd == 'E' && x != 0.0f && x != 1.0f)) {
        puldir_refuse(a, "console", PULDIR_R_ARG, 0.0f);
        return;
    }
    switch (cmd) {
    case 'E': (void)PulDir_enable(a, x == 1.0f); break;
    case 'P': (void)PulDir_setPosition(a, x); break;
    case 'V': (void)PulDir_setSpeed(a, x); break;
    case 'A': (void)PulDir_setAcceleration(a, x); break;
    case 'D': (void)PulDir_setDeceleration(a, x); break;
    case 'Z': (void)PulDir_setJerk(a, x); break;
    case 'J': (void)PulDir_jog(a, x); break;
    case 'R': (void)PulDir_moveBy(a, x); break;
    default:  (void)PulDir_moveTo(a, x); break;
    }
}
