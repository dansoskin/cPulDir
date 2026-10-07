/* Main context: setup, commands, getters. */
#include <string.h>
#include "puldir_internal.h"

static bool started;                    /* PulDir_start has run */

void PulDir_init(PulDir* a, const char* name)
{
    memset(a, 0, sizeof *a);
    size_t i = 0;
    if (name)
        for (; i < sizeof a->name - 1u && name[i] != '\0'; i++)
            a->name[i] = name[i];
    a->name[i] = '\0';
    a->conv = 1.0f;
    const float d[4] = { 200.0f, 200.0f, 200.0f, 0.0f };
    for (int k = 0; k < 4; k++)
        a->def[k] = a->lim[k] = d[k];
    a->catchup_ms = PULDIR_CATCHUP_MS;
    a->log_on = true;
    a->slot = PULDIR_MAX_AXES;
    a->mode = PULDIR_MODE_IDLE;
    a->home_state = PULDIR_HOME_IDLE;
    a->refusal = PULDIR_R_NONE;
    /* The plan interval is a constant inside MotionProfile_init's range and
     * the limits are finite and positive, so neither call can refuse. */
    (void)MotionProfile_init(&a->prof, 0.0f, 1.0f / (float)PULDIR_PLAN_HZ);
    (void)MotionProfile_setLimits(&a->prof, (MotionLimits){ d[0], d[1], d[2], d[3] });
}

void PulDir_attachPins(PulDir* a, puldir_pin_t pulse, puldir_pin_t dir,
                       bool pulse_active_low, bool dir_inverted)
{
    a->pul = pulse;
    a->dir = dir;
    a->pul_active_low = pulse_active_low;
    a->dir_inverted = dir_inverted;
    a->has_pins = true;
    port_pin_init_output(pulse, pulse_active_low);        /* inactive */
    port_pin_init_output(dir, !dir_inverted);             /* the level for +1 */
}

bool PulDir_register(PulDir* a)
{
    PulDirRefusal why = PULDIR_R_NONE;
    if (!a->has_pins)
        why = PULDIR_R_NO_PIN;
    else if (a->slot != PULDIR_MAX_AXES || started)
        why = PULDIR_R_BUSY;            /* already registered, or the ticks are running */
    else if (puldir_naxes >= PULDIR_MAX_AXES)
        why = PULDIR_R_FULL;
    if (why != PULDIR_R_NONE) {
        puldir_refuse(a, "register", why, 0.0f);
        return false;
    }
    puldir_pulse_add(a);
    return true;
}

void PulDir_start(void)
{
    started = true;
    port_start(puldir_pulse_tick, puldir_plan_tick);
}
