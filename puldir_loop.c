/* Events and the log: PulDir_loop turns an axis's event bits into log lines. */
#include "puldir_internal.h"

static void (*logger)(const char*);

void PulDir_attachLogger(void (*fn)(const char*))
{
    logger = fn;
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
