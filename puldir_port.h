/* The port contract (guidelines 5.10). Implemented once per platform. */
#ifndef PULDIR_PORT_H
#define PULDIR_PORT_H

#include <stdbool.h>
#include <stdint.h>
#include "puldir_pin.h"                    /* puldir_pin_t, from the port's directory (D15) */

#ifdef __cplusplus
extern "C" {
#endif

#define PULDIR_TIMER_HZ 1000000u           /* every port's timers count at 1 MHz, 1 us per count */

/* The port's attach calls hand the periods over, before any PulDir_init. Each refuses a value
 * out of range and once the timing is fixed (the first PulDir_init or PulDir_start), changing nothing. */
bool puldir_timing_pulse(uint32_t pulse_us);    /* 2 to 1000: one pulse tick, one pulse width */
bool puldir_timing_plan(uint32_t plan_us);      /* 250 to 10000; PulDir_init checks it is 2 to 4096 pulse periods */

/* Levels only: pin modes are the application's (CubeMX's) business. The attach calls park each
 * output with port_pin_write before PulDir_start; after it the pulse tick writes the pulse and
 * direction pins, the plan tick reads the fault pin, and main writes the enable and clear pins
 * under the lock. */
void port_pin_write(puldir_pin_t, bool level);   /* from the pulse tick: one register write */
bool port_pin_read (puldir_pin_t);               /* from the plan tick */
/* Starts the pulse tick every pulse_us, the period the port handed over, in 1 us counts. */
void port_start(void (*pulse_tick)(bool overrun), void (*plan_tick)(void));
/* From the pulse tick, at the end of each boundary tick (the one that latches, every ticks_per_plan
 * ticks from the first): makes plan_tick run once the pulse tick returns, at a lower priority than
 * it. The plan tick then always starts just after the latch it serves, whatever ticks were lost
 * (E38). */
void port_plan_pend(void);
/* Masks at least the library's two interrupts, and no other interrupt may run inside it: the pulse
 * tick then waits at most the longest locked section, a few stores (E39). Never nested by the core.
 * The STM32 port masks every interrupt (PRIMASK). Works before any attach. */
void port_lock(void);
void port_unlock(void);

#ifdef __cplusplus
}
#endif
#endif
