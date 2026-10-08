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

/* The port's timer attach calls hand the periods over, before any PulDir_init. Each refuses a value
 * out of range and once the timing is fixed (the first PulDir_init or PulDir_start), changing nothing. */
bool puldir_timing_pulse(uint32_t pulse_us);    /* 2 to 1000: one pulse tick, one pulse width */
bool puldir_timing_plan(uint32_t plan_us);      /* 250 to 10000; PulDir_init checks it is 2 to 4096 pulse periods */

void port_pin_init_output(puldir_pin_t, bool level);
void port_pin_init_input (puldir_pin_t, bool pullup);
void port_pin_write(puldir_pin_t, bool level);   /* from the pulse tick: one register write */
bool port_pin_read (puldir_pin_t);               /* from the plan tick */
/* Sets the two timers' periods to the pulse_us and plan_us the port handed over, in 1 us counts. */
void port_start(void (*pulse_tick)(bool overrun), void (*plan_tick)(void));
void port_lock(void);                            /* mask only the library's two interrupts */
void port_unlock(void);

#ifdef __cplusplus
}
#endif
#endif
