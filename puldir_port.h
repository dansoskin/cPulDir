/* The port contract (guidelines 5.10). Implemented once per platform. */
#ifndef PULDIR_PORT_H
#define PULDIR_PORT_H

#include <stdbool.h>
#include <stdint.h>
#include "puldir_pin.h"                    /* puldir_pin_t, from the port's directory (D15) */

#ifdef __cplusplus
extern "C" {
#endif

#ifdef PULDIR_TICK_HZ
#error "PULDIR_TICK_HZ is derived: set PULDIR_PULSE_US instead"
#endif
#ifndef PULDIR_PULSE_US
#define PULDIR_PULSE_US 20u                /* one pulse tick is one pulse width; 5u for 100k pulses/s */
#endif
#ifndef PULDIR_PLAN_HZ
#define PULDIR_PLAN_HZ 1000u
#endif
#define PULDIR_TIMER_HZ 1000000u           /* every port's timers count at 1 MHz, 1 us per count */
#define PULDIR_TICK_HZ (PULDIR_TIMER_HZ / PULDIR_PULSE_US)

/* The two timer periods, in counts. */
#define PULDIR_PULSE_COUNTS (PULDIR_PULSE_US)
#define PULDIR_PLAN_COUNTS  (PULDIR_TIMER_HZ / PULDIR_PLAN_HZ)
typedef char puldir_check_pulse_us[(PULDIR_PULSE_US >= 2u) ? 1 : -1];
typedef char puldir_check_plan_counts[(PULDIR_TIMER_HZ % PULDIR_PLAN_HZ) == 0u ? 1 : -1];
typedef char puldir_check_whole_plan[(PULDIR_PLAN_COUNTS % PULDIR_PULSE_COUNTS) == 0u ? 1 : -1];   /* N pulse periods */
typedef char puldir_check_16bit[(PULDIR_PLAN_COUNTS <= 65536u) ? 1 : -1];

void port_pin_init_output(puldir_pin_t, bool level);
void port_pin_init_input (puldir_pin_t, bool pullup);
void port_pin_write(puldir_pin_t, bool level);   /* from the pulse tick: one register write */
bool port_pin_read (puldir_pin_t);               /* from the plan tick */
/* Sets the pulse timer's period to PULDIR_PULSE_COUNTS and the plan timer's to PULDIR_PLAN_COUNTS. */
void port_start(void (*pulse_tick)(bool overrun), void (*plan_tick)(void));
void port_lock(void);                            /* mask only the library's two interrupts */
void port_unlock(void);

#ifdef __cplusplus
}
#endif
#endif
