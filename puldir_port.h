/* The port contract (guidelines 5.10). Implemented once per platform. */
#ifndef PULDIR_PORT_H
#define PULDIR_PORT_H

#include <stdbool.h>
#include <stdint.h>
#include "puldir_pin.h"                    /* puldir_pin_t, from the port's directory (D15) */

#ifdef __cplusplus
extern "C" {
#endif

#ifndef PULDIR_TICK_HZ
#define PULDIR_TICK_HZ 50000u              /* 200000u for 100k pulses/s */
#endif
#ifndef PULDIR_PLAN_HZ
#define PULDIR_PLAN_HZ 1000u
#endif

void port_pin_init_output(puldir_pin_t, bool level);
void port_pin_init_input (puldir_pin_t, bool pullup);
void port_pin_write(puldir_pin_t, bool level);   /* from the pulse tick: one register write */
bool port_pin_read (puldir_pin_t);               /* from the plan tick */
void port_start(void (*pulse_tick)(bool overrun), void (*plan_tick)(void));
void port_lock(void);                            /* mask only the library's two interrupts */
void port_unlock(void);

#ifdef __cplusplus
}
#endif
#endif
