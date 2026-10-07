#ifndef FAKE_PORT_H
#define FAKE_PORT_H
#include <stdbool.h>
#include <stdint.h>
#include "puldir_port.h"

#define FAKE_PINS 64

puldir_pin_t fake_pin(uint8_t id);
bool  fake_level(uint8_t id);                       /* last level written or set */
void  fake_set_input(uint8_t id, bool level);       /* fault pins, from the test */
void  fake_mark_motion_pin(uint8_t id);             /* a pulse or direction pin: written only from the pulse tick */
typedef void (*fake_write_fn)(uint32_t tick, uint8_t id, bool level);
void  fake_on_write(fake_write_fn fn);              /* called for every port_pin_write */
void  fake_pulse(bool overrun);                     /* one pulse tick */
void  fake_plan(void);                              /* one plan tick */
/* n pulse ticks; a plan tick runs fake_plan_phase ticks after every interval boundary */
void  fake_run(uint32_t ticks);
extern uint32_t fake_plan_phase;                    /* default PULDIR_TICKS_PER_PLAN / 2 */
extern uint32_t fake_preempt;                       /* pulse ticks run inside each port_pin_read */
#endif
