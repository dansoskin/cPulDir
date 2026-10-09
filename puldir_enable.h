/* Enable and release (5.8): the enable pin, the enable delay and auto-release. PulDir embeds the
 * struct; the functions below it are the library's own, seen through puldir_internal.h (D20). */
#ifndef PULDIR_ENABLE_H
#define PULDIR_ENABLE_H
#include <stdbool.h>
#include <stdint.h>
#include "puldir_pin.h"

typedef struct {
    puldir_pin_t pin;
    bool     inverted, attached;
    volatile bool enabled;
    uint32_t enabled_at;                 /* tick of the enable edge */
    bool     wait;                       /* the enable delay has not yet passed since enabled_at: hold */
    bool     release_due;                /* auto-release: the axis went IDLE at idle_at */
    uint32_t idle_at;
    bool     fault_hold;                 /* halted by a fault latch: no auto-release until a command (D12) */
    bool     off_due;                    /* enable(false) with the pulse pin high: the plan tick lowers
                                            the enable pin once the pulse pin is low */
    uint32_t enable_ticks, release_ticks; /* setDelays' delays, in pulse ticks */
    bool     auto_release;
} PulDirEnable;
#endif

#if defined(PULDIR_INTERNAL_FUNCTIONS) && !defined(PULDIR_ENABLE_FUNCTIONS)
#define PULDIR_ENABLE_FUNCTIONS
/* Main context, under the lock, for every accepted motion command: enables a disabled drive, whose
 * motion the plan tick then holds for the enable delay, and cancels a pending release. */
void puldir_enable_wake(PulDir*);
/* Plan tick, before the fault input: the enable delay ends, and a deferred disable lowers the
 * enable pin once the pulse pin is low. */
void puldir_enable_service(PulDir*, bool pul_high, uint32_t now);
/* Plan tick, after the axis ran: auto-release once the axis has been IDLE for the release delay. */
void puldir_enable_release(PulDir*, bool was_moving, uint32_t now);
/* Plan tick: the enable delay still holds the axis's motion. */
bool puldir_enable_delaying(const PulDir*);
/* Plan tick: a fault latch halted the axis, so set its fault hold: no auto-release until its next
 * accepted command (D12). */
void puldir_enable_set_fault_hold(PulDir*);
#endif
