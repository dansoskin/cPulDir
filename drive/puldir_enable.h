/* Enable and release (5.8): the enable pin, the enable delay and auto-release. Standalone: it needs
 * only puldir_port.h, and every time and duration comes in as pulse ticks. */
#ifndef PULDIR_ENABLE_H
#define PULDIR_ENABLE_H
#include <stdbool.h>
#include <stdint.h>
#include "puldir_port.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    puldir_pin_t pin;
    bool     inverted, attached;
    volatile bool enabled;
    uint32_t enabled_at;                 /* tick of the enable edge */
    bool     wait;                       /* the enable delay has not yet passed since enabled_at: hold */
    bool     release_due;                /* auto-release: the axis went idle at idle_at */
    uint32_t idle_at;
    bool     held;                       /* halted by a fault latch: no auto-release until a wake (D12) */
    bool     off_due;                    /* disabled with the pulse pin high: a later service lowers
                                            the enable pin once the pulse pin is low */
    uint32_t enable_ticks, release_ticks; /* the two delays */
    bool     auto_release;
} PulDirEnable;

/* Setup: the pin is parked inactive and the drive disabled. */
void puldir_enable_attach(PulDirEnable*, puldir_pin_t pin, bool inverted);
void puldir_enable_set_delays(PulDirEnable*, uint32_t enable_ticks, uint32_t release_ticks);
void puldir_enable_set_auto_release(PulDirEnable*, bool on);   /* off cancels a pending release */
/* For every accepted motion command: enables a disabled drive, whose motion then waits out the
 * enable delay, and cancels a pending release, hold or deferred disable. */
void puldir_enable_wake(PulDirEnable*, uint32_t now);
/* Disables the drive. With the pulse pin high the enable pin stays until a service sees it low. */
void puldir_enable_off(PulDirEnable*, bool pul_high);
/* Each plan interval, first: the enable delay ends, and a deferred disable lowers the enable pin
 * once the pulse pin is low. */
void puldir_enable_service(PulDirEnable*, bool pul_high, uint32_t now);
/* Each plan interval, last: releases the drive release_ticks after the axis went idle from a
 * move, unless it moved again or a fault holds it. */
void puldir_enable_release(PulDirEnable*, bool was_moving, bool idle, uint32_t now);
bool puldir_enable_delaying(const PulDirEnable*);   /* the enable delay still holds the motion */
void puldir_enable_hold(PulDirEnable*);              /* a fault halted the axis: no release until a wake */

#ifdef __cplusplus
}
#endif
#endif
