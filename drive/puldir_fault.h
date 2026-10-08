/* The fault input and its clear pulse (5.8). PulDir embeds the struct; the functions below it
 * are the library's own, seen through puldir_internal.h (D20). */
#ifndef PULDIR_FAULT_H
#define PULDIR_FAULT_H
#include <stdbool.h>
#include <stdint.h>
#include "puldir_pin.h"

typedef struct {
    puldir_pin_t pin, clr_pin;           /* the alarm input, the clear output */
    bool     inverted, attached;
    bool     clr_inverted, clr_attached;
    volatile bool latched;
    bool     seen;                       /* the last sample read active */
    uint32_t since;                      /* the first sample of the current active run */
    uint32_t count, last_tick;           /* latches so far, and the sample tick of the last */
    bool     clearing;                   /* the clear pulse is high since clr_t0 */
    uint32_t clr_t0;
    bool     clr_armed;                  /* a clear pulse ended on a latch: the first sample that
                                            reads the input inactive lifts it */
} PulDirFault;
#endif

#if defined(PULDIR_INTERNAL_FUNCTIONS) && !defined(PULDIR_FAULT_FUNCTIONS)
#define PULDIR_FAULT_FUNCTIONS
/* Plan tick, before the snapshot lock: the alarm input reads active. */
bool puldir_fault_sample(const PulDir*);
/* Plan tick, after the enable service: the clear pulse ends, the sample taken at t_pin is debounced
 * and the latch closes. True if it just closed: the caller halts what it stopped. */
bool puldir_fault_service(PulDir*, bool active, uint32_t t_pin, uint32_t now);
/* The latch is closed: motion is refused. Under the lock wherever a refusal depends on it (D17). */
bool puldir_fault_latched(const PulDir*);
#endif
