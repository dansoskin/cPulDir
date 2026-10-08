/* The fault input and its clear pulse (5.8): the debounce, the latch and the clear output.
 * Standalone: it needs only puldir_port.h, and every time and duration comes in as pulse ticks. */
#ifndef PULDIR_FAULT_H
#define PULDIR_FAULT_H
#include <stdbool.h>
#include <stdint.h>
#include "puldir_port.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Defaults for callers that count in ms; the module itself takes durations in ticks. */
#ifndef PULDIR_FAULT_DEBOUNCE_MS
#define PULDIR_FAULT_DEBOUNCE_MS 2u
#endif
#ifndef PULDIR_FAULT_CLEAR_MS
#define PULDIR_FAULT_CLEAR_MS 10u
#endif

typedef struct {
    puldir_pin_t pin, clr_pin;           /* the alarm input, the clear output */
    bool     inverted, attached;
    bool     clr_inverted, clr_attached;
    uint32_t debounce_ticks, clear_ticks; /* the input must stay active this long; the pulse width */
    volatile bool latched;
    bool     seen;                       /* the last sample read active */
    uint32_t since;                      /* the first sample of the current active run */
    uint32_t count, last_tick;           /* latches so far, and the sample tick of the last */
    bool     clearing;                   /* the clear pulse is high since clr_t0 */
    uint32_t clr_t0;
    bool     clr_armed;                  /* a clear pulse ended on a latch: the first sample that
                                            reads the input inactive lifts it */
} PulDirFault;

/* What a service did to the latch. */
typedef enum { PULDIR_LATCH_SAME, PULDIR_LATCH_CLOSED, PULDIR_LATCH_DROPPED } PulDirLatch;

/* Setup. The clear output is parked inactive. */
void puldir_fault_attach(PulDirFault*, puldir_pin_t pin, bool inverted, uint32_t debounce_ticks);
void puldir_fault_attach_clear(PulDirFault*, puldir_pin_t pin, bool inverted, uint32_t clear_ticks);
/* Raises the clear pin; a later service lowers it. False, changing nothing, while a pulse runs. */
bool puldir_fault_clear(PulDirFault*, uint32_t now);
bool puldir_fault_latched(const PulDirFault*);
bool puldir_fault_sample(const PulDirFault*);       /* the alarm input reads active */
/* Each plan interval: ends the clear pulse (now), or debounces the sample taken at t_pin and
 * closes the latch, or drops it after a clear once the input reads inactive. */
PulDirLatch puldir_fault_service(PulDirFault*, bool active, uint32_t t_pin, uint32_t now);

#ifdef __cplusplus
}
#endif
#endif
