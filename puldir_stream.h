/* Stream mode (5.6): setpoints followed through a first-order hold. PulDir embeds the struct;
 * the functions below it are the library's own, seen through puldir_internal.h (D20). */
#ifndef PULDIR_STREAM_H
#define PULDIR_STREAM_H
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float    sp_pos, sp_vel;             /* pulses, pulses/s; the last setpoint */
    uint32_t sp_tick;                    /* its arrival tick, or streamBegin's before the first */
    bool     have;                       /* a setpoint has arrived since streamBegin */
    bool     live;                       /* plan tick: the generator is idle under the stream */
    volatile bool timed_out;
    float    v_cmd;                      /* pulses/s */
    volatile float err;                  /* pulses: p_hold - P at the boundary */
    uint32_t catchup_ms;
} PulDirStream;
#endif

#if defined(PULDIR_INTERNAL_FUNCTIONS) && !defined(PULDIR_STREAM_FUNCTIONS)
#define PULDIR_STREAM_FUNCTIONS
/* Plan tick, streamBegin's request, before the limits: the generator idles where the pulses are. */
void puldir_stream_begin(PulDir*, int64_t pb);
/* Plan tick, after the limits and the datum: leaves STREAM for STOPPING after streamEnd or stop,
 * or on the setpoint timeout. */
void puldir_stream_check(PulDir*, int64_t pb, uint32_t now);
/* Plan tick: one interval of STREAM, aimed at t_b, where this set takes over. */
void puldir_stream_run(PulDir*, PulDirInc* out, uint64_t base, int64_t pb, int32_t count, uint32_t t_b);
#endif
