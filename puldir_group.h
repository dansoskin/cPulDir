/* Groups (5.7): several axes on one straight line. puldir.h includes this after PulDir; the
 * second half's functions are the library's own, seen through puldir_internal.h (D20). */
#ifndef PULDIR_GROUP_H
#define PULDIR_GROUP_H
#include <stdbool.h>
#include <stdint.h>
#include "puldir.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef PULDIR_MAX_GROUPS
#define PULDIR_MAX_GROUPS 4u
#endif

typedef struct PulDirGroup {
    PulDir* axes[PULDIR_MAX_AXES];
    uint8_t count;
    MotionProfile s;                     /* the path parameter, 0 to 1 */
    uint64_t start[PULDIR_MAX_AXES];     /* 32.32 */
    int32_t  delta[PULDIR_MAX_AXES];     /* whole pulses (D4) */
    volatile bool moving, faulted;
    volatile bool active;                /* the plan tick runs s: from an accepted move to its end or a halt */
    volatile uint8_t req;                /* main -> plan requests, under the lock */
    bool started;                        /* every member's direction was set; s advances */
    bool still;                          /* every delta is zero */
} PulDirGroup;

/* D16: refuses. The group's table keeps a pointer to it: give it static storage. */
bool  PulDirGroup_init(PulDirGroup*, PulDir** axes, uint8_t count);
/* Refuses a member not IDLE or faulted, and a member travel beyond 2^24 pulses.
 * While the move runs, members refuse their settings' setters (R_MODE). */
bool  PulDirGroup_moveTo(PulDirGroup*, const float* targets);
/* Ramped along the line. After a stop mid-path each member ends on its last whole
 * pulse, within one pulse of the line; the final edges need not share a tick. */
void  PulDirGroup_stop(PulDirGroup*);
void  PulDirGroup_halt(PulDirGroup*);
bool  PulDirGroup_isMoving(PulDirGroup*);
bool  PulDirGroup_faulted(PulDirGroup*);

#ifdef __cplusplus
}
#endif
#endif

#if defined(PULDIR_INTERNAL_FUNCTIONS) && !defined(PULDIR_GROUP_FUNCTIONS)
#define PULDIR_GROUP_FUNCTIONS
void puldir_group_tick(const PulDirSnap* snap, PulDirInc* out);   /* every group, before the axes */
/* Under the lock, from main (shadow NULL) or the plan tick (shadow, the set it writes, zeroed too):
 * no rising edge on any member after this tick, every member IDLE, the move abandoned. */
void puldir_group_halt(PulDirGroup*, PulDirInc* shadow);
#endif
