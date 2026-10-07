# Guidelines for the PUL/DIR axis library (puldir)

Status: guidelines, 2026-10-07, revised the same day after the design review
recorded in `docs/superpowers/specs/2026-10-07-puldir-design.md`. Nothing
described here is implemented yet.

Written for the worker who builds the library in a fresh session. It assumes
no knowledge of the firmware that will use it. The one thing to read first is
the library being replaced, `cFlexyStepper`, which lives in the firmware at
`C:/Users/dans/Documents/code/164-race_firmware/Drivers/cFlexyStepper/`
(github.com/dansoskin/cFlexyStepper): `cFlexyStepper.h` for the concepts,
`cFlexyStepper.c` for the old step algorithm, `cFlexyStepper_passthrough.c`
for the first attempt at streaming, `cFlexyStepper_homing_sm.c` for homing.

The library is called **puldir** because that is all it assumes about the
drive: a pulse input and a direction input. Steppers, closed-loop stepper
drives and servo drives with PUL/DIR inputs are all the same thing to it.
The word "stepper" does not appear in its code. A "pulse" is one edge the
drive counts; a "unit" is whatever the caller's conversion makes of pulses.

---

## 1. Purpose

A library that drives **several PUL/DIR drives from one microcontroller**,
each with its own acceleration, deceleration and jerk limits, that can move a
group of them in lock-step, and that can follow a stream of position
setpoints sent every millisecond. Every pulse comes from an interrupt. It
runs on STM32 first (any family, not one part) and on the Arduino framework
next.

Alongside the axis code, and independent of it, the library ships a
**profile generator**: the ramp maths on its own, in two files, with no
dependency on anything else. Callers that need a smooth, ramped setpoint
over time but have no drive behind it use the generator directly. The axis
uses the same generator internally, so there is one implementation of the
ramps.

There is no encoder anywhere: the commanded pulse count is the only position
the library knows. The drives close their own loop.

## 2. Requirements

### 2.1 Stated by the owner

1. **Acceleration, deceleration and jerk.** Asymmetric accel/decel, plus a
   jerk limit. Jerk `0` means "unlimited" and must reproduce the trapezoid
   the current library produces.
2. **Same concepts as cFlexyStepper, new names.** Conversion to user units,
   defaults and restore, the status enum, release after a move, homing, the
   fault pin, the log sink, the `@` console. Names change freely; section
   6.2 maps every old name to its new one, and that map is the migration
   plan for the firmware that uses it. Nothing else about that firmware
   matters here.
3. **Interrupt driven.** The main loop only sends commands and reads state.
   A stalled main loop may stop *new* commands from arriving, never a pulse
   from being on time, and never a safety stop from happening.
4. **Synchronised multi-axis moves.** A group of axes given one target each
   starts together, moves along a straight line in joint space, and emits
   its final pulses on the same tick.
5. **CSP mode.** A caller streams `(position, velocity)` every millisecond
   or slower and the axis follows it directly, with no planner in the way.
6. **Clean.** No dead code, no `unused/` directory, no unit wrappers, no
   feature kept "in case". Short comments. Refuse invalid input instead of
   clamping it, and say why on the log sink.
7. **STM32 first, Arduino next.** The core is plain C with no platform
   includes. One small port per platform, and the STM32 port must not assume
   a family.
8. **The profile generator stands alone.** `motion_profile.h` and
   `motion_profile.c`, pure maths, no statics, no callbacks, no includes
   beyond `<stdint.h>`, `<stdbool.h>`, `<math.h>`. It compiles and tests on
   a PC by itself. The axis depends on it; it depends on nothing.

### 2.2 Decisions the owner did not state

These came up in the previous implementations or in review and must be
decided, not discovered. Build the defaults below unless the owner says
otherwise.

| topic | decision |
|---|---|
| where jerk lives | in the profile generator, once, as a moving-average filter on the trapezoid's velocity (5.2). The axis has jerk because it uses the generator. In stream mode the axis's own ramps are not applied, so a streamed motion is only as smooth as the generator that produced the stream |
| limits during motion | the generator never exceeds its limits except in the one documented landing call (5.2). A lowered speed limit is ramped down to, not clamped to. A target inside the stopping distance is reached by passing it, stopping under the limits and returning |
| homing | keep the current phase machine, run from the plan tick (5.8); a failed or timed-out run **halts** the axis, which today's library does not |
| jog | "run at this speed until told otherwise", speed changeable live; it must **not** reset the position counter, which today's does |
| enable / release | a motion command enables the drive and waits `enable_ms` before the first pulse; with auto-release on, the drive is disabled `release_ms` after the last pulse; `PulDir_enable(false)` during motion halts first |
| halt vs stop | `halt` zeroes velocity this tick and discards the plan; `stop` plans a stop at decel and jerk from the current state. Both leave target = where the axis ended |
| live retarget / override | `moveTo`, `moveBy`, `jog`, `setSpeed`, `setAcceleration`, `setDeceleration` all take effect on the move in progress. `setJerk` is the one exception: it changes the filter length, so it is refused unless the axis is idle (5.2) |
| stream loss | velocity feedforward keeps an axis moving after the stream dies, so a stream **timeout** stops the axis at decel and the caller must `streamBegin` again |
| datum | `setPosition` is refused unless idle; it is the only way to move the datum, and it clears `isHomed` |
| fault pin | sampled and debounced in the plan tick (2 ms), halt on latch, clear pulse 10 ms timed in the plan tick; nothing blocks |
| logging / status | log sink as today, called only from `PulDir_loop` in the main loop; the ticks raise flags |
| velocity readback | exactly `0.0f` when stopped |
| `isMoving` | true from an accepted motion command until the final pulse has been lowered; not derived from velocity |
| `@` console | kept, verb for verb, as the bring-up tool |
| polarity | pulse active-high or active-low, and direction sense, both **per axis**, given with the pins. Today's library has a global compile-time direction macro and no pulse polarity |
| pull-in speed | **not built.** Closed-loop drives start from zero under the ramps. If an open-loop stepper ever crawls at the ends of moves, add `vstart` to `MotionLimits` (default 0): the profile starts at it from standstill and stops from it; nothing else changes |
| pulse width | not a setting of its own: it is one pulse tick, so the tick rate is the pulse width setting (5.3) |
| what does not exist | no encoder, no following error, no multi-block look-ahead, no G-code, no RTOS, no dynamic allocation, no completion callbacks (callers poll), no caller-supplied timestamps on stream setpoints |

### 2.3 Numbers the design must meet

| | |
|---|---|
| axes | 4 on the first machine; compile-time maximum, default 8 |
| pulse rate | **just under 100,000 pulses/s per axis with a 5 µs pulse, on all axes at once**, when the build asks for it (5.3). The first machine needs under 20,000 and will run the default tick |
| typical rates | 1,400 to 8,500 pulses/s on the first machine; conversions around 280 pulses per user unit; limits of 15 units/s, 30 units/s², 30 units/s². Servo drives at 10,000 pulses/rev and 10 rev/s are the 100,000 case |
| pulse timing | drives are closed-loop servo drives with PUL/DIR inputs; their timing requirements are not documented and a 2 µs pulse works today. Section 5.3 lists what the design guarantees; the port validates it against the drive's datasheet |
| streaming | the first consumer streams at 200 Hz; the library must accept **1 kHz**, and any rate up to the plan tick rate |
| CPU | ≤ 10 % of a Cortex-M4F class core for four axes at the default tick; ≤ 25 % at the 100,000 pulses/s configuration, interrupts included. These are estimates until measured on the bench (section 9, step 4) |
| first MCU | a Cortex-M4F STM32 with FPU at 144 MHz; integer-only in the pulse interrupt regardless |

## 3. What the previous implementations taught

Weigh these by how much they were used. cFlexyStepper (3.1) ran a machine for
months. The passthrough (3.2) and synchronized_movement (3.3) were **never
tested in depth**: neither was characterised on hardware, so nothing taken
from them is proven. Use them for the shape of a problem, re-derive the
answer, and prove it under section 8.

### 3.1 cFlexyStepper (`Drivers/cFlexyStepper/`)

A C port of the Arduino FlexyStepper. Each axis has a per-step period
recurrence (`DeterminePeriodOfNextStep`: `next = cur − a·cur³` speeding up,
`next = cur + d·cur³` slowing down) and `processMovement` polls a microsecond
clock from the main loop, emits a pulse when the period has elapsed and
busy-waits 2 µs for the pulse width.

What it got right, keep all of it:

- user units through one `conversion`, every public value in user units,
  the pulse count private;
- asymmetric acceleration and deceleration;
- `setDefaults` / `restoreDefaults`;
- the status enum `IDLE / ENABLED / MOVING / FAULT` and its meaning;
- release after a move;
- the homing phase machine and its `did_home` flag;
- fault pin with debounce and a clear pulse;
- a log sink with per-axis enable, and a name per axis;
- the `@` console (`cFlexyStepper_decode.c`).

What it got wrong, none of which may survive:

- **Pulse timing from the main loop.** A pulse is late by whatever ran
  before it. Four axes are polled one after another so two drives never
  pulse at the same instant. At 20 kHz a late pulse is a missed one.
- **Velocity is not a state.** It is `1/period` of the last pulse, so
  retargeting, speed overrides and streaming all approximate. Reading the
  velocity of a slow axis returns a stale number.
- `setAcceleration` resets deceleration (every caller learned to set decel
  second).
- `jog` moves to ±9,999,999 units and **zeroes the position counter**.
- Homing phases have 120 s budgets and a timeout leaves the motor jogging into
  the hard stop; `stop_cFlexyStepper_homing_sm` exists because of that.
- The stop threshold had to be derived from the same ramp as the clamp or the
  axis overshot, reversed and oscillated around the target (see the comment in
  the struct). A design where position is integrated from velocity cannot have
  this bug.
- Blocking 10 ms fault-clear pulse; a microsecond busy-wait in the pulse path.
- Millimetre and revolution wrappers, a timed-move solver
  (`setTimedTargetPosition`, `solveTimedMove`) nobody calls, an `unused/`
  directory, platform `#ifdef`s inside the main struct.

### 3.2 The passthrough (`cFlexyStepper_passthrough.c`), CSP version 1

CSP was bolted onto the trapezoid planner. Every streamed setpoint computed a
velocity

```
v = |rate| · conversion + error / 0.05
v = min(v, max(feedforward, defaultSpeed · conversion))
v = max(v, 1.0)
```

and then set `accel = decel = v_ramp² / 2`, which makes the ramp to `v`
exactly one pulse long — the planner reached full speed on the first pulse,
which turned it into direct velocity control by brute force. It drove a
200 Hz stream on the bench, but its tracking error, lag and behaviour at
speed were never measured. Treat it as a sketch of the controller, not a
reference. What it suggests:

- the shape of the controller is right: **velocity feedforward plus a
  proportional catch-up on position error**, catch-up time constant 50 ms;
- the `1.0` pulse/s floor caused a one-second stall at the start of every
  stream (velocity 1 pulse/s until the error grew) — **no floors**;
- logging had to be muted inside it because the planner logged every retarget
  — the stream must be a mode the planner is not part of;
- `endPassthrough` sets target = current and restores defaults — keep that
  exit contract in spirit: after leaving the stream the axis comes to rest
  and its defaults are back;
- it had no stream timeout because the trapezoid planner finished the last
  move and stopped. With real velocity feedforward that safety net is gone
  and must be put back explicitly.

### 3.3 synchronized_movement (deleted; `github.com/dansoskin/synchronized_movement`)

Given per-axis `{current, target, max_speed, acceleration}`, it found the
bottleneck axis (longest minimum trapezoid/triangle time, one acceleration for
both ramps) and solved `v² − a·T·v + a·d = 0` for every other axis so each
independent trapezoid ended at the same time. Its maths passed a 200k-case
host harness (which found one rounding bug on the bottleneck's discriminant),
but it was never run against motors in depth, so its arrival-time claim is
unverified on hardware and it was deleted before it could be.

What reading it shows, not what measuring it showed:

- it synchronises **arrival only**; the axes follow different ramps so the
  joint-space path between start and end is a curve, not a line;
- it cannot survive a retarget or speed change mid-move;
- it assumed symmetric ramps and no jerk.

So: synchronise by sharing **one profile** across the group, not by solving
per-axis speeds. Section 5.7.

## 4. What the surveyed libraries do, and what to borrow

| library | pulse generation | sync | ramps | platform | licence |
|---|---|---|---|---|---|
| TeensyStep | one interval timer plus two timer channels per controller, 300 k pulses/s | lead axis + Bresenham, up to 10 motors | linear only | Teensy 3.x only | MIT |
| FastAccelStepper | ramp generator → queue of `{steps, ticks, dir}` → hardware ISR, no floats in the ISR | experimental | linear; "cubic" start option; speed changes apply on the next `move()` | ESP32, AVR, Pico, SAMD | MIT |
| GRBL `stepper.c` | fixed ~30 kHz ISR; planner emits constant-velocity **segments** (~10 ms); Bresenham across axes; AMASS oversampling at low rates; second timer ends the pulse | by construction (one segment stream) | trapezoid with junction look-ahead | AVR | **GPLv3** |
| Ruckig | not a pulse library: online jerk-limited trajectory generation from any `(p, v, a)` state | n/a | full S-curve, time-optimal | host C++ | MIT (community) |

Borrow:

- **GRBL's structure**: a slow planner emits displacement per interval, a
  fast fixed-rate ISR integrates it and toggles pins, every axis in the same
  ISR so synchronisation is free. Direction is written before the pulse, the
  pulse is ended later, never busy-waited.
- **FastAccelStepper's ISR discipline**: integer only, a handful of
  instructions per axis.
- **TeensyStep's API split**: moves to a target vs. running at a speed with
  live override, an emergency halt, a group object for synchronised moves,
  per-axis pulse and direction polarity.
- **Ruckig's documentation** for the cases an online jerk-limited planner
  must handle from non-zero acceleration. Read it; do not port it.

Do not borrow:

- GRBL code (GPL). Ideas only.
- Bresenham on a lead axis: a fixed-tick accumulator per axis gives the same
  synchronisation with no lead axis and no AMASS.
- FastAccelStepper's "new speed applies on the next move" — we need live.
- Any of their timer bindings.

## 5. Architecture

### 5.1 Four layers

```
 motion_profile.{h,c}     pure maths: (state, limits, target) -> next state at the dt fixed at init. No dependencies.
         ▲
 plan tick (PLAN_HZ IRQ)  per axis: read pulse position, run the profile / stream / group / homing /
                          fault logic, publish the displacement for the next interval
         ▲
 pulse tick (TICK_HZ IRQ) per axis: pos += inc; pulse when the integer part changes. Integer only.
         ▲
 port                     starts the two periodic callbacks, reads and writes pins, masks the library's IRQs

 main context             PulDir_*() commands and getters; PulDir_loop() for logging only
```

Dependencies point one way: port ← pulse tick ← plan tick ← profile. The
profile generator knows nothing above it.

### 5.2 The profile generator

The owner calls it the profiler. It is the whole of the ramp maths and the
only place jerk is implemented. It is two things in one struct: an
acceleration-limited online planner, the **trapezoid**, and a moving-average
**jerk filter** on the trapezoid's velocity. The caller sees only the
filtered output. The review that chose this over an online S-curve planner is
recorded in `docs/superpowers/specs/2026-10-07-puldir-design.md`.

```c
#ifndef MOTION_PROFILE_MAX_TAPS
#define MOTION_PROFILE_MAX_TAPS 256       /* the build raises it for softer jerk */
#endif

typedef struct {
    float vmax, amax, dmax, jmax;     /* jmax 0 = unlimited (trapezoid) */
} MotionLimits;

typedef struct {
    float dt;                         /* seconds per advance, fixed at init */
    float pos, vel, acc;              /* the output: filtered, caller's units */
    float target;
    bool  endless;                    /* run at a speed, no target */
    bool  stopping;                   /* target replaced by "stop as soon as the ramps allow" */
    bool  done;                       /* at target with vel == 0.0f exactly */
    MotionLimits lim;
    /* private: the trapezoid and the jerk filter */
    float    p_t, v_t;                /* the trapezoid's own position and velocity */
    float    taps[MOTION_PROFILE_MAX_TAPS];   /* ring of the last n values of v_t */
    uint16_t n, head;
    float    sum;
} MotionProfile;

void  MotionProfile_init(MotionProfile*, float pos, float dt);
bool  MotionProfile_setLimits(MotionProfile*, MotionLimits);         /* refused if not finite or < 0, or vmax/amax/dmax == 0; a changed jmax refused unless done */
bool  MotionProfile_setTarget(MotionProfile*, float target);         /* live; any finite target */
bool  MotionProfile_setEndless(MotionProfile*, float speed);         /* live; sign = direction */
void  MotionProfile_stop(MotionProfile*);                            /* plan a stop at dmax, jmax */
void  MotionProfile_halt(MotionProfile*);                            /* vel = acc = 0 now, target = pos */
bool  MotionProfile_setPosition(MotionProfile*, float pos);          /* datum; refused unless done */
void  MotionProfile_advance(MotionProfile*);                         /* one step of dt seconds */
float MotionProfile_stopDistance(const MotionProfile*);              /* from the output position, signed */
```

**The trapezoid.** Per `advance`, with `rem = target − p_t` signed:

1. A velocity ceiling: the largest velocity that, after this step, leaves
   exactly its own stopping distance. With a target,
   `v_c = sign(rem) · min(vmax, sqrt((dmax·dt)² + 2·dmax·|rem|) − dmax·dt)`;
   endless, `v_c = speed`; stopping, `v_c = 0`. The discrete form, not
   `sqrt(2·dmax·|rem|)`, is what makes the descent land without a residual;
   do not simplify it.
2. `v_t` moves toward `v_c` by at most `amax·dt` when its magnitude grows and
   `dmax·dt` when it shrinks, and never crosses zero within one call: it lands
   on zero and reverses on the next.
3. `p_t += v_t·dt`.
4. **Landing.** When `|rem| ≤ |v_t|·dt` and `|v_t| ≤ dmax·dt`, this call uses
   `v_t = rem / dt` and then assigns `p_t = target` (the one place `p_t` is
   assigned rather than integrated) and `v_t = 0`. The call changes `v_t` by
   at most `dmax·dt`, so it is not an exception to any limit.

The ceiling is a monotone function of the remaining distance, so there is no
accelerate/decelerate flag to flip at the switching point: the rule cannot
chatter. Following the ceiling, the remaining distance equals
`v_t² / (2·dmax)` after every step until the landing rule takes over. A
target inside the stopping distance, or behind the axis, makes the ceiling
fall faster than `dmax` allows; `v_t` comes down at `dmax`, passes the
target, `rem` changes sign and the same rule brings it back. "Never passes
its target" holds for targets that were reachable when they were set, and
only for those. A lowered `vmax` lowers the ceiling and `v_t` follows it at
`dmax`; a lowered `amax` or `dmax` applies from the next call; a raised
limit applies immediately. `stop` sets the ceiling to zero and clears
`endless`; when `v_t` reaches zero the target becomes `p_t`.

**The jerk filter.** `n = ceil((amax + dmax) / (jmax · dt))`, or 1 when
`jmax = 0`. Each `advance` pushes the new `v_t` into the ring, drops the
oldest, and sets

```
vel  = sum / n
acc  = (vel − vel_previous) / dt
pos += vel · dt
```

The filtered acceleration is `(v_t(t) − v_t(t − n·dt)) / (n·dt)`, so it stays
within `[−dmax, +amax]`. The filtered jerk is at most
`(amax + dmax) / (n·dt) ≤ jmax`, including the acceleration flip of a
triangle move or a retarget, which is why the window is `amax + dmax` and not
the larger of the two. The average of values no larger than `vmax` is no
larger than `vmax`. With `jmax = 0` the ring has one tap and the output is
the trapezoid itself.

`done` becomes true `n` calls after the trapezoid landed. On that call
`sum = 0`, `vel = acc = 0.0f` exactly and `pos = target`: the correction is
the rounding of `n` additions, below a thousandth of a pulse, and it is the
only assignment to `pos`. `MotionProfile_stopDistance` is
`(p_t − pos) + v_t·|v_t| / (2·dmax)`: what the ring still has to deliver,
plus the trapezoid's own stopping distance. `halt` clears the ring, zeroes
`v_t`, `vel`, `acc` and `sum`, and sets `target = pos`. `setPosition` moves
`pos`, `p_t` and `target` together.

A move with jerk therefore finishes `n·dt` after its trapezoid would, and is
jerk-bounded rather than time-optimal. That is the price of the approach, and
it is accepted.

Rules for the generator:

- float only, no `double`, no statics, no allocation, no logging; refusals
  return `false` and change nothing;
- `dt` is fixed at `init`, from 1 ms to 10 ms, because the filter length
  depends on it; results must not depend on it beyond the discretisation
  (test 1);
- a `jmax` that changes `n` is refused unless `done`: jerk is the one limit
  that is not live. An `n` above `MOTION_PROFILE_MAX_TAPS` is refused, and
  the caller's log line states the smallest jerk that fits,
  `(amax + dmax) / (MOTION_PROFILE_MAX_TAPS · dt)`;
- unit-agnostic: the caller decides what a unit is. The axis feeds it
  pulses; a bare caller feeds it degrees or millimetres;
- `vel` is exactly `0.0f` whenever `done`;
- the trapezoid case (`jmax = 0`) must match what cFlexyStepper produces for
  the same speed/accel/decel within one pulse and one `dt`;
- **precision**: `pos` is a float32, so a bare caller must keep its
  coordinates within about seven significant digits (past 2²⁴ units every
  integer is no longer representable). The axis never lets the generator see
  a large number: section 5.4 re-bases it every interval;
- **memory**: a profile is `4 · MOTION_PROFILE_MAX_TAPS` bytes plus a few
  words, 1 KB at the default. The first machine has four axes, two bare
  generators and at most one group: under 8 KB.

### 5.3 The pulse tick

Runs at `TICK_HZ`, a build-time constant (default 50 kHz). Each axis's
pulse position is a 64-bit fixed-point number (32.32: pulses and fraction,
stored unsigned so that wrap is defined and reinterpreted as signed). Per
tick:

1. If a new increment set was published since the last tick, copy the whole
   set into the live increments before touching any axis, so **every axis
   switches increments on the same tick** (5.7).
2. Per axis: if it raised its pulse pin last tick, lower it.
3. Per axis: if the sign of `inc` differs from the axis's current direction,
   and no pulse was raised or lowered on this tick for that axis, write the
   direction pin and **skip the rest for this axis this tick**.
4. Per axis: `pos += inc`. If the integer part changed, raise the pulse pin.
5. Increment the tick counter.

Because `|inc| ≤ 0.5 pulse/tick`, no axis pulses on consecutive ticks, and
the rules above guarantee the following for every tick rate. The port
validates them against the drive's datasheet before the first bench run:

| requirement | guaranteed |
|---|---|
| pulse width | one tick: the pin goes high in one interrupt and low in the next. On a scope it reads a few tens of nanoseconds under the tick period, the cycles between the lower and the raise inside the handler |
| pulse inactive time | at least one tick |
| direction setup before the next pulse | at least one tick |
| direction hold after the last pulse | at least one tick (a direction change never shares a tick with a pulse edge) |
| enable to first pulse | `enable_ms`, a per-axis setting (5.8) |
| last pulse to disable | `release_ms`, a per-axis setting (5.8) |
| inter-axis edge skew | within one interrupt, under a microsecond; all axes see the same increment set from the same tick |

The cost is an int64 add and two compares per axis per tick, about 20
cycles, plus about 50 cycles of interrupt overhead per tick, so CPU load is
proportional to the tick rate and not to the actual pulse rate:

| `TICK_HZ` | pulse | max pulses/s per axis | four axes on a 144 MHz M4F, estimated |
|---|---|---|---|
| 50 kHz (default) | 20 µs | just under 25,000 | ~5 % |
| 100 kHz | 10 µs | just under 50,000 | ~10 % |
| 200 kHz | 5 µs | just under 100,000 | ~20 % |

`TICK_HZ` is a build-time constant the application sets; the port only has
to honour it. The 200 kHz row is a requirement: it must work, with the host
suite run at that setting (test 23). The first machine will use the default.
There is no separate pulse width setting: a drive that needs a 40 µs pulse
is run at a 25 kHz tick. Pulse and direction pins are written through the
axis's polarity, and a pulse pin idles at its inactive level.

`inc` is `int32`, in units of 2⁻³² pulse per tick:
`inc = v_pulses_per_s / TICK_HZ × 2³²`. The largest magnitude is `2³¹ − 1`,
which is one part in two billion **below** half a pulse per tick, so the
ceiling is just under `TICK_HZ / 2` and `setSpeed` refuses anything at or
above `TICK_HZ / 2 / conversion` with a log line.

Hardware pulse generation (a timer compare or DMA per axis) would need a
different port contract, a schedule of edges rather than immediate pin
writes. It is out of scope and nothing in the core should be shaped for it.

### 5.4 The plan tick

Runs at `PLAN_HZ`, a build-time constant (default 1 kHz, never lower), at a
lower interrupt priority than the pulse tick, and may use the FPU. It costs
a few hundred cycles per axis per run, so 2 kHz is affordable when a
consumer streams at 1 kHz and wants the setpoint used within half a
millisecond. Keep three quantities distinct throughout:

- the **profile position**: float, in the generator, local to an origin;
- the **pulse position**: 64-bit fixed point, in the pulse tick;
- the **pulses emitted**: the integer part of the pulse position.

Per axis, per run:

1. Snapshot the pulse position `P` (64-bit, under the port lock).
2. **Re-base the generator** on the integer part of `P`: its `pos` becomes
   the fractional part, its trapezoid position and its `target` shift by the
   same origin, `target` saturated to ±2²⁴ pulses. The filter ring holds
   velocities and is untouched. The generator therefore never sees a number
   larger than the remaining distance, and the remaining distance is computed
   in integer arithmetic first.
3. Run the mode (5.5–5.8) to get the profile position at the end of the
   next interval. In PROFILE and STOPPING that is
   `MotionProfile_advance(&axis->prof)`, the profile having been initialised
   with `dt = 1.0f / PLAN_HZ`; the profile position is its filtered `pos`.
4. The displacement to publish is `Δ = profile position − P`, in fixed
   point: the displacement that brings the pulse position onto the profile
   by the end of the interval. The increment is `inc = Δ / n` truncated
   toward zero, with `n = TICK_HZ / PLAN_HZ`. The truncation remainder, and
   any tick the pulse tick skipped for a direction change, are simply part
   of the next interval's `Δ`, so the pulse position never drifts from the
   profile by more than one increment.
5. On the interval in which the generator reports `done`, round `inc` away
   from zero instead, so that the integer boundary of the target is crossed.
   **Exactness means the pulses emitted equal the target's integer pulse
   count.** The fraction left behind is below 2⁻²⁰ pulse and is not
   corrected.
6. Store `inc` into the shadow increment set and the velocity (float) for
   the getters; after the last axis, publish the set (one flag write).

The plan interval is the resolution of acceleration changes; the pulse tick
resolves the pulses. A port with one periodic source may call the plan tick
from every `n`-th pulse tick; the core must not care. That fallback is only
for slow ticks: the plan maths for four axes takes several microseconds,
which fits inside a 20 µs tick and overruns a 5 µs one, so the 200 kHz
configuration needs two periodic sources.

### 5.5 Modes and transitions

```
IDLE        inc = 0, nothing pending
PROFILE     the generator drives the axis toward a target, or endlessly (jog)
STOPPING    PROFILE after stop; also where every timeout and streamEnd goes
STREAM      following streamed setpoints (5.6)
GROUP       position slaved to a group's path parameter (5.7)
HOMING      the homing phases (5.8), running the generator endlessly
```

Status, as today, is `IDLE / ENABLED / MOVING / FAULT`: drive disabled →
IDLE; enabled and not moving → ENABLED; moving → MOVING; fault latched →
FAULT regardless. `isMoving` is true from the acceptance of a motion command
until the final pulse has been lowered, which covers the enable delay, the
first jerk ramp with near-zero velocity, a reversal and the last pulse.

Transition rules, which the host tests exercise case by case (test 19):

| event | rule |
|---|---|
| `halt` | accepted in every mode. Mode → IDLE. A group member's halt halts the whole group |
| `stop` | accepted in every mode; a no-op in IDLE. Mode → STOPPING |
| `moveTo`, `moveBy`, `jog` | accepted in IDLE, PROFILE, STOPPING; refused in STREAM, GROUP, HOMING |
| `streamBegin` | accepted in IDLE, PROFILE, STOPPING (from the current velocity); refused in GROUP, HOMING |
| `stream` (a setpoint) | used only in STREAM; otherwise dropped and the refused flag is raised. It never starts a stream |
| `PulDirGroup_moveTo` | refused if any member is not IDLE, is faulted, or is in another group's move |
| `homeStart` | accepted only in IDLE with the drive not faulted |
| `enable(false)` | halts first if moving, then disables |
| `setPosition` | accepted only in IDLE; clears `isHomed` |
| fault latch | halts the axis; if it is a group member, halts the group and marks the group faulted; clears `isHomed` |
| stream timeout, homing timeout | mode → STOPPING at the axis's decel; a flag records why |
| `isHomed` | set at the end of a successful homing run; cleared by `homeStart`, a fault latch, and `setPosition` |

Independent axes, several groups and streaming axes all run concurrently;
nothing in the library serialises them.

### 5.6 Stream mode (CSP)

```
PulDir_streamBegin(axis);                  /* from standstill or mid-move */
PulDir_stream(axis, position, velocity);   /* every ms or slower, user units */
PulDir_streamEnd(axis);                    /* stop from the current state  */
```

Each setpoint records `(p_sp, v_sp, t_sp)`, with `t_sp` the arrival tick
from the library's own tick counter. There are no caller timestamps: the
caller is on the same chip and a setpoint describes where the axis should
be *now*. A setpoint costs the caller one lock and three stores. It is used
by the next plan tick, so the stream rate may be anything up to `PLAN_HZ`.
Every plan tick:

```
p_hold = p_sp + v_sp · (t − t_sp)                 first-order hold
v_cmd  = v_sp + (p_hold − P) / T_CATCHUP         feedforward + proportional catch-up
v_cmd  = clamp(v_cmd, ±vmax)                     the configured speed limit, and the tick ceiling
```

What applies in STREAM and what does not, stated once:

- the axis's configured speed limit applies, and the hard tick ceiling;
- the axis's acceleration, deceleration and jerk limits do **not** apply.
  The catch-up term acts on error and can demand accelerations beyond
  anything in the source profile; that is by design;
- lag is bounded by one plan interval plus pulse quantisation. The hold
  removes the sample-and-hold lag of the previous implementation, nothing
  more.

Contract:

- `T_CATCHUP` is `PULDIR_CATCHUP_MS`, default 50, a compile-time default
  the caller may override per axis. It must stay at least five plan intervals and two stream periods
  long or the catch-up loop oscillates. The value is inherited from the
  untested passthrough; tune it under test 12, do not trust it.
- **No floor on `v_cmd`**, no logging per setpoint.
- Between `streamBegin` and the first setpoint the axis keeps the velocity
  it had, and the stream timeout runs from `streamBegin`.
- The library does not smooth a first setpoint that disagrees with the
  axis's position and velocity; the caller sends a first setpoint consistent
  with where the axis is. What the library promises is that `streamBegin`
  itself changes nothing.
- **Timeout**: if no setpoint arrives for `STREAM_TIMEOUT_MS` (default 100),
  mode → STOPPING at the axis's decel, starting from `(P, v_cmd, acc = 0)`,
  the `streamTimedOut` flag is set, and `PulDir_loop` logs it once. Setpoints
  arriving afterwards are dropped; the caller must `streamBegin` again.
- `streamEnd` goes to STOPPING the same way, then `restoreDefaults` as
  today.
- The position error `p_hold − P` is readable, for telemetry.

### 5.7 Groups

```
PulDirGroup g;
PulDirGroup_init(&g, (PulDir*[]){&a, &b, &c}, 3);
PulDirGroup_moveTo(&g, (float[]){ta, tb, tc});
PulDirGroup_stop(&g);  PulDirGroup_halt(&g);  PulDirGroup_isMoving(&g);  PulDirGroup_faulted(&g);
```

One **path parameter** `s ∈ [0, 1]` is driven by a `MotionProfile` of its
own, in the plan tick. Its limits are the tightest axis's, scaled by that
axis's travel: `vmax_s = min_i vmax_i / |Δ_i|`, likewise `amax_s`, `dmax_s`,
`jmax_s` (axes with `Δ_i = 0` do not constrain, and an axis with `jmax = 0`
does not constrain `jmax_s`; if none does, `jmax_s = 0`). The group's
profile is initialised at the plan interval like an axis's, so its filter
length follows from `jmax_s`. Each interval every member's profile position
is `start_i + Δ_i · s`, so all members move along a straight line in joint
space. Members are in GROUP mode and refuse individual commands except
`halt` until the group completes.

Synchronisation needs two things the shared profile alone does not give:

- **atomic publication**: the plan tick writes every member's increment
  into the shadow set and publishes once; the pulse tick latches the whole
  set at the start of a tick (5.3). Members therefore never run one tick
  apart;
- **a start tick**: a group move starts when every member's enable delay
  has elapsed and every member's direction is set, which the plan tick
  arranges before advancing `s` for the first time.

"Finishes on the same tick" means: the rising edges of every member's final
pulse fall in the same pulse tick, and the group reports not moving once the
last of those pulses has been lowered. A member with zero travel takes part
in the move (it is in GROUP mode, refuses commands, reports moving) and
emits nothing. A member with one pulse of travel emits it on the landing
interval. Nothing moves until every member is ready, so the group's first
pulse may come a few milliseconds after the call.

No speed argument on the group move: each axis's own limits are its limits.

### 5.8 Homing, fault, enable and release: in the plan tick

Nothing that can stop a drive depends on the main loop. The plan tick runs,
per axis, besides the mode:

- **Homing**, as a sub-mode: `MOVE_TOWARDS_LIMIT` (run at homing speed
  toward the switch) → `DELAY1` → `MOVE_AWAY_FROM_LIMIT` (run opposite at
  0.2× until the switch releases) → `DELAY2` → `ADJUST_POSITION` (move by
  the adjust offset) → `DELAY3` (set position to `zero_pos`,
  `isHomed = true`) → IDLE. Each phase has a timeout, a per-axis setting.
  **A timeout or `homeStop` halts the axis and leaves `isHomed` false.**
  The switch is read through a `const uint8_t*` the caller owns and
  debounces; the caller must update it at least every 10 ms, and if the
  caller stalls the phase timeout is what stops the motor.
- **Fault**: `port_pin_read` on the fault pin every plan tick, latch FAULT
  after `FAULT_DEBOUNCE_MS` (2) continuously active, halt on latch, refuse
  motion while latched. `clearFault` raises the clear pin and the plan tick
  lowers it after `FAULT_CLEAR_MS` (10) and re-samples. Keep the fault
  counter and last-fault tick for the console.
- **Enable delay**: a motion command on a disabled drive enables it and
  holds the profile for `enable_ms` before the first pulse.
- **Release**: with auto-release on, the drive is disabled `release_ms`
  after the final pulse of a move.

`PulDir_loop(axis)` from the main loop does **logging only**: the ticks set
per-axis event bits (landed, stream timeout, homing result, refused
command, fault, pulse-tick overrun) and `PulDir_loop` turns each into one log
line. The log function pointer is never called from an interrupt. Nothing
else depends on how often `PulDir_loop` runs.

### 5.9 Concurrency and timing rules

- Public API is called from **main context only**. The ticks never call out
  except through the port's pin read and write.
- A command (target, limits, mode change, setpoint) is written into the axis
  under `port_lock()` / `port_unlock()`, which mask **only the two library
  interrupts**, never all interrupts. A locked section is a handful of
  stores: under one microsecond, and the test suite's fake port asserts
  that nothing but plain stores happens inside one.
- Values shared between the plan tick and the pulse tick are the shadow
  increment set with its publish flag, and the 64-bit pulse positions read
  under lock. Nothing else is shared.
- Getters return one 32-bit value each, the position getter after a locked
  64-bit read. No multi-field snapshot API.
- Interrupt priorities: pulse tick above plan tick, both above everything
  else in the application except the system tick. The integration task
  verifies that for the application; the library documents the requirement
  and its consequence: any interrupt the application runs above the pulse
  tick adds its duration to the pulse jitter.
- **Overrun**: if the pulse tick's timer has already fired again when the
  handler is entered, the port passes `overrun = true`; the core counts it
  and raises the event bit. One overrun delays a tick, two in a row lose
  one. An overrun in normal operation is a bug to be fixed, not tolerated.
- The tick counter is `uint32_t`; every comparison is by unsigned
  subtraction, so it works across the wrap (about six hours at 200 kHz,
  test 22).
- No allocation, no blocking, no `printf`, no HAL calls inside the core.
  The port's pin write from the pulse tick must be a single register write.
- The CPU figures and the jitter claim in 5.3 are estimates. They become
  measurements at step 4 of section 9, on real pins, under the application's
  other interrupts, with a scope.

### 5.10 The port contract

`puldir_port.h`, implemented once per platform:

```c
typedef struct { ... } puldir_pin_t;    /* opaque to the core; the port defines it */
#ifndef PULDIR_TICK_HZ
#define PULDIR_TICK_HZ 50000u           /* the build overrides both; 200000u for 100k pulses/s */
#endif
#ifndef PULDIR_PLAN_HZ
#define PULDIR_PLAN_HZ 1000u
#endif

void port_pin_init_output(puldir_pin_t, bool level);
void port_pin_init_input (puldir_pin_t, bool pullup);
void port_pin_write(puldir_pin_t, bool level);   /* from the pulse tick: one register write */
bool port_pin_read (puldir_pin_t);               /* from the plan tick */
void port_start(void (*pulse_tick)(bool overrun), void (*plan_tick)(void));
void port_lock(void);  void port_unlock(void);   /* mask only the library's interrupts */
```

The STM32 port, `port/stm32/puldir_port_stm32.{h,c}`, compiles for any
family: it includes `main.h`, which every CubeMX project generates and which
pulls in that family's HAL, and it names no timer. The application owns the
two timers and their priorities in CubeMX and hands them over once, before
`PulDir_start`:

```c
void PulDir_stm32_bind(TIM_HandleTypeDef* pulse_htim, IRQn_Type pulse_irqn,
                       TIM_HandleTypeDef* plan_htim,  IRQn_Type plan_irqn);
void PulDir_stm32_pulseIrq(void);   /* call from the raw TIMx_IRQHandler */
void PulDir_stm32_planIrq(void);    /* likewise */
```

- `port_start` starts both timers' update interrupts.
- The application calls the two IRQ entries from its own `TIMx_IRQHandler`
  in `stm32xxxx_it.c`, not through `HAL_TIM_IRQHandler`, which checks every
  flag on every entry and at 200 kHz would cost more than the tick itself.
- Overrun: the pulse entry clears the update flag on entry and reads it again
  on exit. If it is set again, the next tick is already late; the port passes
  `overrun = true` on that next call.
- `port_lock` disables exactly the two IRQ numbers in the NVIC, with a
  barrier; `port_unlock` re-enables them. A tick that came due inside the lock
  fires on unlock, late by the lock's duration. No PRIMASK, no BASEPRI.
- `puldir_pin_t` is `{ GPIO_TypeDef* port; uint16_t pin; }` and
  `port_pin_write` is one write to `BSRR` or `BRR`. Pin modes are CubeMX's
  business: `port_pin_init_output` only parks the pin at its idle level, as
  the old port did, and `port_pin_init_input` does nothing.
- The port cannot check that the timers run at `PULDIR_TICK_HZ` and
  `PULDIR_PLAN_HZ`; the bench (section 9, step 4) does, with a scope.

The Arduino port honours whatever `PULDIR_TICK_HZ` the build sets; an 8-bit
AVR cannot do 50 kHz with four axes, 20 kHz with two is realistic there.
Headers carry `extern "C"` guards; the core has no C++.

## 6. API

### 6.1 The functions

Everything is in user units. Every `bool` is "accepted"; a refusal logs the
reason through `PulDir_loop` when logging is enabled for that axis, and
changes nothing. `PulDir`, `PulDirGroup` and `MotionProfile` are plain structs
declared in their headers, allocated by the caller; fields are private by
convention.

```c
/* library-wide */
void     PulDir_attachLogger(void (*fn)(const char*));
void     PulDir_start(void);                    /* once, after every PulDir_register */
uint32_t PulDir_ticks(void);
uint32_t PulDir_overruns(void);
void     PulDir_consoleMenu(char* buf, size_t n);

/* axis setup, before PulDir_start() */
void  PulDir_init(PulDir*, const char* name);
void  PulDir_attachPins(PulDir*, puldir_pin_t pulse, puldir_pin_t dir,
                      bool pulse_active_low, bool dir_inverted);
void  PulDir_attachEnable(PulDir*, puldir_pin_t, bool inverted);
void  PulDir_attachFault(PulDir*, puldir_pin_t, bool inverted);
void  PulDir_attachFaultClear(PulDir*, puldir_pin_t, bool inverted);
void  PulDir_setConversion(PulDir*, float pulses_per_unit);
void  PulDir_setDefaults(PulDir*, float speed, float accel, float decel, float jerk);
void  PulDir_restoreDefaults(PulDir*);
void  PulDir_setDelays(PulDir*, uint32_t enable_ms, uint32_t release_ms);
void  PulDir_setAutoRelease(PulDir*, bool);
void  PulDir_setHoming(PulDir*, int direction, float speed, float adjust,
                     const uint8_t* switch_ptr, float zero_pos, uint32_t phase_timeout_ms);
void  PulDir_register(PulDir*);                     /* joins the ticks; static table of PULDIR_MAX_AXES (default 8) */

/* limits, live; setJerk only when idle */
bool  PulDir_setSpeed(PulDir*, float);              float PulDir_speed(PulDir*);
bool  PulDir_setAcceleration(PulDir*, float);       float PulDir_acceleration(PulDir*);
bool  PulDir_setDeceleration(PulDir*, float);       float PulDir_deceleration(PulDir*);
bool  PulDir_setJerk(PulDir*, float);               float PulDir_jerk(PulDir*);

/* motion */
bool  PulDir_moveTo(PulDir*, float position);
bool  PulDir_moveBy(PulDir*, float distance);
bool  PulDir_jog(PulDir*, float speed);
void  PulDir_stop(PulDir*);                         /* ramped */
void  PulDir_halt(PulDir*);                         /* immediate */
bool  PulDir_enable(PulDir*, bool);

/* stream */
bool  PulDir_streamBegin(PulDir*);
void  PulDir_stream(PulDir*, float position, float velocity);
void  PulDir_streamEnd(PulDir*);
bool  PulDir_inStream(PulDir*);
bool  PulDir_streamTimedOut(PulDir*);
float PulDir_streamError(PulDir*);

/* homing */
bool  PulDir_homeStart(PulDir*);
void  PulDir_homeStop(PulDir*);
PulDirHomeState PulDir_homeState(PulDir*);
bool  PulDir_isHomed(PulDir*);

/* state */
float PulDir_position(PulDir*);
float PulDir_velocity(PulDir*);                     /* exactly 0.0f when stopped */
float PulDir_target(PulDir*);
bool  PulDir_setPosition(PulDir*, float);           /* datum; IDLE only */
bool  PulDir_isMoving(PulDir*);
PulDirMode   PulDir_mode(PulDir*);
PulDirStatus PulDir_status(PulDir*);
bool  PulDir_clearFault(PulDir*);
void  PulDir_setLogging(PulDir*, bool);
void  PulDir_loop(PulDir*);                         /* logging only */
void  PulDir_console(PulDir*, char cmd, const char* arg);

/* group */
void  PulDirGroup_init(PulDirGroup*, PulDir** axes, uint8_t count);
bool  PulDirGroup_moveTo(PulDirGroup*, const float* targets);
void  PulDirGroup_stop(PulDirGroup*);
void  PulDirGroup_halt(PulDirGroup*);
bool  PulDirGroup_isMoving(PulDirGroup*);
bool  PulDirGroup_faulted(PulDirGroup*);
```

`PulDir_setSpeed`, `setAcceleration`, `setDeceleration` and `setJerk` write
through to the axis's `MotionProfile` limits in pulses; they are the only
place the conversion is applied to a limit. Setting one never changes
another (today's `setAcceleration` resets deceleration). `PulDir_setJerk` is
refused unless the axis is idle, for the reason in 5.2; the other three are
live.

The public position getter returns a float32, which carries about seven
significant digits: at one million units the resolution is 0.06 units. The
pulse position underneath is exact; a caller that needs more than seven
digits reads in a smaller unit.

### 6.2 Names, old to new

The concepts survive; the names do not. The axis type is `PulDir`, as
`FlexyStepper` was the axis before: `FlexyStepper_x` becomes `PulDir_x`.
This table is the migration plan for the firmware that uses the old library.

| cFlexyStepper | puldir | note |
|---|---|---|
| `FlexyStepper` | `PulDir` | |
| `FlexyStepper_Init` | `PulDir_init` | |
| `connectToPins` | `PulDir_attachPins` | gains both polarities |
| `connectEnablePin` / `connectFaultPin` / `connectFaultClearPin` | `PulDir_attachEnable` / `PulDir_attachFault` / `PulDir_attachFaultClear` | |
| `setConversion` | `PulDir_setConversion` | |
| `setDefaults` / `restoreDefaults` | `PulDir_setDefaults` / `PulDir_restoreDefaults` | gains jerk |
| `setSpeed` / `getTargetSpeed` | `PulDir_setSpeed` / `PulDir_speed` | |
| `setAcceleration` / `getAcceleration` | `PulDir_setAcceleration` / `PulDir_acceleration` | no longer resets decel |
| `setDeceleration` / `getDeceleration` | `PulDir_setDeceleration` / `PulDir_deceleration` | |
| — | `PulDir_setJerk` / `PulDir_jerk` | new |
| `setTargetPosition(pos, release)` | `PulDir_moveTo(pos)` | release becomes `PulDir_setAutoRelease` |
| `setTargetPositionRelative(d, release)` | `PulDir_moveBy(d)` | |
| `jog` | `PulDir_jog` | no longer zeroes the position |
| `setTargetPositionToStop` | `PulDir_stop` | |
| `Estop(release)` | `PulDir_halt` | release is the auto-release setting |
| `en_motor` | `PulDir_enable` | halts first if moving |
| `beginPassthrough` / `streamSetpoint` / `endPassthrough` / `inPassthrough` | `PulDir_streamBegin` / `PulDir_stream` / `PulDir_streamEnd` / `PulDir_inStream` | |
| — | `PulDir_streamTimedOut` / `PulDir_streamError` | new |
| `set_homing` | `PulDir_setHoming` | gains the phase timeout |
| `start_cFlexyStepper_homing_sm` / `stop_…` / `get_…_state` / `.homing.did_home` | `PulDir_homeStart` / `PulDir_homeStop` / `PulDir_homeState` / `PulDir_isHomed` | `cFlexyStepper_homing_sm_loop` is gone: the plan tick runs it |
| `getCurrentPosition` / `getCurrentVelocity` | `PulDir_position` / `PulDir_velocity` | |
| `setCurrentPosition` | `PulDir_setPosition` | IDLE only, clears `isHomed` |
| `isMoving` / `motionComplete` | `PulDir_isMoving` | one predicate; `motionComplete` was its complement |
| `getStatus` | `PulDir_status` | |
| — | `PulDir_mode` | new |
| `clearFault` | `PulDir_clearFault` | non-blocking |
| `attach_logger` / `enable_logging` | `PulDir_attachLogger` / `PulDir_setLogging` | |
| `FlexyStepper_loop` | `PulDir_loop` | logging only now |
| `attach_timer_for_micros` | — | gone; the tick counter is the clock |
| `decode` / `decode_menu` | `PulDir_console` / `PulDir_consoleMenu` | |
| `setTimedTargetPosition` / `solveTimedMove` / `setStepsPerMillimeter` / `…InRevolutions` / `processMovement` | — | gone |
| — | `PulDirGroup_*`, `MotionProfile_*`, `PulDir_setDelays`, `PulDir_start`, `PulDir_ticks`, `PulDir_overruns` | new |

## 7. Rules of construction

- C99, `-Wall -Wextra` clean, no warnings suppressed.
- One source of truth for every constant; defaults as `#ifndef` macros so a
  project can override them in its build, as `FLEXY_FAULT_DEBOUNCE_US` is.
- No function that nothing calls. No parameter that nothing reads.
- Comments say why, in one or two lines. The owner strips long ones.
- Refuse, do not clamp.
- The generator and the core compile on a PC with a fake port. That is not
  optional: the only way to prove the landing rule, the jerk profile and the
  group sync is to count fake pulses.
- Integer maths in the pulse tick, float in the plan tick and the generator,
  no `double` on the target.
- Position and tick arithmetic is **unsigned** with explicit casts, so that
  wrap is defined behaviour: the 64-bit pulse position wraps after 2³¹
  pulses in either direction, and the tick counter after 2³² ticks.
  Document both; guard neither.

## 8. Acceptance tests (host, under `test/`, plain C asserts, a `run.sh`)

The generator is tested bare. The axis is tested through a fake port that
records `(tick, axis, pin, level)` for every write, serves the fault and
switch inputs from the test, and lets the test drive the two tick functions
itself. Each test states its expected numbers. The host compiler is the
`gcc` that the firmware's `test/run.sh` (in `164-race_firmware`) uses from a
bash shell; copy that script's shape. From step 5 of section 9 on, `run.sh` builds the
axis suite twice, at the default tick and at 200 kHz.

Generator:

1. **dt independence.** Two profiles initialised at `dt` = 1 ms and 5 ms and
   given the same move land at the same target, with peak velocity and total
   time equal within one coarse step, with and without jerk.
2. **Trapezoid.** Speed 15, accel 30, decel 30, move 9 units: time and peak
   match the analytic trapezoid; `vel` never exceeds 15; `done` with
   `vel == 0.0f` exactly and `pos == 9` exactly.
3. **Asymmetric ramps.** Accel 20, decel 90: the decel phase is 4.5× shorter
   than the accel phase.
4. **S-curve.** With jerk set, `acc` never changes by more than `jmax·dt`
   per call and stays within `[−dmax, +amax]`; `vel` never overshoots
   `vmax`; the profile lands exactly, `done` arriving exactly `n` calls after
   the trapezoid landed. With `jmax = 0` the output equals the trapezoid bit
   for bit.
5. **Landing and retarget.** 1,000 random moves at random limits: exact
   landing, never passes a target that was reachable when set. Retarget
   behind the axis while cruising: decelerates, reverses once, lands
   exactly. Speed raised mid-cruise ramps up at `amax`.
6. **Stop and halt.** `stop` from cruise, with and without jerk, lands at
   `pos + stopDistance` (within one `dt` of travel), including when called
   during the acceleration ramp with `acc` non-zero. `halt` zeroes `vel` and
   `acc` on that call.
7. **Endless.** Set speed, change it, reverse it, stop. `pos` is continuous
   throughout.
8. **Lowered limits mid-move.** `vmax` lowered while cruising: velocity
   comes down at `dmax` (and `jmax`), never steps. `amax` lowered while
   accelerating: `acc` comes inside the new bound at `jmax`.
9. **Infeasible retarget.** Target moved to inside the stopping distance:
   the axis passes it, stops under the limits, returns and lands exactly.
   Across all generator tests, every call satisfies
   `|Δvel| ≤ max(amax, dmax)·dt` and, with jerk set, `|Δacc| ≤ jmax·dt`, the
   landing call included.

Axis:

10. **Parity with the generator.** A bare generator and an axis given the
    same move in the same units: the pulse timestamps match the generator's
    integer crossings within one pulse tick, over the whole move, with and
    without jerk.
11. **Exact pulse count.** 1,000 random moves with fractional targets: the
    pulses emitted equal the target's integer pulse count; the pulse
    position never trails the profile by more than one increment, including
    across direction changes and their skipped ticks.
12. **Stream tracking.** Feed a sine (`p`, `dp/dt`) for 10 s, once at 200 Hz
    and once at 1 kHz with the setpoints out of phase with the plan tick:
    steady-state error below 2 pulses, no stall at start, no pulse gap
    longer than the sine's own motion allows.
13. **Stream loss and re-entry.** Stop feeding: the axis decelerates at its
    decel from the last commanded velocity, stops, flag set, mode leaves
    STREAM; setpoints sent afterwards are dropped with the refused flag;
    `streamBegin` restores service. Between `streamBegin` and the first
    setpoint the velocity is unchanged.
14. **Direction setup, hold and polarity.** No direction change shares a
    tick with a pulse edge; a pulse follows a direction change no earlier
    than the next tick. With `pulse_active_low` the pulse pin idles high
    and pulses low; with `dir_inverted` the direction pin reads the
    opposite for the same move.
15. **Rate cap.** `setSpeed` at or above `TICK_HZ/2/conversion` is refused;
    nothing ever stores `|inc| > 2³¹ − 1`.
16. **Latch.** Four axes whose increments all change on the same plan tick
    switch on the same pulse tick, with the pulse tick interleaved at every
    possible point of the plan tick's writes.
17. **Halt.** No pulse after the tick it is called in; position reads back
    where the pulses stopped; `isMoving` false once the last pulse is
    lowered.
18. **Group.** Three axes with travels 1,000 / 250 / −60,000 pulses and
    different limits, plus a zero-travel member and a one-pulse member: the
    final pulses of every moving member fall in the same tick; at every plan
    tick each member's position equals `start + Δ·s` within one pulse; the
    zero-travel member reports moving and emits nothing.
19. **Transitions.** Every row of the table in 5.5: a member fault halts
    the group; a member's `halt` halts the group; a member's `moveTo` is
    refused; `moveTo` during STOPPING is accepted; `homeStart` in STREAM is
    refused; `enable(false)` during motion halts first; `isMoving` is true
    from an accepted `moveTo` before the first pulse.
20. **Homing.** Fake switch: the success path sets `isHomed` and the datum;
    a phase timeout halts and leaves `isHomed` false; a switch byte that
    stops updating ends in a timeout halt; `isHomed` clears on a fault
    latch, on `homeStart` and on `setPosition`.
21. **Enable and release delays.** No pulse before `enable_ms` after the
    enable pin goes active; the enable pin goes inactive `release_ms` after
    the last pulse, and never while pulses remain.
22. **Tick rollover.** With the tick counter started 1,000 ticks below its
    wrap: the stream timeout, a homing phase timeout and the fault clear
    pulse all measure correctly across the wrap.
23. **Full rate.** The whole axis suite built with `TICK_HZ = 200000`: a
    move at 99,990 pulses/s shows 5 µs pulses at a 10 µs period, and the
    cap test refuses 100,000.
24. **Jerk refusals.** `MotionProfile_setLimits` with a changed `jmax` is
    refused unless `done`, and a `jmax` needing more than
    `MOTION_PROFILE_MAX_TAPS` taps is refused; `PulDir_setJerk` on a moving
    axis is refused, changes nothing, and `PulDir_loop` logs the smallest
    jerk that fits.

Bench tests on real drives come after, as separate steps; the host suite is
what makes them short.

## 9. Layout and order of work

This repository, `cPulDir`, becomes a submodule under `Drivers/` of
`164-race_firmware`, next to `cFlexyStepper` (which stays until an
integration task deletes it).

```
motion_profile.h       the generator: MotionLimits, MotionProfile, the functions of 5.2
motion_profile.c       pure maths, nothing else
puldir.h               config macros, PulDir, PulDirGroup, the enums, the whole API of 6.1
puldir.c               commands, getters, defaults (main context), the axis table, PulDir_start
puldir_plan.c          the plan tick: re-basing, profile, stream, group, homing, fault, delays
puldir_pulse.c         the pulse tick
puldir_group.c         PulDirGroup_*
puldir_loop.c          PulDir_loop: the deferred log lines
puldir_console.c       the @ console, plus Z<jerk> and z
puldir_port.h          the port contract
port/stm32/            puldir_port_stm32.{h,c}, any family
port/arduino/          the Arduino port, a later spec
test/                  the tests of section 8, the fake port, run.sh
docs/                  these guidelines, the specs, the plans
README.md              the generator alone, adding an axis, the port contract, the numbers
```

Order:

1. `motion_profile` with the trapezoid only (`jmax = 0`): tests 1, 2, 3, 5,
   6, 7, 8, 9.
2. The jerk filter in the generator: test 4 and the generator half of 24;
   re-run 1, 5, 6, 8, 9 with jerk.
3. Fake port and the pulse tick with hand-written increments: tests 14, 16,
   and the increment half of 15.
4. **The STM32 port, on the bench, before any planning exists.** This is
   the one step that leaves this repository. On a branch of
   `164-race_firmware`: add `cPulDir` as a submodule under `Drivers/`,
   configure two free basic timers in CubeMX (TIM6 and TIM7 are unused in the
   `.ioc`) at `PULDIR_TICK_HZ` and `PULDIR_PLAN_HZ`, the pulse tick at the
   highest priority and the plan tick next, call the two IRQ entries from
   the handlers, and add one bench console verb that loads hand-written
   increments into the board's three PUL/DIR channels (MOTOR1 to MOTOR3 in
   `main.h`) plus a fourth axis on any spare output, so that the four-axis
   CPU figure of 2.3 is measured. cFlexyStepper stays untouched. Pulse all
   four at the default tick and at 200 kHz, with the application's USB, CAN
   and UART interrupts running. Measure with a scope: pulse width, period
   jitter, interrupt duration, overrun count. This turns the estimates of
   5.3 and 5.9 into numbers and catches a hopeless tick rate before the rest
   is built on it.
5. Plan tick on top of the generator: tests 10, 11, 15, 17, 23 and the axis
   half of 24.
6. Stream: tests 12, 13.
7. Group: tests 18, 19.
8. Homing, fault, delays in the plan tick: tests 20, 21, 22.
9. `@` console, verb for verb from `cFlexyStepper_decode.c`, plus a jerk
   verb.
10. One drive on the bench with motion, then two, then the jitter and
    tracking claims validated against the drive's behaviour.

Commit after each numbered step with the tests green. Do not start step 10
before step 8 is green on the host.

## 10. Assumptions to confirm with the owner

- The drives' pulse timing requirements are unknown; one tick (20 µs at the
  default `TICK_HZ`, 5 µs at 200 kHz) is assumed safe because 2 µs works
  today. Step 4 of section 9 is where that is confirmed.
- The Arduino target is a 32-bit board (Teensy, ESP32, SAMD, RP2040); an AVR
  port would need a lower `TICK_HZ` and fewer axes.
