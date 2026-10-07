# puldir: decisions from the design review

Date: 2026-10-07
Status: designed, not implemented
Design: `docs/puldir-guidelines.md` in this repository is the design. This
document records what the review decided, what it rejected and why, so that
no closed question is reopened without the reasons in front of whoever
reopens it. If the two disagree, this document wins; fix the guidelines.

## Scope of this cycle

The whole library on STM32: the profile generator, the axis core (pulse tick
and plan tick), stream mode, groups, homing, fault, enable and release delays,
the `@` console, the STM32 port and the host test suite, in the order of
section 9 of the guidelines. Two things are deliberately later specs:

- the Arduino port. The port contract keeps it possible; nothing is built
  until there is a board to test on;
- the migration of `164-race_firmware` from cFlexyStepper to puldir, in which
  the gimbal's two virtual master axes become bare `MotionProfile`s and its
  two lean motors become `PulDir` axes. Section 6.2 of the guidelines is that
  migration's name map.

## Decisions

### 1. Jerk is a moving-average filter on a trapezoid

The generator is an acceleration-limited online planner (the trapezoid) whose
velocity passes through a moving average of `n = ceil((amax + dmax) / (jmax · dt))`
taps before it is integrated. The caller sees only the filtered output.
Section 5.2 of the guidelines has the full rule. The running sum is
recomputed from the ring every `n` calls, when `head` wraps, so float
rounding cannot walk the average off during an endless jog; the cost
amortises to one addition per call.

Why it holds the limits:

- filtered acceleration is `(v_t(t) − v_t(t − n·dt)) / (n·dt)`, and the
  trapezoid's velocity changes by at most `amax` upward and `dmax` downward
  over any window, so it stays within `[−dmax, +amax]`;
- filtered jerk is `(a_t(t) − a_t(t − n·dt)) / (n·dt)`, and the trapezoid's
  acceleration takes values in `{+amax, 0, −dmax}`, so the largest step is
  `amax + dmax` and the jerk is at most `jmax`. The window is `amax + dmax`,
  not the larger of the two, so that a triangle move's flip from `+amax` to
  `−dmax`, or a retarget's, is covered;
- the average of values no larger than `vmax` is no larger than `vmax`;
- `jmax = 0` gives one tap, which is the trapezoid itself. The gimbal runs
  with jerk off today, so nothing changes for it until jerk is switched on.

What it costs, accepted:

- a move with jerk completes `n·dt` after its trapezoid would. With
  `amax = dmax = 30` and `jmax = 300` that is 200 ms;
- the profile is jerk-bounded, not time-optimal;
- `4 · MOTION_PROFILE_MAX_TAPS` bytes per profile, 1 KB at the default of
  256; under 8 KB on the first machine;
- `dt` must be known when the limits are set, which is decision 2;
- the window must stay valid while a profile is in flight, which is
  decision 3.

Rejected: the online S-curve planner of the original 5.2, which decided
accelerate/decelerate each tick from a stopping distance that includes the
current acceleration. With jerk, the final approach has velocity falling as
`t²`, so the landing window is only reached if the jerk-up instant is solved
exactly from a non-zero `(rem, v, a)`; solved late the axis stops short and
needs a jerk-limited re-ramp for a sub-pulse, solved early it overshoots and
reverses. The decision also chatters at the switching surface at discrete
`dt` unless latched. That is the problem Ruckig exists to solve, and it would
have been the most expensive piece of the library by far.

Rejected: a time-optimal seven-segment recompute every tick (Ruckig-style).
Correct and delay-free, but polynomial root finding in float32 on an M4 for
a four-axis gimbal is out of proportion.

### 2. `dt` is fixed per profile at `init`

`MotionProfile_init(p, pos, dt)`; `MotionProfile_advance(p)` takes no
argument. The filter length depends on `dt`, so `setLimits` must know it.
Every caller uses a fixed interval anyway: the axis and the group use the
plan interval, a bare caller its own tick. Test 1, dt independence, becomes
two profiles initialised at different `dt`.

### 3. The jerk window is frozen while a profile is in flight

While a profile is not `done`, `n` does not change, and `setLimits` is
accepted only if the current window still bounds the new jerk:
`(amax + dmax) / (jmax · dt) ≤ n`, with `jmax = 0` always accepted. When the
profile is done, `n` is recomputed. So speed is always live; lowering accel
or decel and raising jerk are live; raising accel or decel beyond the
window's rounding slack, lowering jerk, and switching jerk on are refused
until the profile is idle. With jerk off the window is one tap and nothing
is refused except switching jerk on, so the gimbal's live speed, accel and
decel commands are unaffected. `PulDir_restoreDefaults` applies all four
limits or none under the same rule and returns whether it did, and
`streamEnd` restores the defaults on the idle generator before seeding it
from the stream velocity. An `n` above `MOTION_PROFILE_MAX_TAPS` is refused
too, and the log line states the smallest jerk that fits.

The first version of this decision kept accel and decel live; the review
found that raising either with the window frozen breaks the jerk bound,
because the window is sized from `amax + dmax`. Resizing a ring that holds a
ramp is a jerk violation whichever way it is done, hence the freeze.

### 4. The trapezoid's decel rule is a velocity ceiling

Each `advance` computes the largest velocity that, after this step, leaves
exactly its own stopping distance:

```
v_c = sign(rem) · min(vmax, sqrt((dmax·dt)² + 2·dmax·|rem|) − dmax·dt)
```

and moves `v_t` toward it by at most `amax·dt` when its magnitude grows and
`dmax·dt` when it shrinks, never crossing zero within one call. Because the
ceiling is a monotone function of the remaining distance there is no
accelerate/decelerate flag, so nothing can chatter; following the ceiling,
the remaining distance equals `v_t² / (2·dmax)` after every step and the
landing rule of 5.2 finishes the move exactly once `|v_t| ≤ dmax·dt`. A
target inside the stopping distance makes the ceiling fall faster than
`dmax` allows, so `v_t` comes down at `dmax`, passes the target, `rem`
changes sign and the same rule brings it back. The discrete form of the
ceiling, not `sqrt(2·dmax·|rem|)`, is what makes the descent land without a
residual; the implementer must not simplify it.

### 5. Names: `PulDir` is the axis

As `FlexyStepper` was the axis in the old library: `PulDir ax;
PulDir_moveTo(&ax, 10.0f)`. Library-wide calls keep the same prefix
(`PulDir_start`, `PulDir_attachLogger`), groups are `PulDirGroup_*`, the
generator is `MotionProfile_*`. The guidelines' `Axis` was rejected because
it is a generic identifier the application already uses (`gimbal_axis_t`,
`axis_tilt`, `axis_move_to`). The migration is `FlexyStepper_x` to
`PulDir_x`.

### 6. The STM32 port: application-owned timers, raw IRQ entries, NVIC lock

Section 5.10 of the guidelines has the detail. The points that were open:

- the port includes `main.h`, which every CubeMX project generates and which
  pulls in the right family's HAL, so the port names no family;
- the application configures the two timers and their priorities in CubeMX
  and binds them once with `PulDir_stm32_bind`;
- the application calls `PulDir_stm32_pulseIrq` and `PulDir_stm32_planIrq`
  from its raw `TIMx_IRQHandler`, not through `HAL_TIM_IRQHandler`, whose
  flag sweep would cost more than the tick at 200 kHz;
- overrun is detected by re-reading the update flag at exit and reported on
  the next call;
- `port_lock` is `NVIC_DisableIRQ` on exactly the two timer IRQs with a
  barrier, never PRIMASK or BASEPRI;
- `puldir_pin_t` is `{ GPIO_TypeDef* port; uint16_t pin; }` and the pulse
  write is one `BSRR`/`BRR` store.

### 7. The bench step reaches into the firmware

Section 9, step 4 is mandatory before any planning code exists and needs a
real board with the application's other interrupts running. It is done on a
branch of `164-race_firmware`: add `cPulDir` as a submodule under
`Drivers/`, configure TIM6 and TIM7 (unused in the `.ioc`) at the two tick
rates, call the IRQ entries, and add one bench console verb that loads
hand-written increments into the board's three PUL/DIR channels plus a fourth
axis on any spare output, so that the four-axis CPU figure is measured.
cFlexyStepper stays untouched; the gimbal migration remains a later spec.

### 8. Tests 4 and 9 lose their exception; test 24 is added

The filtered output has no landing exception: every call satisfies the
velocity, acceleration and jerk bounds, and tests 4 and 9 demand it
everywhere. Test 4 also checks that `jmax = 0` reproduces the trapezoid bit
for bit and that `done` arrives exactly `n` calls after the trapezoid lands.
Test 24 covers the frozen window of decision 3, accepted and refused cases
both, and test 7 checks that the running sum does not drift over ten
million calls.

### 9. Documents live here

- `docs/puldir-guidelines.md`: the design, revised for the decisions above.
  The copy in `164-race_firmware/docs/` is the pre-review version and stays
  until the integration task replaces it with migration notes.
- this file: the record of the review.
- `docs/superpowers/plans/`: the implementation plan.
- `README.md`: the generator alone, adding an axis, the port contract, the
  numbers. Written as part of the plan.

## Revised after review

A review the same day found three things, fixed here and in the guidelines:
raising accel or decel live broke the jerk bound (decision 3); the "limits
during motion" row of section 2.2 still mentioned the landing exception that
decision 8 removed; the running sum drifted during an endless jog
(decision 1).

## Still assumed, to confirm on the bench

- The drives' pulse timing requirements are unknown; one tick (20 µs at the
  default `PULDIR_TICK_HZ`, 5 µs at 200 kHz) is assumed safe because 2 µs
  works today. Step 4 of section 9 confirms it.
- The Arduino target, when it comes, is a 32-bit board.
