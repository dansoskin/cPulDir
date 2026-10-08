# puldir

puldir drives several PUL/DIR motor drives (steppers, closed-loop steppers, servo drives) from one microcontroller. Every pulse is written from a timer interrupt, so the pulse train does not depend on how busy the main loop is. Each axis has an acceleration, deceleration and jerk limited motion profile, live retargeting, jogging, homing, fault input handling and optional enable and release timing. Axes can be moved together along a straight line (groups), or follow a stream of position and velocity setpoints from a host. The core is plain C99 with no HAL. The STM32 port is in `port/stm32/`.

## The generator on its own

`motion_profile.h` and `motion_profile.c` need nothing else. It is an acceleration-limited trapezoid whose velocity passes through a moving-average jerk filter. You call `advance` at a fixed interval and read `pos`, `vel` and `acc`. Units are yours. A jerk of 0 means no jerk limit.

```c
#include <stdio.h>
#include "motion_profile.h"

int main(void)
{
    MotionProfile p;
    MotionProfile_init(&p, 0.0f, 0.001f);                    /* start at 0, advance every 1 ms */
    MotionProfile_setLimits(&p, (MotionLimits){ 10.0f, 50.0f, 50.0f, 400.0f });
    MotionProfile_setTarget(&p, 5.0f);
    unsigned calls = 0;
    while (!p.done) {
        MotionProfile_advance(&p);
        calls++;
    }
    printf("%u ms, pos %.3f, vel %.1f\n", calls, (double)p.pos, (double)p.vel);
    return 0;
}
```

The limits are speed, acceleration, deceleration, jerk. `done` is set when the profile is at the target with `vel` exactly 0. Setters return false and change nothing if given a bad value. The jerk filter window holds at most `MOTION_PROFILE_MAX_TAPS` (256) velocities; a jerk too low for that is refused. Each profile is about 1 KB. Keep coordinates within about seven significant digits, because `pos` is a float. Build with `-lm`.

## Adding an axis

Files to add to your build: `motion_profile.c`, `puldir.c`, `puldir_pulse.c`, `puldir_plan.c`, `puldir_loop.c`, `puldir_group.c`, `puldir_console.c`, and a port (below). Put the repo root and exactly one port directory on the include path (`port/stm32` or `test`). That directory supplies `puldir_pin.h`, which defines `puldir_pin_t`. Link with `-lm`.

The order matters. Limits and defaults are held in pulses, so `PulDir_setConversion` must come before `PulDir_setDefaults`. A later `setConversion` keeps the limits fixed in pulses and rescales the user-unit values.

```c
#include "puldir.h"

static PulDir x;

void app_setup(void)
{
    PulDir_init(&x, "X");
    PulDir_attachPins(&x, PIN_X_PUL, PIN_X_DIR, false, false);  /* pulse active low?, dir inverted? */
    PulDir_attachEnable(&x, PIN_X_ENA, false);                  /* optional; also Fault and FaultClear */
    PulDir_setConversion(&x, 280.0f);                           /* pulses per user unit, first */
    PulDir_setDefaults(&x, 15.0f, 30.0f, 30.0f, 0.0f);          /* speed, accel, decel, jerk */
    PulDir_register(&x);
    PulDir_start();                                             /* after every axis is registered */
}

void app_loop(void)
{
    PulDir_loop(&x);        /* every pass of the main loop, for every axis */
}
```

`PIN_X_PUL` and the others are `puldir_pin_t` values. On STM32 that is `{ GPIOC, GPIO_PIN_12 }`. Configure the pins as outputs (and inputs for fault) in CubeMX. The pulse pin idles at its inactive level.

Then move it from the main context:

```c
PulDir_moveBy(&x, 1.0f);          /* also moveTo, jog, stop, halt */
```

Calls validate and refuse bad input, they never clamp. A refusal returns false and logs one line from `PulDir_loop` through the logger set with `PulDir_attachLogger`. `PulDir_loop` also logs landings and faults, so call it for every axis. Setup calls (`setConversion`, `setDefaults`, `setDelays`, `setHoming`) return false on refusal too. Check them. `PulDir_init` returns false for a NULL name or one longer than 15 characters, and then writes nothing, so it has nothing to log through.

Targets from `moveTo` and `moveBy` are rounded to whole pulses. `PulDir_stop` ramps down; `PulDir_halt` stops at once and leaves the drive enabled. A fault input latch halts and does not release the drive. `PulDir_console(&x, cmd, arg)` is a one-letter command console for a UART; `PulDir_consoleMenu` prints the verbs.

## The port

`puldir_port.h` is the contract: `port_pin_init_output`, `port_pin_init_input`, `port_pin_write` (called from the pulse tick, one register write), `port_pin_read`, `port_start(pulse_tick, plan_tick)`, and `port_lock` and `port_unlock`. The lock must mask only the library's two interrupts. `PULDIR_TICK_HZ` and `PULDIR_PLAN_HZ` come from the build. The port cannot check that its timers run at those rates. The scope does.

### STM32 binding

`port/stm32/puldir_port_stm32.c` includes `main.h` and compiles for any STM32 family. It names no timer. You own two basic timers and hand them over once, before `PulDir_start`:

```c
PulDir_stm32_bind(&htim6, TIM6_DAC_IRQn, &htim7, TIM7_DAC_IRQn);   /* pulse timer, plan timer */
```

Call the two raw IRQ entries from your own handlers. Do not go through `HAL_TIM_IRQHandler`. Its flag checks cost more than the tick at 200 kHz. Put the calls in the CubeMX user sections, or untick "Call HAL handler" for these two timers, because regeneration overwrites the rest.

```c
void TIM6_DAC_IRQHandler(void) { PulDir_stm32_pulseIrq(); }
void TIM7_DAC_IRQHandler(void) { PulDir_stm32_planIrq(); }
```

Timer setup, in CubeMX:

- Both timers count up, same clock, update interrupt enabled, no output.
- The pulse timer period is one tick: at a 144 MHz timer clock, 2880 clocks for 50 kHz, 720 for 200 kHz.
- The plan timer period must be exactly N pulse periods, where N = `PULDIR_TICK_HZ / PULDIR_PLAN_HZ` (50 or 200 here). 144,000 clocks gives 1 kHz. If you change one period, change the other.
- `port_start` zeroes both counters and starts the pulse timer, then the plan timer. The plan tick then fires just after each interval boundary and has the whole interval to publish. This only holds if the clocks match and the periods divide exactly.

Priorities (CubeMX, NVIC):

- Pulse timer: the highest priority in the system, 0. Nothing at that level may run long.
- Plan timer: next, 1.
- Everything else (UART, DMA, USB, CAN): 2 or lower urgency. Any interrupt left at 0 adds its duration to the pulse jitter. That includes SysTick, which CubeMX sets to 0 by default.

`port_lock` disables exactly those two IRQ numbers in the NVIC. A tick that comes due inside the lock fires on unlock, late by the lock's duration. Before `PulDir_stm32_bind` the lock does nothing.

Main context only: call every `PulDir_*` function from the main loop (or one thread), never from an interrupt, including the UART interrupt. Do not call them from the two timer handlers. Pass commands to the main loop through a flag or a queue.

The bench procedure for the first board is `docs/bench-step4.md`.

## Groups and streams

Group: members move along a straight line and finish on the same tick. One path profile runs for the group, with each member's limits applied to its own travel and the tightest one winning. While the move runs, members refuse their own limit setters.

```c
static PulDirGroup g;                       /* static: the library keeps a pointer to it */
static PulDir* members[] = { &x, &y };
PulDirGroup_init(&g, members, 2u);          /* false if refused */
const float to[] = { 10.0f, 4.0f };         /* one target per member, user units */
PulDirGroup_moveTo(&g, to);                 /* members must be IDLE; refused otherwise */
```

Also `PulDirGroup_stop`, `_halt`, `_isMoving`, `_faulted`. A fault on any member halts the group.

Stream: a host sends position and velocity setpoints, 200 Hz up to the plan rate. The axis tracks them with a first-order hold.

```c
PulDir_streamBegin(&x);                     /* from IDLE, or a move in progress */
PulDir_stream(&x, position, velocity);      /* each setpoint, user units and units/s */
PulDir_streamEnd(&x);                       /* ramps to rest */
```

If no setpoint arrives for `PULDIR_STREAM_TIMEOUT_MS`, the axis ramps to a stop at its deceleration and `PulDir_streamTimedOut` reports it. `PulDir_streamError` gives the tracking error in pulses. `PulDir_setStreamCatchup` sets the catch-up time per axis.

## Configuration macros

Define these on the compiler command line. Defaults are from the headers.

| macro | default | meaning |
|---|---|---|
| `PULDIR_TICK_HZ` | 50000u | pulse tick rate; 200000u for 100k pulses/s |
| `PULDIR_PLAN_HZ` | 1000u | plan tick rate; 1000 to 4000, must divide the tick rate, N = tick / plan in 2..4096 |
| `PULDIR_MAX_AXES` | 8u | axis table size |
| `PULDIR_MAX_GROUPS` | 4u | group table size |
| `PULDIR_CATCHUP_MS` | 50u | stream catch-up time |
| `PULDIR_STREAM_TIMEOUT_MS` | 100u | stream setpoint timeout |
| `PULDIR_FAULT_DEBOUNCE_MS` | 2u | fault input must stay active this long |
| `PULDIR_FAULT_CLEAR_MS` | 10u | fault clear pulse width |
| `PULDIR_HOMING_DELAY_MS` | 1000u | pause between homing phases |
| `PULDIR_TICK_START` | 0u | first tick count; tests only |
| `MOTION_PROFILE_MAX_TAPS` | 256 | jerk filter window; raise for softer jerk |

A bad combination fails to compile.

## The numbers

| tick rate | pulse width | max pulses/s per axis | four axes, CPU (estimate) |
|---|---|---|---|
| 50 kHz (default) | 20 us | just under 25,000 | about 5 % |
| 100 kHz | 10 us | just under 50,000 | about 10 % |
| 200 kHz | 5 us | just under 100,000 | about 20 % |

- The pulse is one tick wide, and the gap, the DIR setup and the DIR hold are each at least one tick. There is no separate pulse width setting. A drive that needs a 40 us pulse runs at a 25 kHz tick.
- The maximum rate is just under half the tick rate. Setters refuse a speed at or above it.
- The plan tick runs at 1 kHz by default. It is the stream rate limit: setpoints can arrive up to that rate.
- CPU is a Cortex-M4F at 144 MHz: about 20 cycles per axis per tick plus about 50 cycles of interrupt overhead. It scales with the tick rate, not the pulse rate.
- Pulse jitter is a few tens of ns when nothing else runs at the pulse priority.
- CPU and jitter figures are estimates until the bench (`docs/bench-step4.md`) measures them.
- Memory: about 1.4 KB per axis (the profile's 1 KB of taps is most of it) plus about 100 bytes of pulse state, and about 1.3 KB per group. Four axes and one group is under 8 KB.

## Known limits

- Stream tracking at the default 50 ms catch-up lags by 3 to 6 pulses on a fast sine, not the 2 pulses a short catch-up gives. A 1 kHz stream at 5 ms meets 2 pulses. Use `PulDir_setStreamCatchup` to tune it. The default is an open decision (E18 in the plan).
- A group member's travel is at most 2^24 pulses. Longer moves are refused.
- Position is a float in user units, so it is exact only to about seven digits (2^24 units). The axis keeps the count in whole pulses and re-bases the profile, so the count itself does not drift.
- A single move longer than 2^31 pulses wraps.
- Group objects need static storage.
- Members of a moving group refuse their limit setters and `PulDir_stop`.

## Tests and target compile

Host tests need only gcc, from bash:

```
bash test/run.sh
```

It builds and runs the generator tests, then the axis tests against `test/fake_port.c` at the default tick, at 200 kHz, at a 2 kHz plan rate on a 100 kHz tick, and with the tick counter near its wrap. It takes about 25 s. All use `-Wall -Wextra -Werror`.

Target compile, from bash with arm-none-eabi-gcc on the path and `$FW` set to a CubeMX project (STM32G473 here; change `-DSTM32G473xx` and the HAL include for another family):

```
for f in motion_profile.c puldir*.c port/stm32/puldir_port_stm32.c; do
  arm-none-eabi-gcc -std=c99 -O2 -Wall -Wextra -Werror -Wdouble-promotion \
    -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
    -DSTM32G473xx -DUSE_HAL_DRIVER -I. -Iport/stm32 -I$FW/Core/Inc \
    -I$FW/Drivers/STM32G4xx_HAL_Driver/Inc -I$FW/Drivers/CMSIS/Device/ST/STM32G4xx/Include \
    -I$FW/Drivers/CMSIS/Include -c "$f" -o /dev/null || exit 1
done
```

## Where the decisions are

- `docs/puldir-guidelines.md`: the design. Section 5.3 has the pulse tick and the numbers, 5.10 the port contract.
- `docs/superpowers/specs/2026-10-07-puldir-design.md`: the design review record.
- `docs/superpowers/plans/2026-10-07-puldir.md`: the plan. "Plan decisions" (D1 to D20) and the "Execution record" (E1 to E24) at the end say where the library differs from the guidelines. Where they differ, the execution record wins.
- `docs/bench-step4.md`: the bench for tick rates, jitter and CPU.
