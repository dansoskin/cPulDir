# puldir

puldir drives several PUL/DIR motor drives (steppers, closed-loop steppers, servo drives) from one microcontroller. Every pulse is written from a timer interrupt, so the pulse train does not depend on how busy the main loop is. Each axis has a jerk-limited motion profile with its own speed, acceleration and deceleration settings, live retargeting, jogging, homing, fault input handling and optional enable and release timing. Axes can move together along a straight line (groups), or follow a stream of position and velocity setpoints. The core is plain C99 with no HAL. The STM32 port is in `port/stm32/`.

## Adding it to a project

Add `motion_profile.c`, `puldir.c`, `puldir_pulse.c`, `puldir_plan.c`, `puldir_loop.c`, `puldir_group.c`, `puldir_console.c`, `puldir_enable.c`, `puldir_fault.c`, `puldir_homing.c`, `puldir_stream.c` and `port/stm32/puldir_port_stm32.c` to the build. Put the repo root and `port/stm32` on the include path. That directory supplies `puldir_pin.h`, which defines `puldir_pin_t` as a GPIO port and pin. Link with `-lm`. The library formats its log lines with `snprintf`, including `%f`, so a newlib-nano build must link float printf (`-u _printf_float`). newlib's `%f` allocates from the heap on its first use and needs roughly 1 KB of stack; logging happens only in main context, so both costs fall on the main loop, never on the interrupts. The planner alone (`motion_profile.c` and `.h`) needs nothing else.

## STM32 timer setup

puldir needs one timer and one interrupt vector: a pulse timer that writes the pins, and a plan IRQ that runs the profiles. The pulse tick pends the plan IRQ once every plan period, at the end of the tick that latches a new set, so the plan tick always starts just after the boundary it serves, even if a pulse tick was lost (E38 in the plan). Any timer with an update interrupt will do for the pulse timer. The plan IRQ is any vector the application does not otherwise use. The port works on any family, because it includes `main.h` and names no timer.

In this README, `htim_pulse` stands for your pulse timer's handle, `PULSE_TIM_IRQn` and `PULSE_TIM_IRQHandler` for its interrupt names, and `PLAN_IRQn` and `PLAN_IRQHandler` for the plan vector's.

**CubeMX.** For the pulse timer:

- Set the prescaler so the counter runs at 1 MHz: PSC = timer clock in MHz − 1. That is 143 on a 144 MHz timer clock, 169 on 170 MHz. Count up, no outputs.
- Leave the period alone. The library writes it in `PulDir_start`.
- Enable the update interrupt and give it the highest priority in the application (0).

For the plan IRQ, pick a vector nothing else uses. A timer's vector is a good choice; the timer itself is never started, so it raises nothing. On a G4, TIM7's `TIM7_DAC_IRQn` serves if the DAC2 and DAC4 interrupts are unused. Give it the next priority (1), below the pulse timer: either activate that timer in CubeMX, enable its interrupt and set the priority there, or skip CubeMX and call `HAL_NVIC_SetPriority(PLAN_IRQn, 1, 0)` before `PulDir_start`. `PulDir_start` clears and enables it, and calls `Error_Handler()` if its preemption priority is not below the pulse timer's: a vector left at its reset priority, 0, would hold the pulse ticks off for the whole plan tick. Put everything else (UART, DMA, USB, CAN, SysTick) at 2 or lower urgency. Anything left at 0 adds its run time to the pulse jitter.

**The IRQ handlers.** Call the raw entries `PulDir_stm32_pulseIrq()` and `PulDir_stm32_planIrq()`, not `HAL_TIM_IRQHandler`, whose flag checks cost more than the whole tick at a 5 µs pulse. Put the call in the generated handler's first USER CODE section and return before the HAL call. Regeneration keeps the user sections.

```c
/* in stm32xxxx_it.c; for example TIM6 and TIM7 on a G4: TIM6_DAC_IRQHandler, htim6, TIM7_DAC_IRQHandler */
void PULSE_TIM_IRQHandler(void)
{
  /* USER CODE BEGIN PULSE_TIM_IRQn 0 */
  PulDir_stm32_pulseIrq();
  return;                       /* skip the generated HAL call */
  /* USER CODE END PULSE_TIM_IRQn 0 */
  HAL_TIM_IRQHandler(&htim_pulse);
}

void PLAN_IRQHandler(void)
{
  /* USER CODE BEGIN PLAN_IRQn 0 */
  PulDir_stm32_planIrq();
  return;
  /* USER CODE END PLAN_IRQn 0 */
}
```

Include `puldir_port_stm32.h` in the file's USER CODE Includes section. The alternative is to untick "Generate IRQ handler" for the two interrupts (NVIC, Code generation) and define the handlers yourself, each with only the one call. If CubeMX does not know the plan vector (its timer not activated), it generates no handler: define `PLAN_IRQHandler` yourself, with only the one call, in any file. Shared vectors (`TIM1_UP_TIM16_IRQHandler` and the like) are fine: the pulse entry returns at once if its timer's update flag is clear and the plan entry unless the pulse tick pended it, so call it first and then whatever else the vector serves.

**In code.** Hand the pulse timer and the plan IRQ over first, before any `PulDir_init`, with their periods in µs:

```c
PulDir_stm32_attachPulseTimer(&htim_pulse, PULSE_TIM_IRQn, 20u);   /* pulse_us: one pulse tick */
PulDir_stm32_attachPlanIrq(PLAN_IRQn, 1000u);                      /* plan_us: one plan tick */
```

- `pulse_us` is the pulse width and the pulse tick. 20 gives a 50 kHz tick and up to just under 25,000 pulses/s per axis. 5 gives a 200 kHz tick and up to just under 100,000 pulses/s.
- `plan_us` is the plan period. 1000 (1 kHz) suits most uses.

Each call returns false and keeps nothing if its value is out of range or the timing is already fixed. The rules are under "Timing". Example 2 shows the calls in place. `PulDir_start` writes the pulse period, puts the counter one count before its first update, clears and enables the plan IRQ and starts the pulse timer; the first pulse tick pends the first plan tick. If either attach call was not made it calls `Error_Handler()`.

**Setup order.** Init refuses without the timers, register refuses an axis without init and pins or after start, and a group refuses members that are not registered:

1. Attach the pulse timer and the plan IRQ, with their periods.
2. `PulDir_attachLogger`, so init can log.
3. `PulDir_init` with the conversion, for each axis.
4. Pins, defaults, delays, homing.
5. `PulDir_register`.
6. `PulDirGroup_init`, if you use groups.
7. `PulDir_start`.

**Main context only.** Call every `PulDir_*` function from the main loop (or one thread), never from an interrupt, the UART's included. Pass commands to the main loop through a flag or a queue. The bench procedure for a first board is `docs/bench-step4.md`.

## Examples

All five use one gimbal axis or two: 280 pulses per degree, 15 deg/s, 30 deg/s², jerk 300 deg/s³. The pins are `puldir_pin_t` values, configured in CubeMX as outputs for pulse, direction and enable and an input for the alarm. With CubeMX user labels they look like this, and `PIN_PAN_PUL` and `PIN_PAN_DIR` the same way:

```c
#define PIN_TILT_PUL ((puldir_pin_t){ TILT_PUL_GPIO_Port, TILT_PUL_Pin })
#define PIN_TILT_DIR ((puldir_pin_t){ TILT_DIR_GPIO_Port, TILT_DIR_Pin })
#define PIN_TILT_ENA ((puldir_pin_t){ TILT_ENA_GPIO_Port, TILT_ENA_Pin })
#define PIN_TILT_ALM ((puldir_pin_t){ TILT_ALM_GPIO_Port, TILT_ALM_Pin })
```

### 1. The planner alone

Use a bare `MotionProfile` when something else does the moving and you only need a smooth setpoint: a servo over CAN, a PWM level, a virtual master axis (example 5). It needs no drive, no timers and nothing from the rest of the library.

```c
#include <stdbool.h>
#include "main.h"                       /* HAL_GetTick */
#include "motion_profile.h"

void servo_send(float pos, float vel);  /* yours: a CAN frame, a PWM level */

static MotionProfile tilt;              /* units are yours: here deg, deg/s */
static uint32_t tilt_ms;                /* the last millisecond advanced */

void tilt_setup(void)
{
    MotionProfile_init(&tilt, 0.0f, 0.001f);        /* start at 0 deg, dt = 1 ms */
    MotionProfile_setSettings(&tilt, (MotionSettings){ 15.0f, 30.0f, 30.0f, 300.0f });
    MotionProfile_setTarget(&tilt, 20.0f);          /* deg */
    tilt_ms = HAL_GetTick();
}

/* Every pass of the main loop. After a slow pass it catches up, so no time is lost. */
void tilt_loop(void)
{
    bool stepped = false;
    while ((uint32_t)(HAL_GetTick() - tilt_ms) >= 1u) {
        tilt_ms++;
        MotionProfile_advance(&tilt);
        stepped = true;
    }
    if (stepped)
        servo_send(tilt.pos, tilt.vel);
}
```

The other way to keep the beat is a 1 ms timer interrupt. Then mask that interrupt in main around `setTarget` and around reading `pos` and `vel`.

```c
extern TIM_HandleTypeDef htim_1ms;      /* a timer with a 1 ms update */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    if (htim == &htim_1ms)
        MotionProfile_advance(&tilt);
}
```

Read `tilt.pos` and `tilt.vel` after each advance, and `tilt.acc` if you need it. `tilt.done` is true once the profile is at the target with `vel` exactly 0. Here that takes 2,037 advances, about 2 s. `setTarget` works at any time, also in motion. For a jog and a stop:

```c
MotionProfile_setEndless(&tilt, -5.0f);   /* run at -5 deg/s with no target */
MotionProfile_stop(&tilt);                /* ramp down at the decel setting; done once at rest */
```

The profile measures no time itself. `dt` is fixed at init, and each `advance` assumes exactly `dt` has passed. Call it late and the motion runs slow; call it twice and it runs fast. The settings are speed, acceleration, deceleration and jerk. Speed and the two ramps are what a move uses (a short move never reaches the speed). Jerk is a bound the filtered motion stays under: an ordinary move peaks near half of it, since the window is sized for a reversal (E31). A jerk of 0 means no jerk limit. Setters return false and change nothing if given a bad value. The jerk filter holds at most `MOTION_PROFILE_MAX_TAPS` (256) samples, so 2 × max(accel, decel) / (jerk × dt) must be 256 or less (a reversal puts both signs of the larger ramp one window apart): at these ramps the jerk must be at least 234.4 deg/s³. Each profile is about 1 KB. `pos` is a float, so keep coordinates within about seven significant digits.

### 2. One motor

The full setup for one axis, in the order the library requires: the pulse timer and the plan IRQ, the logger, `PulDir_init` with the conversion, then pins, settings and delays, `PulDir_register`, and `PulDir_start` last. The conversion is fixed at init, so every setting after it is in user units.

```c
#include <stdio.h>
#include "main.h"
#include "puldir.h"
#include "puldir_port_stm32.h"

extern TIM_HandleTypeDef htim_pulse;              /* your pulse timer, from CubeMX */

static PulDir tilt;

static void log_line(const char* s) { printf("%s\r\n", s); }

void app_setup(void)
{
    if (!PulDir_stm32_attachPulseTimer(&htim_pulse, PULSE_TIM_IRQn, 20u)   /* first: 20 us pulse */
        || !PulDir_stm32_attachPlanIrq(PLAN_IRQn, 1000u))                   /* 1 ms plan tick */
        Error_Handler();
    PulDir_attachLogger(log_line);                    /* before init, which logs */
    if (!PulDir_init(&tilt, "tilt", 280.0f))          /* name up to 15 characters; pulses per deg */
        Error_Handler();                              /* the refusal is already logged */
    PulDir_attachPins(&tilt, PIN_TILT_PUL, PIN_TILT_DIR,
                      false,                          /* pulse active low? */
                      false);                         /* direction inverted? */
    PulDir_attachEnable(&tilt, PIN_TILT_ENA, false);  /* optional; true if active low */
    PulDir_attachFault(&tilt, PIN_TILT_ALM, false);   /* optional; true if active low */
    bool ok = PulDir_setDefaults(&tilt, 15.0f, 30.0f, 30.0f, 300.0f)
           && PulDir_setDelays(&tilt, 100u, 1000u);   /* ms: enable to first pulse, idle to release */
    PulDir_setAutoRelease(&tilt, true);               /* release the drive after each move */
    ok = ok && PulDir_register(&tilt);
    if (!ok) {
        PulDir_loop(&tilt);                           /* logs which call was refused, and why */
        Error_Handler();
    }
    PulDir_start();                                   /* after every axis is registered */
}

void app_loop(void)                                   /* every pass of the main loop */
{
    PulDir_loop(&tilt);                               /* logs refusals, landings and faults */
}
```

Then, from the main context:

```c
if (!PulDir_moveTo(&tilt, 45.0f)) {
    /* refused, and nothing changed; the next PulDir_loop logs why */
}
printf("tilt %s at %.3f deg, %.2f deg/s\r\n", PulDir_isMoving(&tilt) ? "moving" : "still",
       (double)PulDir_position(&tilt), (double)PulDir_velocity(&tilt));
```

At init the log reads `[tilt] max speed 89.2857 units/s, resolution 0.0036 units/pulse (pulse 20 us, plan 1000 us)`: the rate cap and one pulse, in degrees. What happens on the move: the enable pin goes active, the axis waits the 100 ms enable delay, then ramps out 12,600 pulses. `PulDir_loop` logs `[tilt] landed at 45.0000` and `isMoving` goes false, 3.8 s after the call. One second later the drive is released. Targets are rounded to whole pulses. `moveBy` is the relative form. Jog, stop and halt:

```c
PulDir_jog(&tilt, -5.0f);   /* run at -5 deg/s until told otherwise */
PulDir_stop(&tilt);         /* ramp down at the decel setting */
PulDir_halt(&tilt);         /* stop at once, no ramp */
```

Calls validate and refuse bad input. They never clamp. A refused call returns false, changes nothing and leaves one line for the next `PulDir_loop` to log:

```c
if (!PulDir_jog(&tilt, 100.0f)) {
    /* logged: "[tilt] jog refused: speed at or above the rate cap 89.2857" */
}
```

The cap is just under half the tick rate: 25,000 pulses/s at a 20 µs pulse, which is 89.3 deg/s here, the "max speed" init logged. Setup calls return false on refusal too, so check them. `PulDir_init` refuses a NULL name or one longer than 15 characters, a conversion that is not finite and positive, and missing or mismatched timing. It then writes nothing and logs the reason at once, for example `[tilt] init refused: timers not attached`. An axis whose init was refused is refused by `PulDir_register` too. A datum may be set with `PulDir_setPosition` before `PulDir_register`, which takes it over, but motion commands are refused until the axis is registered: `[tilt] moveTo refused: not registered`.

`PulDir_stop` and `PulDir_halt` leave the drive enabled, unless auto-release is on: then it is released after the release delay, as after any move. A fault input latch halts the axis and always keeps the drive enabled; `PulDir_clearFault` and `PulDir_attachFaultClear` handle the clear. Homing is `PulDir_setHoming`, then `PulDir_homeStart`; while it runs the setters of speed, acceleration, deceleration and jerk are refused. `PulDir_console(&tilt, cmd, arg)` is a one-letter command console for a UART, and `PulDir_consoleMenu` prints the verbs.

### 3. Two motors, independent

Two axes, each with its own settings, each commanded on its own. Use it when the axes do not need to arrive together. The includes, `log_line` and the timer handle are as in example 2.

```c
static PulDir pan, tilt;

static bool axis_setup(PulDir* a, const char* name, puldir_pin_t pul, puldir_pin_t dir,
                       float speed, float accel)
{
    if (!PulDir_init(a, name, 280.0f))  /* pulses per deg; a refusal is logged */
        return false;
    PulDir_attachPins(a, pul, dir, false, false);
    return PulDir_setDefaults(a, speed, accel, accel, 10.0f * accel)
        && PulDir_register(a);
}

void app_setup(void)
{
    if (!PulDir_stm32_attachPulseTimer(&htim_pulse, PULSE_TIM_IRQn, 20u)
        || !PulDir_stm32_attachPlanIrq(PLAN_IRQn, 1000u))
        Error_Handler();
    PulDir_attachLogger(log_line);
    if (!axis_setup(&pan, "pan", PIN_PAN_PUL, PIN_PAN_DIR, 15.0f, 30.0f)
        || !axis_setup(&tilt, "tilt", PIN_TILT_PUL, PIN_TILT_DIR, 10.0f, 20.0f)) {
        PulDir_loop(&pan);
        PulDir_loop(&tilt);
        Error_Handler();
    }
    PulDir_start();
}

void app_loop(void)
{
    PulDir_loop(&pan);                  /* every axis, every pass */
    PulDir_loop(&tilt);
}
```

Start both, then retarget pan while both move:

```c
PulDir_moveTo(&pan, 60.0f);             /* both start now */
PulDir_moveTo(&tilt, -20.0f);

/* 1 s later, while both move: */
PulDir_moveTo(&pan, 5.0f);              /* pan turns round, tilt carries on */
```

What happens: at 1 s pan is near 10 deg at full speed. It ramps down, turns round near 15 deg and lands at 5 deg, 2.9 s after the start. Tilt never notices and lands at -20 deg at 2.7 s. They are not synchronised: each follows its own ramp and they finish at different times. For a shared finish, use a group.

### 4. Two motors in sync

A group moves its members along one shared path: a straight line in joint space, from where they are to one target each. Every member's final pulse falls on the same tick. Use it for coordinated moves where the axes must start and finish together. `pan` and `tilt` are set up as in example 3.

```c
static PulDirGroup gimbal;              /* static: the library keeps a pointer to it */
static PulDir* gimbal_axes[] = { &pan, &tilt };

/* in app_setup, after both axes are registered and before PulDir_start */
if (!PulDirGroup_init(&gimbal, gimbal_axes, 2u))
    Error_Handler();
```

```c
const float to[] = { 60.0f, -20.0f };   /* pan, tilt in deg */
if (!PulDirGroup_moveTo(&gimbal, to)) {
    /* refused, e.g. a member still moving; logged by that member's PulDir_loop */
}

if (!PulDirGroup_isMoving(&gimbal)) { /* both have landed */ }
PulDirGroup_stop(&gimbal);              /* ramp down along the line */
PulDirGroup_halt(&gimbal);              /* stop at once */
```

What happens: each member's own settings, over its travel, bound the path's, and the smallest wins. Pan has 60 deg to go at 15 deg/s and tilt 20 deg at 10 deg/s, so pan sets the pace and tilt runs at a third of pan's speed. Both land 4.7 s after the call, their last pulses on the same tick.

Members must be idle to join a move. While it runs they refuse their own commands and changes to their settings; `PulDir_halt` on a member halts the whole group. A fault on any member halts the group and `PulDirGroup_faulted` reports it. Each member's travel is limited to 2^24 pulses (59,918 deg here). After a `PulDirGroup_stop` each member ends within one pulse of the line, but the final edges need not share a tick.

### 5. Two motors in CSP, fed by a profile

When the axes must follow a curve, a group is not enough, because it only moves in straight lines. Here a camera traces a 10 deg circle: pan and tilt are sine and cosine functions of one master angle. A bare `MotionProfile` (example 1) is the master, advanced every 1 ms in the main loop. The axes follow through the stream calls (cyclic synchronous position). The axes are set up as in example 3.

```c
#include <math.h>

#define R_DEG 10.0f                     /* circle radius, deg */

static MotionProfile scan;              /* the master: an angle in rad */
static uint32_t scan_ms;
static bool scanning;
static float pan0, tilt0;               /* where the circle starts */

bool scan_start(void)
{
    MotionProfile_init(&scan, 0.0f, 0.001f);
    MotionProfile_setSettings(&scan, (MotionSettings){ 1.0f, 1.0f, 1.0f, 10.0f });  /* rad/s, rad/s², rad/s³ */
    MotionProfile_setTarget(&scan, 6.2831853f);                                     /* one turn */
    pan0 = PulDir_position(&pan);
    tilt0 = PulDir_position(&tilt);
    scanning = PulDir_streamBegin(&pan) && PulDir_streamBegin(&tilt);
    if (!scanning)
        PulDir_streamEnd(&pan);         /* pan may have begun; this stops it */
    scan_ms = HAL_GetTick();
    return scanning;
}

void scan_loop(void)                    /* every pass of the main loop */
{
    if (!scanning)
        return;
    if (!PulDir_inStream(&pan) || !PulDir_inStream(&tilt)) {
        PulDir_streamEnd(&pan);         /* timed out after a stall: both are stopping */
        PulDir_streamEnd(&tilt);
        scanning = false;
        return;
    }
    bool stepped = false;
    while ((uint32_t)(HAL_GetTick() - scan_ms) >= 1u) {
        scan_ms++;
        MotionProfile_advance(&scan);
        stepped = true;
    }
    if (!stepped)
        return;
    const float th = scan.pos, w = scan.vel;        /* rad, rad/s */
    /* position = f(th), velocity = f'(th) * w, the chain rule */
    PulDir_stream(&pan, pan0 + R_DEG * sinf(th), R_DEG * cosf(th) * w);
    PulDir_stream(&tilt, tilt0 + R_DEG * (1.0f - cosf(th)), R_DEG * sinf(th) * w);
    if (scan.done) {
        PulDir_streamEnd(&pan);         /* ramp to rest, back on the defaults */
        PulDir_streamEnd(&tilt);
        scanning = false;
    }
}
```

What happens: the master turns 2π rad in 7.5 s. Each axis peaks at 10 deg/s and 10 deg/s², well under its speed and acceleration settings. Each tracks the master within 1.3 pulses, and both end at rest on the start of the circle 0.2 s after `streamEnd`.

- `PulDir_stream` is main-context only, so the beat lives in the main loop. Send a setpoint only on a new step; resending an old one would restart the hold from a stale point.
- Setpoints may arrive at 200 Hz up to the plan rate. The axis holds each one to first order (position plus velocity times age) until the next.
- If the main loop stalls for 100 ms (`PULDIR_STREAM_TIMEOUT_MS`), each axis stops on its own at its decel, logs `[pan] stream timed out`, and `PulDir_streamTimedOut` reads true. To go on, call `PulDir_streamBegin` again.
- At the default 50 ms catch-up, expect a few pulses of lag on sharp accelerations: 3 to 6 pulses on a sine at 27,600 pulses/s². `PulDir_setStreamCatchup(&pan, 5u)` lowers it; a 1 kHz stream at 5 ms tracks within 2 pulses. The minimum is five plan intervals. The default is an open decision (E18 in the plan). `PulDir_streamError` gives the tracking error in user units.

## The port

`puldir_port.h` is the contract: `port_pin_write` (one register write: the pulse tick writes the pulse and direction pins, main writes the enable and clear pins under the lock, and the attach calls park every output with it before `PulDir_start`), `port_pin_read`, `port_start(pulse_tick, plan_tick)`, `port_plan_pend`, and `port_lock` and `port_unlock`. Pin modes are the application's (CubeMX's) business. The lock must mask at least the library's two interrupts, and no other interrupt may run inside it, so the pulse tick waits at most the longest locked section (E39). Every port takes a pulse timer counting at `PULDIR_TIMER_HZ` (1 MHz, 1 µs per count) and a way to run the plan tick on demand below the pulse tick's priority. Its attach calls take the two periods in µs and hand them to the library with `puldir_timing_pulse` and `puldir_timing_plan`, keeping the timer or the IRQ only if the library accepts its period. `port_start` then sets the pulse timer's period to `pulse_us` counts and starts it. The pulse tick calls `port_plan_pend` at the end of every boundary tick, the one that latches a new set, every N ticks from the first; the port makes the plan tick run as soon as the pulse tick returns. So the plan period is exactly N pulse periods by construction, and each plan tick starts just after the latch it serves with N − 1 ticks to publish, even after a lost or late pulse tick (E38). The port cannot check that the counter runs at 1 MHz. The scope does.

The STM32 `port_lock` saves PRIMASK and masks every interrupt; `port_unlock` restores PRIMASK. A locked section is a few stores, so other interrupts wait at most that long too. A tick that comes due inside the lock fires on unlock, late by the lock's duration, and no other interrupt can run inside the lock to lengthen it. (Masking only the two library interrupts in the NVIC let a UART, DMA, CAN or USB interrupt that preempted main inside a locked section hold the pulse tick off for its whole run: on the bench that lost pulse ticks, E39.) If the plan tick publishes after its boundary (it ran longer than its N − 1 ticks of slack, or was held past them), the pulse tick latches the set on the next tick and runs it for the rest of the interval, and the next plan tick corrects the shortfall from the real position. The axes still land exactly when a set is at most two ticks late, at any speed; k ticks late at an approach increment of i pulses per tick, one extra pulse past the target appears once k × i reaches a pulse (three ticks late above a third of a pulse per tick). `PulDir_lateSets` counts those sets and `PulDir_loop` logs a change of the count as `plan tick late, total N`, at most once a second, as it does `pulse tick overrun, total N` (E40); a count that grows means the plan tick runs too long (bench step 6). The lock works before anything is attached. The host tests use `test/fake_port.c`, where pins are an array and the test calls the ticks.

## Configuration macros

Define these on the compiler command line. Each default lives in the header of the module that uses it.

| macro | default | header | meaning |
|---|---|---|---|
| `PULDIR_MAX_AXES` | 8u | `puldir.h` | axis table size |
| `PULDIR_MAX_GROUPS` | 4u | `puldir_group.h` | group table size |
| `PULDIR_CATCHUP_MS` | 50u | `puldir_stream.h` | stream catch-up time |
| `PULDIR_STREAM_TIMEOUT_MS` | 100u | `puldir_stream.h` | stream setpoint timeout |
| `PULDIR_FAULT_DEBOUNCE_MS` | 2u | `puldir_fault.h` | fault input must stay active this long |
| `PULDIR_FAULT_CLEAR_MS` | 10u | `puldir_fault.h` | fault clear pulse width |
| `PULDIR_HOMING_DELAY_MS` | 1000u | `puldir_homing.h` | pause between homing phases |
| `PULDIR_TICK_START` | 0u | `puldir.h` | first tick count; tests only |
| `MOTION_PROFILE_MAX_TAPS` | 256 | `motion_profile.h` | jerk filter window; raise for softer jerk |

## Timing

The two rates are arguments of the attach calls, not macros, so one build serves every rate.

- `pulse_us`, the pulse width and pulse tick, is 2 to 1000 µs. The tick rate is 1e6 / `pulse_us`.
- `plan_us`, the plan period, is 250 to 10000 µs (the generator's dt range).
- The plan period must be a whole number of pulse periods: `plan_us` % `pulse_us` == 0, and N = `plan_us` / `pulse_us` in 2 to 4096.

Each attach call checks its own range. `PulDir_init` checks the ratio, refuses without valid timing, and fixes the timing: from the first accepted init (or `PulDir_start`) on, attach calls are refused. `PulDirGroup_init` refuses without timing too.

## The numbers

| `pulse_us` | tick rate | max pulses/s per axis | four axes, CPU (estimate) |
|---|---|---|---|
| 20 | 50 kHz | just under 25,000 | about 5 % |
| 10 | 100 kHz | just under 50,000 | about 10 % |
| 5 | 200 kHz | just under 100,000 | about 20 % |

- The pulse is one tick wide, and the gap, the DIR setup and the DIR hold are each at least one tick. A drive that needs a 40 µs pulse attaches the pulse timer with `pulse_us` 40, a 25 kHz tick.
- The maximum rate is just under half the tick rate. Setters refuse a speed at or above it. `PulDir_init` logs it in user units.
- The plan tick runs every `plan_us`, 1 kHz at 1000, pended by the pulse tick every N ticks. It is the stream rate limit: setpoints can arrive up to that rate.
- CPU is a Cortex-M4F at 144 MHz: about 20 cycles per axis per tick plus about 50 cycles of interrupt overhead. It scales with the tick rate, not the pulse rate.
- Pulse jitter is a few tens of ns when nothing else runs at the pulse priority. CPU and jitter figures are estimates until the bench (`docs/bench-step4.md`) measures them.
- Memory: about 1.4 KB per axis (the profile's 1 KB of taps is most of it) plus about 100 bytes of pulse state, and about 1.3 KB per group. Four axes and one group is under 8 KB.

## Known limits

- Stream tracking at the default 50 ms catch-up lags by a few pulses on sharp accelerations (example 5).
- Position is a float in user units, so it is exact only to about seven digits (2^24 units). The axis keeps the count in whole pulses and re-bases the profile, so the count itself does not drift.
- A single move longer than 2^31 pulses wraps.
- Build the library at -O2, even in a firmware built at -O0 (per-file options). At -O0 the longest locked sections take several µs: the plan tick's snapshot about 1.3 µs per axis, a group halt about 2.3 µs per member, the plan tick's fault halt of a group about 4 µs per member, a settings change about 5 µs, at 144 MHz. With four axes or members some of them pass a 10 µs pulse period, and a pulse tick held past its next update is lost (E39).

## Tests and target compile

Host tests need only gcc, from bash:

```
bash test/run.sh
```

It builds and runs the generator tests, then builds the axis tests against `test/fake_port.c` once and runs every case at three timings: 20/1000 µs (a 50 kHz tick), 5/1000 µs (200 kHz) and 10/500 µs (a 2 kHz plan rate on a 100 kHz tick). A second build starts the tick counter near its wrap and runs at 20/1000 µs. One case runs as `build/test/test_puldir <case> <pulse_us> <plan_us>`. It takes about 25 s. All use `-Wall -Wextra -Werror`.

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
- `docs/superpowers/plans/2026-10-07-puldir.md`: the plan. "Plan decisions" (D1 to D20) and the "Execution record" (E1 to E40) at the end say where the library differs from the guidelines. Where they differ, the execution record wins. E38 starts the plan tick from the pulse tick instead of a timer of its own, E39 makes the STM32 lock mask every interrupt, and E40 logs the overrun and late-set totals at most once a second each.
- `docs/bench-step4.md`: the bench for tick rates, jitter and CPU.

## GUI

`gui/` holds a PyQt5 front end for the console: a reusable axis panel
(`gui/puldir_view`) that any app can embed, and a demo app with a terminal. See
[gui/README.md](gui/README.md).
