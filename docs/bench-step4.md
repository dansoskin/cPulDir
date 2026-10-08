# Bench step 4: the STM32 port under load

For the owner. Section 9, step 4 of `puldir-guidelines.md`. Everything here is on a branch of `164-race_firmware`; this repository only supplies the port, which compiles clean for the G473 against the firmware's HAL (`arm-none-eabi-gcc -Wall -Wextra -Werror`, with `-DSTM32G473xx -DUSE_HAL_DRIVER` and nothing else extra).

The bench drives the real library through its console (jog commands) rather than hand-written increments, because the plan tick exists by the time you run this.

## 1. What is measured, and why

Guidelines 5.3 and 5.9 estimate the pulse tick's cost and jitter. They are estimates. This bench turns them into numbers on real pins, with four axes pulsing and the application's USB, CAN and UART interrupts running:

- pulse width (it is one tick, `BENCH_PULSE_US`, the pulse timer's attach argument: 20 us, a 50 kHz tick, then 5 us, a 200 kHz tick),
- period jitter, peak to peak, on the GPIO_OUT1 marker and on a PUL pin at a rate that divides the tick evenly,
- the pulse handler's duration, typical and worst,
- CPU load = handler duration x tick rate,
- overrun count after one minute (`PulDir_overruns()`),
- late-set count after one minute (`PulDir_lateSets()`): sets the plan tick published after their boundary,
- the plan tick's phase: it must run just after the pulse tick that latches a set.

It runs at `BENCH_PULSE_US` = 20 and 5. If a pulse length is hopeless, only that argument changes; the library writes both timer periods from the attach arguments, and nothing else changes.

## 2. CubeMX (on a branch of the firmware)

The library itself names no timer and no STM32 family: it takes any two timers whose counters run at 1 MHz. This bench uses TIM6 and TIM7 on the G473. The `.ioc` has neither. The timer clock is 144 MHz (`RCC.APB1TimFreq_Value=144000000`). The NVIC priority group is 4 (16 preemption levels, no sub-priority).

| timer | role | prescaler (PSC) | counter |
|---|---|---|---|
| TIM6 | pulse tick | 143 | 144e6 / 144 = 1 MHz |
| TIM7 | plan tick | 143 | 144e6 / 144 = 1 MHz |

Leave the period (ARR) at anything: `port_start` writes it from the attach arguments, `pulse_us` counts for the pulse timer and `plan_us` counts (1000) for the plan timer, so the plan period is exactly N pulse periods (N = 50 at 20 us, 200 at 5 us). The library latches increment sets only at interval boundaries, every N pulse ticks counted from the first one. `port_start` puts both counters one count before their first update and starts the pulse timer, then the plan timer, back to back. Each plan update then follows a latch tick by the few cycles between the two starts; the pulse tick has the higher priority and runs first, so the plan tick has the whole interval to publish.

Both: Activated, no clock-source options (basic timers), counter mode up, auto-reload preload either way, trigger output reset. In NVIC Settings enable "TIM6, DAC1 and DAC3 global interrupt" and "TIM7, DAC2 and DAC4 global interrupt" (the update interrupt).

Priorities (preemption, sub is always 0). Today USART2, DMA1 channels 1 and 2 and FDCAN1 IT0/IT1 are all at 0, the highest; USB is at 5 (`usb_handler.c`); SysTick is set to 0 in `setup()`.

| interrupt | today | set to |
|---|---|---|
| TIM6_DAC (pulse tick) | n/a | 0 |
| TIM7_DAC (plan tick) | n/a | 1 |
| USART2 | 0 | 2 |
| DMA1 channel 1 | 0 | 2 |
| DMA1 channel 2 | 0 | 2 |
| FDCAN1 IT0, IT1 | 0 | 2 |
| USB (HP, LP, wake-up) | 5 | 5, unchanged |
| SysTick | 0 | 0 as is (the one allowed peer, a few hundred ns per ms); optionally move `HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0)` in `functions.c` (line 57) to priority 1 so it cannot add to the pulse jitter |

Note that `stm32g4xx_hal_msp.c` also calls `HAL_NVIC_SetPriority(USART2_IRQn / FDCAN1_IT*_IRQn, 0, 0)` and the generated `MX_DMA_Init` sets the DMA ones; CubeMX writes these from the `.ioc`, so change them there, not in the generated files. After regenerating, verify by reading `NVIC->IP[]` or the generated lines.

### Regeneration warning

CubeMX will generate `TIM6_DAC_IRQHandler` and `TIM7_DAC_IRQHandler` in `Core/Src/stm32g4xx_it.c` with a body of `HAL_TIM_IRQHandler(&htim6);`. **Regeneration overwrites everything outside the `USER CODE` markers.** So: either untick "Call HAL handler" for TIM6 and TIM7 in NVIC Settings > Code generation (the handler body is then empty between the markers), or after each regeneration delete the generated `HAL_TIM_IRQHandler(...)` line. Your code goes only inside the user sections below (`USER CODE BEGIN TIM6_DAC_IRQn 0` and `... TIM7_DAC_IRQn 0`, in `stm32g4xx_it.c`; `USER CODE BEGIN Includes` and `USER CODE BEGIN PV`/`PFP` as marked). Code placed anywhere else is lost.

## 3. Code to add

### 3.1 Submodule and CMake

```bash
git submodule add https://github.com/dansoskin/cPulDir Drivers/cPulDir
```

In `CMakeLists.txt`, in the existing user sections:

```cmake
# in target_sources(... PRIVATE
    ${CMAKE_SOURCE_DIR}/Drivers/cPulDir/motion_profile.c
    ${CMAKE_SOURCE_DIR}/Drivers/cPulDir/puldir.c
    ${CMAKE_SOURCE_DIR}/Drivers/cPulDir/puldir_pulse.c
    ${CMAKE_SOURCE_DIR}/Drivers/cPulDir/puldir_plan.c
    ${CMAKE_SOURCE_DIR}/Drivers/cPulDir/puldir_loop.c
    ${CMAKE_SOURCE_DIR}/Drivers/cPulDir/puldir_group.c
    ${CMAKE_SOURCE_DIR}/Drivers/cPulDir/puldir_console.c
    ${CMAKE_SOURCE_DIR}/Drivers/cPulDir/port/stm32/puldir_port_stm32.c
    ${CMAKE_SOURCE_DIR}/Core/Src/puldir_bench.c

# in target_include_directories(... PRIVATE
    ${CMAKE_SOURCE_DIR}/Drivers/cPulDir
    ${CMAKE_SOURCE_DIR}/Drivers/cPulDir/port/stm32
```

No compile definitions: the rates are arguments of the timer attach calls in `puldir_bench.c`.

(`puldir_console.c` is required: `bench_console` calls `PulDir_console`. `puldir_group.c` is required too: the plan tick calls into it, so the link fails without it.)

### 3.2 `Core/Src/stm32g4xx_it.c`

Add the include and the two handlers' bodies inside the user sections (the function shells are CubeMX's):

```c
/* USER CODE BEGIN Includes */
#include "puldir_port_stm32.h"
void bench_plan_phase(void);
/* USER CODE END Includes */
```

```c
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
  GPIO_OUT1_GPIO_Port->BSRR = GPIO_OUT1_Pin;             /* scope: handler duration */
  PulDir_stm32_pulseIrq();
  GPIO_OUT1_GPIO_Port->BSRR = (uint32_t)GPIO_OUT1_Pin << 16;
  return;                                                 /* skip any generated HAL call */
  /* USER CODE END TIM6_DAC_IRQn 0 */
}

void TIM7_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_DAC_IRQn 0 */
  bench_plan_phase();                                     /* step 6: where the plan tick falls */
  PulDir_stm32_planIrq();
  return;
  /* USER CODE END TIM7_DAC_IRQn 0 */
}
```

If CubeMX generated `HAL_TIM_IRQHandler(&htim6);` after the user section, the early `return` makes it dead code, but delete it anyway when you can (the compiler may warn about unreachable code only with `-Wunreachable-code`).

Note GPIO_OUT1, 2 and 3 (PB7, PC6, PC7) are alternate-function timer pins in the `.ioc` (TIM4_CH2, TIM8_CH1, TIM8_CH2), so `BSRR` would not drive them. `bench_setup()` below reconfigures them as push-pull outputs at run time, after `MX_*_Init`, so you do not have to edit the `.ioc` for them. The bench therefore must not run together with anything using TIM4_CH2 or TIM8 outputs; check nothing does in the current build.

### 3.3 `Core/Src/puldir_bench.c` (new file)

```c
#include "main.h"
#include "globals.h"
#include "puldir.h"
#include "puldir_port_stm32.h"
#include <stdlib.h>

extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;

#define PIN(n) ((puldir_pin_t){ n##_GPIO_Port, n##_Pin })
static PulDir bench[4];

/* puldir lines carry no terminator */
static void bench_log(const char* s) { log_data("%s\r\n", s); }

#define BENCH_PULSE_US 20u              /* 5u for the second run */
#define BENCH_PLAN_US 1000u

/* PulDir_ticks() % N on entry to every plan tick (step 6). */
#define BENCH_N (BENCH_PLAN_US / BENCH_PULSE_US)
static volatile uint32_t phase_min = BENCH_N, phase_max;

void bench_plan_phase(void)
{
    const uint32_t ph = PulDir_ticks() % BENCH_N;
    if (ph < phase_min)
        phase_min = ph;
    if (ph > phase_max)
        phase_max = ph;
}

static void out_pin(GPIO_TypeDef* port, uint16_t pin)
{
    GPIO_InitTypeDef g = {0};
    g.Pin = pin;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(port, &g);
}

void bench_setup(void)
{
    /* PUL and DIR pins at full speed, and the three scope/axis-4 pins as plain outputs. */
    out_pin(MOTOR1_PUL_GPIO_Port, MOTOR1_PUL_Pin);  out_pin(MOTOR1_DIR_GPIO_Port, MOTOR1_DIR_Pin);
    out_pin(MOTOR2_PUL_GPIO_Port, MOTOR2_PUL_Pin);  out_pin(MOTOR2_DIR_GPIO_Port, MOTOR2_DIR_Pin);
    out_pin(MOTOR3_PUL_GPIO_Port, MOTOR3_PUL_Pin);  out_pin(MOTOR3_DIR_GPIO_Port, MOTOR3_DIR_Pin);
    out_pin(GPIO_OUT1_GPIO_Port, GPIO_OUT1_Pin);    /* handler-duration marker */
    out_pin(GPIO_OUT2_GPIO_Port, GPIO_OUT2_Pin);    /* axis 4 PUL */
    out_pin(GPIO_OUT3_GPIO_Port, GPIO_OUT3_Pin);    /* axis 4 DIR */

    /* The timers first, then the logger: init logs each axis's top speed. */
    if (!PulDir_stm32_attachPulseTimer(&htim6, TIM6_DAC_IRQn, BENCH_PULSE_US)
        || !PulDir_stm32_attachPlanTimer(&htim7, TIM7_DAC_IRQn, BENCH_PLAN_US))
        Error_Handler();
    PulDir_attachLogger(bench_log);

    const puldir_pin_t pul[4] = { PIN(MOTOR1_PUL), PIN(MOTOR2_PUL), PIN(MOTOR3_PUL), PIN(GPIO_OUT2) };
    const puldir_pin_t dir[4] = { PIN(MOTOR1_DIR), PIN(MOTOR2_DIR), PIN(MOTOR3_DIR), PIN(GPIO_OUT3) };
    /* The same enable pins the stock firmware uses (gimbal_axes.c), active high. */
    const puldir_pin_t ena[3] = { PIN(FET1_MOTOR1), PIN(FET1_MOTOR2), PIN(MOTOR3_ENA) };
    static const char* const name[4] = { "b1", "b2", "b3", "b4" };

    for (int i = 0; i < 4; i++)
    {
        (void)PulDir_init(&bench[i], name[i], 1.0f);           /* units are pulses */
        PulDir_attachPins(&bench[i], pul[i], dir[i], false, false);
        if (i < 3)
            PulDir_attachEnable(&bench[i], ena[i], false);
        /* pulses/s, pulses/s^2: just under the tick ceiling (tick/2), no jerk limit */
        const float v = 0.495f * (1.0e6f / (float)BENCH_PULSE_US);
        (void)PulDir_setDefaults(&bench[i], v, 2.0f * v, 2.0f * v, 0.0f);
        (void)PulDir_register(&bench[i]);
    }

    /* Whether HAL left the update flags set decides the plan tick's phase (step 6). */
    log_data("UIF before start: pulse %lu plan %lu\r\n",
             (unsigned long)((htim6.Instance->SR & TIM_SR_UIF) != 0u),
             (unsigned long)((htim7.Instance->SR & TIM_SR_UIF) != 0u));
    PulDir_start();
}

void bench_loop(void)
{
    for (int i = 0; i < 4; i++)
        PulDir_loop(&bench[i]);
}

/* B<axis>,<cmd><arg>   e.g. B1,J24000   B2,k   Bs */
void bench_console(const char* axis, const char* cmd)
{
    if (axis[0] == 's' && axis[1] == '\0')
    {
        log_data("ticks %lu  overruns %lu  late sets %lu  plan phase %lu..%lu\n",
                 (unsigned long)PulDir_ticks(), (unsigned long)PulDir_overruns(),
                 (unsigned long)PulDir_lateSets(), (unsigned long)phase_min, (unsigned long)phase_max);
        return;
    }
    int n = atoi(axis);
    if (n < 1 || n > 4 || cmd[0] == '\0')
    {
        log_data("Usage: B<1..4>,<verb><arg>  or  Bs\n");
        return;
    }
    PulDir_console(&bench[n - 1], cmd[0], cmd + 1);
}
```

Axes 1 to 3 have enable pins, so their first motion command drives the enable pin active (guidelines 5.8). The enable delay and the release delay are left at 0 and auto-release is off, as cFlexyStepper behaved: the pin stays active after the jog stops, and only `PulDir_enable(false)` (the console's `K`, which halts first) lowers it. Before connecting a drive for the pulse-width check, add `(void)PulDir_setDelays(&bench[i], <ms>, 0u);` in the loop if the drive's datasheet asks for a wait between enable and the first pulse.

If `PulDir_setDefaults` refuses the limits (it logs the refusal), lower them. The console verbs are those of `puldir_console.c`, the same letters as `@` in `cFlexyStepper_decode.c` plus `Z`/`z` for jerk: `J<speed>` jog, `k` stop, `K` halt then release the drive, `s` status. `PulDir_consoleMenu` prints the full list. Axis 4 has no enable pin, so on it `K` halts and replies `halted, no enable pin to release`; that is not a failure. Replies go through the logger attached with `PulDir_attachLogger`.

### 3.4 `Core/Src/decode_packet.c`

Top-level letters already taken in `decode_uart`: `~ ! @ # $ p P R T S s K k`. `B` is free, so it is the bench verb. (Checked against the whole `switch`.) Add the prototype near the other forward declarations:

```c
void bench_console(const char* axis, const char* cmd);
```

and this case before `default:`:

```c
        case 'B':   /* bench: B<axis>,<verb><arg> to puldir, Bs = ticks, overruns, late sets, plan phase */
            bench_console(result[0], result[1]);
            break;
```

`result[0]` is the text after `B` up to the first comma, `result[1]` the rest. So `Bs` gives axis `s`; `B1,J24000` gives axis `1`, cmd `J24000`.

### 3.5 `Core/Src/functions.c`

```c
void bench_setup(void);
void bench_loop(void);
```

at the top, `bench_setup();` as the last line of `setup()` (after `roulette_setup();`, so the SysTick and USB priorities are already set), and `bench_loop();` at the top of `loop()`.

Do not command the old axes (`@`, `#`, `R`, `K`) during the bench: they share the MOTOR pins. Build with `BENCH_PULSE_US` 20u first.

## 4. Procedure

1. Drives disconnected. Scope probes: PUL of axes 1 to 4 (PC12, PE9, PE8, PC6) and `GPIO_OUT1` (PB7, the handler marker). Trigger on the marker for the duration and its edge-to-edge jitter. The pulse is always one tick wide. It is the spacing between pulses, rising edge to rising edge, that is 2 or 3 ticks at 24,000 pulses/s on a 50 kHz tick (40 or 60 us; 10 or 15 us at 200 kHz, 99,000/s), so PUL jitter is only meaningful at a rate that divides the tick evenly (step 3a).
2. Start USB log streaming (the GUI's usual stream), CAN traffic (the 1 Hz ping or a heavier source), and a stream of UART console lines (for example `p` at 100 Hz). Keep all of it running through every run below.
3. **50 kHz run** (`BENCH_PULSE_US` 20u): send `Bs` (record ticks), then `B1,J24000`, `B2,J24000`, `B3,J24000`, `B4,J24000`. Let it run one minute. Measure on the scope: pulse width, period jitter peak to peak (persistence on a rising edge of one PUL, trigger on the previous one), marker width typical and worst. Send `Bs` again: ticks must have advanced by 50,000 per second, and record overruns. Stop with `B1,k` to `B4,k` (or `B1,K` to `B4,K` to halt).
3a. **Jitter run, 50 kHz**: jog axis 1 alone at 12,500 pulses/s (`B1,J12500`, a PUL period of exactly 4 ticks) with the same traffic; measure the PUL period jitter peak to peak. Also measure the marker's period jitter (edge to edge, rising edge to rising edge): that is the pulse tick's own jitter, and it needs no divisibility.
4. **200 kHz run**: set `BENCH_PULSE_US` to 5u and rebuild. Jog all four at 99,000 pulses/s (`B1,J99000` ... `B4,J99000`), same traffic, one minute, same measurements.
4a. **Jitter run, 200 kHz**: jog axis 1 alone at 50,000 pulses/s (a PUL period of exactly 4 ticks), same measurements.
5. Connect one drive (axis 1). At 50 kHz the pulse is 20 us wide. Check the drive's datasheet minimum pulse width and DIR setup time, then run `B1,J2000`, `B1,k`, and confirm it turns and stops without lost or extra steps (mark the shaft, count revolutions: 2000 pulses per second for ten seconds is 20,000 pulses).
6. **Phasing**, at both rates. After reset, record the `UIF before start` line. Then send `Bs` before and after a minute of jogging. The simple check is `late sets`, `PulDir_lateSets()`: it counts every set the plan tick published after its boundary, and it must read 0 and stay 0. A late set latches on the next tick and runs the rest of its interval, and the next plan tick corrects the shortfall, so the axes still land exactly; but a count that grows means the plan tick is running too late or too long, and `PulDir_loop` logs each new batch as `plan tick late, total N`. `plan phase a..b` says why: it is `PulDir_ticks() % N` on entry to every plan tick so far, and must read `1..1`, the plan tick running just after the pulse tick that latches a set, with the whole interval to publish. `0..0` means it runs one tick before the latch and has one tick to publish: any plan tick longer than that is a late set. The firmware's HAL sets URS around its UG, so `UIF before start: pulse 0 plan 0` is expected; `port_start` forces its own update and clears the flag either way, then puts both counters one count before their first update. If you read `0..0`, that preset is not taking effect: report it before anything is built on the port.

## 5. Results

CPU load = (typical handler duration, from the marker width plus about 24 cycles (170 ns at 144 MHz) of exception entry and exit that the marker does not cover) x tick rate. Pass: zero overruns, zero late sets, jitter under 1 us (from the jitter runs 3a and 4a, not the full-rate runs), CPU at most 10 % (50 kHz) or 25 % (200 kHz).

| measurement | 50 kHz, 4 axes at 24,000/s | 200 kHz, 4 axes at 99,000/s |
|---|---|---|
| pulse width (us) | | |
| period jitter, one PUL at 12,500/s (50 kHz) or 50,000/s (200 kHz) (us) | | |
| period jitter, marker (us) | | |
| pulse handler, typical (us) | | |
| pulse handler, worst (us) | | |
| CPU load (typical duration x rate, %) | | |
| overruns after one minute (`Bs`) | | |
| late sets after one minute (`Bs`, expect 0) | | |
| UIF before start (pulse, plan) | | |
| plan phase (`Bs`, expect 1..1) | | |
| ticks counted after one minute (expect 3.0e6 / 12.0e6) | | |
| result (pass / fail) | | |

The one-drive check: drive accepted 20 us pulses (yes/no), steps lost over 20,000 pulses: ______.

## 6. If it fails

If the CPU is over budget, first try making `port_pin_write` `static inline` (defined in `port/stm32/puldir_pin.h`, so the pulse tick inlines its one store) or building with LTO (`-flto`), and measure again.

If 200 kHz overruns or the CPU limit is still exceeded, record the worst handler duration, and raise `BENCH_PULSE_US` (10u is a 100 kHz tick). Nothing else changes. If 50 kHz fails, record the same numbers and stop: the design needs rethinking before anything is built on it. In either case, look first at what runs above or at the pulse tick's priority: any interrupt left at preemption 0 (including SysTick) adds its duration to the jitter.
