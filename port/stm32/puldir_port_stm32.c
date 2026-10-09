#include "puldir_port.h"
#include "puldir_port_stm32.h"

static TIM_HandleTypeDef* pulse_tim;
static TIM_HandleTypeDef* plan_tim;
static IRQn_Type pulse_irq, plan_irq;
static uint32_t pulse_counts, plan_counts;     /* the periods, in 1 us counts */
/* Until port_start, a shared vector seeing a stale update flag on an attached timer calls these. */
static void no_pulse(bool overrun) { (void)overrun; }
static void no_plan(void) {}
static void (*pulse_fn)(bool overrun) = no_pulse;
static void (*plan_fn)(void) = no_plan;
static bool late;                       /* the update flag was set again before the last exit */

bool PulDir_stm32_attachPulseTimer(TIM_HandleTypeDef* htim, IRQn_Type irqn, uint32_t pulse_us)
{
    if (!puldir_timing_pulse(pulse_us))
        return false;
    pulse_tim = htim;
    pulse_irq = irqn;
    pulse_counts = pulse_us;
    return true;
}

bool PulDir_stm32_attachPlanTimer(TIM_HandleTypeDef* htim, IRQn_Type irqn, uint32_t plan_us)
{
    if (!puldir_timing_plan(plan_us))
        return false;
    plan_tim = htim;
    plan_irq = irqn;
    plan_counts = plan_us;
    return true;
}

/* A period of `counts` 1 us counts, the counter left at its last count so the first update is one count away. */
static void set_period(TIM_HandleTypeDef* htim, uint32_t counts)
{
    __HAL_TIM_SET_AUTORELOAD(htim, counts - 1u);    /* the counter runs 0..ARR: 5 counts is ARR 4 */
    htim->Instance->EGR = TIM_EGR_UG;               /* a preloaded ARR takes effect now */
    htim->Instance->SR = ~TIM_SR_UIF;
    __HAL_TIM_SET_COUNTER(htim, counts - 1u);
}

/* Both first updates come one count after start, the pulse timer's first: each plan tick (N pulse
 * periods, N = ticks_per_plan) then follows a latch tick (1, N+1, ...) by the start skew, under one
 * pulse period. */
void port_start(void (*pulse)(bool), void (*plan)(void))
{
    if (pulse_tim == NULL || plan_tim == NULL)
        Error_Handler();                        /* attach both timers before PulDir_start */
    pulse_fn = pulse;
    plan_fn = plan;
    set_period(pulse_tim, pulse_counts);
    set_period(plan_tim, plan_counts);
    HAL_TIM_Base_Start_IT(pulse_tim);
    HAL_TIM_Base_Start_IT(plan_tim);
}

/* Not through HAL_TIM_IRQHandler: its flag sweep costs more than the tick at a 5 us pulse. */
void PulDir_stm32_pulseIrq(void)
{
    TIM_TypeDef* t = pulse_tim->Instance;
    if ((t->SR & TIM_SR_UIF) == 0u)
        return;                                     /* the vector's other source */
    t->SR = ~TIM_SR_UIF;                            /* rc_w0: clears only UIF */
    pulse_fn(late);
    late = (t->SR & TIM_SR_UIF) != 0u;
}

void PulDir_stm32_planIrq(void)
{
    TIM_TypeDef* t = plan_tim->Instance;
    if ((t->SR & TIM_SR_UIF) == 0u)
        return;
    t->SR = ~TIM_SR_UIF;
    plan_fn();
}

/* Until both timers are attached the IRQ numbers are not known (zero is a real IRQ), so do nothing. */
void port_lock(void)
{
    if (pulse_tim == NULL || plan_tim == NULL)
        return;
    NVIC_DisableIRQ(pulse_irq);
    NVIC_DisableIRQ(plan_irq);
    __DSB();
    __ISB();
}

void port_unlock(void)
{
    if (pulse_tim == NULL || plan_tim == NULL)
        return;
    NVIC_EnableIRQ(plan_irq);
    NVIC_EnableIRQ(pulse_irq);
}

/* BSRR's upper half resets: one store, and it exists on every family (F4 has no BRR). */
void port_pin_write(puldir_pin_t p, bool level)
{
    p.port->BSRR = level ? (uint32_t)p.pin : (uint32_t)p.pin << 16;
}

bool port_pin_read(puldir_pin_t p) { return (p.port->IDR & p.pin) != 0u; }
