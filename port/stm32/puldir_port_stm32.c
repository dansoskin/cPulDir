#include "puldir_port.h"
#include "puldir_port_stm32.h"

static TIM_HandleTypeDef* pulse_tim;
static TIM_HandleTypeDef* plan_tim;
static IRQn_Type pulse_irq, plan_irq;
static void (*pulse_fn)(bool overrun);
static void (*plan_fn)(void);
static bool late;                       /* the update flag was set again before the last exit */
static volatile bool bound;             /* PulDir_stm32_bind has run */

void PulDir_stm32_bind(TIM_HandleTypeDef* p, IRQn_Type pi, TIM_HandleTypeDef* q, IRQn_Type qi)
{
    pulse_tim = p; pulse_irq = pi;
    plan_tim = q;  plan_irq = qi;
    bound = true;
}

void port_start(void (*pulse)(bool), void (*plan)(void))
{
    pulse_fn = pulse;
    plan_fn = plan;
    /* The core latches on pulse ticks 1, N+1, ...: at ARR the pulse timer's first update is
     * immediate, so each plan tick (period N pulses) lands on a latch tick and runs right after it. */
    __HAL_TIM_SET_COUNTER(pulse_tim, __HAL_TIM_GET_AUTORELOAD(pulse_tim));
    __HAL_TIM_SET_COUNTER(plan_tim, 0);
    HAL_TIM_Base_Start_IT(pulse_tim);
    HAL_TIM_Base_Start_IT(plan_tim);
}

/* Not through HAL_TIM_IRQHandler: its flag sweep costs more than the tick at 200 kHz. */
void PulDir_stm32_pulseIrq(void)
{
    pulse_tim->Instance->SR = ~TIM_SR_UIF;          /* rc_w0: clears only UIF */
    pulse_fn(late);
    late = (pulse_tim->Instance->SR & TIM_SR_UIF) != 0u;
}

void PulDir_stm32_planIrq(void)
{
    plan_tim->Instance->SR = ~TIM_SR_UIF;
    plan_fn();
}

/* Before bind the IRQ numbers are not known (zero is a real IRQ), so do nothing. */
void port_lock(void)
{
    if (!bound)
        return;
    NVIC_DisableIRQ(pulse_irq);
    NVIC_DisableIRQ(plan_irq);
    __DSB();
    __ISB();
}

void port_unlock(void)
{
    if (!bound)
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

/* Pin modes are CubeMX's business; only park the output at its idle level. */
void port_pin_init_output(puldir_pin_t p, bool level) { port_pin_write(p, level); }
void port_pin_init_input(puldir_pin_t p, bool pullup) { (void)p; (void)pullup; }
