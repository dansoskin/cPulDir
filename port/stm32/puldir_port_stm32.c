#include "puldir_port.h"
#include "puldir_port_stm32.h"

static TIM_HandleTypeDef* pulse_tim;
static IRQn_Type pulse_irq, plan_irq;
static bool plan_attached;              /* zero is a real IRQ number */
static uint32_t pulse_counts;           /* the pulse period, in 1 us counts */
/* Until port_start, a shared vector seeing a stale update flag on the pulse timer calls this. */
static void no_pulse(bool overrun) { (void)overrun; }
static void (*pulse_fn)(bool overrun) = no_pulse;
static void (*plan_fn)(void);
static bool late;                       /* the update flag was set again before the last exit */
static volatile bool plan_due;          /* port_plan_pend ran: the plan IRQ's own source */

bool PulDir_stm32_attachPulseTimer(TIM_HandleTypeDef* htim, IRQn_Type irqn, uint32_t pulse_us)
{
    if (!puldir_timing_pulse(pulse_us))
        return false;
    pulse_tim = htim;
    pulse_irq = irqn;
    pulse_counts = pulse_us;
    return true;
}

/* The plan period goes to the library only: the pulse tick pends this IRQ every plan_us (E38). */
bool PulDir_stm32_attachPlanIrq(IRQn_Type irqn, uint32_t plan_us)
{
    if (!puldir_timing_plan(plan_us))
        return false;
    plan_irq = irqn;
    plan_attached = true;
    return true;
}

/* The pulse period, the counter left at its last count so the first update is one count after start. */
void port_start(void (*pulse)(bool), void (*plan)(void))
{
    if (pulse_tim == NULL || !plan_attached)
        Error_Handler();                        /* attach the pulse timer and the plan IRQ before PulDir_start */
    pulse_fn = pulse;
    plan_fn = plan;
    TIM_HandleTypeDef* h = pulse_tim;
    __HAL_TIM_SET_AUTORELOAD(h, pulse_counts - 1u);  /* the counter runs 0..ARR: 5 counts is ARR 4 */
    h->Instance->EGR = TIM_EGR_UG;                  /* a preloaded ARR takes effect now */
    h->Instance->SR = ~TIM_SR_UIF;
    __HAL_TIM_SET_COUNTER(h, pulse_counts - 1u);
    NVIC_ClearPendingIRQ(plan_irq);                 /* no plan tick before the first pend */
    NVIC_EnableIRQ(plan_irq);
    HAL_TIM_Base_Start_IT(h);
}

/* From the pulse tick: the plan IRQ, at a lower priority, runs as soon as the pulse tick returns. */
void port_plan_pend(void)
{
    plan_due = true;
    NVIC_SetPendingIRQ(plan_irq);
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
    if (!plan_due)
        return;                                     /* the vector's other source */
    plan_due = false;                               /* first: a pend during the run runs it again */
    plan_fn();
}

/* Until both are attached the IRQ numbers are not known (zero is a real IRQ), so do nothing. */
void port_lock(void)
{
    if (pulse_tim == NULL || !plan_attached)
        return;
    NVIC_DisableIRQ(pulse_irq);
    NVIC_DisableIRQ(plan_irq);
    __DSB();
    __ISB();
}

void port_unlock(void)
{
    if (pulse_tim == NULL || !plan_attached)
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
