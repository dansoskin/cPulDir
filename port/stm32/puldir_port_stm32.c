#include "puldir_port.h"
#include "puldir_port_stm32.h"

static TIM_HandleTypeDef* pulse_tim;
static IRQn_Type pulse_irq, plan_irq;
static bool plan_attached;              /* zero is a real IRQ number */
static uint32_t saved_primask;          /* the core never nests the lock (the fake port asserts it) */
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

/* The plan period goes to the library only: the pulse tick pends this IRQ every plan_us (E38).
 * A system vector (IRQn < 0) is refused: CMSIS pends and enables only NVIC peripheral vectors, so
 * the plan tick would never run (E41). */
bool PulDir_stm32_attachPlanIrq(IRQn_Type irqn, uint32_t plan_us)
{
    if ((int32_t)irqn < 0 || !puldir_timing_plan(plan_us))
        return false;
    plan_irq = irqn;
    plan_attached = true;
    return true;
}

static uint32_t preemption(IRQn_Type irqn)
{
    uint32_t pre, sub;
    NVIC_DecodePriority(NVIC_GetPriority(irqn), NVIC_GetPriorityGrouping(), &pre, &sub);
    return pre;
}

/* The pulse period, the counter left at its last count so the first update is one count after start.
 * A plan IRQ not below the pulse IRQ (a vector left at reset priority 0) would hold pulse ticks off
 * for the whole plan tick, so it is refused. */
void port_start(void (*pulse)(bool), void (*plan)(void))
{
    if (pulse_tim == NULL || !plan_attached || preemption(plan_irq) <= preemption(pulse_irq))
        Error_Handler();                        /* attach both, the plan IRQ below the pulse IRQ */
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
    /* Cleared before the run: a pend after this store runs the plan tick again once this run returns;
     * one between the load above and this store merges with this run, as two pends do in the NVIC. */
    plan_due = false;
    plan_fn();
}

/* Every interrupt, for the few stores of a locked section: masked in the NVIC alone, the lock let
 * any other interrupt that preempted main inside it hold the pulse tick off for its whole run (E39).
 * Main and the plan tick never hold it at once, so one saved PRIMASK serves both. */
void port_lock(void)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    saved_primask = primask;
}

void port_unlock(void)
{
    __set_PRIMASK(saved_primask);
}

/* BSRR's upper half resets: one store, and it exists on every family (F4 has no BRR). */
void port_pin_write(puldir_pin_t p, bool level)
{
    p.port->BSRR = level ? (uint32_t)p.pin : (uint32_t)p.pin << 16;
}

bool port_pin_read(puldir_pin_t p) { return (p.port->IDR & p.pin) != 0u; }
