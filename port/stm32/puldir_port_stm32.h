/* The STM32 port: an application-owned pulse timer, a plan IRQ the pulse tick pends (E38), raw
 * IRQ entries, a PRIMASK lock (E39). Compiles for any family (main.h brings in its HAL) and names
 * no timer. */
#ifndef PULDIR_PORT_STM32_H
#define PULDIR_PORT_STM32_H
#include <stdbool.h>
#include <stdint.h>
#include "main.h"
#ifdef __cplusplus
extern "C" {
#endif
/* The application sets the pulse timer's prescaler for a 1 MHz count and enables its update
 * interrupt; port_start writes its period from pulse_us. The plan IRQ is any NVIC peripheral vector
 * (IRQn >= 0) the application does not otherwise use (a timer's, that timer never started, will
 * do), set to a preemption
 * priority below the pulse timer's before PulDir_start, which otherwise calls Error_Handler;
 * port_start enables it, and the pulse tick pends it once every plan_us. */
/* First, before PulDir_init. Each refuses a period out of range (pulse 2 to 1000 us, plan 250 to
 * 10000 us) or a call after the timing is fixed, attachPlanIrq also a system vector (IRQn < 0),
 * and then keeps nothing. */
bool PulDir_stm32_attachPulseTimer(TIM_HandleTypeDef* htim, IRQn_Type irqn, uint32_t pulse_us);
bool PulDir_stm32_attachPlanIrq(IRQn_Type irqn, uint32_t plan_us);
/* Call from the raw IRQ handler. Each returns at once unless its own source is due (the timer's
 * update flag, the pulse tick's pend), so a vector shared with another source is safe. */
void PulDir_stm32_pulseIrq(void);
void PulDir_stm32_planIrq(void);
#ifdef __cplusplus
}
#endif
#endif
