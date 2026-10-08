/* The STM32 port: two application-owned timers, raw IRQ entries, NVIC lock.
 * Compiles for any family (main.h brings in its HAL) and names no timer. */
#ifndef PULDIR_PORT_STM32_H
#define PULDIR_PORT_STM32_H
#include <stdbool.h>
#include <stdint.h>
#include "main.h"
#ifdef __cplusplus
extern "C" {
#endif
/* The application sets each prescaler for a 1 MHz count, enables the update interrupt and gives the
 * pulse IRQ a higher priority than the plan IRQ. port_start writes both periods from these arguments. */
/* First, before PulDir_init. Each refuses a period out of range (pulse 2 to 1000 us, plan 250 to
 * 10000 us) or a call after the timing is fixed, and then keeps nothing. Until both are attached,
 * port_lock/port_unlock do nothing. */
bool PulDir_stm32_attachPulseTimer(TIM_HandleTypeDef* htim, IRQn_Type irqn, uint32_t pulse_us);
bool PulDir_stm32_attachPlanTimer(TIM_HandleTypeDef* htim, IRQn_Type irqn, uint32_t plan_us);
/* Call from the raw IRQ handler. Each returns at once if its timer's update flag is clear,
 * so a vector shared with another source (TIM1_UP_TIM16 and the like) is safe. */
void PulDir_stm32_pulseIrq(void);
void PulDir_stm32_planIrq(void);
#ifdef __cplusplus
}
#endif
#endif
