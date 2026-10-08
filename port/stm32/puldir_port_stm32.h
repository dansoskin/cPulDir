/* The STM32 port: two application-owned timers, raw IRQ entries, NVIC lock.
 * Compiles for any family (main.h brings in its HAL) and names no timer. */
#ifndef PULDIR_PORT_STM32_H
#define PULDIR_PORT_STM32_H
#include "main.h"
#ifdef __cplusplus
extern "C" {
#endif
/* The application sets each prescaler for a 1 MHz count, enables the update interrupt and gives the
 * pulse IRQ a higher priority than the plan IRQ. port_start writes both periods (PULDIR_*_COUNTS). */
/* Once each, before PulDir_start. Until both are attached, port_lock/port_unlock do nothing. */
void PulDir_stm32_attachPulseTimer(TIM_HandleTypeDef* htim, IRQn_Type irqn);
void PulDir_stm32_attachPlanTimer(TIM_HandleTypeDef* htim, IRQn_Type irqn);
/* Call from the raw IRQ handler. Each returns at once if its timer's update flag is clear,
 * so a vector shared with another source (TIM1_UP_TIM16 and the like) is safe. */
void PulDir_stm32_pulseIrq(void);
void PulDir_stm32_planIrq(void);
#ifdef __cplusplus
}
#endif
#endif
