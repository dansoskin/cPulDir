/* The STM32 port: application-owned timers, raw IRQ entries, NVIC lock.
 * Compiles for any family: main.h brings in that family's HAL. */
#ifndef PULDIR_PORT_STM32_H
#define PULDIR_PORT_STM32_H
#include "main.h"
#ifdef __cplusplus
extern "C" {
#endif
/* Once, before PulDir_start. Until it is called, port_lock/port_unlock do nothing. */
void PulDir_stm32_bind(TIM_HandleTypeDef* pulse_htim, IRQn_Type pulse_irqn,
                       TIM_HandleTypeDef* plan_htim,  IRQn_Type plan_irqn);
void PulDir_stm32_pulseIrq(void);   /* call from the raw TIMx_IRQHandler */
void PulDir_stm32_planIrq(void);
#ifdef __cplusplus
}
#endif
#endif
