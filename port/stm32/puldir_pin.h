#ifndef PULDIR_PIN_H
#define PULDIR_PIN_H
#include "main.h"                               /* the CubeMX project's family HAL */
typedef struct { GPIO_TypeDef* port; uint16_t pin; } puldir_pin_t;
#endif
