#ifndef __LED_H
#define __LED_H

#include "stm32f10x.h"                  // Device header


#define MCU_LED_GPIO_PORT GPIOA
#define MCU_LED_GPIO_PIN GPIO_Pin_12
#define MCU_LED_GPIO_CLK RCC_APB2Periph_GPIOA

void LED_Init(void);
void LED_ON(void);


#endif