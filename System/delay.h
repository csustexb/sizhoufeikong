#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f10x.h"                  // Device header

#define TIM TIM1

//由于主频72MHz被分频为1MHz，每个机械周期则被分为1us
void Delay_us(uint32_t num);
//void Delay_ms(uint32_t num);

#endif