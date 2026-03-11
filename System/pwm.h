#ifndef __PWM_H
#define __PWM_H

#include "stm32f10x.h"                  // Device header

/*

PWM1->PA2 TIM2_CH3
PWM2->PA3 TIM2_CH4
PWM3->PB8 TIM4_CH3
PWM4->PB9 TIM4_CH4

*/

void PWM_Init(void);
void PWM_SerCompare1(uint16_t compare);
void PWM_SerCompare2(uint16_t compare);
void PWM_SerCompare3(uint16_t compare);
void PWM_SerCompare4(uint16_t compare);

#endif