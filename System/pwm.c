#include "pwm.h"

/*

PWM1->PA2 TIM2_CH3
PWM2->PA3 TIM2_CH4
PWM3->PB8 TIM4_CH3
PWM4->PB9 TIM4_CH4

*/

void PWM_Init()
{
	// Define GPIO, TIM, PWM structs
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	//Enable timer and GPIO clocks
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM4, ENABLE);

	//Configure GPIO AF_PP for PWM output
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//Configure TIM2 and TIM4
	TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 20000 - 1;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 20000 - 1;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	//Configure PWM channels
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	//Initialize PWM channels
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);

	//Enable preload registers
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM2, ENABLE);
	TIM_ARRPreloadConfig(TIM4, ENABLE);

	//Enable timers
	TIM_Cmd(TIM2, ENABLE);
	TIM_Cmd(TIM4, ENABLE);

}

/* S5: Hard PWM limits — hardware-level motor safety clamp */
#define PWM_MIN_US  1000
#define PWM_MAX_US  2000

static uint16_t PWM_Clamp(uint16_t val)
{
	if (val < PWM_MIN_US) return PWM_MIN_US;
	if (val > PWM_MAX_US) return PWM_MAX_US;
	return val;
}

/* Set channel CCR value (with hard limit) */

void PWM_SetCompare1(uint16_t compare)
{
	TIM_SetCompare3(TIM2, PWM_Clamp(compare));
}

void PWM_SetCompare2(uint16_t compare)
{
	TIM_SetCompare4(TIM2, PWM_Clamp(compare));
}

void PWM_SetCompare3(uint16_t compare)
{
	TIM_SetCompare3(TIM4, PWM_Clamp(compare));
}

void PWM_SetCompare4(uint16_t compare)
{
	TIM_SetCompare4(TIM4, PWM_Clamp(compare));
}
