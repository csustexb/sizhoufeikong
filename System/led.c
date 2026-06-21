#include "led.h"

void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(MCU_LED_GPIO_CLK, ENABLE);

	GPIO_InitStructure.GPIO_Pin   = MCU_LED_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MCU_LED_GPIO_PORT, &GPIO_InitStructure);

	/* Start with LED off */
	LED_OFF();
}

void LED_ON(void)
{
	GPIO_SetBits(MCU_LED_GPIO_PORT, MCU_LED_GPIO_PIN);
}

void LED_OFF(void)
{
	GPIO_ResetBits(MCU_LED_GPIO_PORT, MCU_LED_GPIO_PIN);
}
