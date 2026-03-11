#include "led.h"

void LED_Init(void)
{
	//定义结构体
	GPIO_InitTypeDef GPIO_InitStructure;
	//开启GPIOA的时钟
	RCC_APB2PeriphClockCmd(MCU_LED_GPIO_CLK,ENABLE);
	//配置具体引脚，模式，工作频率
	GPIO_InitStructure.GPIO_Pin = MCU_LED_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	//初始化GPIO
	GPIO_Init(MCU_LED_GPIO_PORT, &GPIO_InitStructure);
	
}
