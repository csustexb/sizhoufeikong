#include "delay.h"

/*
由于while(num--)每次执行的指令不准确，故用此方法来定时都无法精确地设置时间
我们选择使用硬件的定时计数来写延时
由于主频72MHz被分频为1MHz，定时器每隔1us就会自动计数
同时由于此时定时器计数无法得知，且是否会重装初始值都无法确定
我们使用如下方式来进行避免
*/

void Delay_us(uint32_t num)
{
	uint16_t start = TIM->CNT;
	while((TIM->CNT - start) < num );
}

void Delay_ms(uint32_t num)
{
	for(uint32_t i =0; i < num; i++) Delay_us(1000);
}