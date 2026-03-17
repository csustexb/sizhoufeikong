#include "stm32f10x.h"
#include "delay.h"
#include "nrf24l01.h"

int main(void)
{
		NRF24_GPIO_SPI_Init();
	uint8_t buf[]={1,2,3,4,5,6,7,8};
	while(1)
	{
		NRF24_ClearIRQFlags();
		Delay_ms(10);
	}
}