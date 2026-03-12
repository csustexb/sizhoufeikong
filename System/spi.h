#ifndef __SPI_H
#define __SPI_H

#include "stm32f10x.h"                  // Device header

/*
对于SPI协议，由于硬件有着稳定高效的功能，且可支持的频率非常高
因此我们不需要自己写软件SPI协议
而I2C由于硬件的问题，需要自己重写软件I2C才能避免问题
*/
#define SPI_CLK RCC_APB2Periph_SPI1
#define SPI_GPIO_CLK RCC_APB2Periph_GPIOA
#define SPI_GPIO_PORT GPIOA
#define SPIx SPI1
#define SPI_CLK_PIN GPIO_Pin_5
#define SPI_MISO_PIN GPIO_Pin_6
#define SPI_MOSI_PIN GPIO_Pin_7
#define SPI_CS_PORT GPIOA
#define SPI_CS_PIN GPIO_Pin_4
#define SPI_CS_High() GPIO_SetBits(SPI_CS_PORT, SPI_CS_PIN)
#define SPI_CS_Low() GPIO_ResetBits(SPI_CS_PORT, SPI_CS_PIN)

//配置SPI的控制参数,极性和相位(CPOL and CPHA)
typedef struct{
	uint8_t CPOL;
	uint8_t CPHA;
	uint8_t FirstBit;//决定是从高位还是低位开始发送
	uint8_t BaudRatePrescaler;//波特率分频
} SPI_ConfigTypedef;

void SPI_ConfigInit(SPI_ConfigTypedef *spiconfig);//初始化，同时配置控制参数
//由于SPI协议规定的，在一个采样沿，主机不仅要发送数据，同时还要接受数据
//所以可以把函数封装成如下
//txdata是要发送的数据，本函数返回的值是接收到的数据
uint8_t SPI_ReadWriteByte(uint16_t txdata);
//由于多字节无法通过函数返回，我们选择在参数里加入需要保存的数组
void SPI_ReadWriteMulti(uint8_t *txbuf, uint8_t *rxbuf, uint16_t len);
//由多字节读写的函数来封装多字节读和多字节写
void SPI_ReadMulti(uint8_t *rxbuf, uint16_t len);
void SPI_WriteMulti(uint8_t *txbuf, uint16_t len);

#endif