#include "spi.h"

/*
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
void SPI_ConfigInit(SPI_ConfigTypedef *spiconfig);
uint8_t SPI_ReadWriteByte(uint16_t txdata);
void SPI_ReadWriteMulti(uint8_t *txbuf, uint8_t *rxbuf, uint16_t len);
void SPI_ReadMulti(uint8_t *rxbuf, uint16_t len);
void SPI_WriteMulti(uint8_t *txbuf, uint16_t len);
*/

void SPI_ConfigInit(SPI_ConfigTypedef *spiconfig)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;
	
	RCC_APB2PeriphClockCmd(SPI_CLK | SPI_GPIO_CLK, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = SPI_MOSI_PIN | SPI_CLK_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//需要借助SPI外设高速控制
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//GPIO响应速率
	GPIO_Init(SPI_GPIO_PORT, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = SPI_MISO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(SPI_GPIO_PORT, &GPIO_InitStructure);
	
	//配置片选引脚，查看数据手册。空闲时需要取消片选
	GPIO_InitStructure.GPIO_Pin = SPI_CS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//主机控制
	SPI_CS_High();
	
	//SPI配置
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//全双工模式
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;//主机模式
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = spiconfig->CPOL;
	SPI_InitStructure.SPI_CPHA = spiconfig->CPHA;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;//软件管理片选
	SPI_InitStructure.SPI_BaudRatePrescaler = spiconfig->BaudRatePrescaler;
	SPI_InitStructure.SPI_FirstBit = spiconfig->FirstBit;
	SPI_InitStructure.SPI_CRCPolynomial = 7;//CRC多项式默认值
	SPI_Init(SPIx, &SPI_InitStructure);
	SPI_Cmd(SPIx, ENABLE);
}

//私有函数
static void SPI_WaitTxEmpty(void)
{
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET);//等待缓冲区变空
}

static void SPI_WaitRxEmpty(void)
{
	while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_RXNE) == RESET);
}

uint8_t SPI_ReadWriteByte(uint16_t txdata)
{
	SPI_WaitTxEmpty();
	SPI_I2S_SendData(SPIx, txdata);
	SPI_WaitRxEmpty();
	return SPI_I2S_ReceiveData(SPIx);
}

void SPI_ReadWriteMulti(uint8_t *txbuf, uint8_t *rxbuf, uint16_t len)
{
	uint16_t i =0;
	for(i = 0; i < len; i++)
	{
		rxbuf[i] = SPI_ReadWriteByte(txbuf[i]);
	}
}

void SPI_ReadMulti(uint8_t *rxbuf, uint16_t len)
{
	uint16_t i =0;
	for(i = 0; i < len; i++)
	{
		//由于读和写操作是同步操作的，所以想要接收数据，随便发数据就行
		rxbuf[i] = SPI_ReadWriteByte(0xFF);
	}
}

void SPI_WriteMulti(uint8_t *txbuf, uint16_t len)
{
	uint16_t i =0;
	for(i = 0; i < len; i++)
	{
		SPI_ReadWriteByte(txbuf[i]);
	}
}











