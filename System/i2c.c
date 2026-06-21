#include "i2c.h"
#include "delay.h"
/*
PB6 -> SCL
PB7 -> SDA

STM32F1 GPIO mode bits:
输入上拉/下拉 : 0x8
开漏输出50MHz : 0x7
*/

#define GPIO_MODE_INPUT_PU   0x8U
#define GPIO_MODE_OUT_OD_50M 0x7U

/* PB6 对应 CRL[27:24], PB7 对应 CRL[31:28] */
#define SCL_CRL_SHIFT   (I2C_SCL_PIN * 4U)
#define SDA_CRL_SHIFT   (I2C_SDA_PIN * 4U)

#define SCL_CRL_MASK    (0xFU << SCL_CRL_SHIFT)
#define SDA_CRL_MASK    (0xFU << SDA_CRL_SHIFT)
/**
  * @brief  Configure SCL as open-drain output
  * @param  无
  * @retval 无
  */
static void Soft_I2C_SCL_OutOD(void)
{
    I2C_PORT->CRL &= ~SCL_CRL_MASK;
    I2C_PORT->CRL |=  (GPIO_MODE_OUT_OD_50M << SCL_CRL_SHIFT);
}
/**
  * @brief  Configure SDA as output
  * @param  无
  * @retval 无
  */
void Soft_SDA_OutPut(void)
{
    I2C_PORT->CRL &= ~SDA_CRL_MASK;
    I2C_PORT->CRL |=  (GPIO_MODE_OUT_OD_50M << SDA_CRL_SHIFT);
}

/**
  * @brief  Configure SDA as input
  * @param  无
  * @retval 无
  */
void Soft_SDA_InPut(void)
{
    /* Input pull-up: set ODR high first */
    I2C_PORT->BSRR = I2C_SDA_PIN_MASK;
    I2C_PORT->CRL &= ~SDA_CRL_MASK;
    I2C_PORT->CRL |=  (GPIO_MODE_INPUT_PU << SDA_CRL_SHIFT);
}
/**
  * @brief  SDA initialization
  * @param  无
  * @retval 无
  */
void Soft_I2C_Init(void)
{
    /* Enable GPIOB clock */
    RCC->APB2ENR |= I2C_CLK;

    /* SCL always open-drain output */
    Soft_I2C_SCL_OutOD();

    /* SDA defaults to open-drain output */
    Soft_SDA_OutPut();

    /* Release bus */
    SDA_High();
    SCL_High();
}

/**
  * @brief  I2C start condition
  * @param  无
  * @retval 无
  * @note  I2C 起始位的时序要求：SCL 为高电平时，SDA 从高拉低
  */
void Soft_I2C_Start(void)
{
	// Prevent bus contention: pull SDA high first
	Soft_SDA_OutPut();
	//Ensure SDA is high while SCL is high
	SDA_High();
	SCL_High();
	Delay_us(5);
	//Pull SDA low while SCL is high
	SDA_Low();
	Delay_us(5);
	SCL_Low();
}

/**
  * @brief  I2C stop condition
  * @param  无
  * @retval 无
  * @note  I2C 终止位的时序要求：SCL 为高电平时，SDA 从低拉高
  */
void Soft_I2C_Stop(void)
{
	// Prevent bus contention: pull SDA high first
	Soft_SDA_OutPut();
	//Ensure SDA is low while SCL is high
	SDA_Low();
	Delay_us(1);
	SCL_High();
	Delay_us(5);
	SDA_High();
	Delay_us(5);
}
/**
  * @brief  Master sends ACK
  * @param  无
  * @retval 无
  * @note  I2C Master sends ACK的时序要求：第九个周期，SDA为低电平
  */
//Master sends ACK
void Soft_I2C_Ack(void)
{
	//Ensure correct data with robust timing
	Soft_SDA_OutPut();
	SDA_Low();
	Delay_us(5);//Setup time
	SCL_High();
	Delay_us(5);
	SCL_Low();
	Delay_us(5);//Optional: low level is sufficient
	SDA_High();
}
/**
  * @brief  Master sends NACK
  * @param  无
  * @retval 无
  * @note  I2C Master sends NACK的时序要求：第九个周期，SDA为高电平
  */
//Master sends NACK
void Soft_I2C_NoAck(void)
{
	Soft_SDA_OutPut();
	SDA_High();
	Delay_us(1);
	SCL_High();
	Delay_us(5);
	SCL_Low();
	Delay_us(5);

}
/**
  * @brief  Master waits for ACK
  * @param  无
  * @retval 无
  * @note  I2C Master waits for ACK的时序要求：第九个周期，主机读取的SDA为高电平
  */
//Check ACK bit to verify reception
uint8_t Soft_I2C_WaitAck(void)
{
    uint16_t timeout = 0;

    SDA_High();                // Release SDA for slave control
    Soft_SDA_InPut();          // Switch to input
    Delay_us(5);               // SDASetup time，可与下文对称
    SCL_High();                // 9th clock high
    Delay_us(5);

    while (SDA_Read())
    {
        Delay_us(1);           // 1us delay per poll, prevent fast timeout
        if (++timeout > 200)    // 200us timeout, sufficient for slow devices
        {
            SCL_Low();
            Soft_SDA_OutPut();
            return 1;          // Timeout: no ACK
        }
    }
    SCL_Low();
    Soft_SDA_OutPut();
    Delay_us(5);               // Hold SCL low to let slave release SDA
    return 0;                  // ACK received
}

void Soft_I2C_SendByte(uint8_t data)
{
	Soft_SDA_OutPut();
	for (uint8_t i = 8; i > 0; i--){
		if (data & 0x80) SDA_High();
		else SDA_Low();
		data <<= 1;

		Delay_us(5);
		SCL_High();
		Delay_us(5);
		SCL_Low();
	}
	//Pull SDA high for slave ACK control
	SDA_High();

}

uint8_t Soft_I2C_ReadByte(void)
{
	uint8_t data =0;
	Soft_SDA_InPut();
	SDA_High();
	for (int i = 0; i < 8; i++){
		data <<= 1;
		SCL_High();

		Delay_us(5);
		if (SDA_Read()) data |= 0x01;
		SCL_Low();
		Delay_us(5);
	}
	Soft_SDA_OutPut();

	return data;
}

//The key: 7-bit addr << 1 has bit0=0, matching write
//7-bit addr << 1: bit0 always 0 for write

uint8_t Soft_I2C_SendByte_Point(uint8_t slave_addr, uint8_t reg, uint8_t data)
{
	//
	Soft_I2C_Start();

	Soft_I2C_SendByte(slave_addr << 1);
	if (Soft_I2C_WaitAck())
	{
		Soft_I2C_Stop();
		return 0;
	}

	Soft_I2C_SendByte(reg);
	if (Soft_I2C_WaitAck())
	{
		Soft_I2C_Stop();
		return 0;
	}

	Soft_I2C_SendByte(data);
	if (Soft_I2C_WaitAck())
	{
		Soft_I2C_Stop();
		return 0;
	}
	Soft_I2C_Stop();

	return 1;

}
//*data avoids ambiguity when read value is 0
uint8_t Soft_I2C_ReadByte_Point(uint8_t slave_addr, uint8_t reg, uint8_t *data)
{
	//Null check
	if (data == 0) return 0;
	Soft_I2C_Start();

	Soft_I2C_SendByte(slave_addr << 1);
	if (Soft_I2C_WaitAck())
	{
		Soft_I2C_Stop();
		return 0;
	}

	Soft_I2C_SendByte(reg);
	if (Soft_I2C_WaitAck())
	{
		Soft_I2C_Stop();
		return 0;
	}
	//Switch to read mode
	Soft_I2C_Start();
	Soft_I2C_SendByte((slave_addr<<1) | 0x01);
	if (Soft_I2C_WaitAck())
	{
		Soft_I2C_Stop();
		return 0;
	}

	*data = Soft_I2C_ReadByte();//Value set via pointer
	Soft_I2C_NoAck();
	Soft_I2C_Stop();

	return 1;
}

uint8_t Soft_I2C_Send(uint8_t slave_addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
	if ((buf == 0) && (len != 0)) return 0;
	Soft_I2C_Start();


	Soft_I2C_SendByte(slave_addr << 1);
	if (Soft_I2C_WaitAck())
	{
		Soft_I2C_Stop();
		return 0;
	}

	Soft_I2C_SendByte(reg);
	if (Soft_I2C_WaitAck())
	{
		Soft_I2C_Stop();
		return 0;
	}

	or (uint8_t i = 0; i < len; i++)
	{
		Soft_I2C_SendByte(buf[i]);

		if (Soft_I2C_WaitAck())
		{
			Soft_I2C_Stop();
			return 0;
		}

	}

	Soft_I2C_Stop();

	return 1;
}

uint8_t Soft_I2C_Read(uint8_t slave_addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
	if ((buf == 0) && (len != 0)) return 0;
	Soft_I2C_Start();

	Soft_I2C_SendByte(slave_addr << 1);
	if (Soft_I2C_WaitAck())
	{
		Soft_I2C_Stop();
		return 0;
	}

	Soft_I2C_SendByte(reg);
	if (Soft_I2C_WaitAck())
	{
		Soft_I2C_Stop();
		return 0;
	}
	Soft_I2C_Start();
	Soft_I2C_SendByte((slave_addr<<1) | 0x01);
	if (Soft_I2C_WaitAck())
	{
		Soft_I2C_Stop();
		return 0;
	}

	or (uint8_t i = 0; i < len; i++)
	{
		buf[i] = Soft_I2C_ReadByte();
		if (i == len-1) Soft_I2C_NoAck();
		else Soft_I2C_Ack();
	}

	Soft_I2C_Stop();

	return 1;
}

