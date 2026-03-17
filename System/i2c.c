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

static void Soft_I2C_SCL_OutOD(void)
{
    I2C_PORT->CRL &= ~SCL_CRL_MASK;
    I2C_PORT->CRL |=  (GPIO_MODE_OUT_OD_50M << SCL_CRL_SHIFT);
}

void Soft_SDA_OutPut(void)
{
    I2C_PORT->CRL &= ~SDA_CRL_MASK;
    I2C_PORT->CRL |=  (GPIO_MODE_OUT_OD_50M << SDA_CRL_SHIFT);
}

void Soft_SDA_InPut(void)
{
    /* 输入上拉：先把 ODR 置 1 */
    I2C_PORT->BSRR = I2C_SDA_PIN_MASK;
    I2C_PORT->CRL &= ~SDA_CRL_MASK;
    I2C_PORT->CRL |=  (GPIO_MODE_INPUT_PU << SDA_CRL_SHIFT);
}

void Soft_I2C_Init(void)
{
    /* 开 GPIOB 时钟 */
    RCC->APB2ENR |= I2C_CLK;

    /* SCL 始终开漏输出 */
    Soft_I2C_SCL_OutOD();

    /* SDA 默认也开漏输出 */
    Soft_SDA_OutPut();

    /* 释放总线 */
    SDA_High();
    SCL_High();
}


//起始位的设置
void Soft_I2C_Start(void)
{
	Soft_SDA_OutPut();
	//先确保SDA在SCL为高的时候，SDA一定为高
	SDA_High();
	SCL_High();
	Delay_us(5);
	//只要在SCL为高的时候SDA拉低就行了
	SDA_Low();
	Delay_us(5);
	SCL_Low();
}

//终止位的设置
void Soft_I2C_Stop(void)
{
	Soft_SDA_OutPut();
	SDA_Low();
	Delay_us(1);
	SCL_High();
	Delay_us(5);
	SDA_High();
	Delay_us(5);
}

//主设备发送应答
void Soft_I2C_Ack(void)
{
	//为了确保数正确，加一个确保没有错误
	Soft_SDA_OutPut();
	SDA_Low();
	Delay_us(5);//建立时间
	SCL_High();
	Delay_us(5);
	SCL_Low();
	Delay_us(5);//可以不加，反正只要是低电平就行
	SDA_High();
}

//主设备发送非应答
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

//接收应答位确定数据正确与否
uint8_t Soft_I2C_WaitAck(void)
{
	uint16_t count = 0;
	
	SDA_High();//释放SDA，让从机控制
	Soft_SDA_InPut();//切输入模式
	Delay_us(1);
	SCL_High();
	Delay_us(5);
	
	while(SDA_Read())
	{
		count++;
		if(count>100){
			SCL_Low();
			//超时也要开启输出模式
			Soft_SDA_OutPut();
			return 1;
		}
	}
	SCL_Low();
	Soft_SDA_OutPut();
	Delay_us(5);
	
	return 0;
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
	//主动拉高SDA，这样从机才能控制SDA进行应答
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
		if(SDA_Read()) data |= 0x01;
		SCL_Low();
		Delay_us(5);
	}		
	Soft_SDA_OutPut();
	
	return data;
}

//此处精彩的地方就在于由于主设备要确定写入的数据要到正确的设备
//七位地址左移一位后，bit0始终为0，始终符合写入地址的操作

uint8_t Soft_I2C_SendByte_Point(uint8_t slave_addr, uint8_t reg, uint8_t data)
{
	//
	Soft_I2C_Start();
	
	Soft_I2C_SendByte(slave_addr << 1);
	if(Soft_I2C_WaitAck())
	{
		Soft_I2C_Stop();
		return 0;
	}
	
	Soft_I2C_SendByte(reg);
	if(Soft_I2C_WaitAck())
	{
		Soft_I2C_Stop();
		return 0;
	}
	
	Soft_I2C_SendByte(data);
	if(Soft_I2C_WaitAck())
	{
		Soft_I2C_Stop();
		return 0;
	}
	Soft_I2C_Stop();
	
	return 1;
	
}
//*data的存在是为了避免读取到的数据也是0导致错判
uint8_t Soft_I2C_ReadByte_Point(uint8_t slave_addr, uint8_t reg, uint8_t *data)
{
	//空指针判断
	if (data == 0) return 0;
	Soft_I2C_Start();
	
	Soft_I2C_SendByte(slave_addr << 1);
	if(Soft_I2C_WaitAck())
	{
		Soft_I2C_Stop();
		return 0;
	}
	
	Soft_I2C_SendByte(reg);
	if(Soft_I2C_WaitAck())
	{
		Soft_I2C_Stop();
		return 0;
	}
	//转为读模式
	Soft_I2C_Start();
	Soft_I2C_SendByte((slave_addr<<1) | 0x01);
	if(Soft_I2C_WaitAck())
	{
		Soft_I2C_Stop();
		return 0;
	}
	
	*data = Soft_I2C_ReadByte();//在调用函数的时候自动修改好了
	Soft_I2C_NoAck();
	Soft_I2C_Stop();
	
	return 1;
}

uint8_t Soft_I2C_Send(uint8_t slave_addr, uint8_t reg, uint8_t *buf, uint8_t len)
{
	if ((buf == 0) && (len != 0)) return 0;
	Soft_I2C_Start();
	
	
	Soft_I2C_SendByte(slave_addr << 1);
	if(Soft_I2C_WaitAck())
	{
		Soft_I2C_Stop();
		return 0;
	}
	
	Soft_I2C_SendByte(reg);
	if(Soft_I2C_WaitAck())
	{
		Soft_I2C_Stop();
		return 0;
	}	
	
	for(uint8_t i = 0; i < len; i++)
	{
		Soft_I2C_SendByte(buf[i]);
		
		if(Soft_I2C_WaitAck())
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
	if(Soft_I2C_WaitAck())
	{
		Soft_I2C_Stop();
		return 0;
	}
	
	Soft_I2C_SendByte(reg);
	if(Soft_I2C_WaitAck())
	{
		Soft_I2C_Stop();
		return 0;
	}
	Soft_I2C_Start();
	Soft_I2C_SendByte((slave_addr<<1) | 0x01);
	if(Soft_I2C_WaitAck())
	{
		Soft_I2C_Stop();
		return 0;
	}
	
	for(uint8_t i = 0; i < len; i++)
	{
		buf[i] = Soft_I2C_ReadByte();
		if (i == len-1) Soft_I2C_NoAck();
		else Soft_I2C_Ack();
	}
	
	Soft_I2C_Stop();
	
	return 1;
}

