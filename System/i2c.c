#include "i2c.h"
#include "delay.h"
/*
#define I2C_PORT GPIOB
#define I2C_CLK RCC_APB2Periph_GPIOB
#define I2C_SCL_PIN GPIO_Pin_6
#define I2C_SDA_PIN GPIO_Pin_7


#define SDA_High() GPIO_SetBits(I2C_PORT, I2C_SDA_PIN)
#define SDA_Low() GPIO_ResetBits(I2C_PORT, I2C_SDA_PIN)
#define SCL_High() GPIO_SetBits(I2C_PORT, I2C_SCL_PIN)
#define SCL_Low() GPIO_ResetBits(I2C_PORT, I2C_SCL_PIN)
#define SDA_Read() ((I2C_PORT->IDR & I2C_SDA_PIN) ? 1 : 0)

void Soft_I2C_OutPut(void);
void Soft_I2C_InPut(void);
void Soft_I2C_Start(void);
void Soft_I2C_Stop(void);
void Soft_I2C_Ack(void);
void Soft_I2C_NoAck(void);
uint8_t Soft_I2C_WaitAck(void);
void Soft_I2C_SendByte(uint8_t data);
uint8_t Soft_I2C_ReadByte(void);
uint8_t Soft_I2C_SendByte_Point(uint8_t slave_addr, uint8_t reg, uint8_t data);//slave_add表示从设备地址
uint8_t Soft_I2C_ReadByte_Point(uint8_t slave_addr, uint8_t reg);
uint8_t Soft_2C_Send(uint8_t slave_addr, uint8_t reg, uint8_t *buf, uint8_t len);
uint8_t Soft_I2C_Read(uint8_t slave_addr, uint8_t reg, uint8_t *buf, uint8_t len);
*/

//主机输出模式
void Soft_I2C_OutPut(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(I2C_CLK, ENABLE); 
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Pin = I2C_SCL_PIN | I2C_SDA_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(I2C_PORT, &GPIO_InitStructure);
	
	//释放总线，将SDA和SCL拉至高电平
	SDA_High();
	SCL_High();
	
}

//主机读取模式
void Soft_I2C_InPut(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(I2C_CLK, ENABLE); 
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = I2C_SCL_PIN | I2C_SDA_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(I2C_PORT, &GPIO_InitStructure);
	
	SDA_High();
	SCL_High();
}

//起始位的设置
void Soft_I2C_Start(void)
{
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
	SDA_Low();
	SCL_High();
	Delay_us(5);
	SDA_High();
	Delay_us(5);
	SCL_Low();
}

//主设备发送应答
void Soft_I2C_Ack(void)
{
	SDA_Low();
	SCL_High();
	Delay_us(5);
	SCL_Low();
	Delay_us(5);//可以不加，反正只要是低电平就行
	SDA_High();
}

//主设备发送非应答
void Soft_I2V_NoAck(void)
{
	SDA_High();
	SCL_High();
	Delay_us(5);
	SCL_Low();
	Delay_us(5);
	SDA_Low();
}

//接收应答位确定数据正确与否
uint8_t Soft_I2C_WaitAck(void)
{
	uint16_t count = 0;
	
	SDA_High();//释放SDA，让从机控制
	Delay_us(1);
	SCL_High();
	Soft_I2C_InPut();
	
	while(SDA_Read())
	{
		count++;
		if(count>100){
			SCL_Low();
			return 1;
		}
	}
	SCL_Low();
	Delay_us(5);
	
	return 0;
}

void Soft_I2C_SendByte(uint8_t data)
{
	
	for (uint8_t i = 8; i > 0; i--){
		if (data & 0x80) SDA_High();
		else SDA_Low();
		data <<= 1;
	
		Delay_us(5);
		SCL_High();
		Delay_us(5);
		SCL_Low();
	}

}

uint8_t Soft_I2C_ReadByte(void)
{
	uint8_t data =0;
	
	SDA_High();
	for (int i = 8; i > 0; i--){
		data <<= 1;
		SCL_High();
		
		Delay_us(5);
		if(SDA_Read()) data |= 0x01;
		SCL_Low();
		Delay_us(5);
	}		
	
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
	
	Soft_I2C_Start();

	
}
	










