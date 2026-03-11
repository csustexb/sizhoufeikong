#ifndef  __I2C_H
#define  __I2C_H


/*
对于I2C通信，其规则需要我们重写
为了更好的操作SDA和SCL的高低电平变换，我们选择直接进行宏定义                                                                                                                    
我们需要定义准备发送，停止发送。单字节读写，指定位置读写，多字节读写
I2C的工作频率若设置为100KHz，每一个工作周期就是10us

*/

#include "stm32f10x.h"                  // Device header

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

#endif