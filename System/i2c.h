#ifndef  __I2C_H
#define  __I2C_H


/*
对于I2C通信，其规则需要我们重写
为了更好的操作SDA和SCL的高低电平变换，我们选择直接进行宏定义                                                                                                                    
我们需要定义准备发送，停止发送。单字节读写，指定位置读写，多字节读写
I2C的工作频率若设置为100KHz，每一个工作周期就是10us
I2C在空闲的时候，SCL和SDA都是高电平，所以每次操作之后都要置为高电平
*/

#include "stm32f10x.h"                  // Device header
#include <stdint.h>

#define I2C_PORT GPIOB
#define I2C_CLK  RCC_APB2ENR_IOPBEN

#define I2C_SCL_PIN       6
#define I2C_SDA_PIN       7
// SDA 和 SCL 
#define I2C_SCL_PIN_MASK  (1U << I2C_SCL_PIN)//0x40
#define I2C_SDA_PIN_MASK  (1U << I2C_SDA_PIN)//0x80
// BSRR	端口位设置/复位寄存器	写1到低16位对应引脚输出高电平；写1到高16位对应引脚输出低电平。写0无影响。
// BRR	端口位复位寄存器	写1到相应位将引脚拉低。写0无影响。
// IDR	端口输入数据寄存器	只读，某位为1表示对应引脚输入为高电平，0为低电平。
/* 电平控制 */
#define SDA_High()    (I2C_PORT->BSRR = I2C_SDA_PIN_MASK)
#define SDA_Low()     (I2C_PORT->BRR  = I2C_SDA_PIN_MASK)

#define SCL_High()    (I2C_PORT->BSRR = I2C_SCL_PIN_MASK)
#define SCL_Low()     (I2C_PORT->BRR  = I2C_SCL_PIN_MASK)

#define SDA_Read()    ((I2C_PORT->IDR & I2C_SDA_PIN_MASK) ? 1 : 0)
#define SCL_Read()    ((I2C_PORT->IDR & I2C_SCL_PIN_MASK) ? 1 : 0)

void Soft_SCL_OutPut(void);
void Soft_SDA_OutPut(void);
void Soft_SDA_InPut(void);
void Soft_I2C_Init(void);
void Soft_I2C_Start(void);
void Soft_I2C_Stop(void);
void Soft_I2C_Ack(void);
void Soft_I2C_NoAck(void);
uint8_t Soft_I2C_WaitAck(void);
void Soft_I2C_SendByte(uint8_t data);
uint8_t Soft_I2C_ReadByte(void);
uint8_t Soft_I2C_SendByte_Point(uint8_t slave_addr, uint8_t reg, uint8_t data);//slave_add表示从设备地址
uint8_t Soft_I2C_ReadByte_Point(uint8_t slave_addr, uint8_t reg, uint8_t *data);
uint8_t Soft_I2C_Send(uint8_t slave_addr, uint8_t reg, uint8_t *buf, uint8_t len);
uint8_t Soft_I2C_Read(uint8_t slave_addr, uint8_t reg, uint8_t *buf, uint8_t len);

#endif