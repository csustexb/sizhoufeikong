#include "usart.h"

//USART1初始化
void UART_Init(void)
{
	//定义GPIO和USART1的结构体
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	//使能GPIO和USART时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
	
	//配置TX(PA9)为推挽输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//配置RX(PA10)为浮空输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//配置USART
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength =	USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//硬件流控制方式
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//这样是配置全双工模式，既可以发送也可以接受
	USART_Init(USART1,&USART_InitStructure);
	
	//使能USART1
	USART_Cmd(USART1,ENABLE);
	
}

//重定向printf
int fputc(int ch, FILE *f)
{
	while (USART_GetFlagStatus(USART1,USART_FLAG_TC) == RESET);//等待上一个字符是否发送完毕
	USART_SendData(USART1,(uint8_t) ch);
	return ch;//标准库的约定，fputc成功的时候需要返回该字符
}