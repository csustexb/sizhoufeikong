#ifndef __USART_H
#define __USART_H

#include "stm32f10x.h"                  // Device header
#include <stdio.h>          //为了重定义printf函数



void UART_Init(void);
int fputc(int ch, FILE *f);


#endif