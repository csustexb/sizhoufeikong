#ifndef USART_H
#define USART_H

#include "stm32f10x.h"
#include <stdio.h>

#define UART_RX_BUF_SIZE 256

typedef struct {
    uint8_t buf[UART_RX_BUF_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} UART_RingBuf_t;

void UART_Init(void);
void UART_EnableRX(void);
uint8_t UART_GetChar(uint8_t *ch);
uint16_t UART_Available(void);
int fputc(int ch, FILE *f);

#endif