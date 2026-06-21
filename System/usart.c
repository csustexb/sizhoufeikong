#include "usart.h"

#pragma import(__use_no_semihosting_swi)

struct __FILE { int handle; };
FILE __stdout;
FILE __stdin;

static UART_RingBuf_t g_rx_ring = {0};

//USART1初始化
void UART_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	USART_Cmd(USART1, ENABLE);
}

void UART_EnableRX(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 12;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

uint8_t UART_GetChar(uint8_t *ch)
{
	uint16_t tail;
	if (g_rx_ring.head == g_rx_ring.tail) return 0;
	tail = g_rx_ring.tail;
	*ch = g_rx_ring.buf[tail];
	g_rx_ring.tail = (tail + 1) % UART_RX_BUF_SIZE;
	return 1;
}

uint16_t UART_Available(void)
{
	return (uint16_t)((g_rx_ring.head - g_rx_ring.tail) % UART_RX_BUF_SIZE);
}

void USART1_IRQHandler(void)
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		uint8_t data = (uint8_t)USART_ReceiveData(USART1);
		uint16_t next_head = (g_rx_ring.head + 1) % UART_RX_BUF_SIZE;
		if (next_head != g_rx_ring.tail)
		{
			g_rx_ring.buf[g_rx_ring.head] = data;
			g_rx_ring.head = next_head;
		}
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}





int fputc(int ch, FILE *f)
{
    USART_SendData(USART1, (uint8_t)ch);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    return ch;
}

void _sys_exit(int x)
{
    x = x;
    while (1);
}