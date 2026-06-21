#include "logger.h"
#include "mpu6050.h"
#include "fly_ctrl.h"
#include "delay.h"
#include <string.h>
#include <stdio.h>

/* SPI2 引脚定义 (SD 卡专用) */
#define SD_SPI            SPI2
#define SD_SPI_CLK        RCC_APB1Periph_SPI2
#define SD_GPIO_CLK       RCC_APB2Periph_GPIOB
#define SD_SCK_PIN        GPIO_Pin_13
#define SD_MISO_PIN       GPIO_Pin_14
#define SD_MOSI_PIN       GPIO_Pin_15
#define SD_CS_PIN         GPIO_Pin_12
#define SD_CS_PORT        GPIOB
#define SD_CS_LOW()       GPIO_ResetBits(SD_CS_PORT, SD_CS_PIN)
#define SD_CS_HIGH()      GPIO_SetBits(SD_CS_PORT, SD_CS_PIN)

/* SD 卡命令 */
#define CMD0    0x40
#define CMD1    0x41
#define CMD8    0x48
#define CMD16   0x50
#define CMD17   0x51
#define CMD24   0x58
#define CMD55   0x77
#define ACMD41  0x69

/* 日志状态 */
#define LOG_MAGIC     0x4C4F4745
#define ENTRIES_PER_SECTOR  (LOG_SECTOR_SIZE / LOG_ENTRY_SIZE)

static uint8_t  sd_ready = 0;
static uint32_t current_sector = 0;
static uint32_t entry_count = 0;
static uint32_t log_seq = 0;
static uint8_t  sector_buf[LOG_SECTOR_SIZE] = {0};

static QueueHandle_t log_queue = NULL;

/* ---- 底层 SPI2 操作 ---- */
static void SD_SPI_Init(void)
{
	GPIO_InitTypeDef gpio;
	SPI_InitTypeDef spi;

	RCC_APB1PeriphClockCmd(SD_SPI_CLK, ENABLE);
	RCC_APB2PeriphClockCmd(SD_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);

	gpio.GPIO_Speed = GPIO_Speed_50MHz;

	gpio.GPIO_Pin = SD_SCK_PIN | SD_MOSI_PIN;
	gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(SD_CS_PORT, &gpio);

	gpio.GPIO_Pin = SD_MISO_PIN;
	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(SD_CS_PORT, &gpio);

	gpio.GPIO_Pin = SD_CS_PIN;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(SD_CS_PORT, &gpio);
	SD_CS_HIGH();

	spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	spi.SPI_Mode = SPI_Mode_Master;
	spi.SPI_DataSize = SPI_DataSize_8b;
	spi.SPI_CPOL = SPI_CPOL_Low;
	spi.SPI_CPHA = SPI_CPHA_1Edge;
	spi.SPI_NSS = SPI_NSS_Soft;
	spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	spi.SPI_FirstBit = SPI_FirstBit_MSB;
	spi.SPI_CRCPolynomial = 7;
	SPI_Init(SD_SPI, &spi);
	SPI_Cmd(SD_SPI, ENABLE);
}

static uint8_t SD_ReadWriteByte(uint8_t data)
{
	while (SPI_I2S_GetFlagStatus(SD_SPI, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SD_SPI, data);
	while (SPI_I2S_GetFlagStatus(SD_SPI, SPI_I2S_FLAG_RXNE) == RESET);
	return SPI_I2S_ReceiveData(SD_SPI);
}

static void SD_DummyClocks(uint16_t n)
{
	while (n--) SD_ReadWriteByte(0xFF);
}

static uint8_t SD_GoIdle(void)
{
	uint16_t i;
	SD_CS_HIGH();
	SD_DummyClocks(80);
	SD_CS_LOW();
	SD_ReadWriteByte(CMD0 | 0x00);
	SD_ReadWriteByte(0x00);
	SD_ReadWriteByte(0x00);
	SD_ReadWriteByte(0x00);
	SD_ReadWriteByte(0x00);
	SD_ReadWriteByte(0x95);
	or (i = 0; i < 1000; i++)
	{
		uint8_t r = SD_ReadWriteByte(0xFF);
		if (r == 0x01) { SD_CS_HIGH(); SD_DummyClocks(8); return 1; }
	}
	SD_CS_HIGH();
	return 0;
}

static uint8_t SD_SendIfCond(void)
{
	uint16_t i;
	uint8_t r3[4];
	SD_CS_LOW();
	SD_ReadWriteByte(CMD8 | 0x00);
	SD_ReadWriteByte(0x00);
	SD_ReadWriteByte(0x00);
	SD_ReadWriteByte(0x01);
	SD_ReadWriteByte(0xAA);
	SD_ReadWriteByte(0x87);
	or (i = 0; i < 100; i++)
	{
		r3[0] = SD_ReadWriteByte(0xFF);
		if (r3[0] != 0xFF) break;
	}
	r3[1] = SD_ReadWriteByte(0xFF);
	r3[2] = SD_ReadWriteByte(0xFF);
	r3[3] = SD_ReadWriteByte(0xFF);
	SD_CS_HIGH();
	SD_DummyClocks(8);
	if (r3[0] == 0x01 && r3[3] == 0xAA) return 1;
	if (r3[0] == 0x05) return 1;
	return 0;
}

static uint8_t SD_SendOpCond(void)
{
	uint32_t i;
	or (i = 0; i < 50000; i++)
	{
		SD_CS_LOW();
		SD_ReadWriteByte(CMD55 | 0x00);
		SD_ReadWriteByte(0x00);
		SD_ReadWriteByte(0x00);
		SD_ReadWriteByte(0x00);
		SD_ReadWriteByte(0x00);
		SD_ReadWriteByte(0x01);
		while (SD_ReadWriteByte(0xFF) == 0xFF);

		SD_ReadWriteByte(ACMD41 | 0x00);
		SD_ReadWriteByte(0x40);
		SD_ReadWriteByte(0x00);
		SD_ReadWriteByte(0x00);
		SD_ReadWriteByte(0x00);
		SD_ReadWriteByte(0x01);

		{
			uint16_t j;
			uint8_t r1;
			or (j = 0; j < 100; j++)
			{
				r1 = SD_ReadWriteByte(0xFF);
				if (r1 != 0xFF) break;
			}
			SD_CS_HIGH();
			SD_DummyClocks(8);
			if (r1 == 0x00) return 1;
		}
	}
	return 0;
}

static uint8_t SD_SetBlockLen(void)
{
	uint16_t i;
	uint8_t r1;
	SD_CS_LOW();
	SD_ReadWriteByte(CMD16 | 0x00);
	SD_ReadWriteByte(0x00);
	SD_ReadWriteByte(0x00);
	SD_ReadWriteByte(0x02);
	SD_ReadWriteByte(0x00);
	SD_ReadWriteByte(0x01);
	or (i = 0; i < 100; i++)
	{
		r1 = SD_ReadWriteByte(0xFF);
		if (r1 != 0xFF) break;
	}
	SD_CS_HIGH();
	SD_DummyClocks(8);
	return (r1 == 0x00);
}

static uint8_t SD_ReadBlock(uint32_t sector, uint8_t *buf, uint16_t len)
{
	uint16_t i;
	uint8_t r1;
	uint32_t timeout;

	SD_CS_LOW();
	SD_ReadWriteByte(CMD17 | 0x00);
	SD_ReadWriteByte((uint8_t)(sector >> 24));
	SD_ReadWriteByte((uint8_t)(sector >> 16));
	SD_ReadWriteByte((uint8_t)(sector >> 8));
	SD_ReadWriteByte((uint8_t)(sector));
	SD_ReadWriteByte(0x01);

	timeout = 1000000;
	while (timeout--)
	{
		r1 = SD_ReadWriteByte(0xFF);
		if (r1 != 0xFF) break;
	}
	if (r1 != 0x00)
	{
		SD_CS_HIGH();
		return 0;
	}

	timeout = 1000000;
	while (timeout--)
	{
		if (SD_ReadWriteByte(0xFF) == 0xFE) break;
	}
	if (timeout == 0) { SD_CS_HIGH(); return 0; }

	for (i = 0; i < len; i++) buf[i] = SD_ReadWriteByte(0xFF);
	SD_ReadWriteByte(0xFF);
	SD_ReadWriteByte(0xFF);
	SD_CS_HIGH();
	SD_DummyClocks(8);
	return 1;
}

static uint8_t SD_WriteBlock(uint32_t sector, const uint8_t *buf, uint16_t len)
{
	uint16_t i;
	uint8_t r1;
	uint32_t timeout;

	SD_CS_LOW();
	SD_ReadWriteByte(CMD24 | 0x00);
	SD_ReadWriteByte((uint8_t)(sector >> 24));
	SD_ReadWriteByte((uint8_t)(sector >> 16));
	SD_ReadWriteByte((uint8_t)(sector >> 8));
	SD_ReadWriteByte((uint8_t)(sector));
	SD_ReadWriteByte(0x01);

	timeout = 100000;
	while (timeout--)
	{
		r1 = SD_ReadWriteByte(0xFF);
		if (r1 != 0xFF) break;
	}
	if (r1 != 0x00)
	{
		SD_CS_HIGH();
		return 0;
	}

	SD_ReadWriteByte(0xFE);
	for (i = 0; i < len; i++) SD_ReadWriteByte(buf[i]);
	SD_ReadWriteByte(0xFF);
	SD_ReadWriteByte(0xFF);

	timeout = 100000;
	while (timeout--)
	{
		r1 = SD_ReadWriteByte(0xFF);
		if ((r1 & 0x05) == 0x05) break;
	}
	if ((r1 & 0x1F) != 0x05)
	{
		SD_CS_HIGH();
		SD_DummyClocks(8);
		return 0;
	}

	timeout = 100000;
	while (timeout--)
	{
		if (SD_ReadWriteByte(0xFF) != 0x00) break;
	}
	SD_CS_HIGH();
	SD_DummyClocks(8);
	return 1;
}

static uint8_t SD_InitCard(void)
{
	SD_SPI_Init();
	Delay_ms(10);
	if (!SD_GoIdle()) return 0;
	SD_SendIfCond();
	if (!SD_SendOpCond()) return 0;
	if (!SD_SetBlockLen()) return 0;
	return 1;
}

/* ---- 日志高层 ---- */
static uint32_t LOG_FindNextSector(void)
{
	uint32_t sec = 0;
	LogEntry_t *entry;
	while (1)
	{
		if (!SD_ReadBlock(sec, sector_buf, LOG_SECTOR_SIZE)) break;
		entry = (LogEntry_t *)sector_buf;
		if (entry->magic != LOG_MAGIC) break;
		if (entry_count > 0) { sec++; entry_count = 0; continue; }
		{
			uint8_t i;
			for	 (i = 0; i < ENTRIES_PER_SECTOR; i++)
			{
				entry = (LogEntry_t *)(sector_buf + i * LOG_ENTRY_SIZE);
				if (entry->magic != LOG_MAGIC || entry->seq == 0xFFFFFFFF)
				{
					entry_count = i;
					current_sector = sec;
					return 1;
				}
			}
		}
		sec++;
	}
	if (!SD_ReadBlock(0, sector_buf, LOG_SECTOR_SIZE) ||
	   ((LogEntry_t *)sector_buf)->magic != LOG_MAGIC)
	{
		entry_count = 0;
		current_sector = 0;
		memset(sector_buf, 0, LOG_SECTOR_SIZE);
	}
	return current_sector;
}

uint8_t LOG_Init(void)
{
	if (!SD_InitCard()) return 0;
	sd_ready = 1;
	log_queue = xQueueCreate(10, sizeof(LogEntry_t));
	if (!log_queue) return 0;
	LOG_FindNextSector();
	Delay_ms(1);
	printf("SD card ready, start sector: %lu\r\n", (unsigned long)current_sector);
	return 1;
}

void LOG_WriteEntry(const LogEntry_t *entry)
{
	if (!sd_ready || !log_queue) return;
	xQueueSend(log_queue, entry, 0);
}

void LOG_Task(void *pvParameters)
{
	(void)pvParameters;
	LogEntry_t entry;

	if (!LOG_Init())
	{
		printf("SD card init failed, logger disabled\r\n");
		while (1) vTaskDelay(pdMS_TO_TICKS(1000));
	}

	while (1)
	{
		if (xQueueReceive(log_queue, &entry, portMAX_DELAY) == pdTRUE)
		{
			uint8_t *dst = sector_buf + entry_count * LOG_ENTRY_SIZE;
			memcpy(dst, &entry, LOG_ENTRY_SIZE);
			entry_count++;

			if (entry_count >= ENTRIES_PER_SECTOR)
			{
				if (!SD_WriteBlock(current_sector, sector_buf, LOG_SECTOR_SIZE))
				{
					printf("SD write fail at sector %lu\r\n", (unsigned long)current_sector);
				}
				current_sector++;
				entry_count = 0;
				memset(sector_buf, 0, LOG_SECTOR_SIZE);
			}
		}
	}
}
