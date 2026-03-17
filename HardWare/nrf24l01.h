#ifndef __NRF24L01_H
#define __NRF24L01_H

#include "stm32f10x.h"
#include <stdint.h>

/* =========================
   用户可修改的引脚定义
   ========================= */
#define NRF24_CE_PORT       GPIOA
#define NRF24_CE_PIN        GPIO_Pin_2

#define NRF24_CSN_PORT      GPIOA
#define NRF24_CSN_PIN       GPIO_Pin_4

#define NRF24_IRQ_PORT      GPIOB
#define NRF24_IRQ_PIN       GPIO_Pin_0

/* 默认地址宽度 5 字节 */
#define NRF24_ADDR_WIDTH    5

/* 固定载荷长度
   遥控场景下建议固定长度，简单稳定 */
#define NRF24_PAYLOAD_SIZE  12

/* 电平控制 */
#define NRF24_CE_H()        GPIO_SetBits(NRF24_CE_PORT, NRF24_CE_PIN)
#define NRF24_CE_L()        GPIO_ResetBits(NRF24_CE_PORT, NRF24_CE_PIN)

#define NRF24_CSN_H()       GPIO_SetBits(NRF24_CSN_PORT, NRF24_CSN_PIN)
#define NRF24_CSN_L()       GPIO_ResetBits(NRF24_CSN_PORT, NRF24_CSN_PIN)

#define NRF24_IRQ_READ()    ((NRF24_IRQ_PORT->IDR & NRF24_IRQ_PIN) ? 1 : 0)

/* =========================
   NRF24L01+ 指令
   ========================= */
#define NRF24_CMD_R_REGISTER        0x00
#define NRF24_CMD_W_REGISTER        0x20
#define NRF24_CMD_R_RX_PAYLOAD      0x61
#define NRF24_CMD_W_TX_PAYLOAD      0xA0
#define NRF24_CMD_FLUSH_TX          0xE1
#define NRF24_CMD_FLUSH_RX          0xE2
#define NRF24_CMD_REUSE_TX_PL       0xE3
#define NRF24_CMD_NOP               0xFF

/* =========================
   NRF24L01+ 寄存器
   ========================= */
#define NRF24_REG_CONFIG            0x00
#define NRF24_REG_EN_AA             0x01
#define NRF24_REG_EN_RXADDR         0x02
#define NRF24_REG_SETUP_AW          0x03
#define NRF24_REG_SETUP_RETR        0x04
#define NRF24_REG_RF_CH             0x05
#define NRF24_REG_RF_SETUP          0x06
#define NRF24_REG_STATUS            0x07
#define NRF24_REG_OBSERVE_TX        0x08
#define NRF24_REG_RPD               0x09
#define NRF24_REG_RX_ADDR_P0        0x0A
#define NRF24_REG_RX_ADDR_P1        0x0B
#define NRF24_REG_RX_ADDR_P2        0x0C
#define NRF24_REG_RX_ADDR_P3        0x0D
#define NRF24_REG_RX_ADDR_P4        0x0E
#define NRF24_REG_RX_ADDR_P5        0x0F
#define NRF24_REG_TX_ADDR           0x10
#define NRF24_REG_RX_PW_P0          0x11
#define NRF24_REG_FIFO_STATUS       0x17
#define NRF24_REG_DYNPD             0x1C
#define NRF24_REG_FEATURE           0x1D

/* STATUS 标志位 */
#define NRF24_STATUS_RX_DR          0x40
#define NRF24_STATUS_TX_DS          0x20
#define NRF24_STATUS_MAX_RT         0x10

/* FIFO_STATUS 标志位 */
#define NRF24_FIFO_TX_REUSE         0x40
#define NRF24_FIFO_TX_FULL          0x20
#define NRF24_FIFO_TX_EMPTY         0x10
#define NRF24_FIFO_RX_FULL          0x02
#define NRF24_FIFO_RX_EMPTY         0x01

/* 配置参数 */
#define NRF24_MODE_TX               0
#define NRF24_MODE_RX               1

/* 适合遥控的控制包
   总长度 12 字节 */
typedef struct
{
    uint16_t throttle;   /* 1000~2000 */
    int16_t  roll;       /* -500~500 */
    int16_t  pitch;      /* -500~500 */
    int16_t  yaw;        /* -500~500 */
    uint8_t  sw1;        /* 开关 */
    uint8_t  sw2;        /* 开关 */
    uint8_t  seq;        /* 递增序号 */
    uint8_t  checksum;   /* 前11字节求和 */
} RC_CtrlPacket_t;

/* =========================
   底层驱动接口
   ========================= */
void NRF24_GPIO_SPI_Init(void);
uint8_t NRF24_Check(void);

uint8_t NRF24_ReadReg(uint8_t reg);
uint8_t NRF24_WriteReg(uint8_t reg, uint8_t value);
uint8_t NRF24_ReadBuf(uint8_t reg, uint8_t *buf, uint8_t len);
uint8_t NRF24_WriteBuf(uint8_t reg, const uint8_t *buf, uint8_t len);

void NRF24_FlushTx(void);
void NRF24_FlushRx(void);
void NRF24_ClearIRQFlags(void);

uint8_t NRF24_SetMode(uint8_t mode, const uint8_t *addr, uint8_t channel, uint8_t payload_len);

uint8_t NRF24_SendPacket(const uint8_t *data, uint8_t len);
uint8_t NRF24_IsDataReady(void);
uint8_t NRF24_ReceivePacket(uint8_t *data, uint8_t len);

/* =========================
   遥控应用层接口
   ========================= */
uint8_t RC_PacketPack(RC_CtrlPacket_t *pkt);
uint8_t RC_PacketCheck(const RC_CtrlPacket_t *pkt);

uint8_t RC_SendControl(RC_CtrlPacket_t *pkt);
uint8_t FC_ReceiveControl(RC_CtrlPacket_t *pkt);

#endif