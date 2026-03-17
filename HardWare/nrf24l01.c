#include "nrf24l01.h"
#include "delay.h"

/* 默认使用 SPI1:
   PA5 -> SCK
   PA6 -> MISO
   PA7 -> MOSI
*/

static uint8_t NRF24_SPI_RW(uint8_t data)
{
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(SPI1, data);

    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
    return (uint8_t)SPI_I2S_ReceiveData(SPI1);
}
static void NRF24_SPI_WaitDone(void)
{
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET);
}
void NRF24_GPIO_SPI_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;

    /* 开时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
                           RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_AFIO  |
                           RCC_APB2Periph_SPI1, ENABLE);

    /* SPI1 引脚 */
    /* PA5 SCK, PA7 MOSI -> 复用推挽输出 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* PA6 MISO -> 浮空输入 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* CE, CSN -> 推挽输出 */
    GPIO_InitStructure.GPIO_Pin = NRF24_CE_PIN | NRF24_CSN_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* IRQ -> 上拉输入 */
    GPIO_InitStructure.GPIO_Pin = NRF24_IRQ_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    NRF24_CE_L();
    NRF24_CSN_H();

    /* SPI1 配置 */
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI1, &SPI_InitStructure);

    SPI_Cmd(SPI1, ENABLE);

    Delay_ms(5);
}

uint8_t NRF24_ReadReg(uint8_t reg)
{
    uint8_t value;

    NRF24_CSN_L();//拉低对应片选
    NRF24_SPI_RW(NRF24_CMD_R_REGISTER | (reg & 0x1F));//读和写寄存器的地址都是后五位，
    value = NRF24_SPI_RW(NRF24_CMD_NOP);
	
    NRF24_CSN_H();
    return value;
}

uint8_t NRF24_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t status;

    NRF24_CSN_L();
    status = NRF24_SPI_RW(NRF24_CMD_W_REGISTER | (reg & 0x1F));
    NRF24_SPI_RW(value);
	NRF24_SPI_WaitDone();//等待数据完全发送完毕
    NRF24_CSN_H();

    return status;
}

uint8_t NRF24_ReadBuf(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t status;
    uint8_t i;

    NRF24_CSN_L();
    status = NRF24_SPI_RW(NRF24_CMD_R_REGISTER | (reg & 0x1F));
    for (i = 0; i < len; i++)
    {
        buf[i] = NRF24_SPI_RW(NRF24_CMD_NOP);
    }
    NRF24_CSN_H();
	Delay_us(1);
    return status;
}

uint8_t NRF24_WriteBuf(uint8_t reg, const uint8_t *buf, uint8_t len)
{
    uint8_t status;
    uint8_t i;

    NRF24_CSN_L();
    status = NRF24_SPI_RW(NRF24_CMD_W_REGISTER | (reg & 0x1F));
    for (i = 0; i < len; i++)
    {
        NRF24_SPI_RW(buf[i]);
    }
    NRF24_CSN_H();

    return status;
}
//清除Tx模式
void NRF24_FlushTx(void)
{
    NRF24_CSN_L();
    NRF24_SPI_RW(NRF24_CMD_FLUSH_TX);
    NRF24_CSN_H();
}

void NRF24_FlushRx(void)
{
    NRF24_CSN_L();
    NRF24_SPI_RW(NRF24_CMD_FLUSH_RX);
    NRF24_CSN_H();
}

void NRF24_ClearIRQFlags(void)
{
    /* 写1清除 RX_DR / TX_DS / MAX_RT */
    NRF24_WriteReg(NRF24_REG_STATUS,
                   NRF24_STATUS_RX_DR |
                   NRF24_STATUS_TX_DS |
                   NRF24_STATUS_MAX_RT);
}

uint8_t NRF24_Check(void)
{
    uint8_t i;
    uint8_t tx_buf[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
    uint8_t rx_buf[5] = {0};

	//NRF24_REG_TX_ADDR这个地址是可读可写的地址
	//通过对同一地址操作来判断问题
    NRF24_WriteBuf(NRF24_REG_TX_ADDR, tx_buf, 5);
    NRF24_ReadBuf(NRF24_REG_TX_ADDR, rx_buf, 5);

    for (i = 0; i < 5; i++)
    {
        if (rx_buf[i] != tx_buf[i])
        {
            return 0;
        }
    }

    return 1;
}

/* mode:
   NRF24_MODE_TX / NRF24_MODE_RX
*/
uint8_t NRF24_SetMode(uint8_t mode, const uint8_t *addr, uint8_t channel, uint8_t payload_len)
{
    if (addr == 0) return 0;//判空指针
    if (payload_len == 0 || payload_len > 32) return 0;//判固定包长度
    if (channel > 125) return 0;//判通道编号

    NRF24_CE_L();//使芯片待机，再调整参数

    /* 关闭动态载荷和高级特性，固定载荷最稳 */
    NRF24_WriteReg(NRF24_REG_FEATURE, 0x00);
    NRF24_WriteReg(NRF24_REG_DYNPD, 0x00);

    /* 自动应答只开 PIPE0 */
    NRF24_WriteReg(NRF24_REG_EN_AA, 0x01);

    /* 只使能 PIPE0 */
    NRF24_WriteReg(NRF24_REG_EN_RXADDR, 0x01);

    /* 地址宽度 5字节 */
    NRF24_WriteReg(NRF24_REG_SETUP_AW, 0x03);

    /* 自动重发:
       ARD = 500us
       ARC = 10次
       0x1A = 0001 1010
    */
    NRF24_WriteReg(NRF24_REG_SETUP_RETR, 0x1A);

    /* 射频通道 */
    NRF24_WriteReg(NRF24_REG_RF_CH, channel);//规定通讯频道

    /* RF_SETUP:
       1Mbps, 0dBm
       常用值 0x07
    */
    NRF24_WriteReg(NRF24_REG_RF_SETUP, 0x07);

    /* TX地址和PIPE0接收地址设置相同，
       这样自动应答才能正常工作 */
    NRF24_WriteBuf(NRF24_REG_TX_ADDR, addr, NRF24_ADDR_WIDTH);
    NRF24_WriteBuf(NRF24_REG_RX_ADDR_P0, addr, NRF24_ADDR_WIDTH);

    /* 固定载荷长度 */
    NRF24_WriteReg(NRF24_REG_RX_PW_P0, payload_len);

    /* 清中断 */
    NRF24_ClearIRQFlags();

    /* 清空FIFO */
	//清除残留数据
    NRF24_FlushTx();
    NRF24_FlushRx();

    if (mode == NRF24_MODE_TX)
    {
        /* CONFIG:
           EN_CRC=1
           CRCO=1   -> 2字节CRC
           PWR_UP=1
           PRIM_RX=0
           即 0000 1110 = 0x0E
        */
        NRF24_WriteReg(NRF24_REG_CONFIG, 0x0E);
        Delay_ms(2);
        NRF24_CE_L();
    }
    else
    {
        /* RX 模式:
           0000 1111 = 0x0F
        */
        NRF24_WriteReg(NRF24_REG_CONFIG, 0x0F);
        Delay_ms(2);
        NRF24_CE_H();
    }

    return 1;
}

uint8_t NRF24_SendPacket(const uint8_t *data, uint8_t len)
{
    uint8_t status;
    uint16_t timeout = 0;
    uint8_t i;

    if (data == 0) return 0;
    if (len == 0 || len > 32) return 0;

    NRF24_CE_L();
    NRF24_ClearIRQFlags();
    NRF24_FlushTx();

    NRF24_CSN_L();
	//开启写数据模式
    NRF24_SPI_RW(NRF24_CMD_W_TX_PAYLOAD);
    for (i = 0; i < len; i++)
    {
        NRF24_SPI_RW(data[i]);
    }
    NRF24_CSN_H();

    /* 拉高 CE，启动发送 */
    NRF24_CE_H();
    Delay_us(20);
    NRF24_CE_L();

    /* 等待发送完成或失败 */
    do
    {
        status = NRF24_ReadReg(NRF24_REG_STATUS);
        timeout++;
        if (timeout > 60000)
        {
            NRF24_FlushTx();
            NRF24_ClearIRQFlags();
            return 0;
        }
    }
	//若即使没用发送完，发送失败也没被检测，需要及时跳出
    while ((status & (NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT)) == 0);//NRF24_STATUS_TX_DS和NRF24_STATUS_MAX_RT只会有一个置1

    /* 清中断 */
    NRF24_ClearIRQFlags();

    if (status & NRF24_STATUS_TX_DS)
    {
        return 1;
    }

    if (status & NRF24_STATUS_MAX_RT)
    {
        NRF24_FlushTx();
        return 0;
    }

    return 0;
}
//检测是否有数据到来
uint8_t NRF24_IsDataReady(void)
{
	//当收到新的数据包的时候，硬件会自动将NRF24_STATUS_RX_DR置1
    uint8_t status = NRF24_ReadReg(NRF24_REG_STATUS);

    if (status & NRF24_STATUS_RX_DR)
    {
        return 1;
    }

    return 0;
}

uint8_t NRF24_ReceivePacket(uint8_t *data, uint8_t len)
{
    uint8_t i;

    if (data == 0) return 0;
    if (len == 0 || len > 32) return 0;

	//判断是否有数据
    if (!NRF24_IsDataReady())
    {
        return 0;
    }

    NRF24_CSN_L();
	//开启读数据模式
    NRF24_SPI_RW(NRF24_CMD_R_RX_PAYLOAD);
    for (i = 0; i < len; i++)
    {
        data[i] = NRF24_SPI_RW(NRF24_CMD_NOP);
    }
    NRF24_CSN_H();

    NRF24_ClearIRQFlags();

    /* 如果FIFO异常，也顺手清一下 */
    if ((NRF24_ReadReg(NRF24_REG_FIFO_STATUS) & NRF24_FIFO_RX_EMPTY) == 0)
    {
        /* 还有数据可继续读，这里不强制 flush */
    }

    return 1;
}

/* =========================
   遥控应用层
   ========================= */

static uint8_t RC_Checksum8(const uint8_t *buf, uint8_t len)
{
    uint8_t i;
    uint8_t sum = 0;

    for (i = 0; i < len; i++)
    {
        sum += buf[i];
    }

    return sum;
}

uint8_t RC_PacketPack(RC_CtrlPacket_t *pkt)
{
    if (pkt == 0) return 0;

    pkt->checksum = 0;
    pkt->checksum = RC_Checksum8((uint8_t *)pkt, NRF24_PAYLOAD_SIZE - 1);
    return 1;
}

uint8_t RC_PacketCheck(const RC_CtrlPacket_t *pkt)
{
    uint8_t sum;

    if (pkt == 0) return 0;

    sum = RC_Checksum8((const uint8_t *)pkt, NRF24_PAYLOAD_SIZE - 1);

    if (sum == pkt->checksum)
    {
        return 1;
    }

    return 0;
}

uint8_t RC_SendControl(RC_CtrlPacket_t *pkt)
{
    if (pkt == 0) return 0;

    RC_PacketPack(pkt);
    return NRF24_SendPacket((const uint8_t *)pkt, NRF24_PAYLOAD_SIZE);
}

uint8_t FC_ReceiveControl(RC_CtrlPacket_t *pkt)
{
    if (pkt == 0) return 0;

    if (!NRF24_ReceivePacket((uint8_t *)pkt, NRF24_PAYLOAD_SIZE))
    {
        return 0;
    }

    if (!RC_PacketCheck(pkt))
    {
        return 0;
    }

    return 1;
}