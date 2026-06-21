#include "mpu6050.h"
#include "Mahony.h"
#include <stddef.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static uint8_t mahony_initialized = 0;
static MPU6050_Angle g_mpu6050_angle = {0};
static float gyro_x_offset = 0.0f;
static float gyro_y_offset = 0.0f;
static float gyro_z_offset = 0.0f;
static MPU6050_Gyro g_mpu6050_gyro = {0};

// 中断信号量（用于通知任务读取数据）
SemaphoreHandle_t xMpuSemaphore = NULL;

static uint8_t MPU6050_WriteReg(uint8_t reg, uint8_t data)
{
    return Soft_I2C_SendByte_Point(MPU6050_ADDR, reg, data);
}

static uint8_t MPU6050_ReadReg(uint8_t reg, uint8_t *data)
{
    return Soft_I2C_ReadByte_Point(MPU6050_ADDR, reg, data);
}

uint8_t MPU6050_GetID(uint8_t *id)
{
    if (id == 0) return 0;
    return MPU6050_ReadReg(MPU6050_WHO_AM_I, id);
}

uint8_t MPU6050_Init(void)
{
    uint8_t id = 0;

    Soft_I2C_Init();
    Delay_ms(50);

    if (MPU6050_GetID(&id) == 0) return 0;
    if (id != MPU6050_ADDR) return 0;

    // 解除睡眠
    if (MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x00) == 0) return 0;
    if (MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00) == 0) return 0;

	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x03);   // 1kHz / (1+3) = 250Hz

    MPU6050_WriteReg(MPU6050_CONFIG, 0x03);       // DLPF 约44Hz，延迟更适中

    // 陀螺仪 ±250°/s
    if (MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x00) == 0) return 0;

    // 加速度 ±2g
    if (MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x00) == 0) return 0;

    Delay_ms(50);

    return 1;
}

uint8_t MPU6050_GetRawData(MPU6050_RawData *rawdata)
{
    uint8_t buf[14];

    if (rawdata == 0) return 0;
    if (!Soft_I2C_Read(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, buf, 14)) return 0;

    rawdata->ACCEL_X = (int16_t)((buf[0]  << 8) | buf[1]);
    rawdata->ACCEL_Y = (int16_t)((buf[2]  << 8) | buf[3]);
    rawdata->ACCEL_Z = (int16_t)((buf[4]  << 8) | buf[5]);

    rawdata->GYRO_X  = (int16_t)((buf[8]  << 8) | buf[9]);
    rawdata->GYRO_Y  = (int16_t)((buf[10] << 8) | buf[11]);
    rawdata->GYRO_Z  = (int16_t)((buf[12] << 8) | buf[13]);

    return 1;
}

void MPU6050_GetData(MPU6050_RawData *rawdata, MPU6050_Data *data)
{
    if (rawdata == 0 || data == 0) return;

    // ±2g -> 16384 LSB/g
    data->ACCEL_X = (float)rawdata->ACCEL_X / 16384.0f;
    data->ACCEL_Y = (float)rawdata->ACCEL_Y / 16384.0f;
    data->ACCEL_Z = (float)rawdata->ACCEL_Z / 16384.0f;

    // ±250°/s -> 131.0 LSB/(°/s)
    data->GYRO_X = (float)rawdata->GYRO_X / 131.0f;
    data->GYRO_Y = (float)rawdata->GYRO_Y / 131.0f;
    data->GYRO_Z = (float)rawdata->GYRO_Z / 131.0f;
}
//陀螺仪零漂校准
uint8_t MPU6050_GyroCalibrate(uint16_t times)
{
    MPU6050_RawData raw;
    uint32_t i;
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_z = 0.0f;

    if (times == 0) return 0;

    or (i = 0; i < times; i++)
    {
        if (!MPU6050_GetRawData(&raw)) return 0;

        sum_x += raw.GYRO_X;
        sum_y += raw.GYRO_Y;
        sum_z += raw.GYRO_Z;

        Delay_ms(2);
    }

	//使用的是static变量，直接存储在库里
    gyro_x_offset = sum_x / times;
    gyro_y_offset = sum_y / times;
    gyro_z_offset = sum_z / times;

    return 1;
}

void MPU6050_AngleUpdate(float dt)
{
    MPU6050_RawData raw;
    MPU6050_Data data;

    float gyro_x, gyro_y, gyro_z;

    if (dt <= 0.0f) return;
    if (!MPU6050_GetRawData(&raw)) return;

    MPU6050_GetData(&raw, &data);

    // 去零偏
    gyro_x = ((float)raw.GYRO_X - gyro_x_offset) / 131.0f;
    gyro_y = ((float)raw.GYRO_Y - gyro_y_offset) / 131.0f;
    gyro_z = ((float)raw.GYRO_Z - gyro_z_offset) / 131.0f;

    // 保存给内环 PID 用
    g_mpu6050_gyro.GYRO_X = gyro_x;
    g_mpu6050_gyro.GYRO_Y = gyro_y;
    g_mpu6050_gyro.GYRO_Z = gyro_z;

    if (!mahony_initialized)
    {
        Mahony_Init();
        mahony_initialized = 1;
    }

    // Mahony 滤波
    Mahony_Update(gyro_x, gyro_y, gyro_z, data.ACCEL_X, data.ACCEL_Y, data.ACCEL_Z, dt);
    Mahony_GetEuler(&g_mpu6050_angle.Roll, &g_mpu6050_angle.Pitch, &g_mpu6050_angle.Yaw);
}

void MPU6050_GetAngle(MPU6050_Angle *angle)
{
    if (angle == 0) return;

    angle->Pitch = g_mpu6050_angle.Pitch;
    angle->Roll  = g_mpu6050_angle.Roll;
    angle->Yaw   = g_mpu6050_angle.Yaw;
}

void MPU6050_GetGyro(MPU6050_Gyro *gyro)
{
    if (gyro == 0) return;

    gyro->GYRO_X = g_mpu6050_gyro.GYRO_X;
    gyro->GYRO_Y = g_mpu6050_gyro.GYRO_Y;
    gyro->GYRO_Z = g_mpu6050_gyro.GYRO_Z;
}

// 设置 Mahony 滤波器比例增益 Kp
void MPU6050_SetMahonyKp(float kp)
{
    Mahony_SetKp(kp);
}

// 设置 Mahony 滤波器积分增益 Ki
void MPU6050_SetMahonyKi(float ki)
{
    Mahony_SetKi(ki);
}

/**
 * @brief 初始化 MPU6050 中断（PB4 - EXTI4）
 * @return 1: 成功, 0: 失败
 */
uint8_t MPU6050_INT_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // 配置 MPU6050 的中断输出
    // 0x37 INT_PIN_CFG：中断引脚配置（低有效、推挽、保存直到读取状态）
    if (MPU6050_WriteReg(MPU6050_INT_CONFIG, 0x00) == 0) return 0;  // 边缘触发，推挽输出

    // 0x38 INT_ENABLE：使能 data ready 中断
    if (MPU6050_WriteReg(MPU6050_INT_ENABLE, 0x01) == 0) return 0;  // Enable DATA_RDY_EN

    // 配置 GPIO PB4
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;      // 下拉输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 配置 EXTI4（PB4 -> EXTI4）
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource4);

    EXTI_InitStructure.EXTI_Line = EXTI_Line4;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  // 上升沿触发
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // 配置 NVIC
    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 12;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    return 1;
}

/**
 * @brief MPU6050 中断处理函数 — 仅清标志 + 给信号量，不做 I2C/浮点运算
 */
void MPU6050_INT_Handler(void)
{
    if (EXTI_GetITStatus(EXTI_Line4) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line4);

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (xMpuSemaphore != NULL)
        {
            xSemaphoreGiveFromISR(xMpuSemaphore, &xHigherPriorityTaskWoken);
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void EXTI4_IRQHandler(void)
{
    MPU6050_INT_Handler();
}
