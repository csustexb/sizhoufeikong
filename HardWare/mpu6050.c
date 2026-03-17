#include "mpu6050.h"
#include "i2c.h"
#include "delay.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static MPU6050_Angle g_mpu6050_angle = {0};
static float gyro_x_offset = 0.0f;
static float gyro_y_offset = 0.0f;
static float gyro_z_offset = 0.0f;
static MPU6050_Gyro g_mpu6050_gyro = {0};

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
    if(id == 0) return 0;
    return MPU6050_ReadReg(MPU6050_WHO_AM_I, id);
}

uint8_t MPU6050_Init(void)
{
    uint8_t id = 0;

    Soft_I2C_Init();
    Delay_ms(50);

    if(MPU6050_GetID(&id) == 0) return 0;
    if(id != MPU6050_ADDR) return 0;

    // 解除睡眠
    if(MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x00) == 0) return 0;
    if(MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00) == 0) return 0;

	MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x03);   // 1kHz / (1+3) = 250Hz

    MPU6050_WriteReg(MPU6050_CONFIG, 0x03);       // DLPF 约44Hz，延迟更适中

    // 陀螺仪 ±250°/s
    if(MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x00) == 0) return 0;

    // 加速度 ±2g
    if(MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x00) == 0) return 0;

    Delay_ms(50);

    return 1;
}

uint8_t MPU6050_GetRawData(MPU6050_RawData *rawdata)
{
    uint8_t buf[14];

    if(rawdata == 0) return 0;
    if(!Soft_I2C_Read(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, buf, 14)) return 0;

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
    if(rawdata == 0 || data == 0) return;

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

    if(times == 0) return 0;

    for(i = 0; i < times; i++)
    {
        if(!MPU6050_GetRawData(&raw)) return 0;

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

    float pitch_acc, roll_acc;
    float gyro_x, gyro_y, gyro_z;

    if(dt <= 0.0f) return;
    if(!MPU6050_GetRawData(&raw)) return;

    MPU6050_GetData(&raw, &data);

    // 去零偏
    gyro_x = ((float)raw.GYRO_X - gyro_x_offset) / 131.0f;
    gyro_y = ((float)raw.GYRO_Y - gyro_y_offset) / 131.0f;
    gyro_z = ((float)raw.GYRO_Z - gyro_z_offset) / 131.0f;

    // 保存给内环 PID 用
    g_mpu6050_gyro.GYRO_X = gyro_x;
    g_mpu6050_gyro.GYRO_Y = gyro_y;
    g_mpu6050_gyro.GYRO_Z = gyro_z;

    // 加速度算角度
    roll_acc  = atan2f(data.ACCEL_Y, data.ACCEL_Z) * 180.0f / 3.1415926f;
    pitch_acc = atan2f(-data.ACCEL_X, sqrtf(data.ACCEL_Y * data.ACCEL_Y + data.ACCEL_Z * data.ACCEL_Z)) * 180.0f / 3.1415926f;

    // 互补滤波
    g_mpu6050_angle.Roll  = 0.98f * (g_mpu6050_angle.Roll  + gyro_x * dt) + 0.02f * roll_acc;
    g_mpu6050_angle.Pitch = 0.98f * (g_mpu6050_angle.Pitch + gyro_y * dt) + 0.02f * pitch_acc;

    // 注意：6050没有磁力计，Yaw这里只能积分，会漂
    g_mpu6050_angle.Yaw += gyro_z * dt;
}

void MPU6050_GetAngle(MPU6050_Angle *angle)
{
    if(angle == 0) return;

    angle->Pitch = g_mpu6050_angle.Pitch;
    angle->Roll  = g_mpu6050_angle.Roll;
    angle->Yaw   = g_mpu6050_angle.Yaw;
}

void MPU6050_GetGyro(MPU6050_Gyro *gyro)
{
    if(gyro == 0) return;

    gyro->GYRO_X = g_mpu6050_gyro.GYRO_X;
    gyro->GYRO_Y = g_mpu6050_gyro.GYRO_Y;
    gyro->GYRO_Z = g_mpu6050_gyro.GYRO_Z;
}