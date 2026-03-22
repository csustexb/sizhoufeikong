#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f10x.h"
#include "i2c.h"
#include "delay.h"
#define MPU6050_ADDR         0x68

#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1A
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_CONFIG 0x1C

#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40

#define MPU6050_TEMP_OUT_H   0x41
#define MPU6050_TEMP_OUT_L   0x42

#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_GYRO_XOUT_L  0x44
#define MPU6050_GYRO_YOUT_H  0x45
#define MPU6050_GYRO_YOUT_L  0x46
#define MPU6050_GYRO_ZOUT_H  0x47
#define MPU6050_GYRO_ZOUT_L  0x48

#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_PWR_MGMT_2   0x6C
#define MPU6050_WHO_AM_I     0x75

typedef struct
{
    int16_t ACCEL_X;
    int16_t ACCEL_Y;
    int16_t ACCEL_Z;

    int16_t GYRO_X;
    int16_t GYRO_Y;
    int16_t GYRO_Z;
}MPU6050_RawData;

typedef struct
{
    float ACCEL_X;   // g
    float ACCEL_Y;
    float ACCEL_Z;

    float GYRO_X;    // °/s
    float GYRO_Y;
    float GYRO_Z;
}MPU6050_Data;

typedef struct
{
    float Pitch;     // 俯仰角
    float Roll;      // 横滚角
    float Yaw;       // 这里只做陀螺积分，长期会漂
}MPU6050_Angle;

typedef struct
{
    float GYRO_X;
    float GYRO_Y;
    float GYRO_Z;
}MPU6050_Gyro;

void MPU6050_GetGyro(MPU6050_Gyro *gyro);

uint8_t MPU6050_Init(void);
uint8_t MPU6050_GetID(uint8_t *id);
uint8_t MPU6050_GetRawData(MPU6050_RawData *rawdata);
void MPU6050_GetData(MPU6050_RawData *rawdata, MPU6050_Data *data);

uint8_t MPU6050_GyroCalibrate(uint16_t times);
void MPU6050_AngleUpdate(float dt);
void MPU6050_GetAngle(MPU6050_Angle *angle);
void MPU6050_GetGyro(MPU6050_Gyro *gyro);

// 权重改变函数
void MPU6050_SetAccWeight(float weight);
void MPU6050_SetGyroWeight(float weight);
float MPU6050_GetAccWeight(void);
float MPU6050_GetGyroWeight(void);

#endif