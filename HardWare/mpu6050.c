#include "mpu6050.h"
#include "i2c.h"
#include "delay.h"

/*
#define MPU6050_ADDR 0x68
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C

#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_PWR_MGMT_2 0x6C
#define MPU6050_WHO_AM_I 0x75
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C

#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_ACCEL_XOUT_L    0x3C
#define MPU6050_ACCEL_YOUT_H    0x3D
#define MPU6050_ACCEL_YOUT_L    0x3E
#define MPU6050_ACCEL_ZOUT_H    0x3F
#define MPU6050_ACCEL_ZOUT_L    0x40

#define MPU6050_TEMP_OUT_H      0x41
#define MPU6050_TEMP_OUT_L      0x42

#define MPU6050_GYRO_XOUT_H     0x43
#define MPU6050_GYRO_XOUT_L     0x44
#define MPU6050_GYRO_YOUT_H     0x45
#define MPU6050_GYRO_YOUT_L     0x46
#define MPU6050_GYRO_ZOUT_H     0x47
#define MPU6050_GYRO_ZOUT_L     0x48
*/

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

uint8_t MPU6050_Init()
{
	uint8_t id = 0;
	
	Soft_I2C_Init();
	Delay_ms(10);
	//判断是否为空指针
	if(MPU6050_GetID(&id) == 0) return 0;
	if(id!=MPU6050_ADDR) return 0;
	
	if(MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x00) == 0) return 0;
	if(MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00) == 0) return 0;
	if(MPU6050_WriteReg(MPU6050_SMPLRT_DIV, 0x07) == 0) return 0;
	
	if(MPU6050_WriteReg(MPU6050_CONFIG, 0x06) == 0) return 0;
	if(MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x00) == 0) return 0;
	if(MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x00) == 0) return 0;
	
	return 1;
	
}

uint8_t MPU6050_GetRawData(MPU6050_RawData *rawdata)
{
	uint8_t buf[14];
	
	if(!Soft_I2C_Read(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, buf, 14)) return 0;
	//变量类型要改成uint16_t的
	rawdata->ACCEL_X =(uint16_t)((buf[0]<<8) | buf[1]);
	rawdata->ACCEL_Y =(uint16_t)((buf[2]<<8) | buf[3]);
	rawdata->ACCEL_Z =(uint16_t)((buf[4]<<8) | buf[5]);
	
	rawdata->GYRO_X =(uint16_t)((buf[8]<<8) | buf[9]);
	rawdata->GYRO_Y =(uint16_t)((buf[10]<<8) | buf[10]);
	rawdata->GYRO_Z =(uint16_t)((buf[12]<<8) | buf[13]);
	
	return 1;
}
//解算原始数据
/*
灵敏度=65536/量程
16384 LSB/g
131.072 LSB/g
*/
void MPU6050_GetData(MPU6050_RawData *rawdata, MPU6050_Data *data)
{
	//灵敏度相当于分辨率
	data->ACCEL_X = (float)rawdata->ACCEL_X/16384.0f;
	data->ACCEL_Y = (float)rawdata->ACCEL_Y/16384.0f;
	data->ACCEL_Z = (float)rawdata->ACCEL_Z/16384.0f;
	
	data->GYRO_X = (float)rawdata->GYRO_X/131.072f;
	data->GYRO_Y = (float)rawdata->GYRO_Y/131.072f;
	data->GYRO_Z = (float)rawdata->GYRO_Z/131.072f;
}











