#ifndef MAHONY_H
#define MAHONY_H

#include "stm32f10x.h"

void Mahony_Init(void);
void Mahony_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt);
void Mahony_GetEuler(float *roll, float *pitch, float *yaw);
void Mahony_GetQuaternion(float *q0, float *q1, float *q2, float *q3);
void Mahony_SetKp(float kp);
void Mahony_SetKi(float ki);

#endif
