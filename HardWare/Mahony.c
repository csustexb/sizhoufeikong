#include "Mahony.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float twoKp = 2.0f * 0.5f;
static float twoKi = 2.0f * 0.0f;
static float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;

void Mahony_Init(void)
{
	q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
	integralFBx = 0.0f; integralFBy = 0.0f; integralFBz = 0.0f;
}

void Mahony_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	if (ax != 0.0f || ay != 0.0f || az != 0.0f)
	{
		recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;

		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		if (twoKi > 0.0f)
		{
			integralFBx += twoKi * halfex * dt;
			integralFBy += twoKi * halfey * dt;
			integralFBz += twoKi * halfez * dt;
			gx += integralFBx;
			gy += integralFBy;
			gz += integralFBz;
		}
		else
		{
			integralFBx = 0.0f;
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	gx *= 0.5f * dt;
	gy *= 0.5f * dt;
	gz *= 0.5f * dt;
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += ( qa * gx + qc * gz - q3 * gy);
	q2 += ( qa * gy - qb * gz + q3 * gx);
	q3 += ( qa * gz + qb * gy - qc * gx);

	recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

void Mahony_GetEuler(float *roll, float *pitch, float *yaw)
{
	if (roll)  *roll  = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / M_PI;
	if (pitch) *pitch = asinf(2.0f * (q0 * q2 - q3 * q1)) * 180.0f / M_PI;
	if (yaw)   *yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / M_PI;
}

void Mahony_GetQuaternion(float *out_q0, float *out_q1, float *out_q2, float *out_q3)
{
	if (out_q0) *out_q0 = q0;
	if (out_q1) *out_q1 = q1;
	if (out_q2) *out_q2 = q2;
	if (out_q3) *out_q3 = q3;
}

void Mahony_SetKp(float kp)
{
	twoKp = 2.0f * kp;
}

void Mahony_SetKi(float ki)
{
	twoKi = 2.0f * ki;
}
