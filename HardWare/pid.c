#include "pid.h"
#include <stddef.h>

static float PID_limit(float x, float limit)
{
	if (x > limit) return limit;
	if (x < -limit) return -limit;
	return x;
}

void PID_Init(PID_t *pid, float kp, float ki, float kd, float i_limit, float outlimit)
{
	pid->ki = ki;
	pid->kp = kp;
	pid->kd = kd;
	pid->integral_limit = i_limit;
	pid->output_limit = outlimit;
}

float PID_Output(PID_t *pid, float target, float measure, float dt)
{
	float output;
	float error;
	float derivative;

	error = target - measure;
	pid->integral += dt * error;
	pid->integral = PID_limit(pid->integral, pid->integral_limit);

	derivative = (error - pid->last_error) / dt;

	output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
	output = PID_limit(output, pid->output_limit);

	pid->last_error = error;

	return output;
}

void PID_Reset(PID_t *pid)
{
	pid->integral = 0.0f;
	pid->last_error = 0.0f;
}

// Set PID Kp
void PID_SetKp(PID_t *pid, float kp)
{
	if (pid != NULL)
	{
		pid->kp = kp;
	}
}

// Set PID Ki
void PID_SetKi(PID_t *pid, float ki)
{
	if (pid != NULL)
	{
		pid->ki = ki;
	}
}

// Set PID Kd
void PID_SetKd(PID_t *pid, float kd)
{
	if (pid != NULL)
	{
		pid->kd = kd;
	}
}

// Set all PID gains
void PID_SetGains(PID_t *pid, float kp, float ki, float kd)
{
	if (pid != NULL)
	{
		pid->kp = kp;
		pid->ki = ki;
		pid->kd = kd;
	}
}

// Get PID Kp
float PID_GetKp(PID_t *pid)
{
	if (pid != NULL)
	{
		return pid->kp;
	}
	return 0.0f;
}

// Get PID Ki
float PID_GetKi(PID_t *pid)
{
	if (pid != NULL)
	{
		return pid->ki;
	}
	return 0.0f;
}

// Get PID Kd
float PID_GetKd(PID_t *pid)
{
	if (pid != NULL)
	{
		return pid->kd;
	}
	return 0.0f;
}