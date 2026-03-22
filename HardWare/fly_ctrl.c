#include "fly_ctrl.h"
#include "mpu6050.h"
#include "pid.h"
#include "pwm.h"

/*
对于使用pid算法来说，很多参数使用内部变量是最好的
这样所有变量复杂的计算全都集中在库内
可以有效地减少逻辑编写难度

PID需要建立外环参数和内环参数，同时两个结构体也要定义
*/

//外环参数，也就是角度,对应任何一个参数都要进行pid进行精准调控，所以全是PID_t的结构体
static PID_t pid_roll_angle;
static PID_t pid_pitch_angle;

//内环参数 
static PID_t pid_roll_rate;
static PID_t pid_pitch_rate;
static PID_t pid_yaw_rate;
//目标值
static Fly_target fly_target = {0};
static Motor_Out motor_out = {0};

static float Limit(float x, float min, float max)
{
    if(x < min) return min;
    if(x > max) return max;
    return x;
}

void Fly_Init(void)
{
    PID_Init(&pid_roll_angle,  4.0f, 0.0f, 0.8f,  50.0f, 200.0f);
    PID_Init(&pid_pitch_angle, 4.0f, 0.0f, 0.8f,  50.0f, 200.0f);

    // 内环：输出电机修正量
    PID_Init(&pid_roll_rate,   0.8f, 0.0f, 0.08f, 100.0f, 300.0f);
    PID_Init(&pid_pitch_rate,  0.8f, 0.0f, 0.08f, 100.0f, 300.0f);
    PID_Init(&pid_yaw_rate,    1.2f, 0.0f, 0.00f, 100.0f, 300.0f);
}
uint8_t Fly_Control_Update(float dt)
{
	MPU6050_Angle angle;
	MPU6050_Gyro gyro;
	
	float target_gx, target_gy;
	float roll_out, pitch_out, yaw_out;
	
	if(dt <= 0) return 0;
	//更新姿态
	MPU6050_AngleUpdate(dt);
	MPU6050_GetAngle(&angle);
	MPU6050_GetGyro(&gyro);
	//更新外环数据
	/*
	PID通过控制角速度来控制角度，通过力矩来控制角速度
	所以我们需要计算出所需力矩就行
	*/
	target_gx = PID_Output(&pid_roll_angle,fly_target.target_roll,angle.Roll,dt);
	target_gy = PID_Output(&pid_pitch_angle,fly_target.target_pitch,angle.Pitch,dt);
	//更新内环数据,测量值就是陀螺仪，陀螺仪本身就是角速度
	roll_out = PID_Output(&pid_roll_rate,target_gx,gyro.GYRO_X,dt);
	pitch_out = PID_Output(&pid_pitch_rate,target_gy,gyro.GYRO_Y,dt);
	yaw_out = PID_Output(&pid_yaw_rate,fly_target.target_yaw_rate,gyro.GYRO_Z,dt);
	
	//混动控制四个油门
	/*
		M1 				M2
		
	
	
		M3				M4	
	*/
	motor_out.m1 = fly_target.throttle - pitch_out - roll_out - yaw_out;
	motor_out.m2 = fly_target.throttle - pitch_out + roll_out + yaw_out;
	motor_out.m3 = fly_target.throttle + pitch_out + roll_out - yaw_out;
	motor_out.m4 = fly_target.throttle + pitch_out - roll_out + yaw_out;
	//限幅一下
	motor_out.m1 = Limit(motor_out.m1, 1000.0f, 2000.0f); 
	motor_out.m1 = Limit(motor_out.m2, 1000.0f, 2000.0f); 
	motor_out.m1 = Limit(motor_out.m3, 1000.0f, 2000.0f); 
	motor_out.m1 = Limit(motor_out.m4, 1000.0f, 2000.0f); 
	
	PWM_SerCompare1(motor_out.m1);
	PWM_SerCompare2(motor_out.m2);
	PWM_SerCompare3(motor_out.m3);
	PWM_SerCompare4(motor_out.m4);
	
}
//初始化四轴数据
void Fly_Control_SetTarget(float throttle, float target_roll, float target_pitch, float target_yaw_rate)
{
	fly_target.throttle = throttle;
	fly_target.target_roll = target_roll;
	fly_target.target_pitch = target_pitch;
	fly_target.target_yaw_rate = target_yaw_rate;
}
//时刻更新电机数据
uint8_t Fly_Control_GetMotorOut(Motor_Out *motor)
{
	if(motor == 0) return 0;
	*motor = motor_out;
	return 1;
}





