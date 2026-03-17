#include "fly_ctrl.h"
#include "mpu6050.h"
#include "pid.h"

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

static Fly_target fly_target = {0};
static Motor_Out motor_out = {0};

void Fly_Init(void)
{
    PID_Init(&pid_roll_angle,  4.0f, 0.0f, 0.8f,  50.0f, 200.0f);
    PID_Init(&pid_pitch_angle, 4.0f, 0.0f, 0.8f,  50.0f, 200.0f);

    // 内环：输出电机修正量
    PID_Init(&pid_roll_rate,   0.8f, 0.0f, 0.08f, 100.0f, 300.0f);
    PID_Init(&pid_pitch_rate,  0.8f, 0.0f, 0.08f, 100.0f, 300.0f);
    PID_Init(&pid_yaw_rate,    1.2f, 0.0f, 0.00f, 100.0f, 300.0f);
}
