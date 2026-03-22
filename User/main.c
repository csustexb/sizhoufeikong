#include "stm32f10x.h"
#include "usart.h"
#include "mpu6050.h"
#include "fly_ctrl.h"
#include "pwm.h"
#include "adc.h"
#include "nrf24l01.h"
#include <stdio.h>

/* FreeRTOS头文件 */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* 任务堆栈大小定义 */
#define CTRL_TASK_STACK_SIZE    (256)      // 飞行控制任务
#define SENSOR_TASK_STACK_SIZE  (256)      // 传感器采集任务
#define ROCKET_TASK_STACK_SIZE  (256)      // 摇杆控制任务
#define MONITOR_TASK_STACK_SIZE (128)      // 监测任务
#define LED_TASK_STACK_SIZE     (128)      // LED指示任务

/*
在FreeRTOSConfig.h中，以下宏定义控制了FreeRTOS的功能：
#define configUSE_MUTEXES			1
#define configUSE_COUNTING_SEMAPHORES 1
#define configUSE_QUEUE_SETS		1
Mutex：防止多个任务同时抢同一个资源
Counting Semaphore：用来计数事件或管理多个同类资源
Queue Set：让一个任务同时等多个队列/信号量
将这几个宏定义设置为1，才能使用这些功能
*/

/* 任务优先级定义（0~4，4最高） */
#define CTRL_TASK_PRIORITY      (4)        // 最高优先级：飞行控制必须及时
#define ROCKET_TASK_PRIORITY    (3)        // 高优先级：摇杆控制
#define SENSOR_TASK_PRIORITY    (3)        // 高优先级：传感器采集
#define MONITOR_TASK_PRIORITY   (2)        // 中优先级：监测输出
#define LED_TASK_PRIORITY       (1)        // 最低优先级：LED指示

/* 全局数据结构 */
typedef struct
{
    MPU6050_Angle angle;
    MPU6050_Gyro gyro;
} SensorData_t;

/* 摇杆数据结构 - 更新于Rocket_Control_Task */
typedef struct
{
    float throttle;        // 油门 (0.0 ~ 100.0)
    float roll_input;      // 横滚输入 (-100.0 ~ 100.0)
    float pitch_input;     // 俯仰输入 (-100.0 ~ 100.0)
    float yaw_input;       // 偏航输入 (-100.0 ~ 100.0)
    uint8_t armed;         // 解武装标志 (0=未武装, 1=已武装)
} RocketInput_t;

static SensorData_t sensor_data = {0};
static Motor_Out motor_out = {0};
static RocketInput_t rocket_input = {0};  // 摇杆输入

/* 同步信号量和互斥锁 */
static SemaphoreHandle_t sensor_semaphore;   // 传感器数据更新信号量
static SemaphoreHandle_t motor_mutex;        // 电机数据互斥锁
static SemaphoreHandle_t sensor_mutex;       // 传感器数据互斥锁
static SemaphoreHandle_t rocket_mutex;       // 摇杆数据互斥锁

/* =============================================================
 * 摇杆数据处理函数
 * 功能：将ADC原始值转换为百分比输入
 * ============================================================= */

/**
 * @brief ADC值转换为摇杆输入百分比
 * @param adc_value ADC原始值 (0-4095 for 12-bit)
 * @param deadzone 死区范围 (0-2048, 默认100)
 * @param is_throttle 是否为油门（油门只有正值）
 * @return 百分比值 (-100~100 或 0~100)
 */
static float ADC_RawToPercent(uint16_t adc_value, uint16_t deadzone, uint8_t is_throttle)
{
    // 12位ADC，中点为2048
    const uint16_t ADC_MID = 2048;
    const float ADC_MAX = 4095.0f;
    
    float percent;
    
    if(is_throttle)
    {
        // 油门模式：0-4095 → 0-100%
        percent = (adc_value / ADC_MAX) * 100.0f;
    }
    else
    {
        // 摇杆模式：处理死区
        int16_t offset = (int16_t)adc_value - ADC_MID;
        
        if(offset > -deadzone && offset < deadzone)
        {
            // 在死区内，视为0
            return 0.0f;
        }
        
        // 超出死区的部分进行线性映射
        if(offset >= 0)
        {
            // 正方向：deadzone ~ 2047 → 0 ~ 100%
            percent = ((float)offset - deadzone) / (ADC_MID - deadzone) * 100.0f;
        }
        else
        {
            // 负方向：-2048 ~ -deadzone → -100 ~ 0%
            percent = ((float)offset + deadzone) / (ADC_MID - deadzone) * 100.0f;
        }
    }
    
    // 限制范围
    if(percent > 100.0f) percent = 100.0f;
    if(percent < -100.0f) percent = -100.0f;
    
    return percent;
}

/**
 * @brief 一阶低通滤波器
 * @param old_value 上一次的值
 * @param new_value 新测量值
 * @param alpha 滤波系数 (0.0~1.0, 0.1表示新值权重10%)
 * @return 滤波后的值
 */
static float LowPassFilter(float old_value, float new_value, float alpha)
{
    return old_value * (1.0f - alpha) + new_value * alpha;
}

/* =============================================================
 * 摇杆控制任务（高优先级）
 * 功能：通过无线模块和ADC读取摇杆数据，转换为飞行输入
 * 周期：20ms（50Hz）
 * ============================================================= */
/* =============================================================
 * NRF24L01接收处理
 * 功能：接收NRF24L01的控制包并转换为摇杆输入
 * ============================================================= */
static uint8_t NRF24_ReceiveRocketData(RocketInput_t *pRocket)
{
    RC_CtrlPacket_t rc_pkt;
    
    // 检查是否有新数据
    if(!NRF24_IsDataReady())
    {
        return 0;  // 没有新数据
    }
    
    // 接收控制包
    if(!FC_ReceiveControl(&rc_pkt))
    {
        return 0;  // 接收失败或校验错误
    }
    
    /* 
     * NRF24L01的控制包格式：
     * - throttle: 1000~2000 (PWM脉宽微秒)
     * - roll:     -500~500   (角度或速度)
     * - pitch:    -500~500   (角度或速度)
     * - yaw:      -500~500   (角度或速度)
     * - sw1, sw2: 开关状态
     */
    
    // 油门：1000~2000us → 0~100%
    // 遥控通常映射：1000us=0%, 1500us=50%, 2000us=100%
    pRocket->throttle = ((float)(rc_pkt.throttle - 1000) / 1000.0f) * 100.0f;
    if(pRocket->throttle < 0.0f) pRocket->throttle = 0.0f;
    if(pRocket->throttle > 100.0f) pRocket->throttle = 100.0f;
    
    // 横滚、俯仰、偏航：-500~500 → -100~100%
    pRocket->roll_input = ((float)rc_pkt.roll / 500.0f) * 100.0f;
    pRocket->pitch_input = ((float)rc_pkt.pitch / 500.0f) * 100.0f;
    pRocket->yaw_input = ((float)rc_pkt.yaw / 500.0f) * 100.0f;
    
    // 武装标志：通常由开关控制
    // sw1 可用作武装开关 (0=未武装, 非0=已武装)
    pRocket->armed = (rc_pkt.sw1 != 0) ? 1 : 0;
    
    return 1;  // 成功接收并转换
}

/* =============================================================
 * 任务：摇杆控制任务（优先级3）
 * 功能：通过NRF24L01接收遥控信号，转换为飞行输入
 * 周期：20ms（50Hz）
 * ============================================================= */
void Rocket_Control_Task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20);  // 20ms周期
    
    RocketInput_t temp_rocket;
    
    // 上一次的过滤值（用于低通滤波）
    static float last_throttle = 0.0f;
    static float last_roll = 0.0f;
    static float last_pitch = 0.0f;
    static float last_yaw = 0.0f;
    
    // 低通滤波系数（0.15 = 新值权重15%）
    const float FILTER_ALPHA = 0.15f;
    
    // 通信超时计数器（10个周期未收到数据 = 200ms超时）
    uint8_t timeout_count = 0;
    const uint8_t TIMEOUT_THRESHOLD = 10;
    
    printf("摇杆控制任务启动，准备接收NRF24L01信号...\r\n");
    
    while(1)
    {
        // 尝试从NRF24L01接收遥控数据
        if(NRF24_ReceiveRocketData(&temp_rocket))
        {
            // 成功接收到新数据
            timeout_count = 0;
            
            // 应用低通滤波器（平滑数据）
            temp_rocket.throttle = LowPassFilter(last_throttle, temp_rocket.throttle, FILTER_ALPHA);
            temp_rocket.roll_input = LowPassFilter(last_roll, temp_rocket.roll_input, FILTER_ALPHA);
            temp_rocket.pitch_input = LowPassFilter(last_pitch, temp_rocket.pitch_input, FILTER_ALPHA);
            temp_rocket.yaw_input = LowPassFilter(last_yaw, temp_rocket.yaw_input, FILTER_ALPHA);
            
            // 保存滤波值用于下一次迭代
            last_throttle = temp_rocket.throttle;
            last_roll = temp_rocket.roll_input;
            last_pitch = temp_rocket.pitch_input;
            last_yaw = temp_rocket.yaw_input;
        }
        else
        {
            // 未接收到新数据
            timeout_count++;
            
            if(timeout_count >= TIMEOUT_THRESHOLD)
            {
                // 通信丢失 - 安全状态：断油、清零指令、解武装
                printf("[WARNING] NRF24L01通信丢失!\r\n");
                temp_rocket.throttle = 0.0f;
                temp_rocket.roll_input = 0.0f;
                temp_rocket.pitch_input = 0.0f;
                temp_rocket.yaw_input = 0.0f;
                temp_rocket.armed = 0;  // 立即解武装
                
                // 保存的值保持不变，用于下次恢复
                timeout_count = TIMEOUT_THRESHOLD;  // 不再增加
            }
            // 否则保持上一次的值（可选的失链处理）
        }
        
        // 保存摇杆数据到共享变量（受保护）
        if(xSemaphoreTake(rocket_mutex, pdMS_TO_TICKS(2)) == pdTRUE)
        {
            rocket_input = temp_rocket;
            xSemaphoreGive(rocket_mutex);
        }
        
        // 周期性等待
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/* =============================================================
 * 任务2：飞行控制任务（最高优先级）
 * 功能：执行PID控制算法，计算电机速度
 * 周期：10ms（100Hz）
 * ============================================================= */
void Fly_Control_Task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);  // 10ms周期
    Motor_Out temp_motor;
    RocketInput_t temp_rocket;
    
    while(1)
    {
        // 获取摇杆输入
        if(xSemaphoreTake(rocket_mutex, pdMS_TO_TICKS(2)) == pdTRUE)
        {
            temp_rocket = rocket_input;
            xSemaphoreGive(rocket_mutex);
        }
        
        // 只有在武装状态下才执行控制
        if(temp_rocket.armed == 1)
        {
            // 根据摇杆输入转换为飞行目标
            // 油门：0-100% → PWM 1000-2000us
            float throttle = 1000.0f + (temp_rocket.throttle / 100.0f) * 1000.0f;
            
            // 角度范围：-45° ~ +45°
            float target_roll = (temp_rocket.roll_input / 100.0f) * 45.0f;
            float target_pitch = (temp_rocket.pitch_input / 100.0f) * 45.0f;
            
            // 角速度范围：-180°/s ~ +180°/s
            float target_yaw_rate = (temp_rocket.yaw_input / 100.0f) * 180.0f;
            
            // 设置飞行目标
            Fly_Control_SetTarget(throttle, target_roll, target_pitch, target_yaw_rate);
        }
        else
        {
            // 未武装状态：电机停止
            Fly_Control_SetTarget(1000.0f, 0.0f, 0.0f, 0.0f);
        }
        
        // 获取最新的传感器数据
        if(xSemaphoreTake(sensor_mutex, pdMS_TO_TICKS(2)) == pdTRUE)
        {
            // 执行飞行控制更新（dt = 0.01s）
            Fly_Control_Update(0.01f);
            Fly_Control_GetMotorOut(&temp_motor);
            
            xSemaphoreGive(sensor_mutex);
        }
        
        // 更新电机输出（受保护的写入）
        if(xSemaphoreTake(motor_mutex, pdMS_TO_TICKS(1)) == pdTRUE)
        {
            motor_out = temp_motor;
            xSemaphoreGive(motor_mutex);
        }
        
        // 周期性等待
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/* =============================================================
 * 任务3：传感器采集任务（高优先级）
 * 功能：读取MPU6050传感器数据，更新角度
 * 周期：5ms（200Hz）
 * ============================================================= */
void Sensor_Read_Task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5);   // 5ms周期
    
    while(1)
    {
        // 更新姿态数据
        MPU6050_AngleUpdate(0.005f);  // dt = 0.005s = 5ms
        
        // 临界区：更新共享数据
        if(xSemaphoreTake(sensor_mutex, pdMS_TO_TICKS(2)) == pdTRUE)
        {
            MPU6050_GetAngle(&sensor_data.angle);
            MPU6050_GetGyro(&sensor_data.gyro);
            xSemaphoreGive(sensor_mutex);
        }
        
        // 通知飞行控制任务数据已更新
        xSemaphoreGive(sensor_semaphore);
        
        // 周期性等待
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
/* =============================================================
 * 任务4：监测与通信任务（低优先级）
 * 功能：通过UART输出调试信息
 * 周期：200ms
 * ============================================================= */
void Monitor_Task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(200);  // 200ms周期
    
    SensorData_t temp_sensor;
    Motor_Out temp_motor;
    RocketInput_t temp_rocket;
    
    while(1)
    {
        // 读取传感器数据（受保护）
        if(xSemaphoreTake(sensor_mutex, pdMS_TO_TICKS(5)) == pdTRUE)
        {
            temp_sensor = sensor_data;
            xSemaphoreGive(sensor_mutex);
        }
        
        // 读取电机输出（受保护）
        if(xSemaphoreTake(motor_mutex, pdMS_TO_TICKS(5)) == pdTRUE)
        {
            temp_motor = motor_out;
            xSemaphoreGive(motor_mutex);
        }
        
        // 读取摇杆输入（受保护）
        if(xSemaphoreTake(rocket_mutex, pdMS_TO_TICKS(5)) == pdTRUE)
        {
            temp_rocket = rocket_input;
            xSemaphoreGive(rocket_mutex);
        }
        
        // 输出调试信息
        printf("\r\n===== 摇杆输入 =====\r\n");
        printf("油门: %.1f%%  横滚: %.1f%%  俯仰: %.1f%%  偏航: %.1f%%\r\n",
               temp_rocket.throttle,
               temp_rocket.roll_input,
               temp_rocket.pitch_input,
               temp_rocket.yaw_input);
        printf("武装状态: %s\r\n", temp_rocket.armed ? "已武装" : "未武装");
        
        printf("===== 姿态数据 =====\r\n");
        printf("Pitch: %.2f°  Roll: %.2f°  Yaw: %.2f°\r\n", 
               temp_sensor.angle.Pitch, 
               temp_sensor.angle.Roll, 
               temp_sensor.angle.Yaw);
        
        printf("===== 角速度 =====\r\n");
        printf("GX: %.2f  GY: %.2f  GZ: %.2f (°/s)\r\n", 
               temp_sensor.gyro.GYRO_X, 
               temp_sensor.gyro.GYRO_Y, 
               temp_sensor.gyro.GYRO_Z);
        
        printf("===== 电机输出 =====\r\n");
        printf("M1: %.0f  M2: %.0f  M3: %.0f  M4: %.0f\r\n", 
               temp_motor.m1, 
               temp_motor.m2, 
               temp_motor.m3, 
               temp_motor.m4);
        
        // 周期性等待
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/* =============================================================
 * 主函数
 * ============================================================= */
int main(void)
{
    /* 硬件初始化 */
    UART_Init();
    printf("系统启动...\r\n");
    
    /* NRF24L01初始化 - RX模式 */
    printf("正在初始化NRF24L01...\r\n");
    NRF24_GPIO_SPI_Init();
    Delay_ms(100);
    
    uint8_t nrf_addr[5] = {0x12, 0x34, 0x56, 0x78, 0x9A};  // 接收地址
    uint8_t nrf_channel = 40;  // 2.4GHz通道
    
    if(!NRF24_Check())
    {
        printf("NRF24L01检测失败！\r\n");
        while(1);
    }
    
    if(!NRF24_SetMode(NRF24_MODE_RX, nrf_addr, nrf_channel, NRF24_PAYLOAD_SIZE))
    {
        printf("NRF24L01初始化失败！\r\n");
        while(1);
    }
    printf("NRF24L01初始化成功，等待遥控信号...\r\n");
    
    /* MPU6050初始化 */
    printf("正在校准陀螺仪...\r\n");
    if(MPU6050_GyroCalibrate(50) == 0)
    {
        printf("陀螺仪校准失败！\r\n");
        while(1);
    }
    
    printf("正在初始化MPU6050...\r\n");
    if(MPU6050_Init() == 0)
    {
        printf("MPU6050初始化失败！\r\n");
        while(1);
    }
    
    /* 飞行控制初始化 */
    Fly_Init();
    printf("飞行控制初始化完成\r\n");
    
    /* 创建同步原语 */
    sensor_semaphore = xSemaphoreCreateBinary();
    motor_mutex = xSemaphoreCreateMutex();
    sensor_mutex = xSemaphoreCreateMutex();
    rocket_mutex = xSemaphoreCreateMutex();
    
    if(sensor_semaphore == NULL || motor_mutex == NULL || sensor_mutex == NULL || rocket_mutex == NULL)
    {
        printf("创建同步原语失败！\r\n");
        while(1);
    }
    
    /* 创建任务 */
    if(xTaskCreate(Rocket_Control_Task, 
                   "RocketCtrl", 
                   SENSOR_TASK_STACK_SIZE, 
                   NULL, 
                   ROCKET_TASK_PRIORITY, 
                   NULL) != pdPASS)
    {
        printf("创建摇杆控制任务失败！\r\n");
        while(1);
    }
    
    if(xTaskCreate(Fly_Control_Task, 
                   "FlightCtrl", 
                   CTRL_TASK_STACK_SIZE, 
                   NULL, 
                   CTRL_TASK_PRIORITY, 
                   NULL) != pdPASS)
    {
        printf("创建飞行控制任务失败！\r\n");
        while(1);
    }
    
    if(xTaskCreate(Sensor_Read_Task, 
                   "SensorRead", 
                   SENSOR_TASK_STACK_SIZE, 
                   NULL, 
                   SENSOR_TASK_PRIORITY, 
                   NULL) != pdPASS)
    {
        printf("创建传感器任务失败！\r\n");
        while(1);
    }
    
    if(xTaskCreate(Monitor_Task, 
                   "Monitor", 
                   MONITOR_TASK_STACK_SIZE, 
                   NULL, 
                   MONITOR_TASK_PRIORITY, 
                   NULL) != pdPASS)
    {
        printf("创建监测任务失败！\r\n");
        while(1);
    }
    
    printf("启动FreeRTOS调度器...\r\n");
    
    /* 启动FreeRTOS调度器 */
    vTaskStartScheduler();
    
    /* 如果程序运行到这里说明内存不足 */
    printf("启动失败！\r\n");
    while(1);
}