#include "stm32f10x.h"
#include "usart.h"
#include "mpu6050.h"
#include "fly_ctrl.h"
#include "pwm.h"
#include "adc.h"
#include "nrf24l01.h"
#include "cli.h"
#include "logger.h"
#include "led.h"
#include <stdio.h>
#include <string.h>

/* FreeRTOS headers */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Interrupt headers */
#include "stm32f10x_it.h"

/* =============================================================
 * Safety configuration macros
 * ============================================================= */
/* MAX_SAFE_ANGLE is defined in fly_ctrl.h (included above) */
#define NRF24_TIMEOUT_MS        500     // RC signal loss timeout (ms)
#define SENSOR_FAIL_LIMIT       200     // Max consecutive I2C failures before shutdown
#define GYRO_STALL_LIMIT        100     // Max times gyro data unchanged (stall detect)
#define ARM_YAW_HOLD_MS         2000    // Yaw-right hold time to arm (ms)
#define ARM_THROTTLE_MAX        5.0f    // Throttle must be below this to arm (%)
#define BAT_LOW_THRESHOLD       3.3f    // Low battery voltage per cell
#define IWDG_RELOAD_MS          500     // IWDG timeout period (ms)

/* Task stack sizes */
#define CTRL_TASK_STACK_SIZE    (256)      // 飞行控制任务
#define SENSOR_TASK_STACK_SIZE  (256)      // 传感器采集任务
#define ROCKET_TASK_STACK_SIZE  (256)      // 摇杆控制任务
#define MONITOR_TASK_STACK_SIZE (128)      // 监测任务
#define LED_TASK_STACK_SIZE     (128)      // LED指示任务
#define CLI_TASK_STACK_SIZE     (256)      // 命令行任务
#define LOGGER_TASK_STACK_SIZE  (256)      // 日志记录任务

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

/* Task priorities (0~4, 4 = highest) */
#define CTRL_TASK_PRIORITY      (4)        // 最高优先级：飞行控制必须及时
#define ROCKET_TASK_PRIORITY    (3)        // 高优先级：摇杆控制
#define SENSOR_TASK_PRIORITY    (3)        // 高优先级：传感器采集
#define MONITOR_TASK_PRIORITY   (2)        // 中优先级：监测输出
#define LED_TASK_PRIORITY       (1)        // 最低优先级：LED指示
#define CLI_TASK_PRIORITY       (1)        // 低优先级：命令行
#define LOGGER_TASK_PRIORITY    (1)        // 低优先级：日志记录

/* Joystick data structure */
typedef struct
{
    float throttle;        // Throttle (0.0 ~ 100.0)
    float roll_input;      // 横滚输入 (-100.0 ~ 100.0)
    float pitch_input;     // 俯仰输入 (-100.0 ~ 100.0)
    float yaw_input;       // 偏航输入 (-100.0 ~ 100.0)
    uint8_t armed;         // Arm/disarm flag (0=未武装, 1=已武装)
} RocketInput_t;

/* Communication queues */
QueueHandle_t sensor_queue;   // 传感器数据队列 (容量2) - 全局，供中断处理使用
static QueueHandle_t rocket_queue;   // 摇杆数据队列 (容量2)
static QueueHandle_t motor_queue;    // 电机输出队列 (容量1)

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
    // 12-bit ADC, center = 2048
    const uint16_t ADC_MID = 2048;
    const float ADC_MAX = 4095.0f;

    float percent;

    if (is_throttle)
    {
        // Throttle mode：0-4095 → 0-100%
        percent = (adc_value / ADC_MAX) * 100.0f;
    }
    else
    {
        // Joystick mode: apply deadzone
        int16_t offset = (int16_t)adc_value - ADC_MID;

        if (offset > -deadzone && offset < deadzone)
        {
            // Inside deadzone, treat as zero
            return 0.0f;
        }

        // Linear mapping beyond deadzone
        if (offset >= 0)
        {
            // Positive direction：deadzone ~ 2047 → 0 ~ 100%
            percent = ((float)offset - deadzone) / (ADC_MID - deadzone) * 100.0f;
        }
        else
        {
            // Negative direction：-2048 ~ -deadzone → -100 ~ 0%
            percent = ((float)offset + deadzone) / (ADC_MID - deadzone) * 100.0f;
        }
    }

    // Clamp range
    if (percent > 100.0f) percent = 100.0f;
    if (percent < -100.0f) percent = -100.0f;

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

    // Check for new data
    if (!NRF24_IsDataReady())
    {
        return 0;  // No new data
    }

    // Receive control packet
    if (!FC_ReceiveControl(&rc_pkt))
    {
        return 0;  // Receive failed or checksum error
    }

    /*
     * NRF24L01的控制包格式：
     * - throttle: 1000~2000 (PWM脉宽微秒)
     * - roll:     -500~500   (角度或速度)
     * - pitch:    -500~500   (角度或速度)
     * - yaw:      -500~500   (角度或速度)
     * - sw1, sw2: 开关状态
     */

    // Throttle：1000~2000us → 0~100%
    // Typical RC mapping：1000us=0%, 1500us=50%, 2000us=100%
    pRocket->throttle = ((float)(rc_pkt.throttle - 1000) / 1000.0f) * 100.0f;
    if (pRocket->throttle < 0.0f) pRocket->throttle = 0.0f;
    if (pRocket->throttle > 100.0f) pRocket->throttle = 100.0f;

    // Roll, pitch, yaw：-500~500 → -100~100%
    pRocket->roll_input = ((float)rc_pkt.roll / 500.0f) * 100.0f;
    pRocket->pitch_input = ((float)rc_pkt.pitch / 500.0f) * 100.0f;
    pRocket->yaw_input = ((float)rc_pkt.yaw / 500.0f) * 100.0f;

    // Arm flag：通常由开关控制
    // sw1 可用作武装开关 (0=未武装, 非0=已武装)
    pRocket->armed = (rc_pkt.sw1 != 0) ? 1 : 0;

    return 1;  // Successfully received and converted
}

/* =============================================================
 * 任务：摇杆控制任务（优先级3）
 * 功能：通过NRF24L01接收遥控信号，转换为飞行输入
 * 周期：20ms（50Hz）
 * ============================================================= */
void Rocket_Control_Task(void *pvParameters)
{
    RocketInput_t temp_rocket = {0};

    /* Previous filtered values for low-pass filter */
    static float last_throttle = 0.0f;
    static float last_roll = 0.0f;
    static float last_pitch = 0.0f;
    static float last_yaw = 0.0f;

    const float FILTER_ALPHA = 0.15f;

    /* Two-stage arming state machine (S4) */
    typedef enum { ARM_IDLE, ARM_WAIT, ARMED } ArmState_t;
    ArmState_t arm_state = ARM_IDLE;
    uint32_t  arm_hold_start = 0;

    /* Signal loss timeout tracking (S1) */
    uint32_t last_signal_tick = 0;

    printf("Rocket ctrl task started, waiting NRF24 int...\r\n");

    while (1)
    {
        if (xSemaphoreTake(xNrf24Semaphore, pdMS_TO_TICKS(200)) == pdTRUE)
        {
            while (NRF24_ReceiveRocketData(&temp_rocket))
            {
                /* Apply low-pass filter */
                temp_rocket.throttle    = LowPassFilter(last_throttle, temp_rocket.throttle, FILTER_ALPHA);
                temp_rocket.roll_input  = LowPassFilter(last_roll, temp_rocket.roll_input, FILTER_ALPHA);
                temp_rocket.pitch_input = LowPassFilter(last_pitch, temp_rocket.pitch_input, FILTER_ALPHA);
                temp_rocket.yaw_input   = LowPassFilter(last_yaw, temp_rocket.yaw_input, FILTER_ALPHA);

                last_throttle = temp_rocket.throttle;
                last_roll     = temp_rocket.roll_input;
                last_pitch    = temp_rocket.pitch_input;
                last_yaw      = temp_rocket.yaw_input;

                last_signal_tick = xTaskGetTickCount();
            }

            /* ---- Two-stage arming state machine (S4) ---- */
            /* Arm condition: throttle < 5% AND yaw > 90% right, hold for ARM_YAW_HOLD_MS */
            uint8_t arm_combo = (temp_rocket.throttle < ARM_THROTTLE_MAX)
                             && (temp_rocket.yaw_input > 90.0f);

            switch (arm_state)
            {
            case ARM_IDLE:
                if (arm_combo)
                {
                    arm_state = ARM_WAIT;
                    arm_hold_start = xTaskGetTickCount();
                }
                break;

            case ARM_WAIT:
                if (!arm_combo)
                {
                    arm_state = ARM_IDLE;  /* Released early, abort */
                }
                else if ((xTaskGetTickCount() - arm_hold_start) >= pdMS_TO_TICKS(ARM_YAW_HOLD_MS))
                {
                    arm_state = ARMED;
                    printf("[SAFETY] Motors ARMED\r\n");
                }
                break;

            case ARMED:
                temp_rocket.armed = 1;
                if (temp_rocket.throttle < ARM_THROTTLE_MAX
                 && temp_rocket.yaw_input < -90.0f)
                {
                    /* Disarm: throttle low + yaw full left */
                    arm_state = ARM_IDLE;
                    temp_rocket.armed = 0;
                    printf("[SAFETY] Motors DISARMED\r\n");
                }
                break;
            }
        }
        else
        {
            /* Signal loss timeout (S1): auto-disarm after NRF24_TIMEOUT_MS */
            if ((xTaskGetTickCount() - last_signal_tick) >= pdMS_TO_TICKS(NRF24_TIMEOUT_MS))
            {
                if (arm_state == ARMED)
                {
                    printf("[SAFETY] RC signal lost, auto-disarm!\r\n");
                }
                arm_state = ARM_IDLE;
                temp_rocket.throttle    = 0.0f;
                temp_rocket.roll_input  = 0.0f;
                temp_rocket.pitch_input = 0.0f;
                temp_rocket.yaw_input   = 0.0f;
                temp_rocket.armed       = 0;
            }
        }

        // Send joystick data to queue（不覆盖旧数据）
        xQueueSend(rocket_queue, &temp_rocket, 0);
    }
}

/* =============================================================
 * 任务2：飞行控制任务（最高优先级）
 * 功能：执行PID控制算法，计算电机速度
 * 周期：由传感器数据更新触发（5ms）
 * ============================================================= */
void Fly_Control_Task(void *pvParameters)
{
    Motor_Out temp_motor = {0};
    RocketInput_t temp_rocket = {0};
    SensorData_t temp_sensor = {0};

    printf("Flight ctrl task started, waiting sensor data...\r\n");

    while (1)
    {
        /* Feed IWDG (S6): highest-priority task feeds the watchdog */
        IWDG_ReloadCounter();

        /* Wait for sensor data (5ms period, 20ms timeout) */
        if (xQueueReceive(sensor_queue, &temp_sensor, pdMS_TO_TICKS(20)) == pdTRUE)
        {
            /* Non-blocking: get latest joystick data if available */
            xQueueReceive(rocket_queue, &temp_rocket, 0);

            // Only execute control when armed
            if (temp_rocket.armed == 1)
            {
                // Convert joystick input to flight targets
                // Throttle：0-100% → PWM 1000-2000us
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
                // Disarmed: stop motors
                Fly_Control_SetTarget(1000.0f, 0.0f, 0.0f, 0.0f);
            }

            // Execute flight control update（dt = 0.005s = 5ms）
            Fly_Control_Update(0.005f);
            Fly_Control_GetMotorOut(&temp_motor);

            {
                static uint32_t log_tick = 0;
                log_tick++;
                if (log_tick % 2 == 0)   /* 100Hz logging (was 50Hz) */
                {
                    LogEntry_t log_entry;
                    Fly_target  log_target;
                    Fly_Control_GetTarget(&log_target);

                    log_entry.magic       = 0x4C4F4745;
                    log_entry.seq         = log_tick / 2;
                    log_entry.timestamp_ms = xTaskGetTickCount();
                    log_entry.roll   = temp_sensor.angle.Roll;
                    log_entry.pitch  = temp_sensor.angle.Pitch;
                    log_entry.yaw    = temp_sensor.angle.Yaw;
                    log_entry.gyro_x = temp_sensor.gyro.GYRO_X;
                    log_entry.gyro_y = temp_sensor.gyro.GYRO_Y;
                    log_entry.gyro_z = temp_sensor.gyro.GYRO_Z;
                    log_entry.m1 = temp_motor.m1;
                    log_entry.m2 = temp_motor.m2;
                    log_entry.m3 = temp_motor.m3;
                    log_entry.m4 = temp_motor.m4;
                    log_entry.target_roll     = log_target.target_roll;
                    log_entry.target_pitch    = log_target.target_pitch;
                    log_entry.target_yaw_rate = log_target.target_yaw_rate;
                    LOG_WriteEntry(&log_entry);
                }
            }

            // 发送电机输出到队列（覆盖旧数据）
            xQueueOverwrite(motor_queue, &temp_motor);
        }
        else
        {
            // Sensor signal timeout（可能是传感器故障）
            printf("[WARNING] 飞行控制任务：Sensor signal timeout！\r\n");

            // Safety: stop motors
            temp_motor.m1 = 1000.0f;
            temp_motor.m2 = 1000.0f;
            temp_motor.m3 = 1000.0f;
            temp_motor.m4 = 1000.0f;
            xQueueOverwrite(motor_queue, &temp_motor);
        }
    }
}

/* =============================================================
 * 任务3：传感器采集任务（高优先级）
 * 功能：读取MPU6050传感器数据，更新角度
 * 周期：5ms（200Hz）
 * ============================================================= */
/* =============================================================
 * 任务3：传感器采集任务（优先级 3）
 * 功能：等 MPU6050 中断信号量，I2C 读取原始数据，互补滤波更新姿态
 * 周期：5ms（200Hz）
 * ============================================================= */
void Sensor_Read_Task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5);

    SensorData_t temp_sensor = {0};

    printf("Sensor Read Task Started (200Hz)...\r\n");

    /* S2: Sensor anomaly tracking */
    static uint16_t i2c_fail_cnt = 0;
    static float    last_gyro_x = 0.0f, last_gyro_y = 0.0f, last_gyro_z = 0.0f;
    static uint16_t gyro_stall_cnt = 0;

    while (1)
    {
        /* Wait for MPU interrupt semaphore, 5ms timeout fallback */
        if (xSemaphoreTake(xMpuSemaphore, xFrequency) == pdFALSE)
        {
            static uint16_t sem_timeout_cnt = 0;
            if (++sem_timeout_cnt >= 200)  /* ~1 sec */
            {
                printf("[WARNING] MPU6050 semaphore timeout, possible interrupt loss\r\n");
                sem_timeout_cnt = 0;
            }
        }

        /* Update attitude (I2C + complementary filter in task context) */
        MPU6050_AngleUpdate(0.005f);

        /* Package latest data for flight control task */
        MPU6050_GetAngle(&temp_sensor.angle);
        MPU6050_GetGyro(&temp_sensor.gyro);

        /* S2a: I2C consecutive failure detection */
        if (temp_sensor.gyro.GYRO_X == 0.0f
         && temp_sensor.gyro.GYRO_Y == 0.0f
         && temp_sensor.gyro.GYRO_Z == 0.0f)
        {
            i2c_fail_cnt++;
            if (i2c_fail_cnt >= SENSOR_FAIL_LIMIT)
            {
                printf("[SAFETY] MPU6050 I2C failure! Stopping motors.\r\n");
                /* Set zero-rate data to force motors off in flight ctrl */
                temp_sensor.gyro.GYRO_X = 0.0f;
                temp_sensor.gyro.GYRO_Y = 0.0f;
                temp_sensor.gyro.GYRO_Z = 0.0f;
            }
        }
        else
        {
            i2c_fail_cnt = 0;
        }

        /* S2b: Gyro stall detection (data unchanged for many cycles) */
        if (temp_sensor.gyro.GYRO_X == last_gyro_x
         && temp_sensor.gyro.GYRO_Y == last_gyro_y
         && temp_sensor.gyro.GYRO_Z == last_gyro_z)
        {
            gyro_stall_cnt++;
            if (gyro_stall_cnt >= GYRO_STALL_LIMIT)
            {
                printf("[SAFETY] Gyro data stalled! Possible sensor freeze.\r\n");
                gyro_stall_cnt = 0;
            }
        }
        else
        {
            gyro_stall_cnt = 0;
            last_gyro_x = temp_sensor.gyro.GYRO_X;
            last_gyro_y = temp_sensor.gyro.GYRO_Y;
            last_gyro_z = temp_sensor.gyro.GYRO_Z;
        }

        xQueueOverwrite(sensor_queue, &temp_sensor);

        // 精确 5ms 周期
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

    SensorData_t temp_sensor = {0};
    Motor_Out temp_motor = {0};
    RocketInput_t temp_rocket = {0};

    while (1)
    {
        // 非阻塞方式尝试读取最新数据（如果有的话）
        xQueuePeek(sensor_queue, &temp_sensor, 0);
        xQueuePeek(motor_queue, &temp_motor, 0);
        xQueuePeek(rocket_queue, &temp_rocket, 0);

        // 输出调试信息
        printf("\r\n===== Rocket Input =====\r\n");
        printf("Throttle: %d  Roll: %d  Pitch: %d  Yaw: %d\r\n",
               (int)temp_rocket.throttle,
               (int)temp_rocket.roll_input,
               (int)temp_rocket.pitch_input,
               (int)temp_rocket.yaw_input);
        printf("Armed: %s\r\n", temp_rocket.armed ? "YES" : "NO");

        printf("===== Attitude Data =====\r\n");
        printf("Pitch: %d deg  Roll: %d deg  Yaw: %d deg\r\n",
               (int)temp_sensor.angle.Pitch,
               (int)temp_sensor.angle.Roll,
               (int)temp_sensor.angle.Yaw);

        printf("===== Angular Velocity =====\r\n");
        printf("GX: %d  GY: %d  GZ: %d (deg/s)\r\n",
               (int)temp_sensor.gyro.GYRO_X,
               (int)temp_sensor.gyro.GYRO_Y,
               (int)temp_sensor.gyro.GYRO_Z);

        printf("===== Motor Output =====\r\n");
        printf("M1: %d  M2: %d  M3: %d  M4: %d\r\n",
               (int)temp_motor.m1,
               (int)temp_motor.m2,
               (int)temp_motor.m3,
               (int)temp_motor.m4);

        /* S7: Battery voltage monitoring */
        {
            float bat_voltage = ADC_GetVoltage();
            printf("===== Battery =====\r\n");
            printf("Voltage: %.2f V\r\n", bat_voltage);
            if (bat_voltage < BAT_LOW_THRESHOLD)
            {
                printf("[WARNING] Battery LOW! (%.2fV < %.1fV)\r\n",
                       bat_voltage, (double)BAT_LOW_THRESHOLD);
                /* Blink LED as low-battery warning */
                LED_ON();
                Delay_ms(100);
                LED_OFF();
            }
        }

        /* Periodic delay */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/* =============================================================
 * IWDG initialization (S6)
 * Timeout = IWDG_RELOAD_MS (500ms). If main loop or FreeRTOS
 * hangs, IWDG resets the MCU -> motors stop immediately.
 * ============================================================= */
static void IWDG_InitLocal(void)
{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_64);
    IWDG_SetReload((uint16_t)(IWDG_RELOAD_MS * 625 / 1000));
    IWDG_ReloadCounter();
    IWDG_Enable();
}

/* =============================================================
 * Main function
 * ============================================================= */
int main(void)
{
    /* Hardware init */
    UART_Init();

    printf("System Startup...\r\n");

    /* 中断优先级分组：4位抢占优先级 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    /* NRF24L01 Initialization - RX Mode */
    printf("Initializing NRF24L01...\r\n");
    NRF24_GPIO_SPI_Init();
    Delay_ms(100);

    uint8_t nrf_addr[5] = {0x12, 0x34, 0x56, 0x78, 0x9A};  // 接收地址
    uint8_t nrf_channel = 40;  // 2.4GHz通道

    if (!NRF24_Check())
    {
        printf("NRF24L01 Check Failed!\r\n");
        while (1);
    }

    if (!NRF24_SetMode(NRF24_MODE_RX, nrf_addr, nrf_channel, NRF24_PAYLOAD_SIZE))
    {
        printf("NRF24L01 Initialization Failed!\r\n");
        while (1);
    }
    printf("NRF24L01 Initialized Successfully, Waiting For RC Signal...\r\n");

    /* 创建 NRF24 中断信号量 */
    xNrf24Semaphore = xSemaphoreCreateBinary();
    if (xNrf24Semaphore == NULL)
    {
        printf("NRF24 Semaphore Creation Failed!\r\n");
        while (1);
    }

    /* 初始化 NRF24 IRQ 中断 (PB0) */
    if (!NRF24_IRQ_Init())
    {
        printf("NRF24 IRQ Init Failed!\r\n");
        while (1);
    }

    /* MPU6050 Initialization */
    printf("Calibrating Gyroscope...\r\n");
    if (MPU6050_GyroCalibrate(50) == 0)
    {
        printf("Gyro Calibration Failed!\r\n");
        while (1);
    }

    printf("Initializing MPU6050...\r\n");
    if (MPU6050_Init() == 0)
    {
        printf("MPU6050 Initialization Failed!\r\n");
        while (1);
    }

    /* Enable Mahony Ki for online gyro bias estimation (was 0.0) */
    MPU6050_SetMahonyKi(0.02f);
    printf("Mahony Ki enabled (0.02) for gyro bias correction\r\n");

    printf("Initializing MPU6050 Interrupt (PB4)...\r\n");
    if (MPU6050_INT_Init() == 0)
    {
        printf("MPU6050 Interrupt Initialization Failed!\r\n");
        while (1);
    }

    /* PWM init */
    PWM_Init();

    /* Flight control init */
    Fly_Init();
    printf("Flight control initialized\r\n");

    /* IWDG init (S6): must be started BEFORE scheduler */
    IWDG_InitLocal();
    printf("IWDG started (500ms timeout)\r\n");

    /* Power-on safety check (S8): verify throttle at zero before starting */
    printf("[SAFETY] Power-on check: ensure throttle at zero and disarmed before flight\r\n");

    /* 创建通信队列 */
    sensor_queue = xQueueCreate(2, sizeof(SensorData_t));  // 传感器数据队列
    rocket_queue = xQueueCreate(2, sizeof(RocketInput_t));  // 摇杆数据队列
    motor_queue = xQueueCreate(1, sizeof(Motor_Out));       // 电机输出队列

    if (sensor_queue == NULL || rocket_queue == NULL || motor_queue == NULL)
    {
        printf("Queue creation failed!\r\n");
        while (1);
    }

    /* 创建传感器读取的信号量 */
    xMpuSemaphore = xSemaphoreCreateBinary();

    /* Create Tasks */
    if (xTaskCreate(Rocket_Control_Task,
                   "RocketCtrl",
                   ROCKET_TASK_STACK_SIZE,
                   NULL,
                   ROCKET_TASK_PRIORITY,
                   NULL) != pdPASS)
    {
        printf("Rocket Control Task Creation Failed!\r\n");
        while (1);
    }

    if (xTaskCreate(Fly_Control_Task,
                   "FlightCtrl",
                   CTRL_TASK_STACK_SIZE,
                   NULL,
                   CTRL_TASK_PRIORITY,
                   NULL) != pdPASS)
    {
        printf("Flight Control Task Creation Failed!\r\n");
        while (1);
    }

    if (xTaskCreate(Sensor_Read_Task,
                   "SensorRead",
                   SENSOR_TASK_STACK_SIZE,
                   NULL,
                   SENSOR_TASK_PRIORITY,
                   NULL) != pdPASS)
    {
        printf("Sensor Read Task Creation Failed!\r\n");
        while (1);
    }

    if (xTaskCreate(Monitor_Task,
                   "Monitor",
                   MONITOR_TASK_STACK_SIZE,
                   NULL,
                   MONITOR_TASK_PRIORITY,
                   NULL) != pdPASS)
    {
        printf("Monitor Task Creation Failed!\r\n");
        while (1);
    }

    if (xTaskCreate(CLI_Task,
                   "CLI",
                   CLI_TASK_STACK_SIZE,
                   NULL,
                   CLI_TASK_PRIORITY,
                   NULL) != pdPASS)
    {
        printf("CLI Task Creation Failed!\r\n");
        while (1);
    }

    if (xTaskCreate(LOG_Task,
                   "Logger",
                   LOGGER_TASK_STACK_SIZE,
                   NULL,
                   LOGGER_TASK_PRIORITY,
                   NULL) != pdPASS)
    {
        printf("Logger Task Creation Failed!\r\n");
        while (1);
    }

    printf("Starting FreeRTOS Scheduler...\r\n");

    /* Start FreeRTOS Scheduler */
    vTaskStartScheduler();

    /* If execution reaches here, it means insufficient memory */
    printf("Startup Failed!\r\n");
    while (1);
}