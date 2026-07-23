#include "app_tasks.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "delay.h"
#include "encoder.h"
#include "motor.h"
#include "mpu6050.h"
#include "my_robot_usart.h"
#include "OLED.h"
#include "pid.h"

#define CONTROL_PERIOD_MS              5U
#define CONTROL_DT_S                   0.005f
#define ROS_TX_PERIOD_MS               20U
#define DISPLAY_PERIOD_MS              100U
#define SAFETY_PERIOD_MS               20U
#define IMU_PERIOD_MS                  4U
#define ROS_CMD_TIMEOUT_MS             300U

#define ROS_RX_QUEUE_LENGTH            256U

#define WHEEL_ACCELERATION_MMPS2       500.0f
#define WHEEL_DECELERATION_MMPS2       1000.0f
#define WHEEL_SPEED_FEEDFORWARD_GAIN   1.24f
#define WHEEL_ZERO_EPSILON_MMPS        0.01f
#define REVERSAL_ZERO_HOLD_MS          20U

#define IMU_GYRO_CALIBRATION_SAMPLES   200U
#define IMU_GYRO_CALIBRATION_MIN_OK    180U

#define CONTROL_TASK_PRIORITY          5U
#define ROS_RX_TASK_PRIORITY           4U
#define SAFETY_TASK_PRIORITY           4U
#define IMU_TASK_PRIORITY              3U
#define ROS_TX_TASK_PRIORITY           2U
#define DISPLAY_TASK_PRIORITY          1U

typedef struct
{
    PID *pid;
    float commandTarget;
    float rampedTarget;
    float pendingTarget;
    uint16_t zeroHoldTicks;
    uint8_t reversing;
} WheelControl;

extern PID mypid1, mypid2, mypid3;
extern int KeyNum1, KeyNum2, KeyNum3;

extern volatile float Vy_dipan, Vx_dipan, W;
extern float v1_jisuan, v2_jisuan, v3_jisuan;

extern volatile float speed_actual1, speed_actual2, speed_actual3;
extern volatile float speed_capture1, speed_capture2, speed_capture3;
extern volatile float Vx_cal, Vy_cal, W_cal;

extern volatile int flag_stop;
extern volatile int flag;
extern volatile int f, x, w;

extern short testSend4;
extern unsigned char testSend5;
extern unsigned char testRece4;

volatile uint8_t g_imuReady;
volatile int16_t g_imuAccelX;
volatile int16_t g_imuAccelY;
volatile int16_t g_imuAccelZ;
volatile int16_t g_imuGyroX;
volatile int16_t g_imuGyroY;
volatile int16_t g_imuGyroZ;
volatile int16_t g_imuTemperatureCentiDeg;
volatile uint32_t g_imuSampleCount;
volatile uint32_t g_imuReadErrorCount;

static long s_gyroBiasX;
static long s_gyroBiasY;
static long s_gyroBiasZ;

static QueueHandle_t s_rosRxQueue;
static volatile TickType_t s_lastRosCmdTick;
static volatile uint8_t s_rosTimeout;
static volatile AppCmdSource s_cmdSource = APP_CMD_SOURCE_NONE;

static WheelControl s_wheel[3];

static void ControlTask(void *argument);
static void RosRxTask(void *argument);
static void RosTxTask(void *argument);
static void DisplayTask(void *argument);
static void SafetyTask(void *argument);
static void ImuTask(void *argument);

static float AbsFloat(float value);
static uint8_t OppositeSigns(float first, float second);
static float SlewRateLimit(float current, float target);
static void WheelControlInit(void);
static void WheelControlSetTarget(WheelControl *wheel, float target);
static int WheelControlUpdate(WheelControl *wheel, float feedback);
static void WheelControlReset(WheelControl *wheel);
static void MotorStopAndResetPid(void);
static int RoundFloatToInt(float value);

uint8_t App_IMU_Prepare(void)
{
    unsigned int i;
    unsigned int validSamples = 0U;
    long sumX = 0L;
    long sumY = 0L;
    long sumZ = 0L;
    short ax;
    short ay;
    short az;
    short temperatureRaw;
    short gx;
    short gy;
    short gz;

    g_imuReady = 0U;
    g_imuSampleCount = 0U;
    g_imuReadErrorCount = 0U;

    if (MPU6050_Init() != 0U)
    {
        return 0U;
    }

    /* The chassis must remain stationary during this approximately 0.8 s step. */
    for (i = 0U; i < IMU_GYRO_CALIBRATION_SAMPLES; i++)
    {
        if (MPU_Get_Raw6Axis(&ax, &ay, &az, &temperatureRaw, &gx, &gy, &gz) == 0U)
        {
            sumX += gx;
            sumY += gy;
            sumZ += gz;
            validSamples++;
        }
        delay_ms(IMU_PERIOD_MS);
    }

    if (validSamples < IMU_GYRO_CALIBRATION_MIN_OK)
    {
        return 0U;
    }

    s_gyroBiasX = sumX / (long)validSamples;
    s_gyroBiasY = sumY / (long)validSamples;
    s_gyroBiasZ = sumZ / (long)validSamples;
    g_imuReady = 1U;
    return 1U;
}

void App_CreateTasks(void)
{
    BaseType_t createResult = pdPASS;

    WheelControlInit();

    s_rosRxQueue = xQueueCreate(ROS_RX_QUEUE_LENGTH, sizeof(uint8_t));
    if (s_rosRxQueue == NULL)
    {
        motor_StopAll();
        while (1) { }
    }

    /* ControlTask is the only owner of normal PWM writes. */
    if (xTaskCreate(ControlTask, "control", 256, NULL, CONTROL_TASK_PRIORITY, NULL) != pdPASS) createResult = pdFAIL;
    if (xTaskCreate(RosRxTask, "ros_rx", 256, NULL, ROS_RX_TASK_PRIORITY, NULL) != pdPASS) createResult = pdFAIL;
    if (xTaskCreate(SafetyTask, "safety", 160, NULL, SAFETY_TASK_PRIORITY, NULL) != pdPASS) createResult = pdFAIL;
    if (xTaskCreate(RosTxTask, "ros_tx", 192, NULL, ROS_TX_TASK_PRIORITY, NULL) != pdPASS) createResult = pdFAIL;
    if (xTaskCreate(DisplayTask, "display", 192, NULL, DISPLAY_TASK_PRIORITY, NULL) != pdPASS) createResult = pdFAIL;

    if (g_imuReady != 0U)
    {
        if (xTaskCreate(ImuTask, "imu", 256, NULL, IMU_TASK_PRIORITY, NULL) != pdPASS) createResult = pdFAIL;
    }

    if (createResult != pdPASS)
    {
        motor_StopAll();
        while (1) { }
    }
}

void App_RosRxByteFromISR(uint8_t byte)
{
    App_RosRxBufferFromISR(&byte, 1U);
}

void App_RosRxBufferFromISR(const uint8_t *data, uint16_t length)
{
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    uint16_t i;

    if ((s_rosRxQueue != NULL) && (data != NULL))
    {
        for (i = 0U; i < length; i++)
        {
            (void)xQueueSendFromISR(s_rosRxQueue, &data[i], &higherPriorityTaskWoken);
        }
        portYIELD_FROM_ISR(higherPriorityTaskWoken);
    }
}

static void ControlTask(void *argument)
{
    TickType_t lastWakeTime = xTaskGetTickCount();

    (void)argument;

    for (;;)
    {
        float vxTarget;
        float vyTarget;
        float wzTarget;
        uint8_t stopNow;
        int16_t delta1;
        int16_t delta2;
        int16_t delta3;
        float measured1;
        float measured2;
        float measured3;

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(CONTROL_PERIOD_MS));

        Encoder_Sample5ms(&delta1,
                          &delta2,
                          &delta3,
                          &measured1,
                          &measured2,
                          &measured3);
        speed_actual1 = measured1;
        speed_actual2 = measured2;
        speed_actual3 = measured3;
        speed_capture1 = (float)delta1;
        speed_capture2 = (float)delta2;
        speed_capture3 = (float)delta3;

        Speed_cal(speed_actual1, speed_actual2, speed_actual3);

        taskENTER_CRITICAL();
        if (flag == 1)
        {
            Vx_dipan = (float)f;
            Vy_dipan = (float)x;
            W = 0.01f * (float)w;
            flag = 0;
            s_cmdSource = APP_CMD_SOURCE_BT;
            s_rosTimeout = 0U;
        }

        stopNow = (uint8_t)((flag_stop == 1) || (s_rosTimeout != 0U));
        vxTarget = Vx_dipan;
        vyTarget = Vy_dipan;
        wzTarget = W;
        taskEXIT_CRITICAL();

        if (stopNow != 0U)
        {
            taskENTER_CRITICAL();
            Vx_dipan = 0.0f;
            Vy_dipan = 0.0f;
            W = 0.0f;
            taskEXIT_CRITICAL();

            /* Emergency/timeout stop deliberately bypasses the target ramp. */
            MotorStopAndResetPid();
            continue;
        }

        Speed_Target(vxTarget, vyTarget, wzTarget);
        WheelControlSetTarget(&s_wheel[0], v1_jisuan);
        WheelControlSetTarget(&s_wheel[1], v2_jisuan);
        WheelControlSetTarget(&s_wheel[2], v3_jisuan);

        KeyNum1 = WheelControlUpdate(&s_wheel[0], speed_actual1);
        KeyNum2 = WheelControlUpdate(&s_wheel[1], speed_actual2);
        KeyNum3 = WheelControlUpdate(&s_wheel[2], speed_actual3);

        motor_Set1(KeyNum1);
        motor_Set2(KeyNum2);
        motor_Set3(KeyNum3);
    }
}

static void RosRxTask(void *argument)
{
    uint8_t byte;
    int vxReceive;
    int vyReceive;
    int wzReceive;
    unsigned char ctrlFlag;

    (void)argument;

    for (;;)
    {
        if (xQueueReceive(s_rosRxQueue, &byte, portMAX_DELAY) == pdPASS)
        {
            if (usartParseOneByte(byte, &vxReceive, &vyReceive, &wzReceive, &ctrlFlag))
            {
                taskENTER_CRITICAL();
                Vx_dipan = (float)vxReceive;
                Vy_dipan = (float)vyReceive;
                W = (float)wzReceive / 1000.0f;
                testRece4 = ctrlFlag;
                s_lastRosCmdTick = xTaskGetTickCount();
                s_rosTimeout = 0U;
                s_cmdSource = APP_CMD_SOURCE_ROS;
                taskEXIT_CRITICAL();
            }
        }
    }
}

static void RosTxTask(void *argument)
{
    TickType_t lastWakeTime = xTaskGetTickCount();

    (void)argument;

    for (;;)
    {
        short vx;
        short vy;
        short wz;

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(ROS_TX_PERIOD_MS));

        taskENTER_CRITICAL();
        vx = (short)Vx_cal;
        vy = (short)Vy_cal;
        wz = (short)(W_cal * 1000.0f);
        taskEXIT_CRITICAL();

        usartSendData(vx, vy, wz, testSend4, testSend5);
    }
}

static void DisplayTask(void *argument)
{
    TickType_t lastWakeTime = xTaskGetTickCount();

    (void)argument;

    for (;;)
    {
        float wheel1;
        float wheel2;
        float wheel3;
        float vx;
        float vy;
        float wz;

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(DISPLAY_PERIOD_MS));

        taskENTER_CRITICAL();
        wheel1 = speed_actual1;
        wheel2 = speed_actual2;
        wheel3 = speed_actual3;
        vx = Vx_cal;
        vy = Vy_cal;
        wz = W_cal;
        taskEXIT_CRITICAL();

        OLED_ShowSignedNum(1, 1, (int)wheel1, 4);
        OLED_ShowSignedNum(2, 1, (int)wheel3, 4);
        OLED_ShowSignedNum(1, 7, (int)wheel2, 4);
        OLED_ShowSignedNum(3, 7, (int)vy, 4);
        OLED_ShowSignedNum(3, 1, (int)vx, 4);
        OLED_ShowSignedNum(4, 1, (int)(wz * 100.0f), 4);
    }
}

static void SafetyTask(void *argument)
{
    TickType_t lastWakeTime = xTaskGetTickCount();

    (void)argument;

    for (;;)
    {
        TickType_t now;
        TickType_t lastRos;
        AppCmdSource source;

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(SAFETY_PERIOD_MS));

        taskENTER_CRITICAL();
        now = xTaskGetTickCount();
        lastRos = s_lastRosCmdTick;
        source = s_cmdSource;
        taskEXIT_CRITICAL();

        if ((source == APP_CMD_SOURCE_ROS) &&
            (lastRos != 0U) &&
            ((now - lastRos) > pdMS_TO_TICKS(ROS_CMD_TIMEOUT_MS)))
        {
            taskENTER_CRITICAL();
            s_rosTimeout = 1U;
            Vx_dipan = 0.0f;
            Vy_dipan = 0.0f;
            W = 0.0f;
            taskEXIT_CRITICAL();
        }
    }
}

static void ImuTask(void *argument)
{
    TickType_t lastWakeTime = xTaskGetTickCount();

    (void)argument;

    for (;;)
    {
        short ax;
        short ay;
        short az;
        short temperatureRaw;
        short gx;
        short gy;
        short gz;

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(IMU_PERIOD_MS));

        if (MPU_Get_Raw6Axis(&ax, &ay, &az, &temperatureRaw, &gx, &gy, &gz) == 0U)
        {
            taskENTER_CRITICAL();
            g_imuAccelX = ax;
            g_imuAccelY = ay;
            g_imuAccelZ = az;
            g_imuGyroX = (short)((long)gx - s_gyroBiasX);
            g_imuGyroY = (short)((long)gy - s_gyroBiasY);
            g_imuGyroZ = (short)((long)gz - s_gyroBiasZ);
            g_imuTemperatureCentiDeg =
                (short)(3653L + ((long)temperatureRaw * 100L) / 340L);
            g_imuSampleCount++;
            taskEXIT_CRITICAL();
        }
        else
        {
            taskENTER_CRITICAL();
            g_imuReadErrorCount++;
            taskEXIT_CRITICAL();
        }
    }
}

static float AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static uint8_t OppositeSigns(float first, float second)
{
    return (uint8_t)(((first > WHEEL_ZERO_EPSILON_MMPS) &&
                      (second < -WHEEL_ZERO_EPSILON_MMPS)) ||
                     ((first < -WHEEL_ZERO_EPSILON_MMPS) &&
                      (second > WHEEL_ZERO_EPSILON_MMPS)));
}

static float SlewRateLimit(float current, float target)
{
    float rate;
    float maximumStep;
    float difference = target - current;
    uint8_t accelerating;

    accelerating = (uint8_t)(((AbsFloat(current) <= WHEEL_ZERO_EPSILON_MMPS) &&
                              (AbsFloat(target) > WHEEL_ZERO_EPSILON_MMPS)) ||
                             ((!OppositeSigns(current, target)) &&
                              (AbsFloat(target) > AbsFloat(current))));

    rate = (accelerating != 0U) ? WHEEL_ACCELERATION_MMPS2 :
                                  WHEEL_DECELERATION_MMPS2;
    maximumStep = rate * CONTROL_DT_S;

    if (difference > maximumStep) return current + maximumStep;
    if (difference < -maximumStep) return current - maximumStep;
    return target;
}

static void WheelControlInit(void)
{
    s_wheel[0].pid = &mypid1;
    s_wheel[1].pid = &mypid2;
    s_wheel[2].pid = &mypid3;
    WheelControlReset(&s_wheel[0]);
    WheelControlReset(&s_wheel[1]);
    WheelControlReset(&s_wheel[2]);
}

static void WheelControlSetTarget(WheelControl *wheel, float target)
{
    if (wheel->reversing != 0U)
    {
        wheel->pendingTarget = target;
        return;
    }

    if (OppositeSigns(wheel->rampedTarget, target) != 0U)
    {
        wheel->reversing = 1U;
        wheel->pendingTarget = target;
        wheel->zeroHoldTicks = REVERSAL_ZERO_HOLD_MS / CONTROL_PERIOD_MS;
        return;
    }

    wheel->commandTarget = target;
}

static int WheelControlUpdate(WheelControl *wheel, float feedback)
{
    float activeTarget;
    float feedforward;
    float output;

    activeTarget = (wheel->reversing != 0U) ? 0.0f : wheel->commandTarget;
    wheel->rampedTarget = SlewRateLimit(wheel->rampedTarget, activeTarget);

    if ((wheel->reversing != 0U) &&
        (AbsFloat(wheel->rampedTarget) <= WHEEL_ZERO_EPSILON_MMPS))
    {
        wheel->rampedTarget = 0.0f;
        PID_Reset(wheel->pid);

        if (wheel->zeroHoldTicks > 0U)
        {
            wheel->zeroHoldTicks--;
            return 0;
        }

        wheel->reversing = 0U;
        wheel->commandTarget = wheel->pendingTarget;
        return 0;
    }

    if ((wheel->reversing == 0U) &&
        (AbsFloat(wheel->commandTarget) <= WHEEL_ZERO_EPSILON_MMPS) &&
        (AbsFloat(wheel->rampedTarget) <= WHEEL_ZERO_EPSILON_MMPS))
    {
        wheel->rampedTarget = 0.0f;
        PID_Reset(wheel->pid);
        return 0;
    }

    feedforward = WHEEL_SPEED_FEEDFORWARD_GAIN * wheel->rampedTarget;
    output = feedforward +
             PID_Calc(wheel->pid, wheel->rampedTarget, feedback, CONTROL_DT_S);
    return RoundFloatToInt(output);
}

static void WheelControlReset(WheelControl *wheel)
{
    wheel->commandTarget = 0.0f;
    wheel->rampedTarget = 0.0f;
    wheel->pendingTarget = 0.0f;
    wheel->zeroHoldTicks = 0U;
    wheel->reversing = 0U;
    PID_Reset(wheel->pid);
}

static void MotorStopAndResetPid(void)
{
    motor_StopAll();
    KeyNum1 = 0;
    KeyNum2 = 0;
    KeyNum3 = 0;
    WheelControlReset(&s_wheel[0]);
    WheelControlReset(&s_wheel[1]);
    WheelControlReset(&s_wheel[2]);
}

static int RoundFloatToInt(float value)
{
    return (value >= 0.0f) ? (int)(value + 0.5f) : (int)(value - 0.5f);
}
