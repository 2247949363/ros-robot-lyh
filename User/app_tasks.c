#include "app_tasks.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "encoder.h"
#include "motor.h"
#include "my_robot_usart.h"
#include "OLED.h"
#include "pid.h"

#define CONTROL_PERIOD_MS       5U
#define ROS_TX_PERIOD_MS        20U
#define DISPLAY_PERIOD_MS       100U
#define SAFETY_PERIOD_MS        20U
#define ROS_CMD_TIMEOUT_MS      300U

#define ROS_RX_QUEUE_LENGTH     256U

#define WHEEL_RADIUS_M          0.029f
#define ENCODER_LINE_COUNT      13.0f
#define MOTOR_REDUCTION_RATIO   74.8f
#define PI_F                    3.1416f

#define SPEED_FILTER_LOW_THRESHOLD_MMPS 180.0f
#define SPEED_FILTER_LOW_ALPHA          0.25f
#define SPEED_FILTER_HIGH_ALPHA         0.65f
#define SPEED_FILTER_ZERO_THRESHOLD_MMPS 1.0f

#define CONTROL_TASK_PRIORITY   5U
#define ROS_RX_TASK_PRIORITY    4U
#define SAFETY_TASK_PRIORITY    4U
#define ROS_TX_TASK_PRIORITY    2U
#define DISPLAY_TASK_PRIORITY   1U

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

static QueueHandle_t s_rosRxQueue;
static volatile TickType_t s_lastRosCmdTick;
static volatile uint8_t s_rosTimeout;
static volatile AppCmdSource s_cmdSource = APP_CMD_SOURCE_NONE;
static float s_filteredWheelSpeed1;
static float s_filteredWheelSpeed2;
static float s_filteredWheelSpeed3;

static void ControlTask(void *argument);
static void RosRxTask(void *argument);
static void RosTxTask(void *argument);
static void DisplayTask(void *argument);
static void SafetyTask(void *argument);
static float EncoderCountToSpeedMmps(int16_t count, float sampleTimeSec);
static float AbsFloat(float value);
static float WheelSpeedFilterUpdate(float filteredSpeed, float rawSpeed, float targetSpeed);
static void WheelSpeedFilterReset(void);
static void MotorStopAndResetPid(void);

void App_CreateTasks(void)
{
    s_rosRxQueue = xQueueCreate(ROS_RX_QUEUE_LENGTH, sizeof(uint8_t));
    if (s_rosRxQueue == NULL)
    {
        while (1)
        {
        }
    }

    /*
     * ControlTask owns the motor PWM output. Other tasks only update command
     * state, so motor writes have one clear owner and the 5 ms loop stays stable.
     */
    xTaskCreate(ControlTask, "control", 256, NULL, CONTROL_TASK_PRIORITY, NULL);
    xTaskCreate(RosRxTask, "ros_rx", 256, NULL, ROS_RX_TASK_PRIORITY, NULL);
    xTaskCreate(SafetyTask, "safety", 160, NULL, SAFETY_TASK_PRIORITY, NULL);
    xTaskCreate(RosTxTask, "ros_tx", 192, NULL, ROS_TX_TASK_PRIORITY, NULL);
    xTaskCreate(DisplayTask, "display", 192, NULL, DISPLAY_TASK_PRIORITY, NULL);
}

void App_RosRxByteFromISR(uint8_t byte)
{
    App_RosRxBufferFromISR(&byte, 1U);
}

void App_RosRxBufferFromISR(const uint8_t *data, uint16_t length)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint16_t i;

    if ((s_rosRxQueue != NULL) && (data != NULL))
    {
        for (i = 0; i < length; i++)
        {
            (void)xQueueSendFromISR(s_rosRxQueue, &data[i], &xHigherPriorityTaskWoken);
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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
        float rawSpeed1;
        float rawSpeed2;
        float rawSpeed3;
        uint8_t stopNow;

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(CONTROL_PERIOD_MS));

        speed_capture1 = TIM2_Encoder_Get();
        speed_capture2 = TIM3_Encoder_Get();
        speed_capture3 = TIM4_Encoder_Get();

        rawSpeed1 = EncoderCountToSpeedMmps((int16_t)speed_capture1, CONTROL_PERIOD_MS / 1000.0f);
        rawSpeed2 = EncoderCountToSpeedMmps((int16_t)speed_capture2, CONTROL_PERIOD_MS / 1000.0f);
        rawSpeed3 = EncoderCountToSpeedMmps((int16_t)speed_capture3, CONTROL_PERIOD_MS / 1000.0f);

        taskENTER_CRITICAL();
        if (flag == 1)
        {
            Vx_dipan = (float)f;
            Vy_dipan = (float)x;
            W = 0.01f * (float)w;
            flag = 0;
            s_cmdSource = APP_CMD_SOURCE_BT;
            s_rosTimeout = 0;
        }

        stopNow = (uint8_t)((flag_stop == 1) || (s_rosTimeout != 0));
        vxTarget = Vx_dipan;
        vyTarget = Vy_dipan;
        wzTarget = W;
        taskEXIT_CRITICAL();

        if (stopNow)
        {
            taskENTER_CRITICAL();
            Vx_dipan = 0.0f;
            Vy_dipan = 0.0f;
            W = 0.0f;
            taskEXIT_CRITICAL();

            MotorStopAndResetPid();
            continue;
        }

        Speed_Target(vxTarget, vyTarget, wzTarget);

        s_filteredWheelSpeed1 = WheelSpeedFilterUpdate(s_filteredWheelSpeed1, rawSpeed1, v1_jisuan);
        s_filteredWheelSpeed2 = WheelSpeedFilterUpdate(s_filteredWheelSpeed2, rawSpeed2, v2_jisuan);
        s_filteredWheelSpeed3 = WheelSpeedFilterUpdate(s_filteredWheelSpeed3, rawSpeed3, v3_jisuan);

        speed_actual1 = s_filteredWheelSpeed1;
        speed_actual2 = s_filteredWheelSpeed2;
        speed_actual3 = s_filteredWheelSpeed3;

        Speed_cal(speed_actual1, speed_actual2, speed_actual3);

        PID_Calc(&mypid1, v1_jisuan, speed_actual1);
        KeyNum1 = (int)mypid1.output;

        PID_Calc(&mypid2, v2_jisuan, speed_actual2);
        KeyNum2 = (int)mypid2.output;

        PID_Calc(&mypid3, v3_jisuan, speed_actual3);
        KeyNum3 = (int)mypid3.output;

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
                s_rosTimeout = 0;
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
            s_rosTimeout = 1;
            Vx_dipan = 0.0f;
            Vy_dipan = 0.0f;
            W = 0.0f;
            taskEXIT_CRITICAL();
        }
    }
}

static float EncoderCountToSpeedMmps(int16_t count, float sampleTimeSec)
{
    float countsPerRev = ENCODER_LINE_COUNT * MOTOR_REDUCTION_RATIO;
    float wheelCircumferenceM = 2.0f * PI_F * WHEEL_RADIUS_M;

    return ((float)count / sampleTimeSec) / countsPerRev * wheelCircumferenceM * 1000.0f;
}

static float AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static float WheelSpeedFilterUpdate(float filteredSpeed, float rawSpeed, float targetSpeed)
{
    float rawAbs = AbsFloat(rawSpeed);
    float targetAbs = AbsFloat(targetSpeed);
    float alpha = SPEED_FILTER_HIGH_ALPHA;

    if ((rawAbs < SPEED_FILTER_ZERO_THRESHOLD_MMPS) &&
        (targetAbs < SPEED_FILTER_ZERO_THRESHOLD_MMPS))
    {
        return 0.0f;
    }

    if ((rawAbs < SPEED_FILTER_LOW_THRESHOLD_MMPS) &&
        (targetAbs < SPEED_FILTER_LOW_THRESHOLD_MMPS))
    {
        alpha = SPEED_FILTER_LOW_ALPHA;
    }

    return filteredSpeed + alpha * (rawSpeed - filteredSpeed);
}

static void WheelSpeedFilterReset(void)
{
    s_filteredWheelSpeed1 = 0.0f;
    s_filteredWheelSpeed2 = 0.0f;
    s_filteredWheelSpeed3 = 0.0f;

    speed_actual1 = 0.0f;
    speed_actual2 = 0.0f;
    speed_actual3 = 0.0f;
    Speed_cal(0.0f, 0.0f, 0.0f);
}

static void MotorStopAndResetPid(void)
{
    motor_Set1(0);
    motor_Set2(0);
    motor_Set3(0);

    KeyNum1 = 0;
    KeyNum2 = 0;
    KeyNum3 = 0;

    PID_Reset(&mypid1);
    PID_Reset(&mypid2);
    PID_Reset(&mypid3);

    WheelSpeedFilterReset();
}
