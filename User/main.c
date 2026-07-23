#include <stdint.h>
#include "stm32f10x.h"      // Device header
#include "Delay.h"
#include "motor.h"
#include "OLED.h"
#include "pid.h"
#include "encoder.h"
#include "lanya.h"
#include "USART3.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "my_robot_usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_tasks.h"

int KeyNum1, KeyNum2, KeyNum3;
PID mypid1, mypid2, mypid3;
int num = 0;

extern float angle_to_radian;
#define L (0.117f)

volatile float Vy_dipan = 0.0f, Vx_dipan = 0.0f, W = 0.0f;
extern float v1_jisuan, v2_jisuan, v3_jisuan;
extern float PWM1, PWM2, PWM3;

volatile float speed_actual1, speed_actual2, speed_actual3;
volatile float speed_capture1, speed_capture2, speed_capture3;

extern u8 lanya_js[6], lanya_i;
extern volatile int f, x, w;
extern volatile int flag;

extern int tuxiang_pian, tuxiang_jiao;
extern int flag_tiao;
extern volatile int flag_stop;
int jiao, pian;

/* Initial speed-loop parameters copied from the proven reference controller. */
float Kp = 4.0f, Ki = 5.0f, Kd = 0.01f, Iout = 50.0f, Out = 1000.0f;

float Pitch, Roll, Yaw;

volatile int16_t second;

extern volatile float Vx_cal, Vy_cal, W_cal;

short testSend1 = 5000;
short testSend2 = 2000;
short testSend3 = 1000;
short testSend4 = 3000;
unsigned char testSend5 = 0x05;

int Vx_receive = 0;
int Vy_receive = 0;
int W_receive = 0;
unsigned char testRece4 = 0x00;

int main(void)
{
    delay_init();

    /* FreeRTOS on Cortex-M works best with all priority bits used for preemption. */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    OLED_Init();
    motor_Init();
    USART2_Config();

    /* MPU6050: PC0=SCL, PC1=SDA; 250 Hz raw sampling after startup calibration. */
    (void)App_IMU_Prepare();

    PID_Init(&mypid1, Kp, Ki, Kd, Iout, Out);
    PID_Init(&mypid2, Kp, Ki, Kd, Iout, Out);
    PID_Init(&mypid3, Kp, Ki, Kd, Iout, Out);

    Vy_dipan = 0.0f;
    Vx_dipan = 0.0f;
    W = 0.0f;

    App_CreateTasks();

    /* Enable ROS USART after the RX queue exists. */
    USART3_Config();

    vTaskStartScheduler();

    while (1)
    {
        motor_Set1(0);
        motor_Set2(0);
        motor_Set3(0);
    }
}

void USART3_IRQHandler(void)
{
    USART3_DMARxIdleHandlerFromISR();
}

void vApplicationMallocFailedHook(void)
{
    motor_Set1(0);
    motor_Set2(0);
    motor_Set3(0);
    taskDISABLE_INTERRUPTS();
    while (1)
    {
    }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;

    motor_Set1(0);
    motor_Set2(0);
    motor_Set3(0);
    taskDISABLE_INTERRUPTS();
    while (1)
    {
    }
}
