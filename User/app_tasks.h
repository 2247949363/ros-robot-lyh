#ifndef __APP_TASKS_H
#define __APP_TASKS_H

#include <stdint.h>

#include "FreeRTOS.h"
#include "stm32f10x.h"
#include "task.h"

typedef enum
{
    APP_CMD_SOURCE_NONE = 0,
    APP_CMD_SOURCE_ROS,
    APP_CMD_SOURCE_BT
} AppCmdSource;

extern volatile uint8_t g_imuReady;
extern volatile int16_t g_imuAccelX;
extern volatile int16_t g_imuAccelY;
extern volatile int16_t g_imuAccelZ;
extern volatile int16_t g_imuGyroX;
extern volatile int16_t g_imuGyroY;
extern volatile int16_t g_imuGyroZ;
extern volatile int16_t g_imuTemperatureCentiDeg;
extern volatile uint32_t g_imuSampleCount;
extern volatile uint32_t g_imuReadErrorCount;

/* Call before starting FreeRTOS; performs WHO_AM_I and stationary gyro bias. */
uint8_t App_IMU_Prepare(void);
void App_CreateTasks(void);
void App_RosRxByteFromISR(uint8_t byte);
void App_RosRxBufferFromISR(const uint8_t *data, uint16_t length);

#endif
