#ifndef __APP_TASKS_H
#define __APP_TASKS_H

#include <stdint.h>
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"

typedef enum
{
    APP_CMD_SOURCE_NONE = 0,
    APP_CMD_SOURCE_ROS,
    APP_CMD_SOURCE_BT
} AppCmdSource;

void App_CreateTasks(void);
void App_RosRxByteFromISR(uint8_t byte);
void App_RosRxBufferFromISR(const uint8_t *data, uint16_t length);

#endif

