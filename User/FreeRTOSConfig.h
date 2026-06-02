#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "system_stm32f10x.h"

/*
 * STM32F103 has 4 implemented NVIC priority bits.
 * ISRs that call FreeRTOS FromISR APIs must use a numerically equal or
 * larger priority than configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY.
 */
#define configPRIO_BITS                              4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY     15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#define configKERNEL_INTERRUPT_PRIORITY             ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY        ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )

#define configUSE_PREEMPTION                        1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION     1
#define configCPU_CLOCK_HZ                          ( ( unsigned long ) SystemCoreClock )
#define configTICK_RATE_HZ                          ( ( TickType_t ) 1000 )
#define configMAX_PRIORITIES                        6
#define configMINIMAL_STACK_SIZE                    ( ( unsigned short ) 128 )
#define configTOTAL_HEAP_SIZE                       ( ( size_t ) ( 12 * 1024 ) )
#define configMAX_TASK_NAME_LEN                     12
#define configUSE_16_BIT_TICKS                      0
#define configIDLE_SHOULD_YIELD                     1
#define configUSE_MUTEXES                           1
#define configQUEUE_REGISTRY_SIZE                   0
#define configCHECK_FOR_STACK_OVERFLOW              2
#define configUSE_MALLOC_FAILED_HOOK                1
#define configUSE_IDLE_HOOK                         0
#define configUSE_TICK_HOOK                         0
#define configUSE_TIMERS                            0
#define configUSE_TICKLESS_IDLE                     0

#define INCLUDE_vTaskPrioritySet                    0
#define INCLUDE_uxTaskPriorityGet                   0
#define INCLUDE_vTaskDelete                         0
#define INCLUDE_vTaskSuspend                        0
#define INCLUDE_vTaskDelayUntil                     1
#define INCLUDE_vTaskDelay                          1
#define INCLUDE_xTaskGetSchedulerState              1

/* Map Cortex-M exception vectors to the FreeRTOS portable layer. */
#define vPortSVCHandler                             SVC_Handler
#define xPortPendSVHandler                          PendSV_Handler
#define xPortSysTickHandler                         SysTick_Handler

#endif
