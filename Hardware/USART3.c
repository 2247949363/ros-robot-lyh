#include <stdint.h>
#include <string.h>
#include "stm32f10x.h"
#include "USART3.h"
#include "app_tasks.h"

static uint8_t s_usart3RxDmaBuffer[USART3_DMA_RX_BUFFER_SIZE];
static uint16_t s_usart3RxLastPos;
static uint8_t s_usart3TxDmaBuffer[USART3_DMA_TX_BUFFER_SIZE];
static volatile uint8_t s_usart3TxBusy;

static void USART3_GPIO_Config(void);
static void USART3_DMA_Config(void);
static void USART3_NVIC_Config(void);
static void USART3_ProcessDmaRxRangeFromISR(uint16_t start, uint16_t end);

void USART3_Config(void)
{
    USART_InitTypeDef USART_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    USART3_GPIO_Config();

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);

    USART3_DMA_Config();
    USART3_NVIC_Config();

    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
    USART_Cmd(USART3, ENABLE);
}

void USART3_DMARxIdleHandlerFromISR(void)
{
    if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
    {
        volatile uint16_t dummy;
        uint16_t currentPos;

        /* Clear IDLE: read SR first, then DR. */
        dummy = USART3->SR;
        dummy = USART3->DR;
        (void)dummy;

        currentPos = (uint16_t)(USART3_DMA_RX_BUFFER_SIZE - DMA_GetCurrDataCounter(DMA1_Channel3));
        if (currentPos >= USART3_DMA_RX_BUFFER_SIZE)
        {
            currentPos = 0U;
        }

        if (currentPos != s_usart3RxLastPos)
        {
            if (currentPos > s_usart3RxLastPos)
            {
                USART3_ProcessDmaRxRangeFromISR(s_usart3RxLastPos, currentPos);
            }
            else
            {
                USART3_ProcessDmaRxRangeFromISR(s_usart3RxLastPos, USART3_DMA_RX_BUFFER_SIZE);
                if (currentPos > 0U)
                {
                    USART3_ProcessDmaRxRangeFromISR(0U, currentPos);
                }
            }

            s_usart3RxLastPos = currentPos;
        }
    }
}

uint8_t USART3_SendBytesDMA(const uint8_t *data, uint16_t length)
{
    DMA_InitTypeDef DMA_InitStructure;
    uint8_t accepted = 0U;

    if ((data == NULL) || (length == 0U) || (length > USART3_DMA_TX_BUFFER_SIZE))
    {
        return 0U;
    }

    __disable_irq();
    if (s_usart3TxBusy == 0U)
    {
        s_usart3TxBusy = 1U;
        accepted = 1U;
    }
    __enable_irq();

    if (accepted == 0U)
    {
        return 0U;
    }

    memcpy(s_usart3TxDmaBuffer, data, length);

    DMA_Cmd(DMA1_Channel2, DISABLE);
    USART_DMACmd(USART3, USART_DMAReq_Tx, DISABLE);
    DMA_DeInit(DMA1_Channel2);
    DMA_ClearFlag(DMA1_FLAG_GL2 | DMA1_FLAG_TC2 | DMA1_FLAG_HT2 | DMA1_FLAG_TE2);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)s_usart3TxDmaBuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = length;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel2, &DMA_InitStructure);

    DMA_ITConfig(DMA1_Channel2, DMA_IT_TC | DMA_IT_TE, ENABLE);
    USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
    DMA_Cmd(DMA1_Channel2, ENABLE);

    return 1U;
}

uint8_t USART3_IsTxBusy(void)
{
    return s_usart3TxBusy;
}

void DMA1_Channel2_IRQHandler(void)
{
    if (DMA_GetITStatus(DMA1_IT_TC2) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_IT_TC2);
        DMA_Cmd(DMA1_Channel2, DISABLE);
        USART_DMACmd(USART3, USART_DMAReq_Tx, DISABLE);
        s_usart3TxBusy = 0U;
    }

    if (DMA_GetITStatus(DMA1_IT_TE2) != RESET)
    {
        DMA_ClearITPendingBit(DMA1_IT_TE2);
        DMA_Cmd(DMA1_Channel2, DISABLE);
        USART_DMACmd(USART3, USART_DMAReq_Tx, DISABLE);
        s_usart3TxBusy = 0U;
    }
}

static void USART3_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

static void USART3_DMA_Config(void)
{
    DMA_InitTypeDef DMA_InitStructure;

    DMA_DeInit(DMA1_Channel3);
    DMA_ClearFlag(DMA1_FLAG_GL3 | DMA1_FLAG_TC3 | DMA1_FLAG_HT3 | DMA1_FLAG_TE3);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)s_usart3RxDmaBuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = USART3_DMA_RX_BUFFER_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel3, &DMA_InitStructure);
    DMA_Cmd(DMA1_Channel3, ENABLE);

    s_usart3RxLastPos = 0U;
    s_usart3TxBusy = 0U;
}

static void USART3_NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

static void USART3_ProcessDmaRxRangeFromISR(uint16_t start, uint16_t end)
{
    if (end > start)
    {
        App_RosRxBufferFromISR(&s_usart3RxDmaBuffer[start], (uint16_t)(end - start));
    }
}

/* Legacy variables retained for older vision-debug code that referenced USART3.c. */
u8 tuxiang_js[8];
u8 tuxiang_i;
int tuxiang_pian = 0;
int tuxiang_jiao = 0;
int tuxiang_t = 0;
int flag_tiao = 0;
