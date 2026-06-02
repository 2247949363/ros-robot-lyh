#ifndef __USART3_H
#define __USART3_H

#include <stdint.h>
#include "stm32f10x.h"

#define USART3_DMA_RX_BUFFER_SIZE 128U
#define USART3_DMA_TX_BUFFER_SIZE 64U

void USART3_Config(void);
void USART3_DMARxIdleHandlerFromISR(void);
uint8_t USART3_SendBytesDMA(const uint8_t *data, uint16_t length);
uint8_t USART3_IsTxBusy(void);

#endif
