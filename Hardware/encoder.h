#ifndef __ENCODER_H
#define __ENCODER_H


void TIM2_encoder_Init(void);
void TIM2_IQHandler(void);
int16_t TIM2_Encoder_Get(void);

void TIM3_encoder_Init(void);
void TIM3_IQHandler(void);
int16_t TIM3_Encoder_Get(void);


void TIM4_encoder_Init(void);
void TIM4_IQHandler(void);
int16_t TIM4_Encoder_Get(void);



#endif
