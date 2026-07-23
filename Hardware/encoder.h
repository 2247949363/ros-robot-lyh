#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"

/* Parameters of the 11-line, 30:1 geared motor used by the reference robot. */
#define ENCODER_LINE_COUNT          11.0f
#define ENCODER_QUADRATURE_FACTOR   4.0f
#define MOTOR_REDUCTION_RATIO       30.0f
#define WHEEL_DIAMETER_MM           65.0f
#define ENCODER_SPEED_WINDOW_SIZE   2U

void TIM2_encoder_Init(void);
void TIM3_encoder_Init(void);
void TIM4_encoder_Init(void);

/* Legacy single-counter accessors. They no longer clear the hardware counter. */
int16_t TIM2_Encoder_Get(void);
int16_t TIM3_Encoder_Get(void);
int16_t TIM4_Encoder_Get(void);

/*
 * Capture all three free-running counters every 5 ms. The returned speed is
 * the moving average over the latest 2 samples (10 ms window), in mm/s.
 */
void Encoder_ResetSpeedFilter(void);
void Encoder_Sample5ms(int16_t *delta1,
                       int16_t *delta2,
                       int16_t *delta3,
                       float *speed1Mmps,
                       float *speed2Mmps,
                       float *speed3Mmps);

#endif
