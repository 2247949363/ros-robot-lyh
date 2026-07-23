#include "encoder.h"

#include <string.h>

#define PI_F                       3.1415926f
#define ENCODER_SAMPLE_TIME_S      0.005f
#define ENCODER_WINDOW_TIME_S      (ENCODER_SAMPLE_TIME_S * ENCODER_SPEED_WINDOW_SIZE)
#define ENCODER_COUNTS_PER_REV     (ENCODER_LINE_COUNT * ENCODER_QUADRATURE_FACTOR * MOTOR_REDUCTION_RATIO)
#define WHEEL_MM_PER_COUNT         ((PI_F * WHEEL_DIAMETER_MM) / ENCODER_COUNTS_PER_REV)

static int16_t s_lastCount[3];
static int16_t s_deltaHistory[3][ENCODER_SPEED_WINDOW_SIZE];
static int32_t s_deltaSum[3];
static uint8_t s_historyIndex;

static void EncoderTimerInit(TIM_TypeDef *tim,
                             uint32_t timClock,
                             GPIO_TypeDef *gpio,
                             uint32_t gpioClock,
                             uint16_t pins)
{
    GPIO_InitTypeDef gpioInit;
    TIM_TimeBaseInitTypeDef timeBase;
    TIM_ICInitTypeDef inputCapture;

    RCC_APB1PeriphClockCmd(timClock, ENABLE);
    RCC_APB2PeriphClockCmd(gpioClock, ENABLE);

    gpioInit.GPIO_Mode = GPIO_Mode_IPU;
    gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
    gpioInit.GPIO_Pin = pins;
    GPIO_Init(gpio, &gpioInit);

    timeBase.TIM_ClockDivision = TIM_CKD_DIV1;
    timeBase.TIM_CounterMode = TIM_CounterMode_Up;
    timeBase.TIM_Period = 0xFFFFU;
    timeBase.TIM_Prescaler = 0U;
    timeBase.TIM_RepetitionCounter = 0U;
    TIM_TimeBaseInit(tim, &timeBase);

    TIM_ICStructInit(&inputCapture);
    inputCapture.TIM_Channel = TIM_Channel_1;
    inputCapture.TIM_ICFilter = 0x0FU;
    TIM_ICInit(tim, &inputCapture);
    inputCapture.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(tim, &inputCapture);

    TIM_EncoderInterfaceConfig(tim,
                               TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising,
                               TIM_ICPolarity_Rising);
    TIM_SetCounter(tim, 0U);
    TIM_Cmd(tim, ENABLE);
}

void TIM2_encoder_Init(void)
{
    EncoderTimerInit(TIM2,
                     RCC_APB1Periph_TIM2,
                     GPIOA,
                     RCC_APB2Periph_GPIOA,
                     GPIO_Pin_0 | GPIO_Pin_1);
}

void TIM3_encoder_Init(void)
{
    EncoderTimerInit(TIM3,
                     RCC_APB1Periph_TIM3,
                     GPIOA,
                     RCC_APB2Periph_GPIOA,
                     GPIO_Pin_6 | GPIO_Pin_7);
}

void TIM4_encoder_Init(void)
{
    EncoderTimerInit(TIM4,
                     RCC_APB1Periph_TIM4,
                     GPIOB,
                     RCC_APB2Periph_GPIOB,
                     GPIO_Pin_6 | GPIO_Pin_7);
}

int16_t TIM2_Encoder_Get(void)
{
    return (int16_t)TIM_GetCounter(TIM2);
}

int16_t TIM3_Encoder_Get(void)
{
    return (int16_t)TIM_GetCounter(TIM3);
}

int16_t TIM4_Encoder_Get(void)
{
    return (int16_t)TIM_GetCounter(TIM4);
}

void Encoder_ResetSpeedFilter(void)
{
    memset(s_deltaHistory, 0, sizeof(s_deltaHistory));
    memset(s_deltaSum, 0, sizeof(s_deltaSum));
    s_historyIndex = 0U;
    s_lastCount[0] = (int16_t)TIM_GetCounter(TIM2);
    s_lastCount[1] = (int16_t)TIM_GetCounter(TIM3);
    s_lastCount[2] = (int16_t)TIM_GetCounter(TIM4);
}

void Encoder_Sample5ms(int16_t *delta1,
                       int16_t *delta2,
                       int16_t *delta3,
                       float *speed1Mmps,
                       float *speed2Mmps,
                       float *speed3Mmps)
{
    int16_t now[3];
    int16_t delta[3];
    uint8_t i;

    now[0] = (int16_t)TIM_GetCounter(TIM2);
    now[1] = (int16_t)TIM_GetCounter(TIM3);
    now[2] = (int16_t)TIM_GetCounter(TIM4);

    for (i = 0U; i < 3U; i++)
    {
        /* Signed subtraction remains correct when the 16-bit counter wraps. */
        delta[i] = (int16_t)(now[i] - s_lastCount[i]);
        s_lastCount[i] = now[i];

        s_deltaSum[i] -= s_deltaHistory[i][s_historyIndex];
        s_deltaHistory[i][s_historyIndex] = delta[i];
        s_deltaSum[i] += delta[i];
    }

    s_historyIndex++;
    if (s_historyIndex >= ENCODER_SPEED_WINDOW_SIZE)
    {
        s_historyIndex = 0U;
    }

    if (delta1 != 0) *delta1 = delta[0];
    if (delta2 != 0) *delta2 = delta[1];
    if (delta3 != 0) *delta3 = delta[2];

    if (speed1Mmps != 0) *speed1Mmps = ((float)s_deltaSum[0] * WHEEL_MM_PER_COUNT) / ENCODER_WINDOW_TIME_S;
    if (speed2Mmps != 0) *speed2Mmps = ((float)s_deltaSum[1] * WHEEL_MM_PER_COUNT) / ENCODER_WINDOW_TIME_S;
    if (speed3Mmps != 0) *speed3Mmps = ((float)s_deltaSum[2] * WHEEL_MM_PER_COUNT) / ENCODER_WINDOW_TIME_S;
}
