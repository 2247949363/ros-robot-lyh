#include "pwm.h"

static uint16_t s_pwmPeriod = 7199U;

static uint16_t PWM_NormalizedToCompare(uint16_t normalized)
{
    uint32_t compare;

    if (normalized > PWM_LOGICAL_MAX)
    {
        normalized = PWM_LOGICAL_MAX;
    }

    compare = ((uint32_t)normalized * ((uint32_t)s_pwmPeriod + 1U)) /
              PWM_LOGICAL_MAX;
    return (uint16_t)compare;
}

void PWM_Init(uint16_t arr, uint16_t prescaler)
{
    GPIO_InitTypeDef gpioInit;
    TIM_TimeBaseInitTypeDef timeBase;
    TIM_OCInitTypeDef outputCompare;

    s_pwmPeriod = arr;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    gpioInit.GPIO_Mode = GPIO_Mode_AF_PP;
    gpioInit.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
    gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpioInit);

    timeBase.TIM_ClockDivision = TIM_CKD_DIV1;
    timeBase.TIM_CounterMode = TIM_CounterMode_Up;
    timeBase.TIM_Period = arr;
    timeBase.TIM_Prescaler = prescaler;
    timeBase.TIM_RepetitionCounter = 0U;
    TIM_TimeBaseInit(TIM1, &timeBase);
    TIM_ARRPreloadConfig(TIM1, ENABLE);

    TIM_OCStructInit(&outputCompare);
    outputCompare.TIM_OCMode = TIM_OCMode_PWM1;
    outputCompare.TIM_OCPolarity = TIM_OCPolarity_High;
    outputCompare.TIM_OutputState = TIM_OutputState_Enable;
    outputCompare.TIM_Pulse = 0U;

    TIM_OC1Init(TIM1, &outputCompare);
    TIM_OC2Init(TIM1, &outputCompare);
    TIM_OC3Init(TIM1, &outputCompare);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_GenerateEvent(TIM1, TIM_EventSource_Update);
    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void PWM_SetCompare1(u16 compare)
{
    TIM_SetCompare1(TIM1, PWM_NormalizedToCompare(compare));
}

void PWM_SetCompare2(u16 compare)
{
    TIM_SetCompare2(TIM1, PWM_NormalizedToCompare(compare));
}

void PWM_SetCompare3(u16 compare)
{
    TIM_SetCompare3(TIM1, PWM_NormalizedToCompare(compare));
}
