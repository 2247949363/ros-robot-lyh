#ifndef __PWM_H
#define __PWM_H

#include "stm32f10x.h"

#define PWM_LOGICAL_MAX 1000U

void PWM_Init(uint16_t arr, uint16_t prescaler);
void PWM_SetCompare1(u16 compare);
void PWM_SetCompare2(u16 compare);
void PWM_SetCompare3(u16 compare);

#endif
