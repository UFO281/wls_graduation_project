/**
 * @file TB6612FNG.c
 * @author wls
 * @brief Drive motor chip
 * @version 0.1
 * @date 2022-10-19
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef _TB6612FNG_H
#define _TB6612FNG_H

#include "stm32f10x.h"     // Device header

void Drive_Motor_R_L(unsigned int R,unsigned int L);

void Drive_Motor_GPIO_Init(void);

#endif



#if 0

Drive_Motor_GPIO_Init();;// Init drive motor IC GPIO

Drive_Motor_GPIO_Init(0,1);

#endif

