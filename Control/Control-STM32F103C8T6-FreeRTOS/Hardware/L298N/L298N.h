#ifndef _L298N_H
#define _L298N_H
#include "sys.h"  
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

//�����Ҳ��������ӵ���ת�ͷ�ת

void wheels_RL(unsigned int R,unsigned int L);
void wheels_GPIO_Init(void);

#endif



