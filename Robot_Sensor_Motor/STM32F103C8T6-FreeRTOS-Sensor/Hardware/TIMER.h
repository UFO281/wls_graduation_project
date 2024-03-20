/**
 * @file TIMER.h
 * @author wls
 * @brief Timer Config
 * @version 0.1
 * @date 2022-11-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef TIMER_H
#define TIMER_H

#include "stm32f10x.h"
#include "EXTI.h"

extern uint16_t R_V,L_V;//�ⲿ�жϲ�����������ؼ���ֵ

extern uint16_t RR_Value,LL_Value;



void Timer1_Counting_Init(void);

void Timer2_Timing_Init(uint16_t T_Count,uint16_t PSC);//TIM2 ���ó� ��ʱ����

void Timer2_Counting_Init(void);//TIM2 ���óɼ�������

uint16_t Get_TIM2_Counter_Value(void);

void Timer3_Timing_Init(uint16_t T_Count,uint16_t PSC);

void Timer3_PWM_Init(uint16_t T_Count,uint16_t PSC);

void TIM3_PWM_SetCompare1_2(uint16_t Compare1,uint16_t Compare2);

void Timer4_Timing_Init(uint16_t T_Count,uint16_t PSC);


void ConfigureTimeForRunTimeStats(void);

extern unsigned long long FreeRTOSRunTimeTicks;


#endif

#if 0 

//-----------------TIM1 TIM2 �ӵ��������������٣�TIM4��ʱ100ms ����ת��-----------

#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Timer.h"



int main(void)
{
	OLED_Init();
    Timer1_Counting_Init();//PA12    
	//Timer2_Timing_Init(10000,7200);//72MHZ ����7200��Ƶ Ƶ��Ϊ10KHZ ������������Ϊ100us ��ʱ���ж�ʱ��100*100us=10ms
	Timer2_Counting_Init();//PA15

    //Timer3_Timing_Init(10000,7200);//72MHZ ����7200��Ƶ Ƶ��Ϊ10KHZ ������������Ϊ100us ��ʱ���ж�ʱ��100*100us=10ms

    Timer4_Timing_Init(1000,7200);//72MHZ ����7200��Ƶ Ƶ��Ϊ10KHZ ������������Ϊ100us ��ʱ���ж�ʱ��100*100us=10ms
	
    OLED_ShowString(1, 1, "R_n:");
	OLED_ShowString(2, 1, "L_n:");	
    OLED_ShowString(3, 1, "Num:");
 
	while (1)
	{
		
		OLED_ShowNum(1, 6, R_n, 5);// TIM1 PA12 �� R����������ת�� ��λms/Ȧ ��ü�ֵΪ12ms/R 83R/s
        OLED_ShowNum(2, 6,L_n,  5);// TIM2 PA15 ��L����������ת�� ��λms/Ȧ ��ü�ֵΪ12ms/R 83R/s
        OLED_ShowNum(3, 6,Num,  5);//TIM4 ��ʱ���ж����ֵ 100ms��һ���ж�

	}
}

//-----------------TIM1 TIM2 �ӵ��������������٣�TIM4��ʱ100ms ����ת��-----------


//-----------------TIM1 TIM2 �ӵ��������������٣�TIM3_PWM_MODE,TIM4��ʱ100ms ����ת��-----------

#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Timer.h"



int main(void)
{

	OLED_Init();
    Timer1_Counting_Init();//PA12    
	//Timer2_Timing_Init(10000,7200);//72MHZ ����7200��Ƶ Ƶ��Ϊ10KHZ ������������Ϊ100us ��ʱ���ж�ʱ��100*100us=10ms
	Timer2_Counting_Init();//PA15

    //Timer3_Timing_Init(10000,7200);//72MHZ ����7200��Ƶ Ƶ��Ϊ10KHZ ������������Ϊ100us ��ʱ���ж�ʱ��100*100us=10ms

    Timer3_PWM_Init(1000,72);// 1KHZpwm PWM���ֵΪ1000 CH1PA6 CH2PA7
    
    Timer4_Timing_Init(1000,7200);//72MHZ ����7200��Ƶ Ƶ��Ϊ10KHZ ������������Ϊ100us ��ʱ���ж�ʱ��100*100us=10ms
	
    OLED_ShowString(1, 1, "R_n:");
	OLED_ShowString(2, 1, "L_n:");	
    OLED_ShowString(3, 1, "Num:");
 
	while (1)
	{
		
		OLED_ShowNum(1, 6, R_n, 5);// TIM1 PA12 �� R����������ת�� ��λms/Ȧ ��ü�ֵΪ12ms/R 83R/s
        OLED_ShowNum(2, 6,L_n,  5);// TIM2 PA15 ��L����������ת�� ��λms/Ȧ ��ü�ֵΪ12ms/R 83R/s
        OLED_ShowNum(3, 6,Num,  5);//TIM4 ��ʱ���ж����ֵ 100ms��һ���ж�
        TIM3_PWM_SetCompare1_2(Num,Num);
        Num%=1200;
	}
}


//-----------------TIM1 TIM2 �ӵ��������������٣�TIM3_PWM_MODE,TIM4��ʱ100ms ����ת��-----------




//-----------------------EXTI+Timer��ʱ�������������------------------------

#include "stm32f10x.h"     // Device header
#include "Delay.h"
#include "OLED.h"
#include "Timer.h"
#include "TB6612FNG.h"
#include "EXTI.h"




/**
 * @brief ������
 * 
 * @return int 
 */
int main(void)
{
    int N=0;
    Extern_Interrupt_Init();//PA12 PA15
    Drive_Motor_GPIO_Init();
	OLED_Init();
    //Timer1_Counting_Init();//PA12    
	//Timer2_Timing_Init(10000,7200);//72MHZ ����7200��Ƶ Ƶ��Ϊ10KHZ ������������Ϊ100us ��ʱ���ж�ʱ��100*100us=10ms
	//Timer2_Counting_Init();//PA15

    //Timer3_Timing_Init(10000,7200);//72MHZ ����7200��Ƶ Ƶ��Ϊ10KHZ ������������Ϊ100us ��ʱ���ж�ʱ��100*100us=10ms

    Timer3_PWM_Init(2000,0);// 2KHZpwm PWM���ֵΪ1000 CH1PA6 CH2PA7
    
    Timer4_Timing_Init(1000,7200);//72MHZ ����7200��Ƶ Ƶ��Ϊ10KHZ ������������Ϊ100us ��ʱ���ж�ʱ��100*100us=10ms
	
    OLED_ShowString(1, 1, "R_n:");
	OLED_ShowString(2, 1, "L_n:");	
    OLED_ShowString(3, 1, "Num:");
    Drive_Motor_R_L(0,1);
 
	while (1)
	{
		OLED_ShowNum(1, 6,RR_Value, 5);// TIM1 PA12 �� R����������ת�� ��λms/Ȧ ��ü�ֵΪ12ms/R 83R/s
        OLED_ShowNum(2, 6,LL_Value,  5);// TIM2 PA15 ��L����������ת�� ��λms/Ȧ ��ü�ֵΪ12ms/R 83R/s
        OLED_ShowNum(3, 6,N,  5);//TIM4 ��ʱ���ж����ֵ 100ms��һ���ж�
        TIM3_PWM_SetCompare1_2(N,N);
        N++;
        N%=2500;
	}
}

//-----------------------EXTI+Timer��ʱ�������������------------------------



#endif




