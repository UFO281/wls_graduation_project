/**
 * @file ADC.h
 * @author wls
 * @brief ADC example
 * STM32F103C8T6 ADC chanel
 * 
    chanel    ADC1    ADC2    ADC3 (C8T6 haven't)
    chanel_0   PA0     PA0     PA0
    chanel_1   PA1     PA1     PA1
    chanel_2   PA2     PA2     PA2
    chanel_3   PA3     PA3     PA3
    chanel_4   PA4     PA4     PF6
    chanel_5   PA5     PA5     PF7
    chanel_6   PA6     PA6     PF8
    chanel_7   PA7     PA7     PF9
    chanel_8   PB0     PB0     PF10
    chanel_9   PB1     PB1
    chanel_10  PC0     PC0     PC0
    chanel_11  PC1     PC1     PC1
    chanel_12  PC2     PC2     PC2
    chanel_13  PC3     PC3     PC3
    chanel_14  PC4     PC4
    chanel_15  PC5     PC5
    chanel_16  temperature sensor chanel
    chanel_17  Internal reference voltage

 * @version 0.1
 * @date 2022-10-22
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef WADC_H
#define WADC_H

#include "stm32f10x.h"


extern uint16_t ADC_CH[2];

extern uint16_t ADC_CH0,ADC_CH1,ADC_CH2;

void WADC_DMA_Init(void);

void WADC_Init(void);

uint16_t AD_GetValue(uint8_t ADC_Channel);


#endif



#if 0 

//------------------ADC+DMA 读取到的数据 不是特别稳定------------

#include "stm32f10x.h"     // Device header
#include "Delay.h"
#include "OLED.h"
#include "Timer.h"
//#include "TB6612FNG.h" //Drive Motor
//#include "EXTI.h"
#include "wadc.h"
//#include "WDMA.h"






/**  
 * @brief 主函数   
 *  
 * @return int    
 */  
int main(void)
{

	WADC_DMA_Init();//多通道ADC初始化 NO +DMA

	OLED_Init();
  
    OLED_ShowString(1, 1, "CH0:");
	OLED_ShowString(2, 1, "CH1:");	
    OLED_ShowString(3, 1, "Key:");
   
	while(1)
	{
        ADC_CH0= ( ( ADC_CH[0] /4096.0 ) *3300 ) / 100 ;//单位 100mv
        ADC_CH1= ( ( ADC_CH[1] /4096.0 ) *3300 ) / 100 ;//单位 100mv
        
		OLED_ShowNum(1, 6,ADC_CH0, 5);
        OLED_ShowNum(2, 6,ADC_CH1, 5);
        OLED_ShowNum(3, 6,99,  5);


	}
}


//------------------ADC+DMA 读取到的数据 不是特别稳定------------




//------------------ADC没+DMA，读取到的数据很稳定------------

#include "stm32f10x.h"     // Device header
#include "Delay.h"
#include "OLED.h"
#include "Timer.h"
//#include "TB6612FNG.h" //Drive Motor
//#include "EXTI.h"
#include "wadc.h"
//#include "WDMA.h"






/**  
 * @brief 主函数   
 *  
 * @return int    
 */  
int main(void)
{

	WADC_Init();//多通道ADC初始化 NO +DMA

	OLED_Init();
  
    OLED_ShowString(1, 1, "CH0:");
	OLED_ShowString(2, 1, "CH1:");	
    OLED_ShowString(3, 1, "Key:");
   
	while(1)
	{
        ADC_CH0= ((AD_GetValue(ADC_Channel_0) /4096.0) *3300)/100 ;//单位 100mv
        ADC_CH1= ((AD_GetValue(ADC_Channel_1) /4096.0) *3300)/100 ;//单位 100mv
        
		OLED_ShowNum(1, 6,ADC_CH0, 5);
        OLED_ShowNum(2, 6,ADC_CH1, 5);
        OLED_ShowNum(3, 6,99,  5);

       /// Delay_ms(50);
	}
}

//------------------ADC没+DMA，读取到的数据很稳定------------


#endif

