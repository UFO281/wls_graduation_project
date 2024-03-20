/**
 * @file WDMA.h
 * @author wls
 * @brief DMA config,DMA 是个数据搬运工，把数据可以从 RAM搬到 RAM ，从外设寄存器 搬到RAM，
 * 从RAM搬到外设寄存器
 * ，定义的数组变量结构体数据 ，就存储在RAM数据寄存器区域里
 * @version 0.1
 * @date 2022-10-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef WDMA_H
#define WDMA_H

#include "stm32f10x.h"

void W_DMA_Init(uint32_t PeripheralBaseAddr ,uint32_t MemoryBaseAddr ,int count );

void StarRun_DMA(void);



#endif


#if 0

//------------------DMA-SRAM-单次数据搬运-例子-----------------------------

#include "stm32f10x.h"     // Device header
#include "Delay.h"
#include "OLED.h"
#include "Timer.h"
//#include "TB6612FNG.h" //Drive Motor
//#include "EXTI.h"
#include "wadc.h"
#include "WDMA.h"


u8 DataA[3]={1,3,4};
u8 DataB[3]={0};



/**  
 * @brief 主函数   
 *  
 * @return int    
 */  
int main(void)
{

    u8 key=0;

    int ADC_CH0=0,ADC_CH1=0;


	WADC_Init();//单通道ADC初始化

	OLED_Init();
  
    //OLED_ShowString(1, 1, "CH0:");    
	//OLED_ShowString(2, 1, "CH1:");   	   
    //OLED_ShowString(3, 1, "Key:");     
   



    W_DMA_Init((uint32_t)DataA,(uint32_t)DataB,3);//把数据 用DMA 从DataA 搬运到 DataB 数组长度为3 搬运三次 内部地址自增

    OLED_ShowNum(1,1,DataB[0],5);   
    OLED_ShowNum(2,1,DataB[1],5);   
    OLED_ShowNum(3,1,DataB[2],5);   

	while (1)
	{
        //ADC_CH0= ((Get_ADC_value(ADC_Channel_0)/4096.0) *3300)/100 ;//单位 100mv
        //ADC_CH1= ((Get_ADC_value(ADC_Channel_1)/4096.0) *3300)/100 ;//单位 100mv
        
        
		//OLED_ShowNum(1, 6,ADC_CH0, 5);// 
        //OLED_ShowNum(2, 6,ADC_CH1,  5);// 
        //OLED_ShowNum(3, 6,key,  5);//

	}
}
//------------------DMA-SRAM-单次数据搬运-例子-----------------------------



//------------------DMA-SRAM-多次数据搬运-例子-----------------------------

#include "stm32f10x.h"     // Device header
#include "Delay.h"
#include "OLED.h"
#include "Timer.h"
//#include "TB6612FNG.h" //Drive Motor
//#include "EXTI.h"
#include "wadc.h"
#include "WDMA.h"


u8 DataA[3]={1,3,4};
u8 DataB[3]={0};



/**  
 * @brief 主函数   
 *  
 * @return int    
 */  
int main(void)
{

    u8 key=0;

    int ADC_CH0=0,ADC_CH1=0;


	WADC_Init();//单通道ADC初始化

	OLED_Init();
  
    //OLED_ShowString(1, 1, "CH0:");
	//OLED_ShowString(2, 1, "CH1:");	
    //OLED_ShowString(3, 1, "Key:");
   

    W_DMA_Init((uint32_t)DataA,(uint32_t)DataB,3);//把数据 用DMA 从DataA 搬运到 DataB 数组长度为3 搬运三次 内部地址自增


	while (1)
	{

        //ADC_CH0= ((Get_ADC_value(ADC_Channel_0)/4096.0) *3300)/100 ;//单位 100mv
        //ADC_CH1= ((Get_ADC_value(ADC_Channel_1)/4096.0) *3300)/100 ;//单位 100mv
        
        
		//OLED_ShowNum(1, 6,ADC_CH0, 5);// 
        //OLED_ShowNum(2, 6,ADC_CH1,  5);// 
        //OLED_ShowNum(3, 6,key,  5);//

        StarRun_DMA();//开启DMA转运 转运指定次数后 3次  就关闭了 所以得重新配置开启下

        OLED_ShowNum(1,1,DataB[0],5);
        OLED_ShowNum(2,1,DataB[1],5);
        OLED_ShowNum(3,1,DataB[2],5); 

        for(int i=0;i<3;i++)  ++DataA[i];
        
        Delay_ms(500);
        

	}
}



//------------------DMA-SRAM-多次数据搬运-例子-----------------------------

#endif


