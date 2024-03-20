/**
 * @file USART.h
 * @author wls
 * @brief ����ͨ������ 
 * STM32F103C8T6  ��USART1(����������IO)    USART2      USART3
 * 
 * USART1_CTS :  PA11 
 * USART1_RTS :  PA12
 * USART1_CK :   PA8   
 * 
 * USART1_TX :  PA9    ���� PB6
 * USART1_RX :  PA10   ���� PB7
 * 
 * 
 * USART2_CTS :  PA0 
 * USART2_RTS :  PA1 
 * USART2_CK  :  PA4   
 * 
 * USART2_TX :  PA2    
 * USART2_RX :  PA3  
 * 
 * 
 * 
 * USART3_CTS :  PB13 
 * USART3_RTS :  PB14
 * USART3_CK  :  PB12  
 * 
 * USART3_TX :  PB10   
 * USART3_RX :  PB11 
 * 
 * 
 *  
 * @version 0.1
 * @date 2022-10-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __USART_H
#define __USART_H

#include "stm32f10x.h"
#include "stdio.h"
#include "stdarg.h"
#include <string.h>

#define W_USART1  1   //���� ��1  ��������0
#define W_USART2  0
#define W_USART3  0

extern uint8_t Rx_Data,Rx_Flag,Rx_Count;
extern char Tx_Pack[],Rx_Pack[];
// extern char Pa[];

extern uint8_t Rx_State;

void WUsart1_Init(void);
void WUsart2_Init(void);
void WUsart3_Init(void);


void Send_Byte(char Byte);
void Send_String(char *s);
int fputc(int ch,FILE *f);
void S_Printf(char *format,...);

uint8_t Get_Usart1_RxData(void);
uint8_t Get_Usart1_RxFlag(void);

u8 Get_Rx_Packge_State(void);
char* Get_RxPackge(void);





#endif


#if 0


//-------------------sendstring printf-------------------
#include "stm32f10x.h"     // Device header
#include "Delay.h"
#include "OLED.h"
#include "Timer.h"
//#include "TB6612FNG.h" //Drive Motor
//#include "EXTI.h"
#include "wadc.h"
//#include "WDMA.h"
#include "USART.h"// ����





/**  
 * @brief ������   
 *  
 * @return int    
 */  
int main(void)
{

	WADC_Init();//��ͨ��ADC��ʼ�� NO +DMA
    WUsart_Init();//USART��ʼ��  

	OLED_Init();
  
    OLED_ShowString(1, 1, "CH0:");
	OLED_ShowString(2, 1, "CH1:");	
    OLED_ShowString(3, 1, "Key:");
   
	while(1)
	{
		
        ADC_CH0= ( ( AD_GetValue(ADC_Channel_0) /4096.0 ) *3300 ) / 100 ;//��λ 100mv
        ADC_CH1= ( ( AD_GetValue(ADC_Channel_1) /4096.0 ) *3300 ) / 100 ;//��λ 100mv
        
		OLED_ShowNum(1, 6,ADC_CH0, 5);
        OLED_ShowNum(2, 6,ADC_CH0, 5);
        OLED_ShowNum(3, 6,99,  5);
        //Send_String("npn\n");
        printf("X %d Y: %d \n",ADC_CH0,ADC_CH1);

	}
}

//-------------------sendstring printf-------------------


//-----------����һ���ֽ�����------------

        if(Get_Usart1_RxFlag())
        {
            Send_Byte(Get_Usart1_RxData());     
            //OLED_ShowString(4, 1, Rx_Pack);       
        }
//-----------����һ���ֽ�����------------




//-----------USART1 �շ��ַ���------------

#include "stm32f10x.h"     // Device header
#include "Delay.h"
#include "OLED.h"
#include "Timer.h"
//#include "TB6612FNG.h" //Drive Motor
//#include "EXTI.h"
#include "wadc.h"
//#include "WDMA.h"
#include "USART.h"// ����





/**  
 * @brief ������   
 *  
 * @return int    
 */  
int main(void)
{

	WADC_Init();//��ͨ��ADC��ʼ�� NO +DMA
    WUsart_Init();//USART��ʼ��  

	OLED_Init();
  
    OLED_ShowString(1, 1, "CH0:");
	OLED_ShowString(2, 1, "CH1:");	


 
	while(1)
	{
		
        ADC_CH0= ( ( AD_GetValue(ADC_Channel_0) /4096.0 ) *3300 ) / 100 ;//��λ 100mv
        ADC_CH1= ( ( AD_GetValue(ADC_Channel_1) /4096.0 ) *3300 ) / 100 ;//��λ 100mv
        
		OLED_ShowNum(1, 6,ADC_CH0, 5);
        OLED_ShowNum(2, 6,ADC_CH1, 5);
   
        //Send_String("npn\n");
        //printf("X %d Y: %d \n",ADC_CH0,ADC_CH1);
        //S_Printf("\t ������ʥ��� ����˶ X %d Y: %d \r\n",ADC_CH0,ADC_CH1);

        if(Get_Rx_Packge_State())
        {
            OLED_ShowString(3, 2, Rx_Pack);
            Send_String(Get_RxPackge());
            
        }
        


	

	}
}
//-----------USART1 �շ��ַ���------------


#include "stm32f10x.h"     // Device header
#include "Delay.h"
#include "OLED.h"
#include "Timer.h"
//#include "TB6612FNG.h" //Drive Motor
//#include "EXTI.h"
#include "wadc.h"
//#include "WDMA.h"
#include "USART.h"// ����





/**  
 * @brief ������   
 *  
 * @return int    
 */  
int main(void)
{

	WADC_Init();//��ͨ��ADC��ʼ�� NO +DMA
    WUsart_Init();//USART��ʼ��  

	OLED_Init();
  
    OLED_ShowString(1, 1, "CH0:");
	OLED_ShowString(2, 1, "CH1:");	

    
 
	while(1)
	{
		
        ADC_CH0= ( ( AD_GetValue(ADC_Channel_0) /4096.0 ) *3300 ) / 100 ;//��λ 100mv
        ADC_CH1= ( ( AD_GetValue(ADC_Channel_1) /4096.0 ) *3300 ) / 100 ;//��λ 100mv
        
		OLED_ShowNum(1, 6,ADC_CH0, 5);
        OLED_ShowNum(2, 6,ADC_CH1, 5);
   
        //Send_String("npn\n");
        //printf("X %d Y: %d \n",ADC_CH0,ADC_CH1);
        //S_Printf("\t ������ʥ��� ����˶ X %d Y: %d \r\n",ADC_CH0,ADC_CH1);

        if(Get_Rx_Packge_State())
        {
            OLED_ShowString(3, 2, Rx_Pack);
            Send_String(Get_RxPackge());

            OLED_ShowNum(4, 6,(int16_t)strcmp(Rx_Pack,"w#"), 5);
            
        }
        


	

	}
}


#include "stm32f10x.h"     // Device header
#include "Delay.h"
#include "OLED.h"
#include "Timer.h"
//#include "TB6612FNG.h" //Drive Motor
//#include "EXTI.h"
#include "wadc.h"
//#include "WDMA.h"
#include "USART.h"// ����




//-----------------�����յ��������Ƿ���ȷ---------------

/**  
 * @brief ������   
 *  
 * @return int    
 */  
int main(void)
{

	WADC_Init();//��ͨ��ADC��ʼ�� NO +DMA
    WUsart3_Init();//USART1��ʼ��  

	OLED_Init();
  
    OLED_ShowString(1, 1, "CH0:");
	OLED_ShowString(2, 1, "CH1:");	

    char *Rx_string=NULL;
 
	while(1)
	{
		
        ADC_CH0= ( ( AD_GetValue(ADC_Channel_0) /4096.0 ) *3300 ) / 100 ;//��λ 100mv
        ADC_CH1= ( ( AD_GetValue(ADC_Channel_1) /4096.0 ) *3300 ) / 100 ;//��λ 100mv
        
		OLED_ShowNum(1, 6,ADC_CH0, 5);
        OLED_ShowNum(2, 6,ADC_CH1, 5);
   
        //Send_String("npn\n");
        //printf("X %d Y: %d \n",ADC_CH0,ADC_CH1);
        //S_Printf("\t ������ʥ��� ����˶ X %d Y: %d \r\n",ADC_CH0,ADC_CH1);

        if(Get_Rx_Packge_State())
        {
            Rx_string=Get_RxPackge();
           // OLED_ShowString(3, 2, Rx_string);
            S_Printf("\t ������ʥ��� ����˶ X %d Y: %d %s \r\n",ADC_CH0,ADC_CH1,Rx_string);
            S_Printf("���յ�������: %i %i %i %i \n",Rx_string[0],Rx_string[1],Rx_string[2],Rx_string[3]);
            OLED_ShowNum(3,2,strcmp(Rx_string,"wl#"),5);//�ԱȽ��յ����ַ������� �� "w1#"���
            OLED_ShowNum(4,2,sizeof("w#") ,5);
            Rx_string=NULL;
        }
        
	}
}
//-----------------�����յ��������Ƿ���ȷ---------------





#endif





