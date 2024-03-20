/**
 * @file WTask.h
 * @author wls
 * @brief �û��Զ��������� ����
 * @version 0.1
 * @date 2022-11-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef WTASK_H
#define WTASK_H


#include "stm32f10x.h"

#include "FreeRTOS.h"
#include "TIMER.h"//Timer X Config
#include "queue.h"
#include "semphr.h"
#include "task.h" //FreeRTOS Task Data 

#include "string.h"

#include "oled.h"//��ʾ�� 0.96 OLED

#include "delay.h"//��ʱ����

#include "MPU6050\mpu6050.h"//MPU6050
#include "inv_mpu.h"

#include "usart.h"

#include "EXTI.h" //�ⲿ�ж�

#include "L298N\L298N.h"//

// #include "NRF24L01\24l01.h" 

#include "CAN/can.h"


#define Use_AHT10   1   // use AHT10 temp sensor

#define Use_DHT11   0    // use DHT11 temp sensor


#if (Use_AHT10==1)
    #include "AHT10\AHT10.h"
#elif (Use_DHT11==1)
    #include "DHT11\dht11.h"   
#endif







//--------------------������������--------------------




//--------------------������������--------------------




//-------------------------������������--------------------

void HardWare_Init(void);
int Start_Task(void);
void AppTaskCreate(void);
void Task_0(void * pvParameters);
void Task_1(void * pvParameters);
void Task_2(void * pvParameters);
void Task_3(void * pvParameters);
void Task_4( void * pvParameters );
void Task_5( void * pvParameters );
uint16_t Get_Absolute_value(int x);

//-------------------------������������--------------------









#endif








