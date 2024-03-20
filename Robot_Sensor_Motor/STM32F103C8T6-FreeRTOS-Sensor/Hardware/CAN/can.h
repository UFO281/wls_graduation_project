#ifndef __CAN_H
#define __CAN_H

#include "sys.h"	    
#include "delay.h"
#include "usart.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_can.h"
#include "misc.h"
#include "stm32f10x_rcc.h"






//CAN����RX0�ж�ʹ��
#define CAN_RX0_INT_ENABLE	1		//0,��ʹ��;1,ʹ��.	

#define PASS_ID     ( (unsigned int)0x1234 )
	
extern char CAN_INT_Flag;

										 							 				    
u8 CAN_Config(u8 mode);//CAN��ʼ��
 
u8 Can_Send_Msg(u8* msg,u8 len); //��������

u8 Can_Receive_Msg(u8 *buf); //��������



#endif

















