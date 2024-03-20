#ifndef __CAN_H
#define __CAN_H

#include "sys.h"	    
#include "delay.h"
#include "usart.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_can.h"
#include "misc.h"
#include "stm32f10x_rcc.h"






//CAN接收RX0中断使能
#define CAN_RX0_INT_ENABLE	1		//0,不使能;1,使能.	

#define PASS_ID     ( (unsigned int)0x1234 )
	
extern char CAN_INT_Flag;

										 							 				    
u8 CAN_Config(u8 mode);//CAN初始化
 
u8 Can_Send_Msg(u8* msg,u8 len); //发送数据

u8 Can_Receive_Msg(u8 *buf); //接收数据



#endif

















