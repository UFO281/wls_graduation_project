#ifndef _AHT10_H__
#define _AHT10_H__

#include  "sys.h"

#define AHT10_ADDRESS 0x70
#define AHT10_WRITE 0x70
#define AHT10_READ 0x71


extern void AHT10Init(void);
extern u8 AHT10Check(void);
extern void AHT10Reset(void);
extern u8 AHT10ReadData(float *temperature,u8 *humidity);



//-----------------------IIC---------------------------------------
//IO方向设置
#define SDA_IN()  {GPIOB->CRL&=0XFFF0FFFF;GPIOB->CRL|=(u32)8<<16;}
#define SDA_OUT() {GPIOB->CRL&=0XFFF0FFFF;GPIOB->CRL|=(u32)3<<16;}

//IO操作函数	 
#define IIC_SCL    PBout(3) //SCL
#define IIC_SDA    PBout(4) //SDA	 
#define READ_SDA   PBin(4)  //输入SDA 

//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口				 
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8 IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	  

//-----------------------IIC---------------------------------------





#endif


#if 0
/**
 * @file main.c
 * @author 机器人202 王令硕 18
 * @brief 轮式机器人端源代码
 * @version 0.1
 * @date 2022-09-20
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "delay.h"
#include "sys.h"
#include "lcd.h"
//#include "pic.h" //LCD show picture
#include "24l01.h"//无线通信模块 NRF24L01
#include "aht10.h"//温湿度传感器 
#include "WEXTI.h"
#include "WTimer.h"
#include "WPWM.h"//PWM
#include "stm32f10x_tim.h"
#include "L298N.h"//电机驱动
#include "stm32f10x_exti.h"



int main(void)
{
	u8 tx_data[2]={0};
	u8 rx_data[2]={5};
	float AHT10_temp=0.0f;
	u8 AHT10_humi=0;
	unsigned int pwm_R=0,pwm_L=0;
	short x=0;
    u8 Tx=0;

	delay_init();

	AHT10Init();//温湿度传感器初始化
	AHT10Reset();

	NRF24L01_Init(); //初始化无线通信模块NRF24L01
	while(NRF24L01_Check()){//检查是否能正常工作 
		//失败了就会陷入这个死循环
		//LCD_ShowString(9,10,"NRF ERROR!",WHITE,DARKBLUE,16,0);// 9列 10行

	}

	LCD_Init();//LCD初始化
	LCD_Fill(0,0,LCD_W,LCD_H,BLACK);//背景设置

	LCD_ShowString(29,5,"RX Data",WHITE,BLACK,16,0);// 49列 5行
	LCD_ShowString(15,30,"S:",WHITE,BLACK,16,0);// 10列 40行
	
	LCD_ShowString(10,60,"temp:",WHITE,BLACK,16,0);// 10列 40行
	LCD_ShowString(108,60,"C",WHITE,BLACK,16,0);// 89列 10行

	LCD_ShowString(10,75,"Humi:",WHITE,BLACK,16,0);// 10列 40行
	LCD_ShowString(86,75,"%",WHITE,BLACK,16,0);// 89列 10行

	LCD_ShowString(10,95,"R:",WHITE,BLACK,16,0);// 10列 40行
	LCD_ShowString(70,95,"L:",WHITE,BLACK,16,0);// 10列 40行

	//WEXTI_Init();//外部中断初始化
	Wpwm_Init(99,359);//2KHZ PWM
	//WTimer_Init();//定时器初始化
	wheels_GPIO_Init();

	while(1)
	{
		delay_ms(50);
		LCD_ShowIntNum(28,95,pwm_R,4,WHITE,BRED,16);//RITH 
		LCD_ShowIntNum(88,95,pwm_L,4,WHITE,RED,16);//LEFT

		AHT10ReadData(&AHT10_temp,&AHT10_humi);//读取温湿度传感器的数据
		LCD_ShowFloatNum1(58,60,AHT10_temp,4,WHITE,BLACK,16);
		LCD_ShowIntNum(58,75,AHT10_humi,3,WHITE,BLACK,16);
			           
//-----------------NRF24L01接收模式-------------------------       
        //接收模式
        NRF24L01_Init(); //初始化NRF24L01
        while(NRF24L01_Check());//检查失败进入死循环
        NRF24L01_RX_Mode();//接收模式初始化 
       
        							
        NRF24L01_RxPacket(rx_data);//一旦接收到信息,则显示出来.rx_data[0]传给车的信号  1前进  2后退  3左转 4右转 0刹车() 其他并不作为
        LCD_ShowIntNum(32,30,rx_data[0],2,WHITE,BLACK,16);
        
        //----------小车接收到数据 进行处理控制电机---------------
        switch (rx_data[0]) 
        {
            case 0:// 0刹车
                LCD_ShowString(58,30,"STOP!",WHITE,RED,16,0);// 10列 40行
                wheels_RL(2,2);//轮子停转
                x=1;
                break;
            
            case 1://前进
                wheels_RL(0,1);//轮子正转
                LCD_ShowString(58,30,"GoGo!",BLACK,GREEN,16,0);// 10列 40行
                pwm_R=pwm_R+30;
                pwm_L=pwm_L+30;
                if((pwm_R>=350) || (pwm_L>=350) ){ pwm_R=350;pwm_L=350; }
                TIM_SetCompare1(TIM3, pwm_R);
                TIM_SetCompare2(TIM3, pwm_L);
                break;

            case 2://后退
                wheels_RL(1,0);//轮子反转
                LCD_ShowString(58,30,"Back!",BLACK,BROWN,16,0);// 10列 40行
                pwm_R=pwm_R+30;
                pwm_L=pwm_L+30;
                if((pwm_R>=350) || (pwm_L>=350) ){ pwm_R=350;pwm_L=350; }
                TIM_SetCompare1(TIM3, pwm_R);
                TIM_SetCompare2(TIM3, pwm_L);
                break;

            case 3://左转
                wheels_RL(0,1);//轮子正转
                LCD_ShowString(58,30,"<-Lt!",BLACK,GREEN,16,0);// 10列 40行
                pwm_R=pwm_R+10;
                if(pwm_R>=350) pwm_R=350;
                if(pwm_L>45) pwm_L=pwm_L-10;
                TIM_SetCompare1(TIM3, pwm_R);
                TIM_SetCompare2(TIM3, pwm_L);
                break;

            case 4://右转
                wheels_RL(0,1);//轮子正转
                LCD_ShowString(58,30,"->Rt!",BLACK,GREEN,16,0);// 10列 40行
                pwm_L=pwm_L+10;
                if(pwm_L>=350) pwm_L=350;
                if(pwm_R>45) pwm_R=pwm_R-10;
                TIM_SetCompare1(TIM3, pwm_R);
                TIM_SetCompare2(TIM3, pwm_L);
                break;
            
            case 5://休息
                LCD_ShowString(58,30,"rest!",WHITE,BLUE,16,0);// 10列 40
                pwm_R=0;
                pwm_L=0;
                TIM_SetCompare1(TIM3, pwm_R);
                TIM_SetCompare2(TIM3, pwm_L);
                break;

        }
        //----------小车接收到数据 进行处理控制电机---------------
//-----------------NRF24L01接收模式-------------------------  
    	

        //------------------发送模式---------------------------
		while ((x>=1) && (x<25))
        {   
            while(Tx==0){//发送模式
                NRF24L01_Init(); //初始化NRF24L01
                while(NRF24L01_Check());//检查失败进入死循环
                NRF24L01_TX_Mode();//发送模式初始化 
                ++Tx;
            }
            delay_ms(50);
            AHT10ReadData(&AHT10_temp,&AHT10_humi);//读取温湿度数据 
            LCD_ShowFloatNum1(58,60,AHT10_temp,4,WHITE,BLACK,16);
		    LCD_ShowIntNum(58,75,AHT10_humi,3,WHITE,BLACK,16);
            tx_data[0]=AHT10_temp;
            tx_data[1]=AHT10_humi;
            if(NRF24L01_TxPacket(tx_data)==TX_OK);//printf("\tsend is OK!\n");	
            x++;
            if(x==24) rx_data[0]=5;
        }
        x%=25;//发送完成后 赋值为0
        Tx=0;
        //------------------发送模式---------------------------

        

    }

}






#endif
