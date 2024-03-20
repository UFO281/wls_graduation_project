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
//IO��������
#define SDA_IN()  {GPIOB->CRL&=0XFFF0FFFF;GPIOB->CRL|=(u32)8<<16;}
#define SDA_OUT() {GPIOB->CRL&=0XFFF0FFFF;GPIOB->CRL|=(u32)3<<16;}

//IO��������	 
#define IIC_SCL    PBout(3) //SCL
#define IIC_SDA    PBout(4) //SDA	 
#define READ_SDA   PBin(4)  //����SDA 

//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��				 
void IIC_Start(void);				//����IIC��ʼ�ź�
void IIC_Stop(void);	  			//����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);			//IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void IIC_Ack(void);					//IIC����ACK�ź�
void IIC_NAck(void);				//IIC������ACK�ź�

void IIC_Write_One_Byte(u8 daddr,u8 addr,u8 data);
u8 IIC_Read_One_Byte(u8 daddr,u8 addr);	  

//-----------------------IIC---------------------------------------





#endif


#if 0
/**
 * @file main.c
 * @author ������202 ����˶ 18
 * @brief ��ʽ�����˶�Դ����
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
#include "24l01.h"//����ͨ��ģ�� NRF24L01
#include "aht10.h"//��ʪ�ȴ����� 
#include "WEXTI.h"
#include "WTimer.h"
#include "WPWM.h"//PWM
#include "stm32f10x_tim.h"
#include "L298N.h"//�������
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

	AHT10Init();//��ʪ�ȴ�������ʼ��
	AHT10Reset();

	NRF24L01_Init(); //��ʼ������ͨ��ģ��NRF24L01
	while(NRF24L01_Check()){//����Ƿ����������� 
		//ʧ���˾ͻ����������ѭ��
		//LCD_ShowString(9,10,"NRF ERROR!",WHITE,DARKBLUE,16,0);// 9�� 10��

	}

	LCD_Init();//LCD��ʼ��
	LCD_Fill(0,0,LCD_W,LCD_H,BLACK);//��������

	LCD_ShowString(29,5,"RX Data",WHITE,BLACK,16,0);// 49�� 5��
	LCD_ShowString(15,30,"S:",WHITE,BLACK,16,0);// 10�� 40��
	
	LCD_ShowString(10,60,"temp:",WHITE,BLACK,16,0);// 10�� 40��
	LCD_ShowString(108,60,"C",WHITE,BLACK,16,0);// 89�� 10��

	LCD_ShowString(10,75,"Humi:",WHITE,BLACK,16,0);// 10�� 40��
	LCD_ShowString(86,75,"%",WHITE,BLACK,16,0);// 89�� 10��

	LCD_ShowString(10,95,"R:",WHITE,BLACK,16,0);// 10�� 40��
	LCD_ShowString(70,95,"L:",WHITE,BLACK,16,0);// 10�� 40��

	//WEXTI_Init();//�ⲿ�жϳ�ʼ��
	Wpwm_Init(99,359);//2KHZ PWM
	//WTimer_Init();//��ʱ����ʼ��
	wheels_GPIO_Init();

	while(1)
	{
		delay_ms(50);
		LCD_ShowIntNum(28,95,pwm_R,4,WHITE,BRED,16);//RITH 
		LCD_ShowIntNum(88,95,pwm_L,4,WHITE,RED,16);//LEFT

		AHT10ReadData(&AHT10_temp,&AHT10_humi);//��ȡ��ʪ�ȴ�����������
		LCD_ShowFloatNum1(58,60,AHT10_temp,4,WHITE,BLACK,16);
		LCD_ShowIntNum(58,75,AHT10_humi,3,WHITE,BLACK,16);
			           
//-----------------NRF24L01����ģʽ-------------------------       
        //����ģʽ
        NRF24L01_Init(); //��ʼ��NRF24L01
        while(NRF24L01_Check());//���ʧ�ܽ�����ѭ��
        NRF24L01_RX_Mode();//����ģʽ��ʼ�� 
       
        							
        NRF24L01_RxPacket(rx_data);//һ�����յ���Ϣ,����ʾ����.rx_data[0]���������ź�  1ǰ��  2����  3��ת 4��ת 0ɲ��() ����������Ϊ
        LCD_ShowIntNum(32,30,rx_data[0],2,WHITE,BLACK,16);
        
        //----------С�����յ����� ���д�����Ƶ��---------------
        switch (rx_data[0]) 
        {
            case 0:// 0ɲ��
                LCD_ShowString(58,30,"STOP!",WHITE,RED,16,0);// 10�� 40��
                wheels_RL(2,2);//����ͣת
                x=1;
                break;
            
            case 1://ǰ��
                wheels_RL(0,1);//������ת
                LCD_ShowString(58,30,"GoGo!",BLACK,GREEN,16,0);// 10�� 40��
                pwm_R=pwm_R+30;
                pwm_L=pwm_L+30;
                if((pwm_R>=350) || (pwm_L>=350) ){ pwm_R=350;pwm_L=350; }
                TIM_SetCompare1(TIM3, pwm_R);
                TIM_SetCompare2(TIM3, pwm_L);
                break;

            case 2://����
                wheels_RL(1,0);//���ӷ�ת
                LCD_ShowString(58,30,"Back!",BLACK,BROWN,16,0);// 10�� 40��
                pwm_R=pwm_R+30;
                pwm_L=pwm_L+30;
                if((pwm_R>=350) || (pwm_L>=350) ){ pwm_R=350;pwm_L=350; }
                TIM_SetCompare1(TIM3, pwm_R);
                TIM_SetCompare2(TIM3, pwm_L);
                break;

            case 3://��ת
                wheels_RL(0,1);//������ת
                LCD_ShowString(58,30,"<-Lt!",BLACK,GREEN,16,0);// 10�� 40��
                pwm_R=pwm_R+10;
                if(pwm_R>=350) pwm_R=350;
                if(pwm_L>45) pwm_L=pwm_L-10;
                TIM_SetCompare1(TIM3, pwm_R);
                TIM_SetCompare2(TIM3, pwm_L);
                break;

            case 4://��ת
                wheels_RL(0,1);//������ת
                LCD_ShowString(58,30,"->Rt!",BLACK,GREEN,16,0);// 10�� 40��
                pwm_L=pwm_L+10;
                if(pwm_L>=350) pwm_L=350;
                if(pwm_R>45) pwm_R=pwm_R-10;
                TIM_SetCompare1(TIM3, pwm_R);
                TIM_SetCompare2(TIM3, pwm_L);
                break;
            
            case 5://��Ϣ
                LCD_ShowString(58,30,"rest!",WHITE,BLUE,16,0);// 10�� 40
                pwm_R=0;
                pwm_L=0;
                TIM_SetCompare1(TIM3, pwm_R);
                TIM_SetCompare2(TIM3, pwm_L);
                break;

        }
        //----------С�����յ����� ���д�����Ƶ��---------------
//-----------------NRF24L01����ģʽ-------------------------  
    	

        //------------------����ģʽ---------------------------
		while ((x>=1) && (x<25))
        {   
            while(Tx==0){//����ģʽ
                NRF24L01_Init(); //��ʼ��NRF24L01
                while(NRF24L01_Check());//���ʧ�ܽ�����ѭ��
                NRF24L01_TX_Mode();//����ģʽ��ʼ�� 
                ++Tx;
            }
            delay_ms(50);
            AHT10ReadData(&AHT10_temp,&AHT10_humi);//��ȡ��ʪ������ 
            LCD_ShowFloatNum1(58,60,AHT10_temp,4,WHITE,BLACK,16);
		    LCD_ShowIntNum(58,75,AHT10_humi,3,WHITE,BLACK,16);
            tx_data[0]=AHT10_temp;
            tx_data[1]=AHT10_humi;
            if(NRF24L01_TxPacket(tx_data)==TX_OK);//printf("\tsend is OK!\n");	
            x++;
            if(x==24) rx_data[0]=5;
        }
        x%=25;//������ɺ� ��ֵΪ0
        Tx=0;
        //------------------����ģʽ---------------------------

        

    }

}






#endif
