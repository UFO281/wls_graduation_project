#ifndef __MPU6050_H
#define __MPU6050_H
#include "mpuiic.h"   												  	  


 
//MPU6050 AD0���ƽ�=GND,INT=GND
//#define MPU_AD0_CTRL			PAout(15)	//����AD0��ƽ,�Ӷ�����MPU��ַ

//#define MPU_ACCEL_OFFS_REG		0X06	//accel_offs�Ĵ���,�ɶ�ȡ�汾��,�Ĵ����ֲ�δ�ᵽ
//#define MPU_PROD_ID_REG			0X0C	//prod id�Ĵ���,�ڼĴ����ֲ�δ�ᵽ
#define MPU_SELF_TESTX_REG		0X0D	//�Լ�Ĵ���X
#define MPU_SELF_TESTY_REG		0X0E	//�Լ�Ĵ���Y
#define MPU_SELF_TESTZ_REG		0X0F	//�Լ�Ĵ���Z
#define MPU_SELF_TESTA_REG		0X10	//�Լ�Ĵ���A
#define MPU_SAMPLE_RATE_REG		0X19	//����Ƶ�ʷ�Ƶ��
#define MPU_CFG_REG				0X1A	//���üĴ���
#define MPU_GYRO_CFG_REG		0X1B	//���������üĴ���
#define MPU_ACCEL_CFG_REG		0X1C	//���ٶȼ����üĴ���
#define MPU_MOTION_DET_REG		0X1F	//�˶���ֵⷧ���üĴ���
#define MPU_FIFO_EN_REG			0X23	//FIFOʹ�ܼĴ���
#define MPU_I2CMST_CTRL_REG		0X24	//IIC�������ƼĴ���
#define MPU_I2CSLV0_ADDR_REG	0X25	//IIC�ӻ�0������ַ�Ĵ���
#define MPU_I2CSLV0_REG			0X26	//IIC�ӻ�0���ݵ�ַ�Ĵ���
#define MPU_I2CSLV0_CTRL_REG	0X27	//IIC�ӻ�0���ƼĴ���
#define MPU_I2CSLV1_ADDR_REG	0X28	//IIC�ӻ�1������ַ�Ĵ���
#define MPU_I2CSLV1_REG			0X29	//IIC�ӻ�1���ݵ�ַ�Ĵ���
#define MPU_I2CSLV1_CTRL_REG	0X2A	//IIC�ӻ�1���ƼĴ���
#define MPU_I2CSLV2_ADDR_REG	0X2B	//IIC�ӻ�2������ַ�Ĵ���
#define MPU_I2CSLV2_REG			0X2C	//IIC�ӻ�2���ݵ�ַ�Ĵ���
#define MPU_I2CSLV2_CTRL_REG	0X2D	//IIC�ӻ�2���ƼĴ���
#define MPU_I2CSLV3_ADDR_REG	0X2E	//IIC�ӻ�3������ַ�Ĵ���
#define MPU_I2CSLV3_REG			0X2F	//IIC�ӻ�3���ݵ�ַ�Ĵ���
#define MPU_I2CSLV3_CTRL_REG	0X30	//IIC�ӻ�3���ƼĴ���
#define MPU_I2CSLV4_ADDR_REG	0X31	//IIC�ӻ�4������ַ�Ĵ���
#define MPU_I2CSLV4_REG			0X32	//IIC�ӻ�4���ݵ�ַ�Ĵ���
#define MPU_I2CSLV4_DO_REG		0X33	//IIC�ӻ�4д���ݼĴ���
#define MPU_I2CSLV4_CTRL_REG	0X34	//IIC�ӻ�4���ƼĴ���
#define MPU_I2CSLV4_DI_REG		0X35	//IIC�ӻ�4�����ݼĴ���

#define MPU_I2CMST_STA_REG		0X36	//IIC����״̬�Ĵ���
#define MPU_INTBP_CFG_REG		0X37	//�ж�/��·���üĴ���
#define MPU_INT_EN_REG			0X38	//�ж�ʹ�ܼĴ���
#define MPU_INT_STA_REG			0X3A	//�ж�״̬�Ĵ���

#define MPU_ACCEL_XOUTH_REG		0X3B	//���ٶ�ֵ,X���8λ�Ĵ���
#define MPU_ACCEL_XOUTL_REG		0X3C	//���ٶ�ֵ,X���8λ�Ĵ���
#define MPU_ACCEL_YOUTH_REG		0X3D	//���ٶ�ֵ,Y���8λ�Ĵ���
#define MPU_ACCEL_YOUTL_REG		0X3E	//���ٶ�ֵ,Y���8λ�Ĵ���
#define MPU_ACCEL_ZOUTH_REG		0X3F	//���ٶ�ֵ,Z���8λ�Ĵ���
#define MPU_ACCEL_ZOUTL_REG		0X40	//���ٶ�ֵ,Z���8λ�Ĵ���

#define MPU_TEMP_OUTH_REG		0X41	//�¶�ֵ�߰�λ�Ĵ���
#define MPU_TEMP_OUTL_REG		0X42	//�¶�ֵ��8λ�Ĵ���

#define MPU_GYRO_XOUTH_REG		0X43	//������ֵ,X���8λ�Ĵ���
#define MPU_GYRO_XOUTL_REG		0X44	//������ֵ,X���8λ�Ĵ���
#define MPU_GYRO_YOUTH_REG		0X45	//������ֵ,Y���8λ�Ĵ���
#define MPU_GYRO_YOUTL_REG		0X46	//������ֵ,Y���8λ�Ĵ���
#define MPU_GYRO_ZOUTH_REG		0X47	//������ֵ,Z���8λ�Ĵ���
#define MPU_GYRO_ZOUTL_REG		0X48	//������ֵ,Z���8λ�Ĵ���

#define MPU_I2CSLV0_DO_REG		0X63	//IIC�ӻ�0���ݼĴ���
#define MPU_I2CSLV1_DO_REG		0X64	//IIC�ӻ�1���ݼĴ���
#define MPU_I2CSLV2_DO_REG		0X65	//IIC�ӻ�2���ݼĴ���
#define MPU_I2CSLV3_DO_REG		0X66	//IIC�ӻ�3���ݼĴ���

#define MPU_I2CMST_DELAY_REG	0X67	//IIC������ʱ����Ĵ���
#define MPU_SIGPATH_RST_REG		0X68	//�ź�ͨ����λ�Ĵ���
#define MPU_MDETECT_CTRL_REG	0X69	//�˶������ƼĴ���
#define MPU_USER_CTRL_REG		0X6A	//�û����ƼĴ���
#define MPU_PWR_MGMT1_REG		0X6B	//��Դ����Ĵ���1
#define MPU_PWR_MGMT2_REG		0X6C	//��Դ����Ĵ���2 
#define MPU_FIFO_CNTH_REG		0X72	//FIFO�����Ĵ����߰�λ
#define MPU_FIFO_CNTL_REG		0X73	//FIFO�����Ĵ����Ͱ�λ
#define MPU_FIFO_RW_REG			0X74	//FIFO��д�Ĵ���
#define MPU_DEVICE_ID_REG		0X75	//����ID�Ĵ���
 
//���AD0��(9��)�ӵ�,IIC��ַΪ0X68(���������λ).
//�����V3.3,��IIC��ַΪ0X69(���������λ).
#define MPU_ADDR				0X68


////��Ϊģ��AD0Ĭ�Ͻ�GND,����תΪ��д��ַ��,Ϊ0XD1��0XD0(�����VCC,��Ϊ0XD3��0XD2)  
//#define MPU_READ    0XD1
//#define MPU_WRITE   0XD0

u8 MPU_Init(void); 								//��ʼ��MPU6050
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf);//IIC����д
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf); //IIC������ 
u8 MPU_Write_Byte(u8 reg,u8 data);				//IICдһ���ֽ�
u8 MPU_Read_Byte(u8 reg);						//IIC��һ���ֽ�

u8 MPU_Set_Gyro_Fsr(u8 fsr);
u8 MPU_Set_Accel_Fsr(u8 fsr);
u8 MPU_Set_LPF(u16 lpf);
u8 MPU_Set_Rate(u16 rate);
u8 MPU_Set_Fifo(u8 sens);


short MPU_Get_Temperature(void);
u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz);
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az);

#endif



#if 0

#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "lcd.h"
#include "usart.h"
#include "mpu6050.h"
#include "usmart.h"   
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
 
 
/************************************************
 ALIENTEK��ӢSTM32������ʵ��30
 MPU6050���ᴫ���� ʵ��     
 ����֧�֣�www.openedv.com
 �Ա����̣�http://eboard.taobao.com 
 ��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡSTM32���ϡ�
 ������������ӿƼ����޹�˾  
 ���ߣ�����ԭ�� @ALIENTEK
************************************************/



//����1����1���ַ� 
//c:Ҫ���͵��ַ�
void usart1_send_char(u8 c)
{   	
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); //ѭ������,ֱ���������   
	USART_SendData(USART1,c);  
} 
//�������ݸ�����������λ�����(V2.6�汾)
//fun:������. 0XA0~0XAF
//data:���ݻ�����,���28�ֽ�!!
//len:data����Ч���ݸ���
void usart1_niming_report(u8 fun,u8*data,u8 len)
{
	u8 send_buf[32];
	u8 i;
	if(len>28)return;	//���28�ֽ����� 
	send_buf[len+3]=0;	//У��������
	send_buf[0]=0X88;	//֡ͷ
	send_buf[1]=fun;	//������
	send_buf[2]=len;	//���ݳ���
	for(i=0;i<len;i++)send_buf[3+i]=data[i];			//��������
	for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];	//����У���	
	for(i=0;i<len+4;i++)usart1_send_char(send_buf[i]);	//�������ݵ�����1 
}
//���ͼ��ٶȴ��������ݺ�����������
//aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ
//gyrox,gyroy,gyroz:x,y,z�������������������ֵ
void mpu6050_send_data(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz)
{
	u8 tbuf[12]; 
	tbuf[0]=(aacx>>8)&0XFF;
	tbuf[1]=aacx&0XFF;
	tbuf[2]=(aacy>>8)&0XFF;
	tbuf[3]=aacy&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	tbuf[5]=aacz&0XFF; 
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;
	usart1_niming_report(0XA1,tbuf,12);//�Զ���֡,0XA1
}	
//ͨ������1�ϱ���������̬���ݸ�����
//aacx,aacy,aacz:x,y,z������������ļ��ٶ�ֵ
//gyrox,gyroy,gyroz:x,y,z�������������������ֵ
//roll:�����.��λ0.01�ȡ� -18000 -> 18000 ��Ӧ -180.00  ->  180.00��
//pitch:������.��λ 0.01�ȡ�-9000 - 9000 ��Ӧ -90.00 -> 90.00 ��
//yaw:�����.��λΪ0.1�� 0 -> 3600  ��Ӧ 0 -> 360.0��
void usart1_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw)
{
	u8 tbuf[28]; 
	u8 i;
	for(i=0;i<28;i++)tbuf[i]=0;//��0
	tbuf[0]=(aacx>>8)&0XFF;
	tbuf[1]=aacx&0XFF;
	tbuf[2]=(aacy>>8)&0XFF;
	tbuf[3]=aacy&0XFF;
	tbuf[4]=(aacz>>8)&0XFF;
	tbuf[5]=aacz&0XFF; 
	tbuf[6]=(gyrox>>8)&0XFF;
	tbuf[7]=gyrox&0XFF;
	tbuf[8]=(gyroy>>8)&0XFF;
	tbuf[9]=gyroy&0XFF;
	tbuf[10]=(gyroz>>8)&0XFF;
	tbuf[11]=gyroz&0XFF;	
	tbuf[18]=(roll>>8)&0XFF;
	tbuf[19]=roll&0XFF;
	tbuf[20]=(pitch>>8)&0XFF;
	tbuf[21]=pitch&0XFF;
	tbuf[22]=(yaw>>8)&0XFF;
	tbuf[23]=yaw&0XFF;
	usart1_niming_report(0XAF,tbuf,28);//�ɿ���ʾ֡,0XAF
}  
 	
 int main(void)
 {	 
	u8 t=0,report=1;			//Ĭ�Ͽ����ϱ�
	u8 key;
	float pitch,roll,yaw; 		//ŷ����
	short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
	short gyrox,gyroy,gyroz;	//������ԭʼ����
	short temp;					//�¶�	
	 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	 //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(500000);	 	//���ڳ�ʼ��Ϊ500000
	delay_init();	//��ʱ��ʼ�� 
	usmart_dev.init(72);		//��ʼ��USMART
	LED_Init();		  			//��ʼ����LED���ӵ�Ӳ���ӿ�
	KEY_Init();					//��ʼ������
	LCD_Init();			   		//��ʼ��LCD  
	MPU_Init();					//��ʼ��MPU6050
 	POINT_COLOR=RED;			//��������Ϊ��ɫ 
	LCD_ShowString(30,50,200,16,16,"ELITE STM32");	
	LCD_ShowString(30,70,200,16,16,"MPU6050 TEST");	
	LCD_ShowString(30,90,200,16,16,"ATOM@ALIENTEK");
	LCD_ShowString(30,110,200,16,16,"2015/1/17"); 
	while(mpu_dmp_init())
 	{
		LCD_ShowString(30,130,200,16,16,"MPU6050 Error");
		delay_ms(200);
		LCD_Fill(30,130,239,130+16,WHITE);
 		delay_ms(200);
	}  
	LCD_ShowString(30,130,200,16,16,"MPU6050 OK");
	LCD_ShowString(30,150,200,16,16,"KEY0:UPLOAD ON/OFF");
	POINT_COLOR=BLUE;//��������Ϊ��ɫ 
 	LCD_ShowString(30,170,200,16,16,"UPLOAD ON ");	 
 	LCD_ShowString(30,200,200,16,16," Temp:    . C");	
 	LCD_ShowString(30,220,200,16,16,"Pitch:    . C");	
 	LCD_ShowString(30,240,200,16,16," Roll:    . C");	 
 	LCD_ShowString(30,260,200,16,16," Yaw :    . C");	 
 	while(1)
	{
		key=KEY_Scan(0);
		if(key==KEY0_PRES)
		{
			report=!report;
			if(report)LCD_ShowString(30,170,200,16,16,"UPLOAD ON ");
			else LCD_ShowString(30,170,200,16,16,"UPLOAD OFF");
		}
		if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
		{ 
			temp=MPU_Get_Temperature();	//�õ��¶�ֵ
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������
			if(report)mpu6050_send_data(aacx,aacy,aacz,gyrox,gyroy,gyroz);//���Զ���֡���ͼ��ٶȺ�������ԭʼ����
			if(report)usart1_report_imu(aacx,aacy,aacz,gyrox,gyroy,gyroz,(int)(roll*100),(int)(pitch*100),(int)(yaw*10));
			if((t%10)==0)
			{ 
				if(temp<0)
				{
					LCD_ShowChar(30+48,200,'-',16,0);		//��ʾ����
					temp=-temp;		//תΪ����
				}else LCD_ShowChar(30+48,200,' ',16,0);		//ȥ������ 
				LCD_ShowNum(30+48+8,200,temp/100,3,16);		//��ʾ��������	    
				LCD_ShowNum(30+48+40,200,temp%10,1,16);		//��ʾС������ 
				temp=pitch*10;
				if(temp<0)
				{
					LCD_ShowChar(30+48,220,'-',16,0);		//��ʾ����
					temp=-temp;		//תΪ����
				}else LCD_ShowChar(30+48,220,' ',16,0);		//ȥ������ 
				LCD_ShowNum(30+48+8,220,temp/10,3,16);		//��ʾ��������	    
				LCD_ShowNum(30+48+40,220,temp%10,1,16);		//��ʾС������ 
				temp=roll*10;
				if(temp<0)
				{
					LCD_ShowChar(30+48,240,'-',16,0);		//��ʾ����
					temp=-temp;		//תΪ����
				}else LCD_ShowChar(30+48,240,' ',16,0);		//ȥ������ 
				LCD_ShowNum(30+48+8,240,temp/10,3,16);		//��ʾ��������	    
				LCD_ShowNum(30+48+40,240,temp%10,1,16);		//��ʾС������ 
				temp=yaw*10;
				if(temp<0)
				{
					LCD_ShowChar(30+48,260,'-',16,0);		//��ʾ����
					temp=-temp;		//תΪ����
				}else LCD_ShowChar(30+48,260,' ',16,0);		//ȥ������ 
				LCD_ShowNum(30+48+8,260,temp/10,3,16);		//��ʾ��������	    
				LCD_ShowNum(30+48+40,260,temp%10,1,16);		//��ʾС������  
				t=0;
				LED0=!LED0;//LED��˸
			}
		}
		t++; 
	} 	
}
 



//-------------------NEW-------------------
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
#include "TIMER\Timer.h"
#include "EXTI\EXTI.h"

#include "lcd.h"
//#include "pic.h" //LCD show picture
#include "24l01.h"//����ͨ��ģ�� NRF24L01
#include "aht10.h"//��ʪ�ȴ����� 
#include "L298N.h"//�������
#include "mpu6050.h" 
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 


/**
 * @brief ��������ֵ����
 * 
 * @param x �з��� (+ / - )����ֵ
 * @return uint16_t �޷��ŵ� ������
 */
uint16_t Get_Absolute_value(int x)
{
    if(x<0) return -x;
    else return x;
}



/**
 * @brief ������
 *STM32F103C8T6 ����IO
 PA 0 1 2 3 4 5 ,
 PB 0 1   3 4   6 7 8 10 11 13 14 15
 PC 14 15
 * @return int 0
 */
int main(void)
{
	delay_init();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	 //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
    float pitch,roll,yaw; 		//ŷ����
	short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
	short gyrox,gyroy,gyroz;	//������ԭʼ����

    MPU_Init();		//��ʼ��MPU6050 PB10 PB11
    while(mpu_dmp_init());//MPU6050 ��ʼ��ʧ�� �������������ѭ����

    int ROLL,YAW;//�洢 ת��int �Ƕ�ֵ

	u8 tx_data[2]={0};
	u8 rx_data[2]={5};
	float AHT10_temp=0.0f;
	u8 AHT10_humi=0;
	unsigned int pwm_R=0,pwm_L=0,pwm=0;
	short x=0;
    u8 Tx=0;

    float KP=3.8,KD=0.5;
    uint16_t err=0;//���



	//AHT10Init();//��ʪ�ȴ�������ʼ�� PB3 PB4
	//AHT10Reset();

	//NRF24L01_Init(); //��ʼ������ͨ��ģ��NRF24L01 ,PB6 PB7 PB8, PB13, PB14, PB15
	//while(NRF24L01_Check()){//����Ƿ����������� 
		//ʧ���˾ͻ����������ѭ��
		//LCD_ShowString(9,10,"NRF ERROR!",WHITE,DARKBLUE,16,0);// 9�� 10��

	//}

	LCD_Init();//LCD��ʼ�� PA0 PA1 PA2 PA3 PA4 PA5
	LCD_Fill(0,0,LCD_W,LCD_H,BLACK);//��������

	LCD_ShowString(29,5,"RX Data",WHITE,BLACK,16,0);// 49�� 5��
	LCD_ShowString(15,30,"S:",WHITE,BLACK,16,0);// 10�� 40��
	
	LCD_ShowString(10,60,"temp:",WHITE,BLACK,16,0);// 10�� 40��
	LCD_ShowString(108,60,"C",WHITE,BLACK,16,0);// 89�� 10��

	LCD_ShowString(10,75,"Humi:",WHITE,BLACK,16,0);// 10�� 40��
	LCD_ShowString(86,75,"%",WHITE,BLACK,16,0);// 89�� 10��

	LCD_ShowString(10,95,"P:",WHITE,BLACK,16,0);// 10�� 40��
	LCD_ShowString(70,95,"R:",WHITE,BLACK,16,0);// 10�� 40��

	//WEXTI_Init();//�ⲿ�жϳ�ʼ��
	Wpwm_Init(99,359);//2KHZ PWM  PA6 ,PA7 
	//WTimer_Init();//TIM2 ��ʱ����ʼ��
	wheels_GPIO_Init();//���IO PB0 PB1 PC14 PC15

    wheels_RL(0,0);//��ת
	while(1)
	{

		mpu_dmp_get_data(&pitch,&roll,&yaw);//�õ�������pitch �ͷ�����roll yaw���������
		
		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������
			

        //if(pitch<-5) wheels_RL(1,1);//��ת

        //if(pitch>5) wheels_RL(0,0);//��ת

        if((pitch>5) || (pitch<-5))
        {
            pitch=Get_Absolute_value((int)pitch); 
                  
            err=pitch;//������ ʵ��ֵ-0        
            //pwm=(int)(KP*err);//  
            //TIM_SetCompare1(TIM3, pwm);
            //TIM_SetCompare2(TIM3, pwm);   
        }
        else{//
            //pwm=0;

        }
        pwm++;
        pwm%=1000;
            TIM_SetCompare1(TIM3, pwm);
            TIM_SetCompare2(TIM3, pwm);           
     
        ROLL    =(int)roll;     if(ROLL<0) ROLL=-ROLL;//������  
        YAW     =(int)yaw;       if(YAW<0) YAW=-YAW;//�����

        //LCD_ShowIntNum(28,95,Get_Absolute_value(((int)pitch)),4,WHITE,BRED,16);//Pitch
        //LCD_ShowIntNum(88,95,ROLL,4,WHITE,RED,16);//ROll
		
               
        LCD_ShowIntNum(88,95,pwm,4,WHITE,RED,16);//ROll

		delay_ms(200);
       
        



#if 0
		//AHT10ReadData(&AHT10_temp,&AHT10_humi);//��ȡ��ʪ�ȴ�����������
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
                wheels_RL(1,1);//������ת
                LCD_ShowString(58,30,"GoGo!",BLACK,GREEN,16,0);// 10�� 40��
                pwm_R=pwm_R+30;
                pwm_L=pwm_L+30;
                if((pwm_R>=350) || (pwm_L>=350) ){ pwm_R=350;pwm_L=350; }
                TIM_SetCompare1(TIM3, pwm_R);
                TIM_SetCompare2(TIM3, pwm_L);
                break;

            case 2://����
                wheels_RL(0,0);//���ӷ�ת
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

#endif 

    }

}




//-----------------NEW--------------------



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
#include "Timer.h"
#include "EXTI.h"

#include "lcd.h"
//#include "pic.h" //LCD show picture
#include "24l01.h"//����ͨ��ģ�� NRF24L01
#include "aht10.h"//��ʪ�ȴ����� 
#include "L298N.h"//�������
//#include "TB6612FNG.h"//�������ģ��
#include "mpu6050.h" 
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 


/**
 * @brief ��������ֵ����
 * 
 * @param x �з��� (+ / - )����ֵ
 * @return uint16_t �޷��ŵ� ������
 */
uint16_t Get_Absolute_value(int x)
{
    if(x<0) return -x;
    else return x;
}



/**
 * @brief ������
 *STM32F103C8T6 ����IO
 PA 0 1 2 3 4 5 ,
 PB 0 1   3 4   6 7 8 10 11 13 14 15
 PC 14 15
 * @return int 0
 */
int main(void)
{
	delay_init();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	 //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
    float pitch,roll,yaw; 		//ŷ����
	short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
	short gyrox,gyroy,gyroz;	//������ԭʼ����

    MPU_Init();		//��ʼ��MPU6050 PB10 PB11
    while(mpu_dmp_init());//MPU6050 ��ʼ��ʧ�� �������������ѭ����

    int PITCH,ROLL,YAW;//�洢 ת��int �Ƕ�ֵ

	u8 tx_data[2]={0};
	u8 rx_data[2]={5};
	float AHT10_temp=0.0f;
	u8 AHT10_humi=0;
	unsigned int pwm_R=0,pwm_L=0,pwm=0;
	short x=0;
    u8 Tx=0;

    uint16_t KP=40,KD=5;
    uint16_t err=0;//���



	//AHT10Init();//��ʪ�ȴ�������ʼ�� PB3 PB4
	//AHT10Reset();

	//NRF24L01_Init(); //��ʼ������ͨ��ģ��NRF24L01 ,PB6 PB7 PB8, PB13, PB14, PB15
	//while(NRF24L01_Check()){//����Ƿ����������� 
		//ʧ���˾ͻ����������ѭ��
		//LCD_ShowString(9,10,"NRF ERROR!",WHITE,DARKBLUE,16,0);// 9�� 10��

	//}

	LCD_Init();//LCD��ʼ�� PA0 PA1 PA2 PA3 PA4 PA5
	LCD_Fill(0,0,LCD_W,LCD_H,BLACK);//��������

	//LCD_ShowString(29,5,"RX Data",WHITE,BLACK,16,0);// 49�� 5��
	LCD_ShowString(10,5,"Rx:",WHITE,BLACK,16,0);// 10�� 40��
	
	LCD_ShowString(10,25,"Temp :",WHITE,BLACK,16,0);// 10�� 25��
	LCD_ShowString(96,25,"C",WHITE,BLACK,16,0);// 96�� 25��

	LCD_ShowString(10,42,"Humi :",WHITE,BLACK,16,0);// 10�� 40��
	LCD_ShowString(96,42,"%",WHITE,BLACK,16,0);// 96�� 10��

	LCD_ShowString(10,65,"P:",WHITE,BLACK,16,0);// 10�� 40��
	LCD_ShowString(70,65,"R:",WHITE,BLACK,16,0);// 10�� 40��

    LCD_ShowString(10,85,"rn:",WHITE,BLACK,16,0);// 10�� 40��
	LCD_ShowString(70,85,"ln:",WHITE,BLACK,16,0);// 10�� 40��

	
	wheels_GPIO_Init();//���IO PB0 PB1 PC14 PC15

    wheels_RL(0,0);//��ת

    Extern_Interrupt_Init();//PA12 PA15 �ⲿ�жϼ��� ����

    Timer3_PWM_Init(1000,0);// 2KHZpwm PWM���ֵΪ1000 CH1PA6 CH2PA7
    
    Timer4_Timing_Init(1000,7200);//72MHZ ����7200��Ƶ Ƶ��Ϊ10KHZ ������������Ϊ100us ��ʱ���ж�ʱ��100*100us=10ms
	
    while(1)
	{

		mpu_dmp_get_data(&pitch,&roll,&yaw);//�õ�������pitch �ͷ�����roll yaw���������
		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������
        
        ROLL  = Get_Absolute_value((int)roll);    
        YAW   = Get_Absolute_value((int)yaw) ;     

        PITCH=Get_Absolute_value((int)pitch);//ȡ����ֵ��PITCH 		
        err=PITCH;//������ ʵ��ֵ-0    
        pwm=(int)(KP*err);//  

        if(pitch<0 && pitch>-10) wheels_RL(1,1);//��ת 

        if(pitch>10)   wheels_RL(0,0);//��ת 
        
        if((PITCH<5) || (PITCH>45)) pwm=0;//��ת


        
        TIM_SetCompare1(TIM3, pwm);
        TIM_SetCompare2(TIM3, pwm);   

  
        pwm%=1000;  

  


        LCD_ShowIntNum(28,65,PITCH,4,WHITE,BRED,16);//Pitch
        LCD_ShowIntNum(88,65,ROLL,4,WHITE,RED,16);//ROll

        LCD_ShowIntNum(32,85,RR_Value,4,WHITE,BLUE,16);//Pitch
        LCD_ShowIntNum(92,85,LL_Value,4,WHITE,BLUE,16);//ROll
		
               
		//delay_ms(90);
       
        

#if 0
		//AHT10ReadData(&AHT10_temp,&AHT10_humi);//��ȡ��ʪ�ȴ�����������
		LCD_ShowFloatNum1(58,25,AHT10_temp,4,WHITE,BLACK,16);
		LCD_ShowIntNum(58,42,AHT10_humi,3,WHITE,BLACK,16);
			           
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
                wheels_RL(1,1);//������ת
                LCD_ShowString(58,30,"GoGo!",BLACK,GREEN,16,0);// 10�� 40��
                pwm_R=pwm_R+30;
                pwm_L=pwm_L+30;
                if((pwm_R>=350) || (pwm_L>=350) ){ pwm_R=350;pwm_L=350; }
                TIM_SetCompare1(TIM3, pwm_R);
                TIM_SetCompare2(TIM3, pwm_L);
                break;

            case 2://����
                wheels_RL(0,0);//���ӷ�ת
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

#endif 

    }

}






#endif


































