#ifndef __MPU6050_H
#define __MPU6050_H
#include "mpuiic.h"   												  	  


 
//MPU6050 AD0控制脚=GND,INT=GND
//#define MPU_AD0_CTRL			PAout(15)	//控制AD0电平,从而控制MPU地址

//#define MPU_ACCEL_OFFS_REG		0X06	//accel_offs寄存器,可读取版本号,寄存器手册未提到
//#define MPU_PROD_ID_REG			0X0C	//prod id寄存器,在寄存器手册未提到
#define MPU_SELF_TESTX_REG		0X0D	//自检寄存器X
#define MPU_SELF_TESTY_REG		0X0E	//自检寄存器Y
#define MPU_SELF_TESTZ_REG		0X0F	//自检寄存器Z
#define MPU_SELF_TESTA_REG		0X10	//自检寄存器A
#define MPU_SAMPLE_RATE_REG		0X19	//采样频率分频器
#define MPU_CFG_REG				0X1A	//配置寄存器
#define MPU_GYRO_CFG_REG		0X1B	//陀螺仪配置寄存器
#define MPU_ACCEL_CFG_REG		0X1C	//加速度计配置寄存器
#define MPU_MOTION_DET_REG		0X1F	//运动检测阀值设置寄存器
#define MPU_FIFO_EN_REG			0X23	//FIFO使能寄存器
#define MPU_I2CMST_CTRL_REG		0X24	//IIC主机控制寄存器
#define MPU_I2CSLV0_ADDR_REG	0X25	//IIC从机0器件地址寄存器
#define MPU_I2CSLV0_REG			0X26	//IIC从机0数据地址寄存器
#define MPU_I2CSLV0_CTRL_REG	0X27	//IIC从机0控制寄存器
#define MPU_I2CSLV1_ADDR_REG	0X28	//IIC从机1器件地址寄存器
#define MPU_I2CSLV1_REG			0X29	//IIC从机1数据地址寄存器
#define MPU_I2CSLV1_CTRL_REG	0X2A	//IIC从机1控制寄存器
#define MPU_I2CSLV2_ADDR_REG	0X2B	//IIC从机2器件地址寄存器
#define MPU_I2CSLV2_REG			0X2C	//IIC从机2数据地址寄存器
#define MPU_I2CSLV2_CTRL_REG	0X2D	//IIC从机2控制寄存器
#define MPU_I2CSLV3_ADDR_REG	0X2E	//IIC从机3器件地址寄存器
#define MPU_I2CSLV3_REG			0X2F	//IIC从机3数据地址寄存器
#define MPU_I2CSLV3_CTRL_REG	0X30	//IIC从机3控制寄存器
#define MPU_I2CSLV4_ADDR_REG	0X31	//IIC从机4器件地址寄存器
#define MPU_I2CSLV4_REG			0X32	//IIC从机4数据地址寄存器
#define MPU_I2CSLV4_DO_REG		0X33	//IIC从机4写数据寄存器
#define MPU_I2CSLV4_CTRL_REG	0X34	//IIC从机4控制寄存器
#define MPU_I2CSLV4_DI_REG		0X35	//IIC从机4读数据寄存器

#define MPU_I2CMST_STA_REG		0X36	//IIC主机状态寄存器
#define MPU_INTBP_CFG_REG		0X37	//中断/旁路设置寄存器
#define MPU_INT_EN_REG			0X38	//中断使能寄存器
#define MPU_INT_STA_REG			0X3A	//中断状态寄存器

#define MPU_ACCEL_XOUTH_REG		0X3B	//加速度值,X轴高8位寄存器
#define MPU_ACCEL_XOUTL_REG		0X3C	//加速度值,X轴低8位寄存器
#define MPU_ACCEL_YOUTH_REG		0X3D	//加速度值,Y轴高8位寄存器
#define MPU_ACCEL_YOUTL_REG		0X3E	//加速度值,Y轴低8位寄存器
#define MPU_ACCEL_ZOUTH_REG		0X3F	//加速度值,Z轴高8位寄存器
#define MPU_ACCEL_ZOUTL_REG		0X40	//加速度值,Z轴低8位寄存器

#define MPU_TEMP_OUTH_REG		0X41	//温度值高八位寄存器
#define MPU_TEMP_OUTL_REG		0X42	//温度值低8位寄存器

#define MPU_GYRO_XOUTH_REG		0X43	//陀螺仪值,X轴高8位寄存器
#define MPU_GYRO_XOUTL_REG		0X44	//陀螺仪值,X轴低8位寄存器
#define MPU_GYRO_YOUTH_REG		0X45	//陀螺仪值,Y轴高8位寄存器
#define MPU_GYRO_YOUTL_REG		0X46	//陀螺仪值,Y轴低8位寄存器
#define MPU_GYRO_ZOUTH_REG		0X47	//陀螺仪值,Z轴高8位寄存器
#define MPU_GYRO_ZOUTL_REG		0X48	//陀螺仪值,Z轴低8位寄存器

#define MPU_I2CSLV0_DO_REG		0X63	//IIC从机0数据寄存器
#define MPU_I2CSLV1_DO_REG		0X64	//IIC从机1数据寄存器
#define MPU_I2CSLV2_DO_REG		0X65	//IIC从机2数据寄存器
#define MPU_I2CSLV3_DO_REG		0X66	//IIC从机3数据寄存器

#define MPU_I2CMST_DELAY_REG	0X67	//IIC主机延时管理寄存器
#define MPU_SIGPATH_RST_REG		0X68	//信号通道复位寄存器
#define MPU_MDETECT_CTRL_REG	0X69	//运动检测控制寄存器
#define MPU_USER_CTRL_REG		0X6A	//用户控制寄存器
#define MPU_PWR_MGMT1_REG		0X6B	//电源管理寄存器1
#define MPU_PWR_MGMT2_REG		0X6C	//电源管理寄存器2 
#define MPU_FIFO_CNTH_REG		0X72	//FIFO计数寄存器高八位
#define MPU_FIFO_CNTL_REG		0X73	//FIFO计数寄存器低八位
#define MPU_FIFO_RW_REG			0X74	//FIFO读写寄存器
#define MPU_DEVICE_ID_REG		0X75	//器件ID寄存器
 
//如果AD0脚(9脚)接地,IIC地址为0X68(不包含最低位).
//如果接V3.3,则IIC地址为0X69(不包含最低位).
#define MPU_ADDR				0X68


////因为模块AD0默认接GND,所以转为读写地址后,为0XD1和0XD0(如果接VCC,则为0XD3和0XD2)  
//#define MPU_READ    0XD1
//#define MPU_WRITE   0XD0

u8 MPU_Init(void); 								//初始化MPU6050
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf);//IIC连续写
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf); //IIC连续读 
u8 MPU_Write_Byte(u8 reg,u8 data);				//IIC写一个字节
u8 MPU_Read_Byte(u8 reg);						//IIC读一个字节

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
 ALIENTEK精英STM32开发板实验30
 MPU6050六轴传感器 实验     
 技术支持：www.openedv.com
 淘宝店铺：http://eboard.taobao.com 
 关注微信公众平台微信号："正点原子"，免费获取STM32资料。
 广州市星翼电子科技有限公司  
 作者：正点原子 @ALIENTEK
************************************************/



//串口1发送1个字符 
//c:要发送的字符
void usart1_send_char(u8 c)
{   	
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); //循环发送,直到发送完毕   
	USART_SendData(USART1,c);  
} 
//传送数据给匿名四轴上位机软件(V2.6版本)
//fun:功能字. 0XA0~0XAF
//data:数据缓存区,最多28字节!!
//len:data区有效数据个数
void usart1_niming_report(u8 fun,u8*data,u8 len)
{
	u8 send_buf[32];
	u8 i;
	if(len>28)return;	//最多28字节数据 
	send_buf[len+3]=0;	//校验数置零
	send_buf[0]=0X88;	//帧头
	send_buf[1]=fun;	//功能字
	send_buf[2]=len;	//数据长度
	for(i=0;i<len;i++)send_buf[3+i]=data[i];			//复制数据
	for(i=0;i<len+3;i++)send_buf[len+3]+=send_buf[i];	//计算校验和	
	for(i=0;i<len+4;i++)usart1_send_char(send_buf[i]);	//发送数据到串口1 
}
//发送加速度传感器数据和陀螺仪数据
//aacx,aacy,aacz:x,y,z三个方向上面的加速度值
//gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
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
	usart1_niming_report(0XA1,tbuf,12);//自定义帧,0XA1
}	
//通过串口1上报结算后的姿态数据给电脑
//aacx,aacy,aacz:x,y,z三个方向上面的加速度值
//gyrox,gyroy,gyroz:x,y,z三个方向上面的陀螺仪值
//roll:横滚角.单位0.01度。 -18000 -> 18000 对应 -180.00  ->  180.00度
//pitch:俯仰角.单位 0.01度。-9000 - 9000 对应 -90.00 -> 90.00 度
//yaw:航向角.单位为0.1度 0 -> 3600  对应 0 -> 360.0度
void usart1_report_imu(short aacx,short aacy,short aacz,short gyrox,short gyroy,short gyroz,short roll,short pitch,short yaw)
{
	u8 tbuf[28]; 
	u8 i;
	for(i=0;i<28;i++)tbuf[i]=0;//清0
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
	usart1_niming_report(0XAF,tbuf,28);//飞控显示帧,0XAF
}  
 	
 int main(void)
 {	 
	u8 t=0,report=1;			//默认开启上报
	u8 key;
	float pitch,roll,yaw; 		//欧拉角
	short aacx,aacy,aacz;		//加速度传感器原始数据
	short gyrox,gyroy,gyroz;	//陀螺仪原始数据
	short temp;					//温度	
	 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(500000);	 	//串口初始化为500000
	delay_init();	//延时初始化 
	usmart_dev.init(72);		//初始化USMART
	LED_Init();		  			//初始化与LED连接的硬件接口
	KEY_Init();					//初始化按键
	LCD_Init();			   		//初始化LCD  
	MPU_Init();					//初始化MPU6050
 	POINT_COLOR=RED;			//设置字体为红色 
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
	POINT_COLOR=BLUE;//设置字体为蓝色 
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
			temp=MPU_Get_Temperature();	//得到温度值
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
			if(report)mpu6050_send_data(aacx,aacy,aacz,gyrox,gyroy,gyroz);//用自定义帧发送加速度和陀螺仪原始数据
			if(report)usart1_report_imu(aacx,aacy,aacz,gyrox,gyroy,gyroz,(int)(roll*100),(int)(pitch*100),(int)(yaw*10));
			if((t%10)==0)
			{ 
				if(temp<0)
				{
					LCD_ShowChar(30+48,200,'-',16,0);		//显示负号
					temp=-temp;		//转为正数
				}else LCD_ShowChar(30+48,200,' ',16,0);		//去掉负号 
				LCD_ShowNum(30+48+8,200,temp/100,3,16);		//显示整数部分	    
				LCD_ShowNum(30+48+40,200,temp%10,1,16);		//显示小数部分 
				temp=pitch*10;
				if(temp<0)
				{
					LCD_ShowChar(30+48,220,'-',16,0);		//显示负号
					temp=-temp;		//转为正数
				}else LCD_ShowChar(30+48,220,' ',16,0);		//去掉负号 
				LCD_ShowNum(30+48+8,220,temp/10,3,16);		//显示整数部分	    
				LCD_ShowNum(30+48+40,220,temp%10,1,16);		//显示小数部分 
				temp=roll*10;
				if(temp<0)
				{
					LCD_ShowChar(30+48,240,'-',16,0);		//显示负号
					temp=-temp;		//转为正数
				}else LCD_ShowChar(30+48,240,' ',16,0);		//去掉负号 
				LCD_ShowNum(30+48+8,240,temp/10,3,16);		//显示整数部分	    
				LCD_ShowNum(30+48+40,240,temp%10,1,16);		//显示小数部分 
				temp=yaw*10;
				if(temp<0)
				{
					LCD_ShowChar(30+48,260,'-',16,0);		//显示负号
					temp=-temp;		//转为正数
				}else LCD_ShowChar(30+48,260,' ',16,0);		//去掉负号 
				LCD_ShowNum(30+48+8,260,temp/10,3,16);		//显示整数部分	    
				LCD_ShowNum(30+48+40,260,temp%10,1,16);		//显示小数部分  
				t=0;
				LED0=!LED0;//LED闪烁
			}
		}
		t++; 
	} 	
}
 



//-------------------NEW-------------------
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
#include "TIMER\Timer.h"
#include "EXTI\EXTI.h"

#include "lcd.h"
//#include "pic.h" //LCD show picture
#include "24l01.h"//无线通信模块 NRF24L01
#include "aht10.h"//温湿度传感器 
#include "L298N.h"//电机驱动
#include "mpu6050.h" 
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 


/**
 * @brief 产生绝对值函数
 * 
 * @param x 有符号 (+ / - )整数值
 * @return uint16_t 无符号的 正整数
 */
uint16_t Get_Absolute_value(int x)
{
    if(x<0) return -x;
    else return x;
}



/**
 * @brief 主函数
 *STM32F103C8T6 已用IO
 PA 0 1 2 3 4 5 ,
 PB 0 1   3 4   6 7 8 10 11 13 14 15
 PC 14 15
 * @return int 0
 */
int main(void)
{
	delay_init();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
    float pitch,roll,yaw; 		//欧拉角
	short aacx,aacy,aacz;		//加速度传感器原始数据
	short gyrox,gyroy,gyroz;	//陀螺仪原始数据

    MPU_Init();		//初始化MPU6050 PB10 PB11
    while(mpu_dmp_init());//MPU6050 初始化失败 程序就陷入这死循环了

    int ROLL,YAW;//存储 转化int 角度值

	u8 tx_data[2]={0};
	u8 rx_data[2]={5};
	float AHT10_temp=0.0f;
	u8 AHT10_humi=0;
	unsigned int pwm_R=0,pwm_L=0,pwm=0;
	short x=0;
    u8 Tx=0;

    float KP=3.8,KD=0.5;
    uint16_t err=0;//误差



	//AHT10Init();//温湿度传感器初始化 PB3 PB4
	//AHT10Reset();

	//NRF24L01_Init(); //初始化无线通信模块NRF24L01 ,PB6 PB7 PB8, PB13, PB14, PB15
	//while(NRF24L01_Check()){//检查是否能正常工作 
		//失败了就会陷入这个死循环
		//LCD_ShowString(9,10,"NRF ERROR!",WHITE,DARKBLUE,16,0);// 9列 10行

	//}

	LCD_Init();//LCD初始化 PA0 PA1 PA2 PA3 PA4 PA5
	LCD_Fill(0,0,LCD_W,LCD_H,BLACK);//背景设置

	LCD_ShowString(29,5,"RX Data",WHITE,BLACK,16,0);// 49列 5行
	LCD_ShowString(15,30,"S:",WHITE,BLACK,16,0);// 10列 40行
	
	LCD_ShowString(10,60,"temp:",WHITE,BLACK,16,0);// 10列 40行
	LCD_ShowString(108,60,"C",WHITE,BLACK,16,0);// 89列 10行

	LCD_ShowString(10,75,"Humi:",WHITE,BLACK,16,0);// 10列 40行
	LCD_ShowString(86,75,"%",WHITE,BLACK,16,0);// 89列 10行

	LCD_ShowString(10,95,"P:",WHITE,BLACK,16,0);// 10列 40行
	LCD_ShowString(70,95,"R:",WHITE,BLACK,16,0);// 10列 40行

	//WEXTI_Init();//外部中断初始化
	Wpwm_Init(99,359);//2KHZ PWM  PA6 ,PA7 
	//WTimer_Init();//TIM2 定时器初始化
	wheels_GPIO_Init();//电机IO PB0 PB1 PC14 PC15

    wheels_RL(0,0);//正转
	while(1)
	{

		mpu_dmp_get_data(&pitch,&roll,&yaw);//得到俯仰角pitch 和翻滚角roll yaw航向角数据
		
		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
			

        //if(pitch<-5) wheels_RL(1,1);//反转

        //if(pitch>5) wheels_RL(0,0);//正转

        if((pitch>5) || (pitch<-5))
        {
            pitch=Get_Absolute_value((int)pitch); 
                  
            err=pitch;//误差等于 实际值-0        
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
     
        ROLL    =(int)roll;     if(ROLL<0) ROLL=-ROLL;//翻滚角  
        YAW     =(int)yaw;       if(YAW<0) YAW=-YAW;//航向角

        //LCD_ShowIntNum(28,95,Get_Absolute_value(((int)pitch)),4,WHITE,BRED,16);//Pitch
        //LCD_ShowIntNum(88,95,ROLL,4,WHITE,RED,16);//ROll
		
               
        LCD_ShowIntNum(88,95,pwm,4,WHITE,RED,16);//ROll

		delay_ms(200);
       
        



#if 0
		//AHT10ReadData(&AHT10_temp,&AHT10_humi);//读取温湿度传感器的数据
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
                wheels_RL(1,1);//轮子正转
                LCD_ShowString(58,30,"GoGo!",BLACK,GREEN,16,0);// 10列 40行
                pwm_R=pwm_R+30;
                pwm_L=pwm_L+30;
                if((pwm_R>=350) || (pwm_L>=350) ){ pwm_R=350;pwm_L=350; }
                TIM_SetCompare1(TIM3, pwm_R);
                TIM_SetCompare2(TIM3, pwm_L);
                break;

            case 2://后退
                wheels_RL(0,0);//轮子反转
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

#endif 

    }

}




//-----------------NEW--------------------



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
#include "Timer.h"
#include "EXTI.h"

#include "lcd.h"
//#include "pic.h" //LCD show picture
#include "24l01.h"//无线通信模块 NRF24L01
#include "aht10.h"//温湿度传感器 
#include "L298N.h"//电机驱动
//#include "TB6612FNG.h"//电机驱动模块
#include "mpu6050.h" 
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 


/**
 * @brief 产生绝对值函数
 * 
 * @param x 有符号 (+ / - )整数值
 * @return uint16_t 无符号的 正整数
 */
uint16_t Get_Absolute_value(int x)
{
    if(x<0) return -x;
    else return x;
}



/**
 * @brief 主函数
 *STM32F103C8T6 已用IO
 PA 0 1 2 3 4 5 ,
 PB 0 1   3 4   6 7 8 10 11 13 14 15
 PC 14 15
 * @return int 0
 */
int main(void)
{
	delay_init();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
    float pitch,roll,yaw; 		//欧拉角
	short aacx,aacy,aacz;		//加速度传感器原始数据
	short gyrox,gyroy,gyroz;	//陀螺仪原始数据

    MPU_Init();		//初始化MPU6050 PB10 PB11
    while(mpu_dmp_init());//MPU6050 初始化失败 程序就陷入这死循环了

    int PITCH,ROLL,YAW;//存储 转化int 角度值

	u8 tx_data[2]={0};
	u8 rx_data[2]={5};
	float AHT10_temp=0.0f;
	u8 AHT10_humi=0;
	unsigned int pwm_R=0,pwm_L=0,pwm=0;
	short x=0;
    u8 Tx=0;

    uint16_t KP=40,KD=5;
    uint16_t err=0;//误差



	//AHT10Init();//温湿度传感器初始化 PB3 PB4
	//AHT10Reset();

	//NRF24L01_Init(); //初始化无线通信模块NRF24L01 ,PB6 PB7 PB8, PB13, PB14, PB15
	//while(NRF24L01_Check()){//检查是否能正常工作 
		//失败了就会陷入这个死循环
		//LCD_ShowString(9,10,"NRF ERROR!",WHITE,DARKBLUE,16,0);// 9列 10行

	//}

	LCD_Init();//LCD初始化 PA0 PA1 PA2 PA3 PA4 PA5
	LCD_Fill(0,0,LCD_W,LCD_H,BLACK);//背景设置

	//LCD_ShowString(29,5,"RX Data",WHITE,BLACK,16,0);// 49列 5行
	LCD_ShowString(10,5,"Rx:",WHITE,BLACK,16,0);// 10列 40行
	
	LCD_ShowString(10,25,"Temp :",WHITE,BLACK,16,0);// 10列 25行
	LCD_ShowString(96,25,"C",WHITE,BLACK,16,0);// 96列 25行

	LCD_ShowString(10,42,"Humi :",WHITE,BLACK,16,0);// 10列 40行
	LCD_ShowString(96,42,"%",WHITE,BLACK,16,0);// 96列 10行

	LCD_ShowString(10,65,"P:",WHITE,BLACK,16,0);// 10列 40行
	LCD_ShowString(70,65,"R:",WHITE,BLACK,16,0);// 10列 40行

    LCD_ShowString(10,85,"rn:",WHITE,BLACK,16,0);// 10列 40行
	LCD_ShowString(70,85,"ln:",WHITE,BLACK,16,0);// 10列 40行

	
	wheels_GPIO_Init();//电机IO PB0 PB1 PC14 PC15

    wheels_RL(0,0);//正转

    Extern_Interrupt_Init();//PA12 PA15 外部中断计数 测速

    Timer3_PWM_Init(1000,0);// 2KHZpwm PWM最大值为1000 CH1PA6 CH2PA7
    
    Timer4_Timing_Init(1000,7200);//72MHZ 进行7200分频 频率为10KHZ 单个计数周期为100us 定时器中断时间100*100us=10ms
	
    while(1)
	{

		mpu_dmp_get_data(&pitch,&roll,&yaw);//得到俯仰角pitch 和翻滚角roll yaw航向角数据
		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
        
        ROLL  = Get_Absolute_value((int)roll);    
        YAW   = Get_Absolute_value((int)yaw) ;     

        PITCH=Get_Absolute_value((int)pitch);//取绝对值的PITCH 		
        err=PITCH;//误差等于 实际值-0    
        pwm=(int)(KP*err);//  

        if(pitch<0 && pitch>-10) wheels_RL(1,1);//反转 

        if(pitch>10)   wheels_RL(0,0);//正转 
        
        if((PITCH<5) || (PITCH>45)) pwm=0;//不转


        
        TIM_SetCompare1(TIM3, pwm);
        TIM_SetCompare2(TIM3, pwm);   

  
        pwm%=1000;  

  


        LCD_ShowIntNum(28,65,PITCH,4,WHITE,BRED,16);//Pitch
        LCD_ShowIntNum(88,65,ROLL,4,WHITE,RED,16);//ROll

        LCD_ShowIntNum(32,85,RR_Value,4,WHITE,BLUE,16);//Pitch
        LCD_ShowIntNum(92,85,LL_Value,4,WHITE,BLUE,16);//ROll
		
               
		//delay_ms(90);
       
        

#if 0
		//AHT10ReadData(&AHT10_temp,&AHT10_humi);//读取温湿度传感器的数据
		LCD_ShowFloatNum1(58,25,AHT10_temp,4,WHITE,BLACK,16);
		LCD_ShowIntNum(58,42,AHT10_humi,3,WHITE,BLACK,16);
			           
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
                wheels_RL(1,1);//轮子正转
                LCD_ShowString(58,30,"GoGo!",BLACK,GREEN,16,0);// 10列 40行
                pwm_R=pwm_R+30;
                pwm_L=pwm_L+30;
                if((pwm_R>=350) || (pwm_L>=350) ){ pwm_R=350;pwm_L=350; }
                TIM_SetCompare1(TIM3, pwm_R);
                TIM_SetCompare2(TIM3, pwm_L);
                break;

            case 2://后退
                wheels_RL(0,0);//轮子反转
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

#endif 

    }

}






#endif


































