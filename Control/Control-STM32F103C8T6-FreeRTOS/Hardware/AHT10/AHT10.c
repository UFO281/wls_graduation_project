#include "aht10.h"
#include "delay.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"


//--------------I2C ��ʼ��---------

//��ʼ��IIC
void IIC_Init(void)
{					     
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );	//ʹ��GPIOBʱ��
	   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB,GPIO_Pin_3|GPIO_Pin_4); 	//PB6,PB7 �����
}
//����IIC��ʼ�ź�
void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_us(4);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL=1; 
	IIC_SDA=1;//����I2C���߽����ź�
	delay_us(4);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
	IIC_SDA=1;delay_us(1);	   
	IIC_SCL=1;delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}
//������ACKӦ��		    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        //IIC_SDA=(txd&0x80)>>7;
		if((txd&0x80)>>7)
			IIC_SDA=1;
		else
			IIC_SDA=0;
		txd<<=1; 	  
		delay_us(2);   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL=1;
		delay_us(2); 
		IIC_SCL=0;	
		delay_us(2);
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        delay_us(2);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}



//--------------I2C ��ʼ��---------




/**
brief AHT10��ʼ������
param NONE
return NONE
*/
void AHT10Init()
{
	//IIC_Init();
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_3|GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //����Ϊ�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIOBA	
	GPIO_SetBits(GPIOB,GPIO_Pin_3|GPIO_Pin_4);				 //PA4 �����
	
	IIC_Start();
	IIC_Send_Byte(AHT10_ADDRESS);
	IIC_Send_Byte(0xe1);	
	IIC_Send_Byte(0x08);
	IIC_Send_Byte(0x00);
	IIC_Stop();	
	delay_ms(40);//��ʱ20ms�ô������ȶ�
}

/**
brief ���AHT10�Ƿ����
param NONE
return 0����  1������
*/
u8 AHT10Check(void)
{
	u8 ack=0;
	IIC_Start();
	IIC_Send_Byte(AHT10_ADDRESS);
	ack=IIC_Wait_Ack();
	IIC_Stop();	
	return ack;
}

/**
brief AHT10��λ
param NONE
return NONE
*/
void AHT10Reset(void)
{
	IIC_Start();
	IIC_Send_Byte(AHT10_WRITE);
	IIC_Wait_Ack();
	IIC_Send_Byte(0xba);
	IIC_Wait_Ack();
	IIC_Stop();	
}

/**
brief ���AHT10����ʪ������
param *temperature����Ҫ�������¶����ݣ�floatָ������,���ȷ�Χ+-0.3C
param *humidity����Ҫ������ʪ�����ݣ�u8ָ������,���ȷ�Χ+-2RH
return 0 ���������� 1������ʧ��
*/
u8 AHT10ReadData(float *temperature,u8 *humidity)
{
	u8 ack;
	u32 SRH=0,ST=0;
	u8 databuff[6];
	IIC_Start();
	IIC_Send_Byte(AHT10_WRITE);
	IIC_Wait_Ack();
	IIC_Send_Byte(0xac);
	IIC_Wait_Ack();
	IIC_Send_Byte(0x33);
	IIC_Wait_Ack();
	IIC_Send_Byte(0x00);
	IIC_Wait_Ack();
	IIC_Stop();	  
	delay_ms(80);//��ʱһ��ȴ����ݶ���
	IIC_Start();
	IIC_Send_Byte(AHT10_READ);
	IIC_Wait_Ack();
	ack=IIC_Read_Byte(1);
	if((ack&0x40)==0)
	{
		databuff[0]=IIC_Read_Byte(1);
		databuff[1]=IIC_Read_Byte(1);
		databuff[2]=IIC_Read_Byte(1);
		databuff[3]=IIC_Read_Byte(1);
		databuff[4]=IIC_Read_Byte(0);
		IIC_Stop();
		SRH=(databuff[0]<<12)+(databuff[1]<<4)+(databuff[2]>>4);
		ST=((databuff[2]&0X0f)<<16)+(databuff[3]<<8)+(databuff[4]);
		*humidity=(int)(SRH*100.0/1024/1024+0.5);
		*temperature=((int)(ST*2000.0/1024/1024+0.5))/10.0-50;
		return 0;
	}
	IIC_Stop();	
	return 1;
}
