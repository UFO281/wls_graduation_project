#include "wadc.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"


uint16_t ADC_CH[2]={0};


/**
 * @brief (缺点 读取到的数据很不稳定)ADC+DMA配合  配置 CH0 PA0 CH1 PA1 单通道 通过
 * 
 */
void WADC_DMA_Init(void)
{

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);//open DMA1 clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE );//OPEN ADC1 on APB2 clock 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//open GPIOA clock
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);//ADC CLOCK =72MHZ/6=12MHZ
	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_AIN ;// input analog signal
	GPIO_InitStructure.GPIO_Pin= GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	ADC_RegularChannelConfig( ADC1,ADC_Channel_0,1,ADC_SampleTime_239Cycles5 );//RANK 1 =chanel_0
	
	ADC_RegularChannelConfig( ADC1,ADC_Channel_1,2,ADC_SampleTime_239Cycles5 );//RANK 2 =chanel_1
	
	
	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//1 个ADC模式
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//12数据 右对齐
	ADC_InitStructure.ADC_ExternalTrigConv =ADC_ExternalTrigConv_None ;//不选择外部触发源 选择内部的
	ADC_InitStructure.ADC_ContinuousConvMode =ENABLE ;//连续转换模式
	ADC_InitStructure.ADC_ScanConvMode =ENABLE ;//多通道扫描模式
	ADC_InitStructure.ADC_NbrOfChannel = 2;//扫描的通道数
	ADC_Init(ADC1,&ADC_InitStructure);



    DMA_InitTypeDef DMA_InitStrture;
	
	DMA_InitStrture.DMA_PeripheralBaseAddr=(uint32_t)&ADC1->DR ;//外设基地址
	DMA_InitStrture.DMA_PeripheralDataSize  = DMA_PeripheralDataSize_HalfWord ;//外设数据宽度 字节传输
	DMA_InitStrture.DMA_PeripheralInc=DMA_PeripheralInc_Disable;//起始自增 NO
	
	DMA_InitStrture.DMA_MemoryBaseAddr=(uint32_t)ADC_CH ;//存储器起始地址
	DMA_InitStrture.DMA_MemoryDataSize=DMA_MemoryDataSize_HalfWord;//数据宽度
	DMA_InitStrture.DMA_MemoryInc=DMA_MemoryInc_Enable;//自增 yes
	DMA_InitStrture.DMA_DIR=DMA_DIR_PeripheralSRC;//DMA数据传输方向控制  外设寄存器为源方向，目的方向为存储器
	
	DMA_InitStrture.DMA_BufferSize=2;//传输计数次数  转运一次递减一次
	DMA_InitStrture.DMA_Mode=DMA_Mode_Circular;//是否使用自动重装
	DMA_InitStrture.DMA_M2M=DMA_M2M_Disable ;//硬件触发
	DMA_InitStrture.DMA_Priority= DMA_Priority_Low ;
	DMA_Init( DMA1_Channel1 ,&DMA_InitStrture);//配置DMA1 通道1 
	DMA_Cmd(DMA1_Channel1,ENABLE);//开启DMA	

	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1,ENABLE );
	
	ADC_ResetCalibration(ADC1);// 复位校准
	while( ADC_GetResetCalibrationStatus(ADC1)  );//得到复位校准重置状态  0为完成
	ADC_StartCalibration(ADC1);//开始硬件自动校准
	while( ADC_GetCalibrationStatus(ADC1)) ;//获取校准状态
	
    ADC_SoftwareStartConvCmd(ADC1,ENABLE);//开始转换   
    
   

}






/**
 * @brief ADC 不用DMA 获取单通道的值
 * 
 */
void WADC_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
		
	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);
	
	ADC_Cmd(ADC1, ENABLE);
	
	ADC_ResetCalibration(ADC1);
	while (ADC_GetResetCalibrationStatus(ADC1) == SET);
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1) == SET);
}


uint16_t ADC_CH0=0,ADC_CH1=0,ADC_CH2=0;//存储ADC 通道 CH0 CH1 CH2的值

/**
 * @brief 获取指定通道的值
 * 
 * @param ADC_Channel the ADC channel to configure. 
  *   This parameter can be one of the following values:
  *     @arg ADC_Channel_0: ADC Channel0 selected
  *     @arg ADC_Channel_1: ADC Channel1 selected
  *     @arg ADC_Channel_2: ADC Channel2 selected
  *     @arg ADC_Channel_3: ADC Channel3 selected
  *     @arg ADC_Channel_4: ADC Channel4 selected
  *     @arg ADC_Channel_5: ADC Channel5 selected
  *     @arg ADC_Channel_6: ADC Channel6 selected
  *     @arg ADC_Channel_7: ADC Channel7 selected
  *     @arg ADC_Channel_8: ADC Channel8 selected
  *     @arg ADC_Channel_9: ADC Channel9 selected
  *     @arg ADC_Channel_10: ADC Channel10 selected
  *     @arg ADC_Channel_11: ADC Channel11 selected
  *     @arg ADC_Channel_12: ADC Channel12 selected
  *     @arg ADC_Channel_13: ADC Channel13 selected
  *     @arg ADC_Channel_14: ADC Channel14 selected
 * @return uint16_t 通道的 ADC的值
 */
uint16_t AD_GetValue(uint8_t ADC_Channel)
{
	ADC_RegularChannelConfig(ADC1, ADC_Channel, 1, ADC_SampleTime_55Cycles5);
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	return ADC_GetConversionValue(ADC1);
}
