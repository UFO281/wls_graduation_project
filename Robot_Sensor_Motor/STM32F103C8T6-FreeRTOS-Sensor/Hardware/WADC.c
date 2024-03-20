#include "wadc.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"


uint16_t ADC_CH[2]={0};


/**
 * @brief (ȱ�� ��ȡ�������ݺܲ��ȶ�)ADC+DMA���  ���� CH0 PA0 CH1 PA1 ��ͨ�� ͨ��
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
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//1 ��ADCģʽ
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//12���� �Ҷ���
	ADC_InitStructure.ADC_ExternalTrigConv =ADC_ExternalTrigConv_None ;//��ѡ���ⲿ����Դ ѡ���ڲ���
	ADC_InitStructure.ADC_ContinuousConvMode =ENABLE ;//����ת��ģʽ
	ADC_InitStructure.ADC_ScanConvMode =ENABLE ;//��ͨ��ɨ��ģʽ
	ADC_InitStructure.ADC_NbrOfChannel = 2;//ɨ���ͨ����
	ADC_Init(ADC1,&ADC_InitStructure);



    DMA_InitTypeDef DMA_InitStrture;
	
	DMA_InitStrture.DMA_PeripheralBaseAddr=(uint32_t)&ADC1->DR ;//�������ַ
	DMA_InitStrture.DMA_PeripheralDataSize  = DMA_PeripheralDataSize_HalfWord ;//�������ݿ�� �ֽڴ���
	DMA_InitStrture.DMA_PeripheralInc=DMA_PeripheralInc_Disable;//��ʼ���� NO
	
	DMA_InitStrture.DMA_MemoryBaseAddr=(uint32_t)ADC_CH ;//�洢����ʼ��ַ
	DMA_InitStrture.DMA_MemoryDataSize=DMA_MemoryDataSize_HalfWord;//���ݿ��
	DMA_InitStrture.DMA_MemoryInc=DMA_MemoryInc_Enable;//���� yes
	DMA_InitStrture.DMA_DIR=DMA_DIR_PeripheralSRC;//DMA���ݴ��䷽�����  ����Ĵ���ΪԴ����Ŀ�ķ���Ϊ�洢��
	
	DMA_InitStrture.DMA_BufferSize=2;//�����������  ת��һ�εݼ�һ��
	DMA_InitStrture.DMA_Mode=DMA_Mode_Circular;//�Ƿ�ʹ���Զ���װ
	DMA_InitStrture.DMA_M2M=DMA_M2M_Disable ;//Ӳ������
	DMA_InitStrture.DMA_Priority= DMA_Priority_Low ;
	DMA_Init( DMA1_Channel1 ,&DMA_InitStrture);//����DMA1 ͨ��1 
	DMA_Cmd(DMA1_Channel1,ENABLE);//����DMA	

	ADC_DMACmd(ADC1, ENABLE);
	ADC_Cmd(ADC1,ENABLE );
	
	ADC_ResetCalibration(ADC1);// ��λУ׼
	while( ADC_GetResetCalibrationStatus(ADC1)  );//�õ���λУ׼����״̬  0Ϊ���
	ADC_StartCalibration(ADC1);//��ʼӲ���Զ�У׼
	while( ADC_GetCalibrationStatus(ADC1)) ;//��ȡУ׼״̬
	
    ADC_SoftwareStartConvCmd(ADC1,ENABLE);//��ʼת��   
    
   

}






/**
 * @brief ADC ����DMA ��ȡ��ͨ����ֵ
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


uint16_t ADC_CH0=0,ADC_CH1=0,ADC_CH2=0;//�洢ADC ͨ�� CH0 CH1 CH2��ֵ

/**
 * @brief ��ȡָ��ͨ����ֵ
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
 * @return uint16_t ͨ���� ADC��ֵ
 */
uint16_t AD_GetValue(uint8_t ADC_Channel)
{
	ADC_RegularChannelConfig(ADC1, ADC_Channel, 1, ADC_SampleTime_55Cycles5);
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	return ADC_GetConversionValue(ADC1);
}
