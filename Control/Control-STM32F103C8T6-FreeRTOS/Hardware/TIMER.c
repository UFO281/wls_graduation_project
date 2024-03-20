#include "TIMER.h"




/**
 * @brief TIM1_Count_MODE_Init ETR=PA12
 * 
 */
void Timer1_Counting_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);// TIM1 ���ص���APB2ʱ��������
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    //�����˿���ӳ��ʱ��AFIO ��TIM2CH1ETRԭPA0 ��ΪPA15 
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    //TIM2��ӳ��1 CH1_ETR:PA15 , CH2:PB3 ,CH3: PA2, CH4 PA3 
	//GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE); 


	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//��������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;//
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

    //��TIM1�Ķ�ʱ ʱ����Դ���ó��ڲ�ʱ��72MHZ  ʹ�ö�ʱ��ģʽʱ����
	//TIM_InternalClockConfig(TIM1);
	
    //����TIM1���ⲿʱ�� TIM_ExtTRGPSC_OFF�������з�Ƶ �����ػ�ߵ�ƽ���� �˲������� 0x00
	TIM_ETRClockMode2Config(TIM1, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0x00);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;//���������ʱ�� ��TIM2_CH1_ETR���ⲿʱ�� 0Ϊ����Ƶ
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;//�ظ������� 
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

	TIM_SetCounter(TIM1,0);

	TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);//���óɼ���ģʽ �ر��ж� ������10�Ͳ����ж�
	/*
    TIM_ClearFlag(TIM1, TIM_FLAG_Update);	
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//�����жϷ���״̬  2�� �����ȼ� 0-3 �����ȼ� 0-3
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//0-15
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);*/
	
	TIM_Cmd(TIM1, ENABLE);//����TIM1����
}



/**
 * @brief ��ʱ��1 ISR
 * 
 */
void TIM1_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		//Num ++;
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}





/**
 * @brief TIM2_Timing_Init ����Ϊ��ʱ�ж�ģʽ ���ϼ���
 * 
 * @param T_Count ����ֵ�����Ϊ65536�������ϼ���ģʽ��Ҳ���ж�ֵ 
 * @param PSC�����Ϊ65536�� ��72MHZ���з�Ƶϵ����Ϊ0 ����һ��Ƶ ����Ƶ����72MHZ��Ϊ72�Ļ�����72��Ƶ ����Ƶ��Ϊ1MHZ
 * 
 * ��ʱ�����ж�ʱ�� t= T_Count * (1/PSC ) ��λ����
 * 
 * eg: Timer2_Timing_Init(100,7200); ���Ƕ�72MHZ����Ƶ�ʽ���7200��Ƶ ����Ƶ��Ϊ10KHZ 
 * һ�μ�������ʱ��Ϊ100us ��100 *100us=10ms�����Ծ���10ms��һ��Timer2�ж�
 * 
 */
void Timer2_Timing_Init(uint16_t T_Count,uint16_t PSC)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//TIM2��ʱ��������APB1 ������������Ǵ�APB1�ϵ�TIM2ʱ��
	
	TIM_InternalClockConfig(TIM2);//��TIM2�Ķ�ʱ ʱ����Դ���ó��ڲ�ʱ��72MHZ
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = (T_Count - 1);//��ʱ�ж�ֵ ��������������������ͻ����� �����ж� �ɴ˿����ж�Ƶ�� ���ֵ�ĵ�λ�Ƿ�Ƶ���ʱ������
	TIM_TimeBaseInitStructure.TIM_Prescaler =(PSC - 1);//��72MHZʱ���ź� ���ж��ٷ�Ƶ 7200-1 �ǰ�72MHZ����7200��Ƶ 10KHZ T=100us
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);//TIM2 ����
	
    TIM_SetCounter(TIM2,0);

	TIM_ClearFlag(TIM2, TIM_FLAG_Update);//��ֹһ��ʼ���ͽ��붨ʱ�ж�ISR
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//NVIC �жϹ���Ա �����ж����ȼ�����2 �����ж���ռ���ȼ��������ó�0-3 �����ȼ�Ҳ�������ó�0-3
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//��ռ���ȼ��������ó� 0-15
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//�����ȼ� Ҳ�������ó�0
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM2, ENABLE);//����TIM2����
}




/**
 * @brief Timer2_���óɼ������� ������IO Ϊ CH1_ETR,PA0, Ҳ����ѡ����PA15 IO��Ϊ�ⲿʱ������
 *  �����˿���ӳ��ʱ�� ��TIM2_CH1_ETRԭPA0 ��ΪPA15 
 * //TIM2��ӳ��1���� CH1_ETR:PA15 , CH2:PB3 ,CH3: PA2, CH4 PA3 
 * 
 */
void Timer2_Counting_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    //�����˿���ӳ��ʱ��AFIO ��TIM2CH1ETRԭPA0 ��ΪPA15 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    //TIM2��ӳ��1 CH1_ETR:PA15 , CH2:PB3 ,CH3: PA2, CH4 PA3 
	GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE); 


	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//��������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;//
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
    //��TIM2�Ķ�ʱ ʱ����Դ���ó��ڲ�ʱ��72MHZ
	//TIM_InternalClockConfig(TIM2);

    //����TIM2���ⲿʱ�� TIM_ExtTRGPSC_OFF�������з�Ƶ �����ػ�ߵ�ƽ���� �˲������� 0x00
	TIM_ETRClockMode2Config(TIM2, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0x00);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;//���������ʱ�� ��TIM2_CH1_ETR���ⲿʱ�� 0Ϊ����Ƶ
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;//�ظ������� 
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
	

	TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);//���óɼ���ģʽ �ر��ж� ������10�Ͳ����ж�
	/*
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);	
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);*/
	
	TIM_Cmd(TIM2, ENABLE);//����TIM2����
}

unsigned long long FreeRTOSRunTimeTicks=0;//FreeRTOS ʱ��ͳ�����õĽ��ļ�����

/**
 * @brief ��ʱ��2 ISR 1ms��
 * 
 */
void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		FreeRTOSRunTimeTicks++;
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}


/**
 * @brief �õ�TIM2��������ֵ
 * 
 * @return uint16_t 
 */
uint16_t Get_TIM2_Counter_Value(void)
{
	return TIM_GetCounter(TIM2);//�õ�TIM2�ļ���ֵ
}


/**
 * @brief ��ʼ�� TIM2 ʹ��Ϊ FreeRTOS ��ʱ��ͳ���ṩʱ��
 * 
 */
void ConfigureTimeForRunTimeStats(void)
{
    FreeRTOSRunTimeTicks=0;

    /*���Ƕ�72MHZ����Ƶ�ʽ���7200��Ƶ ����Ƶ��Ϊ10KHZ 
    һ�μ�������ʱ��Ϊ100us ��1000 *100us=100ms��
    ���Ծ���1ms����һ��Timer2�ж�*/
    Timer2_Timing_Init(1000,7200); /*����Ϊϵͳʼ�ն�ʱ����100��*/
    
}




/**
 * @brief TIM3_Timing_Init ����Ϊ��ʱ�ж�ģʽ ���ϼ���
 * 
 * @param T_Count ����ֵ�����Ϊ65536�������ϼ���ģʽ��Ҳ���ж�ֵ 
 * @param PSC�����Ϊ65536�� ��72MHZ���з�Ƶϵ����Ϊ0 ����һ��Ƶ ����Ƶ����72MHZ��Ϊ72�Ļ�����72��Ƶ ����Ƶ��Ϊ1MHZ
 * 
 * ��ʱ�����ж�ʱ�� t= T_Count * (1/PSC ) ��λ����
 * 
 * eg: Timer3_Timing_Init(100,7200); ���Ƕ�72MHZ����Ƶ�ʽ���7200��Ƶ ����Ƶ��Ϊ10KHZ 
 * һ�μ�������ʱ��Ϊ100us ��100 *100us=10ms�����Ծ���10ms��һ��Timer3�ж�
 * 
 */
void Timer3_Timing_Init(uint16_t T_Count,uint16_t PSC)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);//TIM2��ʱ��������APB1 ������������Ǵ�APB1�ϵ�TIM2ʱ��
	
    //��TIM3�Ķ�ʱ ʱ����Դ���ó��ڲ�ʱ��72MHZ
	TIM_InternalClockConfig(TIM3);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = (T_Count - 1);//��ʱ�ж�ֵ ��������������������ͻ����� �����ж� �ɴ˿����ж�Ƶ�� ���ֵ�ĵ�λ�Ƿ�Ƶ���ʱ������
	TIM_TimeBaseInitStructure.TIM_Prescaler =(PSC - 1);//��72MHZʱ���ź� ���ж��ٷ�Ƶ 7200-1 �ǰ�72MHZ����7200��Ƶ 10KHZ T=100us
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);//TIM3 ����
	
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);//��ֹһ��ʼ���ͽ��붨ʱ�ж�ISR
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//NVIC �жϹ���Ա �����ж����ȼ�����2 �����ж���ռ���ȼ��������ó�0-15 �����ȼ�Ҳ�������ó�0-15
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;//��ռ���ȼ��������ó� 0-15
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//�����ȼ� Ҳ�������ó�0-3
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM3, ENABLE);//����TIM3����
}



/**
 * @brief Timer3_PWM_Init, CH1 PA6,CH2 PA7,CH3 PB0,CH4 PB1
 * ʹ����CH1 PA6 ,CH2 PA7
 * PWM(f)=72MHZ/T_Count,ǰ���� PSC=0
 * PWM(Duty Cycle)=CCR/T_Count,
 * PWMռ�ձ�,PWM��1ģʽ�����ϼ�����CNT<CCRʱ��CHX����ߵ�ƽ ����CCR�Ƚ����ֵ�Ϳ��Կ��Ƹߵ�ƽռ�ձ�
 * PWM�ֱ��ʣ�1/T_Count���������ó�1000���ֱ���Ϊ1/1000
 * 
 * @param T_Count ������ֵ ������PWMƵ�ʣ��ֱ��� �������ó�1000
 * @param PSC ��Ƶϵ�� 0���Ƕ�72MHZ����Ƶ
 * 
 *      eg: Timer3_PWM_Init(1000,72);// 1KHZpwm PWM���ֵΪ1000 CH1PA6 CH2PA7
 * 
 */
void Timer3_PWM_Init(uint16_t T_Count,uint16_t PSC)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
//	GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE);
//	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;//GPIO_Pin_6 7
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	TIM_InternalClockConfig(TIM3);//ѡ��ϵͳʱ��72MH��Ϊ��ʱ��ʱ��Դ
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = T_Count-1 ; //��ʱ����������ģʽ ����ֵ Ҳ���ж�ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler = PSC ;		//PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
	
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);//���ṹ�庯�� ����ʼֵ 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//CNT<CCR ����ߵ�ƽ
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;		//CCR ��ʼ������Ƚ�ֵ=0

    TIM_OC1Init(TIM3, &TIM_OCInitStructure);//��ʼ��TIM3 PWM ͨ��1 CH1 PA6
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);//��ʼ��TIM3 PWM ͨ��2 CH2 PA7
	
	TIM_Cmd(TIM3, ENABLE);
}




/**
 * @brief TIM3_PWM ͨ���ıȽ�ֵ  ͨ���Ƚ�ֵ/������ֵ == PWMռ�ձ�
 * 
 * @param Compare1 ͨ��1 �Ƚ�ֵ  PA6
 * @param Compare2 ͨ��2 �Ƚ�ֵ  PA7
 */
void TIM3_PWM_SetCompare1_2(uint16_t Compare1,uint16_t Compare2)
{
	TIM_SetCompare1(TIM3, Compare1);//CH1_PA6 RR
	TIM_SetCompare2(TIM3, Compare2);//CH2_PA7 LL
}




/**
 * @brief ��ʱ��3 ISR
 * 
 */
void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
	{
		//Num ++;
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
}




/**
 * @brief TIM4_Timing_Init ����Ϊ��ʱ�ж�ģʽ ���ϼ���
 * 
 * @param T_Count ����ֵ�����Ϊ65536�������ϼ���ģʽ��Ҳ���ж�ֵ 
 * @param PSC�����Ϊ65536�� ��72MHZ���з�Ƶϵ����Ϊ0 ����һ��Ƶ ����Ƶ����72MHZ��Ϊ72�Ļ�����72��Ƶ ����Ƶ��Ϊ1MHZ
 * 
 * ��ʱ�����ж�ʱ�� t= T_Count * (1/PSC ) ��λ����
 * 
 * eg: Timer4_Init(100,7200); ���Ƕ�72MHZ����Ƶ�ʽ���7200��Ƶ ����Ƶ��Ϊ10KHZ 
 * һ�μ�������ʱ��Ϊ100us ��100 *100us=10ms�����Ծ���10ms��һ��Timer2�ж�
 * 
 */
void Timer4_Timing_Init(uint16_t T_Count,uint16_t PSC)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//TIM4��ʱ��������APB1 ������������Ǵ�APB1�ϵ�TIM2ʱ��
	
    //��TIM4�Ķ�ʱ ʱ����Դ���ó��ڲ�ʱ��72MHZ
	TIM_InternalClockConfig(TIM4);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = (T_Count - 1);//��ʱ�ж�ֵ ��������������������ͻ����� �����ж� �ɴ˿����ж�Ƶ�� ���ֵ�ĵ�λ�Ƿ�Ƶ���ʱ������
	TIM_TimeBaseInitStructure.TIM_Prescaler =(PSC - 1);//��72MHZʱ���ź� ���ж��ٷ�Ƶ 7200-1 �ǰ�72MHZ����7200��Ƶ 10KHZ T=100us
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);//TIM4 ����
	
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);//��ֹһ��ʼ���ͽ��붨ʱ�ж�ISR
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//NVIC �жϹ���Ա �����ж����ȼ�����2 �����ж���ռ���ȼ��������ó�0-15 �����ȼ�Ҳ�������ó�0-15
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;//��ռ���ȼ��������ó� 0-15
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//�����ȼ� Ҳ�������ó�0
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM4, ENABLE);//����TIM4����
}




uint16_t RR_Value=0,LL_Value=0;


/**
 * @brief ��ʱ��4 ISR 100ms
 * 
 */
void TIM4_IRQHandler(void)
{

	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
	{


        #if 0 //ʹ��timer count++ �������

                RR_Value=TIM_GetCounter(TIM1);//�õ�TIM1�ļ���ֵ  R���  A�� 13ΪһȦ 

                LL_Value=TIM_GetCounter(TIM2);//�õ�TIM2�ļ���ֵ  L���  A�� 13ΪһȦ

                TIM_SetCounter(TIM1,0);
                TIM_SetCounter(TIM2,0);

        #endif


        #if 1  //ʹ���ⲿ�жϵ������
                
                RR_Value=R_V/11;//100ms (ת����������/11)=Ȧ�� 
                LL_Value=L_V/11; 
                R_V=0; 
                L_V=0;
        #endif

		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}

}
















