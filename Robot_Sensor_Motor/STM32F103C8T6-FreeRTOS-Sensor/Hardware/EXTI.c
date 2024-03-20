#include "EXTI.h"


uint16_t R_V=0,L_V=0;


/**
 * @brief 外部中断定时器初始化配置 选择LIN12 PA12 LIN15 PA15
 * 
 */
void Extern_Interrupt_Init(void)
{

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);//AFIO 有IO复用选择功能，还有管理配置外部中断功能
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//上拉输入 默认高电平
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource12);//选择外部中断通道 PA12
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource15);//选择外部中断通道 PA15
	
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = EXTI_Line12|EXTI_Line15;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;//上升沿触发
	EXTI_Init(&EXTI_InitStructure);
	
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//分组2  抢占优先级配置 0-3 子优先级 0-3
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 12;//
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);

}




/**
 * @brief 外部中断 10-15的ISR
 * 
 */
void EXTI15_10_IRQHandler(void)
{

	if (EXTI_GetITStatus(EXTI_Line12) == SET)
	{
        R_V++;

		EXTI_ClearITPendingBit(EXTI_Line12);
	}

	if (EXTI_GetITStatus(EXTI_Line15) == SET)
	{
        L_V++;
		EXTI_ClearITPendingBit(EXTI_Line15);
	}


}
