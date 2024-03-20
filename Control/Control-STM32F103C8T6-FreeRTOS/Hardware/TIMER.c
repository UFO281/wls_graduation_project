#include "TIMER.h"




/**
 * @brief TIM1_Count_MODE_Init ETR=PA12
 * 
 */
void Timer1_Counting_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);// TIM1 挂载到了APB2时钟总线上
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    //开启端口重映射时钟AFIO 将TIM2CH1ETR原PA0 改为PA15 
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    //TIM2重映射1 CH1_ETR:PA15 , CH2:PB3 ,CH3: PA2, CH4 PA3 
	//GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE); 


	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//上拉输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;//
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

    //把TIM1的定时 时钟来源设置成内部时钟72MHZ  使用定时器模式时候用
	//TIM_InternalClockConfig(TIM1);
	
    //配置TIM1的外部时钟 TIM_ExtTRGPSC_OFF：不进行分频 上升沿或高电平计数 滤波器不用 0x00
	TIM_ETRClockMode2Config(TIM1, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0x00);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;//现在输入的时钟 是TIM2_CH1_ETR的外部时钟 0为不分频
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;//重复计数器 
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

	TIM_SetCounter(TIM1,0);

	TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);//配置成计数模式 关闭中断 计数到10就产生中断
	/*
    TIM_ClearFlag(TIM1, TIM_FLAG_Update);	
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//分配中断分组状态  2组 主优先级 0-3 子优先级 0-3
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//0-15
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);*/
	
	TIM_Cmd(TIM1, ENABLE);//开启TIM1外设
}



/**
 * @brief 定时器1 ISR
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
 * @brief TIM2_Timing_Init 配置为定时中断模式 向上计数
 * 
 * @param T_Count 计数值（最大为65536），向上计数模式，也是中断值 
 * @param PSC（最大为65536） 对72MHZ进行分频系数，为0 就是一分频 计数频率是72MHZ，为72的话就是72分频 计数频率为1MHZ
 * 
 * 定时器的中断时间 t= T_Count * (1/PSC ) 单位是秒
 * 
 * eg: Timer2_Timing_Init(100,7200); 就是对72MHZ计数频率进行7200分频 计数频率为10KHZ 
 * 一次计数周期时间为100us ，100 *100us=10ms，所以就是10ms进一次Timer2中断
 * 
 */
void Timer2_Timing_Init(uint16_t T_Count,uint16_t PSC)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//TIM2的时钟总线是APB1 所以这个函数是打开APB1上的TIM2时钟
	
	TIM_InternalClockConfig(TIM2);//把TIM2的定时 时钟来源设置成内部时钟72MHZ
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = (T_Count - 1);//定时中断值 自增计数到这个计数器就会清零 产生中断 由此控制中断频率 这个值的单位是分频后的时钟周期
	TIM_TimeBaseInitStructure.TIM_Prescaler =(PSC - 1);//对72MHZ时钟信号 进行多少分频 7200-1 是把72MHZ进行7200分频 10KHZ T=100us
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);//TIM2 配置
	
    TIM_SetCounter(TIM2,0);

	TIM_ClearFlag(TIM2, TIM_FLAG_Update);//防止一初始化就进入定时中断ISR
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//NVIC 中断管理员 进行中断优先级分组2 所有中断抢占优先级可以配置成0-3 子优先级也可以配置成0-3
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//抢占优先级可以配置成 0-15
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//子优先级 也可以配置成0
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM2, ENABLE);//开启TIM2外设
}




/**
 * @brief Timer2_配置成计数功能 其输入IO 为 CH1_ETR,PA0, 也可以选择复用PA15 IO作为外部时钟输入
 *  开启端口重映射时钟 将TIM2_CH1_ETR原PA0 改为PA15 
 * //TIM2重映射1方案 CH1_ETR:PA15 , CH2:PB3 ,CH3: PA2, CH4 PA3 
 * 
 */
void Timer2_Counting_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    //开启端口重映射时钟AFIO 将TIM2CH1ETR原PA0 改为PA15 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    //TIM2重映射1 CH1_ETR:PA15 , CH2:PB3 ,CH3: PA2, CH4 PA3 
	GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE); 


	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//上拉输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;//
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
    //把TIM2的定时 时钟来源设置成内部时钟72MHZ
	//TIM_InternalClockConfig(TIM2);

    //配置TIM2的外部时钟 TIM_ExtTRGPSC_OFF：不进行分频 上升沿或高电平计数 滤波器不用 0x00
	TIM_ETRClockMode2Config(TIM2, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0x00);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;//现在输入的时钟 是TIM2_CH1_ETR的外部时钟 0为不分频
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;//重复计数器 
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
	

	TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);//配置成计数模式 关闭中断 计数到10就产生中断
	/*
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);	
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);*/
	
	TIM_Cmd(TIM2, ENABLE);//开启TIM2外设
}

unsigned long long FreeRTOSRunTimeTicks=0;//FreeRTOS 时间统计所用的节拍计数器

/**
 * @brief 定时器2 ISR 1ms，
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
 * @brief 得到TIM2计数器的值
 * 
 * @return uint16_t 
 */
uint16_t Get_TIM2_Counter_Value(void)
{
	return TIM_GetCounter(TIM2);//得到TIM2的计数值
}


/**
 * @brief 初始化 TIM2 使其为 FreeRTOS 的时间统计提供时基
 * 
 */
void ConfigureTimeForRunTimeStats(void)
{
    FreeRTOSRunTimeTicks=0;

    /*就是对72MHZ计数频率进行7200分频 计数频率为10KHZ 
    一次计数周期时间为100us ，1000 *100us=100ms，
    所以就是1ms，进一次Timer2中断*/
    Timer2_Timing_Init(1000,7200); /*设置为系统始终定时器的100倍*/
    
}




/**
 * @brief TIM3_Timing_Init 配置为定时中断模式 向上计数
 * 
 * @param T_Count 计数值（最大为65536），向上计数模式，也是中断值 
 * @param PSC（最大为65536） 对72MHZ进行分频系数，为0 就是一分频 计数频率是72MHZ，为72的话就是72分频 计数频率为1MHZ
 * 
 * 定时器的中断时间 t= T_Count * (1/PSC ) 单位是秒
 * 
 * eg: Timer3_Timing_Init(100,7200); 就是对72MHZ计数频率进行7200分频 计数频率为10KHZ 
 * 一次计数周期时间为100us ，100 *100us=10ms，所以就是10ms进一次Timer3中断
 * 
 */
void Timer3_Timing_Init(uint16_t T_Count,uint16_t PSC)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);//TIM2的时钟总线是APB1 所以这个函数是打开APB1上的TIM2时钟
	
    //把TIM3的定时 时钟来源设置成内部时钟72MHZ
	TIM_InternalClockConfig(TIM3);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = (T_Count - 1);//定时中断值 自增计数到这个计数器就会清零 产生中断 由此控制中断频率 这个值的单位是分频后的时钟周期
	TIM_TimeBaseInitStructure.TIM_Prescaler =(PSC - 1);//对72MHZ时钟信号 进行多少分频 7200-1 是把72MHZ进行7200分频 10KHZ T=100us
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);//TIM3 配置
	
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);//防止一初始化就进入定时中断ISR
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//NVIC 中断管理员 进行中断优先级分组2 所有中断抢占优先级可以配置成0-15 子优先级也可以配置成0-15
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;//抢占优先级可以配置成 0-15
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//子优先级 也可以配置成0-3
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM3, ENABLE);//开启TIM3外设
}



/**
 * @brief Timer3_PWM_Init, CH1 PA6,CH2 PA7,CH3 PB0,CH4 PB1
 * 使用了CH1 PA6 ,CH2 PA7
 * PWM(f)=72MHZ/T_Count,前提是 PSC=0
 * PWM(Duty Cycle)=CCR/T_Count,
 * PWM占空比,PWM用1模式，向上计数：CNT<CCR时，CHX输出高电平 控制CCR比较输出值就可以控制高电平占空比
 * PWM分辨率，1/T_Count，建议设置成1000，分辨率为1/1000
 * 
 * @param T_Count 最大计数值 控制着PWM频率，分辨率 建议配置成1000
 * @param PSC 分频系数 0就是对72MHZ不分频
 * 
 *      eg: Timer3_PWM_Init(1000,72);// 1KHZpwm PWM最大值为1000 CH1PA6 CH2PA7
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
	
	TIM_InternalClockConfig(TIM3);//选择系统时钟72MH作为定时器时钟源
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = T_Count-1 ; //定时器自增计数模式 清零值 也是中断值
	TIM_TimeBaseInitStructure.TIM_Prescaler = PSC ;		//PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
	
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);//给结构体函数 赋初始值 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;//CNT<CCR 输出高电平
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;		//CCR 初始化输出比较值=0

    TIM_OC1Init(TIM3, &TIM_OCInitStructure);//初始化TIM3 PWM 通道1 CH1 PA6
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);//初始化TIM3 PWM 通道2 CH2 PA7
	
	TIM_Cmd(TIM3, ENABLE);
}




/**
 * @brief TIM3_PWM 通道的比较值  通道比较值/最大计数值 == PWM占空比
 * 
 * @param Compare1 通道1 比较值  PA6
 * @param Compare2 通道2 比较值  PA7
 */
void TIM3_PWM_SetCompare1_2(uint16_t Compare1,uint16_t Compare2)
{
	TIM_SetCompare1(TIM3, Compare1);//CH1_PA6 RR
	TIM_SetCompare2(TIM3, Compare2);//CH2_PA7 LL
}




/**
 * @brief 定时器3 ISR
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
 * @brief TIM4_Timing_Init 配置为定时中断模式 向上计数
 * 
 * @param T_Count 计数值（最大为65536），向上计数模式，也是中断值 
 * @param PSC（最大为65536） 对72MHZ进行分频系数，为0 就是一分频 计数频率是72MHZ，为72的话就是72分频 计数频率为1MHZ
 * 
 * 定时器的中断时间 t= T_Count * (1/PSC ) 单位是秒
 * 
 * eg: Timer4_Init(100,7200); 就是对72MHZ计数频率进行7200分频 计数频率为10KHZ 
 * 一次计数周期时间为100us ，100 *100us=10ms，所以就是10ms进一次Timer2中断
 * 
 */
void Timer4_Timing_Init(uint16_t T_Count,uint16_t PSC)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//TIM4的时钟总线是APB1 所以这个函数是打开APB1上的TIM2时钟
	
    //把TIM4的定时 时钟来源设置成内部时钟72MHZ
	TIM_InternalClockConfig(TIM4);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period = (T_Count - 1);//定时中断值 自增计数到这个计数器就会清零 产生中断 由此控制中断频率 这个值的单位是分频后的时钟周期
	TIM_TimeBaseInitStructure.TIM_Prescaler =(PSC - 1);//对72MHZ时钟信号 进行多少分频 7200-1 是把72MHZ进行7200分频 10KHZ T=100us
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);//TIM4 配置
	
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);//防止一初始化就进入定时中断ISR
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//NVIC 中断管理员 进行中断优先级分组2 所有中断抢占优先级可以配置成0-15 子优先级也可以配置成0-15
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;//抢占优先级可以配置成 0-15
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//子优先级 也可以配置成0
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM4, ENABLE);//开启TIM4外设
}




uint16_t RR_Value=0,LL_Value=0;


/**
 * @brief 定时器4 ISR 100ms
 * 
 */
void TIM4_IRQHandler(void)
{

	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
	{


        #if 0 //使用timer count++ 电机测速

                RR_Value=TIM_GetCounter(TIM1);//得到TIM1的计数值  R电机  A相 13为一圈 

                LL_Value=TIM_GetCounter(TIM2);//得到TIM2的计数值  L电机  A相 13为一圈

                TIM_SetCounter(TIM1,0);
                TIM_SetCounter(TIM2,0);

        #endif


        #if 1  //使用外部中断电机测速
                
                RR_Value=R_V/11;//100ms (转的上升沿数/11)=圈数 
                LL_Value=L_V/11; 
                R_V=0; 
                L_V=0;
        #endif

		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}

}
















