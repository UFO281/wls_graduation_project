#include "can.h"
// #include "led.h"
// #include "delay.h"
// #include "usart.h"
// #include "stm32f10x_gpio.h"
// #include "stm32f10x_can.h"
// #include "misc.h"
// #include "stm32f10x_rcc.h"








/**
 * @brief CAN初始化
 * 
 * @param tsjw 重新同步跳跃时间单元.范围:CAN_SJW_1tq~ CAN_SJW_4tq
 * @param tbs2 时间段2的时间单元.   范围:CAN_BS2_1tq~CAN_BS2_8tq
 * @param tbs1 时间段1的时间单元.   范围:CAN_BS1_1tq ~CAN_BS1_16tq
 * @param brp 波特率分频器.范围:1~1024;  tq=(brp)*tpclk1
 *      波特率=Fpclk1/((tbs1+1+tbs2+1+1)*brp)
 * 
 * @param mode CAN_Mode_Normal  : 普通模式 （可以在总线上收发数据）    
 *              CAN_Mode_LoopBack: 回环模式 （只能往总线上发送数据，不能从总线上接收数据，发送的数据也立即发送到接收端了）
 *              CAN_Mode_Silent ：静默模式（只能从总线上接收数据，不能往总线发送数据，发送的数据也直接被送到接收端了）
 *              CAN_Mode_Silent_LoopBack：静默回环模式（不向总线发送数据，也不从总线接收数据，发送的数据直接送到接收端） 
 * 
 * Fpclk1的时钟在初始化的时候设置为36M,
 * 如果设置CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,CAN_Mode_LoopBack);
 * 则波特率为:36M/((8+9+1)*4)=500Kbps
 * 
 * @return u8  0：初始化OK
 *          其他：失败;
 */


/**
 * @brief 
 * 
 * @param mode CAN_Mode_Normal  : 普通模式 （可以在总线上收发数据）    
 *              CAN_Mode_LoopBack: 回环模式 （只能往总线上发送数据，不能从总线上接收数据，发送的数据也立即发送到接收端了）
 *              CAN_Mode_Silent ：静默模式（只能从总线上接收数据，不能往总线发送数据，发送的数据也直接被送到接收端了）
 *              CAN_Mode_Silent_LoopBack：静默回环模式（不向总线发送数据，也不从总线接收数据，发送的数据直接送到接收端） 
 * 
 * @return u8  0：初始化OK
 *           其他：失败;
 */
u8 CAN_Config(u8 mode)
{

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//使能PORTA时钟	                   											 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟	

    /**
     * @brief GPIO 配置
     * 
     */
    GPIO_InitTypeDef        GPIO_InitStructure; 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽
    GPIO_Init(GPIOA, &GPIO_InitStructure);		//初始化IO

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//上拉输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化IO
	  

    /**
     * @brief CAN外设配置
     * 
     */
    CAN_InitTypeDef        CAN_InitStructure;
    CAN_InitStructure.CAN_ABOM=ENABLE;	 //软件自动离线管理	 //
    CAN_InitStructure.CAN_AWUM=ENABLE;	 //睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)//
    CAN_InitStructure.CAN_Mode= mode; // CAN_Mode_Normal CAN_Mode_LoopBack 模式设置： mode:0,普通模式;1,回环模式; //
    CAN_InitStructure.CAN_NART=ENABLE;	//开启报文自动传送 
    CAN_InitStructure.CAN_RFLM=ENABLE;	 //报文锁定,新的不会覆盖旧的 // 
    CAN_InitStructure.CAN_TTCM=DISABLE;	//非时间触发通信模式  时间戳 //
    CAN_InitStructure.CAN_TXFP=DISABLE;	 //FIFO发送顺序 由报文ID决定 //

    /* 配置CAN 100Kbps波特率 */
  	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位  CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=CAN_BS1_9tq; //Tbs1=tbs1+1个时间单位CAN_BS1_1tq - CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=CAN_BS2_8tq;//Tbs2=tbs2+1个时间单位CAN_BS2_1tq - CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=20; //分频系数(Fdiv)为brp+1	
  	CAN_Init(CAN1, &CAN_InitStructure);  // 初始化CAN1 


    /**
     * @brief 过滤器配置
     * 
     */
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
  	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
    CAN_FilterInitStructure.CAN_FilterNumber=0;	 //过滤器0
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
   	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;/*列表过滤模式*/ 
  	CAN_FilterInitStructure.CAN_FilterIdHigh= ((PASS_ID<<3 | CAN_Id_Extended | CAN_RTR_Data ) &0xFFFF0000)<<16;////32位ID
  	CAN_FilterInitStructure.CAN_FilterIdLow=((PASS_ID<<3 | CAN_Id_Extended | CAN_RTR_Data ) &0xFFFF);
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xFFFF;//32位MASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xFFFF; /*掩码为1 则将过滤ID 和 被选择ID必须一致*/
  	CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化


#if CAN_RX0_INT_ENABLE

   	NVIC_InitTypeDef  NVIC_InitStructure;
    CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.		    
  	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;     // 主优先级为6
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);

#endif



	return 0;

}   
 

 
#if CAN_RX0_INT_ENABLE	//使能RX0中断

CanRxMsg RxMessage;
char CAN_INT_Flag=0;

/**
 * @brief 中断服务函数
 * 
 */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);/*会自动清除标志位*/
	CAN_INT_Flag=1;
}

#endif



/**
 * @brief can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)	
 * 
 * @param msg 数据指针,最大为8个字节
 * @param len 数据长度(最大为8)
 * @return u8 0: OK!
 *        other: failed!
 */
u8 Can_Send_Msg(u8* msg,u8 len)
{	
    u8 mbox;
    u16 i=0;
    CanTxMsg TxMessage;
    TxMessage.StdId=0; // 标准标识符 
    TxMessage.ExtId=PASS_ID; // 设置扩展标示符 
    TxMessage.RTR=CAN_RTR_Data; // 数据帧
    TxMessage.IDE=CAN_Id_Extended; //
    TxMessage.DLC=len;						// 要发送的数据长度
    
    for(i=0;i<len;i++)
    {
        TxMessage.Data[i]=msg[i];			          
    }
    i=0;
    
    mbox= CAN_Transmit(CAN1, &TxMessage);

    while( (CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed) && (i<0XFFF) )
    {
        i++;	//等待发送结束
    }
    
    if(i>=0XFFF)return 1;
    return 0;		

}



/**
 * @brief can口接收数据
 * 
 * @param buf 数据缓存区
 * @return u8 0: recive failed!
 *        other: recive data lenth
 */
u8 Can_Receive_Msg(u8 *buf)
{		   		   
    u8 i=0;
	u8 ret=0;
	

    while ( RxMessage.Data[i]!='\0')
    {
        buf[i]=RxMessage.Data[i];
        i++;

    }
    
	memset(RxMessage.Data,0,sizeof(RxMessage.Data));
	ret = RxMessage.DLC;
	RxMessage.DLC=0;

    return ret;	

}














