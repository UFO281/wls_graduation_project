#include "can.h"
// #include "led.h"
// #include "delay.h"
// #include "usart.h"
// #include "stm32f10x_gpio.h"
// #include "stm32f10x_can.h"
// #include "misc.h"
// #include "stm32f10x_rcc.h"








/**
 * @brief CAN��ʼ��
 * 
 * @param tsjw ����ͬ����Ծʱ�䵥Ԫ.��Χ:CAN_SJW_1tq~ CAN_SJW_4tq
 * @param tbs2 ʱ���2��ʱ�䵥Ԫ.   ��Χ:CAN_BS2_1tq~CAN_BS2_8tq
 * @param tbs1 ʱ���1��ʱ�䵥Ԫ.   ��Χ:CAN_BS1_1tq ~CAN_BS1_16tq
 * @param brp �����ʷ�Ƶ��.��Χ:1~1024;  tq=(brp)*tpclk1
 *      ������=Fpclk1/((tbs1+1+tbs2+1+1)*brp)
 * 
 * @param mode CAN_Mode_Normal  : ��ͨģʽ ���������������շ����ݣ�    
 *              CAN_Mode_LoopBack: �ػ�ģʽ ��ֻ���������Ϸ������ݣ����ܴ������Ͻ������ݣ����͵�����Ҳ�������͵����ն��ˣ�
 *              CAN_Mode_Silent ����Ĭģʽ��ֻ�ܴ������Ͻ������ݣ����������߷������ݣ����͵�����Ҳֱ�ӱ��͵����ն��ˣ�
 *              CAN_Mode_Silent_LoopBack����Ĭ�ػ�ģʽ���������߷������ݣ�Ҳ�������߽������ݣ����͵�����ֱ���͵����նˣ� 
 * 
 * Fpclk1��ʱ���ڳ�ʼ����ʱ������Ϊ36M,
 * �������CAN_Mode_Init(CAN_SJW_1tq,CAN_BS2_8tq,CAN_BS1_9tq,4,CAN_Mode_LoopBack);
 * ������Ϊ:36M/((8+9+1)*4)=500Kbps
 * 
 * @return u8  0����ʼ��OK
 *          ������ʧ��;
 */


/**
 * @brief 
 * 
 * @param mode CAN_Mode_Normal  : ��ͨģʽ ���������������շ����ݣ�    
 *              CAN_Mode_LoopBack: �ػ�ģʽ ��ֻ���������Ϸ������ݣ����ܴ������Ͻ������ݣ����͵�����Ҳ�������͵����ն��ˣ�
 *              CAN_Mode_Silent ����Ĭģʽ��ֻ�ܴ������Ͻ������ݣ����������߷������ݣ����͵�����Ҳֱ�ӱ��͵����ն��ˣ�
 *              CAN_Mode_Silent_LoopBack����Ĭ�ػ�ģʽ���������߷������ݣ�Ҳ�������߽������ݣ����͵�����ֱ���͵����նˣ� 
 * 
 * @return u8  0����ʼ��OK
 *           ������ʧ��;
 */
u8 CAN_Config(u8 mode)
{

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//ʹ��PORTAʱ��	                   											 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	

    /**
     * @brief GPIO ����
     * 
     */
    GPIO_InitTypeDef        GPIO_InitStructure; 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);		//��ʼ��IO

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��IO
	  

    /**
     * @brief CAN��������
     * 
     */
    CAN_InitTypeDef        CAN_InitStructure;
    CAN_InitStructure.CAN_ABOM=ENABLE;	 //����Զ����߹���	 //
    CAN_InitStructure.CAN_AWUM=ENABLE;	 //˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)//
    CAN_InitStructure.CAN_Mode= mode; // CAN_Mode_Normal CAN_Mode_LoopBack ģʽ���ã� mode:0,��ͨģʽ;1,�ػ�ģʽ; //
    CAN_InitStructure.CAN_NART=ENABLE;	//���������Զ����� 
    CAN_InitStructure.CAN_RFLM=ENABLE;	 //��������,�µĲ��Ḳ�Ǿɵ� // 
    CAN_InitStructure.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ  ʱ��� //
    CAN_InitStructure.CAN_TXFP=DISABLE;	 //FIFO����˳�� �ɱ���ID���� //

    /* ����CAN 100Kbps������ */
  	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;	//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ  CAN_SJW_1tq	 CAN_SJW_2tq CAN_SJW_3tq CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=CAN_BS1_9tq; //Tbs1=tbs1+1��ʱ�䵥λCAN_BS1_1tq - CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=CAN_BS2_8tq;//Tbs2=tbs2+1��ʱ�䵥λCAN_BS2_1tq - CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=20; //��Ƶϵ��(Fdiv)Ϊbrp+1	
  	CAN_Init(CAN1, &CAN_InitStructure);  // ��ʼ��CAN1 


    /**
     * @brief ����������
     * 
     */
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
    CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
  	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
    CAN_FilterInitStructure.CAN_FilterNumber=0;	 //������0
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32λ 
   	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;/*�б����ģʽ*/ 
  	CAN_FilterInitStructure.CAN_FilterIdHigh= ((PASS_ID<<3 | CAN_Id_Extended | CAN_RTR_Data ) &0xFFFF0000)<<16;////32λID
  	CAN_FilterInitStructure.CAN_FilterIdLow=((PASS_ID<<3 | CAN_Id_Extended | CAN_RTR_Data ) &0xFFFF);
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xFFFF;//32λMASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0xFFFF; /*����Ϊ1 �򽫹���ID �� ��ѡ��ID����һ��*/
  	CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��


#if CAN_RX0_INT_ENABLE

   	NVIC_InitTypeDef  NVIC_InitStructure;
    CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.		    
  	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;     // �����ȼ�Ϊ6
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);

#endif



	return 0;

}   
 

 
#if CAN_RX0_INT_ENABLE	//ʹ��RX0�ж�

CanRxMsg RxMessage;
char CAN_INT_Flag=0;

/**
 * @brief �жϷ�����
 * 
 */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);/*���Զ������־λ*/
	CAN_INT_Flag=1;
}

#endif



/**
 * @brief can����һ������(�̶���ʽ:IDΪ0X12,��׼֡,����֡)	
 * 
 * @param msg ����ָ��,���Ϊ8���ֽ�
 * @param len ���ݳ���(���Ϊ8)
 * @return u8 0: OK!
 *        other: failed!
 */
u8 Can_Send_Msg(u8* msg,u8 len)
{	
    u8 mbox;
    u16 i=0;
    CanTxMsg TxMessage;
    TxMessage.StdId=0; // ��׼��ʶ�� 
    TxMessage.ExtId=PASS_ID; // ������չ��ʾ�� 
    TxMessage.RTR=CAN_RTR_Data; // ����֡
    TxMessage.IDE=CAN_Id_Extended; //
    TxMessage.DLC=len;						// Ҫ���͵����ݳ���
    
    for(i=0;i<len;i++)
    {
        TxMessage.Data[i]=msg[i];			          
    }
    i=0;
    
    mbox= CAN_Transmit(CAN1, &TxMessage);

    while( (CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed) && (i<0XFFF) )
    {
        i++;	//�ȴ����ͽ���
    }
    
    if(i>=0XFFF)return 1;
    return 0;		

}



/**
 * @brief can�ڽ�������
 * 
 * @param buf ���ݻ�����
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














