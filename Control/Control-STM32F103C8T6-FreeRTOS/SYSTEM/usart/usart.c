#include "sys.h"
#include "usart.h"	 
#include "string.h"
////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//FreeRTOSʹ��	  
#endif




//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	 
#if 1   
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 

#endif 

 
 



/**
 * @brief USART1��ʼ�� 
 * ������: 115200  ��
 * ���ݳ���: 8 bit  ��
 * ��У�� ��
 * 1λֹͣλ��
 * �����أ�
 * 
 */
void WUsart1_Init(void)
{

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE );//OPEN USART on APB2 clock 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//open GPIOA clock
	

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_AF_PP ;// TX input analog signal
	GPIO_InitStructure.GPIO_Pin= GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode= GPIO_Mode_IPU ;// RX input UP signal
	GPIO_InitStructure.GPIO_Pin= GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitTypeDef USART_InitStructure;  
	USART_InitStructure.USART_BaudRate=115200;//���������� 115200   
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//ʹ��Ӳ��������     
	USART_InitStructure.USART_Parity= USART_Parity_No;//��Ҫ��żУ��     
	USART_InitStructure.USART_Mode=  USART_Mode_Rx | USART_Mode_Tx;// ����ģʽ �� ����ģʽ       
	USART_InitStructure.USART_StopBits=USART_StopBits_1;//ֹͣλ Ϊ1 
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;   
	USART_Init(USART1,&USART_InitStructure);   
	

	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//ָ�������ж�Դ  RXNE���ܼĴ������˱�־λ
	// NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//��ռ���ȼ� ����0-3 �����ȼ� 0-3
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel= USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(USART1,ENABLE); 
	

}




uint8_t  Rx_Data=0,Rx_Flag=0,Rx_Count=0;//���յ��ֽ����� �ͽ����ж�

char Tx_Pack[4]={0},Rx_Pack[64]={0};//���� ���ݰ� �������ݰ�

uint8_t Rx_State=0;//����������ɱ�־λ

/**
 * @brief USART1 ISR �жϷ�����
 * 
 */
void USART1_IRQHandler(void)
{
    
    if( USART_GetITStatus (USART1,USART_IT_RXNE ) )//�����жϱ�־λ
	{
		Rx_Flag=1;//�Զ���� USART1  �жϱ�־λ
		Rx_Data=USART_ReceiveData(USART1);//���յ������� Ϊһ���ֽ�
        Rx_Pack[Rx_Count]=Rx_Data;


        if( (char)Rx_Data=='\n' )
        {
           // Rx_Pack[Rx_Count]='\n';
            Rx_State=1;//���յ������ַ� ��ʾ�������    
        }

        Rx_Count++;
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);		
		
	}

}




/**
 * @brief USART2��ʼ�� 
 * ������: 115200  ��
 * ���ݳ���: 8 bit  ��
 * ��У�� ��
 * 1λֹͣλ��
 * �����أ�
 * 
 */
void WUsart2_Init(void)
{

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE );//OPEN USART2 on APB1 clock 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//open GPIOA clock
	

	GPIO_InitTypeDef GPIO_InitStructure;  
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_AF_PP ;// ouput analog signal  
	GPIO_InitStructure.GPIO_Pin= GPIO_Pin_2;  
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;  
	GPIO_Init(GPIOA, &GPIO_InitStructure);   

    GPIO_InitStructure.GPIO_Mode= GPIO_Mode_IPU ;// input UP signal
	GPIO_InitStructure.GPIO_Pin= GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitTypeDef USART_InitStructure;  
	USART_InitStructure.USART_BaudRate=115200;//���������� 115200   
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//ʹ��Ӳ��������     
	USART_InitStructure.USART_Parity= USART_Parity_No;//��Ҫ��żУ��     
	USART_InitStructure.USART_Mode=  USART_Mode_Rx | USART_Mode_Tx;// ����ģʽ �� ����ģʽ       
	USART_InitStructure.USART_StopBits=USART_StopBits_1;//ֹͣλ Ϊ1 
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;   
	USART_Init(USART2,&USART_InitStructure);   
	

	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);//ָ�������ж�Դ  RXNE���ܼĴ������˱�־λ
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//��ռ���ȼ� ����0-3 �����ȼ� 0-3
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel= USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=2;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(USART2,ENABLE); 
	
}



/**
 * @brief USART2 ISR �жϷ�����
 * 
 */
void USART2_IRQHandler(void)
{
    
    if( USART_GetITStatus (USART2,USART_IT_RXNE ) )//�����жϱ�־λ
	{
		Rx_Flag=1;//�Զ���� USART1  �жϱ�־λ
		Rx_Data=USART_ReceiveData(USART2);//���յ������� Ϊһ���ֽ�
        Rx_Pack[Rx_Count]=Rx_Data;

        if((char)Rx_Data=='#')
        {
           // Rx_Pack[Rx_Count]='\n';
            Rx_State=1;//���յ������ַ� ��ʾ�������    
        }

        Rx_Count++;
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);		
		
	}

}





/**
 * @brief USART3��ʼ�� 
 * ������: 115200  ��
 * ���ݳ���: 8 bit  ��
 * ��У�� ��
 * 1λֹͣλ��
 * �����أ�
 * 
 */
void WUsart3_Init(void)
{

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE );//OPEN USART3 on APB1 clock 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//open GPIOA clock
	

	GPIO_InitTypeDef GPIO_InitStructure;  
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_AF_PP ;// ouput analog signal  
	GPIO_InitStructure.GPIO_Pin= GPIO_Pin_10;  
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;  
	GPIO_Init(GPIOB, &GPIO_InitStructure);   

    GPIO_InitStructure.GPIO_Mode= GPIO_Mode_IPU ;// input UP signal
	GPIO_InitStructure.GPIO_Pin= GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	USART_InitTypeDef USART_InitStructure;  
	USART_InitStructure.USART_BaudRate=115200;//���������� 115200   
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//ʹ��Ӳ��������     
	USART_InitStructure.USART_Parity= USART_Parity_No;//��Ҫ��żУ��     
	USART_InitStructure.USART_Mode=  USART_Mode_Rx | USART_Mode_Tx;// ����ģʽ �� ����ģʽ       
	USART_InitStructure.USART_StopBits=USART_StopBits_1;//ֹͣλ Ϊ1 
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;   
	USART_Init(USART3,&USART_InitStructure);   
	

	USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);//ָ�������ж�Դ  RXNE���ܼĴ������˱�־λ
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//��ռ���ȼ� ����0-3 �����ȼ� 0-3
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel= USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=3;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(USART3,ENABLE); 
	
}



/**
 * @brief USART3 ISR �жϷ�����
 * 
 */
void USART3_IRQHandler(void)
{
    
    if( USART_GetITStatus (USART3,USART_IT_RXNE ) )//�����жϱ�־λ
	{
		Rx_Flag=1;//�Զ���� USART3  �жϱ�־λ
		Rx_Data=USART_ReceiveData(USART3);//���յ������� Ϊһ���ֽ�
        Rx_Pack[Rx_Count]=Rx_Data;
        
        if((char)Rx_Data=='#')
        {
           // Rx_Pack[Rx_Count]='\n';
            Rx_State=1;//���յ������ַ� ��ʾ�������    
        }

        Rx_Count++;
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);		
		
	}

}



/**
 * @brief ����1 Byte
 * 
 * @param Byte 1���ֽ�����
 */
void Send_Byte(char Byte)
{

#if (W_USART1==1)
	USART_SendData(USART1,Byte);
	while( USART_GetFlagStatus  (USART1,USART_FLAG_TXE) == RESET ); 
#endif

#if (W_USART2==1)
    USART_SendData(USART2,Byte);
	while( USART_GetFlagStatus  (USART2,USART_FLAG_TXE) == RESET ); 
#endif

#if (W_USART3==1)
    USART_SendData(USART3,Byte);
	while( USART_GetFlagStatus  (USART3,USART_FLAG_TXE) == RESET ); 
#endif


}




/**
 * @brief �����ַ���
 * 
 * @param s �ַ���
 */
void Send_String(char *s )
{
    /*do
    {
      Send_Byte(*s);

    } while (*s++);*/

    while(*s) Send_Byte(*s++);
    
}


int fputc(int ch,FILE *f)
{
    Send_Byte(ch);
    return ch;

}


/**
 * @brief ����USARTҲ����ʹ�õ��������
 * 
 * @param format 
 * @param ... 
 */
void S_Printf(char *format,...)
{
    char s[100]={0};
    va_list arg;
    va_start(arg,format);
    vsprintf(s,format,arg);
    va_end(arg);
    Send_String(s);

}








/**
 * @brief ��ȡ���ڽ��յ������ݰ�
 * 
 * @return char* 
 */
char* Get_RxPackge(void)
{   
    char Pa[64]={0};//���ڽ������ݲֿ�

    strcpy(Pa,Rx_Pack);

    memset(Rx_Pack,0,Rx_Count);
    Rx_Count=0;

    return Pa;

}



/**
 * @brief ��⴮�ڽ������ݰ��Ľ���״̬
 * 
 * @return u8 1���������  0������ʧ��
 */
u8 Get_Rx_Packge_State(void)
{
    if(Rx_State)
    {
        Rx_State=0;
        return 1;
    }
    return 0;

}



/**
 * @brief ��ȡ�ж��еĽ������� (1 Byte)
 * 
 * @return uint8_t ���յĵ��ֽ�����
 */
uint8_t Get_Usart1_RxData(void)
{
    return Rx_Data;
}




/**
 * @brief ��ȡ USART1 �жϱ�־�ź� ���ֽ����ݽ���
 * 
 * @return uint8_t 1���������������ж���  0��δ�����ж�
 */
uint8_t Get_Usart1_RxFlag(void)
{
	if(Rx_Flag)
    {
		Rx_Flag=0; //�����ж����ݱ�־λ ��⵽�� ��0      
        return 1;    
    }
		
    return 0;
     
}	












