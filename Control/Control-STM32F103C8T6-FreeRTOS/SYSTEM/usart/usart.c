#include "sys.h"
#include "usart.h"	 
#include "string.h"
////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//FreeRTOS使用	  
#endif




//加入以下代码,支持printf函数,而不需要选择use MicroLIB	 
#if 1   
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 

#endif 

 
 



/**
 * @brief USART1初始化 
 * 波特率: 115200  ，
 * 数据长度: 8 bit  ，
 * 无校验 ，
 * 1位停止位，
 * 无流控，
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
	USART_InitStructure.USART_BaudRate=115200;//波特率设置 115200   
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//使用硬件流控制     
	USART_InitStructure.USART_Parity= USART_Parity_No;//不要奇偶校验     
	USART_InitStructure.USART_Mode=  USART_Mode_Rx | USART_Mode_Tx;// 发送模式 和 接收模式       
	USART_InitStructure.USART_StopBits=USART_StopBits_1;//停止位 为1 
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;   
	USART_Init(USART1,&USART_InitStructure);   
	

	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//指定串口中断源  RXNE接受寄存器满了标志位
	// NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//抢占优先级 设置0-3 子优先级 0-3
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel= USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(USART1,ENABLE); 
	

}




uint8_t  Rx_Data=0,Rx_Flag=0,Rx_Count=0;//接收单字节数据 和接收中断

char Tx_Pack[4]={0},Rx_Pack[64]={0};//发送 数据包 接收数据包

uint8_t Rx_State=0;//接收数据完成标志位

/**
 * @brief USART1 ISR 中断服务函数
 * 
 */
void USART1_IRQHandler(void)
{
    
    if( USART_GetITStatus (USART1,USART_IT_RXNE ) )//接收中断标志位
	{
		Rx_Flag=1;//自定义的 USART1  中断标志位
		Rx_Data=USART_ReceiveData(USART1);//接收到的数据 为一个字节
        Rx_Pack[Rx_Count]=Rx_Data;


        if( (char)Rx_Data=='\n' )
        {
           // Rx_Pack[Rx_Count]='\n';
            Rx_State=1;//接收到结束字符 表示接收完成    
        }

        Rx_Count++;
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);		
		
	}

}




/**
 * @brief USART2初始化 
 * 波特率: 115200  ，
 * 数据长度: 8 bit  ，
 * 无校验 ，
 * 1位停止位，
 * 无流控，
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
	USART_InitStructure.USART_BaudRate=115200;//波特率设置 115200   
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//使用硬件流控制     
	USART_InitStructure.USART_Parity= USART_Parity_No;//不要奇偶校验     
	USART_InitStructure.USART_Mode=  USART_Mode_Rx | USART_Mode_Tx;// 发送模式 和 接收模式       
	USART_InitStructure.USART_StopBits=USART_StopBits_1;//停止位 为1 
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;   
	USART_Init(USART2,&USART_InitStructure);   
	

	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);//指定串口中断源  RXNE接受寄存器满了标志位
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//抢占优先级 设置0-3 子优先级 0-3
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel= USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=2;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(USART2,ENABLE); 
	
}



/**
 * @brief USART2 ISR 中断服务函数
 * 
 */
void USART2_IRQHandler(void)
{
    
    if( USART_GetITStatus (USART2,USART_IT_RXNE ) )//接收中断标志位
	{
		Rx_Flag=1;//自定义的 USART1  中断标志位
		Rx_Data=USART_ReceiveData(USART2);//接收到的数据 为一个字节
        Rx_Pack[Rx_Count]=Rx_Data;

        if((char)Rx_Data=='#')
        {
           // Rx_Pack[Rx_Count]='\n';
            Rx_State=1;//接收到结束字符 表示接收完成    
        }

        Rx_Count++;
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);		
		
	}

}





/**
 * @brief USART3初始化 
 * 波特率: 115200  ，
 * 数据长度: 8 bit  ，
 * 无校验 ，
 * 1位停止位，
 * 无流控，
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
	USART_InitStructure.USART_BaudRate=115200;//波特率设置 115200   
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//使用硬件流控制     
	USART_InitStructure.USART_Parity= USART_Parity_No;//不要奇偶校验     
	USART_InitStructure.USART_Mode=  USART_Mode_Rx | USART_Mode_Tx;// 发送模式 和 接收模式       
	USART_InitStructure.USART_StopBits=USART_StopBits_1;//停止位 为1 
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;   
	USART_Init(USART3,&USART_InitStructure);   
	

	USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);//指定串口中断源  RXNE接受寄存器满了标志位
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//抢占优先级 设置0-3 子优先级 0-3
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel= USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=3;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(USART3,ENABLE); 
	
}



/**
 * @brief USART3 ISR 中断服务函数
 * 
 */
void USART3_IRQHandler(void)
{
    
    if( USART_GetITStatus (USART3,USART_IT_RXNE ) )//接收中断标志位
	{
		Rx_Flag=1;//自定义的 USART3  中断标志位
		Rx_Data=USART_ReceiveData(USART3);//接收到的数据 为一个字节
        Rx_Pack[Rx_Count]=Rx_Data;
        
        if((char)Rx_Data=='#')
        {
           // Rx_Pack[Rx_Count]='\n';
            Rx_State=1;//接收到结束字符 表示接收完成    
        }

        Rx_Count++;
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);		
		
	}

}



/**
 * @brief 发送1 Byte
 * 
 * @param Byte 1个字节数据
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
 * @brief 发送字符串
 * 
 * @param s 字符串
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
 * @brief 其他USART也可以使用的输出函数
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








u8 Uart1_Rx[5][4]={0};//串口接收数据仓库

/**
 * @brief 获取串口接收到的数据包
 * 
 * @return char* 
 */
void* Get_RxPackge(void)
{   

    // strcpy(Pa,Rx_Pack);
    memcpy(Uart1_Rx,Rx_Pack,Rx_Count);

    memset(Rx_Pack,0,Rx_Count);
    Rx_Count=0;

    return Uart1_Rx;

}



/**
 * @brief 检测串口接收数据包的接收状态
 * 
 * @return u8 1：接收完成  0：接受失败
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
 * @brief 获取中断中的接收数据 (1 Byte)
 * 
 * @return uint8_t 接收的单字节数据
 */
uint8_t Get_Usart1_RxData(void)
{
    return Rx_Data;
}




/**
 * @brief 获取 USART1 中断标志信号 单字节数据接收
 * 
 * @return uint8_t 1：产生接收数据中断了  0：未产生中断
 */
uint8_t Get_Usart1_RxFlag(void)
{
	if(Rx_Flag)
    {
		Rx_Flag=0; //接收中断数据标志位 检测到后 清0      
        return 1;    
    }
		
    return 0;
     
}	












