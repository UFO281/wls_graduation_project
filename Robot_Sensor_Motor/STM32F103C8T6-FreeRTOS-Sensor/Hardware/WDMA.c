#include "WDMA.h"

int MyDMA_Count=0;


/**
 * @brief DMA Init Config one channel  ,
 * transform from PeripheralBaseAddr to MemoryBaseAddr
 * 
 * @param PeripheralBaseAddr 外设地址
 * @param MemoryBaseAddr 存储器地址
 * @param count 转运次数
 */
void W_DMA_Init(uint32_t PeripheralBaseAddr ,uint32_t MemoryBaseAddr ,int count )
{
	MyDMA_Count=count;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);//open DMA1 clock

    DMA_InitTypeDef DMA_InitStrture;
	
	DMA_InitStrture.DMA_PeripheralBaseAddr= PeripheralBaseAddr;//外设基地址
	DMA_InitStrture.DMA_PeripheralDataSize  = DMA_PeripheralDataSize_Byte ;//外设数据宽度 字节传输
	DMA_InitStrture.DMA_PeripheralInc=DMA_PeripheralInc_Enable;//起始自增 yes
	
	DMA_InitStrture.DMA_MemoryBaseAddr=MemoryBaseAddr ;//存储器起始地址
	DMA_InitStrture.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;//数据宽度
	DMA_InitStrture.DMA_MemoryInc=DMA_MemoryInc_Enable;//自增 yes
	DMA_InitStrture.DMA_DIR=DMA_DIR_PeripheralSRC;//DMA数据传输方向控制  外设寄存器为源方向，目的方向为存储器
	
	DMA_InitStrture.DMA_BufferSize=count;//传输计数次数  转运一次递减一次
	DMA_InitStrture.DMA_Mode=DMA_Mode_Normal;//是否使用自动重装
	DMA_InitStrture.DMA_M2M=DMA_M2M_Enable ;//软件触发
	DMA_InitStrture.DMA_Priority= DMA_Priority_VeryHigh ;
	DMA_Init( DMA1_Channel1 ,&DMA_InitStrture);//配置DMA1 通道1 
	
	DMA_Cmd(DMA1_Channel1,DISABLE);//关闭DMA
	

}


/**
 * @brief 开启转运 DMA1
 * 
 */
void StarRun_DMA(void)
{
	DMA_Cmd(DMA1_Channel1,DISABLE);//关闭DMA	
	DMA_SetCurrDataCounter(DMA1_Channel1,MyDMA_Count);
	DMA_Cmd(DMA1_Channel1,ENABLE);//开启DMA	
	
	while( DMA_GetFlagStatus(DMA1_FLAG_TC1)==RESET  );//检测是否转运完成
	DMA_ClearFlag(DMA1_FLAG_TC1);
	
}

















