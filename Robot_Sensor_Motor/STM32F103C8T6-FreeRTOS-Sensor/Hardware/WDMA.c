#include "WDMA.h"

int MyDMA_Count=0;


/**
 * @brief DMA Init Config one channel  ,
 * transform from PeripheralBaseAddr to MemoryBaseAddr
 * 
 * @param PeripheralBaseAddr �����ַ
 * @param MemoryBaseAddr �洢����ַ
 * @param count ת�˴���
 */
void W_DMA_Init(uint32_t PeripheralBaseAddr ,uint32_t MemoryBaseAddr ,int count )
{
	MyDMA_Count=count;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);//open DMA1 clock

    DMA_InitTypeDef DMA_InitStrture;
	
	DMA_InitStrture.DMA_PeripheralBaseAddr= PeripheralBaseAddr;//�������ַ
	DMA_InitStrture.DMA_PeripheralDataSize  = DMA_PeripheralDataSize_Byte ;//�������ݿ�� �ֽڴ���
	DMA_InitStrture.DMA_PeripheralInc=DMA_PeripheralInc_Enable;//��ʼ���� yes
	
	DMA_InitStrture.DMA_MemoryBaseAddr=MemoryBaseAddr ;//�洢����ʼ��ַ
	DMA_InitStrture.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;//���ݿ��
	DMA_InitStrture.DMA_MemoryInc=DMA_MemoryInc_Enable;//���� yes
	DMA_InitStrture.DMA_DIR=DMA_DIR_PeripheralSRC;//DMA���ݴ��䷽�����  ����Ĵ���ΪԴ����Ŀ�ķ���Ϊ�洢��
	
	DMA_InitStrture.DMA_BufferSize=count;//�����������  ת��һ�εݼ�һ��
	DMA_InitStrture.DMA_Mode=DMA_Mode_Normal;//�Ƿ�ʹ���Զ���װ
	DMA_InitStrture.DMA_M2M=DMA_M2M_Enable ;//�������
	DMA_InitStrture.DMA_Priority= DMA_Priority_VeryHigh ;
	DMA_Init( DMA1_Channel1 ,&DMA_InitStrture);//����DMA1 ͨ��1 
	
	DMA_Cmd(DMA1_Channel1,DISABLE);//�ر�DMA
	

}


/**
 * @brief ����ת�� DMA1
 * 
 */
void StarRun_DMA(void)
{
	DMA_Cmd(DMA1_Channel1,DISABLE);//�ر�DMA	
	DMA_SetCurrDataCounter(DMA1_Channel1,MyDMA_Count);
	DMA_Cmd(DMA1_Channel1,ENABLE);//����DMA	
	
	while( DMA_GetFlagStatus(DMA1_FLAG_TC1)==RESET  );//����Ƿ�ת�����
	DMA_ClearFlag(DMA1_FLAG_TC1);
	
}

















