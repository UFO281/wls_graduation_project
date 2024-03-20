
#include"TB6612FNG.h"


/**
 * @brief Motor GPio_Init, 
 * IN1 PA8,IN2 PA9,IN3 PA10,IN4 PA11
 * 
 */
void Drive_Motor_GPIO_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//�������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8| GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11;//PA8 PA9 PA10 PA11
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

}


/**
 * @brief ������������� ����ת
 * IN1 PA8,IN2 PA9,IN3 PA10,IN4 PA11
 * @param R (�����Ҳ���): 0��ת 1��ת 2���ֹͣ
 * @param L L(���������)��0��ת 1��ת 2���ֹͣ
 */
void Drive_Motor_R_L(unsigned int R,unsigned int L)
{

    switch(R)
    {// R :��ʾ��ת/��ת  0��ʾ��ת 1��ʾ��ת

        case 0:// INT1,2=01��ת
            GPIO_WriteBit(GPIOA, GPIO_Pin_8, (BitAction)0);//PA8 = 0
		    GPIO_WriteBit(GPIOA, GPIO_Pin_9, (BitAction)1);//PA9 = 1 
            break;//�˳�switch���

        case 1:// INT1,2=1 0��ת
            GPIO_WriteBit(GPIOA, GPIO_Pin_8, (BitAction)1);//PA8 = 1
		    GPIO_WriteBit(GPIOA, GPIO_Pin_9, (BitAction)0);//PA9 = 0
            break;//�˳�switch���

        case 2:// INT1,2=00 ���ֹͣ
            GPIO_WriteBit(GPIOA, GPIO_Pin_8, (BitAction)0);//PA8 = 0
		    GPIO_WriteBit(GPIOA, GPIO_Pin_9, (BitAction)0);//PA9 = 0
            break;//�˳�switch���
    }

    switch(L)
    {// L :��ʾ��ת/��ת  0��ʾ��ת 1��ʾ��ת

        case 0:// INT3,4=01��ת

            GPIO_WriteBit(GPIOA, GPIO_Pin_10, (BitAction)0);//PA8 = 0
		    GPIO_WriteBit(GPIOA, GPIO_Pin_11, (BitAction)1);//PA9 = 1
            break;//�˳�switch���

        case 1:// INT3,4=10��ת
        
            GPIO_WriteBit(GPIOA, GPIO_Pin_10, (BitAction)1);//PA8 = 1
		    GPIO_WriteBit(GPIOA, GPIO_Pin_11, (BitAction)0);//PA9 = 0 
            break;//�˳�switch���
                
        case 2:// INT3,4=00 ���ֹͣ

            GPIO_WriteBit(GPIOA, GPIO_Pin_10, (BitAction)0);//PA8 = 0
		    GPIO_WriteBit(GPIOA, GPIO_Pin_11, (BitAction)0);//PA9 = 0
            break;//�˳�switch���
    }


}
