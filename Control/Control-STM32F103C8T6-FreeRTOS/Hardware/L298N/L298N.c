#include"L298N.h"

/**
 * @brief L298N�������ģ��
 * ���ӵ�� GPIO��ʼ�� PB0 PB1 PC14 PC15
 * 
 */
void wheels_GPIO_Init(void)
{

//-------------GPIO�˿ں�����ʼ��--------------------
    GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );	//ʹ��GPIOBʱ��
    RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOC, ENABLE );	//ʹ��GPIOBʱ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//GPIO_SetBits(GPIOB,GPIO_Pin_0|GPIO_Pin_1);//IN3 IN4 ����

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	//GPIO_SetBits(GPIOC,GPIO_Pin_14|GPIO_Pin_15);//IN1 IN2  ����
//-------------GPIO�˿ں�����ʼ��--------------------

}

/**
 * @brief L298Nģ�� ������������������
 * 
 * @param R R(�����Ҳ���): 0��ת 1��ת  2���ֹͣ(ɲ��) PC14 PC15
 * @param L L(���������)��0��ת 1��ת 2���ֹͣ(ɲ��)  PB0 PB1
 */
void wheels_RL(unsigned int R,unsigned int L)
{

    switch(R){// R���� :��ʾ��ת/��ת  0��ʾ��ת 1��ʾ��ת

        case 0:// INT1,2=01��ת
            GPIO_ResetBits(GPIOC,GPIO_Pin_14);//GPIOE_Pin1 �õ͵�ƽ 0
	        GPIO_SetBits(GPIOC,GPIO_Pin_15);//GPIOE_Pin2 �øߵ�ƽ 1
            break;//�˳�switch���

        case 1:// INT1,2=1 0��ת
            GPIO_SetBits(GPIOC,GPIO_Pin_14);//GPIOE_Pin �øߵ�ƽ 1
            GPIO_ResetBits(GPIOC,GPIO_Pin_15);//GPIOE_Pin0 �õ͵�ƽ 0      
            break;//�˳�switch���
            
        case 2:// INT1,2=0 ���ֹͣ
            GPIO_ResetBits(GPIOC,GPIO_Pin_14);//GPIOE_Pin0 �õ͵�ƽ 0   
            GPIO_ResetBits(GPIOC,GPIO_Pin_15);//GPIOE_Pin0 �õ͵�ƽ 0     
            break;//�˳�switch���
    }

    switch(L){// L���� :��ʾ��ת/��ת  0��ʾ��ת 1��ʾ��ת

        case 0:// INT3,4=01��ת
            GPIO_ResetBits(GPIOB,GPIO_Pin_0);//GPIOA_Pin3 �õ͵�ƽ 0
	        GPIO_SetBits(GPIOB,GPIO_Pin_1);//GPIOA_Pin4 �øߵ�ƽ 1
            break;//�˳�switch���

        case 1:// INT3,4=10��ת
                GPIO_SetBits(GPIOB,GPIO_Pin_0);//GPIOA_Pin_3�øߵ�ƽ 1
                GPIO_ResetBits(GPIOB,GPIO_Pin_1);//GPIOA_Pin_4�õ͵�ƽ 0       
                break;//�˳�switch���
                
        case 2:// INT3,4=00 ���ֹͣ
            GPIO_ResetBits(GPIOB,GPIO_Pin_0);//GPIOE_Pin0 �õ͵�ƽ 0   
            GPIO_ResetBits(GPIOB,GPIO_Pin_1);//GPIOE_Pin0 �õ͵�ƽ 0    
            break;//�˳�switch���
    }


}




