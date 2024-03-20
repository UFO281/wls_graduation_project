
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
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//推挽输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8| GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11;//PA8 PA9 PA10 PA11
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

}


/**
 * @brief 驱动左右两电机 正反转
 * IN1 PA8,IN2 PA9,IN3 PA10,IN4 PA11
 * @param R (控制右侧电机): 0正转 1反转 2电机停止
 * @param L L(控制左侧电机)：0正转 1反转 2电机停止
 */
void Drive_Motor_R_L(unsigned int R,unsigned int L)
{

    switch(R)
    {// R :表示正转/反转  0表示正转 1表示反转

        case 0:// INT1,2=01正转
            GPIO_WriteBit(GPIOA, GPIO_Pin_8, (BitAction)0);//PA8 = 0
		    GPIO_WriteBit(GPIOA, GPIO_Pin_9, (BitAction)1);//PA9 = 1 
            break;//退出switch语句

        case 1:// INT1,2=1 0反转
            GPIO_WriteBit(GPIOA, GPIO_Pin_8, (BitAction)1);//PA8 = 1
		    GPIO_WriteBit(GPIOA, GPIO_Pin_9, (BitAction)0);//PA9 = 0
            break;//退出switch语句

        case 2:// INT1,2=00 电机停止
            GPIO_WriteBit(GPIOA, GPIO_Pin_8, (BitAction)0);//PA8 = 0
		    GPIO_WriteBit(GPIOA, GPIO_Pin_9, (BitAction)0);//PA9 = 0
            break;//退出switch语句
    }

    switch(L)
    {// L :表示正转/反转  0表示正转 1表示反转

        case 0:// INT3,4=01正转

            GPIO_WriteBit(GPIOA, GPIO_Pin_10, (BitAction)0);//PA8 = 0
		    GPIO_WriteBit(GPIOA, GPIO_Pin_11, (BitAction)1);//PA9 = 1
            break;//退出switch语句

        case 1:// INT3,4=10反转
        
            GPIO_WriteBit(GPIOA, GPIO_Pin_10, (BitAction)1);//PA8 = 1
		    GPIO_WriteBit(GPIOA, GPIO_Pin_11, (BitAction)0);//PA9 = 0 
            break;//退出switch语句
                
        case 2:// INT3,4=00 电机停止

            GPIO_WriteBit(GPIOA, GPIO_Pin_10, (BitAction)0);//PA8 = 0
		    GPIO_WriteBit(GPIOA, GPIO_Pin_11, (BitAction)0);//PA9 = 0
            break;//退出switch语句
    }


}
