#include"L298N.h"

/**
 * @brief L298N电机驱动模块
 * 轮子电机 GPIO初始化 PB0 PB1 PC14 PC15
 * 
 */
void wheels_GPIO_Init(void)
{

//-------------GPIO端口函数初始化--------------------
    GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOB, ENABLE );	//使能GPIOB时钟
    RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOC, ENABLE );	//使能GPIOB时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//GPIO_SetBits(GPIOB,GPIO_Pin_0|GPIO_Pin_1);//IN3 IN4 左轮

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	//GPIO_SetBits(GPIOC,GPIO_Pin_14|GPIO_Pin_15);//IN1 IN2  右轮
//-------------GPIO端口函数初始化--------------------

}

/**
 * @brief L298N模块 控制左右两侧两侧电机
 * 
 * @param R R(控制右侧电机): 0正转 1反转  2电机停止(刹车) PC14 PC15
 * @param L L(控制左侧电机)：0正转 1反转 2电机停止(刹车)  PB0 PB1
 */
void wheels_RL(unsigned int R,unsigned int L)
{

    switch(R){// R右轮 :表示正转/反转  0表示正转 1表示反转

        case 0:// INT1,2=01正转
            GPIO_ResetBits(GPIOC,GPIO_Pin_14);//GPIOE_Pin1 置低电平 0
	        GPIO_SetBits(GPIOC,GPIO_Pin_15);//GPIOE_Pin2 置高电平 1
            break;//退出switch语句

        case 1:// INT1,2=1 0反转
            GPIO_SetBits(GPIOC,GPIO_Pin_14);//GPIOE_Pin 置高电平 1
            GPIO_ResetBits(GPIOC,GPIO_Pin_15);//GPIOE_Pin0 置低电平 0      
            break;//退出switch语句
            
        case 2:// INT1,2=0 电机停止
            GPIO_ResetBits(GPIOC,GPIO_Pin_14);//GPIOE_Pin0 置低电平 0   
            GPIO_ResetBits(GPIOC,GPIO_Pin_15);//GPIOE_Pin0 置低电平 0     
            break;//退出switch语句
    }

    switch(L){// L左轮 :表示正转/反转  0表示正转 1表示反转

        case 0:// INT3,4=01正转
            GPIO_ResetBits(GPIOB,GPIO_Pin_0);//GPIOA_Pin3 置低电平 0
	        GPIO_SetBits(GPIOB,GPIO_Pin_1);//GPIOA_Pin4 置高电平 1
            break;//退出switch语句

        case 1:// INT3,4=10反转
                GPIO_SetBits(GPIOB,GPIO_Pin_0);//GPIOA_Pin_3置高电平 1
                GPIO_ResetBits(GPIOB,GPIO_Pin_1);//GPIOA_Pin_4置低电平 0       
                break;//退出switch语句
                
        case 2:// INT3,4=00 电机停止
            GPIO_ResetBits(GPIOB,GPIO_Pin_0);//GPIOE_Pin0 置低电平 0   
            GPIO_ResetBits(GPIOB,GPIO_Pin_1);//GPIOE_Pin0 置低电平 0    
            break;//退出switch语句
    }


}




