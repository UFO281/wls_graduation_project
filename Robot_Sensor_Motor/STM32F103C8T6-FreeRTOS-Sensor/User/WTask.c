
#include "WTask.h"






/**************************** 任务句柄 ********************************/
 /*
 任务句柄是一个指针，用于指向一个任务，当任务创建好之后，它就具有了一个任务句柄
 以后我们要想操作这个任务都需要通过这个任务句柄，如果是自身的任务操作自己，那么
 这个句柄可以为 NULL。
 */

static TaskHandle_t AppTaskCreate_Handle = NULL; /* 创建任务句柄 */

static TaskHandle_t Task0_Handle = NULL; /* Task0_Handle 任务句柄 */

static TaskHandle_t Task1_Handle = NULL; /* Task1_Handle 任务句柄 */

static TaskHandle_t Task2_Handle = NULL; /* Task2_Handle 任务句柄 */

static TaskHandle_t Task3_Handle = NULL; /* Task3_Handle 任务句柄 */

// static TaskHandle_t Task4_Handle = NULL; /* Task4_Handle 任务句柄 */

// static TaskHandle_t Task5_Handle = NULL; /* Task5_Handle 任务句柄 */

//QueueHandle_t MPU6050_Queue_Hnadle=NULL;/*创建MPU6050 传感器的 队列*/

// SemaphoreHandle_t Binary_Semaphore=NULL; /*二值信号量 句柄*/

/**************************** 任务句柄 ********************************/




/**
 * @brief 产生绝对值函数
 * 
 * @param x 有符号 (+ / - )整数值
 * @return uint16_t 无符号的 正整数
 */
uint16_t Get_Absolute_value(int x)
{
    if(x<0) return -x;
    else return x;
}


/**
 * @brief 硬件初始化
 * 已经使用IO：
 * PC 14 15
 * PA 0 9 10
 * PB 0 1 3 4 5 6 7 8 9 10 11 13 14 15
 */
void HardWare_Init(void)
{
 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4 4个bit 都是控制抢占优先级了 没有响应优先级的	 	 
    delay_init(); //延时函数初始化	永远放在 最开始处
    ConfigureTimeForRunTimeStats(); //配置 TIM2 使其为 FreeRTOS 的时间统计提供时基 
    WUsart1_Init();//USART1 初始化 115200波特率 8个数据位 1个停止位 无校验位
    CAN_Config(CAN_Mode_Normal);// CAN_Mode_Normal CAN_Mode_LoopBack CAN初始化环回模式,波特率500Kbps    

#if 1

    OLED_Init();//0.96 OLED 初始化  SCL:PB5  SDA:PB9 
 
    MPU_Init();	//初始化MPU6050 SCL:PB10  SDA:PB11 
    mpu_dmp_init();//MPU6050 初始化失败 程序就陷入这死循环了 

    #if (Use_AHT10==1)
        AHT10Init();//AHT10温湿度传感器初始化  SCL:PB3 SDA:PB4 
        AHT10Reset(); 
    #elif (Use_DHT11==0)
        DHT11_Init();//DHT11温湿度传感器 PA11        
    #endif


	Extern_Interrupt_Init();// 外部中断上升沿触发  CH12:PA12  CH15:PA15   

    wheels_GPIO_Init();//轮子电机GPIO 初始化   PB0 PB1 PC14 PC15

    Timer4_Timing_Init(1000,7200);//72MHZ/7200 = 10KHZ  100us*10*100=100ms

    Timer3_PWM_Init(1000,72);// 1KHZpwm PWM最大值为1000 R:CH1PA6 L:CH2PA7

#endif
    // NRF24L01_Init(); //初始化无线通信模块NRF24L01 ,PB6 PB7 PB8, PB13, PB14, PB15

	// NRF24L01_Check();//检查是否能正常工作  失败了就会陷入这个死循环



}




/**
 * @brief 创建开始任务
 * 
 */
int Start_Task(void)
{

    // Binary_Semaphore= xSemaphoreCreateBinary();
    // if(Binary_Semaphore==NULL) configASSERT( 0 );/*==NULL 二值信号量创建失败*/
  
   // MPU6050_Queue_Hnadle= xQueueCreate( 3, 4 ); /*队列长度3 队列单元大小4Byte*/
    //if(MPU6050_Queue_Hnadle ==NULL ) configASSERT( 0 ); /*如果产生的队列 为NULL  则创建失败 报错*/

    BaseType_t xReturn = pdPASS;/* 定义一个创建信息返回值，默认为 pdPASS */

    /* 创建 AppTaskCreate 任务 */   
    xReturn = xTaskCreate(  (TaskFunction_t )AppTaskCreate, /* 任务入口函数 */
                            (const char* )"AppTaskCreate",/* 任务名字 */
                            (uint16_t )128, /* 任务栈大小 128*4=512 Byte=0.5KB*/
                            (void* )NULL,/* 任务入口函数参数 */
                            (UBaseType_t )31, /* 任务的优先级 */
                            (TaskHandle_t* )&AppTaskCreate_Handle);/* 任务控制块指针 */

    configASSERT( xReturn );
    /* 启动任务调度 */    
    if (pdPASS == xReturn)
    {
        vTaskStartScheduler(); /* 启动任务，开启调度 */   
	    return 0;         
    }

    else return -1;

}





/**
 * @brief  创建 用户任务 类似于根任务
 * 
 */
void AppTaskCreate(void)
{

    BaseType_t xReturn = pdPASS;/* 定义一个创建信息返回值，默认为 pdPASS */

    taskENTER_CRITICAL(); //进入临界区
        
    /* 创建 Task_0 任务 */
    xReturn = xTaskCreate(  (TaskFunction_t )Task_0, /* 任务入口函数 */
                            (const char* )"Task_0",/* 任务名字 */
                            (uint16_t )128, /* 任务栈大小 128字=128*4 Byte=512Byte=0.5KB */
                            (void* )NULL, /* 任务入口函数参数 */
                            (UBaseType_t )30, /* 任务的优先级 */
                            (TaskHandle_t* )&Task0_Handle);/* 任务控制块指针 */
    configASSERT( xReturn );




    /* 创建 Task_1 任务 */
    xReturn = xTaskCreate(  (TaskFunction_t )Task_1, /* 任务入口函数 */
                            (const char* )"Task_1",/* 任务名字 */
                            (uint16_t )512, /* 任务栈大小 128字=128*4 Byte=512Byte=0.5KB */
                            (void* )NULL, /* 任务入口函数参数 */
                            (UBaseType_t )29, /* 任务的优先级 */
                            (TaskHandle_t* )&Task1_Handle);/* 任务控制块指针 */
    configASSERT( xReturn );


    /* 创建 Task_2 任务 */
    xReturn = xTaskCreate(  (TaskFunction_t )Task_2, /* 任务入口函数 */
                            (const char* )"Task_2",/* 任务名字 */
                            (uint16_t )128, /* 任务栈大小 128字=128*4 Byte=512Byte=0.5KB */
                            (void* )NULL, /* 任务入口函数参数 */
                            (UBaseType_t )28, /* 任务的优先级 */
                            (TaskHandle_t* )&Task2_Handle);/* 任务控制块指针 */
    configASSERT( xReturn );


    /* 创建 Task_3 任务 */
    xReturn = xTaskCreate(  (TaskFunction_t )Task_3, /* 任务入口函数 */
                            (const char* )"Task_3",/* 任务名字 */
                            (uint16_t )128, /* 任务栈大小 128字=128*4 Byte=512Byte=0.5KB */
                            (void* )NULL, /* 任务入口函数参数 */
                            (UBaseType_t )27, /* 任务的优先级 */
                            (TaskHandle_t* )&Task3_Handle);/* 任务控制块指针 */
    configASSERT( xReturn );
    

    // /* 创建 Task_4 任务 */
    // xReturn = xTaskCreate(  (TaskFunction_t )Task_4, /* 任务入口函数 */
    //                         (const char* )"Task_4",/* 任务名字 */
    //                         (uint16_t )128, /* 任务栈大小 128字=128*4 Byte=512Byte=0.5KB */
    //                         (void* )NULL, /* 任务入口函数参数 */
    //                         (UBaseType_t )26, /* 任务的优先级 */
    //                         (TaskHandle_t* )&Task4_Handle);/* 任务控制块指针 */
    // configASSERT( xReturn );

    // /* 创建 Task_5 任务 */
    // xReturn = xTaskCreate(  (TaskFunction_t )Task_5, /* 任务入口函数 */
    //                         (const char* )"Task_5",/* 任务名字 */
    //                         (uint16_t )128, /* 任务栈大小 128字=128*4 Byte=512Byte=0.5KB */
    //                         (void* )NULL, /* 任务入口函数参数 */
    //                         (UBaseType_t )25, /* 任务的优先级 */
    //                         (TaskHandle_t* )&Task5_Handle);/* 任务控制块指针 */
    // configASSERT( xReturn );


    vTaskDelete(AppTaskCreate_Handle); //删除 AppTaskCreate 任务

    taskEXIT_CRITICAL(); //退出临界区  
}



float pitch,roll,yaw; 		//欧拉角
short aacx,aacy,aacz;		//加速度传感器原始数据
short gyrox,gyroy,gyroz;	//陀螺仪原始数据

float   AHT10_temp= 0;//检测到的温度数据
u8      AHT10_humi= 0;//检测到的湿度数据



unsigned int pwm_R=0,pwm_L=0,pwm=0;



/**
 * @brief GET TEMP HUMI PITCH ROLL DATA
 * 
 * @param pvParameters 
 */
void Task_0( void * pvParameters )
{   

    OLED_ShowString(1, 1, "Pit:"); 
    OLED_ShowString(1, 9, "Rol:"); 
    
    OLED_ShowString(2, 1, "T:"); 
    OLED_ShowString(2, 7, "C"); 
    
    OLED_ShowString(2, 9, "H:"); 
    OLED_ShowString(2, 15, "%"); 

    OLED_ShowString(3, 1, "Rn:");
    OLED_ShowString(3, 9, "Ln:");


    while(1)
    {
        #if 1

        /*get tmep && humi data*/
        #if (Use_AHT10==1)
            AHT10ReadData(&AHT10_temp,&AHT10_humi);//读取温湿度数据  
        #elif (Use_DHT11==1)
            DHT11_Read_Data(&AHT10_temp,&AHT10_humi);//读取温湿度数据  
        #endif        

        #if 1 /*Get MPU6050 data*/
            mpu_dmp_get_data(&pitch,&roll,&yaw); //得到俯仰角pitch，和翻滚角roll，yaw航向角数据
            MPU_Get_Accelerometer(&aacx,&aacy,&aacz); //得到加速度传感器数据 X Y Z加速度数据
            MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz); //得到陀螺仪数据 X Y Z角速度数据 
        #endif /*Get MPU6050 data*/
        
        OLED_ShowSignedNum(1, 5,(int)pitch,2);//显示俯仰角  
        OLED_ShowSignedNum(1, 13,(int)roll,2);//显示横滚角  
        OLED_ShowSignedNum(2, 3,(int)AHT10_temp,3);//显示温度  
        OLED_ShowNum(2, 11,(int)AHT10_humi,3);//显示湿度  


//        printf("Pitch: %d Roll: %d Temp: %dC Humi: %%%d  \n\n",(int)pitch,(int) roll,
//                                (int)AHT10_temp,AHT10_humi);

//        printf("RR_Value(100ms/cirle):%d LL_Value(100ms/cirle):%d \n\n",RR_Value,LL_Value);

//        OLED_ShowNum(3, 4,RR_Value,3); //显示 右轮转速   
//        OLED_ShowNum(3, 12,LL_Value,3);//显示 左轮转速   

        #endif
        vTaskDelay(100); //延时 1s，也就是 1000 个时钟节拍
    }

}




/**
 * @brief Task_1 CAN_Tx_data
 * 
 * @param pvParameters 
 */
void Task_1( void * pvParameters )
{   

	char CAN_Txbuf[5][4]={0}; 
    int res=5;
    int i=0;

 	while(1)
	{
        
        sprintf(CAN_Txbuf[0], "%d", (int)pitch ); /* 将整型 pitch 转化成字符形式 赋值给数组*/
        sprintf(CAN_Txbuf[1], "%d", (int)roll );
        sprintf(CAN_Txbuf[2], "%d", (int)AHT10_temp); /* 将整型 pitch 转化成字符形式 赋值给数组*/
        sprintf(CAN_Txbuf[3], "%d", (int)AHT10_humi);
        sprintf(CAN_Txbuf[4], "endof\n");

        printf("Pitch:%s\n", CAN_Txbuf[0]);
        printf("Roll:%s\n", CAN_Txbuf[1]);
        printf("Temp:%sC\n", CAN_Txbuf[2]);
        printf("Humi:%%%s\n", CAN_Txbuf[3]);

        for ( i = 0; i < 5; i++)
        {
            res=Can_Send_Msg(CAN_Txbuf[i],sizeof(CAN_Txbuf[i]));//发送8个字节 
            if(res==0)
            {
                printf("\n\nCAN Send OK!! %s\n",CAN_Txbuf[i] );

            }
            else
            {
                printf("\n\nCAN Send Failed!! %s\n",CAN_Txbuf[i] );
            }

        }
        
        printf("can send complte ok!\n");

       
        vTaskDelay(500); //延时 1s，也就是 1000 个时钟节拍
	}


}



/**
 * @brief from Robot main control recive CAN DCmotor control data  
 * 
 * @param pvParameters 
 */
void Task_2( void * pvParameters )
{

    char CAN_Rxbuf[8]={0}; 
	u8 can_rx_lenth=0; 

    while(1)
    {

        can_rx_lenth=Can_Receive_Msg(CAN_Rxbuf);
        if(can_rx_lenth)
        {
            printf("can rx data OK! data lenth:%d \n",can_rx_lenth);

            // printf("can rx DATA: %s\n\n",CAN_Rxbuf );
           
            if ( !strcmp(CAN_Rxbuf,"0") )
            {/*---------STOP-------------*/
                   
                wheels_RL(2,2);//轮子停转
                
                //ret=xSemaphoreGive(Binary_Semaphore);
                //configASSERT(ret);/*释放信号量 +1 返回0则失败*/
                
                pwm_R=0;
                pwm_L=0;
                TIM_SetCompare1(TIM3, pwm_R);
                TIM_SetCompare2(TIM3, pwm_L);

            }
            else if ( !strcmp(CAN_Rxbuf,"1") )
            {/*---------Go Forward-------------*/

                wheels_RL(0,0);//轮子正转
                pwm_R+=50;
                pwm_L+=50;
                if((pwm_R>=1000) || (pwm_L>=1000) ){ pwm_R=1000;pwm_L=1000; }
                TIM_SetCompare1(TIM3, pwm_R);
                TIM_SetCompare2(TIM3, pwm_L);

            }
            else if (!strcmp(CAN_Rxbuf,"2") )
            {/*---------Back-------------*/

                wheels_RL(1,1);//轮子反转
                pwm_R+=50;
                pwm_L+=50;
                if((pwm_R>=1000) || (pwm_L>=1000) ){ pwm_R=1000;pwm_L=1000; }
                TIM_SetCompare1(TIM3, pwm_R);
                TIM_SetCompare2(TIM3, pwm_L);

            }            
            else if (!strcmp(CAN_Rxbuf,"3") )
            {/*---------Left-------------*/

                wheels_RL(0,0);//轮子正转
                pwm_R+=50;
                if(pwm_R>=1000) pwm_R=1000;

                if(pwm_L>20) pwm_L-=20; 
                else pwm_L=0;

                TIM_SetCompare1(TIM3, pwm_R);
                TIM_SetCompare2(TIM3, pwm_L);

            }
            else if (!strcmp(CAN_Rxbuf,"4") )
            {/*---------Right-------------*/
                wheels_RL(0,0);//轮子正转
                pwm_L+=50;
                if(pwm_L>=1000) pwm_L=1000;

                if(pwm_R>20) pwm_R-=20;
                else pwm_R=0;

                TIM_SetCompare1(TIM3, pwm_R);
                TIM_SetCompare2(TIM3, pwm_L);

            }
            else if (!strcmp(CAN_Rxbuf,"5") )
            {/*---------rest-------------*/
                pwm_R=0;
                pwm_L=0;
                TIM_SetCompare1(TIM3, pwm_R);
                TIM_SetCompare2(TIM3, pwm_L);

            }

            can_rx_lenth=0;
            memset(CAN_Rxbuf,0,sizeof(CAN_Rxbuf));

        }else{
            printf(" can rx data failed!\n");
        }
        
        vTaskDelay(500);//延时 100个tick 100个时钟节拍
    }
}





void Task_3( void * pvParameters )
{
   // BaseType_t ret=0;           

	while(1)
    {


        vTaskDelay(10); //延时 s，也就是 1000 个时钟节拍
    }

  
}














