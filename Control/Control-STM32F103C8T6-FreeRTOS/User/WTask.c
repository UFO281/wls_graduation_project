/**
 * @file WTask.c
 * @author wls
 * @brief 用户自定义函数 定义文件
 * @version 0.1
 * @date 2022-11-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "WTask.h"





//-----------------------全局变量定义----------------


//-----------------------全局变量定义----------------





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

static TaskHandle_t Task4_Handle = NULL; /* Task4_Handle 任务句柄 */

static TaskHandle_t Task5_Handle = NULL; /* Task5_Handle 任务句柄 */

//QueueHandle_t MPU6050_Queue_Hnadle=NULL;/*创建MPU6050 传感器的 队列*/

SemaphoreHandle_t Binary_Semaphore=NULL; /*二值信号量 句柄*/

/**************************** 任务句柄 ********************************/







/**
 * @brief Used IO 
 *   PA  0,1,2,3,4,5, 9(Uart1_TX),10(Uart1_RX)   
 *   PB
 *   PC
 * 
 */
void HardWare_Init(void)
{
 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4 4个bit 都是控制抢占优先级了 没有响应优先级的	 	 
    delay_init(); //延时函数初始化	永远放在 最开始处
    ConfigureTimeForRunTimeStats(); //配置 TIM2 使其为 FreeRTOS 的时间统计提供时基 
    WUsart1_Init();//USART1 初始化 115200波特率 8个数据位 1个停止位 无校验位
    WADC_Init();//ADC配置初始化

    LCD_Init();//LCD初始化
	LCD_Fill(0,0,LCD_W,LCD_H,BLACK);    	

    LCD_ShowString(5,25,"Temp:",WHITE,BLACK,16,0);

    LCD_ShowString(70,25,"C",WHITE,BLACK,16,0);

    LCD_ShowString(5,40,"Humi:",WHITE,BLACK,16,0);

    LCD_ShowString(70,40,"%",WHITE,BLACK,16,0);

    LCD_ShowString(20,5,"TX:",WHITE,BLACK,16,0);
	
 
    // OLED_Init();//0.96 OLED 初始化  SCL:PB5  SDA:PB9 

 
    // MPU_Init();	//初始化MPU6050 SCL:PB10  SDA:PB11 
    // mpu_dmp_init();//MPU6050 初始化失败 程序就陷入这死循环了 

    // #if (Use_AHT10==1)
    //     AHT10Init();//温湿度传感器初始化  SCL:PB3 SDA:PB4 
    //     AHT10Reset(); 
    // #elif (Use_DHT11==1)
    //     DHT11_Init();//温湿度传感器 PA11        
    // #endif


	// Extern_Interrupt_Init();// 外部中断上升沿触发  CH12:PA12  CH15:PA15   

    // wheels_GPIO_Init();//轮子电机GPIO 初始化   PB0 PB1 PC14 PC15

    // Timer4_Timing_Init(1000,7200);//72MHZ/7200 = 10KHZ  100us*10*100=100ms

    // Timer3_PWM_Init(1000,72);// 1KHZpwm PWM最大值为1000 R:CH1PA6 L:CH2PA7


    // NRF24L01_Init(); //初始化无线通信模块NRF24L01 ,PB6 PB7 PB8, PB13, PB14, PB15

	// NRF24L01_Check();//检查是否能正常工作  失败了就会陷入这个死循环


}




/**
 * @brief 创建开始任务
 * 
 */
int Start_Task(void)
{

    Binary_Semaphore= xSemaphoreCreateBinary();
    if(Binary_Semaphore==NULL) configASSERT( 0 );/*==NULL 二值信号量创建失败*/
  
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
                            (uint16_t )128, /* 任务栈大小 128字=128*4 Byte=512Byte=0.5KB */
                            (void* )NULL, /* 任务入口函数参数 */
                            (UBaseType_t )29, /* 任务的优先级 */
                            (TaskHandle_t* )&Task1_Handle);/* 任务控制块指针 */
    configASSERT( xReturn );


    // /* 创建 Task_2 任务 */
    // xReturn = xTaskCreate(  (TaskFunction_t )Task_2, /* 任务入口函数 */
    //                         (const char* )"Task_2",/* 任务名字 */
    //                         (uint16_t )128, /* 任务栈大小 128字=128*4 Byte=512Byte=0.5KB */
    //                         (void* )NULL, /* 任务入口函数参数 */
    //                         (UBaseType_t )28, /* 任务的优先级 */
    //                         (TaskHandle_t* )&Task2_Handle);/* 任务控制块指针 */
    // configASSERT( xReturn );


    // /* 创建 Task_3 任务 */
    // xReturn = xTaskCreate(  (TaskFunction_t )Task_3, /* 任务入口函数 */
    //                         (const char* )"Task_3",/* 任务名字 */
    //                         (uint16_t )128, /* 任务栈大小 128字=128*4 Byte=512Byte=0.5KB */
    //                         (void* )NULL, /* 任务入口函数参数 */
    //                         (UBaseType_t )27, /* 任务的优先级 */
    //                         (TaskHandle_t* )&Task3_Handle);/* 任务控制块指针 */
    // configASSERT( xReturn );
    


    vTaskDelete(AppTaskCreate_Handle); //删除 AppTaskCreate 任务

    taskEXIT_CRITICAL(); //退出临界区  
}



// float pitch,roll,yaw; 		//欧拉角
// short aacx,aacy,aacz;		//加速度传感器原始数据
// short gyrox,gyroy,gyroz;	//陀螺仪原始数据

// float   AHT10_temp= 0;//检测到的温度数据
// u8      AHT10_humi= 0;//检测到的湿度数据

// int      tx_data[4]= {0};//接收数据的数据包
// int      rx_data[4]= {5};//发送数据的数据包


// unsigned int pwm_R=0,pwm_L=0,pwm=0;

// short x=0; //模式状态变量




float ADC_CH[3]={0}; // 局部变量，用于保存转换计算后的电压值         

u16 ADC_X=0,ADC_Y=0;//通道1的转化后的值     SIOP刹车信号 为0刹车 1不作为

unsigned char car_data=5;
unsigned char STOP=1;
char Tx_buf[32]={0};

/**
 * @brief Get ADC value (get PS2 rocker )
 * 
 * @param pvParameters 
 */
void Task_0( void * pvParameters )
{
    // BaseType_t ret=0;    

    while(1)
    {

        ADC_CH[0] = ( (float) AD_GetValue(ADC_Channel_8) ) / 4096*(float)3.3;//PB0 Y轴   <155前进  >170后退
        ADC_CH[1] = ( (float) AD_GetValue(ADC_Channel_9) ) / 4096*(float)3.3;//PB1 X轴    <155右转 左轮加速 右轮减速   >170左转 右轮加速 左轮减速    
        ADC_Y=(ADC_CH[0]/1) * 100;// Y轴PB0  <155前进  >170后退
        ADC_X=(ADC_CH[1]/1) * 100;// X轴 PB1  <155右转 左轮加速 右轮减速   >170左转 右轮加速 左轮减速    

        LCD_ShowIntNum(23,65,ADC_Y,4,WHITE,BLUE,16);
        LCD_ShowIntNum(73,65,ADC_X,4,WHITE,RED,16);

        STOP=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10);   
        if(STOP==0)
        {                   
            car_data=0;//既不左右转 又不前后走 包含刹车   
        }
        if(ADC_X<155)
        {
            car_data=4;//右转
        }
        if(ADC_X>170)
        {
            car_data=3;//左转
        }
        if(ADC_Y>170)
        {
           
            car_data=2;//后退
        }
        if(ADC_Y<155)
        {
            // car_data=4;//右转
            car_data=1;//前进
        }
        if((ADC_X>=155)&&(ADC_X<=170) && (ADC_Y>=155)&&(ADC_Y<=170)&&(STOP!=0))
        {
            car_data=5;//既不左右转 又不前后走 也不刹车 静止状态  
        }

        // memset(Tx_buf,0,32);
        switch ( car_data )
        {
            case 0:// 0刹车
                LCD_ShowString(45,5,"0-STOP!",WHITE,RED,16,0);
                strcpy(Tx_buf,"0S\r\n");
                // Delay_ms(10);
                // if(STOP==0) ++x;
                break;

            case 1://前进
                LCD_ShowString(45,5,"1-GoGo!",WHITE,BLUE,16,0);   
                strcpy(Tx_buf,"1G\r\n");
                break;

            case 2://后退
                LCD_ShowString(45,5,"2-Back!",BLACK,YELLOW,16,0); 
                strcpy(Tx_buf,"2B\r\n");
                break;

            case 3://左转
                LCD_ShowString(45,5,"3-LL->!",BLACK,GREEN,16,0); 
                strcpy(Tx_buf,"3L\r\n");
                break;

            case 4://右转
                LCD_ShowString(45,5,"4-RR->!",BLACK,GREEN,16,0);  
                strcpy(Tx_buf,"4R\r\n");

                break;

            case 5://静止
                LCD_ShowString(45,5,"5-Rest!",BLACK,WHITE,16,0);   
                strcpy(Tx_buf,"5X\r\n");
                break;

        }

        //---------------------ADC检测处理程序------------------------ 
                
        // tx_data[0]=car_data; //car_data传给车的信号  1前进  2后退  3左转 4右转 0刹车() 其他并不作为
        


        vTaskDelay(10);//延时 100个tick 100个时钟节拍
    }


}



/**
 * @brief Uart1 RX && TX data
 * 
 * @param pvParameters 
 */
void Task_1( void * pvParameters )
{
   // BaseType_t ret=0;
    char Uart1_Rx[64]={0};   
      

	while(1)
    {

        if (Get_Rx_Packge_State())
        {
            strcpy(Uart1_Rx,Get_RxPackge());
            Send_String(Uart1_Rx);
            LCD_ShowString(15,90,Uart1_Rx,WHITE,BLACK,16,0);
            memset(Uart1_Rx,0,64);
        }

        Send_String(Tx_buf);
        memset(Tx_buf,0,32);


        vTaskDelay(10); //延时 s，也就是 1000 个时钟节拍
    }

  
}




/**
 * @brief Task_2 show_Fuction 
 * Show_Temp_Humi_MPU6050 data
 * 
 * @param pvParameters 
 */
void Task_2( void * pvParameters )
{   




    while(1)
    {

        vTaskDelay(100); //延时 1s，也就是 1000 个时钟节拍
    }

}




/**
 * @brief Task_3 Conrtl_Motor
 * 
 * @param pvParameters 
 */
void Task_3( void * pvParameters )
{   

    while(1)
    {
      

        vTaskDelay(5); //延时 1s，也就是 1000 个时钟节拍
    }

}














