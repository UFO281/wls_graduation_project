/**
 * @file WTask.c
 * @author wls
 * @brief �û��Զ��庯�� �����ļ�
 * @version 0.1
 * @date 2022-11-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "WTask.h"







/**************************** ������ ********************************/
 /*
 ��������һ��ָ�룬����ָ��һ�����񣬵����񴴽���֮�����;�����һ��������
 �Ժ�����Ҫ��������������Ҫͨ�������������������������������Լ�����ô
 ����������Ϊ NULL��
 */

static TaskHandle_t AppTaskCreate_Handle = NULL; /* ���������� */

static TaskHandle_t Task0_Handle = NULL; /* Task0_Handle ������ */

static TaskHandle_t Task1_Handle = NULL; /* Task1_Handle ������ */

//static TaskHandle_t Task2_Handle = NULL; /* Task2_Handle ������ */

//static TaskHandle_t Task3_Handle = NULL; /* Task3_Handle ������ */

//static TaskHandle_t Task4_Handle = NULL; /* Task4_Handle ������ */

//static TaskHandle_t Task5_Handle = NULL; /* Task5_Handle ������ */

//QueueHandle_t MPU6050_Queue_Hnadle=NULL;/*����MPU6050 �������� ����*/

// SemaphoreHandle_t Binary_Semaphore=NULL; /*��ֵ�ź��� ���*/

/**************************** ������ ********************************/







/**
 * @brief Used IO 
 *   PA  0,1,2,3,4,5, 9(Uart1_TX),10(Uart1_RX)   
 *   PB
 *   PC
 * 
 */
void HardWare_Init(void)
{
 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4 4��bit ���ǿ�����ռ���ȼ��� û����Ӧ���ȼ���	 	 
    delay_init(); //��ʱ������ʼ��	��Զ���� �ʼ��
    ConfigureTimeForRunTimeStats(); //���� TIM2 ʹ��Ϊ FreeRTOS ��ʱ��ͳ���ṩʱ�� 
    WUsart1_Init();//USART1 ��ʼ�� 115200������ 8������λ 1��ֹͣλ ��У��λ
    WADC_Init();//ADC���ó�ʼ��

    LCD_Init();//LCD��ʼ��
	LCD_Fill(0,0,LCD_W,LCD_H,BLACK);    	

    LCD_ShowString(20,5,"TX:",WHITE,BLACK,16,0);
    LCD_ShowString(5,25,"Temp:",WHITE,BLACK,16,0);
    // LCD_ShowString(70,25,"C",WHITE,BLACK,16,0);
    LCD_ShowString(5,40,"Humi:",WHITE,BLACK,16,0);
    // LCD_ShowString(70,40,"%",WHITE,BLACK,16,0);
    
    LCD_ShowString(5,65,"Pit:",WHITE,BLACK,16,0);
    LCD_ShowString(5,85,"Rol:",WHITE,BLACK,16,0);


	
 
    // OLED_Init();//0.96 OLED ��ʼ��  SCL:PB5  SDA:PB9 

 
    // MPU_Init();	//��ʼ��MPU6050 SCL:PB10  SDA:PB11 
    // mpu_dmp_init();//MPU6050 ��ʼ��ʧ�� �������������ѭ���� 

    // #if (Use_AHT10==1)
    //     AHT10Init();//��ʪ�ȴ�������ʼ��  SCL:PB3 SDA:PB4 
    //     AHT10Reset(); 
    // #elif (Use_DHT11==1)
    //     DHT11_Init();//��ʪ�ȴ����� PA11        
    // #endif


	// Extern_Interrupt_Init();// �ⲿ�ж������ش���  CH12:PA12  CH15:PA15   

    // wheels_GPIO_Init();//���ӵ��GPIO ��ʼ��   PB0 PB1 PC14 PC15

    // Timer4_Timing_Init(1000,7200);//72MHZ/7200 = 10KHZ  100us*10*100=100ms

    // Timer3_PWM_Init(1000,72);// 1KHZpwm PWM���ֵΪ1000 R:CH1PA6 L:CH2PA7


    // NRF24L01_Init(); //��ʼ������ͨ��ģ��NRF24L01 ,PB6 PB7 PB8, PB13, PB14, PB15

	// NRF24L01_Check();//����Ƿ�����������  ʧ���˾ͻ����������ѭ��


}




/**
 * @brief ������ʼ����
 * 
 */
int Start_Task(void)
{

    // Binary_Semaphore= xSemaphoreCreateBinary();
    // if(Binary_Semaphore==NULL) configASSERT( 0 );/*==NULL ��ֵ�ź�������ʧ��*/
  
   // MPU6050_Queue_Hnadle= xQueueCreate( 3, 4 ); /*���г���3 ���е�Ԫ��С4Byte*/
    //if(MPU6050_Queue_Hnadle ==NULL ) configASSERT( 0 ); /*��������Ķ��� ΪNULL  �򴴽�ʧ�� ����*/

    BaseType_t xReturn = pdPASS;/* ����һ��������Ϣ����ֵ��Ĭ��Ϊ pdPASS */

    /* ���� AppTaskCreate ���� */   
    xReturn = xTaskCreate(  (TaskFunction_t )AppTaskCreate, /* ������ں��� */
                            (const char* )"AppTaskCreate",/* �������� */
                            (uint16_t )128, /* ����ջ��С 128*4=512 Byte=0.5KB*/
                            (void* )NULL,/* ������ں������� */
                            (UBaseType_t )31, /* ��������ȼ� */
                            (TaskHandle_t* )&AppTaskCreate_Handle);/* ������ƿ�ָ�� */

    configASSERT( xReturn );
    /* ����������� */    
    if (pdPASS == xReturn)
    {
        vTaskStartScheduler(); /* �������񣬿������� */   
	    return 0;         
    }

    else return -1;

}





/**
 * @brief  ���� �û����� �����ڸ�����
 * 
 */
void AppTaskCreate(void)
{

    BaseType_t xReturn = pdPASS;/* ����һ��������Ϣ����ֵ��Ĭ��Ϊ pdPASS */

    taskENTER_CRITICAL(); //�����ٽ���
        
    /* ���� Task_0 ���� */
    xReturn = xTaskCreate(  (TaskFunction_t )Task_0, /* ������ں��� */
                            (const char* )"Task_0",/* �������� */
                            (uint16_t )256, /* ����ջ��С 128��=128*4 Byte=512Byte=0.5KB */
                            (void* )NULL, /* ������ں������� */
                            (UBaseType_t )30, /* ��������ȼ� */
                            (TaskHandle_t* )&Task0_Handle);/* ������ƿ�ָ�� */
    configASSERT( xReturn );




    /* ���� Task_1 ���� */
    xReturn = xTaskCreate(  (TaskFunction_t )Task_1, /* ������ں��� */
                            (const char* )"Task_1",/* �������� */
                            (uint16_t )256, /* ����ջ��С 128��=128*4 Byte=512Byte=0.5KB */
                            (void* )NULL, /* ������ں������� */
                            (UBaseType_t )29, /* ��������ȼ� */
                            (TaskHandle_t* )&Task1_Handle);/* ������ƿ�ָ�� */
    configASSERT( xReturn );


    // /* ���� Task_2 ���� */
    // xReturn = xTaskCreate(  (TaskFunction_t )Task_2, /* ������ں��� */
    //                         (const char* )"Task_2",/* �������� */
    //                         (uint16_t )128, /* ����ջ��С 128��=128*4 Byte=512Byte=0.5KB */
    //                         (void* )NULL, /* ������ں������� */
    //                         (UBaseType_t )28, /* ��������ȼ� */
    //                         (TaskHandle_t* )&Task2_Handle);/* ������ƿ�ָ�� */
    // configASSERT( xReturn );


    // /* ���� Task_3 ���� */
    // xReturn = xTaskCreate(  (TaskFunction_t )Task_3, /* ������ں��� */
    //                         (const char* )"Task_3",/* �������� */
    //                         (uint16_t )128, /* ����ջ��С 128��=128*4 Byte=512Byte=0.5KB */
    //                         (void* )NULL, /* ������ں������� */
    //                         (UBaseType_t )27, /* ��������ȼ� */
    //                         (TaskHandle_t* )&Task3_Handle);/* ������ƿ�ָ�� */
    // configASSERT( xReturn );
    


    vTaskDelete(AppTaskCreate_Handle); //ɾ�� AppTaskCreate ����

    taskEXIT_CRITICAL(); //�˳��ٽ���  
}




float ADC_CH[3]={0}; // �ֲ����������ڱ���ת�������ĵ�ѹֵ         

u16 ADC_X=0,ADC_Y=0;//ͨ��1��ת�����ֵ     SIOPɲ���ź� Ϊ0ɲ�� 1����Ϊ

unsigned char car_data=5;
unsigned char STOP=1;
char Tx_buf[8]={0};

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

        ADC_CH[0] = ( (float) AD_GetValue(ADC_Channel_8) ) / 4096*(float)3.3;//PB0 Y��   <155ǰ��  >170����
        ADC_CH[1] = ( (float) AD_GetValue(ADC_Channel_9) ) / 4096*(float)3.3;//PB1 X��    <155��ת ���ּ��� ���ּ���   >170��ת ���ּ��� ���ּ���    
        ADC_Y=(ADC_CH[0]/1) * 100;// Y��PB0  <155ǰ��  >170����
        ADC_X=(ADC_CH[1]/1) * 100;// X�� PB1  <155��ת ���ּ��� ���ּ���   >170��ת ���ּ��� ���ּ���    

        // LCD_ShowIntNum(23,65,ADC_Y,4,WHITE,BLUE,16);
        // LCD_ShowIntNum(73,65,ADC_X,4,WHITE,RED,16);

        STOP=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10);   
        if(STOP==0)
        {                   
            car_data=0;//�Ȳ�����ת �ֲ�ǰ���� ����ɲ��   
        }
        if(ADC_X<155)
        {
            car_data=4;//��ת
        }
        if(ADC_X>170)
        {
            car_data=3;//��ת
        }
        if(ADC_Y>170)
        {
           
            car_data=2;//����
        }
        if(ADC_Y<155)
        {
            // car_data=4;//��ת
            car_data=1;//ǰ��
        }
        if((ADC_X>=155)&&(ADC_X<=170) && (ADC_Y>=155)&&(ADC_Y<=170)&&(STOP!=0))
        {
            car_data=5;//�Ȳ�����ת �ֲ�ǰ���� Ҳ��ɲ�� ��ֹ״̬  
        }

        // memset(Tx_buf,0,32);
        switch ( car_data )
        {
            case 0:// 0ɲ��
                LCD_ShowString(45,5,"0-STOP!",WHITE,RED,16,0);
                strcpy(Tx_buf,"0");
                // Delay_ms(10);
                // if(STOP==0) ++x;
                break;

            case 1://ǰ��
                LCD_ShowString(45,5,"1-GoGo!",WHITE,BLUE,16,0);   
                strcpy(Tx_buf,"1");
                break;

            case 2://����
                LCD_ShowString(45,5,"2-Back!",BLACK,YELLOW,16,0); 
                strcpy(Tx_buf,"2");
                break;

            case 3://��ת
                LCD_ShowString(45,5,"3-LL->!",BLACK,GREEN,16,0); 
                strcpy(Tx_buf,"3");
                break;

            case 4://��ת
                LCD_ShowString(45,5,"4-RR->!",BLACK,GREEN,16,0);  
                strcpy(Tx_buf,"4");

                break;

            case 5://��ֹ
                LCD_ShowString(45,5,"5-Rest!",BLACK,WHITE,16,0);   
                strcpy(Tx_buf,"5");
                break;

        }

        //---------------------ADC��⴦�����------------------------ 
                
        // tx_data[0]=car_data; //car_data���������ź�  1ǰ��  2����  3��ת 4��ת 0ɲ��() ����������Ϊ
        


        vTaskDelay(10);//��ʱ 100��tick 100��ʱ�ӽ���
    }


}



/**
 * @brief Uart1 RX && TX data
 * 
 * @param pvParameters 
 */
void Task_1( void * pvParameters )
{

	while(1)
    {

        Send_String(Tx_buf);
        memset(Tx_buf,0,8);
        
	   
        if (Get_Rx_Packge_State())
        {
            // printf("Pitch:%s\n", Uart1_Rx[0]);
            // printf("Roll:%s\n", Uart1_Rx[1]);
            // printf("Temp:%sC\n", Uart1_Rx[2]);
            // printf("Humi:%%%s\n", Uart1_Rx[3]);
            Get_RxPackge();
			LCD_ShowString(50,25,Uart1_Rx[2],WHITE,BLACK,16,0);
			LCD_ShowString(50,40,Uart1_Rx[3],WHITE,BLACK,16,0);
			LCD_ShowString(40,65,Uart1_Rx[0],WHITE,BLACK,16,0);
			LCD_ShowString(40,85,Uart1_Rx[1],WHITE,BLACK,16,0);

            memset(Uart1_Rx,0,Rx_Count);
        }





        vTaskDelay(10); //��ʱ s��Ҳ���� 1000 ��ʱ�ӽ���
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

        vTaskDelay(100); //��ʱ 1s��Ҳ���� 1000 ��ʱ�ӽ���
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
      

        vTaskDelay(5); //��ʱ 1s��Ҳ���� 1000 ��ʱ�ӽ���
    }

}














