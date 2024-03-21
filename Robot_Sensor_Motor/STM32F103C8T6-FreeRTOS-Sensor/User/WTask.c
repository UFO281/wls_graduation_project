
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

static TaskHandle_t Task2_Handle = NULL; /* Task2_Handle ������ */

static TaskHandle_t Task3_Handle = NULL; /* Task3_Handle ������ */

// static TaskHandle_t Task4_Handle = NULL; /* Task4_Handle ������ */

// static TaskHandle_t Task5_Handle = NULL; /* Task5_Handle ������ */

//QueueHandle_t MPU6050_Queue_Hnadle=NULL;/*����MPU6050 �������� ����*/

// SemaphoreHandle_t Binary_Semaphore=NULL; /*��ֵ�ź��� ���*/

/**************************** ������ ********************************/




/**
 * @brief ��������ֵ����
 * 
 * @param x �з��� (+ / - )����ֵ
 * @return uint16_t �޷��ŵ� ������
 */
uint16_t Get_Absolute_value(int x)
{
    if(x<0) return -x;
    else return x;
}


/**
 * @brief Ӳ����ʼ��
 * �Ѿ�ʹ��IO��
 * PC 14 15
 * PA 0 9 10
 * PB 0 1 3 4 5 6 7 8 9 10 11 13 14 15
 */
void HardWare_Init(void)
{
 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4 4��bit ���ǿ�����ռ���ȼ��� û����Ӧ���ȼ���	 	 
    delay_init(); //��ʱ������ʼ��	��Զ���� �ʼ��
    ConfigureTimeForRunTimeStats(); //���� TIM2 ʹ��Ϊ FreeRTOS ��ʱ��ͳ���ṩʱ�� 
    WUsart1_Init();//USART1 ��ʼ�� 115200������ 8������λ 1��ֹͣλ ��У��λ
    CAN_Config(CAN_Mode_Normal);// CAN_Mode_Normal CAN_Mode_LoopBack CAN��ʼ������ģʽ,������500Kbps    

#if 1

    OLED_Init();//0.96 OLED ��ʼ��  SCL:PB5  SDA:PB9 
 
    MPU_Init();	//��ʼ��MPU6050 SCL:PB10  SDA:PB11 
    mpu_dmp_init();//MPU6050 ��ʼ��ʧ�� �������������ѭ���� 

    #if (Use_AHT10==1)
        AHT10Init();//AHT10��ʪ�ȴ�������ʼ��  SCL:PB3 SDA:PB4 
        AHT10Reset(); 
    #elif (Use_DHT11==0)
        DHT11_Init();//DHT11��ʪ�ȴ����� PA11        
    #endif


	Extern_Interrupt_Init();// �ⲿ�ж������ش���  CH12:PA12  CH15:PA15   

    wheels_GPIO_Init();//���ӵ��GPIO ��ʼ��   PB0 PB1 PC14 PC15

    Timer4_Timing_Init(1000,7200);//72MHZ/7200 = 10KHZ  100us*10*100=100ms

    Timer3_PWM_Init(1000,72);// 1KHZpwm PWM���ֵΪ1000 R:CH1PA6 L:CH2PA7

#endif
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
                            (uint16_t )128, /* ����ջ��С 128��=128*4 Byte=512Byte=0.5KB */
                            (void* )NULL, /* ������ں������� */
                            (UBaseType_t )30, /* ��������ȼ� */
                            (TaskHandle_t* )&Task0_Handle);/* ������ƿ�ָ�� */
    configASSERT( xReturn );




    /* ���� Task_1 ���� */
    xReturn = xTaskCreate(  (TaskFunction_t )Task_1, /* ������ں��� */
                            (const char* )"Task_1",/* �������� */
                            (uint16_t )512, /* ����ջ��С 128��=128*4 Byte=512Byte=0.5KB */
                            (void* )NULL, /* ������ں������� */
                            (UBaseType_t )29, /* ��������ȼ� */
                            (TaskHandle_t* )&Task1_Handle);/* ������ƿ�ָ�� */
    configASSERT( xReturn );


    /* ���� Task_2 ���� */
    xReturn = xTaskCreate(  (TaskFunction_t )Task_2, /* ������ں��� */
                            (const char* )"Task_2",/* �������� */
                            (uint16_t )128, /* ����ջ��С 128��=128*4 Byte=512Byte=0.5KB */
                            (void* )NULL, /* ������ں������� */
                            (UBaseType_t )28, /* ��������ȼ� */
                            (TaskHandle_t* )&Task2_Handle);/* ������ƿ�ָ�� */
    configASSERT( xReturn );


    /* ���� Task_3 ���� */
    xReturn = xTaskCreate(  (TaskFunction_t )Task_3, /* ������ں��� */
                            (const char* )"Task_3",/* �������� */
                            (uint16_t )128, /* ����ջ��С 128��=128*4 Byte=512Byte=0.5KB */
                            (void* )NULL, /* ������ں������� */
                            (UBaseType_t )27, /* ��������ȼ� */
                            (TaskHandle_t* )&Task3_Handle);/* ������ƿ�ָ�� */
    configASSERT( xReturn );
    

    // /* ���� Task_4 ���� */
    // xReturn = xTaskCreate(  (TaskFunction_t )Task_4, /* ������ں��� */
    //                         (const char* )"Task_4",/* �������� */
    //                         (uint16_t )128, /* ����ջ��С 128��=128*4 Byte=512Byte=0.5KB */
    //                         (void* )NULL, /* ������ں������� */
    //                         (UBaseType_t )26, /* ��������ȼ� */
    //                         (TaskHandle_t* )&Task4_Handle);/* ������ƿ�ָ�� */
    // configASSERT( xReturn );

    // /* ���� Task_5 ���� */
    // xReturn = xTaskCreate(  (TaskFunction_t )Task_5, /* ������ں��� */
    //                         (const char* )"Task_5",/* �������� */
    //                         (uint16_t )128, /* ����ջ��С 128��=128*4 Byte=512Byte=0.5KB */
    //                         (void* )NULL, /* ������ں������� */
    //                         (UBaseType_t )25, /* ��������ȼ� */
    //                         (TaskHandle_t* )&Task5_Handle);/* ������ƿ�ָ�� */
    // configASSERT( xReturn );


    vTaskDelete(AppTaskCreate_Handle); //ɾ�� AppTaskCreate ����

    taskEXIT_CRITICAL(); //�˳��ٽ���  
}



float pitch,roll,yaw; 		//ŷ����
short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
short gyrox,gyroy,gyroz;	//������ԭʼ����

float   AHT10_temp= 0;//��⵽���¶�����
u8      AHT10_humi= 0;//��⵽��ʪ������



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
            AHT10ReadData(&AHT10_temp,&AHT10_humi);//��ȡ��ʪ������  
        #elif (Use_DHT11==1)
            DHT11_Read_Data(&AHT10_temp,&AHT10_humi);//��ȡ��ʪ������  
        #endif        

        #if 1 /*Get MPU6050 data*/
            mpu_dmp_get_data(&pitch,&roll,&yaw); //�õ�������pitch���ͷ�����roll��yaw���������
            MPU_Get_Accelerometer(&aacx,&aacy,&aacz); //�õ����ٶȴ��������� X Y Z���ٶ�����
            MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz); //�õ����������� X Y Z���ٶ����� 
        #endif /*Get MPU6050 data*/
        
        OLED_ShowSignedNum(1, 5,(int)pitch,2);//��ʾ������  
        OLED_ShowSignedNum(1, 13,(int)roll,2);//��ʾ�����  
        OLED_ShowSignedNum(2, 3,(int)AHT10_temp,3);//��ʾ�¶�  
        OLED_ShowNum(2, 11,(int)AHT10_humi,3);//��ʾʪ��  


//        printf("Pitch: %d Roll: %d Temp: %dC Humi: %%%d  \n\n",(int)pitch,(int) roll,
//                                (int)AHT10_temp,AHT10_humi);

//        printf("RR_Value(100ms/cirle):%d LL_Value(100ms/cirle):%d \n\n",RR_Value,LL_Value);

//        OLED_ShowNum(3, 4,RR_Value,3); //��ʾ ����ת��   
//        OLED_ShowNum(3, 12,LL_Value,3);//��ʾ ����ת��   

        #endif
        vTaskDelay(100); //��ʱ 1s��Ҳ���� 1000 ��ʱ�ӽ���
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
        
        sprintf(CAN_Txbuf[0], "%d", (int)pitch ); /* ������ pitch ת�����ַ���ʽ ��ֵ������*/
        sprintf(CAN_Txbuf[1], "%d", (int)roll );
        sprintf(CAN_Txbuf[2], "%d", (int)AHT10_temp); /* ������ pitch ת�����ַ���ʽ ��ֵ������*/
        sprintf(CAN_Txbuf[3], "%d", (int)AHT10_humi);
        sprintf(CAN_Txbuf[4], "endof\n");

        printf("Pitch:%s\n", CAN_Txbuf[0]);
        printf("Roll:%s\n", CAN_Txbuf[1]);
        printf("Temp:%sC\n", CAN_Txbuf[2]);
        printf("Humi:%%%s\n", CAN_Txbuf[3]);

        for ( i = 0; i < 5; i++)
        {
            res=Can_Send_Msg(CAN_Txbuf[i],sizeof(CAN_Txbuf[i]));//����8���ֽ� 
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

       
        vTaskDelay(500); //��ʱ 1s��Ҳ���� 1000 ��ʱ�ӽ���
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
                   
                wheels_RL(2,2);//����ͣת
                
                //ret=xSemaphoreGive(Binary_Semaphore);
                //configASSERT(ret);/*�ͷ��ź��� +1 ����0��ʧ��*/
                
                pwm_R=0;
                pwm_L=0;
                TIM_SetCompare1(TIM3, pwm_R);
                TIM_SetCompare2(TIM3, pwm_L);

            }
            else if ( !strcmp(CAN_Rxbuf,"1") )
            {/*---------Go Forward-------------*/

                wheels_RL(0,0);//������ת
                pwm_R+=50;
                pwm_L+=50;
                if((pwm_R>=1000) || (pwm_L>=1000) ){ pwm_R=1000;pwm_L=1000; }
                TIM_SetCompare1(TIM3, pwm_R);
                TIM_SetCompare2(TIM3, pwm_L);

            }
            else if (!strcmp(CAN_Rxbuf,"2") )
            {/*---------Back-------------*/

                wheels_RL(1,1);//���ӷ�ת
                pwm_R+=50;
                pwm_L+=50;
                if((pwm_R>=1000) || (pwm_L>=1000) ){ pwm_R=1000;pwm_L=1000; }
                TIM_SetCompare1(TIM3, pwm_R);
                TIM_SetCompare2(TIM3, pwm_L);

            }            
            else if (!strcmp(CAN_Rxbuf,"3") )
            {/*---------Left-------------*/

                wheels_RL(0,0);//������ת
                pwm_R+=50;
                if(pwm_R>=1000) pwm_R=1000;

                if(pwm_L>20) pwm_L-=20; 
                else pwm_L=0;

                TIM_SetCompare1(TIM3, pwm_R);
                TIM_SetCompare2(TIM3, pwm_L);

            }
            else if (!strcmp(CAN_Rxbuf,"4") )
            {/*---------Right-------------*/
                wheels_RL(0,0);//������ת
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
        
        vTaskDelay(500);//��ʱ 100��tick 100��ʱ�ӽ���
    }
}





void Task_3( void * pvParameters )
{
   // BaseType_t ret=0;           

	while(1)
    {


        vTaskDelay(10); //��ʱ s��Ҳ���� 1000 ��ʱ�ӽ���
    }

  
}














