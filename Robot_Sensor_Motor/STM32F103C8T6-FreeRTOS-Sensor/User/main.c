/**
 * @file main.c  
 * @author wls   
 * @brief ������ 
 * @version 0.1  
 * @date 2022-11-01  
 * 
 * @copyright Copyright (c) 2022  
 * 
 */   
#include "WTask.h" //�û��Զ�������






/**
 * @brief ������ 
 * 
 * @return int 
 */
int main(void)
{   
	
    HardWare_Init();// Ӳ����ʼ��       	  

    Start_Task();   //��ʼ��������

    while (1); /* ��������ִ�е����� */

}











