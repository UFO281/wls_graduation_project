/**
 * @file main.c
 * @author SIE-Robot202-Wang Ling Shuo
 * @brief Wireless control terminal for graduation project robot project
 * @version 0.1
 * @date 2024-03-16
 * 
 * @copyright Copyright (c) 2024
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











