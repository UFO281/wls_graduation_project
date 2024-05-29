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
#include "WTask.h" //用户自定义任务


int main(void)
{   
	
    HardWare_Init();// 硬件初始化       	  

    Start_Task();   //开始任务任务

    while (1); /* 正常不会执行到这里 */

}











