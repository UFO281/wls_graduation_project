/**
 * @file main.c  
 * @author wls   
 * @brief 主函数 
 * @version 0.1  
 * @date 2022-11-01  
 * 
 * @copyright Copyright (c) 2022  
 * 
 */   
#include "WTask.h" //用户自定义任务






/**
 * @brief 主函数 
 * 
 * @return int 
 */
int main(void)
{   
	
    HardWare_Init();// 硬件初始化       	  

    Start_Task();   //开始任务任务

    while (1); /* 正常不会执行到这里 */

}











