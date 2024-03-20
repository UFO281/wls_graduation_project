/**
 * @file app_gpio.c
 * @author wls
 * @brief CAN RX/TX
 * @version 0.1
 * @date 2024-02-24
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <pthread.h>


#define CAN_DEV_ID     ( (unsigned int)0x1234 ) /* CAN 设备ID*/









int CAN_TR_Data(void)
{

    unsigned char can_rx_lenth=0; 
    char CAN_Txbuf[]={"Iam_King!"}; 
	char CAN_Rxbuf[64]={0}; 
   
    int sockfd;                      // CAN套接字的文件描述符  
    struct ifreq ifr;                // 用于ioctl调用以获取CAN接口索引的结构体  
    struct sockaddr_can addr={0};      // CAN接口的地址信息  
    struct can_frame CAN_TxFrame;          // CAN帧的结构体  
    struct can_frame CAN_RxFrame;          // CAN发送帧的结构体  
    const char *ifname = "can0";     // CAN接口的名称，这里假设为"can0" 

    // 创建CAN原始套接字  
    sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);  
    if (sockfd < 0) 
    {  
        perror("创建CAN套接字失败");  
        return -1;  
    }  
  
    // 获取CAN接口的索引值  
    strcpy(ifr.ifr_name, ifname);  
    if (ioctl(sockfd, SIOCGIFINDEX, &ifr) < 0) 
    {  
        perror("获取CAN接口索引失败");  
        close(sockfd);  
        return -1;  
    }  
  
    // 绑定CAN接口到套接字  
    addr.can_family = AF_CAN;            // 设置地址族为CAN  
    addr.can_ifindex = ifr.ifr_ifindex;  // 设置CAN接口的索引值  
  
    if (bind(sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {  
        perror("绑定CAN接口到套接字失败");  
        close(sockfd);  
        return -1;  
    }  
  
    // 设置要发送的CAN帧  
    CAN_TxFrame.can_id = CAN_DEV_ID| CAN_EFF_FLAG ;  // 设置CAN帧的ID  
    CAN_TxFrame.can_dlc = 8;   // 设置CAN帧的数据长度，最大为8字节 


    strcpy(CAN_TxFrame.data ,CAN_Txbuf);// 写入CAN Send Data

    
    while (1) 
    {  
        /*-----------------------CAN Send-------------------------------------*/
        // 发送CAN帧到CAN总线  
        if (write(sockfd, &CAN_TxFrame, sizeof(struct can_frame)) != sizeof(struct can_frame)) 
        {  
            perror("Send CAN_TxFrame ERROR!");  
            break;  
        }
        else{
            printf("Send CAN ok!\n");  
        }
        /*-----------------------CAN Send-------------------------------------*/


        /*-----------------------CAN Recive----------------------------*/
        if (0 > read(sockfd, &CAN_RxFrame, sizeof(struct can_frame)))
        {
			perror("read error");
			break;
		}

		/* 校验是否接收到错误帧 */
		if (CAN_RxFrame.can_id & CAN_ERR_FLAG)
        {
			printf("Error frame!\n");
			break;
		}

		/* 校验帧格式 */
		if (CAN_RxFrame.can_id & CAN_EFF_FLAG)	//扩展帧
			printf("extended <ID:%0x> ", CAN_RxFrame.can_id & CAN_EFF_MASK);
		else //标准帧
			printf("Standard <ID:%0X> ", CAN_RxFrame.can_id & CAN_SFF_MASK);

		/* 校验帧类型：数据帧还是远程帧 */
		if (CAN_RxFrame.can_id & CAN_RTR_FLAG) 
        {
			printf("remote request\n");
			continue;
		}

		/* 打印数据长度 */
        can_rx_lenth = CAN_RxFrame.can_dlc;
        strcpy(CAN_Rxbuf,CAN_RxFrame.data);
		printf("Rxdata[%d]:%s",can_rx_lenth,CAN_Rxbuf);
		printf("\n");
        can_rx_lenth=0;
        CAN_RxFrame.can_dlc = 0;
        memset(CAN_Rxbuf,0,sizeof(CAN_Rxbuf));
        memset(CAN_RxFrame.data,0,sizeof(CAN_RxFrame.data));

        /*-----------------------CAN Recive----------------------------*/
  
         
        sleep(1);// 如果需要，在这里添加发送延迟   
    }  
  
     
    close(sockfd); // 关闭CAN file 
  
    return 0;  

}














