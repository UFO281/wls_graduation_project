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


#define CAN_DEV_ID     ( (unsigned int)0x1234 ) /* CAN �豸ID*/









int CAN_TR_Data(void)
{

    unsigned char can_rx_lenth=0; 
    char CAN_Txbuf[]={"Iam_King!"}; 
	char CAN_Rxbuf[64]={0}; 
   
    int sockfd;                      // CAN�׽��ֵ��ļ�������  
    struct ifreq ifr;                // ����ioctl�����Ի�ȡCAN�ӿ������Ľṹ��  
    struct sockaddr_can addr={0};      // CAN�ӿڵĵ�ַ��Ϣ  
    struct can_frame CAN_TxFrame;          // CAN֡�Ľṹ��  
    struct can_frame CAN_RxFrame;          // CAN����֡�Ľṹ��  
    const char *ifname = "can0";     // CAN�ӿڵ����ƣ��������Ϊ"can0" 

    // ����CANԭʼ�׽���  
    sockfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);  
    if (sockfd < 0) 
    {  
        perror("����CAN�׽���ʧ��");  
        return -1;  
    }  
  
    // ��ȡCAN�ӿڵ�����ֵ  
    strcpy(ifr.ifr_name, ifname);  
    if (ioctl(sockfd, SIOCGIFINDEX, &ifr) < 0) 
    {  
        perror("��ȡCAN�ӿ�����ʧ��");  
        close(sockfd);  
        return -1;  
    }  
  
    // ��CAN�ӿڵ��׽���  
    addr.can_family = AF_CAN;            // ���õ�ַ��ΪCAN  
    addr.can_ifindex = ifr.ifr_ifindex;  // ����CAN�ӿڵ�����ֵ  
  
    if (bind(sockfd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {  
        perror("��CAN�ӿڵ��׽���ʧ��");  
        close(sockfd);  
        return -1;  
    }  
  
    // ����Ҫ���͵�CAN֡  
    CAN_TxFrame.can_id = CAN_DEV_ID| CAN_EFF_FLAG ;  // ����CAN֡��ID  
    CAN_TxFrame.can_dlc = 8;   // ����CAN֡�����ݳ��ȣ����Ϊ8�ֽ� 


    strcpy(CAN_TxFrame.data ,CAN_Txbuf);// д��CAN Send Data

    
    while (1) 
    {  
        /*-----------------------CAN Send-------------------------------------*/
        // ����CAN֡��CAN����  
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

		/* У���Ƿ���յ�����֡ */
		if (CAN_RxFrame.can_id & CAN_ERR_FLAG)
        {
			printf("Error frame!\n");
			break;
		}

		/* У��֡��ʽ */
		if (CAN_RxFrame.can_id & CAN_EFF_FLAG)	//��չ֡
			printf("extended <ID:%0x> ", CAN_RxFrame.can_id & CAN_EFF_MASK);
		else //��׼֡
			printf("Standard <ID:%0X> ", CAN_RxFrame.can_id & CAN_SFF_MASK);

		/* У��֡���ͣ�����֡����Զ��֡ */
		if (CAN_RxFrame.can_id & CAN_RTR_FLAG) 
        {
			printf("remote request\n");
			continue;
		}

		/* ��ӡ���ݳ��� */
        can_rx_lenth = CAN_RxFrame.can_dlc;
        strcpy(CAN_Rxbuf,CAN_RxFrame.data);
		printf("Rxdata[%d]:%s",can_rx_lenth,CAN_Rxbuf);
		printf("\n");
        can_rx_lenth=0;
        CAN_RxFrame.can_dlc = 0;
        memset(CAN_Rxbuf,0,sizeof(CAN_Rxbuf));
        memset(CAN_RxFrame.data,0,sizeof(CAN_RxFrame.data));

        /*-----------------------CAN Recive----------------------------*/
  
         
        sleep(1);// �����Ҫ����������ӷ����ӳ�   
    }  
  
     
    close(sockfd); // �ر�CAN file 
  
    return 0;  

}














