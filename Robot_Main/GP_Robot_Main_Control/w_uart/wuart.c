/**
 * @file wuart.c
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2024-03-19
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#define _GNU_SOURCE     //��Դ�ļ���ͷ����_GNU_SOURCE��
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <termios.h>
#include <pthread.h>

#include "wuart.h" 





static struct termios old_cfg;  //���ڱ����ն˵����ò���
static int fd;      //�����ն˶�Ӧ���ļ�������



/**
 ** ���ڳ�ʼ������
 ** ����device��ʾ�����ն˵��豸�ڵ�
 **/
static int uart_init(const char *device)
{
    /* �򿪴����ն� */
    fd = open(device, O_RDWR | O_NOCTTY);
    if (0 > fd) 
    {
        fprintf(stderr, "open error: %s: %s\n", device, strerror(errno));
        return -1;
    }

    /* ��ȡ���ڵ�ǰ�����ò��� */
    if (0 > tcgetattr(fd, &old_cfg)) 
    {
        fprintf(stderr, "tcgetattr error: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    return 0;
}





/**
 * @brief ��������
 * 
 * @param cfg ָ��һ��uart_cfg_t�ṹ�����
 * @return int 
 */
static int uart_cfg(const uart_cfg_t *cfg)
{
    struct termios new_cfg = {0};   //��new_cfg��������
    speed_t speed;

    /* ����Ϊԭʼģʽ */
    cfmakeraw(&new_cfg);

    /* ʹ�ܽ��� */
    new_cfg.c_cflag |= CREAD;

    /* ���ò����� */
    switch (cfg->baudrate) 
    {
        case 1200: speed = B1200;
            break;
        case 1800: speed = B1800;
            break;
        case 2400: speed = B2400;
            break;
        case 4800: speed = B4800;
            break;
        case 9600: speed = B9600;
            break;
        case 19200: speed = B19200;
            break;
        case 38400: speed = B38400;
            break;
        case 57600: speed = B57600;
            break;
        case 115200: speed = B115200;
            break;
        case 230400: speed = B230400;
            break;
        case 460800: speed = B460800;
            break;
        case 500000: speed = B500000;
            break;
        default:    //Ĭ������Ϊ115200
            speed = B115200;
            // printf("default baud rate: 115200\n");
            break;
    }

    if (0 > cfsetspeed(&new_cfg, speed))
    {
        fprintf(stderr, "cfsetspeed error: %s\n", strerror(errno));
        return -1;
    }

    /* ��������λ��С */
    new_cfg.c_cflag &= ~CSIZE;  //������λ��صı���λ����
    switch (cfg->dbit) 
    {
        case 5:
            new_cfg.c_cflag |= CS5;
            break;
        case 6:
            new_cfg.c_cflag |= CS6;
            break;
        case 7:
            new_cfg.c_cflag |= CS7;
            break;
        case 8:
            new_cfg.c_cflag |= CS8;
            break;
        default:    //Ĭ������λ��СΪ8
            new_cfg.c_cflag |= CS8;
            // printf("default data bit size: 8\n");
            break;
    }

    /* ������żУ�� */
    switch (cfg->parity) 
    {
        case 'N':       //��У��
            new_cfg.c_cflag &= ~PARENB;
            new_cfg.c_iflag &= ~INPCK;
            break;
        case 'O':       //��У��
            new_cfg.c_cflag |= (PARODD | PARENB);
            new_cfg.c_iflag |= INPCK;
            break;
        case 'E':       //żУ��
            new_cfg.c_cflag |= PARENB;
            new_cfg.c_cflag &= ~PARODD; /* ���PARODD��־������ΪżУ�� */
            new_cfg.c_iflag |= INPCK;
            break;
        default:    //Ĭ������Ϊ��У��
            new_cfg.c_cflag &= ~PARENB;
            new_cfg.c_iflag &= ~INPCK;
            // printf("default parity: N\n");
            break;
    }

    /* ����ֹͣλ */
    switch (cfg->sbit) 
    {
        case 1:     //1��ֹͣλ
            new_cfg.c_cflag &= ~CSTOPB;
            break;
        case 2:     //2��ֹͣλ
            new_cfg.c_cflag |= CSTOPB;
            break;
        default:    //Ĭ������Ϊ1��ֹͣλ
            new_cfg.c_cflag &= ~CSTOPB;
            // printf("default stop bit size: 1\n");
            break;
    }

    /* ��MIN��TIME����Ϊ0 */
    new_cfg.c_cc[VTIME] = 0;
    new_cfg.c_cc[VMIN] = 0;

    /* ��ջ����� */
    if (0 > tcflush(fd, TCIOFLUSH))
    {
        fprintf(stderr, "tcflush error: %s\n", strerror(errno));
        return -1;
    }

    /* д�����á�ʹ������Ч */
    if (0 > tcsetattr(fd, TCSANOW, &new_cfg)) 
    {
        fprintf(stderr, "tcsetattr error: %s\n", strerror(errno));
        return -1;
    }

    /* ����OK �˳� */
    return 0;
}


/***
--dev=/dev/ttymxc2
--brate=115200
--dbit=8
--parity=N
--sbit=1
--type=read
***/


/**
 * @brief ��ӡ������Ϣ
 * 
 * @param app 
 */
static void show_help(const char *app)
{
    printf("Usage: %s [ѡ��]\n"
        "\n��ѡѡ��:\n"
        "  --dev=DEVICE     ָ�������ն��豸����, Ʃ��--dev=/dev/ttymxc2\n"
        "  --type=TYPE      ָ����������, �����ڻ���д����, Ʃ��--type=read(read��ʾ����write��ʾд������ֵ��Ч)\n"
        "\n��ѡѡ��:\n"
        "  --brate=SPEED    ָ�����ڲ�����, Ʃ��--brate=115200\n"
        "  --dbit=SIZE      ָ����������λ����, Ʃ��--dbit=8(��ȡֵΪ: 5/6/7/8)\n"
        "  --parity=PARITY  ָ��������żУ�鷽ʽ, Ʃ��--parity=N(N��ʾ��У�顢O��ʾ��У�顢E��ʾżУ��)\n"
        "  --sbit=SIZE      ָ������ֹͣλ����, Ʃ��--sbit=1(��ȡֵΪ: 1/2)\n"
        "  --help           �鿴������ʹ�ð�����Ϣ\n\n", app);
}



/**
 * @brief �źŴ������������������ݿɶ�ʱ������ת���ú���ִ��
 * 
 * @param sig 
 * @param info 
 * @param context 
 */
static void io_handler(int sig, siginfo_t *info, void *context)
{

    // printf(" Uart Rx io_handler\n");

    unsigned char buf[64] = {0};
    int ret;
    int n;

    if(SIGRTMIN != sig) return;

       
    /* �жϴ����Ƿ������ݿɶ� */
    if (POLL_IN == info->si_code) 
    {
        ret = read(fd, buf, 8);     //һ������8���ֽ�����
        write(fd, buf, 8); 	//һ���򴮿�д��8���ֽ�

        printf("%s",buf);

    }

}



/**
 ** �첽I/O��ʼ������
 **/
static void async_io_init(void)
{
    struct sigaction sigatn;
    int flag;

    /* ʹ���첽I/O */
    flag = fcntl(fd, F_GETFL);  //ʹ�ܴ��ڵ��첽I/O����
    flag |= O_ASYNC;
    fcntl(fd, F_SETFL, flag);

    /* �����첽I/O�������� */
    fcntl(fd, F_SETOWN, getpid());

    /* ָ��ʵʱ�ź�SIGRTMIN��Ϊ�첽I/O֪ͨ�ź� */
    fcntl(fd, F_SETSIG, SIGRTMIN);

    /* Ϊʵʱ�ź�SIGRTMINע���źŴ����� */
    sigatn.sa_sigaction = io_handler;   //�����������ݿɶ�ʱ������ת��io_handler����
    sigatn.sa_flags = SA_SIGINFO;
    sigemptyset(&sigatn.sa_mask);
    sigaction(SIGRTMIN, &sigatn, NULL);
}



int uart_main(void)
{

    uart_cfg_t cfg = {0};
    char *device = "/dev/ttymxc2";

    unsigned char w_buf[] = {"wahaha\r\n"};    //ͨ�����ڷ��ͳ�ȥ������


    /* ���ڳ�ʼ�� */
    if (uart_init(device))
        exit(EXIT_FAILURE);

    /* �������� */
    if (uart_cfg(&cfg)) 
    {
        tcsetattr(fd, TCSANOW, &old_cfg);   //�ָ���֮ǰ������
        close(fd);
        exit(EXIT_FAILURE);
    }



    async_io_init();	//����ʹ���첽I/O��ʽ��ȡ���ڵ����ݣ����øú���ȥ��ʼ�����ڵ��첽I/O


    // while (1);
    
       

    /* �˳� */
    // tcsetattr(fd, TCSANOW, &old_cfg);   //�ָ���֮ǰ������
    // close(fd);
    // exit(EXIT_SUCCESS);

    
}


