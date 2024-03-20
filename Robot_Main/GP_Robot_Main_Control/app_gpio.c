/**
 * @file app_gpio.c
 * @author Robot202 WLS
 * @brief Robot Mian Control
 * @version 0.1
 * @date 2024-03-19
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#define _GNU_SOURCE     //在源文件开头定义_GNU_SOURCE宏
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
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
// #include "w_uart/wuart.h"
// #include "w_can/wcan.h"
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <linux/fb.h>




#define CAN_DEV_ID     ( (unsigned int)0x1234 ) /* CAN 设备ID*/
#define NUM_THREADS     3





static struct termios old_cfg;  //用于保存终端的配置参数
static int uart3_fd;      //串口终端对应的文件描述符


/**
 * @brief uart config struct
 * 
 */
typedef struct uart_hardware_cfg 
{
    unsigned int baudrate;      /* 波特率 */
    unsigned char dbit;         /* 数据位 */
    char parity;                /* 奇偶校验 */
    unsigned char sbit;         /* 停止位 */
} uart_cfg_t;



/**
 ** 串口初始化操作
 ** 参数device表示串口终端的设备节点
 **/
static int uart_init(const char *device)
{
    /* 打开串口终端 */
    uart3_fd = open(device, O_RDWR | O_NOCTTY);
    if (0 > uart3_fd) 
    {
        fprintf(stderr, "open error: %s: %s\n", device, strerror(errno));
        return -1;
    }

    /* 获取串口当前的配置参数 */
    if (0 > tcgetattr(uart3_fd, &old_cfg)) 
    {
        fprintf(stderr, "tcgetattr error: %s\n", strerror(errno));
        close(uart3_fd);
        return -1;
    }

    return 0;
}





/**
 * @brief 串口配置
 * 
 * @param cfg 指向一个uart_cfg_t结构体对象
 * @return int 
 */
static int uart_cfg(const uart_cfg_t *cfg)
{
    struct termios new_cfg = {0};   //将new_cfg对象清零
    speed_t speed;

    /* 设置为原始模式 */
    cfmakeraw(&new_cfg);

    /* 使能接收 */
    new_cfg.c_cflag |= CREAD;

    /* 设置波特率 */
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
        default:    //默认配置为115200
            speed = B115200;
            // printf("default baud rate: 115200\n");
            break;
    }

    if (0 > cfsetspeed(&new_cfg, speed))
    {
        fprintf(stderr, "cfsetspeed error: %s\n", strerror(errno));
        return -1;
    }

    /* 设置数据位大小 */
    new_cfg.c_cflag &= ~CSIZE;  //将数据位相关的比特位清零
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
        default:    //默认数据位大小为8
            new_cfg.c_cflag |= CS8;
            // printf("default data bit size: 8\n");
            break;
    }

    /* 设置奇偶校验 */
    switch (cfg->parity) 
    {
        case 'N':       //无校验
            new_cfg.c_cflag &= ~PARENB;
            new_cfg.c_iflag &= ~INPCK;
            break;
        case 'O':       //奇校验
            new_cfg.c_cflag |= (PARODD | PARENB);
            new_cfg.c_iflag |= INPCK;
            break;
        case 'E':       //偶校验
            new_cfg.c_cflag |= PARENB;
            new_cfg.c_cflag &= ~PARODD; /* 清除PARODD标志，配置为偶校验 */
            new_cfg.c_iflag |= INPCK;
            break;
        default:    //默认配置为无校验
            new_cfg.c_cflag &= ~PARENB;
            new_cfg.c_iflag &= ~INPCK;
            // printf("default parity: N\n");
            break;
    }

    /* 设置停止位 */
    switch (cfg->sbit) 
    {
        case 1:     //1个停止位
            new_cfg.c_cflag &= ~CSTOPB;
            break;
        case 2:     //2个停止位
            new_cfg.c_cflag |= CSTOPB;
            break;
        default:    //默认配置为1个停止位
            new_cfg.c_cflag &= ~CSTOPB;
            // printf("default stop bit size: 1\n");
            break;
    }

    /* 将MIN和TIME设置为0 */
    new_cfg.c_cc[VTIME] = 0;
    new_cfg.c_cc[VMIN] = 0;

    /* 清空缓冲区 */
    if (0 > tcflush(uart3_fd, TCIOFLUSH))
    {
        fprintf(stderr, "tcflush error: %s\n", strerror(errno));
        return -1;
    }

    /* 写入配置、使配置生效 */
    if (0 > tcsetattr(uart3_fd, TCSANOW, &new_cfg)) 
    {
        fprintf(stderr, "tcsetattr error: %s\n", strerror(errno));
        return -1;
    }

    /* 配置OK 退出 */
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
 * @brief 打印帮助信息
 * 
 * @param app 
 */
static void show_help(const char *app)
{
    printf("Usage: %s [选项]\n"
        "\n必选选项:\n"
        "  --dev=DEVICE     指定串口终端设备名称, 譬如--dev=/dev/ttymxc2\n"
        "  --type=TYPE      指定操作类型, 读串口还是写串口, 譬如--type=read(read表示读、write表示写、其它值无效)\n"
        "\n可选选项:\n"
        "  --brate=SPEED    指定串口波特率, 譬如--brate=115200\n"
        "  --dbit=SIZE      指定串口数据位个数, 譬如--dbit=8(可取值为: 5/6/7/8)\n"
        "  --parity=PARITY  指定串口奇偶校验方式, 譬如--parity=N(N表示无校验、O表示奇校验、E表示偶校验)\n"
        "  --sbit=SIZE      指定串口停止位个数, 譬如--sbit=1(可取值为: 1/2)\n"
        "  --help           查看本程序使用帮助信息\n\n", app);
}




unsigned char Uart_RxBuf[8] = {0};
// char Uart_TxBuf[5][4]={0}; 


/**
 * @brief 信号处理函数，当串口有数据可读时，会跳转到该函数执行
 * 
 * @param sig 
 * @param info 
 * @param context 
 */
static void io_handler(int sig, siginfo_t *info, void *context)
{

    // printf(" Uart Rx io_handler\n");

    int ret;
    int n;

    if(SIGRTMIN != sig) return;

       
    /* 判断串口是否有数据可读 */
    if (POLL_IN == info->si_code) 
    {
        ret = read(uart3_fd, Uart_RxBuf, 8);     //一次最多读8个字节数据
        // write(uart3_fd, Uart_RxBuf, 8); 	//一次向串口写入8个字节
        // write(uart3_fd, Uart_RxBuf, 8); 	//一次向串口写入8个字节
        printf("%s",Uart_RxBuf);

    }

}



/**
 ** 异步I/O初始化函数
 **/
static void async_io_init(void)
{
    struct sigaction sigatn;
    int flag;

    /* 使能异步I/O */
    flag = fcntl(uart3_fd, F_GETFL);  //使能串口的异步I/O功能
    flag |= O_ASYNC;
    fcntl(uart3_fd, F_SETFL, flag);

    /* 设置异步I/O的所有者 */
    fcntl(uart3_fd, F_SETOWN, getpid());

    /* 指定实时信号SIGRTMIN作为异步I/O通知信号 */
    fcntl(uart3_fd, F_SETSIG, SIGRTMIN);

    /* 为实时信号SIGRTMIN注册信号处理函数 */
    sigatn.sa_sigaction = io_handler;   //当串口有数据可读时，会跳转到io_handler函数
    sigatn.sa_flags = SA_SIGINFO;
    sigemptyset(&sigatn.sa_mask);
    sigaction(SIGRTMIN, &sigatn, NULL);
}





/**
 * @brief Processing serial port data
 * 
 * @param number 
 * @return void* 
 */
void *Uart_process_thread(void *number)
{
    printf("Uart_process_thread: start \n");

    uart_cfg_t cfg = {0};
    char *device = "/dev/ttymxc2";

    // unsigned char w_buf[] = {"wahaha\r\n"};    //通过串口发送出去的数据


    /* 串口初始化 */
    if (uart_init(device))
        exit(EXIT_FAILURE);

    /* 串口配置 */
    if (uart_cfg(&cfg)) 
    {
        tcsetattr(uart3_fd, TCSANOW, &old_cfg);   //恢复到之前的配置
        close(uart3_fd);
        exit(EXIT_FAILURE);
    }

    async_io_init();	//我们使用异步I/O方式读取串口的数据，调用该函数去初始化串口的异步I/O

    while (1)
    {

        printf("Uart_process_thread: ID %ld\n", number);
        sleep(1);
        // pthread_exit(NULL);
    
    }

    /* 退出 */
    // tcsetattr(uart3_fd, TCSANOW, &old_cfg);   //恢复到之前的配置
    // close(uart3_fd);
    // exit(EXIT_SUCCESS);
    

}


/**
 * @brief Processing CAN data
 * 
 * @param number 
 * @return void* 
 */
void *CAN_process_thread(void *number)
{

    printf("CAN_process_thread: start:  \n");
    // 关闭 can0 设备
    system("ifconfig can0 down");
    // 设置 can0 设备 100Kbps
    system("ip link set can0 up type can bitrate 100000 triple-sampling on");

    unsigned char can_rx_lenth=0; 
    // char *CAN_Txbuf[]={"0","1","2","3","4","5"}; 
    char CAN_Rxbuf[5][4]={0}; 

   
    int CAN_fd;                      // CAN套接字的文件描述符  
    struct ifreq ifr;                // 用于ioctl调用以获取CAN接口索引的结构体  
    struct sockaddr_can addr={0};      // CAN接口的地址信息  
    struct can_frame CAN_TxFrame;          // CAN帧的结构体  
    struct can_frame CAN_RxFrame;          // CAN发送帧的结构体  
    const char *ifname = "can0";     // CAN接口的名称，这里假设为"can0" 

    // 创建CAN原始套接字  
    CAN_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);  
    if (CAN_fd < 0) 
    {  
        perror("创建CAN套接字失败");  
        // return -1;  
    }  
  
    // 获取CAN接口的索引值  
    strcpy(ifr.ifr_name, ifname);  
    if (ioctl(CAN_fd, SIOCGIFINDEX, &ifr) < 0) 
    {  
        perror("获取CAN接口索引失败");  
        close(CAN_fd);  
        // return -1;  
    }  
  
    // 绑定CAN接口到套接字  
    addr.can_family = AF_CAN;            // 设置地址族为CAN  
    addr.can_ifindex = ifr.ifr_ifindex;  // 设置CAN接口的索引值  
  
    if (bind(CAN_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {  
        perror("绑定CAN接口到套接字失败");  
        close(CAN_fd);  
        // return -1;  
    }  
  
    // 设置要发送的CAN帧  
    CAN_TxFrame.can_id = CAN_DEV_ID| CAN_EFF_FLAG ;  // 设置CAN帧的ID  
    CAN_TxFrame.can_dlc = 8;   // 设置CAN帧的数据长度，最大为8字节 



    int i=0; 
    int n=0; 

    while (1)
    {

        strcpy(CAN_TxFrame.data ,Uart_RxBuf);// Uart_Rx_data to can data

        /*-----------------------CAN Send-------------------------------------*/
        // 发送CAN帧到CAN总线  
        if ( write(CAN_fd, &CAN_TxFrame, sizeof(struct can_frame) ) != sizeof(struct can_frame)) 
        {  
            perror("Send CAN_TxFrame ERROR!");  
            // break;  
        }
        else{
            printf("Send CAN ok!\n");  
        }
        /*-----------------------CAN Send-------------------------------------*/



        /*-----------------------CAN Recive----------------------------*/
        if (0 > read(CAN_fd, &CAN_RxFrame, sizeof(struct can_frame)))
        {
			perror("read error");
			// break;
		}

		/* 校验是否接收到错误帧 */
		if (CAN_RxFrame.can_id & CAN_ERR_FLAG)
        {
			printf("Error frame!\n");
			// break;
		}

		/* 校验帧格式 */
		if (CAN_RxFrame.can_id & CAN_EFF_FLAG)	//扩展帧
			printf("extended <ID:%0x> \n", CAN_RxFrame.can_id & CAN_EFF_MASK);
		else //标准帧
			printf("Standard <ID:%0X> \n", CAN_RxFrame.can_id & CAN_SFF_MASK);

		/* 校验帧类型：数据帧还是远程帧 */
		if (CAN_RxFrame.can_id & CAN_RTR_FLAG) 
        {
			printf("remote request\n");
			continue;
		}

        strcpy( CAN_Rxbuf[i],CAN_RxFrame.data );

        if ( !strcmp("end_of",CAN_RxFrame.data) )
        {   /*接收到末尾数据*/
           
            can_rx_lenth=n;
            n=0;
            i=0;
            
            printf("Rxdata[%d]:%s",can_rx_lenth,CAN_Rxbuf);

            printf("Pitch:%s\n", CAN_Rxbuf[0]);
            printf("Roll:%s\n", CAN_Rxbuf[1]);
            printf("Temp:%sC\n", CAN_Rxbuf[2]);
            printf("Humi:%%%s\n", CAN_Rxbuf[3]);

            write(uart3_fd, CAN_Rxbuf, sizeof(CAN_Rxbuf)); /* write to uart data uart send data*/	

            can_rx_lenth=0;
            memset(CAN_Rxbuf,0,sizeof(CAN_Rxbuf));
            memset(CAN_RxFrame.data,0,sizeof(CAN_RxFrame.data));
            printf("\nreset can rxbuf ok\n");

        }
        ++i;    
        ++n;
        
        /*-----------------------CAN Recive----------------------------*/


        printf("CAN_process_thread: ID %ld\n", number);


        sleep(1);
    
    }
    
    close(CAN_fd); // 关闭CAN file 

}




#define Camer_Dev           "/dev/video1" /* 摄像头设备*/
#define FB_DEV              "/dev/fb0"      //LCD设备节点
#define FRAMEBUFFER_COUNT   3               //帧缓冲数量

/*** 摄像头像素格式及其描述信息 ***/
typedef struct camera_format 
{
    unsigned char description[32];  //字符串描述信息
    unsigned int pixelformat;       //像素格式
} cam_fmt;

/*** 描述一个帧缓冲的信息 ***/
typedef struct cam_buf_info 
{
    unsigned short *start;      //帧缓冲起始地址
    unsigned long length;       //帧缓冲长度
} cam_buf_info;

static int width;                       //LCD宽度
static int height;                      //LCD高度
static unsigned short *screen_base = NULL;//LCD显存基地址
static int fb_fd = -1;                  //LCD设备文件描述符
static int v4l2_fd = -1;                //摄像头设备文件描述符
static cam_buf_info buf_infos[FRAMEBUFFER_COUNT];
static cam_fmt cam_fmts[10];
static int frm_width, frm_height;   //视频帧宽度和高度

static int fb_dev_init(void)
{
    struct fb_var_screeninfo fb_var = {0};
    struct fb_fix_screeninfo fb_fix = {0};
    unsigned long screen_size;

    /* 打开framebuffer设备 */
    fb_fd = open(FB_DEV, O_RDWR);
    if (0 > fb_fd) {
        fprintf(stderr, "open error: %s: %s\n", FB_DEV, strerror(errno));
        return -1;
    }

    /* 获取framebuffer设备信息 */
    ioctl(fb_fd, FBIOGET_VSCREENINFO, &fb_var);
    ioctl(fb_fd, FBIOGET_FSCREENINFO, &fb_fix);

    screen_size = fb_fix.line_length * fb_var.yres;
    width = fb_var.xres;
    height = fb_var.yres;

    /* 内存映射 */
    screen_base = mmap(NULL, screen_size, PROT_READ | PROT_WRITE, MAP_SHARED, fb_fd, 0);
    if (MAP_FAILED == (void *)screen_base) {
        perror("mmap error");
        close(fb_fd);
        return -1;
    }

    /* LCD背景刷白 */
    memset(screen_base, 0xFF, screen_size);
    return 0;
}

static int v4l2_dev_init(const char *device)
{
    struct v4l2_capability cap = {0};

    /* 打开摄像头 */
    v4l2_fd = open(device, O_RDWR);
    if (0 > v4l2_fd) {
        fprintf(stderr, "open error: %s: %s\n", device, strerror(errno));
        return -1;
    }

    /* 查询设备功能 */
    ioctl(v4l2_fd, VIDIOC_QUERYCAP, &cap);

    /* 判断是否是视频采集设备 */
    if (!(V4L2_CAP_VIDEO_CAPTURE & cap.capabilities)) {
        fprintf(stderr, "Error: %s: No capture video device!\n", device);
        close(v4l2_fd);
        return -1;
    }

    return 0;
}

static void v4l2_enum_formats(void)
{
    struct v4l2_fmtdesc fmtdesc = {0};

    /* 枚举摄像头所支持的所有像素格式以及描述信息 */
    fmtdesc.index = 0;
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    while (0 == ioctl(v4l2_fd, VIDIOC_ENUM_FMT, &fmtdesc)) {

        // 将枚举出来的格式以及描述信息存放在数组中
        cam_fmts[fmtdesc.index].pixelformat = fmtdesc.pixelformat;
        strcpy(cam_fmts[fmtdesc.index].description, fmtdesc.description);
        fmtdesc.index++;
    }
}

static void v4l2_print_formats(void)
{
    struct v4l2_frmsizeenum frmsize = {0};
    struct v4l2_frmivalenum frmival = {0};
    int i;

    frmsize.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    frmival.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    for (i = 0; cam_fmts[i].pixelformat; i++) {

        printf("format<0x%x>, description<%s>\n", cam_fmts[i].pixelformat,
                    cam_fmts[i].description);

        /* 枚举出摄像头所支持的所有视频采集分辨率 */
        frmsize.index = 0;
        frmsize.pixel_format = cam_fmts[i].pixelformat;
        frmival.pixel_format = cam_fmts[i].pixelformat;
        while (0 == ioctl(v4l2_fd, VIDIOC_ENUM_FRAMESIZES, &frmsize)) {

            printf("size<%d*%d> ",
                    frmsize.discrete.width,
                    frmsize.discrete.height);
            frmsize.index++;

            /* 获取摄像头视频采集帧率 */
            frmival.index = 0;
            frmival.width = frmsize.discrete.width;
            frmival.height = frmsize.discrete.height;
            while (0 == ioctl(v4l2_fd, VIDIOC_ENUM_FRAMEINTERVALS, &frmival)) {

                printf("<%dfps>", frmival.discrete.denominator /
                        frmival.discrete.numerator);
                frmival.index++;
            }
            printf("\n");
        }
        printf("\n");
    }
}

static int v4l2_set_format(void)
{
    struct v4l2_format fmt = {0};
    struct v4l2_streamparm streamparm = {0};

    /* 设置帧格式 */
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;//type类型
    fmt.fmt.pix.width = width;  //视频帧宽度
    fmt.fmt.pix.height = height;//视频帧高度
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;  //像素格式
    if (0 > ioctl(v4l2_fd, VIDIOC_S_FMT, &fmt)) {
        fprintf(stderr, "ioctl error: VIDIOC_S_FMT: %s\n", strerror(errno));
        return -1;
    }

    /*** 判断是否已经设置为我们要求的RGB565像素格式
    如果没有设置成功表示该设备不支持RGB565像素格式 */
    if (V4L2_PIX_FMT_RGB565 != fmt.fmt.pix.pixelformat) {
        fprintf(stderr, "Error: the device does not support RGB565 format!\n");
        return -1;
    }

    frm_width = fmt.fmt.pix.width;  //获取实际的帧宽度
    frm_height = fmt.fmt.pix.height;//获取实际的帧高度
    printf("Video Frame size <%d * %d>\n", frm_width, frm_height);

    /* 获取streamparm */
    streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(v4l2_fd, VIDIOC_G_PARM, &streamparm);

    /** 判断是否支持帧率设置 **/
    if (V4L2_CAP_TIMEPERFRAME & streamparm.parm.capture.capability) {
        streamparm.parm.capture.timeperframe.numerator = 1;
        streamparm.parm.capture.timeperframe.denominator = 30;//30fps
        if (0 > ioctl(v4l2_fd, VIDIOC_S_PARM, &streamparm)) {
            fprintf(stderr, "ioctl error: VIDIOC_S_PARM: %s\n", strerror(errno));
            return -1;
        }
    }

    return 0;
}

static int v4l2_init_buffer(void)
{
    struct v4l2_requestbuffers reqbuf = {0};
    struct v4l2_buffer buf = {0};

    /* 申请帧缓冲 */
    reqbuf.count = FRAMEBUFFER_COUNT;       //帧缓冲的数量
    reqbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    reqbuf.memory = V4L2_MEMORY_MMAP;
    if (0 > ioctl(v4l2_fd, VIDIOC_REQBUFS, &reqbuf)) {
        fprintf(stderr, "ioctl error: VIDIOC_REQBUFS: %s\n", strerror(errno));
        return -1;
    }

    /* 建立内存映射 */
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    for (buf.index = 0; buf.index < FRAMEBUFFER_COUNT; buf.index++) {

        ioctl(v4l2_fd, VIDIOC_QUERYBUF, &buf);
        buf_infos[buf.index].length = buf.length;
        buf_infos[buf.index].start = mmap(NULL, buf.length,
                PROT_READ | PROT_WRITE, MAP_SHARED,
                v4l2_fd, buf.m.offset);
        if (MAP_FAILED == buf_infos[buf.index].start) {
            perror("mmap error");
            return -1;
        }
    }

    /* 入队 */
    for (buf.index = 0; buf.index < FRAMEBUFFER_COUNT; buf.index++) {

        if (0 > ioctl(v4l2_fd, VIDIOC_QBUF, &buf)) {
            fprintf(stderr, "ioctl error: VIDIOC_QBUF: %s\n", strerror(errno));
            return -1;
        }
    }

    return 0;
}

static int v4l2_stream_on(void)
{
    /* 打开摄像头、摄像头开始采集数据 */
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (0 > ioctl(v4l2_fd, VIDIOC_STREAMON, &type)) {
        fprintf(stderr, "ioctl error: VIDIOC_STREAMON: %s\n", strerror(errno));
        return -1;
    }

    return 0;
}

static void v4l2_read_data(void)
{
    struct v4l2_buffer buf = {0};
    unsigned short *base;
    unsigned short *start;
    int min_w, min_h;
    int j;

    if (width > frm_width)
        min_w = frm_width;
    else
        min_w = width;
    if (height > frm_height)
        min_h = frm_height;
    else
        min_h = height;

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    for ( ; ; ) {

        for(buf.index = 0; buf.index < FRAMEBUFFER_COUNT; buf.index++) {

            ioctl(v4l2_fd, VIDIOC_DQBUF, &buf);     //出队
            for (j = 0, base=screen_base, start=buf_infos[buf.index].start;
                        j < min_h; j++) {

                memcpy(base, start, min_w * 2); //RGB565 一个像素占2个字节
                base += width;  //LCD显示指向下一行
                start += frm_width;//指向下一行数据
            }

            // 数据处理完之后、再入队、往复
            ioctl(v4l2_fd, VIDIOC_QBUF, &buf);
        }
    }
}






void *Camera_V4L2_thread(void *number)
{

    printf("Camera_V4L2_thread: start \n");

    while (1)
    {
        /* 初始化LCD */
        if (fb_dev_init())
            exit(EXIT_FAILURE);

        /* 初始化摄像头 */
        if ( v4l2_dev_init( Camer_Dev ) )
            exit(EXIT_FAILURE);

        /* 枚举所有格式并打印摄像头支持的分辨率及帧率 */
        v4l2_enum_formats();
        v4l2_print_formats();

        /* 设置格式 */
        if (v4l2_set_format())
            exit(EXIT_FAILURE);

        /* 初始化帧缓冲：申请、内存映射、入队 */
        if (v4l2_init_buffer())
            exit(EXIT_FAILURE);

        /* 开启视频采集 */
        if (v4l2_stream_on())
            exit(EXIT_FAILURE);

        /* 读取数据：出队 */
        v4l2_read_data();       //在函数内循环采集数据、将其显示到LCD屏

        exit(EXIT_SUCCESS);
        
        
        printf("Camera_V4L2_thread: ID %ld\n", number);



        // pthread_exit(NULL);
        sleep(1);
    
    }
    
}


void Thread_Init(void)
{
    pthread_t threads[NUM_THREADS];
    int ret;
    int i = 0;


    // 创建线程，传入线程ID参数，线程函数thread_func作为线程入口点
    ret = pthread_create(&threads[0], NULL, Uart_process_thread, (void *)00);
    if ( ret ) perror("pthread_create(Uart_process_thread) failed");

    ret = pthread_create(&threads[1], NULL, CAN_process_thread, (void *)01);
    if ( ret ) perror("pthread_create(CAN_process_thread) failed");

    ret = pthread_create(&threads[2], NULL, Camera_V4L2_thread, (void *)02);
    if ( ret ) perror("pthread_create(Camera_V4L2_thread) failed");
    
    

    for (i = 0; i < NUM_THREADS; i++)
    {
        pthread_join(threads[i], NULL);// 等待线程结束
    }
    

    printf("All threads joined, exiting...\n");

    pthread_exit(NULL);

}









int main(int argc, char *argv[])
{


    Thread_Init();


}