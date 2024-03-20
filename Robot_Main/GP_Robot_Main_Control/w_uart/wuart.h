#ifndef WUART_H
#define WUART_H


typedef struct uart_hardware_cfg 
{
    unsigned int baudrate;      /* 波特率 */
    unsigned char dbit;         /* 数据位 */
    char parity;                /* 奇偶校验 */
    unsigned char sbit;         /* 停止位 */
} uart_cfg_t;



static int uart_init(const char *device);
static int uart_cfg(const uart_cfg_t *cfg);
static void show_help(const char *app);
static void io_handler(int sig, siginfo_t *info, void *context);
static void async_io_init(void);
int uart_main(void);








#endif