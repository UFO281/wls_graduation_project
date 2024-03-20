#ifndef WUART_H
#define WUART_H


typedef struct uart_hardware_cfg 
{
    unsigned int baudrate;      /* ������ */
    unsigned char dbit;         /* ����λ */
    char parity;                /* ��żУ�� */
    unsigned char sbit;         /* ֹͣλ */
} uart_cfg_t;



static int uart_init(const char *device);
static int uart_cfg(const uart_cfg_t *cfg);
static void show_help(const char *app);
static void io_handler(int sig, siginfo_t *info, void *context);
static void async_io_init(void);
int uart_main(void);








#endif