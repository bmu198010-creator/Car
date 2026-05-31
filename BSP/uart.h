#ifndef _UART_H
#define _UART_H
#include "board.h"
#include <stdio.h>      // 添加标准IO头文件
#include <stdarg.h>     // 解决 va_list 问题
#include <string.h>     // 解决 strlen/memmove/memcpy 问题


#define RE_0_BUFF_LEN_MAX	1024

extern struct ringbuffer g_rb_Uart_MS901M_RevBuf;

void uart_init(void);

void MS901M_uart1_send_data(unsigned char* str, uint16_t length);

void Print_Data_To_Uart0(void);
void Print_Data_To_MS901M_Uart(void);

//宏替换 printf
//#define printf(...) uart_printf(__VA_ARGS__)
extern int uart_printf(const char *format, ...);


#endif
