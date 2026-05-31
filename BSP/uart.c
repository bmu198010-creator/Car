#include "uart.h"


static uint8_t  g_u8UartRevBuf[RE_0_BUFF_LEN_MAX] = {0},
                g_u8UartSendRevBuf[RE_0_BUFF_LEN_MAX*8],

                g_u8Uart_MS901M_RevBuf[RE_0_BUFF_LEN_MAX*4] = {0},
                g_u8Uart_MS901M_SendRevBuf[RE_0_BUFF_LEN_MAX/2];

struct ringbuffer g_rb_UartRevBuf={
    .read_index		= 0,
    .read_mirror	= 0,
    .write_index	= 0,
    .write_mirror	= 0,
    .buffer_ptr		= g_u8UartRevBuf,
    .buffer_size	= sizeof(g_u8UartRevBuf),
};
struct ringbuffer g_rb_UartSendBuf={
    .read_index		= 0,
    .read_mirror	= 0,
    .write_index	= 0,
    .write_mirror	= 0,
    .buffer_ptr		= g_u8UartSendRevBuf,
    .buffer_size	= sizeof(g_u8UartSendRevBuf),
};

struct ringbuffer g_rb_Uart_MS901M_RevBuf={
    .read_index		= 0,
    .read_mirror	= 0,
    .write_index	= 0,
    .write_mirror	= 0,
    .buffer_ptr		= g_u8Uart_MS901M_RevBuf,
    .buffer_size	= sizeof(g_u8Uart_MS901M_RevBuf),
};
struct ringbuffer g_rb_Uart_MS901M_SendBuf={
    .read_index		= 0,
    .read_mirror	= 0,
    .write_index	= 0,
    .write_mirror	= 0,
    .buffer_ptr		= g_u8Uart_MS901M_SendRevBuf,
    .buffer_size	= sizeof(g_u8Uart_MS901M_SendRevBuf),
};

volatile uint64_t SystemTickCNT = 0;
uint64_t HAL_GetTick(void)
{
    return SystemTickCNT;
}

// 自定义输出函数带时间戳
int uart_printf(const char *format, ...)
{
    va_list args;
    char buffer[1024]; // 根据需要调整缓冲区大小
    int len;

    // 获取当前时间戳（以毫秒为单位）
    uint32_t timestamp = (uint32_t)HAL_GetTick();

    // 计算秒数和毫秒数
    uint32_t seconds = timestamp / 1000;
    uint32_t milliseconds = timestamp % 1000;

    // 确定秒数的最大宽度（根据 uint32_t 的最大值）
    // uint32_t 最大值为 4294967295 ms，即 4294967 秒
    // 因此秒数最多占 7 位
    char timestamp_str[16];
    snprintf(timestamp_str, sizeof(timestamp_str), "[%07u.%03u] ", seconds, milliseconds);

    // 将时间戳和用户提供的格式化字符串合并到缓冲区
    va_start(args, format);
    len = vsnprintf(buffer, sizeof(buffer) - strlen(timestamp_str), format, args);
    va_end(args);

    // 确保有足够的空间容纳时间戳
    if (len > 0 && len < (int)(sizeof(buffer) - strlen(timestamp_str)))
    {
        // 拼接时间戳和用户提供的字符串
        memmove(buffer + strlen(timestamp_str), buffer, len + 1); // 移动原始内容
        memcpy(buffer, timestamp_str, strlen(timestamp_str));      // 添加时间戳

        // 发送拼接后的字符串到环形缓冲区
        ringbuffer_put(&g_rb_UartSendBuf, (uint8_t *)buffer, strlen(buffer));
    }
    return 1;
}

void uart_init(void)
{
    //清除串口中断标志
    NVIC_ClearPendingIRQ(UART_MS901M_INST_INT_IRQN);
    //使能串口中断
    NVIC_EnableIRQ(UART_MS901M_INST_INT_IRQN);

    //清除串口中断标志
    //NVIC_ClearPendingIRQ(UART_DEBUG_INST_INT_IRQN);
    //使能串口中断
    //NVIC_EnableIRQ(UART_DEBUG_INST_INT_IRQN);

    //printf("UART Init\r\n");
}


//void uart0_send_char(char ch)
//{
//    while (DL_UART_isBusy(UART_DEBUG_INST) == true)
//    {
//        ;
//    }
//    DL_UART_Main_transmitData(UART_DEBUG_INST, ch);
//}

//void uart0_send_data(char* str, uint16_t length)
//{
//    for (uint16_t index = 0; index < length; index++)
//    {
//        uart0_send_char(str[index]);
//    }
//}

//void Print_Data_To_Uart0(void)
//{
//    if (ringbuffer_data_len(&g_rb_UartSendBuf))
//    {
//        uint8_t print_buff[RE_0_BUFF_LEN_MAX] = {0};
//        uint16_t l_u8SendDataLen = ringbuffer_get(&g_rb_UartSendBuf, &print_buff[0], RE_0_BUFF_LEN_MAX);
//        uart0_send_data((char*)&print_buff[0], l_u8SendDataLen);
//    }
//}
//串口的中断服务函数
//void UART_DEBUG_INST_IRQHandler(void)
//{
//	static uint8_t rx_buf;
//    //如果产生了串口中断
//    switch( DL_UART_getPendingInterrupt(UART_DEBUG_INST) )
//    {
//        case DL_UART_IIDX_RX://如果是接收中断
//            rx_buf = DL_UART_Main_receiveData(UART_DEBUG_INST);
//            ringbuffer_put(&g_rb_UartRevBuf,&rx_buf, 1);
//        break;
//        default://其他的串口中断
//            break;
//    }
//}








void uart1_send_char(unsigned char ch)
{
    while( DL_UART_isBusy(UART_MS901M_INST) == true )
    {
        ;
    }
    DL_UART_Main_transmitData(UART_MS901M_INST, (char)ch);
}
void MS901M_uart1_send_data(unsigned char* str,uint16_t length)
{
    for(uint16_t index=0; index < length; index++)
    {
        uart1_send_char(str[index]);
    }
}
void Print_Data_To_MS901M_Uart(void)
{
    if(ringbuffer_data_len(&g_rb_Uart_MS901M_SendBuf))
    {
        uint8_t print_buff[RE_0_BUFF_LEN_MAX] = {0};
        uint16_t l_u8SendDataLen = ringbuffer_get(&g_rb_Uart_MS901M_SendBuf, &print_buff[0], RE_0_BUFF_LEN_MAX);
        MS901M_uart1_send_data(&print_buff[0],l_u8SendDataLen);
    }
}
//_MS901M_串口的中断服务函数
void UART_MS901M_INST_IRQHandler(void)
{
    static uint8_t rx_buf;
    //如果产生了串口中断
    switch( DL_UART_getPendingInterrupt(UART_MS901M_INST) )
    {
        case DL_UART_IIDX_RX://如果是接收中断
            rx_buf = DL_UART_Main_receiveData(UART_MS901M_INST);
            ringbuffer_put(&g_rb_Uart_MS901M_RevBuf,&rx_buf, 1);
        break;
        default://其他的串口中断
            break;
    }
}
