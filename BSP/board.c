#include "board.h"
#include <stdio.h>      // 添加标准IO头文件
#include <stdarg.h>     // 解决 va_list 问题
#include <string.h>     // 解决 strlen/memmove/memcpy 问题

void board_init(void)
{
    SYSCFG_DL_init();

    // 定时器中断
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    printf("SYSCFG_DL_init\r\n");

    // uart_init();     // 带有陀螺仪的串口需要最先初始化，否则可能会导致后面收数据和解析数据出问题
    motor_pid_init();
    // adc_init();

    /*
     * OLED 只初始化一次。
     * 不要放到 while(1) 里面反复初始化。
     */
    OLED_Init();
    MPU6050_Init();

    motor_init();       // 开启电机编码器引脚外部中断
    track_init();       // 开启循迹模块初始化
}

// 搭配滴答定时器实现的精确us延时
void delay_us(unsigned long __us)
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 38;

    // 计算需要的时钟数 = 延迟微秒数 * 每微秒的时钟数
    ticks = __us * (80000000 / 1000000);

    // 获取当前的SysTick值
    told = SysTick->VAL;

    while (1)
    {
        // 重复刷新获取当前的SysTick值
        tnow = SysTick->VAL;

        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += SysTick->LOAD - tnow + told;
            }

            told = tnow;

            // 如果达到了需要的时钟数，就退出循环
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }
}

// 搭配滴答定时器实现的精确ms延时
void delay_ms(unsigned long ms)
{
    delay_us(ms * 1000);
}