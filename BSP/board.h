#ifndef	__BOARD_H__
#define __BOARD_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdio.h>      // 添加标准IO头文件
#include <stdarg.h>     // 解决 va_list 问题
//#include <string.h>     // 解决 strlen/memmove/memcpy 问题
#include "math.h"
#include "ringbuffer.h"
#include "bsp_motor.h"
#include "bsp_pid.h"
//#include "bsp_gyro.h"
#include "bsp_track.h"
#include "bsp_oled.h"
#include "bsp_mpu6050.h"
//#include "bsp_Steering.h"
//#include "bsp_ms901m.h"
//#include "iic.h"
#include "uart.h"
//#include "bsp_adc.h"
//#include "fine_line.h"

#define TWO_WHEELS		1	//三轮车
#define AKM_WHEELS		0	//阿克曼车
#define FOUR_WHEELS		0	//四轮车

extern uint8_t Velocity;
extern char Turn;
extern pid_params_t motor_FL_pid, motor_FR_pid, motor_RL_pid, motor_RR_pid;

void board_init(void);
void delay_us(unsigned long __us);
void delay_ms(unsigned long ms);

#endif
