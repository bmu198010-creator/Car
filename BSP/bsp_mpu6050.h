#ifndef __BSP_MPU6050_H__
#define __BSP_MPU6050_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>

#define MPU6050_ADDR_7BIT      0x68u

/*
 * 如果小车右转时角度变负，把 1.0f 改成 -1.0f。
 */
#define MPU6050_YAW_SIGN       (1.0f)

uint8_t MPU6050_Init(void);
void MPU6050_Task(void);
void MPU6050_ResetYaw(void);
uint8_t MPU6050_IsReady(void);
float MPU6050_GetYawDeg(void);

#endif