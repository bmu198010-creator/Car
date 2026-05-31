#ifndef __BSP_TRACK_H__
#define __BSP_TRACK_H__

#include "ti_msp_dl_config.h"

#define S1 (DL_GPIO_readPins(TRACK_S1_PORT, TRACK_S1_PIN))
#define S2 (DL_GPIO_readPins(TRACK_S2_PORT, TRACK_S2_PIN))
#define S3 (DL_GPIO_readPins(TRACK_S3_PORT, TRACK_S3_PIN))
#define S4 (DL_GPIO_readPins(TRACK_S4_PORT, TRACK_S4_PIN))
#define S5 (DL_GPIO_readPins(TRACK_S5_PORT, TRACK_S5_PIN))
#define S6 (DL_GPIO_readPins(TRACK_S6_PORT, TRACK_S6_PIN))
#define S7 (DL_GPIO_readPins(TRACK_S7_PORT, TRACK_S7_PIN))
#define S8 (DL_GPIO_readPins(TRACK_S8_PORT, TRACK_S8_PIN))

void track_init(void);
int track_control(void);
int track_pid(void);
void track_Set_PWM(int pwm_value);
void track_motorsWrite(int speedL, int speedR);
int speed_pid_compute(int target_speed,int measure_speed,int *last_error);


#endif


