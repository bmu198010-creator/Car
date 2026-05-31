#ifndef __BSP_MOTOR_H__
#define __BSP_MOTOR_H__

#include "board.h"

#define ABS(a)      (a > 0 ? a : (-a))

/*
 * 记录当前实际输出给左右轮的 PWM。
 * OLED 只读取这两个变量，不参与控制。
 */
extern volatile int g_pwm_RL_now;
extern volatile int g_pwm_RR_now;

void motor_init(void);

void Motor_PwmSet_FL(int Pwm_FrontLeft);
void Motor_PwmSet_FR(int Pwm_FrontRight);
void Motor_PwmSet_RL(int Pwm_RearLeft);
void Motor_PwmSet_RR(int Pwm_RearRight);

void Motor_Stop(void);
void Motor_Kinematic_Analysis(float velocity, float turn);
void motor_pid_init(void);

#endif