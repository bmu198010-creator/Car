#ifndef __BSP_PID_H__
#define __BSP_PID_H__

#include "ti_msp_dl_config.h"

// PID 状态枚举
typedef enum {
    PID_DISABLE,
    PID_ENABLE,
} pid_status_t;

typedef enum {
    MOTOR_STOP,		// 0默认停止
    MOTOR_SPEED,	// 1速度环
    MOTOR_DIR,		// 2位置环
    MOTOR_TRACK,	// 3循迹
    MOTOR_DEG,      // 4角度
} pid_mode_t;

// PID 参数结构体
typedef struct pid_params_t {  // 结构体完整定义提前
    float kp;
    float ki;
    float kd;

    int target_speed;		// 目标速度, 电机速度控制，0为停止 0-10  (速度单位： 多少个脉冲/10ms)
    int measure_speed;		// 实测速度, 编码器计数值"encoder_cnt"

    int error;
    int last_error;
    int output;

    int output_max;
    int output_min;

    pid_status_t status;
	  pid_mode_t	 pid_mode;

    // 函数指针
    void (*init)(struct pid_params_t *pid, float kp, float ki, float kd, int output_max, int output_min);
    int (*compute)(struct pid_params_t *pid);
    void (*set_status)(struct pid_params_t *pid, pid_status_t status);
    void (*set_mode)(struct pid_params_t *pid, pid_mode_t mode);
} pid_params_t;  // 结构体完整定义结束

void pid_init(pid_params_t *pid, float kp, float ki, float kd, int output_max, int output_min);
int pid_compute(pid_params_t *pid);
void pid_set_status(pid_params_t *pid, pid_status_t status);
void pid_set_mode(pid_params_t *pid, pid_mode_t mode);

#endif
