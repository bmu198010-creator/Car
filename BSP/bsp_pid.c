#include "bsp_pid.h"
#include "board.h"
#include "stdio.h"

pid_params_t motor_FL_pid, motor_FR_pid, motor_RL_pid, motor_RR_pid;

// 初始化 PID 参数
static void _pid_init(pid_params_t *pid, float kp, float ki, float kd, int output_max, int output_min)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->output_max = output_max;
    pid->output_min = output_min;
    pid->target_speed =0;
    pid->measure_speed = 0;
    pid->error = 0;
    pid->last_error = 0;
    pid->output = 0;
    pid->status = PID_DISABLE;
    pid->pid_mode = MOTOR_STOP;
}

// 计算 PID 输出
static int _pid_compute(pid_params_t *pid)
{
    if (pid->status == PID_DISABLE)
    {
        return 0;
    }

    // 计算速度偏差
    pid->error = pid->target_speed - pid->measure_speed;

    // 计算增量式控制量
    pid->output += pid->ki * (pid->error - pid->last_error) + pid->kp * pid->error;  // 使用误差增量更新输出

    // 更新最后一次的偏差
    pid->last_error = pid->error;

    // 限制输出
    if (pid->output > pid->output_max)
    {
        pid->output = pid->output_max;
    }
    else if (pid->output < pid->output_min)
    {
        pid->output = pid->output_min;
    }

    return pid->output;
}

// 设置 PID 状态
static void _pid_set_status(pid_params_t *pid, pid_status_t status)
{
    pid->status = status;
}

// 设置 PID 模式
static void _pid_set_mode(pid_params_t *pid, pid_mode_t pid_mode)
{
    pid->pid_mode = pid_mode;
}

// 对外接口实现
void pid_init(pid_params_t *pid, float kp, float ki, float kd, int output_max, int output_min)
{
    pid->init = _pid_init;
    pid->compute = _pid_compute;
    pid->set_status = _pid_set_status;
    pid->set_mode = _pid_set_mode;

    pid->init(pid, kp, ki, kd, output_max, output_min);
}

int pid_compute(pid_params_t *pid)
{
    return pid->compute(pid);
}

void pid_set_status(pid_params_t *pid, pid_status_t status)
{
    pid->set_status(pid, status);
}

void pid_set_mode(pid_params_t *pid, pid_mode_t mode)
{
    pid->set_mode(pid, mode);
}
