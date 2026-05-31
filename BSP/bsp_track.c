#include "bsp_track.h"
#include "board.h"
#include "stdio.h"

/*
 * 说明：
 * 1. DL_GPIO_readPins() 返回的是引脚掩码，不是 0/1，所以必须先归一化。
 * 2. 当前权重约定：S1~S4 为左侧，S5~S8 为右侧；左侧误差为负，右侧误差为正。
 * 3. 如果你发现改完后不是“左转增强”，而是“右转增强”，把 LEFT_TURN_GAIN 和 RIGHT_TURN_GAIN 的数值互换。
 */

#define TRACK_ACTIVE_HIGH        1       // 若传感器检测到黑线时输出高电平，填 1；若输出低电平，改成 0

#define BASE_SPEED_NORMAL        200
#define BASE_SPEED_CORNER        85
#define MIN_BASE_SPEED           70

#define LEFT_ERROR_GAIN          1.22f
#define RIGHT_ERROR_GAIN         1.16f

#define LEFT_TURN_GAIN           1.18f
#define RIGHT_TURN_GAIN          1.12f

#define PWM_FILTER_OLD           0.60f
#define PWM_FILTER_NEW           0.40f

#define STRAIGHT_DEADBAND        0.90f
#define PID_DEADBAND             22

#define NORMAL_ERROR_SPEED_DROP  12
#define NORMAL_TURN_PWM_LIMIT    150
#define CORNER_TURN_PWM_LIMIT    185
#define TURN_PWM_STEP_LIMIT      35
#define RIGHT_CORNER_TURN_PWM_LIMIT 175
#define RIGHT_TURN_PWM_STEP_LIMIT   26
#define CORNER_REVERSE_PWM_LIMIT 90

float Kp = 8.8f, Ki = 0.0f, Kd = 4.8f;

float P = 0, I = 0, D = 0, PID_value = 0;
float new_error = 0, previous_error = 0;
int line_detected = 0;
int corner_mode = 0;

static float error_momentum = 0;

/*
 * 左直角弯保持计数：
 * 作用：防止小车刚开始左转，中间传感器又扫到线后，
 * 程序马上退出左转，导致左直角弯卡住。
 */
static int left_corner_hold = 0;
static int right_corner_hold = 0;

static int track_to_01(uint32_t raw)
{
#if TRACK_ACTIVE_HIGH
    return raw ? 1 : 0;
#else
    return raw ? 0 : 1;
#endif
}

void track_init(void)
{
    // 跑车时不要在循迹循环里频繁 printf，这里也可以注释掉
    // printf("Track optimized version OK\r\n");
}

int track_scan(void)
{
    int sensor_values[8] = {
        track_to_01(S1), track_to_01(S2), track_to_01(S3), track_to_01(S4),
        track_to_01(S5), track_to_01(S6), track_to_01(S7), track_to_01(S8)
    };

    /*
     * 权重说明：
     * S1/S2 在左侧，给负误差；
     * S7/S8 在右侧，给正误差。
     */
    float error_weight[8] = {-4.2f, -3.0f, -1.8f, -0.7f,
                              0.7f,  1.8f,  3.0f,  4.2f};

    int detected_count = 0;
    float total_error = 0;

    for (int i = 0; i < 8; i++)
    {
        if (sensor_values[i])
        {
            total_error += error_weight[i];
            detected_count++;
        }
    }

    /*
     * 多个传感器同时检测到黑线：
     * 一般出现在十字、宽线、急弯入口等情况。
     */
    if (detected_count >= 6)
    {
        corner_mode = 1;

        int left_active  = sensor_values[0] + sensor_values[1] + sensor_values[2];
        int right_active = sensor_values[5] + sensor_values[6] + sensor_values[7];

        if (left_active > right_active)
        {
            /*
             * 大左弯增强，并且给左转保持。
             */
            left_corner_hold = 12;
            right_corner_hold = 0;
            new_error = -8.8f * LEFT_ERROR_GAIN;
        }
        else if (right_active > left_active)
        {
            /*
             * 右转目前还可以，不要增强太多。
             */
            right_corner_hold = 6;
            new_error = 6.2f * RIGHT_ERROR_GAIN;
        }
        else
        {
            /*
             * 左右都差不多时，按照历史趋势继续走。
             */
            if (error_momentum >= 0)
            {
                right_corner_hold = 5;
                new_error = 6.3f * RIGHT_ERROR_GAIN;
            }
            else
            {
                left_corner_hold = 8;
                right_corner_hold = 0;
                new_error = -7.5f * LEFT_ERROR_GAIN;
            }
        }

        error_momentum = error_momentum * 0.75f + new_error * 0.25f;
        line_detected = 1;
    }
    else if (detected_count > 0)
    {
        /*
         * 普通循迹：先计算平均误差。
         */
        new_error = total_error / detected_count;

        /*
         * 左直角弯判断：
         * S1 或 S2 检测到黑线，并且右侧 S6/S7/S8 没有检测到，
         * 就认为小车进入左侧大弯/直角弯。
         */
        if ((sensor_values[0] || sensor_values[1]) &&
            !(sensor_values[5] || sensor_values[6] || sensor_values[7]))
        {
            corner_mode = 1;

            /*
             * 左转保持 10 个控制周期。
             * 如果左转还是转不过去，可以改成 12。
             * 如果左转太猛，可以改成 6 或 8。
            */
            left_corner_hold = 10;
            right_corner_hold = 0;

            /*
             * 左直角弯单独加强。
             * 只在大左弯时生效，不影响直线。
             */
            new_error = -9.0f * LEFT_ERROR_GAIN;
        }
        else if (left_corner_hold > 0)
        {
            /*
             * 左转保持：
             * 防止刚左转一点，中间传感器扫到线后马上退出左转。
             */
            corner_mode = 1;
            left_corner_hold--;

            new_error = -8.2f * LEFT_ERROR_GAIN;
        }
        else if ((sensor_values[5] || sensor_values[6] || sensor_values[7]) &&
                 !(sensor_values[0] || sensor_values[1] || sensor_values[2]))
        {
            /*
             * 右转判断：
             * 右转目前还可以，只略微增强一点。
            */
            corner_mode = 1;
            right_corner_hold = 6;

            if (sensor_values[6] || sensor_values[7])
            {
                new_error = 5.9f * RIGHT_ERROR_GAIN;
            }
            else
            {
                new_error = 4.7f * RIGHT_ERROR_GAIN;
            }
        }
        else if (right_corner_hold > 0)
        {
            corner_mode = 1;
            right_corner_hold--;

            new_error = 5.2f * RIGHT_ERROR_GAIN;
        }
        else
        {
            /*
             * 普通直线/小弯：
             * 小误差直接归零，减少直线左右抖。
             */
            corner_mode = 0;
            right_corner_hold = 0;

            if (ABS(new_error) < STRAIGHT_DEADBAND)
            {
                new_error = 0;
            }
            else
            {
                if (new_error < 0)
                {
                    new_error *= LEFT_ERROR_GAIN;
                }
                else
                {
                    new_error *= RIGHT_ERROR_GAIN;
                }
            }
        }

        error_momentum = error_momentum * 0.75f + new_error * 0.25f;
        line_detected = 1;
    }
    else
    {
        /*
         * 丢线时：
         * 保持上一次误差，交给 track_control() 里的找线逻辑处理。
         */
        corner_mode = 0;
        new_error = previous_error;
        line_detected = 0;
    }

    return 0;
}

int track_pid(void)
{
    P = new_error;

    /*
     * 直线小误差区域：
     * 不计算 D 项，防止传感器轻微跳变被 D 项放大，
     * 从而造成直线左右抖。
     */
    if (ABS(new_error) < STRAIGHT_DEADBAND)
    {
        I = 0;
        D = 0;
    }
    else
    {
        I += new_error;
        D = new_error - previous_error;
    }

    if (I > 40) I = 40;
    if (I < -40) I = -40;

    PID_value = (Kp * P) + (Ki * I) + (Kd * D);

    previous_error = new_error;

    /*
     * PWM 小输出死区：
     * 很小的修正直接不输出，减少直线左右摆。
     */
    if (ABS(PID_value) < PID_DEADBAND)
    {
        PID_value = 0;
    }

    return (int)PID_value;
}

void track_motorsWrite(int speedL, int speedR)
{
    Motor_PwmSet_RL(speedL);
    Motor_PwmSet_RR(speedR);
}

void track_Set_PWM(int pwm_value)
{
    static int last_pwm = 0;
    static int last_turn_pwm = 0;

    int filtered_pwm = (int)(last_pwm * PWM_FILTER_OLD + pwm_value * PWM_FILTER_NEW);
    last_pwm = filtered_pwm;

    float correction_factor;
    int speed_adjust;

    if (corner_mode)
    {
        speed_adjust = BASE_SPEED_CORNER;

        /*
         * filtered_pwm < 0 对应左转。
         * 左转单独增强，右转保持适中。
         */
        if (filtered_pwm < 0)
        {
            correction_factor = 1.85f;
        }
        else
        {
            correction_factor = 1.65f;
        }
    }
    else
    {
        /*
         * 普通直线/小弯：
         * 降低修正力度，减少左右抖。
         */
        speed_adjust = BASE_SPEED_NORMAL - (int)(ABS(new_error) * NORMAL_ERROR_SPEED_DROP);

        if (speed_adjust < MIN_BASE_SPEED)
        {
            speed_adjust = MIN_BASE_SPEED;
        }

        correction_factor = 0.90f + (ABS(new_error) * 0.05f);
    }

    /*
     * 左右转增益：
     * 左转略强，右转略弱，保证左直角弯更容易过去。
     */
    if (filtered_pwm < 0)
    {
        correction_factor *= LEFT_TURN_GAIN;
    }
    else
    {
        correction_factor *= RIGHT_TURN_GAIN;
    }

    int turn_pwm = (int)(correction_factor * filtered_pwm);
    int turn_pwm_limit = corner_mode ? CORNER_TURN_PWM_LIMIT : NORMAL_TURN_PWM_LIMIT;
    int turn_pwm_step_limit = TURN_PWM_STEP_LIMIT;

    if (turn_pwm > 0)
    {
        turn_pwm_limit = corner_mode ? RIGHT_CORNER_TURN_PWM_LIMIT : NORMAL_TURN_PWM_LIMIT;
        turn_pwm_step_limit = RIGHT_TURN_PWM_STEP_LIMIT;
    }

    if (turn_pwm > turn_pwm_limit)
    {
        turn_pwm = turn_pwm_limit;
    }
    else if (turn_pwm < -turn_pwm_limit)
    {
        turn_pwm = -turn_pwm_limit;
    }

    if (turn_pwm > last_turn_pwm + turn_pwm_step_limit)
    {
        turn_pwm = last_turn_pwm + turn_pwm_step_limit;
    }
    else if (turn_pwm < last_turn_pwm - turn_pwm_step_limit)
    {
        turn_pwm = last_turn_pwm - turn_pwm_step_limit;
    }

    last_turn_pwm = turn_pwm;

    int left_motor_speed  = speed_adjust - turn_pwm;
    int right_motor_speed = speed_adjust + turn_pwm;

    if (corner_mode)
    {
        if (left_motor_speed < -CORNER_REVERSE_PWM_LIMIT)
        {
            left_motor_speed = -CORNER_REVERSE_PWM_LIMIT;
        }

        if (right_motor_speed < -CORNER_REVERSE_PWM_LIMIT)
        {
            right_motor_speed = -CORNER_REVERSE_PWM_LIMIT;
        }
    }

    if (left_motor_speed > 999)
    {
        left_motor_speed = 999;
    }
    else if (left_motor_speed < -999)
    {
        left_motor_speed = -999;
    }

    if (right_motor_speed > 999)
    {
        right_motor_speed = 999;
    }
    else if (right_motor_speed < -999)
    {
        right_motor_speed = -999;
    }

    track_motorsWrite(left_motor_speed, right_motor_speed);
}

int track_control(void)
{
    track_scan();

    if (line_detected)
    {
        int pid_out = track_pid();
        track_Set_PWM(pid_out);
    }
    else
    {
        /*
         * 丢线找线：
         * 如果上一次误差偏左，继续左转找线；
         * 否则右转找线。
         */
        if (previous_error < 0 || error_momentum < 0)
        {
            // 上一次是左侧误差，继续左转找线
            track_motorsWrite(-135, 165);
        }
        else
        {
            // 右侧找线保持弱一点
            track_motorsWrite(125, -105);
        }
    }

    return line_detected;
}

int speed_pid_compute(int target_speed, int measure_speed, int *last_error)
{
    float ki = 0.9f, kp = 4.15f;
    static int output = 0;

    int error = target_speed - measure_speed;

    output += (int)(ki * error + kp * (error - *last_error));
    *last_error = error;

    if (output > 999)
    {
        output = 999;
    }
    else if (output < -999)
    {
        output = -999;
    }

    return output;
}
