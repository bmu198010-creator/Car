#include "bsp_motor.h"
#include "stdio.h"
#include "bsp_pid.h"
// #include "bsp_key.h"

volatile int32_t Encode_CNT_FL, Encode_CNT_FR, Encode_CNT_RL, Encode_CNT_RR;

/*
 * 当前左右轮 PWM 输出值。
 * 只用于 OLED 显示，不参与循迹控制。
 */
volatile int g_pwm_RL_now = 0;
volatile int g_pwm_RR_now = 0;

void motor_init(void)
{
    // 编码器引脚外部中断
    NVIC_EnableIRQ(ENCODER_GPIOA_INT_IRQN);
    NVIC_EnableIRQ(ENCODER_GPIOB_INT_IRQN);

    printf("Motor Init OK\r\n");
}

void motor_pid_init(void)
{
    const float Velcity_Kp = 4.15, Velcity_Ki = 0.9, Velcity_Kd = 0.2;

    pid_init(&motor_FL_pid, Velcity_Kp, Velcity_Ki, Velcity_Kd, 999, -999);
    pid_init(&motor_FR_pid, Velcity_Kp, Velcity_Ki, Velcity_Kd, 999, -999);
    pid_init(&motor_RL_pid, Velcity_Kp, Velcity_Ki, Velcity_Kd, 999, -999);
    pid_init(&motor_RR_pid, Velcity_Kp, Velcity_Ki, Velcity_Kd, 999, -999);

    pid_set_status(&motor_FL_pid, PID_ENABLE);
    pid_set_status(&motor_FR_pid, PID_ENABLE);
    pid_set_status(&motor_RL_pid, PID_ENABLE);
    pid_set_status(&motor_RR_pid, PID_ENABLE);

    pid_set_mode(&motor_FL_pid, MOTOR_SPEED);
    pid_set_mode(&motor_FR_pid, MOTOR_SPEED);
    pid_set_mode(&motor_RL_pid, MOTOR_SPEED);
    pid_set_mode(&motor_RR_pid, MOTOR_SPEED);
}

// 前左电机PWM控制函数
void Motor_PwmSet_FL(int Pwm_FrontLeft)
{
    static int last_dir_FL = 0;

    int current_dir_FL = (Pwm_FrontLeft > 0) ? 1 : -1;

    if (current_dir_FL != last_dir_FL)
    {
        if (Pwm_FrontLeft > 0)
        {
            DL_GPIO_setPins(MOTOR_PORT, MOTOR_AIN2_PIN);
            DL_GPIO_clearPins(MOTOR_PORT, MOTOR_AIN1_PIN);
        }
        else
        {
            DL_GPIO_setPins(MOTOR_PORT, MOTOR_AIN1_PIN);
            DL_GPIO_clearPins(MOTOR_PORT, MOTOR_AIN2_PIN);
        }

        last_dir_FL = current_dir_FL;
    }

    DL_TimerG_setCaptureCompareValue(PWM_T_INST, ABS(Pwm_FrontLeft), GPIO_PWM_T_C0_IDX);
}

// 前右电机PWM控制函数
void Motor_PwmSet_FR(int Pwm_FrontRight)
{
    static int last_dir_FR = 0;

    int current_dir_FR = (Pwm_FrontRight > 0) ? 1 : -1;

    if (current_dir_FR != last_dir_FR)
    {
        if (Pwm_FrontRight > 0)
        {
            DL_GPIO_setPins(MOTOR_PORT, MOTOR_BIN1_PIN);
            DL_GPIO_clearPins(MOTOR_PORT, MOTOR_BIN2_PIN);
        }
        else
        {
            DL_GPIO_setPins(MOTOR_PORT, MOTOR_BIN2_PIN);
            DL_GPIO_clearPins(MOTOR_PORT, MOTOR_BIN1_PIN);
        }

        last_dir_FR = current_dir_FR;
    }

    DL_TimerG_setCaptureCompareValue(PWM_T_INST, ABS(Pwm_FrontRight), GPIO_PWM_T_C1_IDX);
}

// 后左电机PWM控制函数
void Motor_PwmSet_RL(int Pwm_RearLeft)
{
    static int last_dir_RL = 0;

    /*
     * 记录左轮当前 PWM，供 OLED 显示。
     * 不改变原来的电机控制逻辑。
     */
    g_pwm_RL_now = Pwm_RearLeft;

    int current_dir_RL = (Pwm_RearLeft > 0) ? 1 : -1;

    if (current_dir_RL != last_dir_RL)
    {
        if (Pwm_RearLeft > 0)
        {
            DL_GPIO_setPins(MOTOR_PORT, MOTOR_AIN1_PIN);
            DL_GPIO_clearPins(MOTOR_PORT, MOTOR_AIN2_PIN);
        }
        else
        {
            DL_GPIO_setPins(MOTOR_PORT, MOTOR_AIN2_PIN);
            DL_GPIO_clearPins(MOTOR_PORT, MOTOR_AIN1_PIN);
        }

        last_dir_RL = current_dir_RL;
    }

    DL_TimerG_setCaptureCompareValue(PWM_T_INST, ABS(Pwm_RearLeft), GPIO_PWM_T_C0_IDX);
}

// 后右电机PWM控制函数
void Motor_PwmSet_RR(int Pwm_RearRight)
{
    static int last_dir_RR = 0;

    /*
     * 记录右轮当前 PWM，供 OLED 显示。
     * 不改变原来的电机控制逻辑。
     */
    g_pwm_RR_now = Pwm_RearRight;

    int current_dir_RR = (Pwm_RearRight > 0) ? 1 : -1;

    if (current_dir_RR != last_dir_RR)
    {
        if (Pwm_RearRight > 0)
        {
            DL_GPIO_setPins(MOTOR_PORT, MOTOR_BIN2_PIN);
            DL_GPIO_clearPins(MOTOR_PORT, MOTOR_BIN1_PIN);
        }
        else
        {
            DL_GPIO_setPins(MOTOR_PORT, MOTOR_BIN1_PIN);
            DL_GPIO_clearPins(MOTOR_PORT, MOTOR_BIN2_PIN);
        }

        last_dir_RR = current_dir_RR;
    }

    DL_TimerG_setCaptureCompareValue(PWM_T_INST, ABS(Pwm_RearRight), GPIO_PWM_T_C1_IDX);
}

void Motor_Stop(void)
{
    // 前左电机停止
    DL_GPIO_setPins(MOTOR_PORT, MOTOR_AIN1_PIN);
    DL_GPIO_setPins(MOTOR_PORT, MOTOR_AIN2_PIN);
    DL_TimerG_setCaptureCompareValue(PWM_T_INST, 0, GPIO_PWM_T_C0_IDX);

    // 前右电机停止
    DL_GPIO_setPins(MOTOR_PORT, MOTOR_BIN1_PIN);
    DL_GPIO_setPins(MOTOR_PORT, MOTOR_BIN2_PIN);
    DL_TimerG_setCaptureCompareValue(PWM_T_INST, 0, GPIO_PWM_T_C1_IDX);

    // 后左电机停止
    DL_GPIO_setPins(MOTOR_PORT, MOTOR_AIN1_PIN);
    DL_GPIO_setPins(MOTOR_PORT, MOTOR_AIN2_PIN);
    DL_TimerG_setCaptureCompareValue(PWM_T_INST, 0, GPIO_PWM_T_C0_IDX);

    // 后右电机停止
    DL_GPIO_setPins(MOTOR_PORT, MOTOR_BIN1_PIN);
    DL_GPIO_setPins(MOTOR_PORT, MOTOR_BIN2_PIN);
    DL_TimerG_setCaptureCompareValue(PWM_T_INST, 0, GPIO_PWM_T_C1_IDX);
}

/*******************************************************
函数功能：外部中断模拟编码器信号
入口函数：无
返回值：无
***********************************************************/
void GROUP1_IRQHandler(void)
{
    // 读取触发的 GPIOA 和 GPIOB 中断状态
    uint32_t PortA_interrup = DL_GPIO_getRawInterruptStatus(GPIOA, ENCODER_OB1_PIN);
    uint32_t PortB_interrup = DL_GPIO_getRawInterruptStatus(GPIOB, ENCODER_OA2_PIN);

    // 处理 Rear 左轮
    if (PortA_interrup & ENCODER_OB1_PIN)
    {
        if (DL_GPIO_readPins(ENCODER_OA1_PORT, ENCODER_OA1_PIN) & ENCODER_OA1_PIN)
        {
            Encode_CNT_RL++;
        }
        else
        {
            Encode_CNT_RL--;
        }
    }

    // 处理 Rear 右轮
    if (PortB_interrup & ENCODER_OA2_PIN)
    {
        if (DL_GPIO_readPins(ENCODER_OB2_PORT, ENCODER_OB2_PIN) & ENCODER_OB2_PIN)
        {
            Encode_CNT_RR--;
        }
        else
        {
            Encode_CNT_RR++;
        }
    }

    // 清除已触发的中断
    DL_GPIO_clearInterruptStatus(GPIOA, PortA_interrup & ENCODER_OB1_PIN);
    DL_GPIO_clearInterruptStatus(GPIOB, PortB_interrup & ENCODER_OA2_PIN);
}

void Motor_Kinematic_Analysis(float velocity, float turn)
{
    #if AKM_WHEELS
    {
        float m_angle;
        m_angle = 2 * Pi * turn / 360;

        motor_RL_pid.target_speed = velocity * (1 - T * tan(m_angle) * K_1 / 2 / L);
        motor_RR_pid.target_speed = velocity * (1 + T * tan(m_angle) * K_1 / 2 / L);

        // SetServoAngle(turn);
    }

    #elif TWO_WHEELS
    {
        motor_RL_pid.target_speed = velocity - turn;
        motor_RR_pid.target_speed = velocity + turn;
    }

    #elif FOUR_WHEELS
    {
        motor_RR_pid.target_speed = velocity + turn;
        motor_RL_pid.target_speed = velocity - turn;
        motor_FR_pid.target_speed = velocity + turn;
        motor_FL_pid.target_speed = velocity - turn;
    }
    #endif
}