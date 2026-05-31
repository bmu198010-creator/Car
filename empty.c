#include "BSP/board.h"
#include <stdio.h>


extern int corner_mode;
extern int line_detected;
extern float new_error;


uint8_t Velocity = 0;
char Turn = 0;
static float s_f32Yaw_Init;
volatile float g_f32Current_YawDeg = 0.0f;
uint8_t motor_status=0;//定义小车运行状态，0=匀速，1=循迹
int RL_speed=0,RR_speed=0,RL_error=0,RR_error=0,output=0;//定义测量速度、测量误差、输出

extern volatile int g_pwm_RL_now;
extern volatile int g_pwm_RR_now;
extern volatile uint64_t SystemTickCNT;

#define BASIC_TARGET_LAPS        1
#define BASIC_MIN_LAPS           1
#define BASIC_MAX_LAPS           5
#define BASIC_CORNERS_PER_LAP    4
#define BASIC_START_IGNORE_MS    800
#define BASIC_CORNER_CONFIRM_MS  140
#define BASIC_CORNER_RELEASE_MS  220
#define BASIC_CORNER_MIN_GAP_MS  850
#define BASIC_CORNER_ERROR_MIN   5.5f
#define BASIC_STRAIGHT_ERROR_MAX 1.4f
#define BASIC_MAX_RUN_MS         20000

typedef enum {
    BASIC_TRACK_IDLE = 0,
    BASIC_TRACK_RUNNING,
    BASIC_TRACK_DONE,
    BASIC_TRACK_TIMEOUT
} basic_track_state_t;

static basic_track_state_t s_basic_state = BASIC_TRACK_IDLE;
static uint8_t s_basic_target_laps = BASIC_TARGET_LAPS;
static uint8_t s_basic_corner_count = 0;
static uint8_t s_basic_corner_armed = 0;
static uint64_t s_basic_start_tick = 0;
static uint64_t s_basic_last_corner_tick = 0;
static uint64_t s_basic_corner_candidate_tick = 0;
static uint64_t s_basic_straight_tick = 0;

static uint8_t basic_track_limit_laps(uint8_t laps)
{
    if (laps < BASIC_MIN_LAPS)
    {
        return BASIC_MIN_LAPS;
    }

    if (laps > BASIC_MAX_LAPS)
    {
        return BASIC_MAX_LAPS;
    }

    return laps;
}

static float basic_abs_float(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static void basic_track_start(uint8_t laps)
{
    s_basic_target_laps = basic_track_limit_laps(laps);
    s_basic_corner_count = 0;
    s_basic_start_tick = SystemTickCNT;
    s_basic_last_corner_tick = SystemTickCNT;
    s_basic_corner_armed = 0;
    s_basic_corner_candidate_tick = 0;
    s_basic_straight_tick = 0;
    s_basic_state = BASIC_TRACK_RUNNING;

    motor_status = 1;
}

static void basic_track_stop(basic_track_state_t stop_state)
{
    s_basic_state = stop_state;
    motor_status = 2;
    Velocity = 0;
    Turn = 0;
    Motor_Stop();
}

static uint8_t basic_track_task(void)
{
    uint64_t now_tick;
    uint8_t target_corners;
    float abs_error;
    uint8_t corner_candidate;
    uint8_t straight_stable;

    if (s_basic_state == BASIC_TRACK_IDLE)
    {
        basic_track_start(BASIC_TARGET_LAPS);
    }

    if (s_basic_state != BASIC_TRACK_RUNNING)
    {
        Motor_Stop();
        return 0;
    }

    now_tick = SystemTickCNT;

    if ((now_tick - s_basic_start_tick) >= BASIC_MAX_RUN_MS)
    {
        basic_track_stop(BASIC_TRACK_TIMEOUT);
        return 0;
    }

    track_control();
    motor_status = 1;
    abs_error = basic_abs_float(new_error);

    if ((now_tick - s_basic_start_tick) < BASIC_START_IGNORE_MS)
    {
        s_basic_corner_armed = 0;
        s_basic_corner_candidate_tick = 0;
        s_basic_straight_tick = 0;
        return 1;
    }

    corner_candidate = (corner_mode &&
                        line_detected &&
                        (abs_error >= BASIC_CORNER_ERROR_MIN)) ? 1 : 0;

    straight_stable = ((!corner_mode) &&
                       line_detected &&
                       (abs_error <= BASIC_STRAIGHT_ERROR_MAX)) ? 1 : 0;

    if (straight_stable)
    {
        if (s_basic_straight_tick == 0)
        {
            s_basic_straight_tick = now_tick;
        }

        if ((now_tick - s_basic_straight_tick) >= BASIC_CORNER_RELEASE_MS)
        {
            s_basic_corner_armed = 1;
        }
    }
    else
    {
        s_basic_straight_tick = 0;
    }

    if (corner_candidate)
    {
        if (s_basic_corner_candidate_tick == 0)
        {
            s_basic_corner_candidate_tick = now_tick;
        }

        if (s_basic_corner_armed &&
            ((now_tick - s_basic_corner_candidate_tick) >= BASIC_CORNER_CONFIRM_MS) &&
            ((now_tick - s_basic_last_corner_tick) >= BASIC_CORNER_MIN_GAP_MS))
        {
            s_basic_corner_count++;
            s_basic_last_corner_tick = now_tick;
            s_basic_corner_armed = 0;
            s_basic_corner_candidate_tick = 0;
            s_basic_straight_tick = 0;
        }
    }
    else
    {
        s_basic_corner_candidate_tick = 0;
    }

    target_corners = s_basic_target_laps * BASIC_CORNERS_PER_LAP;

    if (s_basic_corner_count >= target_corners)
    {
        basic_track_stop(BASIC_TRACK_DONE);
        return 0;
    }

    return 1;
}

static void OLED_StatusTask_SafeRun(void)
{
    static uint64_t last_oled_tick = 0;
    uint64_t now_tick = SystemTickCNT;

    /*
     * 转弯时不刷新 OLED。
     */
    if (corner_mode)
    {
        return;
    }

    /*
     * 丢线时不刷新 OLED。
     */
    if (!line_detected)
    {
        return;
    }

    /*
     * 误差较大时不刷新 OLED。
     * 说明小车正在修正方向，这时候不能打断控制。
     */
    if (new_error > 1.0f || new_error < -1.0f)
    {
        return;
    }

    /*
     * 直线稳定时，才低频刷新 OLED。
     */
    if ((now_tick - last_oled_tick) >= 300)
    {
        last_oled_tick = now_tick;

        OLED_DisplayCarStatusAngle_LowTask(motor_status,
                                   RL_speed,
                                   RR_speed,
                                   g_pwm_RL_now,
                                   g_pwm_RR_now,
                                   g_f32Current_YawDeg - s_f32Yaw_Init,
                                   MPU6050_IsReady());
    }
}
/*int fputc(int c, FILE *f)
{
    while (DL_UART_isBusy(UART_MS901M_INST) == true);
    DL_UART_Main_transmitData(UART_MS901M_INST, (uint8_t)c);
    return c;
}*/

int main(void)
{
    board_init();
    s_f32Yaw_Init = g_f32Current_YawDeg;

    while (1)
    {
        #if TWO_WHEELS
        {
            basic_track_task();
        }

        #elif AKM_WHEELS
        {
            /* 阿克曼调试 */
            Velocity = 0;
            Turn = 0;
            delay_ms(1000);

            Velocity = 10;
            Turn = -45;
            delay_ms(1000);

            Velocity = 10;
            Turn = 45;
            delay_ms(1000);
        }

        #elif FOUR_WHEELS
        {
            /* 四轮直驱调试 */
            Velocity = 5;
            delay_ms(1000);

            Velocity = 10;
            delay_ms(1000);
        }
        #endif

        /*
         * OLED 低频显示任务：
         * 放在循迹控制后面，不放中断里。
         */
        MPU6050_Task();
       OLED_StatusTask_SafeRun();

        delay_ms(1);
    }
}

// 1ms公共定时器；电机编码器脉冲计数
void TIMER_0_INST_IRQHandler(void)
{
    extern volatile int32_t Encode_CNT_FL, Encode_CNT_FR, Encode_CNT_RL, Encode_CNT_RR;
    extern volatile uint64_t SystemTickCNT;

    static uint64_t EncodeCNT = 0;
    EncodeCNT++;

    if (EncodeCNT % 20 == 0)
    {
        if (DL_TimerA_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_ZERO)
        {
            RL_speed = Encode_CNT_RL;
            RR_speed = -Encode_CNT_RR;

            Encode_CNT_FL = 0;
            Encode_CNT_FR = 0;
            Encode_CNT_RL = 0;
            Encode_CNT_RR = 0;

            /*
             * 注意：
             * motor_status == 1 时，说明小车正在循迹。
             * 循迹时由 bsp_track.c 里的 track_control() 输出 PWM。
             * 这里不能再用速度环重新写 PWM，否则会影响循迹效果。
             */
            if (motor_status == 0)
            {
                Motor_Kinematic_Analysis(Velocity, Turn);

                output = speed_pid_compute(Velocity, RL_speed, &RL_error);
                Motor_PwmSet_RL(output);

                output = speed_pid_compute(Velocity, RR_speed, &RR_error);
                Motor_PwmSet_RR(output);
            }
        }
    }

    SystemTickCNT++;
}
