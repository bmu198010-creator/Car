#include "bsp_mpu6050.h"
#include "board.h"

/*
 * 硬件 I2C 版本 MPU6050 驱动
 *
 * 适配你的底板：
 * I2C_0_SDA -> PA0
 * I2C_0_SCL -> PA1
 *
 * SysConfig 要求：
 * 1. 保留 COMMUNICATIONS -> I2C -> I2C_0
 * 2. I2C_0 配成 Controller / Master
 * 3. SDA = PA0
 * 4. SCL = PA1
 * 5. 删除 GPIO/GYRO 里的 SDA、SCL，避免 PA0/PA1 冲突
 */

#ifndef I2C_0_INST
#error "I2C_0_INST 未定义：请检查 SysConfig 是否已经添加 I2C_0"
#endif

#define MPU6050_REG_SMPLRT_DIV       0x19u
#define MPU6050_REG_CONFIG           0x1Au
#define MPU6050_REG_GYRO_CONFIG      0x1Bu
#define MPU6050_REG_ACCEL_CONFIG     0x1Cu
#define MPU6050_REG_GYRO_ZOUT_H      0x47u
#define MPU6050_REG_PWR_MGMT_1       0x6Bu
#define MPU6050_REG_WHO_AM_I         0x75u

/*
 * 陀螺仪量程 ±500 dps 时，灵敏度为 65.5 LSB/(deg/s)
 */
#define MPU6050_GYRO_SENS_500DPS     (65.5f)

/*
 * 上电静止校准次数。
 * 400 次，每次间隔 2ms，大约 800ms。
 */
#define MPU6050_CALIB_SAMPLES        400u

/*
 * MPU6050 姿态更新周期。
 * 10ms 更新一次角度。
 */
#define MPU6050_UPDATE_PERIOD_MS     10u

/*
 * 静止死区。
 * 如果小车静止时角度仍慢慢漂，可以把 0.25f 改成 0.40f 或 0.50f。
 */
#define MPU6050_GYRO_DEADBAND_DPS    (0.25f)

/*
 * I2C 超时计数。
 * 如果线接错、模块没供电、SDA/SCL 反接，不能让程序卡死。
 */
#define MPU6050_I2C_TIMEOUT          200000u

extern volatile uint64_t SystemTickCNT;
extern volatile float g_f32Current_YawDeg;

static volatile uint8_t s_mpu6050_ready = 0;
static float s_gyro_z_offset_dps = 0.0f;
static uint64_t s_last_update_tick = 0;


/*==========================================================
 *                      I2C 底层函数
 *==========================================================*/

/*
 * 等待 I2C 控制器空闲。
 */
static uint8_t MPU6050_I2C_WaitIdle(void)
{
    volatile uint32_t timeout = MPU6050_I2C_TIMEOUT;

    while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE))
    {
        if (timeout-- == 0)
        {
            return 0;
        }
    }

    return 1;
}

/*
 * 等待 I2C 总线传输完成。
 */
static uint8_t MPU6050_I2C_WaitBusNotBusy(void)
{
    volatile uint32_t timeout = MPU6050_I2C_TIMEOUT;

    while (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_BUSY_BUS)
    {
        if (timeout-- == 0)
        {
            DL_I2C_resetControllerTransfer(I2C_0_INST);
            return 0;
        }
    }

    if (DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_ERROR)
    {
        DL_I2C_resetControllerTransfer(I2C_0_INST);
        return 0;
    }

    return 1;
}

/*
 * 等待 RX FIFO 中有数据。
 */
static uint8_t MPU6050_I2C_WaitRxNotEmpty(void)
{
    volatile uint32_t timeout = MPU6050_I2C_TIMEOUT;

    while (DL_I2C_isControllerRXFIFOEmpty(I2C_0_INST))
    {
        if (timeout-- == 0)
        {
            DL_I2C_resetControllerTransfer(I2C_0_INST);
            return 0;
        }
    }

    return 1;
}

/*
 * 硬件 I2C 发送 len 个字节。
 *
 * 注意：
 * MPU6050 的 7 位地址是 0x68。
 * DL_I2C_startControllerTransfer() 这里填写 7 位地址，不要填 0xD0。
 */
static uint8_t MPU6050_I2C_WriteBytes(uint8_t addr_7bit, const uint8_t *data, uint16_t len)
{
    uint16_t filled_count;

    if ((data == 0) || (len == 0))
    {
        return 0;
    }

    if (!MPU6050_I2C_WaitIdle())
    {
        return 0;
    }

    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
    DL_I2C_flushControllerRXFIFO(I2C_0_INST);
    DL_I2C_resetControllerTransfer(I2C_0_INST);

    /*
     * 你的 MPU6050 写寄存器最多只用到 2 字节：
     * data[0] = 寄存器地址
     * data[1] = 写入数据
     *
     * 这里仍然写成通用形式。
     */
    filled_count = DL_I2C_fillControllerTXFIFO(I2C_0_INST, data, len);

    if (filled_count != len)
    {
        DL_I2C_resetControllerTransfer(I2C_0_INST);
        return 0;
    }

    DL_I2C_startControllerTransfer(I2C_0_INST,
                                   addr_7bit,
                                   DL_I2C_CONTROLLER_DIRECTION_TX,
                                   len);

    if (!MPU6050_I2C_WaitBusNotBusy())
    {
        return 0;
    }

    if (!MPU6050_I2C_WaitIdle())
    {
        return 0;
    }

    return 1;
}

/*
 * 硬件 I2C 读取 len 个字节。
 */
static uint8_t MPU6050_I2C_ReadBytes(uint8_t addr_7bit, uint8_t *buf, uint16_t len)
{
    uint16_t i;

    if ((buf == 0) || (len == 0))
    {
        return 0;
    }

    if (!MPU6050_I2C_WaitIdle())
    {
        return 0;
    }

    DL_I2C_flushControllerTXFIFO(I2C_0_INST);
    DL_I2C_flushControllerRXFIFO(I2C_0_INST);
    DL_I2C_resetControllerTransfer(I2C_0_INST);

    DL_I2C_startControllerTransfer(I2C_0_INST,
                                   addr_7bit,
                                   DL_I2C_CONTROLLER_DIRECTION_RX,
                                   len);

    for (i = 0; i < len; i++)
    {
        if (!MPU6050_I2C_WaitRxNotEmpty())
        {
            return 0;
        }

        buf[i] = DL_I2C_receiveControllerData(I2C_0_INST);
    }

    if (!MPU6050_I2C_WaitBusNotBusy())
    {
        return 0;
    }

    if (!MPU6050_I2C_WaitIdle())
    {
        return 0;
    }

    return 1;
}


/*==========================================================
 *                   MPU6050 寄存器读写
 *==========================================================*/

/*
 * 写 MPU6050 一个寄存器。
 *
 * I2C 实际发送：
 * START + 0x68 + W + reg + data + STOP
 */
static uint8_t MPU6050_WriteReg(uint8_t reg, uint8_t data)
{
    uint8_t packet[2];

    packet[0] = reg;
    packet[1] = data;

    return MPU6050_I2C_WriteBytes(MPU6050_ADDR_7BIT, packet, 2);
}

/*
 * 从 MPU6050 连续读取 len 个寄存器。
 *
 * 这里用的是：
 * 1. 先写寄存器地址 reg
 * 2. 再读 len 个字节
 *
 * 对 MPU6050 来说，这种 STOP 后再读的方式可以正常工作。
 */
static uint8_t MPU6050_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    if ((buf == 0) || (len == 0))
    {
        return 0;
    }

    /*
     * 第一步：写入要读取的寄存器地址。
     */
    if (!MPU6050_I2C_WriteBytes(MPU6050_ADDR_7BIT, &reg, 1))
    {
        return 0;
    }

    /*
     * 第二步：读取寄存器数据。
     */
    if (!MPU6050_I2C_ReadBytes(MPU6050_ADDR_7BIT, buf, len))
    {
        return 0;
    }

    return 1;
}

static int16_t MPU6050_MakeInt16(uint8_t high, uint8_t low)
{
    return (int16_t)(((uint16_t)high << 8) | low);
}

static uint8_t MPU6050_ReadGyroZRaw(int16_t *gyro_z)
{
    uint8_t buf[2];

    if (gyro_z == 0)
    {
        return 0;
    }

    if (!MPU6050_ReadRegs(MPU6050_REG_GYRO_ZOUT_H, buf, 2))
    {
        return 0;
    }

    *gyro_z = MPU6050_MakeInt16(buf[0], buf[1]);

    return 1;
}


/*==========================================================
 *                   MPU6050 校准与姿态计算
 *==========================================================*/

static void MPU6050_CalibrateGyroZ(void)
{
    uint16_t i;
    int16_t raw_z;
    float sum = 0.0f;
    uint16_t valid_count = 0;

    /*
     * 校准期间小车必须静止。
     */
    for (i = 0; i < MPU6050_CALIB_SAMPLES; i++)
    {
        if (MPU6050_ReadGyroZRaw(&raw_z))
        {
            sum += ((float)raw_z / MPU6050_GYRO_SENS_500DPS);
            valid_count++;
        }

        delay_ms(2);
    }

    if (valid_count > 0)
    {
        s_gyro_z_offset_dps = sum / (float)valid_count;
    }
    else
    {
        s_gyro_z_offset_dps = 0.0f;
    }
}


/*==========================================================
 *                         外部接口
 *==========================================================*/

uint8_t MPU6050_Init(void)
{
    uint8_t who_am_i = 0;

    s_mpu6050_ready = 0;
    g_f32Current_YawDeg = 0.0f;
    s_gyro_z_offset_dps = 0.0f;

    /*
     * SYSCFG_DL_init() 已经初始化 I2C_0。
     * 这里不需要再调用 SYSCFG_DL_I2C_0_init()。
     */
    delay_ms(100);

    /*
     * 读取 WHO_AM_I。
     * MPU6050 正常应该返回 0x68。
     */
    if (!MPU6050_ReadRegs(MPU6050_REG_WHO_AM_I, &who_am_i, 1))
    {
        return 0;
    }

    if (who_am_i != 0x68u)
    {
        return 0;
    }

    /*
     * 复位 MPU6050。
     */
    if (!MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_1, 0x80u))
    {
        return 0;
    }

    delay_ms(100);

    /*
     * 唤醒 MPU6050。
     * 0x01 表示使用 X 轴陀螺仪 PLL 作为时钟源，比内部振荡器稳定。
     */
    if (!MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_1, 0x01u))
    {
        return 0;
    }

    delay_ms(10);

    /*
     * 采样率分频。
     * DLPF 打开后内部采样率约 1kHz。
     * 0x07 => 1kHz / (1 + 7) = 125Hz。
     */
    if (!MPU6050_WriteReg(MPU6050_REG_SMPLRT_DIV, 0x07u))
    {
        return 0;
    }

    /*
     * 数字低通滤波。
     * 0x03 对小车振动比较友好。
     */
    if (!MPU6050_WriteReg(MPU6050_REG_CONFIG, 0x03u))
    {
        return 0;
    }

    /*
     * 陀螺仪量程 ±500 dps。
     * 对应灵敏度 65.5 LSB/(deg/s)。
     */
    if (!MPU6050_WriteReg(MPU6050_REG_GYRO_CONFIG, 0x08u))
    {
        return 0;
    }

    /*
     * 加速度计量程 ±2g。
     * 当前只用 Z 轴陀螺仪积分 yaw，暂时不使用加速度。
     */
    if (!MPU6050_WriteReg(MPU6050_REG_ACCEL_CONFIG, 0x00u))
    {
        return 0;
    }

    /*
     * 上电静止校准 Z 轴零偏。
     */
    MPU6050_CalibrateGyroZ();

    g_f32Current_YawDeg = 0.0f;
    s_last_update_tick = SystemTickCNT;
    s_mpu6050_ready = 1;

    return 1;
}

void MPU6050_Task(void)
{
    uint64_t now_tick;
    uint32_t delta_ms;
    float dt;
    int16_t raw_z;
    float gyro_z_dps;
    float yaw;

    if (!s_mpu6050_ready)
    {
        return;
    }

    now_tick = SystemTickCNT;
    delta_ms = (uint32_t)(now_tick - s_last_update_tick);

    if (delta_ms < MPU6050_UPDATE_PERIOD_MS)
    {
        return;
    }

    s_last_update_tick = now_tick;

    dt = (float)delta_ms / 1000.0f;

    /*
     * 防止因为程序卡顿导致积分异常。
     */
    if ((dt <= 0.0f) || (dt > 0.2f))
    {
        dt = (float)MPU6050_UPDATE_PERIOD_MS / 1000.0f;
    }

    if (!MPU6050_ReadGyroZRaw(&raw_z))
    {
        return;
    }

    gyro_z_dps = ((float)raw_z / MPU6050_GYRO_SENS_500DPS) - s_gyro_z_offset_dps;

    /*
     * 小角速度死区，减小静止漂移。
     */
    if ((gyro_z_dps > -MPU6050_GYRO_DEADBAND_DPS) &&
        (gyro_z_dps <  MPU6050_GYRO_DEADBAND_DPS))
    {
        gyro_z_dps = 0.0f;
    }

    yaw = g_f32Current_YawDeg + gyro_z_dps * dt * MPU6050_YAW_SIGN;

    /*
     * 限制角度到 -180 ~ +180 度。
     */
    while (yaw > 180.0f)
    {
        yaw -= 360.0f;
    }

    while (yaw < -180.0f)
    {
        yaw += 360.0f;
    }

    g_f32Current_YawDeg = yaw;
}

void MPU6050_ResetYaw(void)
{
    g_f32Current_YawDeg = 0.0f;
    s_last_update_tick = SystemTickCNT;
}

uint8_t MPU6050_IsReady(void)
{
    return s_mpu6050_ready;
}

float MPU6050_GetYawDeg(void)
{
    return g_f32Current_YawDeg;
}