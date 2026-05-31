# 项目说明

## 基本信息

- 工程名称：`HAHAHAAH`。
- 工程类型：TI Code Composer Studio Theia / Eclipse CDT 工程。
- 目标芯片：`MSPM0G3507`，封装 `LQFP-48(PT)`。
- SDK 与工具：`mspm0_sdk@2.10.00.04`，SysConfig `1.26.2`，TI Arm Clang/TICLANG。
- 目标板来源模板：`empty_LP_MSPM0G3507_nortos_ticlang`，NoRTOS，DriverLib。
- 调试连接：SEGGER J-Link，目标配置文件为 `targetConfigs/MSPM0G3507.ccxml`。
- 当前主要用途：基于 MSPM0G3507 的小车控制程序，包含电机 PWM、编码器测速、八路循迹、OLED 状态显示、MPU6050 航向角估算、UART 缓冲收发等模块。

## 重要操作约束

- 禁止批量删除文件或目录。
- 不要使用 `del /s`、`rd /s`、`rmdir /s`、`Remove-Item -Recurse`、`rm -rf`。
- 需要删除文件时，只能一次删除一个明确路径的文件，例如 `Remove-Item "C:\path\to\file.txt"`。
- 如果需要批量删除文件，应停止操作并询问用户，让用户手动删除。

## 目录结构

- `empty.c`：主程序入口，包含小车基本循迹任务、OLED 安全刷新任务、1 ms 定时器中断逻辑。
- `empty.syscfg`：SysConfig 外设和引脚配置源文件，是生成 `Debug/ti_msp_dl_config.*` 的来源。
- `BSP/`：板级支持与业务模块源码。
- `targetConfigs/`：CCS 自动生成的目标配置，当前为 MSPM0G3507 + J-Link。
- `.ccsproject`、`.cproject`、`.project`、`.settings/`：CCS/Eclipse 工程元数据。
- `Debug/`：CCS 构建输出和 SysConfig 生成文件，已在 `.gitignore` 中忽略。里面有 `HAHAHAAH.out`、`HAHAHAAH.map`、`ti_msp_dl_config.c/h`、makefile 等生成产物。
- `README.md`、`README.html`：TI empty DriverLib 示例原始说明，内容仍偏模板化，不完全反映当前小车业务逻辑。

## 构建与配置

- 当前构建配置名为 `Debug`。
- 编译目标为 Cortex-M0+，Thumb v6-M，小端，soft-float。
- 编译优化等级为 `-O2`，调试信息为 DWARF-3。
- 依赖 MSPM0 SDK 的 include、library、symbols 和 SysConfig manifest 宏。
- 链接输出默认为 `${ProjName}.out`，map 文件为 `${ProjName}.map`。
- `.clangd` 指向 `Debug/.clangd` 的编译数据库，并抑制诊断；该文件注释说明为自动生成文件。

## SysConfig 外设与引脚

- 系统时钟：启用 HFXT，PA5/PA6；使用 SYSPLL，SysTick 周期配置为 80。
- PWM：`PWM_T` 使用 `TIMG6`，时钟分频 8，自动启动；CCP0 为 PB2，CCP1 为 PB3，两个通道均 invert。
- 1 ms 定时器：`TIMER_0` 使用 `TIMA0`，周期 1 ms，MFCLK，预分频 40，中断优先级 3，自动启动。
- I2C：`I2C_0` 使用 I2C0，Fast 模式，SDA=PA0，SCL=PA1。
- UART：`UART_MS901M` 使用 UART0，115200 baud，TX=PA10，RX=PA11。
- ADC：`ADC0_VOLTAGE` 使用 ADC1，输入 PA15，repeat mode。
- MPU/GYRO 中断脚：PA7。
- OLED 软件 I2C 脚：SCL_D=PA28，SDA_D=PA31。
- 电机方向脚：AIN1=PA13，AIN2=PA14，BIN1=PA16，BIN2=PA17。
- 编码器脚：OA1=PA25，OB1=PA26，OA2=PB20，OB2=PB24，相关 GPIO 配置了上升沿中断。
- 循迹传感器：S1=PA8，S2=PA9，S3=PA27，S4=PA24，S5=PA22，S6=PB17，S7=PB16，S8=PA12，均为输入并上拉。
- 调试口：SWCLK=PA20，SWDIO=PA19。

## 运行流程

- `main()` 首先调用 `board_init()`，随后记录初始 yaw 为 `s_f32Yaw_Init`。
- `board_init()` 调用 `SYSCFG_DL_init()` 初始化 SysConfig 生成的外设，然后使能 `TIMER_0` 中断，初始化电机 PID、OLED、MPU6050、电机编码器中断和循迹模块。
- `uart_init()` 当前在 `board_init()` 中被注释掉；UART 相关代码存在，但默认不启用中断初始化。
- `board.h` 中当前选择 `TWO_WHEELS = 1`，`AKM_WHEELS = 0`，`FOUR_WHEELS = 0`。
- 两轮模式主循环运行 `basic_track_task()`，并在循环末尾调用 `MPU6050_Task()` 和 `OLED_StatusTask_SafeRun()`，最后 `delay_ms(1)`。
- 基本循迹任务默认目标为 1 圈，每圈按 4 个弯统计；启动后忽略前 800 ms，弯道确认 140 ms，弯道释放 220 ms，弯道最小间隔 850 ms，总超时 20 s。
- 任务完成或超时时调用 `Motor_Stop()`，并把 `motor_status` 置为 2。

## 中断与时基

- `SystemTickCNT` 定义在 `BSP/uart.c`，由 `TIMER_0_INST_IRQHandler()` 每 1 ms 自增。
- `HAL_GetTick()` 返回 `SystemTickCNT`。
- `TIMER_0_INST_IRQHandler()` 每 20 ms 读取并清零编码器计数，更新 `RL_speed` 和 `RR_speed`。
- 当 `motor_status == 0` 时，定时器中断内执行速度环并写后轮 PWM。
- 当 `motor_status == 1` 时，表示正在循迹，PWM 由 `BSP/bsp_track.c` 的 `track_control()` 输出；定时器中断不要再用速度环重写 PWM，否则会影响循迹效果。
- 编码器 GPIO 中断在 `GROUP1_IRQHandler()` 中处理，后左轮使用 OB1/OA1，后右轮使用 OA2/OB2。

## BSP 模块

- `BSP/board.c/h`：统一包含各 BSP 头文件，负责板级初始化和基于 SysTick 的 `delay_us()`/`delay_ms()`。
- `BSP/bsp_motor.c/h`：电机方向与 PWM 控制、编码器计数、速度 PID 初始化、不同底盘运动学分配。当前重点使用后左 `Motor_PwmSet_RL()` 和后右 `Motor_PwmSet_RR()`，并记录 `g_pwm_RL_now`、`g_pwm_RR_now` 供 OLED 显示。
- `BSP/bsp_track.c/h`：八路循迹控制。传感器默认为检测黑线输出高电平；权重从左到右为负到正；循迹 PID 参数为 `Kp=8.8`、`Ki=0.0`、`Kd=4.8`；普通速度 200，弯道速度 85，最低基础速度 70；丢线时根据历史误差方向执行找线动作。
- `BSP/bsp_pid.c/h`：通用 PID 参数结构和接口，包含状态、模式、目标速度、测量速度、误差、输出限幅和函数指针接口。电机 PID 输出范围为 -999 到 999。
- `BSP/bsp_mpu6050.c/h`：MPU6050 驱动，I2C 地址 `0x68`。初始化会读取 `WHO_AM_I`，复位并唤醒芯片，设置采样率 125 Hz、DLPF、陀螺仪量程正负 500 dps、加速度正负 2g，并用 400 个样本校准 Z 轴零偏。`MPU6050_Task()` 每 10 ms 积分 Z 轴角速度得到 yaw，并限制到 -180 到 +180 度。
- `BSP/bsp_oled.c/h`：SSD1306 类 OLED 软件 I2C 驱动，地址宏为左移后的 `0x78`。支持清屏、清行、字符/字符串显示，以及小车状态和 yaw 的低占用分行刷新。
- `BSP/uart.c/h`：UART 环形缓冲、带时间戳的 `uart_printf()`、UART0/MS901M 收发队列和 UART0 中断接收。注意当前工程初始化中 `uart_init()` 被注释，`printf` 重定向宏也被注释。
- `BSP/ringbuffer.c/h`：通用环形缓冲区实现，支持 put、put_force、get、peek、putchar、getchar、reset、data_len，以及可选动态创建/销毁。

## OLED 刷新策略

- `OLED_StatusTask_SafeRun()` 不在弯道、丢线或误差较大时刷新 OLED，避免显示刷新影响循迹控制。
- 只有直线稳定且距离上次刷新超过 300 ms 时，才调用 `OLED_DisplayCarStatusAngle_LowTask()` 显示模式、左右速度、左右 PWM、相对 yaw 和 MPU6050 是否 ready。

## 已知注意事项

- 代码里不少中文注释在当前读取环境下显示为乱码，说明源码文件很可能不是 UTF-8 编码，或历史保存时编码不一致。修改这些文件时要谨慎，避免无关的大范围重编码。
- `README.md` 是 TI empty 示例说明，没有同步当前小车控制功能。
- `Debug/` 是生成目录，不应手工维护；需要修改引脚和外设时优先改 `empty.syscfg`，再由 CCS/SysConfig 重新生成。
- `Motor_Stop()` 对前后左右的停止逻辑复用了同两路方向脚和两路 PWM；当前工程实际更像两轮后驱控制，前轮相关函数保留但不是主路径。
- `bsp_track.c` 中许多参数是现场调车参数，如速度、转向增益、弯道保持计数、PWM 限幅和滤波系数；改动前应记录原值并在实车上验证。
- `.gitignore` 已忽略 `Debug/`、目标文件、map、out、linkInfo 等构建产物。
