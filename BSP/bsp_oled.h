#ifndef __BSP_OLED_H__
#define __BSP_OLED_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>

/*
 * OLED 地址说明：
 * 常见 SSD1306 OLED 的 7 位地址是 0x3C。
 * 发送时左移 1 位后就是 0x78。
 *
 * 如果你的 OLED 是 0x3D 地址，则这里改成 0x7A。
 */
#define OLED_I2C_ADDR      0x78

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ClearLine(uint8_t line);
void OLED_ShowChar(uint8_t line, uint8_t column, char ch);
void OLED_ShowString(uint8_t line, uint8_t column, const char *str);

/*
 * mode:
 * 0 = SPEED 普通速度模式
 * 1 = TRACK 循迹模式
 */
void OLED_DisplayCarStatus(uint8_t mode,
                           int left_speed,
                           int right_speed,
                           int left_pwm,
                           int right_pwm);

/*
 * 低占用分行刷新版本：
 * 每调用一次只刷新 1 行。
 * 不清屏、不整屏刷新，适合放在循迹主循环后面低频调用。
 */
void OLED_DisplayCarStatus_LowTask(uint8_t mode,
                                   int left_speed,
                                   int right_speed,
                                   int left_pwm,
                                   int right_pwm);


void OLED_DisplayCarStatusAngle_LowTask(uint8_t mode,
                                        int left_speed,
                                        int right_speed,
                                        int left_pwm,
                                        int right_pwm,
                                        float yaw_deg,
                                        uint8_t angle_valid);
#endif