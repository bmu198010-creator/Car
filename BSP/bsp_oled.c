#include "bsp_oled.h"
#include <stdio.h>

/*
 * 本驱动使用 SysConfig 中已经生成的 OLED GPIO 宏：
 *
 * OLED_PORT
 * OLED_SCL_D_PIN
 * OLED_SDA_D_PIN
 *
 * 你的工程里一般对应：
 * SCL_D -> PA28
 * SDA_D -> PA31
 */

/* OLED 引脚控制 */
#define OLED_SCL_HIGH()    DL_GPIO_setPins(OLED_PORT, OLED_SCL_D_PIN)
#define OLED_SCL_LOW()     DL_GPIO_clearPins(OLED_PORT, OLED_SCL_D_PIN)

#define OLED_SDA_HIGH()    DL_GPIO_setPins(OLED_PORT, OLED_SDA_D_PIN)
#define OLED_SDA_LOW()     DL_GPIO_clearPins(OLED_PORT, OLED_SDA_D_PIN)

/* OLED 基本参数 */
#define OLED_WIDTH         128
#define OLED_HEIGHT        64
#define OLED_PAGE_NUM      8

/* 函数声明 */
static void OLED_I2C_Delay(void);
static void OLED_InitDelay(void);
static void OLED_I2C_Start(void);
static void OLED_I2C_Stop(void);
static void OLED_I2C_WriteByte(uint8_t data);
static void OLED_WriteCommand(uint8_t command);
static void OLED_WriteData(uint8_t data);
static void OLED_SetCursor(uint8_t page, uint8_t x);

/*
 * 6x8 ASCII 字库
 * 支持字符范围：' ' 到 '~'
 */
static const uint8_t OLED_F6x8[][6] =
{
    {0x00,0x00,0x00,0x00,0x00,0x00}, /* 32 ' ' */
    {0x00,0x00,0x5F,0x00,0x00,0x00}, /* 33 '!' */
    {0x00,0x07,0x00,0x07,0x00,0x00}, /* 34 '"' */
    {0x14,0x7F,0x14,0x7F,0x14,0x00}, /* 35 '#' */
    {0x24,0x2A,0x7F,0x2A,0x12,0x00}, /* 36 '$' */
    {0x23,0x13,0x08,0x64,0x62,0x00}, /* 37 '%' */
    {0x36,0x49,0x55,0x22,0x50,0x00}, /* 38 '&' */
    {0x00,0x05,0x03,0x00,0x00,0x00}, /* 39 ''' */
    {0x00,0x1C,0x22,0x41,0x00,0x00}, /* 40 '(' */
    {0x00,0x41,0x22,0x1C,0x00,0x00}, /* 41 ')' */
    {0x14,0x08,0x3E,0x08,0x14,0x00}, /* 42 '*' */
    {0x08,0x08,0x3E,0x08,0x08,0x00}, /* 43 '+' */
    {0x00,0x50,0x30,0x00,0x00,0x00}, /* 44 ',' */
    {0x08,0x08,0x08,0x08,0x08,0x00}, /* 45 '-' */
    {0x00,0x60,0x60,0x00,0x00,0x00}, /* 46 '.' */
    {0x20,0x10,0x08,0x04,0x02,0x00}, /* 47 '/' */

    {0x3E,0x51,0x49,0x45,0x3E,0x00}, /* 48 '0' */
    {0x00,0x42,0x7F,0x40,0x00,0x00}, /* 49 '1' */
    {0x42,0x61,0x51,0x49,0x46,0x00}, /* 50 '2' */
    {0x21,0x41,0x45,0x4B,0x31,0x00}, /* 51 '3' */
    {0x18,0x14,0x12,0x7F,0x10,0x00}, /* 52 '4' */
    {0x27,0x45,0x45,0x45,0x39,0x00}, /* 53 '5' */
    {0x3C,0x4A,0x49,0x49,0x30,0x00}, /* 54 '6' */
    {0x01,0x71,0x09,0x05,0x03,0x00}, /* 55 '7' */
    {0x36,0x49,0x49,0x49,0x36,0x00}, /* 56 '8' */
    {0x06,0x49,0x49,0x29,0x1E,0x00}, /* 57 '9' */

    {0x00,0x36,0x36,0x00,0x00,0x00}, /* 58 ':' */
    {0x00,0x56,0x36,0x00,0x00,0x00}, /* 59 ';' */
    {0x08,0x14,0x22,0x41,0x00,0x00}, /* 60 '<' */
    {0x14,0x14,0x14,0x14,0x14,0x00}, /* 61 '=' */
    {0x00,0x41,0x22,0x14,0x08,0x00}, /* 62 '>' */
    {0x02,0x01,0x51,0x09,0x06,0x00}, /* 63 '?' */
    {0x32,0x49,0x79,0x41,0x3E,0x00}, /* 64 '@' */

    {0x7E,0x11,0x11,0x11,0x7E,0x00}, /* 65 'A' */
    {0x7F,0x49,0x49,0x49,0x36,0x00}, /* 66 'B' */
    {0x3E,0x41,0x41,0x41,0x22,0x00}, /* 67 'C' */
    {0x7F,0x41,0x41,0x22,0x1C,0x00}, /* 68 'D' */
    {0x7F,0x49,0x49,0x49,0x41,0x00}, /* 69 'E' */
    {0x7F,0x09,0x09,0x09,0x01,0x00}, /* 70 'F' */
    {0x3E,0x41,0x49,0x49,0x7A,0x00}, /* 71 'G' */
    {0x7F,0x08,0x08,0x08,0x7F,0x00}, /* 72 'H' */
    {0x00,0x41,0x7F,0x41,0x00,0x00}, /* 73 'I' */
    {0x20,0x40,0x41,0x3F,0x01,0x00}, /* 74 'J' */
    {0x7F,0x08,0x14,0x22,0x41,0x00}, /* 75 'K' */
    {0x7F,0x40,0x40,0x40,0x40,0x00}, /* 76 'L' */
    {0x7F,0x02,0x0C,0x02,0x7F,0x00}, /* 77 'M' */
    {0x7F,0x04,0x08,0x10,0x7F,0x00}, /* 78 'N' */
    {0x3E,0x41,0x41,0x41,0x3E,0x00}, /* 79 'O' */
    {0x7F,0x09,0x09,0x09,0x06,0x00}, /* 80 'P' */
    {0x3E,0x41,0x51,0x21,0x5E,0x00}, /* 81 'Q' */
    {0x7F,0x09,0x19,0x29,0x46,0x00}, /* 82 'R' */
    {0x46,0x49,0x49,0x49,0x31,0x00}, /* 83 'S' */
    {0x01,0x01,0x7F,0x01,0x01,0x00}, /* 84 'T' */
    {0x3F,0x40,0x40,0x40,0x3F,0x00}, /* 85 'U' */
    {0x1F,0x20,0x40,0x20,0x1F,0x00}, /* 86 'V' */
    {0x3F,0x40,0x38,0x40,0x3F,0x00}, /* 87 'W' */
    {0x63,0x14,0x08,0x14,0x63,0x00}, /* 88 'X' */
    {0x07,0x08,0x70,0x08,0x07,0x00}, /* 89 'Y' */
    {0x61,0x51,0x49,0x45,0x43,0x00}, /* 90 'Z' */

    {0x00,0x7F,0x41,0x41,0x00,0x00}, /* 91 '[' */
    {0x02,0x04,0x08,0x10,0x20,0x00}, /* 92 '\' */
    {0x00,0x41,0x41,0x7F,0x00,0x00}, /* 93 ']' */
    {0x04,0x02,0x01,0x02,0x04,0x00}, /* 94 '^' */
    {0x40,0x40,0x40,0x40,0x40,0x00}, /* 95 '_' */
    {0x00,0x01,0x02,0x04,0x00,0x00}, /* 96 '`' */

    {0x20,0x54,0x54,0x54,0x78,0x00}, /* 97 'a' */
    {0x7F,0x48,0x44,0x44,0x38,0x00}, /* 98 'b' */
    {0x38,0x44,0x44,0x44,0x20,0x00}, /* 99 'c' */
    {0x38,0x44,0x44,0x48,0x7F,0x00}, /* 100 'd' */
    {0x38,0x54,0x54,0x54,0x18,0x00}, /* 101 'e' */
    {0x08,0x7E,0x09,0x01,0x02,0x00}, /* 102 'f' */
    {0x0C,0x52,0x52,0x52,0x3E,0x00}, /* 103 'g' */
    {0x7F,0x08,0x04,0x04,0x78,0x00}, /* 104 'h' */
    {0x00,0x44,0x7D,0x40,0x00,0x00}, /* 105 'i' */
    {0x20,0x40,0x44,0x3D,0x00,0x00}, /* 106 'j' */
    {0x7F,0x10,0x28,0x44,0x00,0x00}, /* 107 'k' */
    {0x00,0x41,0x7F,0x40,0x00,0x00}, /* 108 'l' */
    {0x7C,0x04,0x18,0x04,0x78,0x00}, /* 109 'm' */
    {0x7C,0x08,0x04,0x04,0x78,0x00}, /* 110 'n' */
    {0x38,0x44,0x44,0x44,0x38,0x00}, /* 111 'o' */
    {0x7C,0x14,0x14,0x14,0x08,0x00}, /* 112 'p' */
    {0x08,0x14,0x14,0x18,0x7C,0x00}, /* 113 'q' */
    {0x7C,0x08,0x04,0x04,0x08,0x00}, /* 114 'r' */
    {0x48,0x54,0x54,0x54,0x20,0x00}, /* 115 's' */
    {0x04,0x3F,0x44,0x40,0x20,0x00}, /* 116 't' */
    {0x3C,0x40,0x40,0x20,0x7C,0x00}, /* 117 'u' */
    {0x1C,0x20,0x40,0x20,0x1C,0x00}, /* 118 'v' */
    {0x3C,0x40,0x30,0x40,0x3C,0x00}, /* 119 'w' */
    {0x44,0x28,0x10,0x28,0x44,0x00}, /* 120 'x' */
    {0x0C,0x50,0x50,0x50,0x3C,0x00}, /* 121 'y' */
    {0x44,0x64,0x54,0x4C,0x44,0x00}, /* 122 'z' */

    {0x00,0x08,0x36,0x41,0x00,0x00}, /* 123 '{' */
    {0x00,0x00,0x7F,0x00,0x00,0x00}, /* 124 '|' */
    {0x00,0x41,0x36,0x08,0x00,0x00}, /* 125 '}' */
    {0x08,0x04,0x08,0x10,0x08,0x00}  /* 126 '~' */
};

static void OLED_I2C_Delay(void)
{
    volatile uint16_t i;

    for (i = 0; i < 20; i++)
    {
        ;
    }
}

static void OLED_InitDelay(void)
{
    volatile uint32_t i;

    for (i = 0; i < 300000; i++)
    {
        ;
    }
}

static void OLED_I2C_Start(void)
{
    OLED_SDA_HIGH();
    OLED_SCL_HIGH();
    OLED_I2C_Delay();

    OLED_SDA_LOW();
    OLED_I2C_Delay();

    OLED_SCL_LOW();
    OLED_I2C_Delay();
}

static void OLED_I2C_Stop(void)
{
    OLED_SDA_LOW();
    OLED_SCL_HIGH();
    OLED_I2C_Delay();

    OLED_SDA_HIGH();
    OLED_I2C_Delay();
}

static void OLED_I2C_WriteByte(uint8_t data)
{
    uint8_t i;

    for (i = 0; i < 8; i++)
    {
        if (data & 0x80)
        {
            OLED_SDA_HIGH();
        }
        else
        {
            OLED_SDA_LOW();
        }

        OLED_I2C_Delay();

        OLED_SCL_HIGH();
        OLED_I2C_Delay();

        OLED_SCL_LOW();
        OLED_I2C_Delay();

        data <<= 1;
    }

    OLED_SDA_HIGH();
    OLED_I2C_Delay();

    OLED_SCL_HIGH();
    OLED_I2C_Delay();

    OLED_SCL_LOW();
    OLED_I2C_Delay();
}

static void OLED_WriteCommand(uint8_t command)
{
    OLED_I2C_Start();
    OLED_I2C_WriteByte(OLED_I2C_ADDR);
    OLED_I2C_WriteByte(0x00);
    OLED_I2C_WriteByte(command);
    OLED_I2C_Stop();
}

static void OLED_WriteData(uint8_t data)
{
    OLED_I2C_Start();
    OLED_I2C_WriteByte(OLED_I2C_ADDR);
    OLED_I2C_WriteByte(0x40);
    OLED_I2C_WriteByte(data);
    OLED_I2C_Stop();
}

static void OLED_SetCursor(uint8_t page, uint8_t x)
{
    if (page >= OLED_PAGE_NUM)
    {
        page = OLED_PAGE_NUM - 1;
    }

    if (x >= OLED_WIDTH)
    {
        x = OLED_WIDTH - 1;
    }

    OLED_WriteCommand(0xB0 | page);
    OLED_WriteCommand(0x10 | ((x & 0xF0) >> 4));
    OLED_WriteCommand(0x00 | (x & 0x0F));
}

void OLED_Clear(void)
{
    uint8_t page;
    uint8_t x;

    for (page = 0; page < OLED_PAGE_NUM; page++)
    {
        OLED_SetCursor(page, 0);

        for (x = 0; x < OLED_WIDTH; x++)
        {
            OLED_WriteData(0x00);
        }
    }
}

void OLED_ClearLine(uint8_t line)
{
    uint8_t x;

    if (line >= OLED_PAGE_NUM)
    {
        return;
    }

    OLED_SetCursor(line, 0);

    for (x = 0; x < OLED_WIDTH; x++)
    {
        OLED_WriteData(0x00);
    }
}

void OLED_Init(void)
{
    OLED_SCL_HIGH();
    OLED_SDA_HIGH();

    OLED_InitDelay();

    OLED_WriteCommand(0xAE);
    OLED_WriteCommand(0x20);
    OLED_WriteCommand(0x02);
    OLED_WriteCommand(0xB0);
    OLED_WriteCommand(0xC8);
    OLED_WriteCommand(0x00);
    OLED_WriteCommand(0x10);
    OLED_WriteCommand(0x40);
    OLED_WriteCommand(0x81);
    OLED_WriteCommand(0x7F);
    OLED_WriteCommand(0xA1);
    OLED_WriteCommand(0xA6);
    OLED_WriteCommand(0xA8);
    OLED_WriteCommand(0x3F);
    OLED_WriteCommand(0xA4);
    OLED_WriteCommand(0xD3);
    OLED_WriteCommand(0x00);
    OLED_WriteCommand(0xD5);
    OLED_WriteCommand(0x80);
    OLED_WriteCommand(0xD9);
    OLED_WriteCommand(0xF1);
    OLED_WriteCommand(0xDA);
    OLED_WriteCommand(0x12);
    OLED_WriteCommand(0xDB);
    OLED_WriteCommand(0x40);
    OLED_WriteCommand(0x8D);
    OLED_WriteCommand(0x14);
    OLED_WriteCommand(0xAF);

    OLED_Clear();
}

void OLED_ShowChar(uint8_t line, uint8_t column, char ch)
{
    uint8_t i;
    uint8_t index;
    uint8_t x;

    if (line >= OLED_PAGE_NUM)
    {
        return;
    }

    if (column > 20)
    {
        return;
    }

    if (ch < ' ' || ch > '~')
    {
        ch = ' ';
    }

    index = (uint8_t)(ch - ' ');
    x = column * 6;

    OLED_SetCursor(line, x);

    for (i = 0; i < 6; i++)
    {
        OLED_WriteData(OLED_F6x8[index][i]);
    }
}

void OLED_ShowString(uint8_t line, uint8_t column, const char *str)
{
    while ((*str != '\0') && (column <= 20))
    {
        OLED_ShowChar(line, column, *str);
        column++;
        str++;
    }
}

void OLED_DisplayCarStatus(uint8_t mode,
                           int left_speed,
                           int right_speed,
                           int left_pwm,
                           int right_pwm)
{
    char buf[22];

    OLED_ClearLine(0);
    if (mode)
    {
        OLED_ShowString(0, 0, "MODE:TRACK");
    }
    else
    {
        OLED_ShowString(0, 0, "MODE:SPEED");
    }

    OLED_ClearLine(2);
    snprintf(buf, sizeof(buf), "LSPD:%4d", left_speed);
    OLED_ShowString(2, 0, buf);

    OLED_ClearLine(3);
    snprintf(buf, sizeof(buf), "RSPD:%4d", right_speed);
    OLED_ShowString(3, 0, buf);

    OLED_ClearLine(5);
    snprintf(buf, sizeof(buf), "LPWM:%4d", left_pwm);
    OLED_ShowString(5, 0, buf);

    OLED_ClearLine(6);
    snprintf(buf, sizeof(buf), "RPWM:%4d", right_pwm);
    OLED_ShowString(6, 0, buf);
}

void OLED_DisplayCarStatus_LowTask(uint8_t mode,
                                   int left_speed,
                                   int right_speed,
                                   int left_pwm,
                                   int right_pwm)
{
    static uint8_t refresh_step = 0;
    char buf[22];

    switch (refresh_step)
    {
        case 0:
            if (mode)
            {
                OLED_ShowString(0, 0, "MODE:TRACK  ");
            }
            else
            {
                OLED_ShowString(0, 0, "MODE:SPEED  ");
            }
            break;

        case 1:
            snprintf(buf, sizeof(buf), "LSPD:%5d   ", left_speed);
            OLED_ShowString(2, 0, buf);
            break;

        case 2:
            snprintf(buf, sizeof(buf), "RSPD:%5d   ", right_speed);
            OLED_ShowString(3, 0, buf);
            break;

        case 3:
            snprintf(buf, sizeof(buf), "LPWM:%5d   ", left_pwm);
            OLED_ShowString(5, 0, buf);
            break;

        default:
            snprintf(buf, sizeof(buf), "RPWM:%5d   ", right_pwm);
            OLED_ShowString(6, 0, buf);
            break;
    }

    refresh_step++;

    if (refresh_step >= 5)
    {
        refresh_step = 0;
    }
}

void OLED_DisplayCarStatusAngle_LowTask(uint8_t mode,
                                        int left_speed,
                                        int right_speed,
                                        int left_pwm,
                                        int right_pwm,
                                        float yaw_deg,
                                        uint8_t angle_valid)
{
    static uint8_t refresh_step = 0;
    char buf[22];

    switch (refresh_step)
    {
        case 0:
            if (mode)
            {
                OLED_ShowString(0, 0, "MODE:TRACK  ");
            }
            else
            {
                OLED_ShowString(0, 0, "MODE:SPEED  ");
            }
            break;

        case 1:
            snprintf(buf, sizeof(buf), "LSPD:%5d   ", left_speed);
            OLED_ShowString(2, 0, buf);
            break;

        case 2:
            snprintf(buf, sizeof(buf), "RSPD:%5d   ", right_speed);
            OLED_ShowString(3, 0, buf);
            break;

        case 3:
            snprintf(buf, sizeof(buf), "LPWM:%5d   ", left_pwm);
            OLED_ShowString(5, 0, buf);
            break;

        case 4:
            snprintf(buf, sizeof(buf), "RPWM:%5d   ", right_pwm);
            OLED_ShowString(6, 0, buf);
            break;

        default:
        {
            if (angle_valid)
            {
                int32_t yaw10;
                long yaw_abs;
                char sign;

                if (yaw_deg >= 0.0f)
                {
                    yaw10 = (int32_t)(yaw_deg * 10.0f + 0.5f);
                    sign = '+';
                }
                else
                {
                    yaw10 = (int32_t)(yaw_deg * 10.0f - 0.5f);
                    sign = '-';
                }

                yaw_abs = (long)((yaw10 >= 0) ? yaw10 : -yaw10);

                snprintf(buf, sizeof(buf), "ANG:%c%3ld.%1lddeg ",
                         sign,
                         yaw_abs / 10,
                         yaw_abs % 10);

                OLED_ShowString(7, 0, buf);
            }
            else
            {
                OLED_ShowString(7, 0, "MPU6050:ERR   ");
            }

            break;
        }
    }

    refresh_step++;

    if (refresh_step >= 6)
    {
        refresh_step = 0;
    }
}