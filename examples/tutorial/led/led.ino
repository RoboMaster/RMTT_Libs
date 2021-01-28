/*
 * Copyright (C) 2020 DJI.
 *
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-08-25     robomaster   first version
 */

#include <RMTT_Libs.h>
#include <Wire.h>

#define BREATH_Hz (0.5)
#define BREATH_RED (0)
#define BREATH_BLUE (255)
#define BREATH_GREEN (255)

RMTT_RGB tt_rgb;            // 实例化RMTT_RGB对象
bool breath_toggle = false; // 亮度改变方向
uint8_t breath_rate = 0;    // Percentage of brightness 亮度百分比
int period = 10;

void setup()
{
    Wire.begin(27, 26); // 初始化I2C总线
    Wire.setClock(400000);
    period = 1000 / BREATH_Hz; // 计算单个呼吸周期时长 单位ms
    tt_rgb.Init();
    tt_rgb.SetRGB(0, 0, 0);
}

void loop()
{
    /* 亮度增或减, 注意breath_toggle的方向避免溢出 */
    breath_rate = breath_toggle ? breath_rate - 1 : breath_rate + 1;
    if ((breath_rate == 100) || (breath_rate == 0))
        breath_toggle = !breath_toggle; // 改变亮度增减方向
    tt_rgb.SetRGB(BREATH_RED * breath_rate / 100.0,
                  BREATH_GREEN * breath_rate / 100.0,
                  BREATH_BLUE * breath_rate / 100.0);
    delay(period / 100); // 延时一个呼吸周期的百分之一
}