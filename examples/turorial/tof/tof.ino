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

#define RANGE_MAX (0.64) // 满探测距离设置为0.64m，使每格表示1cm

RMTT_RGB tt_rgb;
RMTT_Matrix tt_matrix;
RMTT_TOF tt_sensor;
uint8_t tof_display[128] = {0}; // 点阵内容数组
float range = 0;

void setup()
{
    Serial.begin(115200);
    Wire.begin(27, 26);
    Wire.setClock(100000);
    tt_sensor.SetTimeout(500); // 设置单次测量超时时间
    if (!tt_sensor.Init())
    {
            Serial.println("Failed to detect and initialize sensor!");
            while (1)
            {
            }
    }
    tt_matrix.Init(127); // 初始化点阵
    tt_matrix.SetLEDStatus(RMTT_MATRIX_CS, RMTT_MATRIX_SW, RMTT_MATRIX_LED_ON); // 开启点阵LED
}

void loop()
{
    range = tt_sensor.ReadRangeSingleMillimeters();
    range = range / 1000; // 将数据单位转换至米
    Serial.println(range);
    if (tt_sensor.TimeoutOccurred())
    { // 判断当次测量是否未超时（是否为有效测量）
        Serial.print("TIMEOUT");
    }
    else
    {
        for (int i = 0; i < 64; i++)
        {
            if (range / RANGE_MAX > (float)i / (float)63)
            {
                tof_display[i * 2 + 1] =
                    (int)(((float)i / (float)63) * (float)255); // 仅控制红色亮度值以完成渐变色效果
                tof_display[i * 2] = 255;                       // 范围内蓝色始终最大
            }
            else
            {
                /*范围外所有LED熄灭*/
                tof_display[i * 2 + 1] = 0;
                tof_display[i * 2] = 0;
            }
        }
        tt_matrix.SetAllPWM(tof_display); // 发送至点阵屏以显示
        delay(15);
    }
}