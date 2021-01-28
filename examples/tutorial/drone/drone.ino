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

RMTT_Protocol tt_sdk;

void WaitTelloReady()
{
  while (1)
  {
    if (Serial1.available())
    {
      String ret = Serial1.readString();
      if (!strncmp(ret.c_str(), "ETT ok", 6))
      {
        Serial.println(ret.c_str());
        return;
      }
    }
    delay(500);
    tt_sdk.SDKOn();
  }
}


void setup() {
  Wire.begin(27, 26);
  Serial1.begin(1000000, SERIAL_8N1, 23, 18);


  RMTT_RGB::Init();
  RMTT_RGB::SetRGB(255, 0, 0);


  tt_sdk.SDKOn();
  WaitTelloReady();
  /* 阻塞线程并一直读取无人机串口返回数据，此后Tello可以确保已经初始化完毕 */


  RMTT_RGB::SetRGB(0, 255, 255);


}


void loop() {
  pinMode(34, INPUT_PULLUP);


  // 等待按键被按下
  while (digitalRead(34) == 1);


  // LED Blink
  RMTT_RGB::SetRGB(0, 255, 0);
  delay(1000);
  RMTT_RGB::SetRGB(0, 0, 0);
  delay(1000);
  RMTT_RGB::SetRGB(0, 255, 0);
  delay(1000);


  tt_sdk.TakeOff();
  delay(6000);
  tt_sdk.Land();
}