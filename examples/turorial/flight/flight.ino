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

void setup() {
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, 23, 18);

  tt_sdk.startUntilControl();

  tt_sdk.sendTelloCtrlMsg("takeoff");
  tt_sdk.sendTelloCtrlMsg("left 30");
  tt_sdk.sendTelloCtrlMsg("right 30");
  tt_sdk.sendTelloCtrlMsg("land");
  tt_sdk.sendTelloCtrlMsg("takeoff");
  tt_sdk.sendTelloCtrlMsg("up 30");
  tt_sdk.sendTelloCtrlMsg("down 30");
  tt_sdk.sendTelloCtrlMsg("land");
}


void loop() {

}