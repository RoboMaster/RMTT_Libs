/*
 * Copyright (C) 2020 DJI.
 *
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-08-25     robomaster   first version
 */

#pragma once

#include <Arduino.h>

class RMTT_Protocol
{
private:
    void SendCMD(char *cmd);

public:
    RMTT_Protocol();
    RMTT_Protocol(uint16_t time);
    ~RMTT_Protocol();

    void SDKOn();
    void SDKOff();

    void TakeOff();
    void Land();
    void Emergency();
    void Up(int16_t x);
    void Down(int16_t x);
    void Left(int16_t x);
    void Right(int16_t x);
    void Forward(int16_t x);
    void Back(int16_t x);
    void CW(uint16_t x);
    void CCW(uint16_t x);
    void Flip(char x);
    void Go(int16_t x, int16_t y, int16_t z, uint16_t speed);
    void Go(int16_t x, int16_t y, int16_t z, uint16_t speed, char *mid);
    void Stop();
    void Curve(int16_t x1, int16_t y1, int16_t z1, int16_t x2, int16_t y2, int16_t z2, uint16_t speed);
    void Curve(int16_t x1, int16_t y1, int16_t z1, int16_t x2, int16_t y2, int16_t z2, uint16_t speed, char *mid);
    void Jump(int16_t x, int16_t y, int16_t z, uint16_t speed, int16_t yaw, char *mid, char *mid2);

    void SetSpeed(int16_t x);
    void SetRC(int16_t a, int16_t b, int16_t c, int16_t d);
    void SetMon();
    void SetMoff();
    void SetMdirection(uint8_t x);

    void ReadSpeed();
    void ReadBattery();
    void ReadTime();
    void ReadSN();
    void ReadSDKVersion();


    // 函数声明
    String getTelloMsgString(char* cmd, uint32_t timeout);
    int    getTelloMsgInt(char* cmd, uint32_t timeout);
    String getTelloResponseString(uint32_t timeout);
    int    getTelloResponseInt(uint32_t timeout);
    void   startUntilControl();
    int    sendTelloCtrlMsg(char *cmd_str);
};