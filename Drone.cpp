/*
 * Copyright (C) 2020 DJI.
 *
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-08-25     robomaster   first version
 */

#include <Wire.h>
#include <RMTT_Libs.h>

Drone::Drone()
{
    RGB.Init();
    TOF.Init();
    Matrix.Init(127);
    Matrix.SetLEDStatus(RMTT_MATRIX_CS, RMTT_MATRIX_SW, RMTT_MATRIX_LED_ON);
}

void Drone::Init()
{
    RGB.Init();
    TOF.Init();
    Matrix.Init(127);
    Matrix.SetLEDStatus(RMTT_MATRIX_CS, RMTT_MATRIX_SW, RMTT_MATRIX_LED_ON);
}

