/*
 * Copyright (C) 2020 DJI.
 *
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-08-25     robomaster   first version
 */

#include <Arduino.h>
#include <RMTT_Libs.h>
#include <Wire.h>
#include <stdio.h>
#include <string.h>

void setup()
{
    Serial.begin(115200);
    Wire.begin(27, 26);
    Wire.setClock(400000);
    Serial1.begin(1000000, SERIAL_8N1, 23, 18);
    Serial.println();
    RMTT_Matrix::Init(255);
    matrix_effect_init(255);
    matrix_effect_move_str("Hello Kitty", 11, 'r', 'l', 1);
}


void loop()
{

}
