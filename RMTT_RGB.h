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

#include "stdint.h"

// use first channel of 16 channels (started from zero)
#define LEDC_CHANNEL_0 0
#define LEDC_CHANNEL_1 1
#define LEDC_CHANNEL_2 2

// use 13 bit precission for LEDC timer
#define LEDC_TIMER_13_BIT 13

// use 5000 Hz as a LEDC base frequency
#define LEDC_BASE_FREQ 5000

// fade LED PIN (replace with LED_BUILTIN constant for built-in LED)
#define LED_PIN_R 32
#define LED_PIN_G 33
#define LED_PIN_B 25

class RMTT_RGB
{
private:
    static void LEDCAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax);

public:
    RMTT_RGB(){};
    ~RMTT_RGB(){};
    static void Init();
    static void SetRed(uint32_t val, uint32_t valueMax = 255);
    static void SetBlue(uint32_t val, uint32_t valueMax = 255);
    static void SetGreen(uint32_t val, uint32_t valueMax = 255);
    static void SetRGB(uint32_t R, uint32_t G, uint32_t B, uint32_t valueMax = 255);
};
