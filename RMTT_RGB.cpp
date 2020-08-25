/*
 * Copyright (C) 2020 DJI.
 *
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-08-25     robomaster   first version
 */

#include "Arduino.h"
#include "RMTT_RGB.h"

// #define __RMTT_RGB_DEBUG__

void RMTT_RGB::Init(void)
{
    // Setup timer and attach timer to a led pin
    ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
    ledcSetup(LEDC_CHANNEL_1, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
    ledcSetup(LEDC_CHANNEL_2, LEDC_BASE_FREQ, LEDC_TIMER_13_BIT);
    ledcAttachPin(LED_PIN_R, LEDC_CHANNEL_0);
    ledcAttachPin(LED_PIN_G, LEDC_CHANNEL_1);
    ledcAttachPin(LED_PIN_B, LEDC_CHANNEL_2);

#ifdef __RMTT_RGB_DEBUG__
    Serial.println("LEDC Init Successful!");
#endif
}

void RMTT_RGB::SetRed(uint32_t val, uint32_t valueMax)
{
    LEDCAnalogWrite(LEDC_CHANNEL_0, val, valueMax);
    LEDCAnalogWrite(LEDC_CHANNEL_1, 0, 255);
    LEDCAnalogWrite(LEDC_CHANNEL_2, 0, 255);
}

void RMTT_RGB::SetGreen(uint32_t val, uint32_t valueMax)
{
    LEDCAnalogWrite(LEDC_CHANNEL_1, val, valueMax);
    LEDCAnalogWrite(LEDC_CHANNEL_0, 0, 255);
    LEDCAnalogWrite(LEDC_CHANNEL_2, 0, 255);
}

void RMTT_RGB::SetBlue(uint32_t val, uint32_t valueMax)
{
    LEDCAnalogWrite(LEDC_CHANNEL_2, val, valueMax);
    LEDCAnalogWrite(LEDC_CHANNEL_0, 0, 255);
    LEDCAnalogWrite(LEDC_CHANNEL_1, 0, 255);
}

void RMTT_RGB::SetRGB(uint32_t R, uint32_t G, uint32_t B, uint32_t valueMax)
{
    LEDCAnalogWrite(LEDC_CHANNEL_0, R, valueMax);
    LEDCAnalogWrite(LEDC_CHANNEL_1, G, valueMax);
    LEDCAnalogWrite(LEDC_CHANNEL_2, B, valueMax);
}

// Arduino like analogWrite
// value has to be between 0 and valueMax
void RMTT_RGB::LEDCAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax)
{
    // calculate duty, 8191 from 2 ^ 13 - 1
    uint32_t duty = (8191 / valueMax) * min(value, valueMax);

    // write duty to LEDC
    ledcWrite(channel, duty);
}
