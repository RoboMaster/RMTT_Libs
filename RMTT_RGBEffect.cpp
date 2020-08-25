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

static uint8_t led_effect_mode = LED_EFFECT_FACTORY_MODE;
static uint8_t led_effect_next_mode = LED_EFFECT_FACTORY_MODE;

static uint32_t led_r1 = 0, led_b1 = 0, led_g1 = 0;
static uint32_t led_r2 = 0, led_b2 = 0, led_g2 = 0;

/* const param */
static uint8_t  const_flag = 0;
static uint32_t const_r = 0, const_b = 0, const_g = 0;

/* blink param */
static uint32_t blink_time = BLINK_MIN_TIME;

/* breath param */
static uint32_t breath_toggle = 0;
static uint32_t breath_cnt = 0;
static uint32_t breath_time = BREATH_MIN_TIME;

TaskHandle_t ledEffectTaskHandle = NULL;

uint8_t get_led_effect_mode(void)
{
    return led_effect_mode;
}

void led_effect_init(void)
{
    xTaskCreateUniversal(led_effect_task, "led_effect_task", 4096, NULL, 4, &ledEffectTaskHandle, 1);
}

void led_effect_deinit(void)
{
    led_effect_mode = LED_EFFECT_FACTORY_MODE;
    led_effect_next_mode = LED_EFFECT_FACTORY_MODE;
}

void led_effect_blink(uint8_t r1, uint8_t g1, uint8_t b1,
                      uint8_t r2, uint8_t g2, uint8_t b2, float freq)
{
    led_r1 = r1;
    led_g1 = g1;
    led_b1 = b1;
    led_r2 = r2;
    led_g2 = g2;
    led_b2 = b2;
    if (freq != 0)
    {
        blink_time = max(1/freq * 1000 / 2, (float)BLINK_MIN_TIME);
    }
    led_effect_next_mode = LED_EFFECT_BLINK_MODE;
}

void led_effect_breath(uint8_t r, uint8_t g, uint8_t b, float freq)
{
    breath_cnt = 0;
    breath_toggle = 0;
    if (freq != 0)
    {
        breath_time = max(1/freq * 1000 / 40, (float)BREATH_MIN_TIME);
    }
    led_r1 = r;
    led_g1 = g;
    led_b1 = b;
    led_effect_next_mode = LED_EFFECT_BREATH_MODE;
}

void led_effect_set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    const_flag = 1;
    const_r = r;
    const_g = g;
    const_b = b;
    led_effect_next_mode = LED_EFFECT_CONST_MODE;
}

void led_effect_task(void *param)
{
    while (1)
    {
        led_effect_mode = led_effect_next_mode;
        switch (led_effect_mode)
        {
        case LED_EFFECT_CONST_MODE:
        {
            if (const_flag == 1)
            {
                const_flag = 0;
                RMTT_RGB::SetRGB(const_r, const_g, const_b);
            }
            delay(50);
            break;
        }
        case LED_EFFECT_BLINK_MODE:
        {
            RMTT_RGB::SetRGB(led_r1, led_g1, led_b1);
            delay(blink_time);
            RMTT_RGB::SetRGB(led_r2, led_g2, led_b2);
            delay(blink_time);
            break;
        }
        case LED_EFFECT_BREATH_MODE:
        {
            if (!breath_toggle)
            {
                breath_cnt += 5;
            }
            else
            {
                breath_cnt -= 5;
            }

            if ((breath_cnt == 100) || (breath_cnt == 0))
            {
                breath_toggle = ~breath_toggle;
            }

            RMTT_RGB::SetRGB(led_r1 * breath_cnt / 100.0, led_g1 * breath_cnt / 100.0, led_b1 * breath_cnt / 100.0);
            delay(breath_time);
            break;
        }
        case LED_EFFECT_FACTORY_MODE:
        {
            delay(100);
            break;
        }
        default:
        {
            delay(100);
        }
        break;
        }
    }
}
