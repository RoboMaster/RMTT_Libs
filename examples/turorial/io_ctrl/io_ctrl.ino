/*
 * Copyright (C) 2020 DJI.
 *
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-06-15     robomaster   first version
 */

#include <Arduino.h>
#include <RMTT_Libs.h>
#include <RMTT_Shell.h>
#include <Wire.h>
#include <stdio.h>
#include <string.h>
#include "MS5837.h"
#include "io_ctrl.h"

// #define DEBUG

#ifdef DEBUG
#define CommonSerial Serial
#else
#define CommonSerial Serial1
#endif

#define SDK_VERSION "esp32v2.0.0.5"

/* led display mode */
#define LED_FACTORY 0
#define LED_CONST 1
#define LED_BLINK 2
#define LED_BREATH 3

#define BLINK_UNIT 200
#define BREATH_UNIT 20

int ext_cmd_callback(int argc, char *argv[], char argv2[]);
int led_callback(int argc, char *argv[], char argv2[]);
void led_task(void *pParam);

RMTT_RGB tt_rgb;
MS5837 sensor;

TaskHandle_t userTaskHandle = NULL;

uint8_t sensor_failed = 0;
float sensor_depth = 0;
float sensor_length = 0;

void setup()
{
    Serial.begin(115200);
    Wire.begin(27, 14);
    Wire.setClock(400000);
    Serial1.begin(1000000, SERIAL_8N1, 23, 18);
    Serial.println();

    Serial.println("*********RoboMaster Example********");
    Serial.println(SDK_VERSION);
    Serial.println();

    RMTT_RGB::Init();
    io_init();
    shell_cmd_init();

    xTaskCreateUniversal(user_task, "user_task", 8192, NULL, 1, &userTaskHandle, 0);

    cmd_register((char *)"led", led_callback);
    cmd_register((char *)"cmd", ext_cmd_callback);

    led_effect_init();
}

void user_task(void *arg)
{
    // Initialize pressure sensor
    // Returns true if initialization was successful
    // We can't continue with the rest of the program unless we can initialize the sensor
    if (!sensor.init())
    {
        Serial.println("Init failed!");
        Serial.println("Are SDA/SCL connected correctly?");
        Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
        Serial.println("\n\n\n");
        tt_rgb.SetRGB(255, 0, 0);
        sensor_failed = 1;
    }
    else
    {
        tt_rgb.SetRGB(0, 255, 0);
        Serial.println("Init Succeed!");
        // .init sets the sensor model for us but we can override it if required.
        // Uncomment the next line to force the sensor model to the MS5837_30BA.
        sensor.setModel(MS5837::MS5837_30BA);

        sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
    }

    pinMode(26,INPUT);

    while (1)
    {
        // Update pressure and temperature readings
        sensor_length = analogRead(26) / 4095.0f * 125;
        if (sensor_failed == 0)
        {
            sensor.read();
            sensor_depth = sensor.depth();
            // Serial1.print(sensor_depth);
        }
        // Serial.println(sensor_length);
        CommonSerial.printf("len%3.2fdep%5.2f",sensor_length,sensor_depth);
        delay(50);
    }
}

void loop()
{
    /* Serial receive process from drone */
    while (CommonSerial.available())
    {
        int ret = cmd_process(CommonSerial.read());
        if (ret != 0)
        {
            CommonSerial.printf("command error: %d\r\n", ret);
        }
    }
    /* -------!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!-------  */
    /* DO NOT ADD ANY CODE HERE FOR NOT BLOCKING THE RECEIVE FROM THE SERIAL */
    /*     YOU CAN ADD YOUR USER CODE TO THE 'user_task' FUNCTION ABOVE      */
    /* -------!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!-------- */
}

int ext_cmd_callback(int argc, char *argv[], char argv2[])
{
    uint32_t id = 0, mode = 0;
    uint32_t time = 0;
    if (sscanf(argv[1], "%d", &id) &&
        sscanf(argv[2], "%d", &mode) &&
        sscanf(argv[3], "%d", &time))
    {
        io_ctrl(id, time, mode);
    }
    // Serial.printf("cmd %s %s\r\n", argv[1], argv[2]);
    return 0;
}

/**
 * Led command handling, control the color of the LED by input commands
 *
 *  @param argc Control argument
 *  @param argv[] Value argument 1
 *  @param argv2[] Value argument2
 */
int led_callback(int argc, char *argv[], char argv2[])
{
    uint32_t r1, b1, g1, r2, b2, g2;
    if (!strcmp(argv[1], "bl"))
    {
        uint8_t blink_time = 1;
        if ((argc == 9) && sscanf(argv[2], "%d", &blink_time) &&
            sscanf(argv[3], "%d", &r1) && sscanf(argv[4], "%d", &g1) &&
            sscanf(argv[5], "%d", &b1) && sscanf(argv[6], "%d", &r2) &&
            sscanf(argv[7], "%d", &g2) && sscanf(argv[8], "%d", &b2))
        {
            if ((blink_time >= 1) && (blink_time <= 10))
            {
                blink_time = (11 - blink_time) * BLINK_UNIT;
                led_effect_blink(r1, g1, b1, r2, g2, b2, blink_time);
                CommonSerial.print("led ok");
            }
            else
            {
                goto end;
            }
        }
        else
        {
            goto end;
        }
    }
    else if (!strcmp(argv[1], "br"))
    {
        int breath_time = 6;
        if ((argc == 6) && sscanf(argv[2], "%d", &breath_time) &&
            sscanf(argv[3], "%d", &r1) && sscanf(argv[4], "%d", &g1) &&
            sscanf(argv[5], "%d", &b1))
        {
            if ((breath_time >= 1) && (breath_time <= 10))
            {
                breath_time = (11 - breath_time) * BREATH_UNIT;
                led_effect_breath(r1, g1, b1, breath_time);
                CommonSerial.print("led ok");
            }
            else
            {
                goto end;
            }
        }
        else
        {
            goto end;
        }
    }
    else if (argc == 4)
    {
        if (sscanf(argv[1], "%d", &r1) && sscanf(argv[2], "%d", &g1) &&
            sscanf(argv[3], "%d", &b1))
        {
            led_effect_set_rgb(r1, g1, b1);
            CommonSerial.print("led ok");
        }
        else
        {
            goto end;
        }
    }
    else
    {
        goto end;
    }

    return 0;
end:
    CommonSerial.print("led error");
    return 0;
}
