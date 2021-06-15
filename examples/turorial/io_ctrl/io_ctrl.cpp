/*
 * Copyright (C) 2020 DJI.
 *
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-06-15     robomaster   first version
 */

#include "Arduino.h"

uint8_t io_map[][2] = {{2,12},{5,21},{4,15}};
uint32_t io_time_cur[3] = {0};
uint32_t io_time_end[3] = {0};

uint8_t io_mode[3] = {0};

TaskHandle_t ioTaskHandle = NULL;

void io_task(void *arg);

void io_init()
{
    for (int i = 0; i < sizeof(io_map);i++)
    {
        pinMode((((uint8_t*)io_map)[i]), OUTPUT|PULLDOWN);
        digitalWrite(((uint8_t*)io_map)[i], LOW);
    }
    xTaskCreateUniversal(io_task, "io_task", 8192, NULL, 1, &ioTaskHandle, 1);
}

void io_start(uint8_t id)
{
    if (id > 2) return;
    switch (io_mode[id])
    {
    case 0:
        digitalWrite(io_map[id][0], LOW);
        digitalWrite(io_map[id][1], HIGH);
    break;
    case 1:
        digitalWrite(io_map[id][0], HIGH);
        digitalWrite(io_map[id][1], LOW);
    break;
    default:
        return;
    break;
    }
}

void io_stop(uint8_t id)
{
    if (id > 2) return;
    digitalWrite(io_map[id][0], LOW);
    digitalWrite(io_map[id][1], LOW);
}

void io_ctrl(int id, uint32_t time, uint8_t mode)
{
    if (id > 2) return;
    io_mode[id] = mode;
    io_time_end[id] = io_time_cur[id] + time;
    Serial.printf("ioctrl %d %d\r\n", io_time_end[id], io_time_cur[id]);
}


void io_task(void *arg)
{
    for (int i = 0; i < 3; i++)
    {
        io_time_cur[i] = millis();
        io_time_end[i] = millis();
    }
    while (1)
    {
        for (int i = 0; i < 3; i++)
        {
            io_time_cur[i] = millis();
            if (io_time_end[i] > io_time_cur[i])
            {
                io_start(i);
            }
            else
            {
                io_stop(i);
            }
        }
        delay(1);
    }
}