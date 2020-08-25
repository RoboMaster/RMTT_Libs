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

#include <RMTT_RGB.h>
#include <RMTT_TOF.h>
#include <RMTT_Matrix.h>

#include <RMTT_Fonts.h>
#include <RMTT_Protocol.h>
#include <RMTT_Shell.h>
#include <RMTT_GamesirT1d.h>

#include <RMTT_Effect.h>

extern const char matrix_fonts[][8];

#define RMTT_KEY_PIN     34

#define RMTT_SERIAL1_TX  18
#define RMTT_SERIAL1_RX  23

#define RMTT_I2C_SDA     27
#define RMTT_I2C_SCL     26

extern uint8_t sem_init_f;
extern SemaphoreHandle_t rmttLibsI2cSemaphore;

#define RMTT_I2C_BUSY_LOCK() \
    do { \
        if (!sem_init_f) \
        { \
            sem_init_f = 1; \
            rmttLibsI2cSemaphore = xSemaphoreCreateCounting(1,1); \
        } \
        xSemaphoreTake(rmttLibsI2cSemaphore, ( TickType_t )0xFFFFFFFF); \
    }while(0)

#define RMTT_I2C_BUSY_UNLOCK() \
    do { \
        xSemaphoreGive(rmttLibsI2cSemaphore); \
    }while(0)

class Drone
{
public:
    Drone();
    ~Drone(){};
    RMTT_RGB RGB;
    RMTT_TOF TOF;
    RMTT_Matrix Matrix;
    void Init();
};


