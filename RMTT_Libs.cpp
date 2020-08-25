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

uint8_t sem_init_f = 0;

SemaphoreHandle_t rmttLibsI2cSemaphore = NULL;




