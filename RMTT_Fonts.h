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
#include "stdlib.h"
#include "string.h"

int rbpstr2buff(uint8_t buff[128], char str[], uint8_t bright);
int mled_font2buff(uint8_t buff[128], int c, char color, uint8_t bright);
int font8x8_convert(uint8_t buff[128], const char font[8], char color, uint8_t bright);

void string_move_effect2buff(uint8_t buff1[128], uint8_t buff2[128], uint8_t output[128], char mode, uint8_t index);
void graph_move_effect2buff(uint8_t graph[128], uint8_t output[128], char mode, uint8_t index);