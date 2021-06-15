/*
 * Copyright (C) 2020 DJI.
 *
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-06-15     robomaster   first version
 */

#pragma once
#include "stdint.h"

void io_init();
void io_start(uint8_t id);
void io_stop(uint8_t id);
void io_ctrl(int id, uint32_t time, uint8_t mode);
