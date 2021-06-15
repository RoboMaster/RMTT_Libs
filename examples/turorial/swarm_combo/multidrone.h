/*
 * Copyright (C) 2020 DJI.
 *
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-08-25     robomaster   first version
 * 2021-06-15     robomaster   support TT swarm combo
 */

#pragma once

#define MAX_DRONE_NUM 36

void set_freq();
int search();
int wait_rsp();
int wait_mid_valid(int num);
int server_status_clean(void);

void auto_mode_recv();
void enter_auto_mode();
void drone_takeoff(uint16_t drone_num);
void drone_land();
void drone_pos_init(uint8_t *buff, uint8_t drone_num);

uint8_t get_local_id();

void recv_callback_init();
uint8_t enter_auto_mode(uint16_t timeout);

void drone_force_land();
