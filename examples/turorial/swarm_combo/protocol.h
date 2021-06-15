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

#include <Arduino.h>
#include "stdint.h"

#define STRING_CMD           0
#define KEY_FRAME_CMD        1
#define NORMAL_FRAME_CMD     2
#define RESPONSE_CMD         3
#define EXT_STRING_CMD       4
#define STATUS_CMD           5
#define ID_STRING_CMD        6
#define GET_MAX_DISTANCE_CMD 7

#define SOF_FRAME_LENGTH (sizeof(server_head_t))

#pragma pack(push,1)

typedef struct
{
	uint8_t sof;
	uint8_t cmd;
	union
	{
		uint8_t drone_num;
		uint8_t id;
	};
    uint8_t udp_mode;
	uint8_t res[4];
	uint16_t length;
	uint8_t pdata[];
} server_head_t;

typedef struct
{
	uint16_t sum;
} server_tail_t;

struct server_status
{
    uint8_t alive;
    uint8_t has_rsp;
    uint8_t rsp_ok;
    uint8_t mid_valid;
    uint8_t local_id;
};

#pragma pack(pop)

typedef void (*recv_callback_t)(server_head_t *phead);

void callback_register(recv_callback_t callback);
uint16_t ctrl_msg_send(uint8_t cmd, uint8_t id, uint8_t *pdata, uint16_t len);
uint16_t ctrl_package_unpack(uint8_t *pdata, uint16_t len);
uint16_t ctrl_package_pack(uint8_t *buff, uint8_t num, uint8_t cmd, uint8_t *pdata, uint16_t len, uint8_t mode);
