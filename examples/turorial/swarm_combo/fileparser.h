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

#include "stdint.h"
#include "FS.h"
#include "SPIFFS.h"

#define TYPE_STRING_CMD_FOR_ALL 0
#define TYPE_KEY_FRAME          1
#define TYPE_NORMAL_FRAME       2
#define TYPE_EXT_STRING_CMD     3
#define TYPE_EXTEND_FRAME       8
#define TYPE_EXTEND_KEY_FRAME   9

#define MAGIC_NUMBER 0xfaccddbd
#define FILE_VERSION 0

#define NORMAL_MODE 0
#define FORMATION_MODE 1

#define __DEFAULT_LOG__

#ifdef __DEFAULT_LOG__
#define DEBUG(...) Serial.printf(__VA_ARGS__)
#else
#define DEBUG(...)
#endif

enum ParserErr
{
    FILE_NO_EXIST = -1,
    FILE_END = -2,
    FILE_READ_ERROR = -3,
    READ_BUFF_OVER = -4,
};

#pragma pack(push,1)

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	int16_t yaw;
} ctrl_msg_t;

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	int16_t yaw;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t mled_color;
    uint8_t mled_value;
} ctrl_ext_msg_t;

typedef struct
{
    uint32_t magic;
    uint8_t version;
    uint32_t size;
    uint8_t drone_num;
    uint16_t init_yaw;
    uint8_t res[3];
} file_head_t;

typedef struct
{
    uint8_t mode;
    uint8_t id;
} cfg_head_t;

typedef struct
{
    uint8_t type;
    uint16_t length;
    uint16_t time;
    uint8_t id_num;
    uint8_t pdata[];
} data_frame_t;

#pragma pack(pop)

uint16_t get_init_yaw();
uint8_t get_max_drone_num();
int get_formation_file_num();
uint8_t get_formation_id();
void set_formation_id(uint8_t id);
String get_formation_name();
String get_next_formation();
bool check_file_head(String name);
int read_data_frame(File& file, uint8_t *buff, uint16_t len);
void cfg_automode_enable();
void save_file_id(uint8_t id);
uint8_t is_multidrone_mode();
