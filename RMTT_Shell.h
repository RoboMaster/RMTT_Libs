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

#define STR_PARAM_LEN 40
#define STR_PARAM_NUM 15

/* param length <= 64 */
#define GRAPH_PARAM_LEN 70

#define MAX_CMD_NUM  32

/* use single graph buff(argv2) to reduce memory usage */
typedef int (*cmd_handle_cb)(int argc, char *argv[], char argv2[]);

struct sdk_cmd
{
    char name[32];
    uint8_t name_len;
    int hash;
    cmd_handle_cb cb;
};

int shell_cmd_init();
int cmd_register(char* cmd, cmd_handle_cb cb);
int cmd_unknown_handle_register(cmd_handle_cb cb);
int cmd_process(uint8_t val);



