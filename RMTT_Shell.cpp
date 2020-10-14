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
#include "stdint.h"
#include "string.h"
#include "RMTT_Shell.h"

#define RECEIVE_STEP 0
#define LF_STEP      1

uint8_t unpack_step = 0;

uint8_t cmd_buff[256] = {0};
uint8_t cmd_buff_index = 0;

cmd_handle_cb unknown_handle = NULL;

struct sdk_cmd local_cmd[MAX_CMD_NUM] = {0};
static uint8_t cb_num = 0;

char graph_param[GRAPH_PARAM_LEN] = {0};
char muti_param[STR_PARAM_NUM][STR_PARAM_LEN] = {0};
char *p_muti_param[STR_PARAM_NUM] = {0};

int shell_cmd_init()
{
    for(int i = 0; i <  STR_PARAM_NUM; i++)
    {
        p_muti_param[i] = (char *)muti_param[i];
    }
}

static int cal_rotatinghash(char *key, int length);

int cmd_register(char* cmd, cmd_handle_cb cb)
{
    uint8_t str_len = 0;
    str_len = strlen(cmd);

    local_cmd[cb_num].name_len = str_len > 31 ? 31:str_len;
    memcpy((char *)local_cmd[cb_num].name, cmd, local_cmd[cb_num].name_len);
    local_cmd[cb_num].name[str_len] = '\0';
    local_cmd[cb_num].hash = cal_rotatinghash((char*)local_cmd[cb_num].name, local_cmd[cb_num].name_len);
    local_cmd[cb_num].cb = cb;
    cb_num++;
    return 0;
}

int cmd_unknown_handle_register(cmd_handle_cb cb)
{
    if (cb)
    {
        unknown_handle = cb;
    }
    return 0;
}

int cmd_handler(uint8_t *buff, uint16_t len)
{
    int param_cnt = 0;
    char delims[] = " ";
    char *result = NULL;
    char *str = (char*)buff;

    memset(muti_param, 0, STR_PARAM_NUM * STR_PARAM_LEN);
    memset(graph_param, 0, GRAPH_PARAM_LEN);

    result = strtok(str, delims);
    while (result != NULL && param_cnt < (STR_PARAM_NUM))
    {
        int param_len =  strlen(result);
        if(param_len < STR_PARAM_LEN)
        {
            memcpy(muti_param[param_cnt], result, param_len);
            muti_param[param_cnt][param_len] = '\0';
            param_cnt++;
        }
        else
        {
            if (param_len > GRAPH_PARAM_LEN - 1)
            {
                param_len = GRAPH_PARAM_LEN - 1;
            }

            memcpy(graph_param, result, param_len);
            graph_param[param_len] = '\0';
        }

        result = strtok(NULL, delims);
    }

    if(param_cnt != 0)
    {
        int hash = cal_rotatinghash(muti_param[0], strlen(muti_param[0]));
        for(int i = 0; i < cb_num; i++ )
        {
            if (hash == local_cmd[i].hash)
            {
                local_cmd[i].cb(param_cnt, p_muti_param, graph_param);
                return 0;
            }
        }
    }
    /* no command */
    if (unknown_handle)
    {
        unknown_handle(param_cnt, p_muti_param, graph_param);
    }
    return -1;
}

int cmd_process(uint8_t val)
{
    int error = 0;
    switch(unpack_step)
    {
    case RECEIVE_STEP:
    {
        if (val == '\r')
        {
            unpack_step = LF_STEP;
        }
        else if (val == '\n')
        {
            cmd_buff_index = 0;
        }
        else
        {
            if(cmd_buff_index > sizeof(cmd_buff))
            {
                goto end;
            }
            cmd_buff[cmd_buff_index++] = val;
        }
        break;
    }
    case LF_STEP:
    {
        if (val == '\n')
        {
            if(cmd_handler(cmd_buff, cmd_buff_index) != 0)
            {
                error = -2;
            }
            unpack_step = RECEIVE_STEP;
            cmd_buff_index = 0;
            memset(cmd_buff, 0, sizeof(cmd_buff));
        }
        else
        {
            goto end;
        }
        break;
    }
    default:
        break;
    }
    return error;
end:
    memset(cmd_buff, 0, sizeof(cmd_buff));
    unpack_step = RECEIVE_STEP;
    cmd_buff_index = 0;
    return -1;
}

static int cal_rotatinghash(char *key, int length)
{
    int hash = 0;
    int h = hash;     // hash defalult value is0
    int len = length;
    if (h == 0 && len > 0)
    {
        int off = 0;
        char *val = key;
        for (int i = 0; i < len; i++)
        {
            h = 31 * h + val[off++];
        }
        hash = h;
    }
    return h;
}




