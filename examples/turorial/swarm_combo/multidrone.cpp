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

#include <Arduino.h>
#include "stdio.h"
#include "stdint.h"
#include "stdio.h"
#include "RMTT_Libs.h"
#include "protocol.h"
#include <Wire.h>
#include "FS.h"
#include "SPIFFS.h"
#include "fileparser.h"
#include "multidrone.h"

static RMTT_Protocol protocol;

static struct server_status now_server_status = {0};
static int max_distance = 0;
static uint8_t trace_rsp = 0;

static void recv_callback(server_head_t *phead);

uint8_t get_local_id()
{
    return now_server_status.local_id;
}

void enter_auto_mode()
{
    while(!(protocol.getTelloMsgString("[TELLO] enterautomode",1000)==String("ETT ok"))){}
    callback_register(recv_callback);
}

void recv_callback_init()
{
    callback_register(recv_callback);
}

uint8_t enter_auto_mode(uint16_t timeout)
{
    return protocol.getTelloMsgString("[TELLO] enterautomode",timeout)==String("ETT ok");
}

uint8_t setyaw_buff[128] = {0};
uint8_t setyaw_len = 0;
void drone_takeoff(uint16_t drone_num)
{
    setyaw_len = snprintf((char*)setyaw_buff, sizeof(setyaw_buff), "setyaw %d m-1", get_init_yaw());
    for (int i = 0; i < MAX_DRONE_NUM; i++)
    {
        ctrl_msg_send(EXT_STRING_CMD, i, (uint8_t *)"EXT led 0 0 0", (uint16_t)strlen("EXT led 0 0 0"));
        ctrl_msg_send(EXT_STRING_CMD, i, (uint8_t *)"EXT led 0 0 0", (uint16_t)strlen("EXT led 0 0 0"));
        ctrl_msg_send(EXT_STRING_CMD, i, (uint8_t *)"EXT mled g 0", (uint16_t)strlen("EXT mled g 0"));
        ctrl_msg_send(EXT_STRING_CMD, i, (uint8_t *)"EXT mled g 0", (uint16_t)strlen("EXT mled g 0"));
        delay(10);
    }
    ctrl_msg_send(STRING_CMD, 0, (uint8_t *)"clean", (uint16_t)strlen("clean"));
    ctrl_msg_send(STRING_CMD, 0, (uint8_t *)"takeoff", (uint16_t)strlen("takeoff"));
    ctrl_msg_send(STRING_CMD, 0, (uint8_t *)"takeoff", (uint16_t)strlen("takeoff"));
    delay(8000);

    ctrl_msg_send(STRING_CMD, 0, (uint8_t *)"up 30", (uint16_t)strlen("up 30"));
    delay(3000);

    ctrl_msg_send(STRING_CMD, 0, (uint8_t *)"clean", (uint16_t)strlen("clean"));
    ctrl_msg_send(STRING_CMD, 0, (uint8_t *)"pattern 2", (uint16_t)strlen("pattern 2"));
    ctrl_msg_send(STRING_CMD, 0, (uint8_t *)"pattern 2", (uint16_t)strlen("pattern 2"));
    wait_rsp();

    wait_mid_valid(drone_num);

    ctrl_msg_send(STRING_CMD, 0, setyaw_buff, (uint16_t)setyaw_len);
    ctrl_msg_send(STRING_CMD, 0, setyaw_buff, (uint16_t)setyaw_len);
    delay(3500);
}

void drone_land()
{
    for (int i = 0; i < MAX_DRONE_NUM; i++)
    {
        ctrl_msg_send(EXT_STRING_CMD, i, (uint8_t *)"EXT led 0 0 0", (uint16_t)strlen("EXT led 0 0 0"));
        ctrl_msg_send(EXT_STRING_CMD, i, (uint8_t *)"EXT led 0 0 0", (uint16_t)strlen("EXT led 0 0 0"));
        ctrl_msg_send(EXT_STRING_CMD, i, (uint8_t *)"EXT mled g 0", (uint16_t)strlen("EXT mled g 0"));
        ctrl_msg_send(EXT_STRING_CMD, i, (uint8_t *)"EXT mled g 0", (uint16_t)strlen("EXT mled g 0"));
        ctrl_msg_send(EXT_STRING_CMD, i, (uint8_t *)"EXT reboot", (uint16_t)strlen("EXT reboot"));
        ctrl_msg_send(EXT_STRING_CMD, i, (uint8_t *)"EXT reboot", (uint16_t)strlen("EXT reboot"));
        delay(10);
    }
    ctrl_msg_send(STRING_CMD, 0, (uint8_t *)"motorready", (uint16_t)strlen("motorready"));
    ctrl_msg_send(STRING_CMD, 0, (uint8_t *)"motorready", (uint16_t)strlen("motorready"));

    ctrl_msg_send(STRING_CMD, 0, (uint8_t *)"land", (uint16_t)strlen("land"));
    ctrl_msg_send(STRING_CMD, 0, (uint8_t *)"land", (uint16_t)strlen("land"));
}

void drone_force_land()
{
    for (int i = 0; i < 10; i++)
    {
        protocol.getTelloMsgString("[TELLO] enterautomode",10);
        delay(20);
        ctrl_msg_send(STRING_CMD, 0, (uint8_t *)"command", (uint16_t)strlen("command"));
        delay(20);
        ctrl_msg_send(STRING_CMD, 0, (uint8_t *)"land", (uint16_t)strlen("land"));
        delay(20);
    }
}

void drone_pos_init(uint8_t *buff, uint8_t drone_num)
{
    /** start */
    trace_rsp = 0;
    ctrl_msg_send(KEY_FRAME_CMD, drone_num, (uint8_t *)buff, sizeof(ctrl_msg_t) * drone_num);

    while(1)
    {
        if (trace_rsp)
        {
            break;
        }
        delay(100);
    }

    while (max_distance > 25)
    {
        trace_rsp = 0;
        ctrl_msg_send(NORMAL_FRAME_CMD, drone_num, (uint8_t *)buff, sizeof(ctrl_msg_t) * drone_num);
        delay(1000);
        ctrl_msg_send(GET_MAX_DISTANCE_CMD, drone_num, 0,0);
        delay(100);
    }
}


/**
 * @brief receive uart data in auto mode.
 *
 * @param phead
 */
void recv_callback(server_head_t *phead)
{
    // Serial.println("recv");
    if (phead->cmd == RESPONSE_CMD)
    {
        max_distance = *(int*)phead->pdata;
        trace_rsp = 1;
        Serial.printf("max distance %d\r\n", max_distance);
    }
    else if (phead->cmd == GET_MAX_DISTANCE_CMD)
    {
        max_distance = *(int*)phead->pdata;
        Serial.printf("max distance %d\r\n", max_distance);
    }
    else if (phead->cmd == STATUS_CMD)
    {
        memcpy(&now_server_status, phead->pdata, sizeof(now_server_status));
        // Serial.printf("alive %d has_rsp %d rsp_ok %d mid_valid %d local_id %d\r\n", now_server_status.alive,  now_server_status.has_rsp,  now_server_status.rsp_ok,  now_server_status.mid_valid,  now_server_status.local_id);
    }
}

static uint8_t uart1_rx_buff[2048];
static uint16_t uart1_rx_index;
void auto_mode_recv()
{
    uart1_rx_index = 0;
    /* Serial receive process from drone */
    while (Serial1.available())
    {
        uart1_rx_buff[uart1_rx_index++] = Serial1.read();
    }

    if (uart1_rx_index)
    {
        // Serial.printf("%x %x\r\n", uart1_rx_buff[0], uart1_rx_buff[1]);
        ctrl_package_unpack(uart1_rx_buff, uart1_rx_index);
    }
}

int search()
{
    ctrl_msg_send(STRING_CMD, 0, (uint8_t *)"clean", (uint16_t)strlen("clean"));
    ctrl_msg_send(STRING_CMD, 0, (uint8_t *)"command", (uint16_t)strlen("command"));
    ctrl_msg_send(STRING_CMD, 0, (uint8_t *)"command", (uint16_t)strlen("command"));
    ctrl_msg_send(STRING_CMD, 0, (uint8_t *)"command", (uint16_t)strlen("command"));

    delay(300);

    ctrl_msg_send(STRING_CMD, 0, (uint8_t *)"tcpstatus?", (uint16_t)strlen("tcpstatus?"));
    delay(100);

    return now_server_status.rsp_ok;
}

int server_status_clean(void)
{
    memset(&now_server_status, 0, sizeof(now_server_status));
}

void set_freq()
{
    for(int i = 0; i < 5; i++)
    {
        ctrl_msg_send(STRING_CMD, 0, (uint8_t *)"statusfrq 4", (uint16_t)strlen("statusfrq 4"));
        delay(80);
    }
}

int wait_rsp(void)
{
    while(1)
    {
        memset(&now_server_status, 0, sizeof(now_server_status));

        ctrl_msg_send(STRING_CMD, 0, (uint8_t *)"tcpstatus?", (uint16_t)strlen("tcpstatus?"));
        delay(1000);

        if ((now_server_status.alive == now_server_status.has_rsp))
        {
            return now_server_status.alive;
        }

        delay(100);
    }
}

int wait_mid_valid(int num)
{
    uint32_t start = millis();
    while(millis() - start < 6000)
    {
        memset(&now_server_status, 0, sizeof(now_server_status));

        ctrl_msg_send(STRING_CMD, 0, (uint8_t *)"tcpstatus?", (uint16_t)strlen("tcpstatus?"));
        delay(200);

        if (now_server_status.mid_valid == num)
        {
            return num;
        }

        delay(200);
    }
}
