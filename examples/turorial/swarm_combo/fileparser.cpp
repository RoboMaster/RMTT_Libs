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
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "stddef.h"
#include "FS.h"
#include "SPIFFS.h"
#include <RMTT_Libs.h>
#include "fileparser.h"

uint8_t TTformationid = 0;
uint8_t last_TTformationid = 0;
uint8_t max_drone_num = 0;
uint16_t init_yaw = 0;
static uint8_t TTformation[5] = {0};
static uint8_t TTfile_num = 0;

/**
 * @brief Get the formation file num object.
 *        file name must be "TTformation"+"0"~"4"
 *
 * @return int (value < 5)
 */
int get_formation_file_num()
{
    String name = String("/TTformation");
    TTfile_num = 0;
    for (int i = 0; i < 5; i++)
    {
        DEBUG("name %s\r\n", (name + String(i) + String(".bin")).c_str());
        if (SPIFFS.exists(name + String(i) + String(".bin")))
        {
            TTformation[i] = 1;
            TTfile_num++;
        }
    }
    DEBUG("TTfile_num %d\r\n", TTfile_num);
    return TTfile_num;
}

uint8_t get_formation_id()
{
    return TTformationid;
}

void set_formation_id(uint8_t id)
{
    TTformationid = id;
}

uint8_t get_max_drone_num()
{
    return max_drone_num;
}

uint16_t get_init_yaw()
{
    return init_yaw;
}

String get_formation_name()
{
    String name = String("/TTformation");
    return name + String(TTformationid) + String(".bin");
}

String get_next_formation()
{
    String name = String("/TTformation");
    for (int i = TTformationid + 1; i < 5; i++)
    {
        if (TTformation[i] == 1)
        {
            TTformationid = i;
            return name + String(i) + String(".bin");
        }
    }

    for (int i = 0; i <= TTformationid; i++)
    {
        if (TTformation[i] == 1)
        {
            TTformationid = i;
            return name + String(i) + String(".bin");
        }
    }
    return String("");
}

bool check_file_head(String name)
{
    DEBUG("check file %s\r\n", name.c_str());
    if (SPIFFS.exists(name))
    {
        File file = SPIFFS.open(name, FILE_READ);
        if (file)
        {
            file_head_t head;
            if (file.read((uint8_t*)&head, sizeof(file_head_t)) == sizeof(file_head_t))
            {
                DEBUG("head magic %08x version %d drone %d size %d file.size() %d\r\n", head.magic, head.version, head.drone_num, head.size, file.size()) ;
                if ((head.magic == MAGIC_NUMBER)
                 &&(head.version == FILE_VERSION)
                 &&(head.size == file.size()))
                {
                    if (head.drone_num <= 10)
                    {
                        max_drone_num  = head.drone_num;
                        init_yaw = head.init_yaw;
                        return true;
                    }
                }
            }
        }
        file.close();
    }
    return false;
}

/**
 * @brief read one line from a opened file.
 *
 * @param file
 * @param buff
 * @param len
 * @return int
 */
int read_data_frame(File& file, uint8_t *buff, uint16_t len)
{
    uint16_t index = 0;
    uint8_t step = 0;
    data_frame_t* head = (data_frame_t*)buff;
    if (file)
    {
        if (file.available())
        {

            if(file.read(buff, sizeof(data_frame_t)) == sizeof(data_frame_t))
            {
                // DEBUG("head type %d length %d\r\n", head->type, head->length);
                if (head->length + sizeof(data_frame_t) <= len)
                {
                    if(file.read(buff + sizeof(data_frame_t), head->length) == head->length)
                    {
                        return head->length;
                    }
                    else
                    {
                        return FILE_READ_ERROR;
                    }

                }
                else
                {
                    file.seek(file.position() - sizeof(data_frame_t));
                    return READ_BUFF_OVER;
                }
            }
            else
            {
                return FILE_READ_ERROR;
            }
        }
        else
        {
            return FILE_END;
        }
    }
    return FILE_NO_EXIST;
}

void cfg_automode_enable()
{
    if(SPIFFS.exists("/multidronemode.cfg"))
    {
        cfg_head_t cfg = {0};
        File file = SPIFFS.open("/multidronemode.cfg", FILE_READ);
        file.read((uint8_t*)&cfg, sizeof(cfg));
        file.close();
        cfg.mode = FORMATION_MODE;
        file = SPIFFS.open("/multidronemode.cfg", FILE_WRITE);
        DEBUG("mode %d id %d", cfg.mode, cfg.id);
        file.write((uint8_t*)&cfg, sizeof(cfg));
        file.close();

    }
    else
    {
        cfg_head_t cfg = {FORMATION_MODE, 0xFF};
        DEBUG("mode %d id %d", cfg.mode, cfg.id);
        File file = SPIFFS.open("/multidronemode.cfg", FILE_WRITE);
        file.write((uint8_t*)&cfg, sizeof(cfg));
        file.close();
    }
    delay(100);
    ESP.restart();
    return;
}

void save_file_id(uint8_t id)
{
    cfg_head_t cfg = {0};
    File file = SPIFFS.open("/multidronemode.cfg", FILE_READ);
    file.read((uint8_t*)&cfg, sizeof(cfg));
    file.close();
    cfg.id = id;
    DEBUG("cfg mode %d id %d\r\n", cfg.mode, cfg.id);
    file = SPIFFS.open("/multidronemode.cfg", FILE_WRITE);
    file.write((uint8_t*)&cfg, sizeof(cfg));
    file.close();
}

uint8_t is_multidrone_mode()
{
    cfg_head_t cfg = {0};
    uint8_t mode = 0;
    if(SPIFFS.exists("/multidronemode.cfg"))
    {
        File file = SPIFFS.open("/multidronemode.cfg", FILE_READ);
        file.read((uint8_t*)&cfg, sizeof(cfg));
        file.close();
        DEBUG("read cfg mode %d id %d\r\n", cfg.mode, cfg.id);
        mode = cfg.mode;
        if (cfg.id != 0xFF)
        {
            TTformationid = cfg.id;
        }
        else
        {
            for (int i = 0; i < 5; i++)
            {
                if (TTformation[i] == 1)
                {
                    TTformationid = i;
                    break;
                }
            }
        }
        cfg.mode = NORMAL_MODE;
        file = SPIFFS.open("/multidronemode.cfg", FILE_WRITE);
        file.write((uint8_t*)&cfg, sizeof(cfg));
        file.close();
        DEBUG("now mode %d id %d\r\n", cfg.mode, cfg.id);
        DEBUG("cfg file exists\r\n");
    }
    else
    {
        DEBUG("NO cfg file\r\n");
    }

    return mode;
}
