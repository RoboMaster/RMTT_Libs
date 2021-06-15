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
#include <Wire.h>
#include "FS.h"
#include "SPIFFS.h"
#include <RMTT_Libs.h>

#include "fileparser.h"
#include "multidrone.h"
#include "numberfonts.h"
#include "protocol.h"

#include "state_machine.h"

#define SDK_VERSION "esp32v1.0.0.22"

#define WIFIMINVERION ((1 << 24)|(0 << 16)|(0 << 16)|(45))

/* key click */
#define CLICK_CHECK_PERIOD 1000
#define CLICK_LONG_TIME_PRESS 800
/* matrix */
#define MLED_BRIGHT   0xFF

#define RMTT_DEFAULT_MODE 0
#define RMTT_AP_MODE  1
#define RMTT_STA_MODE 2

#define RMTT_UNACTIVE 3
#define RMTT_ACTIVE 4

uint8_t wifi_unactive = RMTT_DEFAULT_MODE;
uint8_t drone_unactive = RMTT_DEFAULT_MODE;
uint8_t wifi_mode = RMTT_DEFAULT_MODE;
uint8_t fmname[64] = {0};
uint8_t fmname_is_update = 0;

RMTT_RGB tt_rgb;
RMTT_Matrix tt_matrix;
RMTT_GamesirT1d *p_tt_gamesir;
RMTT_TOF tt_tof;
RMTT_Protocol tt_sdk;

int tof_range = 0;
bool pair_mode = false;
int i2c_init_failed = 0;

sm_state current_state = CHOOSE_FILE_STATE;

bool check_wifiversion = false;
uint32_t wifiversion = 0;
uint8_t upgrade_mode = 0;

/* AUTO MODE */
uint8_t work_mode = NORMAL_MODE;
uint8_t has_formation_file = 0;

static String file_name;
static uint8_t numbuff[128] = {0};
static uint8_t readbuff[1024] = {0};
static uint8_t tmpbuff[1024] = {0};

TaskHandle_t keyScanTaskHandle = NULL;
TaskHandle_t gamesirTaskHandle = NULL;
TaskHandle_t tofBatteryReadTaskHandle = NULL;
TaskHandle_t bleStatusTaskHandle = NULL;
TaskHandle_t automodeTaskHandle = NULL;

void key_scan_task(void *arg);
void gamesir_task(void *arg);
void tof_battery_read_task(void *arg);
void ble_status_task(void *arg);
void led_task(void *pParam);
void wifi_upgrade();

int led_callback(int argc, char *argv[], char argv2[]);
int matrix_callback(int argc, char *argv[], char argv2[]);
int tof_callback(int argc, char *argv[], char argv2[]);
int version_callback(int argc, char *argv[], char argv2[]);
int rmtt_callback(int argc, char *argv[], char argv2[]);
int custom_callback(int argc, char *argv[], char argv2[]);
int unknown_cmd_callback(int argc, char *argv[], char argv2[]);

bool rmtt_int_is_valid();
bool rmtt_bool_is_valid();
bool rmtt_joystick_mac_is_valid();

int get_rmtt_int();
bool get_rmtt_bool();
uint8_t *get_rmtt_joystick_mac();

void setup_graph();
void matrix_show_graph_from_file();

void normal_setup();
void narmal_loop();

void automode_setup();
void automode_loop();

void drone_data_analysis(uint8_t *buff, uint16_t len);

sm_state choose_file_action();
sm_state entermode_action();
sm_state searchdrone_action();
sm_state countdown_action();
sm_state execute_action();

sm_action state_action[MAX_STATE_NUM] =
{
    {choose_file_action},
    {entermode_action},
    {searchdrone_action},
    {countdown_action},
    {execute_action},
};

// 16x8 heart figure.
static uint8_t tt_graph_buff[] = {
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0, 255, 255,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
      0,   0,   0,   0, 255, 255, 255,   0, 255,   0, 255,   0, 255,   0, 255,   0,
      0,   0,   0,   0, 255, 255,   0,   0,   0,   0, 255,   0,   0,   0,   0,   0,
      0,   0,   0,   0, 255, 255,   0,   0,   0,   0, 255,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 255,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 255,   0,   0,   0,   0,   0,
      0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
};

void setup()
{
    pinMode(34, INPUT_PULLUP);
    if (digitalRead(34) == 0)
    {
        wifi_upgrade();
    }

    // put your setup code here, to run once:
    Serial.begin(115200);
    Wire.begin(27, 26);
    Wire.setClock(400000);
    Serial1.begin(1000000, SERIAL_8N1, 23, 18);
    Serial.println();

    // user key, 0:press, 1:up
    pinMode(34, INPUT_PULLUP);

    Serial.println("*********RoboMaster Tello Talent********");
    Serial.println(SDK_VERSION);
    Serial.println();

    tt_rgb.Init();

    shell_cmd_init();
    led_effect_init();

    SPIFFS.begin(true);

    if (get_formation_file_num())
    {
        has_formation_file = 1;
        work_mode = is_multidrone_mode();
    }

    if (work_mode == NORMAL_MODE)
    {
        normal_setup();
    }
    else
    {
        DEBUG("enter auto mode\r\n");
        automode_setup();
    }
}

void loop()
{
    if (work_mode == NORMAL_MODE)
    {
        narmal_loop();
    }
    else
    {
        automode_loop();
    }
}

void automode_setup()
{
    cmd_register((char*)"led", led_callback);

    file_name = get_formation_name();

    tt_tof.SetTimeout(500);
    if (!tt_tof.Init())
    {
        i2c_init_failed = 1;
        DEBUG("Failed to detect and initialize sensor!");
    }
    else
    {
        // increase timing budget to 200 ms
        tt_tof.SetMeasurementTimingBudget(200000);
        tt_tof.StartContinuous();

        tt_matrix.Init(192);
        tt_matrix.SetLEDStatus(RMTT_MATRIX_CS, RMTT_MATRIX_SW,
                                RMTT_MATRIX_LED_ON);

        cmd_register((char*)"mled", matrix_callback);
        matrix_effect_init(MLED_BRIGHT);
    }
    recv_callback_init();
    xTaskCreateUniversal(key_scan_task, "key_scan_task", 4096, NULL, 2, &keyScanTaskHandle, 1);
    xTaskCreateUniversal(automode_task, "automode_task", 8192, NULL, 1, &automodeTaskHandle, 1);
}

uint8_t tmp_buff[128] = {0};
void drone_data_analysis(uint8_t *buff, uint16_t len)
{
    data_frame_t* frame = (data_frame_t*)buff;
    // DEBUG("frame type %d length %d\r\n", frame->type, frame->length);
    switch(frame->type)
    {
    case TYPE_STRING_CMD_FOR_ALL :
        ctrl_msg_send(STRING_CMD, 0, (uint8_t *)frame->pdata, frame->length);
        delay(frame->time);
        break;
    case TYPE_KEY_FRAME          :
        drone_pos_init((uint8_t *)frame->pdata, frame->id_num);
        ctrl_msg_send(STRING_CMD, 0, (uint8_t *)"tcpstatus?", (uint16_t)strlen("tcpstatus?"));
        delay(50);
        break;
    case TYPE_NORMAL_FRAME       :
        ctrl_msg_send(NORMAL_FRAME_CMD, frame->id_num, (uint8_t *)frame->pdata, frame->length);
        delay(frame->time);
        break;
    case TYPE_EXT_STRING_CMD     :
        if (frame->id_num == get_local_id())
        {
            for (int i = 4; i < frame->length; i++)
            {
                cmd_process(frame->pdata[i]);
            }
            cmd_process('\r');
            cmd_process('\n');
        }
        else
        {
            ctrl_msg_send(EXT_STRING_CMD, frame->id_num, (uint8_t *)frame->pdata, frame->length);
        }
        delay(frame->time);
        break;
    case TYPE_EXTEND_KEY_FRAME   :
        for (int i = 0; i < frame->id_num; i++)
        {
            memcpy(tmp_buff + sizeof(ctrl_msg_t) * i, frame->pdata + sizeof(ctrl_ext_msg_t) * i, sizeof(ctrl_msg_t));
        }
        drone_pos_init((uint8_t *)tmp_buff, frame->id_num);
        ctrl_msg_send(STRING_CMD, 0, (uint8_t *)"tcpstatus?", (uint16_t)strlen("tcpstatus?"));
        delay(50);

        for (int i = 0; i < frame->id_num; i++)
        {
            uint8_t led_len, mled_len;
            uint8_t led_buff[64] = {0};
            uint8_t mled_buff[64] = {0};
            uint8_t* pdata = frame->pdata + sizeof(ctrl_ext_msg_t) * i + sizeof(ctrl_msg_t);
            led_len = snprintf((char*)led_buff, sizeof(led_buff), "EXT led %d %d %d", pdata[0], pdata[1], pdata[2]);
            mled_len = snprintf((char*)mled_buff, sizeof(mled_buff), "EXT mled s %c %c", pdata[3], pdata[4]);
            DEBUG("%s %s\r\n", (char*)led_buff, (char*)mled_buff);
            if (get_local_id() == i)
            {
                for (int i = 4; i < led_len; i++)
                {
                    cmd_process(led_buff[i]);
                }
                cmd_process('\r');
                cmd_process('\n');
                for (int i = 4; i < mled_len; i++)
                {
                    cmd_process(mled_buff[i]);
                }
                cmd_process('\r');
                cmd_process('\n');
            }
            else
            {
                ctrl_msg_send(EXT_STRING_CMD, i, led_buff, led_len);
                ctrl_msg_send(EXT_STRING_CMD, i, mled_buff, mled_len);
            }
        }
        delay(frame->time);
        break;
    case TYPE_EXTEND_FRAME       :
        for (int i = 0; i < frame->id_num; i++)
        {
            memcpy(tmp_buff + sizeof(ctrl_msg_t) * i, frame->pdata + sizeof(ctrl_ext_msg_t) * i, sizeof(ctrl_msg_t));
        }
        ctrl_msg_send(NORMAL_FRAME_CMD, frame->id_num, (uint8_t *)tmp_buff, sizeof(ctrl_msg_t) * frame->id_num);

        for (int i = 0; i < frame->id_num; i++)
        {
            uint8_t led_len, mled_len;
            uint8_t led_buff[64] = {0};
            uint8_t mled_buff[64] = {0};
            uint8_t* pdata = frame->pdata + sizeof(ctrl_ext_msg_t) * i + sizeof(ctrl_msg_t);
            led_len = snprintf((char*)led_buff, sizeof(led_buff), "EXT led %d %d %d", pdata[0], pdata[1], pdata[2]);
            mled_len = snprintf((char*)mled_buff, sizeof(mled_buff), "EXT mled s %c %c", pdata[3], pdata[4]);
            if (get_local_id() == i)
            {
                for (int i = 4; i < led_len; i++)
                {
                    cmd_process(led_buff[i]);
                }
                cmd_process('\r');
                cmd_process('\n');
                for (int i = 4; i < mled_len; i++)
                {
                    cmd_process(mled_buff[i]);
                }
                cmd_process('\r');
                cmd_process('\n');
            }
            else
            {
                ctrl_msg_send(EXT_STRING_CMD, i, led_buff, led_len);
                ctrl_msg_send(EXT_STRING_CMD, i, mled_buff, mled_len);
            }
        }
        delay(frame->time);
        break;
    default:
        break;
    };
}

void automode_task(void *arg)
{

    while (1)
    {
        current_state = state_action[current_state].do_action();
    }
}

void automode_loop()
{
    auto_mode_recv();
}

sm_state choose_file_action()
{
    DEBUG("ENTER choose_file_action\r\n");
    sm_state state = CHOOSE_FILE_STATE;
    clean_event();
    tt_rgb.SetRGB(0, 0, 0);
    memset(numbuff, 0, 128);
    matrix_effect_deinit();
    if (i2c_init_failed == 0)
    {
        tt_matrix.SetAllPWM((uint8_t *)numberfonts[get_formation_id()]);

        uint16_t tof_range = 0;
        uint32_t last_time = millis(), now_time = millis();
        while(1)
        {
            now_time = millis();
            tof_range = tt_tof.ReadRangeContinuousMillimeters();
            // DEBUG("tof_range %d\r\n", tof_range);
            if (tof_range <= 25)
            {
                if (now_time - last_time >= 500)
                {
                    file_name = get_next_formation();
                    tt_matrix.SetAllPWM((uint8_t *)numberfonts[get_formation_id()]);
                    last_time = millis();
                }
            }
            else
            {
                switch (get_event())
                {
                case SINGAL_CLICK_EVENT:
                    DEBUG("file id %d\r\n", get_formation_id());
                    tt_rgb.SetRGB(255, 0, 127);
                    state = ENTER_MODE_STATE;
                    goto __save_id;
                    break;
                case DOUBLE_CLICK_EVENT:
                    return CHOOSE_FILE_STATE;
                    break;
                default:
                    break;
                }

                last_time = millis();
            }
            delay (50);
        }

__save_id:
        save_file_id(get_formation_id());

        if(check_file_head(file_name) == false)
        {
            while(1)
            {
                tt_rgb.SetRGB(255, 0, 0);
                matrix_effect_set_graph((uint8_t *)&errorfonts[1]);
                delay(800);
                tt_rgb.SetRGB(0, 0, 0);
                matrix_effect_set_graph((uint8_t *)&errorfonts[0]);
                delay(800);

                switch (get_event())
                {
                case DOUBLE_CLICK_EVENT:
                    return CHOOSE_FILE_STATE;
                    break;
                default:
                    break;
                }
            }
        }
    }
    else
    {
        return ENTER_MODE_STATE;
    }

    return state;
}

sm_state entermode_action()
{
    DEBUG("ENTER entermode_action\r\n");
    sm_state state = ENTER_MODE_STATE;
    clean_event();

    while(1)
    {
        if (enter_auto_mode(50) == 0)
        {
            delay(100);
            switch (get_event())
            {
            case DOUBLE_CLICK_EVENT:
                return CHOOSE_FILE_STATE;
                break;
            default:
                break;
            }
        }
        else
        {
            state = SEARCH_DEONE_STATE;
            break;
        }

    }
    return state;
}

sm_state searchdrone_action()
{
    DEBUG("ENTER searchdrone_action\r\n");
    uint8_t current_num = 0;
    sm_state state = SEARCH_DEONE_STATE;
    clean_event();
    memset(numbuff, 0, 128);
    uint8_t search_counter = 0;
    while(1)
    {
        server_status_clean();
        current_num = search();
        if (i2c_init_failed == 0)
        {
            if (current_num < 10)
            {
                mled_font2buff(numbuff, current_num + 48, 'r', 0xFF);
            }
            else
            {
                mled_font2buff(numbuff, 'A', 'r', 0xFF);
            }
            matrix_effect_set_graph(numbuff);
        }

        if (current_num == get_max_drone_num())
        {
            search_counter = 4;
            tt_rgb.SetRGB(0, 255, 0);
        }
        else if(current_num > get_max_drone_num())
        {
            tt_rgb.SetRGB(255, 0, 0);
        }
        else if (search_counter == 0)
        {
            tt_rgb.SetRGB(255, 255, 0);
        }
        else
        {
            if (search_counter)
            {
                search_counter--;
            }
        }
        switch (get_event())
        {
        case SINGAL_CLICK_EVENT:
        {
            if (current_num <= get_max_drone_num())
            {
                return COUNTDOWN_STATE;
            }
        }
            break;
        case DOUBLE_CLICK_EVENT:
            return CHOOSE_FILE_STATE;
            break;
        default:
            break;
        }
    }

    return state;
}

sm_state countdown_action()
{
    DEBUG("ENTER countdown_action\r\n");
    sm_state state = COUNTDOWN_STATE;
    clean_event();
    memset(numbuff, 0, 128);

    DEBUG("search complete\r\n");

    tt_rgb.SetRGB(0,0,0);
    if (i2c_init_failed == 0)
    {
        mled_font2buff(numbuff, 3 + 48, 'p', 0xFF);
        matrix_effect_set_graph(numbuff);
    }
    set_freq();
    delay(1000);
    switch (get_event())
    {
    case DOUBLE_CLICK_EVENT:
        return SEARCH_DEONE_STATE;
        break;
    default:
        break;
    }
    tt_rgb.SetRGB(0,255,0);
    if (i2c_init_failed == 0)
    {
        mled_font2buff(numbuff, 2 + 48, 'p', 0xFF);
        matrix_effect_set_graph(numbuff);
    }
    delay(1000);
    switch (get_event())
    {
    case DOUBLE_CLICK_EVENT:
        return SEARCH_DEONE_STATE;
        break;
    default:
        break;
    }
    tt_rgb.SetRGB(0,0,0);
    if (i2c_init_failed == 0)
    {
        mled_font2buff(numbuff, 1 + 48, 'p', 0xFF);
        matrix_effect_set_graph(numbuff);
    }
    delay(1000);
    if (i2c_init_failed == 0)
    {
        matrix_effect_set_graph((uint8_t*)&errorfonts[0]);
    }
    switch (get_event())
    {
    case DOUBLE_CLICK_EVENT:
        return SEARCH_DEONE_STATE;
        break;
    default:
        break;
    }
    state = EXECUTE_STATE;
    return state;
}

sm_state execute_action()
{
    DEBUG("ENTER execute_action\r\n");
    sm_state state = EXECUTE_STATE;
    clean_event();
    memset(numbuff, 0, 128);
    drone_takeoff(get_max_drone_num());

    File file = SPIFFS.open(file_name, FILE_READ);
    file.seek(sizeof(file_head_t));
    DEBUG("current file %s\r\n", file_name.c_str());
    int file_read_num = 0;
    for(;;)
    {
        file_read_num = read_data_frame(file, readbuff, sizeof(readbuff));

        if (file_read_num <= 0)
        {
            DEBUG("read over %d\r\n", file_read_num);
            break;
        }
        drone_data_analysis(readbuff, file_read_num);
        // DEBUG("data analysis\r\n");
        switch (get_event())
        {
        case MULTI_CLICK_EVENT:
            DEBUG("MULTI_CLICK_EVENT happened\r\n");
            goto __end_read;
            break;
        default:
            break;
        }
    }
__end_read:
    file.close();

    drone_land();
    return ENTER_MODE_STATE;
}

void normal_setup()
{
    cmd_register((char*)"led", led_callback);
    cmd_register((char*)"reboot", reboot_callback);
    cmd_register((char*)"version?", version_callback);
    cmd_register((char*)"ETT", rmtt_callback);
    cmd_register((char*)"DIY", custom_callback);
    cmd_unknown_handle_register(unknown_cmd_callback);

    tt_rgb.SetRGB(255, 0, 0);
    delay(200);
    tt_rgb.SetRGB(0, 255, 0);
    delay(200);
    tt_rgb.SetRGB(0, 0, 255);
    delay(200);
    tt_rgb.SetRGB(0, 0, 0);

    if (has_formation_file)
    {
        delay(200);
        tt_rgb.SetRGB(255, 255, 0);
    }

    tt_tof.SetTimeout(500);
    if (!tt_tof.Init())
    {
        i2c_init_failed = 1;
        DEBUG("Failed to detect and initialize sensor!");
    }
    else
    {
        // increase timing budget to 200 ms
        tt_tof.SetMeasurementTimingBudget(200000);
        tt_tof.StartContinuous();
        setup_graph();

        cmd_register((char*)"mled", matrix_callback);
        cmd_register((char*)"tof?", tof_callback);
        matrix_effect_init(MLED_BRIGHT);
        xTaskCreateUniversal(tof_battery_read_task, "tof_battery_read_task", 4096, NULL, 2, &tofBatteryReadTaskHandle, 1);
    }

    p_tt_gamesir = RMTT_GamesirT1d::GetInstance();
    /* large number represent high priority  */
    xTaskCreateUniversal(key_scan_task, "key_scan_task", 4096, NULL, 2, &keyScanTaskHandle, 1);
    xTaskCreateUniversal(gamesir_task, "gamesir_task", 8192, NULL, 3, &gamesirTaskHandle, 0);
    xTaskCreateUniversal(ble_status_task, "ble_status_task", 4096, NULL, 2, &bleStatusTaskHandle, 0);
}

void narmal_loop()
{
    /* Serial receive process from drone */
    while (Serial1.available())
    {
        int ret = cmd_process(Serial1.read());
        if (ret != 0)
        {
            Serial1.printf("command error: %d\r\n", ret);
        }
    }

    if (Serial.available())
    {
        int i = 0;
        char buff[20] = {0};

        while (Serial.available() && (i < 20))
        {
            buff[i++] = Serial.read();
        }
        buff[19] = '\0';

        if (!strcmp(buff, "esp32version?"))
        {
            Serial.printf(SDK_VERSION);
        }
        else if (!strcmp(buff, "file exist?"))
        {
            if (has_formation_file)
            {
                Serial.printf("exist");
            }
            else
            {
                Serial.printf("error");
            }
        }
        else if (!strcmp(buff, "wifiversion?"))
        {
            Serial1.printf("[TELLO] wifiversion?");
            check_wifiversion = true;
        }
        else if (!strcmp(buff, "wifiupgrade"))
        {
            Serial.printf("successful");
            int upgrade_cnt = 0;
            upgrade_mode = 1;
            while (1)
            {
                if (Serial.available())
                {
                    Serial1.write(Serial.read());
                    tt_rgb.SetGreen(255);
                }
                if (Serial1.available())
                {
                    Serial.write(Serial1.read());
                    tt_rgb.SetRed(255);
                }
                if (upgrade_cnt > 1000)
                {
                    tt_rgb.SetGreen(0);
                    tt_rgb.SetRed(0);
                    upgrade_cnt = 0;
                }
                upgrade_cnt++;
            }
        }
    }
    /* -------!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!-------  */
    /* DO NOT ADD ANY CODE HERE FOR NOT BLOCKING THE RECEIVE FROM THE SERIAL */
    /*     YOU CAN ADD YOUR USER CODE TO THE 'user_task' FUNCTION ABOVE      */
    /* -------!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!-------- */
}

/**
 * unknown command
 *
 *  @param argc Control argument
 *  @param argv[] Value argument 1
 *  @param argv2[] Value argument2
 */
int unknown_cmd_callback(int argc, char *argv[], char argv2[])
{
    Serial.printf("unknown cmd %s\r\n", argv[0]);
}

/*******************Matrix control part**************************/

/**
 * Initialize the Matrix and show a TT Logo
 */
void setup_graph()
{
    tt_matrix.Init(192);
    tt_matrix.SetLEDStatus(RMTT_MATRIX_CS, RMTT_MATRIX_SW,
                                RMTT_MATRIX_LED_ON);
    // Set LED brightness for all LEDs from an array.
    tt_matrix.SetAllPWM((uint8_t *)tt_graph_buff);
    delay(200);
    matrix_show_graph_from_file();
}

/**
 * Show the graph pre-settled in the file onto the Matrix
 */
void matrix_show_graph_from_file()
{
    if(SPIFFS.exists("/matrix_graph.txt"))
    {
        int i = 0;
        uint8_t graph_buff[128];
        File file = SPIFFS.open("/matrix_graph.txt", FILE_READ);
        if (file)
        {
            while (file.available() && (i < 128))
            {
                graph_buff[i++] = (char)file.read();
            }
            matrix_effect_set_graph(graph_buff);
        }
        file.close();
    }
}

/**
 * Handling the commands about the Matrix
 *
 * @param argc Control argument
 * @param argv[] Value argument 1
 * @param argv2[] Value argument2
 */
int matrix_callback(int argc, char *argv[], char argv2[])
{
    static uint8_t buff[128] = {0};

    memset(buff, 0, 128);

    if ((!strcmp(argv[1], "g")) || (!strcmp(argv[1], "sg")))
    {
        if (argc == 3)
        {
            if (rbpstr2buff(buff, argv[2], MLED_BRIGHT) != 0)
            {
                goto end;
            }
        }
        else if ((argc == 2) && strlen(argv2))
        {
            if (rbpstr2buff(buff, argv2, MLED_BRIGHT) != 0)
            {
                goto end;
            }
        }
        else
        {
            goto end;
        }

        matrix_effect_set_graph(buff);

        if (!strcmp(argv[1], "sg"))
        {
            File file = SPIFFS.open("/matrix_graph.txt", FILE_WRITE);
            if (file)
            {
                DEBUG("write graph to file\r\n");
                file.write(buff, sizeof(buff));
            }
            file.close();
        }
    }
    else if ((!strcmp(argv[1], "sc")) && (argc == 2))
    {
        if(SPIFFS.exists("/matrix_graph.txt"))
        {
            SPIFFS.remove("/matrix_graph.txt");
        }
        matrix_effect_set_graph(buff);
    }
    else if ((!strcmp(argv[1], "s")) && (argc == 4) && (strlen(argv[2]) == 1))
    {
        if (strlen(argv[3]) == 1)
        {
            if (mled_font2buff(buff, argv[3][0], argv[2][0], MLED_BRIGHT) != 0)
            {
                goto end;
            }
        }
        else if (!strcmp(argv[3], "heart"))
        {
            if (mled_font2buff(buff, 0x104, argv[2][0], MLED_BRIGHT) != 0)
            {
                goto end;
            }
        }
        else
        {
            goto end;
        }

        matrix_effect_set_graph(buff);
    }
    else if ((!strcmp(argv[1], "sl")) && (argc == 3))
    {
        int gcc = 127;
        if (sscanf(argv[2], "%d", &gcc))
        {
            if ((gcc >= 0) && (gcc <= 255))
            {
                tt_matrix.SetGCC(gcc);
            }
            else
            {
                goto end;
            }
        }
        else
        {
            goto end;
        }
    }
    else if (!move_param_is_valid(argv[1], argv[2]))
    {
        float mv_t = 1;

        if (sscanf(argv[3], "%f", &mv_t))
        {
            /* mv_t 0.1~2.5 */
            if (!((mv_t >= 0.09) && (mv_t <= 2.51)))
            {
                goto end;
            }
        }
        else
        {
            goto end;
        }

        if (argc == 5)
        {
            if (argv[2][0] != 'g')
            {
                matrix_effect_move_str(argv[4], strlen(argv[4]), argv[2][0], argv[1][0], mv_t);
            }
            else
            {
                if (rbpstr2buff(buff, argv[4], MLED_BRIGHT) != 0)
                {
                    goto end;
                }
                matrix_effect_move_graph(buff, argv[1][0], mv_t);
            }

        }
        else if ((argc == 4) && strlen(argv2))
        {
            if (argv[2][0] != 'g')
            {
                matrix_effect_move_str(argv2, strlen(argv2), argv[2][0], argv[1][0], mv_t);
            }
            else
            {
                if (rbpstr2buff(buff, argv2, MLED_BRIGHT) != 0)
                {
                    goto end;
                }
                matrix_effect_move_graph(buff, argv[1][0], mv_t);
            }

        }
        /* support string(include ' ') display, the length of one word < 40 */
        else if (argc >= 6)
        {
            if (argv[2][0] != 'g')
            {
                char str_tmp[256] = "";
                int len = 0;

                for (int i = 0; i < argc - 4; i++)
                {
                    if (len + strlen(argv[4+i]) + 1 < 256)
                    {
                        // Serial.printf("Before %s\n %d %s %d\r\n",str_tmp, len, argv[4 + i], strlen(argv[4 + i]));
                        memcpy(str_tmp + len , argv[4 + i], strlen(argv[4 + i]));
                        if (i < argc - 3)
                        {
                            memcpy(str_tmp + len + strlen(argv[4+i]), " ", 1);
                            len += 1;
                        }
                        len += strlen(argv[4+i]);
                        // Serial.printf("After %s %d %s %d\r\n",str_tmp, len, argv[4 + i],strlen(argv[4 + i]));
                    }
                    else
                    {
                        break;
                    }
                }
                // Serial.printf("%s %d\r\n",str_tmp, len);
                matrix_effect_move_str((char *)str_tmp, len, argv[2][0], argv[1][0], mv_t);
            }
        }
        else
        {
            goto end;
        }
    }
    else
    {
        goto end;
    }
    Serial1.print("matrix ok");
    return 0;
end:
    Serial1.print("matrix error");
    return 0;
}

int move_param_is_valid(char *argv, char *color)
{
    if (!strcmp(argv, "l")) {}
    else if (!strcmp(argv, "r")) {}
    else if (!strcmp(argv, "u")) {}
    else if (!strcmp(argv, "d")) {}
    else
    {
        return -1;
    }

    if (!strcmp(color, "r")) {}
    else if (!strcmp(color, "b")) {}
    else if (!strcmp(color, "p")) {}
    else if (!strcmp(color, "g")) {}
    else
    {
        return -1;
    }

    return 0;
}

/*******************Drone control part*****************************/
int rmtt_int = 0;
bool rmtt_bool = false;

bool int_is_valid = false;
bool bool_is_valid = false;

bool mac_is_valid = false;
uint8_t rmtt_mac[6] = {0};

/**
 * Drone connection handling, measure the connection state by serial data
 * feedback from the drone
 *  - Update connection state by parsing data feeded back from inner serial1
 *
 *  @param argc Control argument
 *  @param argv[] Value argument 1
 *  @param argv2[] Value argument2
 */
int rmtt_callback(int argc, char *argv[], char argv2[])
{
    if (!strcmp(argv[1], "ok"))
    {
        bool_is_valid = true;
        rmtt_bool = true;
    }
    else if (!strcmp(argv[1], "error"))
    {
        bool_is_valid = true;
        rmtt_bool = false;
    }
    else if (!strncmp(argv[1], "ap", 2))
    {
        wifi_mode = RMTT_AP_MODE;
    }
    else if (!strncmp(argv[1], "sta", 3))
    {
        wifi_mode = RMTT_STA_MODE;
    }
    else if (!strncmp(argv[1], "wifiunactive", 12))
    {
        wifi_unactive = RMTT_UNACTIVE;
    }
    else if (!strncmp(argv[1], "unactive", 8))
    {
        drone_unactive = RMTT_UNACTIVE;
    }
    else if (!strncmp(argv[1], "active", 8))
    {
        drone_unactive = RMTT_ACTIVE;
    }
    else if (!strncmp(argv[1], "wifiv", 5))
    {
        int version[4] = {0};
        /* report wifi version to PC */
        if (check_wifiversion == true)
        {
            Serial.printf(argv[1]);
        }
        if (sscanf(argv[1], "wifiv%d.%d.%d.%d", &version[0], &version[1], &version[2], &version[3]))
        {
            wifiversion = (version[0] << 24) | (version[1] << 16) | (version[2] << 16) | (version[3]);
            // Serial.printf("\r\nrecv: wifiv%d.%d.%d.%d %x\r\n", version[0], version[1], version[2], version[3], wifiversion);
        }
        wifi_unactive = RMTT_ACTIVE;
    }
    else if (!strncmp(argv[1], "RMTT-", 5))
    {
        memcpy(fmname, argv[1], strlen(argv[1]));
        DEBUG("recv %s\r\n", argv[1]);
        fmname_is_update = 1;
    }
    else if (!strcmp(argv[1], "mac"))
    {
        if ((sscanf(argv[2], "%02x%02x%02x%02x%02x%02x",
                    &rmtt_mac[0], &rmtt_mac[1], &rmtt_mac[2],
                    &rmtt_mac[3], &rmtt_mac[4], &rmtt_mac[5]) == 6)
         &&(argc == 3))
        {
            mac_is_valid = true;

            DEBUG("rmtt_callback(): mac get ok\r\n");
        }
        else
        {
            DEBUG("rmtt_callback(): mac get error");
        }

    }
    else if (sscanf(argv[1], "%d", &rmtt_int) && (argc == 2))
    {
        int_is_valid = true;
    }
    return 0;
}

/**
 * Get MAC valid state of the joystick
 *
 * @return the valid state of the MAC
 */
bool rmtt_joystick_mac_is_valid()
{
    return mac_is_valid;
}

/**
 * Get MAC of the joystick
 *
 * @return MAC of the joystick
 */
uint8_t *get_rmtt_joystick_mac()
{
    mac_is_valid = false;
    return rmtt_mac;
}

/**
 * Get valid state of whether having an Int type data received
 * from the drone
 *
 * @return valid state
 */
bool rmtt_int_is_valid()
{
    return int_is_valid;
}

/**
 * Get valid state of whether having an Boolean type data received
 * from the drone
 *
 * @return valid state
 */
bool rmtt_bool_is_valid()
{
    return bool_is_valid;
}

/**
 * Get the already received Int type data
 *
 * @return Int data from the drone
 */
int get_rmtt_int()
{
    int_is_valid = false;
    return rmtt_int;
}

/**
 * Get the already received Boolean type data
 *
 * @return Boolean data from the drone
 */
bool get_rmtt_bool()
{
    bool_is_valid = false;
    return rmtt_bool;
}

int version_callback(int argc, char *argv[], char argv2[])
{
    Serial1.printf("version %s", SDK_VERSION);
}

int reboot_callback(int argc, char *argv[], char argv2[])
{
    ESP.restart();
}

/**
 * Led command handling, control the color of the LED by input commands
 *
 *  @param argc Control argument
 *  @param argv[] Value argument 1
 *  @param argv2[] Value argument2
 */
int led_callback(int argc, char *argv[], char argv2[])
{
    int r1,b1,g1,r2,b2,g2;
    if (!strcmp(argv[1], "bl"))
    {
        float blink_freq = 1;
        if ((argc == 9)
          &&sscanf(argv[2], "%f", &blink_freq)
          &&sscanf(argv[3], "%d", &r1)
          &&sscanf(argv[4], "%d", &g1)
          &&sscanf(argv[5], "%d", &b1)
          &&sscanf(argv[6], "%d", &r2)
          &&sscanf(argv[7], "%d", &g2)
          &&sscanf(argv[8], "%d", &b2))
        {
            if ((blink_freq >= 0.09) && (blink_freq <= 10.1))
            {
                led_effect_blink(r1, g1, b1, r2, g2, b2, blink_freq);
                Serial1.print("led ok");
            }
            else
            {
                goto end;
            }

        }
        else
        {
            goto end;
        }

    }
    else if (!strcmp(argv[1], "br"))
    {
        float breath_freq = 1;
        if ((argc == 6)
          &&sscanf(argv[2], "%f", &breath_freq)
          &&sscanf(argv[3], "%d", &r1)
          &&sscanf(argv[4], "%d", &g1)
          &&sscanf(argv[5], "%d", &b1))
        {
            if ((breath_freq >= 0.09) && (breath_freq <= 2.51))
            {
                led_effect_breath(r1, g1, b1, breath_freq);
                Serial1.print("led ok");
            }
            else
            {
                goto end;
            }
        }
        else
        {
            goto end;
        }
    }
    else if (argc == 4)
    {
        if (sscanf(argv[1], "%d", &r1)
          &&sscanf(argv[2], "%d", &g1)
          &&sscanf(argv[3], "%d", &b1))
        {
            led_effect_set_rgb(r1, g1, b1);
            Serial1.print("led ok");
        }
        else
        {
            goto end;
        }
    }
    else
    {
        goto end;
    }

    return 0;
end:
    Serial1.print("led error");
    return 0;
}

/**
 * ToF measuring command handling
 *
 * @param arg Parameter about task control
 */
void tof_battery_read_task(void *arg)
{
    int range_ori = 0;
    int range_cm = 0;
    int battery_cnt = 0;

    while(1)
    {
        Serial1.print("[TELLO] wifiversion?");
        delay(50);

        if (wifi_unactive == RMTT_ACTIVE)
        {
            Serial.println("wifi is activing");
            break;
        }
        else if(wifi_unactive == RMTT_UNACTIVE)
        {
            Serial.println("wifi is unactiving");
            if (i2c_init_failed == 0)
            {
                matrix_effect_move_str((char*)"WiFi UNACTIVE!", strlen((char*)"WiFi UNACTIVE!"), 'r', 'l', 1.8);
            }
            while (1)
            {
                delay(100);
            }

        }
    }

    for (;;)
    {
        range_ori = tt_tof.ReadRangeContinuousMillimeters();

        if (range_ori != 65535)
        {
            tof_range = range_ori;
        }

        if (!(battery_cnt % 10))
        {
            if (upgrade_mode == 0)
            {
                tt_sdk.ReadBattery();
            }
        }

        if (tof_range >= 40 && tof_range <= 800)
        {
            range_cm = tof_range / 200.0;
            for (int i = 0; i < 8; i += 2)
            {
                if (2 * range_cm > i)
                {
                    tt_graph_buff[16 * 7 + i] = 0;
                    tt_graph_buff[16 * 7 + i + 1] = 255;
                }
                else
                {
                    tt_graph_buff[16 * 7 + i] = 0;
                    tt_graph_buff[16 * 7 + i + 1] = 0;
                }
            }
        }
        else if (tof_range > 800)
        {
            for (int i = 0; i < 8; i += 2)
            {
                tt_graph_buff[16 * 7 + i] = 0;
                tt_graph_buff[16 * 7 + i + 1] = 255;
            }
        }
        else
        {
            for (int i = 0; i < 8; i += 2)
            {
                tt_graph_buff[16 * 7 + i] = 0;
                tt_graph_buff[16 * 7 + i + 1] = 0;
            }
        }

        if (rmtt_int_is_valid())
        {
            int now_battery = get_rmtt_int();
            int val = now_battery / 25.0;

            for (int i = 8; i < 16; i += 2)
            {
                if (2 * val > i - 8)
                {
                    tt_graph_buff[16 * 7 + i] = 255;
                    tt_graph_buff[16 * 7 + i + 1] = 255;
                }
                else
                {
                    tt_graph_buff[16 * 7 + i] = 0;
                    tt_graph_buff[16 * 7 + i + 1] = 0;
                }
            }

            if ((now_battery >= 10) && (val == 0))
            {
                tt_graph_buff[16 * 7 + 8] = 80;
                tt_graph_buff[16 * 7 + 8 + 1] = 25;
            }
#define ACTIVE_TIMEOUT 3000
            /// 读取电量后判断飞机是否激活
            if (drone_unactive == RMTT_DEFAULT_MODE)
            {
                uint32_t __time = millis();
                while(1)
                {
                    Serial1.print("[TELLO] active?");
                    delay(50);
                    if (drone_unactive == RMTT_ACTIVE)
                    {
                        Serial.println("drone is activing");
                        break;
                    }
                    else if(drone_unactive == RMTT_UNACTIVE)
                    {
                        Serial.println("drone is unactiving");
                        if (i2c_init_failed == 0)
                        {
                            matrix_effect_move_str((char*)"DRONE UNACTIVE!", strlen((char*)"DRONE UNACTIVE!"), 'r', 'l', 1.8);
                        }
                        while (1)
                        {
                            delay(100);
                        }
                    }

                    if (millis() - __time > ACTIVE_TIMEOUT)
                    {
                        Serial.println("drone version is too lower");
                        drone_unactive = RMTT_ACTIVE;
                        break;
                    }
                }
            }

        }

        if (get_matrix_effect_mode() == MATRIX_EFFECT_FACTORY_MODE)
        {
            tt_matrix.SetAllPWM((uint8_t *)tt_graph_buff);
        }
        battery_cnt++;
        delay(100);
    }
}

/**
 * Gamesir joystick pairing process handling
 *  - Detect the press event of the pairing button
 *  - Pair the joystick with the TT Plugin Module
 *
 * @param arg Parameter about task control
 */
void key_scan_task(void *arg)
{
    /*
          1-single click
          2-double click
          3-triple click
          255-long time press
    */
    uint8_t state = 0;
    uint8_t key_toggle = 0;
    uint8_t key_cnt = 0;
    uint8_t key_up_f = 0, key_down_f = 0;

    uint8_t long_time_press = 0;

    uint32_t start_time = millis();
    uint32_t now_press_time = millis();
    for (;;)
    {
        if (digitalRead(34) == 0)
        {
            delay(10);
            now_press_time = millis();
            if(digitalRead(34) == 0)
            {
                if (now_press_time - start_time > CLICK_CHECK_PERIOD)
                {
                    DEBUG("CLICK_CHECK_PERIOD now_press_time %d start_time %d\r\n",now_press_time, start_time);
                    start_time = millis();
                    key_cnt = 1;
                    key_down_f = 0;
                    key_up_f = 0;
                }
                else
                {
                    if ((now_press_time - start_time > CLICK_LONG_TIME_PRESS ) && (key_cnt == 1))
                    {
                        DEBUG("CLICK_LONG_TIME_PRESS\r\n");
                        long_time_press = 1;
                        state = 255;
                        key_cnt = 0;
                        start_time = millis();
                    }
                    else
                    {
                        long_time_press = 0;
                    }
                }

                if (key_down_f)
                {
                    key_cnt++;
                    key_down_f = 0;
                    DEBUG("key_cnt %d\r\n", key_cnt);
                }
            }
        }
        else
        {
            delay(10);
            if(digitalRead(34) != 0)
            {
                if (key_up_f)
                {
                    key_up_f = 0;
                }

                key_down_f = 1;
            }
        }

        now_press_time = millis();

        if ((now_press_time - start_time > CLICK_CHECK_PERIOD)&&(long_time_press == 0))
        {
            if (key_cnt)
            {
                DEBUG("state %d\r\n", key_cnt);
                state = key_cnt;
                key_cnt = 0;
            }
        }

        switch (state)
        {
        case 1:
            if (work_mode == FORMATION_MODE)
            {
                DEBUG("next state\r\n");
                set_event(SINGAL_CLICK_EVENT);
            }
            break;
        case 2:
            if (work_mode == FORMATION_MODE)
            {
                DEBUG("last state\r\n");
                set_event(DOUBLE_CLICK_EVENT);
            }
            else if (work_mode == NORMAL_MODE)
            {
                if (!key_toggle)
                {
                    Serial1.printf("[TELLO] motoron");
                    DEBUG("[TELLO] motoron\r\n");
                }
                else
                {
                    Serial1.printf("[TELLO] motoroff");
                    DEBUG("[TELLO] motoroff\r\n");
                }
                key_toggle = !key_toggle;
            }
            break;
        case 3:
            {
                DEBUG("triple click\r\n");
#define KEY_TIMEOUT 3000

                if (work_mode != NORMAL_MODE)
                {
                    break;
                }

                /// 查询 wifi 模式版本号
                uint32_t __time = millis();
                while (1)
                {
                    if (millis() - __time > KEY_TIMEOUT)
                    {
                        if (i2c_init_failed == 0)
                        {
                            char *info = "TIMEOUT!";
                            matrix_effect_move_str((char*)info, strlen((char*)info), 'r', 'l', 1.8);
                        }
                        goto __end;
                    }
                    Serial1.printf("[TELLO] wifiversion?");
                    delay(50);
                    if ((wifiversion < WIFIMINVERION) && (wifiversion))
                    {
                        if (i2c_init_failed == 0)
                        {
                            matrix_effect_move_str((char*)"VERSION ERROR!", strlen((char*)"VERSION ERROR!"), 'r', 'l', 1.8);
                        }
                        goto __end;
                    }
                    else if(wifiversion >= WIFIMINVERION)
                    {
                        break;
                    }
                }

                /// 查询 wifi 模式
                wifi_mode = RMTT_DEFAULT_MODE;
                fmname_is_update = 0;
                __time = millis();
                while (1)
                {
                    if (millis() - __time > KEY_TIMEOUT)
                    {
                        if (i2c_init_failed == 0)
                        {
                            char *info = "TIMEOUT!";
                            matrix_effect_move_str((char*)info, strlen((char*)info), 'r', 'l', 1.8);
                        }
                        goto __end;
                    }
                    Serial1.print("[TELLO] stamode?");
                    delay(50);
                    if (wifi_mode == RMTT_STA_MODE)
                    {
                        goto __exec;
                    }
                    else if (wifi_mode == RMTT_AP_MODE)
                    {
                        break;
                    }
                }

                /// AP 飞机先获取 AP 名称显示
                __time = millis();
                memset(fmname, 0, sizeof(fmname));
                if (!has_formation_file)
                {
                    while (1)
                    {
                        if (millis() - __time > KEY_TIMEOUT)
                        {
                            if (i2c_init_failed == 0)
                            {
                                char *info = "TIMEOUT!";
                                matrix_effect_move_str((char*)info, strlen((char*)info), 'r', 'l', 1.8);
                            }
                            goto __end;
                        }

                        Serial1.print("[TELLO] fmname?");
                        delay(50);

                        if (fmname_is_update == 1)
                        {
                            if (i2c_init_failed == 0)
                            {
                                DEBUG("fmname: %s\r\n", fmname);
                                matrix_effect_move_str((char*)fmname, strlen((char*)fmname), 'r', 'l', 1.8);
                            }
                            break;
                        }
                    }
                }

                /// 发送 fmhostrun 重启
                __time = millis();
                get_rmtt_bool();
                while (1)
                {
                    if (millis() - __time > 5000)
                    {
                        if (i2c_init_failed == 0)
                        {
                            char *info = "TIMEOUT!";
                            matrix_effect_move_str((char*)info, strlen((char*)info), 'r', 'l', 1.8);
                        }
                        goto __end;
                    }

                    Serial1.print("[TELLO] fmhostrun");
                    delay(50);

                    if (rmtt_bool_is_valid())
                    {
                        break;
                    }
                }
__exec:
                if (has_formation_file)
                {
                    cfg_automode_enable();
                }
__end:
                break;
            }
        case 255:
            if (work_mode == NORMAL_MODE)
            {
                if (!p_tt_gamesir->GetConnectedStatus())
                {
                    pair_mode = true;
                    p_tt_gamesir->SetMACFilterEnable(false);
                }
                DEBUG("BLE PAIR\r\n");
            }
            DEBUG("long time press\r\n");
            break;
        default:
            break;
        }

        if ((work_mode == FORMATION_MODE) && (state > 3))
        {
            set_event(MULTI_CLICK_EVENT);
        }
        else if ((work_mode == NORMAL_MODE) && (state >= 4) && (state != 255))
        {
            DEBUG("force land\r\n");
            tt_rgb.SetRGB(255, 255, 255);
            drone_force_land();
            delay(100);
        }

        state = 0;
    }
}

#define TAKEOFF_TIMEOUT 200

int takeoff_status = 0;

int now_time  = 0;
int last_clean_time  = 0;

/**
 * Gamesir joystick control handling
 *  - Receive command from the joystick
 *  - Control the drone by received command
 *
 * @param arg Parameter about task control
 */
void gamesir_task(void *arg)
{
    uint8_t command_init = 0;
    uint8_t mac_init = 0;

    uint8_t stop_cnt = 5;

    for (;;)
    {
        if (mac_init == 0)
        {
            if (upgrade_mode == 0)
            {
                Serial1.print("[TELLO] getmac?");
            }
            delay(100);

            DEBUG("gamesir_task(): mac is ok?\r\n");

            if (rmtt_joystick_mac_is_valid())
            {
                DEBUG("gamesir_task(): ble mac init\r\n");

                p_tt_gamesir->Init(get_rmtt_joystick_mac());
                mac_init = 1;
            }
        }
        else if ((command_init == 0) && (p_tt_gamesir->GetConnectedStatus()))
        {
            tt_sdk.SDKOn();
            delay(100);
            if (rmtt_bool_is_valid() && (pair_mode != true))
            {
                command_init = 1;
                get_rmtt_bool();
            }
        }
        else if (p_tt_gamesir->DataIsValid())
        {
            // Serial.println("data is update");
            PlainData data = p_tt_gamesir->GetData();

            int lx = ((float)data.left_x_3d - 512) / 512.0 * 100;
            int ly = ((float)data.left_y_3d - 512) / 512.0 * 100;
            int rx = ((float)data.right_x_3d - 512) / 512.0 * 100;
            int ry = ((float)data.right_y_3d - 512) / 512.0 * 100;

            if ((data.btn3 == 0x01) && (data.L2))
            {
                tt_sdk.Flip('f');
            }
            else if ((data.btn3 == 0x03) && (data.L2))
            {
                tt_sdk.Flip('r');
            }
            else if ((data.btn3 == 0x05) && (data.L2))
            {
                tt_sdk.Flip('b');
            }
            else if ((data.btn3 == 0x07) && (data.L2))
            {
                tt_sdk.Flip('l');
            }
            else if ((data.Y) && (data.R2))
            {
                if (takeoff_status == 0)
                {
                    tt_sdk.TakeOff();
                    takeoff_status = 1;
                }
                else
                {
                    tt_sdk.Land();
                    takeoff_status = 0;
                }
            }
            else
            {
#ifdef BLE_JAPAN_CTRL
                tt_sdk.SetRC(lx, -ly, -ry, rx);
#else
                tt_sdk.SetRC(rx, -ry, -ly, lx);
#endif
            }
        }

        /* command can't be send continuously  */
        delay(10);

        /* Regularly send data packet to ensure the drone
        floating steadily after the controller was offline*/
        if ((now_time - last_clean_time > 300) && command_init)
        {
            if (p_tt_gamesir->GetDataOffline())
            {
                if (stop_cnt)
                {
                    tt_sdk.SetRC(0, 0, 0, 0);
                    stop_cnt--;
                }
            }
            else
            {
                /* keepactive */
                Serial1.print("[TELLO] keepalive");
                stop_cnt = 5;
            }
            last_clean_time = millis();
        }
        else
        {

        }

        now_time = millis();
    }
}

/**
 * Gamesir joystick Bluetooth(BLE) connection handling
 *
 * @param arg Parameter about task control
 */
void ble_status_task(void *arg)
{
    static int __led_cnt = 0;
    static uint8_t toggle = 0;
    uint8_t ble_mac[6] = {0};

    while (1)
    {
        if (get_led_effect_mode() == LED_EFFECT_FACTORY_MODE)
        {
            if (p_tt_gamesir->GetConnectedStatus())
            {
                if (pair_mode == true)
                {
                    memcpy(ble_mac, p_tt_gamesir->GetMAC(), 6);
                    p_tt_gamesir->SetMACFilterEnable(true);
                    Serial1.printf("[TELLO] setmac %02x%02x%02x%02x%02x%02x",
                                        ble_mac[0], ble_mac[1], ble_mac[2],
                                        ble_mac[3], ble_mac[4], ble_mac[5]);
                    delay(50);
                    if (rmtt_bool_is_valid())
                    {

                        DEBUG("ble_status_task(): peer is successful\r\n");

                        if (get_rmtt_bool())
                        {
                            pair_mode = false;
                        }
                    }
                }
                tt_rgb.SetBlue(255);
            }
            else if (pair_mode == true)
            {
                if (__led_cnt % 4 == 0)
                {
                    toggle = ~toggle;
                }

                if (!toggle)
                {
                    tt_rgb.SetBlue(0);
                }
                else
                {
                    tt_rgb.SetBlue(255);
                }
            }
            else
            {
                if (!has_formation_file)
                {
                    tt_rgb.SetBlue(0);
                }
            }
        }

        __led_cnt++;
        delay(100);
    }
}

void wifi_upgrade()
{
    int cnt = 0;
    // put your setup code here, to run once:
    Serial.begin(921600);
    Serial1.begin(1000000, SERIAL_8N1, 23, 18);
    tt_rgb.Init();
    tt_rgb.SetRGB(0, 255, 0);
    delay(500);
    while (1)
    {
        if (Serial.available())
        {
            Serial1.write(Serial.read());
            tt_rgb.SetGreen(255);
        }
        if (Serial1.available())
        {
            Serial.write(Serial1.read());
            tt_rgb.SetRed(255);
        }
        if (cnt > 1000)
        {
            tt_rgb.SetGreen(0);
            tt_rgb.SetRed(0);
            cnt = 0;
        }
        cnt++;
    }
}

int tof_callback(int argc, char *argv[], char argv2[])
{
    Serial1.printf("tof %d", tof_range);
}

/**
 * DIY command callback
 *
 *  @param argc Control argument
 *  @param argv[] Value argument 1
 *  @param argv2[] Value argument2
 */
int custom_callback(int argc, char *argv[], char argv2[])
{
    // you can do what you want after receive diy command.
}


