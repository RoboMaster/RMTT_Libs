
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
#include <RMTT_GamesirT1d.h>
#include <RMTT_Libs.h>
#include <RMTT_Protocol.h>
#include <RMTT_Shell.h>
#include <Wire.h>
#include <stdio.h>
#include <string.h>

#include "FS.h"
#include "SPIFFS.h"

// #define __UART0_DEBUG__

#ifdef __UART0_DEBUG__
#define CommonSerial Serial
#else
#define CommonSerial Serial1
#endif

#define SDK_VERSION "esp32v1.0.0.5"

/* led display mode */
#define LED_FACTORY 0
#define LED_CONST   1
#define LED_BLINK   2
#define LED_BREATH  3

#define BLINK_UNIT  200
#define BREATH_UNIT 20

int led_callback(int argc, char *argv[], char argv2[]);
void led_task(void *pParam);

/* matrix */
#define MLED_BRIGHT 0xFF
#define MOVE_UNIT   50

int tof_range = 0;

int matrix_callback(int argc, char *argv[], char argv2[]);
int tof_callback(int argc, char *argv[], char argv2[]);
int rmtt_callback(int argc, char *argv[], char argv2[]);

bool rmtt_int_is_valid();
bool rmtt_bool_is_valid();

int get_rmtt_int();
bool get_rmtt_bool();

void matrix_show_graph_from_file();

RMTT_RGB tt_rgb;
RMTT_Matrix tt_matrix;
RMTT_TOF tt_tof;
RMTT_Protocol tt_sdk;

TaskHandle_t tofBatteryReadTaskHandle = NULL;
TaskHandle_t userTaskHandle = NULL;

void tof_battery_read_task(void *arg);

// 16x8 heart figure.
static uint8_t tt_graph_buff[] = {
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 0,   0, 0,   0, 0,   0,
    0,   0,   0,   0,   255, 255, 0,   0,   0,   0,   0,   0, 0,   0, 0,   0,
    0,   0,   0,   0,   255, 255, 255, 0,   255, 0,   255, 0, 255, 0, 255, 0,
    0,   0,   0,   0,   255, 255, 0,   0,   0,   0,   255, 0, 0,   0, 0,   0,
    0,   0,   0,   0,   255, 255, 0,   0,   0,   0,   255, 0, 0,   0, 0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   255, 0, 0,   0, 0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   255, 0, 0,   0, 0,   0,
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, 0,   0, 0,   0,
};

int tof_callback(int argc, char *argv[], char argv2[])
{
    CommonSerial.printf("tof %d", tof_range);
}

int i2c_init_failed = 0;

/**
 * Initialize the Matrix and show a TT Logo
 */
void setup_graph()
{
    if (!i2c_init_failed)
    {
        RMTT_Matrix::Init(127);
        RMTT_Matrix::SetLEDStatus(RMTT_MATRIX_CS, RMTT_MATRIX_SW,
                                  RMTT_MATRIX_LED_ON);
        // Set LED brightness for all LEDs from an array.
        RMTT_Matrix::SetAllPWM((uint8_t *)tt_graph_buff);
        delay(200);
        matrix_show_graph_from_file();
    }
}

void setup()
{
    Serial.begin(115200);
    Wire.begin(27, 26);
    Wire.setClock(400000);
    Serial1.begin(1000000, SERIAL_8N1, 23, 18);
    Serial.println();

    Serial.println("*********RoboMaster Tello Talent********");
    Serial.println(SDK_VERSION);
    Serial.println();

    RMTT_RGB::Init();
    RMTT_RGB::SetRGB(255, 0, 0);
    delay(200);
    RMTT_RGB::SetRGB(0, 255, 0);
    delay(200);
    RMTT_RGB::SetRGB(0, 0, 255);
    delay(200);
    RMTT_RGB::SetRGB(0, 0, 0);

    shell_cmd_init();

    tt_tof.SetTimeout(500);

    if (!tt_tof.Init())
    {
        i2c_init_failed = 1;
        Serial.println("Failed to detect and initialize sensor!");
    }
    else
    {
        // Increase timing budget to 200 ms
        tt_tof.SetMeasurementTimingBudget(200000);
        tt_tof.StartContinuous();
        setup_graph();

        cmd_register((char *)"mled", matrix_callback);
        cmd_register((char *)"tof?", tof_callback);
        matrix_effect_init(MLED_BRIGHT);

        xTaskCreateUniversal(tof_battery_read_task, "tof_battery_read_task", 8192, NULL, 2,
                             &tofBatteryReadTaskHandle, 1);
        xTaskCreateUniversal(user_task, "user_task", 8192, NULL, 1, &userTaskHandle, 1);
    }

    cmd_register((char *)"led", led_callback);
    cmd_register((char *)"ETT", rmtt_callback);

    led_effect_init();
}

void user_task(void *arg)
{
    while (1)
    {
        /* Put your code here ! */

        delay(10);
    }
}

void loop()
{
    /* Serial receive process from drone */
    while (CommonSerial.available())
    {
        int ret = cmd_process(CommonSerial.read());
        if (ret != 0)
        {
            CommonSerial.printf("command error: %d\r\n", ret);
        }
    }
    /* -------!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!-------  */
    /* DO NOT ADD ANY CODE HERE FOR NOT BLOCKING THE RECEIVE FROM THE SERIAL */
    /*     YOU CAN ADD YOUR USER CODE TO THE 'user_task' FUNCTION ABOVE      */
    /* -------!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!-------- */
}

/*******************Matrix control part**************************/

/**
 * Show the graph pre-settled in the file onto the Matrix
 */
void matrix_show_graph_from_file()
{
    SPIFFS.begin(true);

    if (SPIFFS.exists("/graph_enable.txt"))
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
    if ((!strcmp(argv[1], "g")) || (!strcmp(argv[1], "sg")))
    {
        uint8_t buff[128] = {0};
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
            File file2 = SPIFFS.open("/graph_enable.txt", FILE_WRITE);
            if (file)
            {
                file.write(buff, sizeof(buff));
            }
            file.close();
            file2.close();
        }
    }
    else if ((!strcmp(argv[1], "sc")) && (argc == 2))
    {
        if (SPIFFS.exists("/graph_enable.txt"))
        {
            SPIFFS.remove("/graph_enable.txt");
        }
        uint8_t buff[128] = {0};
        matrix_effect_set_graph(buff);
    }
    else if ((!strcmp(argv[1], "s")) && (argc == 4) &&
             (strlen(argv[2]) == 1))
    {
        uint8_t buff[128] = {0};
        if (strlen(argv[3]) == 1)
        {
            if (mled_font2buff(buff, argv[3][0], argv[2][0], MLED_BRIGHT) !=
                0)
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
        uint8_t mv_t = 0;

        if (sscanf(argv[3], "%d", &mv_t))
        {
            if (!((mv_t >= 0) && (mv_t <= 10)))
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
            matrix_effect_move_str(argv[4], strlen(argv[4]), argv[2][0],
                                   argv[1][0], (11 - mv_t) * MOVE_UNIT);
        }
        else if ((argc == 4) && strlen(argv2))
        {
            matrix_effect_move_str(argv2, strlen(argv2), argv[2][0], argv[1][0],
                                   (11 - mv_t) * MOVE_UNIT);
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
    CommonSerial.print("matrix ok");
    return 0;
end:
    CommonSerial.print("matrix error");
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
    else if (sscanf(argv[1], "%d", &rmtt_int) && (argc == 2))
    {
        int_is_valid = true;
    }
    return 0;
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
    /* Restore the valid state */
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
    /* Restore the valid state */
    bool_is_valid = false;
    return rmtt_bool;
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
    uint32_t r1, b1, g1, r2, b2, g2;
    if (!strcmp(argv[1], "bl"))
    {
        uint8_t blink_time = 1;
        if ((argc == 9) && sscanf(argv[2], "%d", &blink_time) &&
            sscanf(argv[3], "%d", &r1) && sscanf(argv[4], "%d", &g1) &&
            sscanf(argv[5], "%d", &b1) && sscanf(argv[6], "%d", &r2) &&
            sscanf(argv[7], "%d", &g2) && sscanf(argv[8], "%d", &b2))
        {
            if ((blink_time >= 1) && (blink_time <= 10))
            {
                blink_time = (11 - blink_time) * BLINK_UNIT;
                led_effect_blink(r1, g1, b1, r2, g2, b2, blink_time);
                CommonSerial.print("led ok");
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
        int breath_time = 6;
        if ((argc == 6) && sscanf(argv[2], "%d", &breath_time) &&
            sscanf(argv[3], "%d", &r1) && sscanf(argv[4], "%d", &g1) &&
            sscanf(argv[5], "%d", &b1))
        {
            if ((breath_time >= 1) && (breath_time <= 10))
            {
                breath_time = (11 - breath_time) * BREATH_UNIT;
                led_effect_breath(r1, g1, b1, breath_time);
                CommonSerial.print("led ok");
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
        if (sscanf(argv[1], "%d", &r1) && sscanf(argv[2], "%d", &g1) &&
            sscanf(argv[3], "%d", &b1))
        {
            led_effect_set_rgb(r1, g1, b1);
            CommonSerial.print("led ok");
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
    CommonSerial.print("led error");
    return 0;
}

/**
 * ToF measuring command handling
 *
 * @param arg Parameter about task control
 */
void tof_battery_read_task(void *arg)
{
    int range_cm = 0;
    int battery_cnt = 0;
    for (;;)
    {
        tof_range = tt_tof.ReadRangeSingleMillimeters();

        if (!(battery_cnt % 10))
        {
            tt_sdk.ReadBattery();
        }

        if (tof_range >= 40 || tof_range <= 800)
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
            int val = get_rmtt_int() / 25.0;
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
        }

        if (get_matrix_effect_mode() == MATRIX_EFFECT_FACTORY_MODE)
        {
            RMTT_Matrix::SetAllPWM((uint8_t *)tt_graph_buff);
        }
        battery_cnt++;
        delay(100);
    }
}
