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

int rmtt_callback(int argc, char *argv[], char argv2[]);

bool rmtt_int_is_valid();
bool rmtt_bool_is_valid();
bool rmtt_joystick_mac_is_valid();

int get_rmtt_int();
bool get_rmtt_bool();
uint8_t *get_rmtt_joystick_mac();

RMTT_RGB tt_rgb;
RMTT_GamesirT1d *p_tt_gamesir;
RMTT_Protocol tt_sdk;

/* BLE PAIR */
bool pair_mode = false;

TaskHandle_t gamesirPairingTaskHandle = NULL;
TaskHandle_t gamesirTaskHandle = NULL;
TaskHandle_t bleStatusTaskHandle = NULL;
TaskHandle_t userTaskHandle = NULL;

void gamesir_pairing_task(void *arg);
void gamesir_task(void *arg);
void ble_status_task(void *arg);

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

    p_tt_gamesir = RMTT_GamesirT1d::GetInstance();

    shell_cmd_init();
    cmd_register((char *)"ETT", rmtt_callback);

    /* Create tasks for multi-tasking  */
    /* Larger number represents higher priority  */
    xTaskCreateUniversal(gamesir_pairing_task, "gamesir_pairing_task", 4096, NULL, 2,
                         &gamesirPairingTaskHandle, 1);
    xTaskCreateUniversal(gamesir_task, "gamesir_task", 8192, NULL, 3,
                         &gamesirTaskHandle, 0);
    xTaskCreateUniversal(ble_status_task, "ble_status_task", 4096, NULL, 2,
                         &bleStatusTaskHandle, 0);

    xTaskCreateUniversal(user_task, "user_task", 8192, NULL, 1, &userTaskHandle, 1);
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
        int ret = cmd_process(CommonSerial.read()); // Send received data to the command parser
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
 *  - Fetch the MAC of the drone
 *
 * @param arg Parameter about task control
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
    else if (!strcmp(argv[1], "mac"))
    {
        if ((sscanf(argv[2], "%02x%02x%02x%02x%02x%02x", &rmtt_mac[0],
                    &rmtt_mac[1], &rmtt_mac[2], &rmtt_mac[3], &rmtt_mac[4],
                    &rmtt_mac[5]) == 6) &&
            (argc == 3))
        {
            mac_is_valid = true;
            Serial.println("rmtt_callback(): mac get ok");
        }
        else
        {
            Serial.println("rmtt_callback(): mac get error");
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
 * Gamesir joystick pairing process handling
 *  - Detect the press event of the pairing button
 *  - Pair the joystick with the TT Plugin Module
 *
 * @param arg Parameter about task control
 */
void gamesir_pairing_task(void *arg)
{
    int __key_cnt = 0;
    pinMode(34, INPUT_PULLUP);
    for (;;)
    {
        if (digitalRead(34) == 0)
            __key_cnt++;
        else
            __key_cnt = 0;

        if (__key_cnt >= 20 && !p_tt_gamesir->GetConnectedStatus())
        {
            pair_mode = true;
            p_tt_gamesir->SetMACFilterEnable(false);
        }

        delay(100);
    }
}

#define TAKEOFF_TIMEOUT 200

int takeoff_status = 0;

int now_time = 0;
int last_clean_time = 0;

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

    for (;;)
    {
        if (mac_init == 0)
        {
            /* Try to fetch the MAC of any already paired joysticks from the
             * drone */
            CommonSerial.print("[TELLO] getmac?");
            delay(100);
            Serial.println("gamesir_task(): mac is ok?");
            if (rmtt_joystick_mac_is_valid())
            {
                Serial.println("gamesir_task(): ble mac init");
                p_tt_gamesir->Init(get_rmtt_joystick_mac());
                mac_init = 1;
            }
        }
        else if ((command_init == 0) &&
                 (p_tt_gamesir->GetConnectedStatus()))
        {
            tt_sdk.SDKOn();
            delay(100);
            if (rmtt_bool_is_valid())
            {
                command_init = 1;
            }
        }
        else if (p_tt_gamesir->DataIsValid())
        {
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

        /* Regularly send data packet to ensure the drone
        floating steadily after the controller was offline*/

        if ((now_time - last_clean_time > 300) &&
            (p_tt_gamesir->GetDataOffline()) && command_init)
        {
            tt_sdk.SetRC(0, 0, 0, 0);

            last_clean_time = millis();
        }
        else
        {
        }

        now_time = millis();

        delay(10);
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
                    /* Write the MAC of the joystick to the drone */
                    CommonSerial.printf(
                        "[TELLO] setmac %02x%02x%02x%02x%02x%02x", ble_mac[0],
                        ble_mac[1], ble_mac[2], ble_mac[3], ble_mac[4],
                        ble_mac[5]);
                    delay(50);
                    if (rmtt_bool_is_valid())
                    {
                        Serial.println(
                            "ble_status_task(): pairing is successful");
                        if (get_rmtt_bool())
                        {
                            pair_mode = false;
                        }
                    }
                }
                RMTT_RGB::SetBlue(255);
            }
            else if (pair_mode == true)
            {
                if (__led_cnt % 4 == 0)
                {
                    toggle = ~toggle;
                }

                if (!toggle)
                {
                    RMTT_RGB::SetBlue(0);
                }
                else
                {
                    RMTT_RGB::SetBlue(255);
                }
            }
            else
            {
                RMTT_RGB::SetBlue(0);
            }
        }

        __led_cnt++;
        delay(100);
    }
}
