
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

#include "stdint.h"
#include "string.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>

// #define RMTT_BLE_DATA_DUMP
// #define RMTT_BLE_SCAN_DEBUG
// #define RMTT_GAMESIR_DEBUG
// #define RMTT_GAMESIR_LOG

class PlainData
{
public:
    uint16_t left_x_3d;
    uint16_t left_y_3d;
    uint16_t right_x_3d;
    uint16_t right_y_3d;

    uint8_t L2Press;
    uint8_t R2Press;

    union{
        uint8_t btn1;
        struct{
            uint8_t A:1;
            uint8_t B:1;
            uint8_t C:1; /*!< Menu */
            uint8_t X:1;
            uint8_t Y:1;
            uint8_t Z:1; /*!< none */
            uint8_t L1:1;
            uint8_t R1:1;
        };
    };

    union{
        uint8_t btn2;
        struct{
            uint8_t L2:1;
            uint8_t R2:1;
            uint8_t SELECT:1;/*~< C1 */
            uint8_t START:1; /*!< C2 */
            uint8_t HOME:1;  /*!< none */
            uint8_t L3:1;    /*!< none */
            uint8_t R3:1;    /*!< none */
            uint8_t Touch:1; /*!< none */
        };
    };

    uint8_t btn3;
    void ToSerial()
    {
      Serial.printf("left x: %04d left y: %04d right x: %04d right x: %04d", this->left_x_3d, this->left_y_3d, this->right_x_3d, this->right_y_3d);
      Serial.println();
      Serial.printf("L2 %04d %d R1 %04d %d", this->L2Press, this->L2, this->R2Press, this->R2);
      Serial.println();
      Serial.printf("A %d B %d X %d Y %d Menu %d L1 %d R1 %d", this->A, this->B, this->X, this->Y, this->C, this->L1, this->R1);
      Serial.println();
      Serial.printf("Select %d Start %d", this->SELECT, this->START);
      Serial.println();
      Serial.printf("direction %d", this->btn3);
      Serial.println();
    }
};

class RMTT_GamesirT1d
{
private:
    static RMTT_GamesirT1d* pInstance;

    RMTT_GamesirT1d();
    ~RMTT_GamesirT1d();

    static const uint8_t up = 0x01;
    static const uint8_t upRight = 0x02;
    static const uint8_t right = 0x03;
    static const uint8_t rightDown = 0x04;
    static const uint8_t down = 0x05;
    static const uint8_t downLeft = 0x06;
    static const uint8_t left = 0x07;
    static const uint8_t leftUp = 0x08;


    static class PlainData RMTT_GamesirT1dData;

    static BLEScan* pBLEScan;

    static BLEUUID serviceUUID;
    static BLEUUID plaintextUUID;
    static BLEUUID readUUID;

    static BLEAdvertisedDevice* pDevice;
    static BLEClient* pClient;
    static BLERemoteService* pRemoteService;
    static BLERemoteCharacteristic* pRemoteCharacteristic;
    static BLERemoteCharacteristic* pRemoteReadCharacteristic;

    static BLEAdvertisedDeviceCallbacks* pAdvertisedDeviceCallbacks;
    static BLEClientCallbacks* pClientCallbacks;

    static bool connected;
    static bool doconnect;
    static bool dataIsValid;

    static bool dataOffline;

    static uint8_t peerMAC[6];
    static bool macFilterEnable;

public:
    static RMTT_GamesirT1d* GetInstance();

    static bool Init();
    static bool Init(uint8_t *mac);

    static void CreateDevice(BLEAdvertisedDevice advertisedDevice);
    static void SetCharacteristicUUID(BLEUUID uuid);
    static void DataUpdate(uint8_t *data);
    static void ScanUUID();
    static void CleanScanResult();
    static bool ConnectToServer();

    static BLEAdvertisedDevice* GetDevice()
    {
        return pDevice;
    }

    static bool DataIsValid()
    {
        return dataIsValid;
    }


    static void SetMAC(uint8_t *mac)
    {
        if(mac != nullptr)
        {
            memcpy(peerMAC, mac, 6);
        }
    }

    static BLERemoteCharacteristic* GetReadCharacteristic()
    {
        return pRemoteReadCharacteristic;
    }

    static bool SetDataOffline(bool val)
    {
        dataOffline = val;
    }

    static bool GetDataOffline()
    {
        return dataOffline;
    }

    static uint8_t *GetMAC()
    {
        return peerMAC;
    }

    static void SetDoConnect(bool val)
    {
        doconnect = val;
    }

    static bool GetDoConnect()
    {
        return doconnect;
    }

    static void SetConnectedStatus(bool val)
    {
        connected = val;
    }

    static bool GetConnectedStatus()
    {
        return connected;
    }

    static bool MACFilterIsEnable()
    {
        return macFilterEnable;
    }

    static bool SetMACFilterEnable(bool enable)
    {
        macFilterEnable = enable;
    }

    static PlainData GetData()
    {
        dataIsValid = false;
        return RMTT_GamesirT1dData;
    }
};
