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

RMTT_GamesirT1d *RMTT_GamesirT1d::pInstance = nullptr;
BLEScan *RMTT_GamesirT1d::pBLEScan = nullptr;

PlainData RMTT_GamesirT1d::RMTT_GamesirT1dData;

BLEUUID RMTT_GamesirT1d::serviceUUID;
BLEUUID RMTT_GamesirT1d::plaintextUUID;
BLEUUID RMTT_GamesirT1d::readUUID;

BLEAdvertisedDevice *RMTT_GamesirT1d::pDevice = nullptr;
BLEClient *RMTT_GamesirT1d::pClient = nullptr;
BLERemoteService *RMTT_GamesirT1d::pRemoteService = nullptr;
BLERemoteCharacteristic *RMTT_GamesirT1d::pRemoteCharacteristic = nullptr;
BLERemoteCharacteristic *RMTT_GamesirT1d::pRemoteReadCharacteristic = nullptr;
BLEAdvertisedDeviceCallbacks *RMTT_GamesirT1d::pAdvertisedDeviceCallbacks = nullptr;
BLEClientCallbacks *RMTT_GamesirT1d::pClientCallbacks = nullptr;

bool RMTT_GamesirT1d::connected = false;
bool RMTT_GamesirT1d::doconnect = false;
bool RMTT_GamesirT1d::dataIsValid = false;

bool RMTT_GamesirT1d::dataOffline = false;

bool RMTT_GamesirT1d::macFilterEnable = false;
uint8_t RMTT_GamesirT1d::peerMAC[6] = {0};

static void GamesirNotifyCallback(
    BLERemoteCharacteristic *pBLERemoteCharacteristic,
    uint8_t *pData,
    size_t length,
    bool isNotify)
{

  if ((pData[0] == 0xA1) && (pData[1] == 0xC5) && (length == 20))
  {
#ifdef RMTT_BLE_DATA_DUMP
    Serial.print("Notify callback for characteristic ");
    Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    Serial.print(" of data length ");
    Serial.println(length);
    Serial.print("data: ");
    for (int i = 0; i < length; i++)
    {
      Serial.printf("0x%02x ", pData[i]);
    }
    Serial.print("\r\n");
#endif
    RMTT_GamesirT1d::DataUpdate(pData);
#ifdef RMTT_GAMESIR_DEBUG
    Serial.println("RMTT_GamesirT1d DATA update.");
    RMTT_GamesirT1d::GetData().ToSerial();
#endif
  }
}

class GamesirClientCallback : public BLEClientCallbacks
{
  void onConnect(BLEClient *pclient)
  {
  }

  void onDisconnect(BLEClient *pclient)
  {
    RMTT_GamesirT1d::SetConnectedStatus(false);
#ifdef RMTT_GAMESIR_LOG
    Serial.println("BLE Client on Disconnect");
#endif
  }
};

/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class GamesirAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
  /**
    * Called for each advertising BLE server.
    */
  void onResult(BLEAdvertisedDevice advertisedDevice)
  {

    if (!advertisedDevice.haveServiceUUID())
    {
#ifdef RMTT_BLE_SCAN_DEBUG
      Serial.println("onResult():UUID is not found!");
#endif
      goto end;
    }

    if (!strncmp(advertisedDevice.getServiceUUID().toString().c_str(), "00008650", 8))
    {
      BLEAddress macAdress = advertisedDevice.getAddress();
      if(RMTT_GamesirT1d::MACFilterIsEnable())
      {
        if(!memcmp(RMTT_GamesirT1d::GetMAC(), macAdress.getNative(), 6))
        {
          // Successful
        }
        else
        {
          goto end;
        }

      }
      else
      {
        memcpy(RMTT_GamesirT1d::GetMAC(), macAdress.getNative(), 6);
      }
    }
    else
    {
      goto end;
    }

#ifdef RMTT_GAMESIR_DEBUG
    Serial.println("onResult(): UUID Scan Stop!");
#endif
    BLEDevice::getScan()->stop();

#ifdef RMTT_GAMESIR_LOG
    Serial.printf("Advertised Device: %s \r\n", advertisedDevice.toString().c_str());
    Serial.printf("RSSI %d dbm \r\n", advertisedDevice.getRSSI());
#endif

    RMTT_GamesirT1d::SetCharacteristicUUID(advertisedDevice.getServiceUUID());

    delete RMTT_GamesirT1d::GetDevice();

    RMTT_GamesirT1d::CreateDevice(advertisedDevice);

    /* connect server */
    RMTT_GamesirT1d::SetDoConnect(true);

    /* clean results to release memary */
    BLEDevice::getScan()->clearResults();

  end:
    if ((BLEDevice::getScan()->getResults()).getCount() > 50)
    {
#ifdef RMTT_BLE_SCAN_DEBUG
      Serial.println("onResult(): Clean Devices!");
#endif
      BLEDevice::getScan()->clearResults(); // delete results fromBLEScan buffer to release memory
    }
  }
};

RMTT_GamesirT1d::RMTT_GamesirT1d()
{
  BLEDevice::init("Tello EDU");

  pClientCallbacks = new GamesirClientCallback();
  pAdvertisedDeviceCallbacks = new GamesirAdvertisedDeviceCallbacks();

  pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(pClientCallbacks);

  pBLEScan = BLEDevice::getScan(); //create new scan

  pBLEScan->setAdvertisedDeviceCallbacks(pAdvertisedDeviceCallbacks);
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99); // less or equal setInterval value

#ifdef RMTT_GAMESIR_LOG
  Serial.println("RMTT_GamesirT1d(): Created RMTT_GamesirT1d!");
#endif
}

RMTT_GamesirT1d::~RMTT_GamesirT1d()
{
  delete pDevice;
  delete pClient;
  delete pBLEScan;
  delete pAdvertisedDeviceCallbacks;
  delete pClientCallbacks;
}

RMTT_GamesirT1d *RMTT_GamesirT1d::GetInstance()
{
  if (pInstance == NULL)
  {
    pInstance = new RMTT_GamesirT1d();
  }

  return pInstance;
}

static TaskHandle_t connectTaskHandle = NULL;
static TaskHandle_t readTaskHandle = NULL;
static void gamesir_connect_task(void *pvParameters);
static void gamesir_read_task(void *pvParameters);

bool RMTT_GamesirT1d::Init()
{
  if (GetInstance() == nullptr)
  {
#ifdef RMTT_GAMESIR_LOG
    Serial.println("Init(): Create RMTT_GamesirT1d Failed!");
#endif
    return false;
  }
  if (connected)
  {
#ifdef RMTT_GAMESIR_LOG
    Serial.println("Init(): Please disconnect!");
#endif
    return false;
  }

  xTaskCreateUniversal(gamesir_connect_task, "connnectTask", 8192, NULL, 6, &connectTaskHandle, 0);
  xTaskCreateUniversal(gamesir_read_task, "readTask", 4096, NULL, 5, &readTaskHandle, 0);

  macFilterEnable = false;
  return true;
}

bool RMTT_GamesirT1d::Init(uint8_t *mac)
{
  if (GetInstance() == nullptr)
  {
#ifdef RMTT_GAMESIR_LOG
    Serial.println("Init(): Create RMTT_GamesirT1d Failed!");
#endif
    return false;
  }
  if (connected)
  {
#ifdef RMTT_GAMESIR_LOG
    Serial.println("Init(): Please disconnect!");
#endif
    return false;
  }

  xTaskCreateUniversal(gamesir_connect_task, "connnectTask", 8192, NULL, 6, &connectTaskHandle, 0);
  xTaskCreateUniversal(gamesir_read_task, "readTask", 4096, NULL, 5, &readTaskHandle, 0);

  macFilterEnable = true;
  memcpy(peerMAC, mac, 6);
  return true;
}

bool RMTT_GamesirT1d::ConnectToServer()
{

  if (connected == true)
  {
#ifdef RMTT_GAMESIR_LOG
    Serial.println("ConnectToServer(): Failed: Server is connected!");
#endif
    return false;
  }

#ifdef RMTT_GAMESIR_LOG
  Serial.print("ConnectToServer(): Forming a connection to ");
  Serial.println(pDevice->getAddress().toString().c_str());
#endif

  // Connect to the remove BLE Server.
  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
  if (pClient->connect(pDevice))
  {
#ifdef RMTT_GAMESIR_LOG
    Serial.println("ConnectToServer(): Connect is successful");
#endif
  }
  else
  {
#ifdef RMTT_GAMESIR_LOG
    Serial.println("ConnectToServer(): Failed to Connect Device!");
#endif
    return false;
  }

#ifdef RMTT_GAMESIR_LOG
  Serial.println("ConnectToServer(): Connected to server");
#endif

  // Obtain a reference to the service we are after in the remote BLE server.
  pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr)
  {
#ifdef RMTT_GAMESIR_LOG
    Serial.print("ConnectToServer(): Failed to find our service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
#endif
    pClient->disconnect();
    return false;
  }

#ifdef RMTT_GAMESIR_LOG
  Serial.println("ConnectToServer(): Found our service");
#endif

  // Obtain a reference to the characteristic in the service of the remote BLE server.
  pRemoteCharacteristic = pRemoteService->getCharacteristic(plaintextUUID);
  if (pRemoteCharacteristic == nullptr)
  {
#ifdef RMTT_GAMESIR_LOG
    Serial.print("ConnectToServer(): Failed to find our characteristic UUID: ");
    Serial.println(plaintextUUID.toString().c_str());
#endif
    pClient->disconnect();
    return false;
  }

  pRemoteReadCharacteristic = pRemoteService->getCharacteristic(readUUID);
  if (pRemoteReadCharacteristic == nullptr)
  {
#ifdef RMTT_GAMESIR_LOG
    Serial.print("ConnectToServer(): Failed to find our read UUID: ");
#endif
    return false;
  }

#ifdef RMTT_GAMESIR_LOG
  Serial.println("ConnectToServer(): Found our characteristic");
#endif

  if (pRemoteCharacteristic->canNotify())
  {
    pRemoteCharacteristic->registerForNotify(GamesirNotifyCallback);
#ifdef RMTT_GAMESIR_LOG
    Serial.println("ConnectToServer(): GamesirNotify register successful");
#endif
  }
  else
  {
#ifdef RMTT_GAMESIR_LOG
    Serial.println("ConnectToServer(): Failed to register GamesirNotify");
#endif
    return false;
  }

  SetDoConnect(false);
  connected = true;

  return true;
}

void RMTT_GamesirT1d::ScanUUID()
{

  if (connected == false)
  {
#ifdef RMTT_GAMESIR_LOG
    Serial.println("ScanUUID():Scan UUID Start...");
#endif
    BLEDevice::getScan()->start(2, false);
  }
  else
  {
#ifdef RMTT_GAMESIR_LOG
    Serial.println("ScanUUID():Failed: RMTT_GamesirT1d is connected");
#endif
    return;
  }
}

void RMTT_GamesirT1d::DataUpdate(uint8_t *data)
{
  RMTT_GamesirT1dData.left_x_3d = (data[2] << 2) | ((data[3] & 0xC0) >> 6);
  RMTT_GamesirT1dData.left_y_3d = ((data[3] & 0x3F) << 4) | ((data[4] & 0xF0) >> 4);
  RMTT_GamesirT1dData.right_x_3d = ((data[4] & 0x0F) << 6) | ((data[5] & 0xFC) >> 2);
  RMTT_GamesirT1dData.right_y_3d = ((data[5] & 0x03) << 8) | (data[6]);
  RMTT_GamesirT1dData.L2Press = data[7];
  RMTT_GamesirT1dData.R2Press = data[8];
  RMTT_GamesirT1dData.btn1 = data[9];
  RMTT_GamesirT1dData.btn2 = data[10];
  RMTT_GamesirT1dData.btn3 = data[11];
  dataIsValid = true;
}

void RMTT_GamesirT1d::CleanScanResult()
{
  BLEDevice::getScan()->clearResults();
}

void RMTT_GamesirT1d::CreateDevice(BLEAdvertisedDevice advertisedDevice)
{
  pDevice = new BLEAdvertisedDevice(advertisedDevice);
}

static uint32_t read_time = 0;

void RMTT_GamesirT1d::SetCharacteristicUUID(BLEUUID uuid)
{
  serviceUUID = uuid;
  std::string str = uuid.toString();
  str.replace(str.begin() + 7, str.begin() + 8, "1");
  plaintextUUID = plaintextUUID.fromString(str);
  str.replace(str.begin() + 5, str.begin() + 8, "65f");
  readUUID = readUUID.fromString(str);
}

static void gamesir_connect_task(void *pvParameters)
{

  while(1)
  {
    if (RMTT_GamesirT1d::GetDoConnect())
    {
      RMTT_GamesirT1d::ConnectToServer();
    }
    else if (!RMTT_GamesirT1d::GetConnectedStatus())
    {
      RMTT_GamesirT1d::ScanUUID();
    }

    if (RMTT_GamesirT1d::GetConnectedStatus())
    {

    }
    else
    {
      RMTT_GamesirT1d::SetDataOffline(true);
    }

    if (millis() - read_time < 400)
    {
      RMTT_GamesirT1d::SetDataOffline(false);
    }
    else
    {
      RMTT_GamesirT1d::SetDataOffline(true);
    }

    delay(50);
  }
}

static void gamesir_read_task(void *pvParameters)
{
  int read_cnt = 0;
  while(1)
  {

    if (RMTT_GamesirT1d::GetConnectedStatus())
    {
        if(read_cnt % 3 == 0)
        {
          read_time = millis();
          std::string value = RMTT_GamesirT1d::GetReadCharacteristic()->readValue();
        }
    }

    read_cnt++;

    delay(50);
  }
}