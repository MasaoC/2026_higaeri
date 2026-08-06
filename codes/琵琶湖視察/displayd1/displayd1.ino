#include <Wire.h>
#include <Adafruit_DPS310.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// --- ピン定義 ---
constexpr uint8_t kI2cSdaPin = 6;
constexpr uint8_t kI2cSclPin = 7;

// --- Nordic UART Service (NUS) UUIDs ---
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E" 
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E" 

Adafruit_DPS310 dps;
BLEServer* pServer = nullptr;
BLECharacteristic* pTxCharacteristic = nullptr;
bool deviceConnected = false;
bool oldDeviceConnected = false;

float seaLevelPressure = 1013.25;
unsigned long lastMeasureTime = 0;

// BLE接続コールバック
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Wire.begin(kI2cSdaPin, kI2cSclPin, 400000);

  if (!dps.begin_I2C(0x76, &Wire)) {
    Serial.println("Failed to find DPS310!");
    while (1) delay(10);
  }
  // オーバーサンプリングを最大(16回)に設定し、ノイズを極限まで減らす
  dps.configurePressure(DPS310_64HZ, DPS310_16SAMPLES);
  dps.configureTemperature(DPS310_64HZ, DPS310_16SAMPLES);

  delay(500);
  sensors_event_t temp_event, pres_event;
  if (dps.getEvents(&temp_event, &pres_event)) {
    // 起動時の気圧(hPa)を基準(0m)として保存
    seaLevelPressure = pres_event.pressure; 
  }

  BLEDevice::init("XIAO_Alt_Station");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pTxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
                                           CHARACTERISTIC_UUID_RX,
                                           BLECharacteristic::PROPERTY_WRITE |
                                           BLECharacteristic::PROPERTY_WRITE_NR
                                         );

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();
  
  Serial.println("d1: Master & BLE UART Started. Waiting for connection...");
}

void loop() {
  if (!deviceConnected && oldDeviceConnected) {
      delay(500);
      pServer->startAdvertising();
      Serial.println("BLE Disconnected. Advertising restarted.");
      oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
      Serial.println("BLE App Connected!");
      oldDeviceConnected = deviceConnected;
  }

  // 300msごとに送信
  if (millis() - lastMeasureTime >= 300) {
    lastMeasureTime = millis();
    
    uint16_t ultra_raw_cm = 0;
    float pressure = 0.0;
    float temperature = 0.0;
    float baro_altitude = 0.0;

    // --- A. D2基板から超音波データの取得 ---
    if (Wire.requestFrom((uint8_t)0x30, (uint8_t)2) == 2) {
      Wire.readBytes((uint8_t*)&ultra_raw_cm, 2);
    }
    float ultra_altitude = (float)ultra_raw_cm / 100.0;

    // --- B. DPS310から気圧・温度・高度の取得 ---
    sensors_event_t temp_event, pres_event;
    if (dps.getEvents(&temp_event, &pres_event)) {
      pressure = pres_event.pressure;       // hPa単位で取得
      temperature = temp_event.temperature; // ℃単位で取得
      
      // 気圧高度の計算 (基準気圧と比較)
      baro_altitude = 44330.0 * (1.0 - pow((pressure / seaLevelPressure), 0.1902949));
    }

    // --- C. シリアルモニタへの出力 (限界まで細かく) ---
    // %3.f = 小数点以下3桁 (0.001 hPa = 0.1 Pa 単位)
    Serial.printf("Pres:%.3f hPa, Temp:%.2f C, BaroAlt:%.2f m, UltraRaw:%u cm, UltraAlt:%.2f m\n", 
                  pressure, temperature, baro_altitude, ultra_raw_cm, ultra_altitude);

    // --- D. BLE送信 (Bluefruitアプリ用) ---
    if (deviceConnected) {
      char bleMessage[128];
      
      // P:%.3f hPa でセンサーの限界分解能(0.001 hPa)を出力
      snprintf(bleMessage, sizeof(bleMessage), "US:%ucm(%.2fm) P:%.3fhPa T:%.2fC Alt:%.2fm\n", 
               ultra_raw_cm, ultra_altitude, pressure, temperature, baro_altitude);
      
      pTxCharacteristic->setValue((uint8_t*)bleMessage, strlen(bleMessage));
      pTxCharacteristic->notify();
    }
  }
}