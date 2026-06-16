#include <Adafruit_BNO08x.h>
#include <esp_now.h>
#include <FS.h>
#include <HardwareSerial.h>
#include <SD.h>
#include <SPI.h>
#include <time.h>
#include <WiFi.h>
#include <Wire.h>

#include "ModbusConfig.h"
#include "ModbusMaster.h"

constexpr uint8_t kRs485DePin = D2;
constexpr uint8_t kModbusRxPin = D7;  // GPIO20 (UART0 default RX)
constexpr uint8_t kModbusTxPin = D6;  // GPIO21 (UART0 default TX)
constexpr uint8_t kLoggerLedPin = D1;
constexpr uint8_t kSpiSckPin = 8;
constexpr uint8_t kSpiMisoPin = 9;
constexpr uint8_t kSpiMosiPin = 10;
constexpr uint8_t kSdChipSelectPin = 5;

// NTP 時刻同期（WiFiテザリング等）
// WiFi 環境がない場合は接続タイムアウト後にスキップされる
constexpr const char* kNtpSsid         = "furuhashi";   // ← SSIDを入力
constexpr const char* kNtpPassword     = "oldbridge";   // ← パスワードを入力
constexpr long        kNtpGmtOffsetSec = 9L * 3600L;    // JST = UTC+9
constexpr uint32_t    kNtpWifiTimeoutMs  = 30000;        // WiFi接続タイムアウト（テザリング起動待ちを含む）
constexpr uint32_t    kNtpSyncTimeoutMs  = 10000;        // NTP応答タイムアウト

// RS485 が連続でこの回数失敗したら ESP-NOW にフォールバック
constexpr uint32_t kModbusFailureThreshold = 3;
// ESP-NOW パケットがこの時間 (ms) 以内に届いていれば有効とみなす
constexpr uint32_t kEspNowStaleMs = 2000;
constexpr uint8_t kAirDataEspNowDeviceId = 0x01;
constexpr uint8_t kWindEspNowDeviceId = 0x03;

struct EspNowLegacyPacket {
  uint8_t  deviceId;
  uint8_t  reserved;
  uint16_t windSpeed;
  uint16_t as5600Primary;
  uint16_t as5600Secondary;
  uint16_t batteryRaw;
  uint32_t sequenceNumber;
};

struct EspNowAirDataPacket {
  uint8_t  deviceId;
  uint8_t  reserved;
  uint16_t windSpeed;
  uint16_t pulseCountMin;
  uint16_t pulseCountMax;
  uint16_t pulseCountTotal; // 0.5 s 窓の 20 Hz サンプル総パルス数
  uint16_t as5600Primary;
  uint16_t as5600Secondary;
  uint16_t batteryRaw;
  uint32_t sequenceNumber;
};

namespace {
constexpr const char* kLogFilePath = "/flight_log.csv";
constexpr unsigned long kLedBlinkIntervalMs = 500;
constexpr unsigned long kLedFastBlinkIntervalMs = 100;

HardwareSerial MySerial0(0);
ModbusMaster g_master(MySerial0, kRs485DePin, kModbusRxPin, kModbusTxPin);
uint16_t g_airDataBuffer[kAirDataReadCount] = {0};
uint16_t g_displayBuffer[kDisplayReadCount] = {0};
uint16_t g_pendingAirspeed = 0;  // writeHreg に渡すポインタの寿命を保証
bool g_sdReady = false;
bool g_lastAirOk = false;
bool g_lastDisplayOk = false;
bool g_lastCycleOk = true;
bool g_ledState = false;
unsigned long g_lastAirReadAt = 0;
unsigned long g_lastAirWriteAt = 0;
unsigned long g_lastDisplayReadAt = 0;
unsigned long g_lastLogAt = 0;
unsigned long g_lastPollAt = 0;
unsigned long g_lastLedToggleAt = 0;
EspNowAirDataPacket g_airEspNowLatest = {};
EspNowLegacyPacket g_windEspNowLatest = {};
unsigned long g_airEspNowLastReceivedAt = 0;
unsigned long g_windEspNowLastReceivedAt = 0;
uint32_t g_modbusConsecutiveFailures = 0;
unsigned long g_lastAlarmWriteAt = 0;
uint16_t g_lastRollAlarmValue = 0xFFFF;

const uint8_t kBroadcastAddress[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint32_t g_espNowSendSeq = 0;

// ---- DS3231 RTC (Wire直接使用) ----------------------------------------
constexpr uint8_t kRtcI2cAddr = 0x68;
bool g_rtcReady = false;

static uint8_t bcdEncode(uint8_t val) { return ((val / 10) << 4) | (val % 10); }
static uint8_t bcdDecode(uint8_t bcd) { return (bcd >> 4) * 10 + (bcd & 0x0F); }

// NTP でRTCを更新する。WiFiが使えない場合は静かにスキップする。
// ESP-NOW初期化の前に呼び出すこと（干渉防止）。
static void syncTimeViaNtp() {
  if (strlen(kNtpSsid) == 0 || strcmp(kNtpSsid, "YOUR_SSID") == 0) {
    Serial.println("[NTP] SSID not configured, skip");
    return;
  }

  Serial.print("[NTP] connecting to ");
  Serial.print(kNtpSsid);
  Serial.print(" ...");

  WiFi.disconnect(true);   // 前回の接続残骸をクリア
  WiFi.mode(WIFI_OFF);
  delay(100);
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_21dBm);
  WiFi.setMinSecurity(WIFI_AUTH_WPA2_PSK);  // WPA3 SAE 非互換対策：WPA2 に固定
  WiFi.begin(kNtpSsid, kNtpPassword);

  const uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < kNtpWifiTimeoutMs) {
    delay(200);
    Serial.print('.');
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(" FAILED (timeout), skip NTP");
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    return;
  }
  Serial.println(" connected");

  // NTP サーバに問い合わせ（冗長化のため2サーバ指定）
  configTime(kNtpGmtOffsetSec, 0, "ntp.nict.jp", "pool.ntp.org");

  struct tm ti;
  if (!getLocalTime(&ti, kNtpSyncTimeoutMs)) {
    Serial.println("[NTP] getLocalTime timeout, skip");
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    return;
  }

  // RTC (DS3231) に書き込む
  Wire.beginTransmission(kRtcI2cAddr);
  Wire.write(0x00);
  Wire.write(bcdEncode((uint8_t)ti.tm_sec));
  Wire.write(bcdEncode((uint8_t)ti.tm_min));
  Wire.write(bcdEncode((uint8_t)ti.tm_hour));
  Wire.write(0x01);  // 曜日（未使用）
  Wire.write(bcdEncode((uint8_t)ti.tm_mday));
  Wire.write(bcdEncode((uint8_t)(ti.tm_mon + 1)));
  Wire.write(bcdEncode((uint8_t)(ti.tm_year - 100)));  // 年下2桁
  if (Wire.endTransmission() == 0) {
    Serial.printf("[NTP] RTC synced: %04d-%02d-%02d %02d:%02d:%02d\n",
                  ti.tm_year + 1900, ti.tm_mon + 1, ti.tm_mday,
                  ti.tm_hour, ti.tm_min, ti.tm_sec);
  } else {
    Serial.println("[NTP] RTC write failed");
  }

  // WiFi を完全切断してから ESP-NOW 初期化に備える
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(100);  // WiFiスタック安定待ち
}

// OSF フラグを確認し、オシレータが止まっていた場合のみコンパイル時刻を書き込む
static void initRtc() {
  // ── Step 1: コントロールレジスタ 0x0E の EOSC ビット(bit7)を無条件クリア ──
  // EOSC=1 だとバッテリーバックアップ時に発振が停止する。
  // 毎回起動時に確認・クリアすることで、OSFが連鎖的にセットされ続けるのを防ぐ。
  Wire.beginTransmission(kRtcI2cAddr);
  Wire.write(0x0E);
  if (Wire.endTransmission() != 0 || Wire.requestFrom((uint8_t)kRtcI2cAddr, (uint8_t)1) < 1) {
    Serial.println("[RTC] control reg read FAILED");
    g_rtcReady = false;
    return;
  }
  const uint8_t ctrlReg = Wire.read();
  Serial.printf("[RTC] ctrl=0x%02X EOSC=%d\n", ctrlReg, (ctrlReg >> 7) & 1);
  if (ctrlReg & 0x80) {
    Wire.beginTransmission(kRtcI2cAddr);
    Wire.write(0x0E);
    Wire.write(ctrlReg & ~0x80);
    if (Wire.endTransmission() != 0) {
      Serial.println("[RTC] EOSC clear FAILED");
      g_rtcReady = false;
      return;
    }
    Serial.println("[RTC] EOSC cleared");
  }

  // ── Step 2: OSF フラグ（ステータスレジスタ 0x0F の bit7）を読む ──
  Wire.beginTransmission(kRtcI2cAddr);
  Wire.write(0x0F);
  if (Wire.endTransmission() != 0 || Wire.requestFrom((uint8_t)kRtcI2cAddr, (uint8_t)1) < 1) {
    g_rtcReady = false;
    return;
  }
  const uint8_t statusReg = Wire.read();
  g_rtcReady = true;
  Serial.printf("[RTC] status=0x%02X OSF=%d\n", statusReg, (statusReg >> 7) & 1);

  // ── Step 3: コンパイル時刻を計算 ──
  const char* months = "JanFebMarAprMayJunJulAugSepOctNovDec";
  char mon[4] = {__DATE__[0], __DATE__[1], __DATE__[2], '\0'};
  const char* found = strstr(months, mon);
  uint8_t compileMonth = found ? static_cast<uint8_t>((found - months) / 3 + 1) : 1;
  uint8_t compileDay   = (__DATE__[4] == ' ' ? 0 : __DATE__[4] - '0') * 10 + (__DATE__[5] - '0');
  uint8_t compileYear  = (__DATE__[9] - '0') * 10 + (__DATE__[10] - '0');
  uint8_t compileHour  = (__TIME__[0] - '0') * 10 + (__TIME__[1] - '0');
  uint8_t compileMin   = (__TIME__[3] - '0') * 10 + (__TIME__[4] - '0');
  uint8_t compileSec   = (__TIME__[6] - '0') * 10 + (__TIME__[7] - '0');
  Serial.printf("[RTC] compile=20%02d-%02d-%02d %02d:%02d:%02d\n",
                compileYear, compileMonth, compileDay,
                compileHour, compileMin, compileSec);

  const uint32_t compileSerial = ((uint32_t)compileYear  * 100000000UL)
                                + ((uint32_t)compileMonth * 1000000UL)
                                + ((uint32_t)compileDay   * 10000UL)
                                + ((uint32_t)compileHour  * 100UL)
                                + ((uint32_t)compileMin);

  // ── Step 4: OSF=0 かつ RTC 時刻がコンパイル時刻以降なら書き込み不要 ──
  if ((statusReg & 0x80) == 0) {
    Wire.beginTransmission(kRtcI2cAddr);
    Wire.write(0x00);
    if (Wire.endTransmission() == 0 && Wire.requestFrom((uint8_t)kRtcI2cAddr, (uint8_t)7) >= 7) {
      Wire.read();  // sec（スキップ）
      uint8_t rtcMin   = bcdDecode(Wire.read() & 0x7F);
      uint8_t rtcHour  = bcdDecode(Wire.read() & 0x3F);
      Wire.read();  // 曜日（スキップ）
      uint8_t rtcDay   = bcdDecode(Wire.read() & 0x3F);
      uint8_t rtcMonth = bcdDecode(Wire.read() & 0x1F);
      uint8_t rtcYear  = bcdDecode(Wire.read());
      const uint32_t rtcSerial = ((uint32_t)rtcYear  * 100000000UL)
                                + ((uint32_t)rtcMonth * 1000000UL)
                                + ((uint32_t)rtcDay   * 10000UL)
                                + ((uint32_t)rtcHour  * 100UL)
                                + ((uint32_t)rtcMin);
      Serial.printf("[RTC] current=20%02d-%02d-%02d %02d:%02d (serial=%lu)\n",
                    rtcYear, rtcMonth, rtcDay, rtcHour, rtcMin, rtcSerial);
      if (rtcSerial >= compileSerial) {
        return;
      }
      Serial.println("[RTC] time older than compile time, updating");
    }
  } else {
    Serial.println("[RTC] OSF set: oscillator was stopped, writing compile time");
  }

  // ── Step 5: RTC に時刻を書き込む ──
  Wire.beginTransmission(kRtcI2cAddr);
  Wire.write(0x00);
  Wire.write(bcdEncode(compileSec));
  Wire.write(bcdEncode(compileMin));
  Wire.write(bcdEncode(compileHour));
  Wire.write(0x01);              // 曜日（未使用）
  Wire.write(bcdEncode(compileDay));
  Wire.write(bcdEncode(compileMonth));
  Wire.write(bcdEncode(compileYear));
  if (Wire.endTransmission() != 0) {
    g_rtcReady = false;
    return;
  }

  // ── Step 6: OSF フラグをクリア ──
  Wire.beginTransmission(kRtcI2cAddr);
  Wire.write(0x0F);
  Wire.write(statusReg & ~0x80);
  if (Wire.endTransmission() != 0) {
    Serial.println("[RTC] OSF clear FAILED");
    g_rtcReady = false;
    return;
  }
  Serial.println("[RTC] compile time written, OSF cleared");
}

// "YYYY-MM-DD HH:MM:SS" を取得。失敗時は "BOOT+XXXXXms" にフォールバック
static void getRtcTimestamp(char* buf, size_t bufLen) {
  Wire.beginTransmission(kRtcI2cAddr);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0 || Wire.requestFrom((uint8_t)kRtcI2cAddr, (uint8_t)7) < 7) {
    snprintf(buf, bufLen, "BOOT+%lums", millis());
    return;
  }
  uint8_t sec   = bcdDecode(Wire.read() & 0x7F);
  uint8_t min_  = bcdDecode(Wire.read() & 0x7F);
  uint8_t hour  = bcdDecode(Wire.read() & 0x3F);
  Wire.read();  // 曜日（スキップ）
  uint8_t date  = bcdDecode(Wire.read() & 0x3F);
  uint8_t month = bcdDecode(Wire.read() & 0x1F);
  uint8_t year  = bcdDecode(Wire.read());
  snprintf(buf, bufLen, "20%02u-%02u-%02u %02u:%02u:%02u",
           year, month, date, hour, min_, sec);
}

// ---- BNO085 IMU (I2C) -------------------------------------------------
Adafruit_BNO08x g_bno08x(-1);
bool g_imuReady = false;
static unsigned long g_lastImuRetryTime = 0;
static int g_imuFailCount = 0;

struct {
  float roll  = 0.0f;
  float pitch = 0.0f;
  float yaw   = 0.0f;
} g_imuData;

static void readIMUData() {
  if (!g_imuReady) {
    unsigned long now = millis();
    if (now - g_lastImuRetryTime > 5000) {
      g_lastImuRetryTime = now;
      Serial.println("[logger] Retrying BNO085 initialization...");
      if (g_bno08x.begin_I2C()) {
        g_imuReady = g_bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 5000);
        if (g_imuReady) {
          Serial.println("[logger] BNO085 Re-initialization SUCCESS");
          g_imuFailCount = 0;
        } else {
          Serial.println("[logger] BNO085 Re-initialization FAILED to enable report");
        }
      } else {
        Serial.println("[logger] BNO085 Re-initialization FAILED to begin");
      }
    }
    return;
  }

  if (g_bno08x.wasReset()) {
    g_imuReady = g_bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 5000);
    if (!g_imuReady) {
      Serial.println("[logger] BNO085 reset detected, but failed to re-enable report");
    }
    return;
  }

  sh2_SensorValue_t ev;
  if (!g_bno08x.getSensorEvent(&ev)) {
    g_imuFailCount++;
    if (g_imuFailCount > 100) { // 連続失敗時の安全策。一時的な不調なら再初期化に回す
      Serial.println("[logger] BNO085 consecutive reading failures. Resetting state for retry...");
      g_imuReady = false;
      g_imuFailCount = 0;
    }
    return;
  }
  g_imuFailCount = 0;

  if (ev.sensorId != SH2_ARVR_STABILIZED_RV) return;

  const float qr = ev.un.arvrStabilizedRV.real;
  const float qi = ev.un.arvrStabilizedRV.i;
  const float qj = ev.un.arvrStabilizedRV.j;
  const float qk = ev.un.arvrStabilizedRV.k;
  const float sqr = qr * qr, sqi = qi * qi, sqj = qj * qj, sqk = qk * qk;

  g_imuData.yaw   = atan2(2.0f * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr)) * RAD_TO_DEG;
  g_imuData.pitch = asin(-2.0f * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr)) * RAD_TO_DEG;
  g_imuData.roll  = atan2(2.0f * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr)) * RAD_TO_DEG;
}

static void writeRollAlarm() {
  uint16_t alarm = 0;
  if (g_imuData.roll < -5.0f) {
    alarm = 1;  // 左ロール（L-ALARM）
  } else if (g_imuData.roll > 5.0f) {
    alarm = 2;  // 右ロール（R-ALARM）
  }

  if (alarm == g_lastRollAlarmValue) {
    return;
  }

  if (g_master.writeHoldingRegisters(kDisplaySlaveId, kDisplayRollAlarmWriteRegister, &alarm, 1)) {
    g_lastRollAlarmValue = alarm;
  } else {
    g_lastCycleOk = false;
  }
}

// --- Modbus コールバック ---

bool cbAir(Modbus::ResultCode event, uint16_t, void*) {
  g_lastAirOk = (event == Modbus::EX_SUCCESS);
  if (!g_lastAirOk) {
    for (uint16_t& v : g_airDataBuffer) v = 0;
  }
  return true;
}

bool cbWrite(Modbus::ResultCode, uint16_t, void*) {
  return true;
}

bool cbDisplay(Modbus::ResultCode event, uint16_t, void*) {
  g_lastDisplayOk = (event == Modbus::EX_SUCCESS);
  if (!g_lastDisplayOk) {
    for (uint16_t& v : g_displayBuffer) v = 0;
  }
  return true;
}

// --- SD / ログ ---

void initSdCard() {
  if (!SD.begin(kSdChipSelectPin)) {
    g_sdReady = false;
    return;
  }

  if (!SD.exists(kLogFilePath)) {
    File file = SD.open(kLogFilePath, FILE_WRITE);
    if (file) {
      file.println("timestamp,airspeed,pulse_min,pulse_max,pulse_total,wind_board_airspeed,as5600_1,as5600_2,air_battery,baro_alt,pot1,pot2,display_battery,ultrasonic,roll,pitch,yaw");
      file.close();
    }
  }

  g_sdReady = true;
}

void pollDevices() {
  g_lastCycleOk = true;
  g_lastAirOk = g_master.readHoldingRegisters(kAirDataSlaveId, kAirDataReadStart, g_airDataBuffer, kAirDataReadCount);
  if (!g_lastAirOk) {
    g_modbusConsecutiveFailures++;
    g_lastCycleOk = false;
    const bool espNowFresh = (millis() - g_airEspNowLastReceivedAt) < kEspNowStaleMs;
    if (g_modbusConsecutiveFailures >= kModbusFailureThreshold && espNowFresh) {
      // RS485 失敗: ESP-NOW の最終受信値をフォールバックとして使用
      g_airDataBuffer[0] = g_airEspNowLatest.windSpeed;
      g_airDataBuffer[1] = g_airEspNowLatest.as5600Primary;
      g_airDataBuffer[2] = g_airEspNowLatest.as5600Secondary;
      g_airDataBuffer[3] = g_airEspNowLatest.batteryRaw;
    } else {
      for (uint16_t& value : g_airDataBuffer) {
        value = 0;
      }
    }
  } else {
    g_modbusConsecutiveFailures = 0;
  }

  const uint16_t airspeed = g_airDataBuffer[0];
  g_lastDisplayOk = g_master.writeHoldingRegisters(kDisplaySlaveId, kDisplayAirspeedWriteRegister, &airspeed, 1);
  if (!g_lastDisplayOk) {
    g_lastCycleOk = false;
  }

  if (!g_master.readHoldingRegisters(kDisplaySlaveId, kDisplayReadStart, g_displayBuffer, kDisplayReadCount)) {
    g_lastCycleOk = false;
    for (uint16_t& value : g_displayBuffer) {
      value = 0;
    }
  }
}

void writeLogRecord() {
  if (millis() - g_lastLogAt < kLogIntervalMs) return;
  g_lastLogAt = millis();

  char timestamp[24];
  getRtcTimestamp(timestamp, sizeof(timestamp));

  const bool airEspNowFresh = (millis() - g_airEspNowLastReceivedAt) < kEspNowStaleMs;
  const uint16_t airPulseMin   = airEspNowFresh ? g_airEspNowLatest.pulseCountMin   : 0;
  const uint16_t airPulseMax   = airEspNowFresh ? g_airEspNowLatest.pulseCountMax   : 0;
  const uint16_t airPulseTotal = airEspNowFresh ? g_airEspNowLatest.pulseCountTotal : 0;

  char record[512];
  snprintf(record,
           sizeof(record),
           "%s,%.1f,%u,%u,%u,%.1f,%u,%u,%u,%u,%u,%u,%u,%u,%.2f,%.2f,%.2f\n",
           timestamp,
           g_airDataBuffer[0] / 10.0f,
           airPulseMin,
           airPulseMax,
           airPulseTotal,
           g_windEspNowLatest.windSpeed / 10.0f,
           g_airDataBuffer[1],
           g_airDataBuffer[2],
           g_airDataBuffer[3],
           g_displayBuffer[0],
           g_displayBuffer[1],
           g_displayBuffer[2],
           g_displayBuffer[3],
           g_displayBuffer[4],
           g_imuData.roll, g_imuData.pitch, g_imuData.yaw);

  if (!g_sdReady) {
    g_lastCycleOk = false;
    return;
  }

  File file = SD.open(kLogFilePath, FILE_APPEND);
  if (!file) {
    g_lastCycleOk = false;
    g_sdReady = false;
    return;
  }

  file.print(record);
  file.close();
}

void sendDataByEspNow() {
  EspNowAirDataPacket packet = {};
  packet.deviceId = kAirDataEspNowDeviceId; // 0x01
  packet.reserved = 0;
  packet.windSpeed = g_airDataBuffer[0];
  
  const bool airEspNowFresh = (millis() - g_airEspNowLastReceivedAt) < kEspNowStaleMs;
  packet.pulseCountMin = airEspNowFresh ? g_airEspNowLatest.pulseCountMin : 0;
  packet.pulseCountMax = airEspNowFresh ? g_airEspNowLatest.pulseCountMax : 0;
  packet.pulseCountTotal = airEspNowFresh ? g_airEspNowLatest.pulseCountTotal : 0;

  packet.as5600Primary = g_displayBuffer[1];    // pot1
  packet.as5600Secondary = g_displayBuffer[2];  // pot2
  packet.batteryRaw = g_airDataBuffer[3];
  packet.sequenceNumber = g_espNowSendSeq++;

  esp_now_send(kBroadcastAddress, reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
}

}  // namespace

void onEspNowReceive(const esp_now_recv_info_t* /*recvInfo*/, const uint8_t* data, int len) {
  if (len < 1) {
    return;
  }
  const uint8_t deviceId = data[0];
  if (deviceId == kAirDataEspNowDeviceId) {
    if (len == static_cast<int>(sizeof(EspNowAirDataPacket))) {
      g_airEspNowLatest = *reinterpret_cast<const EspNowAirDataPacket*>(data);
    } else if (len == static_cast<int>(sizeof(EspNowLegacyPacket))) {
      const EspNowLegacyPacket* legacyPacket = reinterpret_cast<const EspNowLegacyPacket*>(data);
      g_airEspNowLatest.deviceId = legacyPacket->deviceId;
      g_airEspNowLatest.reserved = legacyPacket->reserved;
      g_airEspNowLatest.windSpeed = legacyPacket->windSpeed;
      g_airEspNowLatest.pulseCountMin = 0;
      g_airEspNowLatest.pulseCountMax = 0;
      g_airEspNowLatest.pulseCountTotal = 0;
      g_airEspNowLatest.as5600Primary = legacyPacket->as5600Primary;
      g_airEspNowLatest.as5600Secondary = legacyPacket->as5600Secondary;
      g_airEspNowLatest.batteryRaw = legacyPacket->batteryRaw;
      g_airEspNowLatest.sequenceNumber = legacyPacket->sequenceNumber;
    } else {
      return;
    }
    g_airEspNowLastReceivedAt = millis();
    return;
  }
  if (deviceId == kWindEspNowDeviceId) {
    if (len != static_cast<int>(sizeof(EspNowLegacyPacket))) {
      return;
    }
    g_windEspNowLatest = *reinterpret_cast<const EspNowLegacyPacket*>(data);
    g_windEspNowLastReceivedAt = millis();
  }
}

namespace {
void updateLed() {
  // SDエラー: 点灯
  if (!g_sdReady) {
    digitalWrite(kLoggerLedPin, HIGH);
    return;
  }

  // Modbus通信中: 早く点滅
  if (g_lastAirOk) {
    if (millis() - g_lastLedToggleAt >= kLedFastBlinkIntervalMs) {
      g_lastLedToggleAt = millis();
      g_ledState = !g_ledState;
      digitalWrite(kLoggerLedPin, g_ledState ? HIGH : LOW);
    }
    return;
  }

  // ESP-NOW受信中: 点滅
  const bool espNowFresh = (millis() - g_airEspNowLastReceivedAt) < kEspNowStaleMs;
  if (espNowFresh) {
    if (millis() - g_lastLedToggleAt >= kLedBlinkIntervalMs) {
      g_lastLedToggleAt = millis();
      g_ledState = !g_ledState;
      digitalWrite(kLoggerLedPin, g_ledState ? HIGH : LOW);
    }
    return;
  }

  digitalWrite(kLoggerLedPin, LOW);
}
}

void setup() {
  Serial.begin(kDebugBaudRate);
  delay(100);

  pinMode(kLoggerLedPin, OUTPUT);
  digitalWrite(kLoggerLedPin, LOW);

  SPI.begin(kSpiSckPin, kSpiMisoPin, kSpiMosiPin, kSdChipSelectPin);
  initSdCard();
  g_master.begin();

  Wire.begin();
  Wire.setTimeOut(150); // BNO085のクロックストレッチング対策（I2Cタイムアウトを150msに引き上げ）
  initRtc();
  syncTimeViaNtp();  // NTP同期（WiFi利用可能時のみ）。ESP-NOW初期化より前に実行。
  Serial.printf("[logger] RTC %s\n", g_rtcReady ? "OK" : "FAILED");

  if (g_bno08x.begin_I2C()) {
    g_imuReady = g_bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 5000);
    Serial.printf("[logger] BNO085 %s\n", g_imuReady ? "OK" : "report FAILED");
  } else {
    Serial.println("[logger] BNO085 init FAILED");
  }

  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_21dBm);
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(onEspNowReceive);

    // 送信ピアの登録（デュアルモニターへのブロードキャスト転送用）
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, kBroadcastAddress, 6);
    peer.channel = 0;
    peer.encrypt = false;
    if (esp_now_add_peer(&peer) == ESP_OK) {
      Serial.println("[logger] ESP-NOW broadcast peer added");
    } else {
      Serial.println("[logger] ESP-NOW failed to add peer");
    }

    Serial.println("[logger] ESP-NOW init OK");
  } else {
    Serial.println("[logger] ESP-NOW init FAILED");
  }
}

void loop() {
  g_master.task();
  readIMUData();

  if (millis() - g_lastAlarmWriteAt >= kRollAlarmWriteIntervalMs) {
    g_lastAlarmWriteAt = millis();
    writeRollAlarm();
  }

  if (millis() - g_lastPollAt >= kLoggerPollIntervalMs) {
    g_lastPollAt = millis();
    pollDevices();
    sendDataByEspNow();
    const bool usingFallback = g_modbusConsecutiveFailures >= kModbusFailureThreshold
                              && (millis() - g_airEspNowLastReceivedAt) < kEspNowStaleMs;
    char ts[24];
    getRtcTimestamp(ts, sizeof(ts));
    Serial.printf("[logger] %s  sd=%s  cycle=%s  src=%s  air=[spd=%.1f pmin=%u pmax=%u as1=%u as2=%u batt=%u]  wind=[spd=%.1f seq=%lu]  disp=[baro=%u pot1=%u pot2=%u batt=%u ultra=%u]  imu=[r=%.1f p=%.1f y=%.1f]\n",
      ts,
      g_sdReady ? "OK" : "ERR",
      g_lastCycleOk ? "OK" : "ERR",
      usingFallback ? "ESPNOW" : "RS485",
      g_airDataBuffer[0] / 10.0f, g_airEspNowLatest.pulseCountMin, g_airEspNowLatest.pulseCountMax,
      g_airDataBuffer[1], g_airDataBuffer[2], g_airDataBuffer[3],
      g_windEspNowLatest.windSpeed / 10.0f, static_cast<unsigned long>(g_windEspNowLatest.sequenceNumber),
      g_displayBuffer[0], g_displayBuffer[1], g_displayBuffer[2], g_displayBuffer[3], g_displayBuffer[4],
      g_imuData.roll, g_imuData.pitch, g_imuData.yaw);
  }

  writeLogRecord();
  updateLed();
}
