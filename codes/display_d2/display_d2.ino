#include <HardwareSerial.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <Wire.h>

#include "DisplayD2Config.h"

constexpr uint8_t kPot1Pin = A0;
constexpr uint8_t kPot2Pin = A1;
constexpr uint8_t kBatteryPin = A2;
// HC-SR04はSerial0(D6=RX, D7=TX)を使用 - ピン指定不要
constexpr uint8_t kStatusLedPin = D10;
constexpr uint8_t kBuzzerPin = D3;
constexpr uint8_t kTftSckPin = 8;
constexpr uint8_t kTftMisoPin = 9;
constexpr uint8_t kTftMosiPin = 10;
constexpr uint32_t kUltrasonicBaudRate = 9600;
constexpr uint32_t kUltrasonicResponseTimeoutMs = 250;  // センサ測定時間に余裕を持たせる
constexpr uint16_t kUltrasonicInvalidReading = 0xFFFF;
constexpr uint16_t kDisplayBackground = TFT_BLACK;
constexpr uint16_t kDisplayForeground = TFT_WHITE;
constexpr uint16_t kDisplayAccent = TFT_CYAN;
constexpr uint16_t kDisplayWarning = TFT_RED;
constexpr uint8_t kUltrasonicCmd = 0xA0;

struct DisplayD2Payload {
  uint16_t potentiometer1;
  uint16_t potentiometer2;
  uint16_t batteryVoltage;
  uint16_t ultrasonicAlt;
};


constexpr uint32_t kDebugPrintIntervalMs = 1000;
constexpr uint32_t kDisplayRefreshIntervalMs = 100;

namespace {
// HC-SR04: Serial0 (XIAO ESP32-C3 デフォルトUART D6/D7)
TFT_eSPI g_tft;
DisplayD2Payload g_payload = {0, 0, 0, 0};
unsigned long g_lastUpdateAt = 0;
unsigned long g_lastDebugAt = 0;
unsigned long g_lastRenderAt = 0;
volatile bool g_rollAlarm = false;
bool g_ultrasonicValid = false;

bool readUltrasonicDistance(uint16_t& distanceCm) {
  while (Serial0.available() > 0) {
    Serial0.read();
  }

  Serial0.write(kUltrasonicCmd);

  const unsigned long startedAt = millis();
  while (Serial0.available() < 3) {
    if (millis() - startedAt > kUltrasonicResponseTimeoutMs) {
      Serial.printf("[us] timeout: available=%d after %lums\n",
        Serial0.available(), millis() - startedAt);
      return false;
    }
    yield();
  }

  const uint8_t byteH = static_cast<uint8_t>(Serial0.read());
  const uint8_t byteM = static_cast<uint8_t>(Serial0.read());
  const uint8_t byteL = static_cast<uint8_t>(Serial0.read());

  Serial.printf("[us] raw: H=0x%02X M=0x%02X L=0x%02X\n", byteH, byteM, byteL);

  const uint32_t rawUm = ((uint32_t)byteH << 16) | ((uint32_t)byteM << 8) | byteL;
  if (rawUm == 0) {
    Serial.println("[us] rawUm==0, skip");
    return false;
  }

  // rawはμm単位 → cmに変換
  distanceCm = static_cast<uint16_t>(rawUm / 10000);
  return true;
}

void writePayload() {
  uint8_t buffer[kDisplayD2PayloadSize] = {0};
  buffer[0] = static_cast<uint8_t>(g_payload.potentiometer1 & 0xFF);
  buffer[1] = static_cast<uint8_t>(g_payload.potentiometer1 >> 8);
  buffer[2] = static_cast<uint8_t>(g_payload.potentiometer2 & 0xFF);
  buffer[3] = static_cast<uint8_t>(g_payload.potentiometer2 >> 8);
  buffer[4] = static_cast<uint8_t>(g_payload.batteryVoltage & 0xFF);
  buffer[5] = static_cast<uint8_t>(g_payload.batteryVoltage >> 8);
  buffer[6] = static_cast<uint8_t>(g_payload.ultrasonicAlt & 0xFF);
  buffer[7] = static_cast<uint8_t>(g_payload.ultrasonicAlt >> 8);
  Wire.write(buffer, kDisplayD2PayloadSize);
}

void updateSensors() {
  if (millis() - g_lastUpdateAt < kDisplayD2SensorIntervalMs) {
    return;
  }
  g_lastUpdateAt = millis();

  g_payload.potentiometer1 = analogRead(kPot1Pin);
  g_payload.potentiometer2 = analogRead(kPot2Pin);
  g_payload.batteryVoltage = analogRead(kBatteryPin);

  uint16_t distanceCm = 0;
  if (readUltrasonicDistance(distanceCm)) {
    g_payload.ultrasonicAlt = distanceCm;
    g_ultrasonicValid = true;
  } else {
    g_ultrasonicValid = false;
  }
}

void renderDisplay() {
  if (millis() - g_lastRenderAt < kDisplayRefreshIntervalMs) {
    return;
  }
  g_lastRenderAt = millis();

  g_tft.fillScreen(kDisplayBackground);
  g_tft.setTextColor(kDisplayForeground, kDisplayBackground);

  char line[40];
  g_tft.drawCentreString("DISPLAY D2", 120, 8, 2);

  g_tft.setTextColor(g_ultrasonicValid ? kDisplayAccent : kDisplayWarning, kDisplayBackground);
  if (g_ultrasonicValid) {
    snprintf(line, sizeof(line), "US: %u cm", g_payload.ultrasonicAlt);
  } else {
    snprintf(line, sizeof(line), "US: ----");
  }
  g_tft.drawString(line, 8, 40, 4);

  g_tft.setTextColor(kDisplayForeground, kDisplayBackground);
  snprintf(line, sizeof(line), "P1:%4u  P2:%4u", g_payload.potentiometer1, g_payload.potentiometer2);
  g_tft.drawString(line, 8, 96, 2);

  snprintf(line, sizeof(line), "BATT:%4u", g_payload.batteryVoltage);
  g_tft.drawString(line, 8, 118, 2);

  g_tft.setTextColor(g_rollAlarm ? kDisplayWarning : TFT_GREEN, kDisplayBackground);
  snprintf(line, sizeof(line), "ROLL:%s", g_rollAlarm ? "ALARM" : "OK");
  g_tft.drawString(line, 8, 140, 2);
}

void printDebug() {
  if (millis() - g_lastDebugAt < kDebugPrintIntervalMs) {
    return;
  }
  g_lastDebugAt = millis();
  Serial.printf("[display_d2] pot1=%u  pot2=%u  batt=%u  ultra=%u  us=%s  roll_alarm=%s\n",
    g_payload.potentiometer1,
    g_payload.potentiometer2,
    g_payload.batteryVoltage,
    g_payload.ultrasonicAlt,
    g_ultrasonicValid ? "OK" : "NG",
    g_rollAlarm ? "ON" : "OFF");
}
}

void onI2CReceive(int len) {
  if (len < 1) return;
  g_rollAlarm = (Wire.read() != 0);
  while (Wire.available()) Wire.read();  // 余分バイトを捨てる
}

void setup() {
  Serial.begin(kDebugBaudRate);
  delay(1000);  // USB CDC (XIAO ESP32-C3) の接続待ち
  Serial.println("[d2] setup start");
  pinMode(kStatusLedPin, OUTPUT);
  digitalWrite(kStatusLedPin, HIGH);
  delay(100);

  pinMode(kBuzzerPin, OUTPUT);
  pinMode(kPot1Pin, INPUT);
  pinMode(kPot2Pin, INPUT);
  pinMode(kBatteryPin, INPUT);

  Serial0.begin(kUltrasonicBaudRate);  // HC-SR04: D6=RX, D7=TX (ピン指定不要)

  // SPI.begin() は TFT_eSPI が内部で初期化するため不要 (呼ぶと競合してクラッシュ)
  g_tft.init();
  g_tft.setRotation(0);
  g_tft.fillScreen(kDisplayBackground);
  g_tft.setTextColor(kDisplayForeground, kDisplayBackground);
  g_tft.drawCentreString("DISPLAY D2", 120, 8, 2);

  Wire.begin(kDisplayD2I2CAddress);
  Wire.onRequest(writePayload);
  Wire.onReceive(onI2CReceive);
}

void loop() {
  updateSensors();
  renderDisplay();
  printDebug();

  if (g_rollAlarm) {
    tone(kBuzzerPin, 1000);
  } else {
    noTone(kBuzzerPin);
  }
}
