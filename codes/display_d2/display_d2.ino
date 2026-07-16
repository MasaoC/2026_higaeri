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
// 移動平均（Smoothing）フィルタ
template <size_t N = 10>
class SmoothingFilter {
public:
  SmoothingFilter() {
    clear();
  }

  void clear() {
    for (size_t i = 0; i < N; ++i) {
      m_readings[i] = 0;
    }
    m_readIndex = 0;
    m_total = 0;
    m_average = 0;
    m_initialized = false;
  }

  void add(int newValue) {
    if (!m_initialized) {
      for (size_t i = 0; i < N; ++i) {
        m_readings[i] = newValue;
      }
      m_total = newValue * N;
      m_average = newValue;
      m_initialized = true;
      return;
    }
    m_total = m_total - m_readings[m_readIndex];
    m_readings[m_readIndex] = newValue;
    m_total = m_total + newValue;
    m_readIndex++;
    if (m_readIndex >= N) {
      m_readIndex = 0;
    }
    m_average = m_total / N;
  }

  int get() const {
    return m_average;
  }

private:
  int m_readings[N];
  size_t m_readIndex;
  int m_total;
  int m_average;
  bool m_initialized;
};

// HC-SR04: Serial0 (XIAO ESP32-C3 デフォルトUART D6/D7)
TFT_eSPI g_tft;
DisplayD2Payload g_payload = {0, 0, 0, 0};
SmoothingFilter<10> g_pot1Filter;
SmoothingFilter<10> g_pot2Filter;
unsigned long g_lastAnalogUpdateAt = 0;
constexpr uint32_t kAnalogUpdateIntervalMs = 10;  // 10msごとにアナログサンプリングして移動平均を更新
unsigned long g_lastUpdateAt = 0;
unsigned long g_lastDebugAt = 0;
unsigned long g_lastRenderAt = 0;
volatile uint8_t g_rollAlarm = 0;  // 0: OK, 1: L-ALARM, 2: R-ALARM
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
  const unsigned long now = millis();

  // ポテンショメータの値を高頻度(10msごと)でサンプリングし、移動平均でノイズ除去・精度向上
  if (now - g_lastAnalogUpdateAt >= kAnalogUpdateIntervalMs) {
    g_lastAnalogUpdateAt = now;
    g_pot1Filter.add(analogRead(kPot1Pin));
    g_pot2Filter.add(analogRead(kPot2Pin));
    g_payload.potentiometer1 = g_pot1Filter.get();
    g_payload.potentiometer2 = g_pot2Filter.get();
    g_payload.batteryVoltage = analogRead(kBatteryPin);
  }

  // 超音波センサーは300ms間隔
  if (now - g_lastUpdateAt >= kDisplayD2SensorIntervalMs) {
    g_lastUpdateAt = now;

    uint16_t distanceCm = 0;
    if (readUltrasonicDistance(distanceCm)) {
      g_payload.ultrasonicAlt = distanceCm;
      g_ultrasonicValid = true;
    } else {
      g_ultrasonicValid = false;
    }
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

  if (g_rollAlarm == 1) {
    g_tft.setTextColor(kDisplayWarning, kDisplayBackground);
    snprintf(line, sizeof(line), "ROLL: L-ALARM");
  } else if (g_rollAlarm == 2) {
    g_tft.setTextColor(kDisplayWarning, kDisplayBackground);
    snprintf(line, sizeof(line), "ROLL: R-ALARM");
  } else {
    g_tft.setTextColor(TFT_GREEN, kDisplayBackground);
    snprintf(line, sizeof(line), "ROLL: OK");
  }
  g_tft.drawString(line, 8, 140, 2);
}

void printDebug() {
  if (millis() - g_lastDebugAt < kDebugPrintIntervalMs) {
    return;
  }
  g_lastDebugAt = millis();
  Serial.printf("[display_d2] pot1=%u  pot2=%u  batt=%u  ultra=%u  us=%s  roll_alarm=%u\n",
    g_payload.potentiometer1,
    g_payload.potentiometer2,
    g_payload.batteryVoltage,
    g_payload.ultrasonicAlt,
    g_ultrasonicValid ? "OK" : "NG",
    g_rollAlarm);
}
}

void onI2CReceive(int len) {
  if (len < 1) return;
  g_rollAlarm = Wire.read();
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

  // 起動時のメロディー (ド・ミ・ソ、またはピロッといういい感じの音)
  // 1500Hzを80ms、2000Hzを120msで鳴らして、軽快な「ピロッ♪」という起動音
  tone(kBuzzerPin, 1500);
  delay(80);
  tone(kBuzzerPin, 2000);
  delay(120);
  noTone(kBuzzerPin);

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

  if (g_rollAlarm == 1) {
    // 左ロール警告音: 高→低の下降スイープ (1400Hz→600Hz, 500ms) + 100ms無音
    const uint32_t phaseL = millis() % 600;
    if (phaseL < 500) {
      tone(kBuzzerPin, static_cast<uint32_t>(1400 - phaseL * 800 / 500));
    } else {
      noTone(kBuzzerPin);
    }
  } else if (g_rollAlarm == 2) {
    // 右ロール警告音: 低→高の上昇スイープ (600Hz→1400Hz, 500ms) + 100ms無音
    const uint32_t phaseR = millis() % 600;
    if (phaseR < 500) {
      tone(kBuzzerPin, static_cast<uint32_t>(600 + phaseR * 800 / 500));
    } else {
      noTone(kBuzzerPin);
    }
  } else {
    noTone(kBuzzerPin);
  }
}
