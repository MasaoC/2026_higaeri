#include <Wire.h>

constexpr uint8_t kI2cSdaPin = 6; // D4
constexpr uint8_t kI2cSclPin = 7; // D5
constexpr uint8_t kUltrasonicCmd = 0xA0;
constexpr uint32_t kUltrasonicResponseTimeoutMs = 250;

// D1へ送信するデータ (距離 cm)
volatile uint16_t g_ultrasonicCm = 0;
unsigned long g_lastUpdateAt = 0;

// D1からのI2Cリクエスト応答
void onRequest() {
  Wire.write((uint8_t*)&g_ultrasonicCm, sizeof(g_ultrasonicCm));
}

void setup() {
  Serial.begin(115200);
  
  // HC-SR04: Serial0 (XIAO ESP32-C3 デフォルトUART D6=RX, D7=TX)
  Serial0.begin(9600);

  // I2C Slave 初期化 (アドレス 0x30)
  Wire.onRequest(onRequest);
  Wire.begin((uint8_t)0x30, kI2cSdaPin, kI2cSclPin, 400000);
  
  Serial.println("d2: Ultrasonic Sensor Node Started.");
}

void loop() {
  // 300ms周期で測定
  if (millis() - g_lastUpdateAt >= 300) {
    g_lastUpdateAt = millis();

    // 受信バッファクリア
    while (Serial0.available() > 0) {
      Serial0.read();
    }

    // 測定コマンド送信
    Serial0.write(kUltrasonicCmd);

    // 応答待ち
    unsigned long startedAt = millis();
    while (Serial0.available() < 3) {
      if (millis() - startedAt > kUltrasonicResponseTimeoutMs) {
        return; // タイムアウト時は抜ける
      }
      yield();
    }

    // 3バイト読み取り (H, M, L)
    uint8_t byteH = static_cast<uint8_t>(Serial0.read());
    uint8_t byteM = static_cast<uint8_t>(Serial0.read());
    uint8_t byteL = static_cast<uint8_t>(Serial0.read());

    uint32_t rawUm = ((uint32_t)byteH << 16) | ((uint32_t)byteM << 8) | byteL;
    
    if (rawUm > 0) {
      // μm から cm へ変換して保持
      g_ultrasonicCm = static_cast<uint16_t>(rawUm / 10000);
    }
  }
}