// ESP-NOW 距離確認用 RX
// display_d1 と同一ハードウェア (XIAO ESP32C3 + TFT_eSPI)。
// 受信状況・RSSI・受信レート・シーケンス欠落数を TFT に表示する。

#include <esp_now.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <WiFi.h>

constexpr uint8_t  kSpiSckPin        = 8;
constexpr uint8_t  kSpiMisoPin       = 9;
constexpr uint8_t  kSpiMosiPin       = 10;
constexpr uint16_t kBgColor          = TFT_WHITE;
constexpr uint32_t kRenderIntervalMs  = 200;
constexpr uint32_t kDebugBaudRate    = 115200;
// この時間 (ms) パケットが届かなければ TIMEOUT 表示
constexpr uint32_t kTimeoutMs        = 2000;

struct RangeTestPacket {
  uint8_t  deviceId;
  uint32_t sequenceNumber;
  uint32_t timestampMs;
};

namespace {
TFT_eSPI g_tft;

uint32_t g_lastSeq          = 0;
uint32_t g_totalReceived    = 0;
uint32_t g_totalDropped     = 0;
unsigned long g_lastReceivedAt = 0;
unsigned long g_lastRenderAt   = 0;
bool     g_everReceived     = false;

void renderDisplay() {
  if (millis() - g_lastRenderAt < kRenderIntervalMs) {
    return;
  }
  g_lastRenderAt = millis();

  const bool timedOut = g_everReceived &&
                        (millis() - g_lastReceivedAt) > kTimeoutMs;
  const bool receiving = g_everReceived && !timedOut;

  g_tft.setTextColor(TFT_BLACK, kBgColor);
  g_tft.drawCentreString("ESP-NOW RX TEST", 120, 10, 2);

  // ステータス
  const uint16_t statusColor = receiving ? TFT_GREEN
                             : (timedOut ? TFT_RED : 0xFE60 /* オレンジ */);
  g_tft.setTextColor(statusColor, kBgColor);
  const char* statusStr = !g_everReceived ? "WAITING..."
                        : (timedOut       ? "TIMEOUT"  : "RECEIVING");
  g_tft.drawCentreString(statusStr, 120, 40, 4);
  g_tft.setTextColor(TFT_BLACK, kBgColor);

  // RSSI
  char line[40];
  snprintf(line, sizeof(line), "RSSI  : N/A         ");
  g_tft.drawString(line, 8, 100, 2);

  // 受信数
  snprintf(line, sizeof(line), "RX    : %-8lu",
           static_cast<unsigned long>(g_totalReceived));
  g_tft.drawString(line, 8, 120, 2);

  // 欠落数
  snprintf(line, sizeof(line), "DROP  : %-8lu",
           static_cast<unsigned long>(g_totalDropped));
  g_tft.setTextColor(g_totalDropped > 0 ? TFT_RED : TFT_BLACK, kBgColor);
  g_tft.drawString(line, 8, 140, 2);
  g_tft.setTextColor(TFT_BLACK, kBgColor);

  // 最終シーケンス番号
  snprintf(line, sizeof(line), "SEQ   : %-8lu",
           static_cast<unsigned long>(g_lastSeq));
  g_tft.drawString(line, 8, 160, 2);

  // 最終受信からの経過時間
  const unsigned long elapsedMs = g_everReceived
                                ? (millis() - g_lastReceivedAt) : 0;
  snprintf(line, sizeof(line), "LAST  : %lu ms  ",
           static_cast<unsigned long>(elapsedMs));
  g_tft.drawString(line, 8, 180, 2);
}
}

#include <esp_arduino_version.h>

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void onEspNowReceive(const esp_now_recv_info_t* recvInfo, const uint8_t* data, int len) {
#else
void onEspNowReceive(const uint8_t* macAddr, const uint8_t* data, int len) {
#endif
  if (len != sizeof(RangeTestPacket)) {
    return;
  }
  const RangeTestPacket* packet =
    reinterpret_cast<const RangeTestPacket*>(data);
  if (packet->deviceId != 0xAA) {
    return;
  }

  const uint32_t incoming = packet->sequenceNumber;

  if (g_everReceived && incoming > g_lastSeq + 1) {
    g_totalDropped += incoming - g_lastSeq - 1;
  }

  g_lastSeq        = incoming;
  g_lastReceivedAt = millis();
  g_totalReceived++;
  g_everReceived   = true;

  Serial.printf("[RX] seq=%lu  drop=%lu\n",
    static_cast<unsigned long>(incoming),
    static_cast<unsigned long>(g_totalDropped));
}

void setup() {
  Serial.begin(kDebugBaudRate);
  delay(200);

  SPI.begin(kSpiSckPin, kSpiMisoPin, kSpiMosiPin);
  g_tft.init();
  g_tft.setRotation(0);
  g_tft.fillScreen(kBgColor);

  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.disconnect();

  Serial.print("[RX] MAC address: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    g_tft.setTextColor(TFT_RED, kBgColor);
    g_tft.drawCentreString("ESP-NOW INIT FAIL", 120, 80, 2);
    Serial.println("[RX] ESP-NOW init FAILED");
    while (true) { delay(1000); }
  }

  esp_now_register_recv_cb(onEspNowReceive);
  Serial.println("[RX] ESP-NOW init OK. Waiting...");
}

void loop() {
  renderDisplay();
}
