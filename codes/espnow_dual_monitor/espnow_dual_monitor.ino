#include <esp_now.h>
#include <WiFi.h>

constexpr uint8_t kStatusLedPin = D0;
constexpr uint8_t kAirDataEspNowDeviceId = 0x01;
constexpr uint8_t kWind1EspNowDeviceId = 0x03;   // 主翼側風速計1 
constexpr uint8_t kWind2EspNowDeviceId = 0x04;   // 主翼側風速計2
constexpr uint32_t kDebugPrintIntervalMs = 500;
constexpr uint32_t kPacketStaleMs = 2000;

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
  uint16_t as5600Primary;
  uint16_t as5600Secondary;
  uint16_t batteryRaw;
  uint32_t sequenceNumber;
};

namespace {
EspNowAirDataPacket g_airDataPacket = {};
EspNowLegacyPacket g_windPacket1 = {};
EspNowLegacyPacket g_windPacket2 = {};
unsigned long g_airDataLastReceivedAt = 0;
unsigned long g_wind1LastReceivedAt = 0;
unsigned long g_wind2LastReceivedAt = 0;
unsigned long g_lastDebugAt = 0;

bool isFresh(unsigned long lastReceivedAt) {
  return (millis() - lastReceivedAt) < kPacketStaleMs;
}
}

#include <esp_arduino_version.h>

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void onEspNowReceive(const esp_now_recv_info_t* recvInfo, const uint8_t* data, int len) {
#else
void onEspNowReceive(const uint8_t* macAddr, const uint8_t* data, int len) {
#endif
  if (len < 1) {
    return;
  }

  const uint8_t deviceId = data[0];
  if (deviceId == kAirDataEspNowDeviceId) {
    if (len == static_cast<int>(sizeof(EspNowAirDataPacket))) {
      g_airDataPacket = *reinterpret_cast<const EspNowAirDataPacket*>(data);
    } else if (len == static_cast<int>(sizeof(EspNowLegacyPacket))) {
      const EspNowLegacyPacket* legacyPacket = reinterpret_cast<const EspNowLegacyPacket*>(data);
      g_airDataPacket.deviceId = legacyPacket->deviceId;
      g_airDataPacket.reserved = legacyPacket->reserved;
      g_airDataPacket.windSpeed = legacyPacket->windSpeed;
      g_airDataPacket.pulseCountMin = 0;
      g_airDataPacket.pulseCountMax = 0;
      g_airDataPacket.as5600Primary = legacyPacket->as5600Primary;
      g_airDataPacket.as5600Secondary = legacyPacket->as5600Secondary;
      g_airDataPacket.batteryRaw = legacyPacket->batteryRaw;
      g_airDataPacket.sequenceNumber = legacyPacket->sequenceNumber;
    } else {
      return;
    }
    g_airDataLastReceivedAt = millis();
    return;
  }

  if (deviceId == kWind1EspNowDeviceId) {
    if (len != static_cast<int>(sizeof(EspNowLegacyPacket))) {
      return;
    }
    g_windPacket1 = *reinterpret_cast<const EspNowLegacyPacket*>(data);
    g_wind1LastReceivedAt = millis();
    return;
  }

  if (deviceId == kWind2EspNowDeviceId) {
    if (len != static_cast<int>(sizeof(EspNowLegacyPacket))) {
      return;
    }
    g_windPacket2 = *reinterpret_cast<const EspNowLegacyPacket*>(data);
    g_wind2LastReceivedAt = millis();
    return;
  }
}

void printStatus() {
  if (millis() - g_lastDebugAt < kDebugPrintIntervalMs) {
    return;
  }

  g_lastDebugAt = millis();
  const bool airFresh = isFresh(g_airDataLastReceivedAt);
  const bool wind1Fresh = isFresh(g_wind1LastReceivedAt);
  const bool wind2Fresh = isFresh(g_wind2LastReceivedAt);

  // 左右両方生きている場合は平均値、片方だけの場合はその片方の値
  float combinedWindSpeed = 0.0f;
  bool windFreshCombined = false;
  unsigned long maxSeq = 0;
  unsigned long minAge = 0;

  if (wind1Fresh && wind2Fresh) {
    combinedWindSpeed = (g_windPacket1.windSpeed + g_windPacket2.windSpeed) / 20.0f;
    windFreshCombined = true;
    maxSeq = (g_windPacket1.sequenceNumber > g_windPacket2.sequenceNumber) ? g_windPacket1.sequenceNumber : g_windPacket2.sequenceNumber;
    minAge = (millis() - g_wind1LastReceivedAt < millis() - g_wind2LastReceivedAt) ? (millis() - g_wind1LastReceivedAt) : (millis() - g_wind2LastReceivedAt);
  } else if (wind1Fresh) {
    combinedWindSpeed = g_windPacket1.windSpeed / 10.0f;
    windFreshCombined = true;
    maxSeq = g_windPacket1.sequenceNumber;
    minAge = millis() - g_wind1LastReceivedAt;
  } else if (wind2Fresh) {
    combinedWindSpeed = g_windPacket2.windSpeed / 10.0f;
    windFreshCombined = true;
    maxSeq = g_windPacket2.sequenceNumber;
    minAge = millis() - g_wind2LastReceivedAt;
  }

  // タイムアウト時は数値を流さず "STALE" という文字列を出力することで、
  // Python 側の正規表現が数値としてパースしないようにし、Python 側で正しく STALE 判定されるようにする
  char w1Str[16];
  if (wind1Fresh) {
    snprintf(w1Str, sizeof(w1Str), "%.1f", g_windPacket1.windSpeed / 10.0f);
  } else {
    strcpy(w1Str, "STALE");
  }

  char w2Str[16];
  if (wind2Fresh) {
    snprintf(w2Str, sizeof(w2Str), "%.1f", g_windPacket2.windSpeed / 10.0f);
  } else {
    strcpy(w2Str, "STALE");
  }

  char airStr[16];
  if (airFresh) {
    snprintf(airStr, sizeof(airStr), "%.1f", g_airDataPacket.windSpeed / 10.0f);
  } else {
    strcpy(airStr, "STALE");
  }

  char combinedWindStr[16];
  if (windFreshCombined) {
    snprintf(combinedWindStr, sizeof(combinedWindStr), "%.1f", combinedWindSpeed);
  } else {
    strcpy(combinedWindStr, "STALE");
  }

  Serial.printf(
    "[dual_monitor] air=%s spd=%s pmin=%u pmax=%u as1=%u as2=%u batt=%u seq=%lu age=%lums | wind=%s spd=%s seq=%lu age=%lums | disp=[pot1=%u pot2=%u] | w1=%s(%s) w2=%s(%s)\n",
    airFresh ? "OK" : "STALE",
    airStr,
    g_airDataPacket.pulseCountMin,
    g_airDataPacket.pulseCountMax,
    g_airDataPacket.as5600Primary,
    g_airDataPacket.as5600Secondary,
    g_airDataPacket.batteryRaw,
    static_cast<unsigned long>(g_airDataPacket.sequenceNumber),
    static_cast<unsigned long>(millis() - g_airDataLastReceivedAt),
    windFreshCombined ? "OK" : "STALE",
    combinedWindStr,
    maxSeq,
    minAge,
    airFresh ? g_airDataPacket.as5600Primary : 0,
    airFresh ? g_airDataPacket.as5600Secondary : 0,
    w1Str,
    wind1Fresh ? "OK" : "STALE",
    w2Str,
    wind2Fresh ? "OK" : "STALE");
}

void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(kStatusLedPin, OUTPUT);
  digitalWrite(kStatusLedPin, LOW);

  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.disconnect();

  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(onEspNowReceive);
    digitalWrite(kStatusLedPin, HIGH);
    Serial.println("[dual_monitor] ESP-NOW init OK");
  } else {
    Serial.println("[dual_monitor] ESP-NOW init FAILED");
  }
}

void loop() {
  printStatus();
  delay(1);
}