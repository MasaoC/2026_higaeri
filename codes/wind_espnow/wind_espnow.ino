#include <esp_now.h>
#include <esp_timer.h>
#include <WiFi.h>

constexpr uint8_t kEncoderPin = D8;
constexpr uint8_t kStatusLedPin = D0;
constexpr uint8_t kWindEspNowDeviceId = 0x03;
constexpr uint32_t kSampleWindowUs = 500000UL;
constexpr float kSampleWindowSec = 0.500f;
constexpr float kWindSpeedPerPps = 1.0f / 1237.6f;
constexpr float kWindSpeedOffset = 0.44f;
constexpr uint32_t kDebugPrintIntervalMs = 1000;
constexpr uint32_t kEncoderDebounceUs = 50UL;  // 最小パルス間隔81μs(10m/s時)より小さく設定

struct EspNowAirDataPacket {
  uint8_t  deviceId;
  uint8_t  reserved;
  uint16_t windSpeed;
  uint16_t as5600Primary;
  uint16_t as5600Secondary;
  uint16_t batteryRaw;
  uint32_t sequenceNumber;
};

namespace {
portMUX_TYPE g_sampleMux = portMUX_INITIALIZER_UNLOCKED;

struct SampleSlot {
  uint32_t pulseCount;
  uint32_t timestampUs;
};

volatile uint32_t g_totalPulseCount = 0;
volatile uint32_t g_lastPulseTimeUs = 0;
volatile SampleSlot g_sampleSlot = {};
volatile bool g_sampleReady = false;
uint32_t g_prevPulseCount = 0;
uint32_t g_sequenceNumber = 0;
uint32_t g_lastPulseCount = 0;
float g_lastPulsesPerSec = 0.0f;
uint16_t g_lastWindSpeedDeci = 0;
unsigned long g_lastDebugAt = 0;

const uint8_t kBroadcastAddress[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void IRAM_ATTR handleEncoderPulse() {
  const uint32_t now = static_cast<uint32_t>(esp_timer_get_time());
  if (now - g_lastPulseTimeUs >= kEncoderDebounceUs) {
    g_totalPulseCount++;
    g_lastPulseTimeUs = now;
  }
}

void onSampleTimer(void* /*arg*/) {
  const uint32_t totalPulseCount = g_totalPulseCount;

  portENTER_CRITICAL(&g_sampleMux);
  g_sampleSlot.pulseCount = totalPulseCount - g_prevPulseCount;
  g_sampleSlot.timestampUs = static_cast<uint32_t>(esp_timer_get_time());
  g_sampleReady = true;
  portEXIT_CRITICAL(&g_sampleMux);

  g_prevPulseCount = totalPulseCount;
}

uint16_t windSpeedToDeci(float windSpeedMps) {
  if (windSpeedMps <= 0.0f) {
    return 0;
  }

  return static_cast<uint16_t>(windSpeedMps * 10.0f + 0.5f);
}

bool updateMeasurement(SampleSlot& sampleSlot, float& pulsesPerSec, uint16_t& windSpeedDeci) {
  if (!g_sampleReady) {
    return false;
  }

  portENTER_CRITICAL(&g_sampleMux);
  sampleSlot.pulseCount = g_sampleSlot.pulseCount;
  sampleSlot.timestampUs = g_sampleSlot.timestampUs;
  g_sampleReady = false;
  portEXIT_CRITICAL(&g_sampleMux);

  pulsesPerSec = static_cast<float>(sampleSlot.pulseCount) / kSampleWindowSec;
  const float windSpeedMps = (kWindSpeedPerPps * pulsesPerSec) + kWindSpeedOffset;
  windSpeedDeci = windSpeedToDeci(windSpeedMps);

  g_lastPulseCount = sampleSlot.pulseCount;
  g_lastPulsesPerSec = pulsesPerSec;
  g_lastWindSpeedDeci = windSpeedDeci;
  return true;
}

void sendEspNow(uint16_t windSpeedDeci) {
  EspNowAirDataPacket packet = {};
  packet.deviceId = kWindEspNowDeviceId;
  packet.windSpeed = windSpeedDeci;
  packet.sequenceNumber = ++g_sequenceNumber;
  esp_now_send(kBroadcastAddress, reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
}

void printDebug() {
  if (millis() - g_lastDebugAt < kDebugPrintIntervalMs) {
    return;
  }

  g_lastDebugAt = millis();
  Serial.printf("[wind_espnow] pulse=%lu  pps=%.1f  wind=%.1f m/s  total=%lu\n",
    static_cast<unsigned long>(g_lastPulseCount),
    g_lastPulsesPerSec,
    g_lastWindSpeedDeci / 10.0f,
    static_cast<unsigned long>(g_totalPulseCount));
}
}

void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(kStatusLedPin, OUTPUT);
  digitalWrite(kStatusLedPin, LOW);
  pinMode(kEncoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(kEncoderPin), handleEncoderPulse, FALLING);

  esp_timer_handle_t sampleTimer;
  const esp_timer_create_args_t timerArgs = {
    .callback = onSampleTimer,
    .arg = nullptr,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "wind500ms",
    .skip_unhandled_events = false,
  };
  ESP_ERROR_CHECK(esp_timer_create(&timerArgs, &sampleTimer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(sampleTimer, kSampleWindowUs));

  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, kBroadcastAddress, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
    digitalWrite(kStatusLedPin, HIGH);
    Serial.println("[wind_espnow] ESP-NOW init OK");
  } else {
    Serial.println("[wind_espnow] ESP-NOW init FAILED");
  }
}

void loop() {
  SampleSlot sampleSlot = {};
  float pulsesPerSec = 0.0f;
  uint16_t windSpeedDeci = 0;

  if (updateMeasurement(sampleSlot, pulsesPerSec, windSpeedDeci)) {
    sendEspNow(windSpeedDeci);
  }

  printDebug();
  delay(1);
}