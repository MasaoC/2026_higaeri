#include <esp_now.h>
#include <esp_timer.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <Wire.h>
#include <SoftwareI2C.h>

#include "ModbusConfig.h"
#include "ModbusSlave.h"

constexpr uint8_t kRs485DePin = D10;
constexpr uint8_t kModbusRxPin = D7;  // GPIO20 (UART0 default RX)
constexpr uint8_t kModbusTxPin = D6;  // GPIO21 (UART0 default TX)
constexpr uint8_t kStatusLedPin = D0;
constexpr uint8_t kEncoderPin = D8;
constexpr uint8_t kBatterySensePin = A0;
constexpr uint8_t kAosSdaPin = D2;  // ソフトウェアI2C SDA (AoS)
constexpr uint8_t kAosSclPin = D3;  // ソフトウェアI2C SCL (AoS)
constexpr bool kEnableAosSensor = true;

constexpr uint8_t kAs5600Address = 0x36;
constexpr uint8_t kAs5600RegAngle = 0x0C;

// Airspeed is sampled at 20 Hz and ESP-NOW broadcasts the 0.5 s min/max envelope.
constexpr uint32_t kAirspeedSamplePeriodUs = 50000UL;
constexpr float kAirspeedSamplePeriodSec = 0.050f;
constexpr uint8_t kEspNowBroadcastSamples = 10;
constexpr float kAirspeedPerPps = 1.0f / 1186.6f;  // encoder1
constexpr float kAirspeedOffset = 0.4f;
constexpr uint32_t kDebugPrintIntervalMs = 1000;
static_assert(kAirspeedSamplePeriodUs * kEspNowBroadcastSamples == 500000UL,
              "ESP-NOW broadcast window must stay at 0.5 seconds");

struct EspNowAirDataPacket {
  uint8_t deviceId;  // 0x01 = air_data
  uint8_t reserved;
  uint16_t windSpeed;        // 0.5 s 窓の最小風速 [0.1 m/s]
  uint16_t pulseCountMin;    // 0.5 s 窓の 20 Hz サンプル最小パルス数
  uint16_t pulseCountMax;    // 0.5 s 窓の 20 Hz サンプル最大パルス数
  uint16_t pulseCountTotal;  // 0.5 s 窓の 20 Hz サンプル総パルス数
  uint16_t as5600Primary;
  uint16_t as5600Secondary;
  uint16_t batteryRaw;
  uint32_t sequenceNumber;
};


struct AirDataSnapshot {
  uint16_t windSpeed;
  uint16_t as5600Primary;
  uint16_t as5600Secondary;
  uint16_t batteryRaw;
};

class AirDataSlaveNode : public ModbusSlaveBase {
public:
  explicit AirDataSlaveNode(HardwareSerial& serial)
    : ModbusSlaveBase(serial, kAirDataSlaveId, kRs485DePin, kModbusRxPin, kModbusTxPin) {}

  void setSnapshot(const AirDataSnapshot& snapshot) {
    snapshot_ = snapshot;
  }

  void task() {
    setHoldingValue(kAirWindSpeedRegister, snapshot_.windSpeed);
    setHoldingValue(kAirAs5600PrimaryRegister, snapshot_.as5600Primary);
    setHoldingValue(kAirAs5600SecondaryRegister, snapshot_.as5600Secondary);
    setHoldingValue(kAirBatteryRegister, snapshot_.batteryRaw);
    ModbusSlaveBase::task();
  }

protected:
  void setupRegisters() override {
    addHoldingRegisters(kAirDataReadStart, kAirDataReadCount, 0);
    addHoldingRegisters(kAirDataWriteStart, kAirDataWriteCount, 0);
  }

  void setupCallbacks() override {
    registerReadHandler(kAirDataReadStart, kAirDataReadCount);
    registerWriteHandler(kAirDataWriteStart, kAirDataWriteCount);
  }

  uint16_t onWriteRegister(TRegister* reg, uint16_t newValue) override {
    if (reg->address.address != kAirDataWriteStart) {
      return newValue;
    }

    if (newValue == kAirCommandLedOn) {
      digitalWrite(kStatusLedPin, HIGH);
    } else if (newValue == kAirCommandLedOff) {
      digitalWrite(kStatusLedPin, LOW);
    }

    return kAirCommandNone;
  }

private:
  AirDataSnapshot snapshot_ = { 0, 0, 0, 0 };
};

namespace {
HardwareSerial MySerial0(0);
AirDataSlaveNode g_slave(MySerial0);
SoftwareI2C Wire1;
static portMUX_TYPE g_sampleMux = portMUX_INITIALIZER_UNLOCKED;

struct SampleSlot {
  uint32_t pulseCount;
};

volatile uint32_t g_totalPulseCount = 0;
volatile SampleSlot g_sampleSlot = {};
volatile bool g_sampleReady = false;
AirDataSnapshot g_snapshot = { 0, 0, 0, 0 };
volatile uint16_t g_aoaRaw = 0;
volatile uint16_t g_aosRaw = 0;
volatile bool g_aoaAvailable = false;
volatile bool g_aosAvailable = false;
unsigned long g_lastDebugAt = 0;
uint32_t g_espNowSequence = 0;
uint32_t g_lastPulseCountInSample = 0;
float g_lastPulsesPerSec = 0.0f;
uint32_t g_lastPulseCountMin = 0;
uint32_t g_lastPulseCountMax = 0;
uint32_t g_lastPulseCountTotal = 0;
uint32_t g_windowPulseCountMin = 0;
uint32_t g_windowPulseCountMax = 0;
uint32_t g_windowPulseCountTotal = 0;
uint8_t g_windowSampleCount = 0;
bool g_broadcastWindowReady = false;
const uint8_t kBroadcastAddress[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

static uint32_t g_prevPulseCount = 0;

uint16_t convertPulseCountToWindSpeed(uint32_t pulseCountInWindow, float elapsedSec);

void sendEspNow() {
  if (!g_broadcastWindowReady) {
    return;
  }
  g_broadcastWindowReady = false;

  EspNowAirDataPacket packet;
  packet.deviceId = 0x01;
  packet.reserved = 0;
  packet.windSpeed = convertPulseCountToWindSpeed(g_lastPulseCountMin, kAirspeedSamplePeriodSec);
  packet.pulseCountMin = static_cast<uint16_t>(g_lastPulseCountMin);
  packet.pulseCountMax = static_cast<uint16_t>(g_lastPulseCountMax);
  packet.pulseCountTotal = static_cast<uint16_t>(g_lastPulseCountTotal);
  packet.as5600Primary = g_snapshot.as5600Primary;
  packet.as5600Secondary = g_snapshot.as5600Secondary;
  packet.batteryRaw = g_snapshot.batteryRaw;
  packet.sequenceNumber = ++g_espNowSequence;

  esp_now_send(kBroadcastAddress, reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
}

void IRAM_ATTR handleEncoderPulse() {
  g_totalPulseCount++;
}

void onSampleTimer(void* /*arg*/) {
  const uint32_t totalPulseCount = g_totalPulseCount;

  portENTER_CRITICAL(&g_sampleMux);
  g_sampleSlot.pulseCount = totalPulseCount - g_prevPulseCount;
  g_sampleReady = true;
  portEXIT_CRITICAL(&g_sampleMux);

  g_prevPulseCount = totalPulseCount;
}

uint16_t convertPulseCountToWindSpeed(uint32_t pulseCountInWindow, float elapsedSec) {
  // Replace this linear model once the actual calibration formula is fixed.
  const float pulsesPerSec = static_cast<float>(pulseCountInWindow) / elapsedSec;
  const float windSpeed = (kAirspeedPerPps * pulsesPerSec) + kAirspeedOffset;

  if (windSpeed <= 0.0f) {
    return 0;
  }

  return static_cast<uint16_t>(windSpeed * 10.0f + 0.5f);
}

void updateWindSpeed() {
  if (!g_sampleReady) {
    return;
  }

  SampleSlot sampleSlot = {};
  portENTER_CRITICAL(&g_sampleMux);
  sampleSlot.pulseCount = g_sampleSlot.pulseCount;
  g_sampleReady = false;
  portEXIT_CRITICAL(&g_sampleMux);

  g_lastPulseCountInSample = sampleSlot.pulseCount;
  g_lastPulsesPerSec = static_cast<float>(sampleSlot.pulseCount) / kAirspeedSamplePeriodSec;

  const uint16_t windSpeed = convertPulseCountToWindSpeed(sampleSlot.pulseCount, kAirspeedSamplePeriodSec);
  g_snapshot.windSpeed = windSpeed;

  if (g_windowSampleCount == 0) {
    g_windowPulseCountMin = sampleSlot.pulseCount;
    g_windowPulseCountMax = sampleSlot.pulseCount;
    g_windowPulseCountTotal = sampleSlot.pulseCount;
  } else {
    if (sampleSlot.pulseCount < g_windowPulseCountMin) {
      g_windowPulseCountMin = sampleSlot.pulseCount;
    }
    if (sampleSlot.pulseCount > g_windowPulseCountMax) {
      g_windowPulseCountMax = sampleSlot.pulseCount;
    }
    g_windowPulseCountTotal += sampleSlot.pulseCount;
  }

  g_windowSampleCount++;
  if (g_windowSampleCount >= kEspNowBroadcastSamples) {
    g_lastPulseCountMin = g_windowPulseCountMin;
    g_lastPulseCountMax = g_windowPulseCountMax;
    g_lastPulseCountTotal = g_windowPulseCountTotal;
    g_windowSampleCount = 0;
    g_broadcastWindowReady = true;
  }
}

bool readAS5600(TwoWire& wire, uint16_t& rawAngle) {
  wire.beginTransmission(kAs5600Address);
  wire.write(kAs5600RegAngle);
  if (wire.endTransmission(false) != 0) {
    return false;
  }
  if (wire.requestFrom(kAs5600Address, 2) < 2) {
    return false;
  }
  rawAngle = ((uint16_t)wire.read() << 8) & 0x0F00;
  rawAngle |= (uint16_t)wire.read();
  return true;
}

bool readAS5600Soft(SoftwareI2C& softWire, uint16_t& rawAngle) {
  softWire.beginTransmission(kAs5600Address);
  softWire.write(kAs5600RegAngle);
  softWire.endTransmission();
  if (softWire.requestFrom(kAs5600Address, (uint8_t)2) != 1) {
    return false;
  }
  rawAngle = ((uint16_t)softWire.read() << 8) & 0x0F00;
  rawAngle |= (uint16_t)softWire.read();
  return true;
}

void aoaTask(void* /*arg*/) {
  for (;;) {
    uint16_t rawAngle = 0;
    g_aoaAvailable = readAS5600(Wire, rawAngle);
    if (g_aoaAvailable) {
      g_aoaRaw = rawAngle;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void aosTask(void* /*arg*/) {
  for (;;) {
    uint16_t rawAngle = 0;
    g_aosAvailable = readAS5600Soft(Wire1, rawAngle);
    if (g_aosAvailable) {
      g_aosRaw = rawAngle;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void updateSensors() {
  updateWindSpeed();
  g_snapshot.as5600Primary = g_aoaRaw;
  g_snapshot.as5600Secondary = kEnableAosSensor ? g_aosRaw : 0;
  g_snapshot.batteryRaw = analogRead(kBatterySensePin);
}

void printDebug() {
  if (millis() - g_lastDebugAt < kDebugPrintIntervalMs) {
    return;
  }
  g_lastDebugAt = millis();
  Serial.printf("[air_data] windSpeed=%.1f  txMin=%.1f  pulseMin=%lu  pulseMax=%lu  pulse20Hz=%lu  pulseRate=%.1f  pulseTotal=%lu  battRaw=%u  aoa=%u%s  aos=%u%s\n",
                g_snapshot.windSpeed / 10.0f,
                convertPulseCountToWindSpeed(g_lastPulseCountMin, kAirspeedSamplePeriodSec) / 10.0f,
                static_cast<unsigned long>(g_lastPulseCountMin),
                static_cast<unsigned long>(g_lastPulseCountMax),
                static_cast<unsigned long>(g_lastPulseCountInSample),
                g_lastPulsesPerSec,
                static_cast<unsigned long>(g_totalPulseCount),
                g_snapshot.batteryRaw,
                g_snapshot.as5600Primary,
                g_aoaAvailable ? "" : "?",
                g_snapshot.as5600Secondary,
                kEnableAosSensor && !g_aosAvailable ? "?" : "");
}
}

void setup() {
  Serial.begin(kDebugBaudRate);
  delay(100);

  pinMode(kStatusLedPin, OUTPUT);
  digitalWrite(kStatusLedPin, LOW);
  pinMode(kEncoderPin, INPUT_PULLUP);
  pinMode(kBatterySensePin, INPUT);
  attachInterrupt(digitalPinToInterrupt(kEncoderPin), handleEncoderPulse, FALLING);

  // ハードウェアI2C (AoA: 迎え角)
  Wire.begin();
  Wire.setClock(400000);
  // ソフトウェアI2C (AoS: 横滑り角): D2=SDA, D3=SCL
  if (kEnableAosSensor) {
    Wire1.begin(kAosSdaPin, kAosSclPin);
  }

  xTaskCreate(aoaTask, "aoa", 2048, nullptr, 1, nullptr);
  if (kEnableAosSensor) {
    xTaskCreate(aosTask, "aos", 2048, nullptr, 1, nullptr);
  }

  esp_timer_handle_t sampleTimer;
  const esp_timer_create_args_t timerArgs = {
    .callback = onSampleTimer,
    .arg = nullptr,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "airdata20hz",
    .skip_unhandled_events = false,
  };
  if (esp_timer_create(&timerArgs, &sampleTimer) == ESP_OK) {
    esp_timer_start_periodic(sampleTimer, kAirspeedSamplePeriodUs);
  } else {
    Serial.println("[air_data] sample timer init FAILED");
  }

  g_slave.begin();

  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_21dBm);
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, kBroadcastAddress, 6);
    peer.channel = 0;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
    Serial.println("[air_data] ESP-NOW init OK");
  } else {
    Serial.println("[air_data] ESP-NOW init FAILED");
  }
}

void loop() {
  updateSensors();
  g_slave.setSnapshot(g_snapshot);
  g_slave.task();
  sendEspNow();
  printDebug();
}
