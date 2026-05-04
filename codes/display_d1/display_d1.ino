#include <Adafruit_DPS310.h>
#include <esp_now.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <WiFi.h>
#include <Wire.h>

#include "ModbusConfig.h"
#include "ModbusSlave.h"
#include "font_data.h"

constexpr uint8_t kRs485DePin = D3;
constexpr uint8_t kModbusRxPin = D7;  // GPIO20 (UART0 default RX)
constexpr uint8_t kModbusTxPin = D6;  // GPIO21 (UART0 default TX)
constexpr uint8_t kSpiSckPin = 8;
constexpr uint8_t kSpiMisoPin = 9;
constexpr uint8_t kSpiMosiPin = 10;
constexpr uint16_t kDisplayBackground = TFT_WHITE;
constexpr uint16_t kGaugeLineColor = TFT_BLACK;
constexpr uint16_t kGaugeSafeColor = TFT_GREEN;
constexpr uint16_t kGaugeCautionColor = TFT_RED;
constexpr uint32_t kDebugPrintIntervalMs = 1000;
constexpr uint8_t kDps310I2CAddress = 0x76;
constexpr float kSeaLevelPressureHpa = 1013.25f;

// ── ESP-NOW フォールバック（air_data と同一の構造体）──
struct EspNowAirDataPacket {
  uint8_t  deviceId;
  uint8_t  reserved;
  uint16_t windSpeed;
  uint16_t as5600Primary;
  uint16_t as5600Secondary;
  uint16_t batteryRaw;
  uint32_t sequenceNumber;
};

// Modbus で対気速度が更新されなければ ESP-NOW に切替えるタイムアウト
constexpr uint32_t kModbusAirspeedTimeoutMs = 2000;
// ESP-NOW パケットがこの時間 (ms) 以内に届いていれば有効とみなす
constexpr uint32_t kEspNowStaleMs = 2000;
constexpr uint8_t kAirDataEspNowDeviceId = 0x01;
constexpr uint8_t kWindEspNowDeviceId = 0x03;

// ── Gauge Layout ──
constexpr int16_t kGaugeCx = 120;
constexpr int16_t kGaugeCy = 125;
constexpr int16_t kGaugeOuterR = 105;
constexpr int16_t kArcOuterR = 103;
constexpr int16_t kArcInnerR = 88;
constexpr int16_t kTickOuterR = 105;
constexpr int16_t kTickMajorInnerR = 85;
constexpr int16_t kTickMinorInnerR = 95;
constexpr int16_t kNeedleLen = 75;
constexpr int16_t kNumberR = 72;
constexpr int16_t kNeedleSpriteWidth = 15;
constexpr int16_t kNeedleSpriteHeight = 88;
constexpr int16_t kNeedlePivotX = kNeedleSpriteWidth / 2;
constexpr int16_t kNeedlePivotY = kNeedleSpriteHeight - 8;
constexpr int16_t kNeedleBossRadius = 6;
constexpr float kGaugeStartDeg = 30.0f;
constexpr float kGaugeSweepDeg = 300.0f;
constexpr float kGaugeMaxSpeed = 10.0f;
constexpr float kGaugeSafeMinSpeed = 5.5f;
constexpr float kGaugeSafeMaxSpeed = 8.0f;

struct DisplayD2Snapshot {
  uint16_t potentiometer1;
  uint16_t potentiometer2;
  uint16_t batteryVoltage;
  uint16_t ultrasonicAlt;
  uint16_t baroAlt;
  bool connected;
  uint32_t successCount;
  uint32_t failureCount;
};

class DisplayD1SlaveNode : public ModbusSlaveBase {
public:
  explicit DisplayD1SlaveNode(HardwareSerial& serial)
    : ModbusSlaveBase(serial, kDisplaySlaveId, kRs485DePin, kModbusRxPin, kModbusTxPin) {}

  void setSensors(const DisplayD2Snapshot& snapshot) {
    sensors_ = snapshot;
  }

  uint16_t airspeed() const {
    return airspeed_;
  }

  uint16_t rollAlarm() const {
    return rollAlarm_;
  }

  unsigned long modbusAirspeedLastAt() const {
    return modbusAirspeedLastAt_;
  }

  void task() {
    setHoldingValue(kDisplayBaroRegister, sensors_.baroAlt);
    setHoldingValue(kDisplayPot1Register, sensors_.potentiometer1);
    setHoldingValue(kDisplayPot2Register, sensors_.potentiometer2);
    setHoldingValue(kDisplayBatteryRegister, sensors_.batteryVoltage);
    setHoldingValue(kDisplayUltrasonicRegister, sensors_.ultrasonicAlt);
    ModbusSlaveBase::task();
  }

protected:
  void setupRegisters() override {
    addHoldingRegisters(kDisplayReadStart, kDisplayReadCount, 0);
    addHoldingRegisters(kDisplayWriteStart, kDisplayWriteCount, 0);
  }

  void setupCallbacks() override {
    registerReadHandler(kDisplayReadStart, kDisplayReadCount);
    registerWriteHandler(kDisplayWriteStart, kDisplayWriteCount);
  }

  uint16_t onWriteRegister(TRegister* reg, uint16_t newValue) override {
    if (reg->address.address == kDisplayAirspeedWriteRegister) {
      airspeed_ = newValue;
      modbusAirspeedLastAt_ = millis();
    } else if (reg->address.address == kDisplayRollAlarmWriteRegister) {
      rollAlarm_ = newValue;
    }
    return newValue;
  }

private:
  DisplayD2Snapshot sensors_ = {0, 0, 0, 0, 0, false, 0, 0};
  uint16_t airspeed_ = 0;
  uint16_t rollAlarm_ = 0;
  unsigned long modbusAirspeedLastAt_ = 0;
};

namespace {
HardwareSerial MySerial0(0);
DisplayD1SlaveNode g_slave(MySerial0);
TFT_eSPI g_tft;
TFT_eSprite g_needleSprite = TFT_eSprite(&g_tft);
Adafruit_DPS310 g_dps;
bool g_dpsReady = false;
DisplayD2Snapshot g_snapshot = {0, 0, 0, 0, 0, false, 0, 0};
unsigned long g_lastPollAt = 0;
unsigned long g_lastRenderAt = 0;
unsigned long g_lastDebugAt = 0;
int16_t g_prevNeedleAngle = -1000;
EspNowAirDataPacket g_airEspNowLatest = {};
EspNowAirDataPacket g_windEspNowLatest = {};
unsigned long g_airEspNowLastReceivedAt = 0;
unsigned long g_windEspNowLastReceivedAt = 0;
bool g_useEspNow = false;
bool g_needleBufferLoaded = false;
bool g_needleSpriteReady = false;
uint16_t* g_needleBackground = nullptr;
int16_t g_needleMinX = 0;
int16_t g_needleMinY = 0;
int16_t g_needleMaxX = 0;
int16_t g_needleMaxY = 0;

uint16_t getAirspeed() {
  const uint16_t modbusAirspeed = g_slave.airspeed();
  const bool modbusFresh =
    (millis() - g_slave.modbusAirspeedLastAt()) <= kModbusAirspeedTimeoutMs;
  const bool windEspNowFresh =
    (millis() - g_windEspNowLastReceivedAt) < kEspNowStaleMs;

  // 新基板の風速をTFT表示に使う。Modbus が古い、または 0 のままなら ESPNOW を優先する。
  g_useEspNow = windEspNowFresh && (!modbusFresh || modbusAirspeed == 0);
  return g_useEspNow ? g_windEspNowLatest.windSpeed : modbusAirspeed;
}

inline float speedToAngleDeg(float speed) {
  return kGaugeStartDeg + (speed / kGaugeMaxSpeed) * kGaugeSweepDeg;
}

inline uint16_t gaugeAngleToArcAngle(float gaugeAngleDeg) {
  int angle = static_cast<int>(lroundf(gaugeAngleDeg)) + 180;
  angle %= 360;
  if (angle < 0) {
    angle += 360;
  }
  return static_cast<uint16_t>(angle);
}

inline int16_t gx(float angleDeg, float r) {
  return kGaugeCx + static_cast<int16_t>(r * sinf(angleDeg * PI / 180.0f));
}

inline int16_t gy(float angleDeg, float r) {
  return kGaugeCy - static_cast<int16_t>(r * cosf(angleDeg * PI / 180.0f));
}

void drawSmoothGaugeBand(float startSpeed, float endSpeed, uint16_t color, bool roundEnds = false) {
  const uint16_t startAngle = gaugeAngleToArcAngle(speedToAngleDeg(startSpeed));
  const uint16_t endAngle = gaugeAngleToArcAngle(speedToAngleDeg(endSpeed));
  g_tft.drawSmoothArc(
    kGaugeCx,
    kGaugeCy,
    kArcOuterR,
    kArcInnerR,
    startAngle,
    endAngle,
    color,
    kDisplayBackground,
    roundEnds);
}

void createNeedleSprite() {
  g_tft.setPivot(kGaugeCx, kGaugeCy);

  g_needleSprite.setColorDepth(16);
  g_needleSprite.createSprite(kNeedleSpriteWidth, kNeedleSpriteHeight);
  g_needleSprite.fillSprite(kDisplayBackground);
  g_needleSprite.setPivot(kNeedlePivotX, kNeedlePivotY);

  const int16_t shaftLeft = kNeedlePivotX - 1;
  const int16_t shaftTop = kNeedlePivotY - kNeedleLen;
  const int16_t tipY = shaftTop + 4;

  g_needleSprite.fillRect(shaftLeft, tipY, 3, kNeedleLen - 4, kGaugeLineColor);
  g_needleSprite.fillTriangle(
    kNeedlePivotX,
    2,
    shaftLeft,
    tipY,
    shaftLeft + 2,
    tipY,
    kGaugeLineColor);
  g_needleSprite.fillCircle(kNeedlePivotX, kNeedlePivotY, kNeedleBossRadius, kGaugeLineColor);

  int16_t minX = 0;
  int16_t minY = 0;
  int16_t maxX = 0;
  int16_t maxY = 0;
  g_needleSprite.getRotatedBounds(45, &minX, &minY, &maxX, &maxY);
  const int32_t bufferWidth = (maxX - minX) + 1;
  const int32_t bufferHeight = (maxY - minY) + 1;
  g_needleBackground = static_cast<uint16_t*>(malloc(bufferWidth * bufferHeight * sizeof(uint16_t)));
  g_needleSpriteReady = (g_needleBackground != nullptr);
  if (!g_needleSpriteReady) {
    Serial.println("[display_d1] needle buffer alloc FAILED");
  }
}

void restoreNeedleBackground() {
  if (!g_needleBufferLoaded || !g_needleSpriteReady) {
    return;
  }

  g_tft.pushRect(
    g_needleMinX,
    g_needleMinY,
    1 + g_needleMaxX - g_needleMinX,
    1 + g_needleMaxY - g_needleMinY,
    g_needleBackground);
  g_needleBufferLoaded = false;
}

void drawNeedleSprite(int16_t angleDeg) {
  if (!g_needleSpriteReady) {
    return;
  }

  if (g_needleSprite.getRotatedBounds(angleDeg, &g_needleMinX, &g_needleMinY, &g_needleMaxX, &g_needleMaxY)) {
    g_tft.readRect(
      g_needleMinX,
      g_needleMinY,
      1 + g_needleMaxX - g_needleMinX,
      1 + g_needleMaxY - g_needleMinY,
      g_needleBackground);
    g_needleBufferLoaded = true;
  }

  g_needleSprite.pushRotated(angleDeg, kDisplayBackground);
}

void drawGaugeFrame() {
  g_tft.drawSmoothArc(kGaugeCx, kGaugeCy, kGaugeOuterR, kGaugeOuterR - 2,
                      0, 360, kGaugeLineColor, kDisplayBackground, false);

  drawSmoothGaugeBand(0.0f, kGaugeSafeMinSpeed, kGaugeCautionColor, true);
  drawSmoothGaugeBand(kGaugeSafeMinSpeed, kGaugeSafeMaxSpeed, kGaugeSafeColor, false);
  drawSmoothGaugeBand(kGaugeSafeMaxSpeed, kGaugeMaxSpeed, kGaugeCautionColor, true);

  g_tft.loadFont(Arial_Black22);
  g_tft.setTextColor(kGaugeLineColor, kDisplayBackground);
  for (int i = 0; i <= 10; i++) {
    float angle = speedToAngleDeg(static_cast<float>(i));
    g_tft.drawLine(gx(angle, kTickOuterR), gy(angle, kTickOuterR),
                   gx(angle, kTickMajorInnerR), gy(angle, kTickMajorInnerR), kGaugeLineColor);
    char num[4];
    snprintf(num, sizeof(num), "%d", i);
    g_tft.drawCentreString(num, gx(angle, kNumberR), gy(angle, kNumberR) - 11, 1);
  }
  g_tft.unloadFont();

  for (int i = 1; i < 20; i += 2) {
    float angle = speedToAngleDeg(i * 0.5f);
    g_tft.drawLine(gx(angle, kTickOuterR), gy(angle, kTickOuterR),
                   gx(angle, kTickMinorInnerR), gy(angle, kTickMinorInnerR), kGaugeLineColor);
  }

  g_tft.fillCircle(kGaugeCx, kGaugeCy, 5, kGaugeLineColor);
}

void pollDisplayD2() {
  if (millis() - g_lastPollAt < kDisplayI2CPollIntervalMs) {
    return;
  }
  g_lastPollAt = millis();

  Wire.beginTransmission(kDisplayD2I2CAddress);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) {
    g_snapshot = {0, 0, 0, 0, 0, false, g_snapshot.successCount, g_snapshot.failureCount + 1};
    return;
  }

  const int receivedBytes = Wire.requestFrom(static_cast<int>(kDisplayD2I2CAddress), static_cast<int>(kDisplayD2PayloadSize));
  if (receivedBytes != kDisplayD2PayloadSize) {
    g_snapshot = {0, 0, 0, 0, 0, false, g_snapshot.successCount, g_snapshot.failureCount + 1};
    return;
  }

  uint8_t buffer[kDisplayD2PayloadSize];
  for (uint8_t index = 0; index < kDisplayD2PayloadSize; ++index) {
    buffer[index] = Wire.read();
  }

  g_snapshot.potentiometer1 = static_cast<uint16_t>((buffer[1] << 8) | buffer[0]);
  g_snapshot.potentiometer2 = static_cast<uint16_t>((buffer[3] << 8) | buffer[2]);
  g_snapshot.batteryVoltage = static_cast<uint16_t>((buffer[5] << 8) | buffer[4]);
  g_snapshot.ultrasonicAlt = static_cast<uint16_t>((buffer[7] << 8) | buffer[6]);
  g_snapshot.connected = true;
  g_snapshot.successCount++;

  // roll alarm をdisplay_d2 に送信（断線時は無視）
  Wire.beginTransmission(kDisplayD2I2CAddress);
  Wire.write(static_cast<uint8_t>(g_slave.rollAlarm()));
  Wire.endTransmission();
}

void updateBarometricAltitude() {
  if (!g_dpsReady) {
    g_snapshot.baroAlt = 0;
    return;
  }

  sensors_event_t temperatureEvent;
  sensors_event_t pressureEvent;
  if (!g_dps.getEvents(&temperatureEvent, &pressureEvent)) {
    g_snapshot.baroAlt = 0;
    return;
  }

  const float altitudeMeters = g_dps.readAltitude(kSeaLevelPressureHpa);
  g_snapshot.baroAlt = static_cast<uint16_t>(altitudeMeters * 10.0f);
}

void renderDisplay() {
  if (millis() - g_lastRenderAt < kDisplayRefreshIntervalMs) return;
  g_lastRenderAt = millis();

  const float currentSpeed = getAirspeed() / 10.0f;
  const float clampedSpeed = currentSpeed < 0.0f ? 0.0f : (currentSpeed > kGaugeMaxSpeed ? kGaugeMaxSpeed : currentSpeed);
  const int16_t currentNeedleAngle = static_cast<int16_t>(lroundf(speedToAngleDeg(clampedSpeed)));

  const bool needleChanged = (g_prevNeedleAngle != currentNeedleAngle);
  if (needleChanged) {
    restoreNeedleBackground();
  }

  g_slave.task();

  // ── Center digital speed (Arial_Black56) ──
  g_tft.loadFont(Arial_Black56);
  g_tft.setTextColor(kGaugeLineColor, kDisplayBackground);
  g_tft.setTextPadding(g_tft.textWidth("00.0"));
  char speedStr[8];
  int whole = static_cast<int>(currentSpeed);
  int frac = static_cast<int>(currentSpeed * 10.0f) % 10;
  snprintf(speedStr, sizeof(speedStr), "%d.%d", whole, frac);
  g_tft.drawCentreString(speedStr, kGaugeCx, kGaugeCy - 28, 1);
  g_tft.setTextPadding(0);
  g_tft.unloadFont();
  g_tft.fillCircle(kGaugeCx, kGaugeCy, 5, kGaugeLineColor);

  g_slave.task();

  // ── Bottom info panel (built-in font 2) ──
  g_tft.setTextColor(kGaugeLineColor, kDisplayBackground);
  char line[42];
  int y = 250;

  snprintf(line, sizeof(line), "B:%-5u Bt:%-5u",
           g_snapshot.baroAlt, g_snapshot.batteryVoltage);
  g_tft.drawString(line, 4, y, 2);
  y += 18;

  snprintf(line, sizeof(line), "P1:%-4u P2:%-4u U:%-4u",
           g_snapshot.potentiometer1, g_snapshot.potentiometer2, g_snapshot.ultrasonicAlt);
  g_tft.drawString(line, 4, y, 2);
  y += 18;

  g_slave.task();

  snprintf(line, sizeof(line), "I2C:%-3s OK:%-5lu E:%-5lu",
           g_snapshot.connected ? "OK" : "ERR",
           static_cast<unsigned long>(g_snapshot.successCount),
           static_cast<unsigned long>(g_snapshot.failureCount));
  g_tft.setTextColor(g_snapshot.connected ? TFT_GREEN : TFT_RED, kDisplayBackground);
  g_tft.drawString(line, 4, y, 2);
  y += 18;

  snprintf(line, sizeof(line), "src:%-6s", g_useEspNow ? "ESPNOW" : "RS485");
  g_tft.setTextColor(g_useEspNow ? TFT_GREEN : kGaugeLineColor, kDisplayBackground);
  g_tft.drawString(line, 4, y, 2);

  if (needleChanged) {
    drawNeedleSprite(currentNeedleAngle);
    g_prevNeedleAngle = currentNeedleAngle;
  }
}

void printDebug() {
  if (millis() - g_lastDebugAt < kDebugPrintIntervalMs) {
    return;
  }
  g_lastDebugAt = millis();
  const unsigned long windEspNowAgeMs = millis() - g_windEspNowLastReceivedAt;
  const unsigned long airEspNowAgeMs = millis() - g_airEspNowLastReceivedAt;
  Serial.printf("[display_d1] airspeed=%u  src=%s  wind_spd=%u  wind_age=%lums  air_age=%lums  baro=%u  pot1=%u  pot2=%u  batt=%u  ultra=%u  i2c=%s  ok=%lu  err=%lu\n",
    getAirspeed(),
    g_useEspNow ? "ESPNOW" : "RS485",
    g_windEspNowLatest.windSpeed,
    windEspNowAgeMs,
    airEspNowAgeMs,
    g_snapshot.baroAlt,
    g_snapshot.potentiometer1,
    g_snapshot.potentiometer2,
    g_snapshot.batteryVoltage,
    g_snapshot.ultrasonicAlt,
    g_snapshot.connected ? "OK" : "ERR",
    static_cast<unsigned long>(g_snapshot.successCount),
    static_cast<unsigned long>(g_snapshot.failureCount));
}
}

void onEspNowReceive(const esp_now_recv_info_t* /*recvInfo*/, const uint8_t* data, int len) {
  if (len != static_cast<int>(sizeof(EspNowAirDataPacket))) return;
  const EspNowAirDataPacket* pkt = reinterpret_cast<const EspNowAirDataPacket*>(data);
  if (pkt->deviceId == kAirDataEspNowDeviceId) {
    g_airEspNowLatest = *pkt;
    g_airEspNowLastReceivedAt = millis();
    return;
  }
  if (pkt->deviceId == kWindEspNowDeviceId) {
    g_windEspNowLatest = *pkt;
    g_windEspNowLastReceivedAt = millis();
  }
}

void setup() {
  Serial.begin(kDebugBaudRate);
  delay(100);

  Wire.begin(6, 7);  // SDA=GPIO6, SCL=GPIO7 (XIAO ESP32C3)
  Wire.setClock(400000);
  Serial.println("[display_d1] Wire begin OK");

  g_dpsReady = g_dps.begin_I2C(kDps310I2CAddress, &Wire);
  Serial.printf("[display_d1] DPS310 %s\n", g_dpsReady ? "OK" : "FAILED");
  if (g_dpsReady) {
    g_dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
    g_dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
  }

  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_21dBm);
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(onEspNowReceive);
    Serial.println("[display_d1] ESP-NOW init OK");
  } else {
    Serial.println("[display_d1] ESP-NOW init FAILED");
  }

  g_tft.init();
  Serial.println("[display_d1] TFT init OK");
  g_tft.setRotation(0);
  g_tft.fillScreen(kDisplayBackground);
  Serial.println("[display_d1] TFT fill OK");
  drawGaugeFrame();
  Serial.println("[display_d1] gauge frame OK");
  createNeedleSprite();
  Serial.println("[display_d1] needle sprite OK");

  g_slave.begin();
  Serial.println("[display_d1] slave begin OK");
}

void loop() {
  pollDisplayD2();
  updateBarometricAltitude();
  g_slave.setSensors(g_snapshot);
  g_slave.task();
  renderDisplay();
  printDebug();
}
