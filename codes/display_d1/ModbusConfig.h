#ifndef MODBUS_CONFIG_H
#define MODBUS_CONFIG_H

#include <Arduino.h>

constexpr uint32_t kDebugBaudRate = 115200;
constexpr uint32_t kModbusBaudRate = 9600;
constexpr uint32_t kModbusTimeoutMs = 100;  // 統一: air_data, logger と同じ値
constexpr uint32_t kDisplayRefreshIntervalMs = 100;
constexpr uint32_t kDisplayI2CPollIntervalMs = 100;
constexpr uint8_t kDisplaySlaveId = 2;
constexpr uint8_t kDisplayD2I2CAddress = 0x30;
constexpr uint8_t kDisplayD2PayloadSize = 12;

constexpr uint16_t kDisplayReadStart = 0;
constexpr uint16_t kDisplayReadCount = 5;
constexpr uint16_t kDisplayWriteStart = 10;
constexpr uint16_t kDisplayWriteCount = 2;

enum DisplayWriteRegister : uint16_t {
  kDisplayAirspeedWriteRegister = kDisplayWriteStart,
  kDisplayRollAlarmWriteRegister
};

enum DisplayRegister : uint16_t {
  kDisplayBaroRegister = kDisplayReadStart,
  kDisplayPot1Register,
  kDisplayPot2Register,
  kDisplayBatteryRegister,
  kDisplayUltrasonicRegister
};

#endif
