#ifndef MODBUS_CONFIG_H
#define MODBUS_CONFIG_H

#include <Arduino.h>

constexpr uint32_t kDebugBaudRate = 115200;
constexpr uint32_t kModbusBaudRate = 9600;
constexpr uint32_t kLoggerPollIntervalMs = 1000;
constexpr uint32_t kRollAlarmWriteIntervalMs = 200;
constexpr uint32_t kModbusTimeoutMs = 300;
constexpr uint8_t kAirDataSlaveId = 1;
constexpr uint8_t kDisplaySlaveId = 2;

constexpr uint32_t kAirReadIntervalMs = 500;
constexpr uint32_t kAirWriteIntervalMs = 500;
constexpr uint32_t kDisplayReadIntervalMs = 1000;
constexpr uint32_t kLogIntervalMs = 500;

constexpr uint16_t kAirDataReadStart = 0;
constexpr uint16_t kAirDataReadCount = 4;
constexpr uint16_t kAirDataWriteStart = 10;
constexpr uint16_t kAirDataWriteCount = 1;

constexpr uint16_t kDisplayReadStart = 0;
constexpr uint16_t kDisplayReadCount = 5;
constexpr uint16_t kDisplayWriteStart = 10;
constexpr uint16_t kDisplayWriteCount = 2;

enum DisplayWriteRegister : uint16_t {
  kDisplayAirspeedWriteRegister = kDisplayWriteStart,
  kDisplayRollAlarmWriteRegister
};

#endif
