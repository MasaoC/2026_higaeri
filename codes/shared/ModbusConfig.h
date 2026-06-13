#ifndef MODBUS_CONFIG_H
#define MODBUS_CONFIG_H

#include <Arduino.h>

constexpr uint32_t kDebugBaudRate = 115200;
constexpr uint32_t kModbusBaudRate = 9600;
constexpr uint32_t kDisplayRefreshIntervalMs = 100;
constexpr uint32_t kDisplayI2CPollIntervalMs = 200;
constexpr uint32_t kDisplayD2SensorIntervalMs = 100;
constexpr uint32_t kLoggerPollIntervalMs = 500;
constexpr uint32_t kModbusTimeoutMs = 100;

constexpr uint8_t kModbusRxPin = 5;
constexpr uint8_t kModbusTxPin = 4;
constexpr uint8_t kAirDataSlaveId = 1;
constexpr uint8_t kDisplaySlaveId = 2;
constexpr uint8_t kDisplayD2I2CAddress = 0x30;
constexpr uint8_t kDisplayD2PayloadSize = 12;
constexpr uint8_t kEncoderPulsesPerRevolution = 100;

constexpr uint16_t kAirDataReadStart = 0;
constexpr uint16_t kAirDataReadCount = 4;
constexpr uint16_t kAirDataWriteStart = 10;
constexpr uint16_t kAirDataWriteCount = 1;

constexpr uint16_t kDisplayReadStart = 0;
constexpr uint16_t kDisplayReadCount = 5;
constexpr uint16_t kDisplayWriteStart = 10;
constexpr uint16_t kDisplayWriteCount = 1;

enum AirDataRegister : uint16_t {
  kAirRotationRegister = kAirDataReadStart,
  kAirAs5600PrimaryRegister,
  kAirAs5600SecondaryRegister,
  kAirBatteryRegister
};

enum DisplayRegister : uint16_t {
  kDisplayBaroRegister = kDisplayReadStart,
  kDisplayPot1Register,
  kDisplayPot2Register,
  kDisplayBatteryRegister,
  kDisplayUltrasonicRegister
};

enum AirDataCommand : uint16_t {
  kAirCommandNone = 0,
  kAirCommandLedOff = 1,
  kAirCommandLedOn = 2
};

#endif
