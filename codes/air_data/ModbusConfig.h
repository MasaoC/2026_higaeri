#ifndef MODBUS_CONFIG_H
#define MODBUS_CONFIG_H

#include <Arduino.h>

constexpr uint32_t kDebugBaudRate = 115200;
constexpr uint32_t kModbusBaudRate = 9600;
constexpr uint32_t kModbusTimeoutMs = 100;
constexpr uint8_t kAirDataSlaveId = 1;

constexpr uint16_t kAirDataReadStart = 0;
constexpr uint16_t kAirDataReadCount = 4;
constexpr uint16_t kAirDataWriteStart = 10;
constexpr uint16_t kAirDataWriteCount = 1;

enum AirDataRegister : uint16_t {
	kAirWindSpeedRegister = kAirDataReadStart,
	kAirAs5600PrimaryRegister,
	kAirAs5600SecondaryRegister,
	kAirBatteryRegister
};

enum AirDataCommand : uint16_t {
	kAirCommandNone = 0,
	kAirCommandLedOff = 1,
	kAirCommandLedOn = 2
};

#endif
