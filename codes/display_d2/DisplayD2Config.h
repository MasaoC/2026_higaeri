#ifndef DISPLAY_D2_CONFIG_H
#define DISPLAY_D2_CONFIG_H

#include <Arduino.h>

constexpr uint32_t kDebugBaudRate = 115200;
constexpr uint32_t kDisplayD2SensorIntervalMs = 300;  // 超音波センサは200ms以上必要、余裕をもって300ms
constexpr uint8_t kDisplayD2I2CAddress = 0x30;
constexpr uint8_t kDisplayD2PayloadSize = 12;

#endif