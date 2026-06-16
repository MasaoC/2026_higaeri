// display_d1.ino - SLAVE (Modbus RTU Slave, ID = 2)
// XIAO ESP32C3
// Using modbus-esp8266 library (verified working 2026-03-28)

#include <HardwareSerial.h>
#include <ModbusRTU.h>

#define BAUDRATE     9600UL
#define DE_PIN       D3      // GPIO5 (DE/RE control pin for MAX485)
#define SLAVE_ID     2
#define REGN_SENSOR  0       // Holding registers 0-3: display data

HardwareSerial MySerial0(0);
ModbusRTU mb;

unsigned long lastPrintMs = 0;
uint32_t loopCount = 0;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("[DISPLAY_D1 SLAVE] Starting...");

  // Initialize UART0 with Modbus RTU standard format (8N1)
  MySerial0.begin(BAUDRATE, SERIAL_8N1, -1, -1);

  // Initialize Modbus RTU server (slave) with TX enable pin for RS485
  // Parameter: &serial, DE_PIN
  mb.begin(&MySerial0, DE_PIN);
  
  // Set slave mode with ID
  mb.slave(SLAVE_ID);
  
  // Add holding registers (4 registers at offset 0)
  mb.addHreg(REGN_SENSOR, 0, 4);

  Serial.printf("[DISPLAY_D1 SLAVE] id=%u  baud=%lu  de=GPIO%d\n", SLAVE_ID, BAUDRATE, DE_PIN);
}

void loop() {
  // Update simulated data in holding registers
  mb.Hreg(REGN_SENSOR + 0, (uint16_t)(millis() / 200) % 1000);
  mb.Hreg(REGN_SENSOR + 1, (uint16_t)(millis() / 150) % 1000);
  mb.Hreg(REGN_SENSOR + 2, (uint16_t)(millis() / 300) % 1000);
  mb.Hreg(REGN_SENSOR + 3, (uint16_t)(millis() / 250) % 1000);

  // Process Modbus protocol messages
  mb.task();
  yield();

  // Print current register values periodically
  if (millis() - lastPrintMs >= 2000) {
    lastPrintMs = millis();
    Serial.printf("[DISPLAY_D1 SLAVE] regs: %u %u %u %u\n",
      mb.Hreg(REGN_SENSOR + 0),
      mb.Hreg(REGN_SENSOR + 1),
      mb.Hreg(REGN_SENSOR + 2),
      mb.Hreg(REGN_SENSOR + 3));
  }

  if (++loopCount % 50000 == 0) {
    Serial.printf("[DISPLAY_D1 SLAVE] alive  loop=%lu\n", (unsigned long)loopCount);
  }
}