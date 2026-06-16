// logger.ino - MASTER (Modbus RTU Master)
// XIAO ESP32C3
// Using modbus-esp8266 library (verified working 2026-03-28)

#include <HardwareSerial.h>
#include <ModbusRTU.h>

#define BAUDRATE          9600UL
#define DE_PIN            D3      // GPIO4 (DE/RE control pin for MAX485)
#define SLAVE_ID_AIR_DATA 1
#define SLAVE_ID_DISPLAY  2
#define REGN_SENSOR       0       // Holding registers 0-3: sensor data
#define SENSOR_DATA_SIZE  4

HardwareSerial MySerial0(0);
ModbusRTU mb;

// Read buffer for slave responses
uint16_t g_readBuf[SENSOR_DATA_SIZE] = {0};

// Transaction tracking for asynchronous reads
bool readComplete = true;
uint16_t currentSlave = 0;
uint8_t readState = 0;  // 0: read air_data, 1: read display

// Callback function for read completion
bool onReadComplete(Modbus::ResultCode event, uint16_t transactionId, void* data) {
  if (event == 0) { // Success (Modbus::EX_SUCCESS)
    if (currentSlave == SLAVE_ID_AIR_DATA) {
      Serial.printf("[LOGGER MASTER] air_data  reg0=%u  reg1=%u  reg2=%u  reg3=%u\n",
        g_readBuf[0], g_readBuf[1], g_readBuf[2], g_readBuf[3]);
    } else if (currentSlave == SLAVE_ID_DISPLAY) {
      Serial.printf("[LOGGER MASTER] display   reg0=%u  reg1=%u  reg2=%u  reg3=%u\n",
        g_readBuf[0], g_readBuf[1], g_readBuf[2], g_readBuf[3]);
    }
  } else {
    Serial.printf("[LOGGER MASTER] slave %d READ FAILED (event=0x%02X)\n", currentSlave, event);
  }
  readComplete = true;
  return true;
}

unsigned long lastReadMs = 0;
uint32_t cycleCount = 0;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("[LOGGER MASTER] Starting...");

  // Initialize UART0 with Modbus RTU standard format (8N1)
  MySerial0.begin(BAUDRATE, SERIAL_8N1, -1, -1);

  // Initialize Modbus RTU master with TX enable pin for RS485
  mb.begin(&MySerial0, DE_PIN);
  
  // Set master mode
  mb.master();

  Serial.printf("[LOGGER MASTER] baud=%lu  de=GPIO%d  slave1=%d  slave2=%d\n",
    BAUDRATE, DE_PIN, SLAVE_ID_AIR_DATA, SLAVE_ID_DISPLAY);
}

void loop() {
  // Process Modbus messages (critical: call every cycle)
  mb.task();
  yield();

  // Read cycle: trigger reads at ~1 second intervals
  if (millis() - lastReadMs < 1000) return;
  lastReadMs = millis();

  // State machine: alternate between reading slave1 and slave2
  if (readComplete) {
    if (readState == 0) {
      // Read air_data slave (ID=1)
      Serial.printf("\n[LOGGER MASTER] ===== cycle %lu =====\n", (unsigned long)++cycleCount);
      currentSlave = SLAVE_ID_AIR_DATA;
      mb.readHreg(SLAVE_ID_AIR_DATA, REGN_SENSOR, g_readBuf, SENSOR_DATA_SIZE, onReadComplete);
      readComplete = false;
      readState = 1;  // Next, read display
    } else {
      // Read display slave (ID=2)
      currentSlave = SLAVE_ID_DISPLAY;
      mb.readHreg(SLAVE_ID_DISPLAY, REGN_SENSOR, g_readBuf, SENSOR_DATA_SIZE, onReadComplete);
      readComplete = false;
      readState = 0;  // Next cycle, read air_data again
    }
  }
}