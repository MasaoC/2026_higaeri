#include <ModbusRTU.h>

const int DATASIZE = 10;
#define SLAVE_ID 1
#define REG_READ      0
#define REG_WRITE    10
#define REG_BAUD_CTRL 300
#define DE_PIN D10 // DE pin for RS485. slave=airdataにあわせた。

ModbusRTU mb;
HardwareSerial MySerial0(0);

const long BAUDRATES[] = {9600, 38400, 115200, 921600, 5000000};
const int NUM_BAUDS = 5;
int currentBaudIdx = 0;

// ===================================================================
// CALLBACKS — TRegister* REQUIRED
// ===================================================================

uint16_t cbRead(TRegister* reg, uint16_t oldValue) {
  uint16_t offset = reg->address.address;
  Serial.printf("[READ]  Offset:%d  Value:%d @ %ld baud\n",
                offset, oldValue, BAUDRATES[currentBaudIdx]);
  return oldValue;
}

uint16_t cbWriteBaud(TRegister* reg, uint16_t newValue) {
  uint16_t offset = reg->address.address;

  Serial.printf("[WRITE BAUD] Offset:%d  Value:%d @ %ld baud\n",
                offset, newValue, BAUDRATES[currentBaudIdx]);

  if (offset == REG_BAUD_CTRL && newValue < NUM_BAUDS && newValue != currentBaudIdx) {
    Serial.printf("[BAUD] Switching to index %d (%ld baud)\n", newValue, BAUDRATES[newValue]);
    switchToBaud(newValue);
    mb.Hreg(REG_BAUD_CTRL, 999);  // Ack
  }

  return newValue;
}

uint16_t cbWrite(TRegister* reg, uint16_t newValue) {
  uint16_t offset = reg->address.address;

  if(offset == REG_WRITE){
    Serial.printf("[WRITE] Offset:%d  Value:%d @ %ld baud\n",
                  offset, newValue, BAUDRATES[currentBaudIdx]);
      uint16_t readOffset = offset - REG_WRITE + REG_READ;
      if (readOffset < REG_READ + DATASIZE) {
        mb.Hreg(readOffset, newValue + 1);
        Serial.printf("[ECHO] Wrote %d to offset %d → READ offset %d = %d\n",
                      newValue, offset, readOffset, newValue + 1);
      }
  }
  else if (offset == REG_WRITE + DATASIZE -1) {
    Serial.printf("[WRITE] Offset:%d  Value:%d @ %ld baud\n",
                  offset, newValue, BAUDRATES[currentBaudIdx]);
  }

  return newValue;
}

// ===================================================================
// Safe baud switch
// ===================================================================
void switchToBaud(int idx) {
  long baud = BAUDRATES[idx];
  Serial.printf("\n[SWITCHING] to %ld baud...\n", baud);
  Serial.flush();

  MySerial0.flush();
  MySerial0.end();
  delay(200);  // Increased for stability
  MySerial0.begin(baud, SERIAL_8N1, -1, -1);
  delay(200);

  // Reset RS485 transceiver
  digitalWrite(DE_PIN, HIGH);
  delay(5);
  digitalWrite(DE_PIN, LOW);

  currentBaudIdx = idx;
  Serial.printf("[READY] Now at %ld baud\n\n", baud);
}

// ===================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(1000);
  Serial.println(F("\n=== MODBUS RTU SLAVE - CORRECTED (NO lastCount) ==="));

  pinMode(DE_PIN, OUTPUT);
  digitalWrite(DE_PIN, LOW);

  MySerial0.begin(9600, SERIAL_8N1, -1, -1);
  mb.begin(&MySerial0, DE_PIN);
  mb.slave(SLAVE_ID);

  mb.addHreg(REG_READ,      100, DATASIZE);
  mb.addHreg(REG_WRITE,     0,   DATASIZE);
  mb.addHreg(REG_BAUD_CTRL, 0);

  // Correct: numregs = how many this callback handles
  mb.onGetHreg(REG_READ,      cbRead,      1);        // Only 1, or DATASIZE if you want multi-read
  mb.onSetHreg(REG_WRITE,     cbWrite,     DATASIZE); // Handles up to 100
  mb.onSetHreg(REG_BAUD_CTRL, cbWriteBaud, 1);

  currentBaudIdx = 0;
  Serial.println(F("[READY] 9600 baud"));
}

void loop() {
  unsigned long t1 = micros();
  mb.task();
  unsigned long taskTime = micros() - t1;
  if (taskTime > 1000) {
    Serial.printf("SLAVE task() = %lu us @ %ld bps\n", taskTime, BAUDRATES[currentBaudIdx]);
  }
}