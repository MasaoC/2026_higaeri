#include <ModbusRTU.h>

const int DATASIZE = 10;
#define SLAVE_ID 1
#define REG_READ 0
#define REG_WRITE 10
#define REG_BAUD_CTRL 300   // Special register: write baud index (0..4) to change baud
#define DE_PIN D3 // DE pin for RS485. master=loggerにあわせた。 
#define TRY_COUNT 1

ModbusRTU mb;
HardwareSerial MySerial0(0);

const long BAUDRATES[] = {9600, 38400, 115200, 921600, 5000000};
const int NUM_BAUDS = 5;
int currentBaudIdx = 0;

uint16_t readBuf[DATASIZE];
uint16_t writeBuf[DATASIZE];

int readSuccessCount = 0;
int writeSuccessCount = 0;
bool testInProgress = false;

unsigned long lastActionTime = 0;
const unsigned long ACTION_INTERVAL = 5000;  // Safe delay between operations
const unsigned long BAUD_SWITCH_DELAY = 2000;

unsigned long readStartTime, writeStartTime;

// Callbacks with full timing
bool cbRead(Modbus::ResultCode event, uint16_t transactionId, void* data) {
  unsigned long callbackTime = micros();
  if (event == Modbus::EX_SUCCESS) {
    Serial.printf("READ OK  | Baud: %ld | Val: %d | CallDelay: %lu us | Total: %lu us\n",
                  BAUDRATES[currentBaudIdx], readBuf[0],
                  callbackTime - readStartTime,
                  callbackTime - readStartTime);
    readSuccessCount++;
  } else {
    Serial.printf("READ FAIL| Baud: %ld | Err: 0x%02X\n", BAUDRATES[currentBaudIdx], event);
  }
  return true;
}

bool cbWrite(Modbus::ResultCode event, uint16_t transactionId, void* data) {
  unsigned long callbackTime = micros();
  if (event == Modbus::EX_SUCCESS) {
    Serial.printf("WRITE OK | Baud: %ld | %d regs | CallDelay: %lu us | Total: %lu us\n",
                  BAUDRATES[currentBaudIdx], DATASIZE,
                  callbackTime - writeStartTime,
                  callbackTime - writeStartTime);
    writeSuccessCount++;
  } else {
    Serial.printf("WRITE FAIL| Baud: %ld | Err: 0x%02X\n", BAUDRATES[currentBaudIdx], event);
  }
  return true;
}

bool cbBaudSwitch(Modbus::ResultCode event, uint16_t transactionId, void* data) {
  if (event == Modbus::EX_SUCCESS) {
    Serial.printf("BAUD SWITCH COMMAND SENT | Next: %ld bps\n", BAUDRATES[currentBaudIdx]);
  }
  return true;
}

void switchToBaud(int idx) {
  long baud = BAUDRATES[idx];
  Serial.printf("\n\n===== SWITCHING TO BAUD: %ld =====\n", baud);

  MySerial0.flush();
  delay(20);  // Critical
  MySerial0.end();
  delay(50);
  MySerial0.begin(baud, SERIAL_8N1, -1, -1);
  delay(50);

  // DO NOT call mb.begin() again — it breaks internal state
  // Just reconfigure DE pin
  if (DE_PIN >= 0) {
    pinMode(DE_PIN, OUTPUT);
    digitalWrite(DE_PIN, LOW);
  }

  currentBaudIdx = idx;
  readSuccessCount = 0;
  writeSuccessCount = 0;
  testInProgress = true;
  lastActionTime = millis();
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(1000);
  Serial.println("\nModbus RTU High-Speed Baud Tester - MASTER");
  Serial.println("Full timing preserved | Auto baud via Modbus control register");

  switchToBaud(0);  // Start at 9600

  mb.begin(&MySerial0, DE_PIN);
  mb.master();
}

void loop() {
  unsigned long t1 = micros();
  mb.task();
  unsigned long taskTime = micros() - t1;
  if (taskTime > 1000) {
    Serial.printf("task() = %lu us @ %ld bps\n", taskTime, BAUDRATES[currentBaudIdx]);
  }

  if (!testInProgress) return;

  if (millis() - lastActionTime < ACTION_INTERVAL) return;
  lastActionTime = millis();

  if (readSuccessCount < TRY_COUNT) {
    // READ TEST
    readStartTime = micros();
    Serial.printf("[Baud %ld] READ #%d -> ", BAUDRATES[currentBaudIdx], readSuccessCount + 1);
    if (mb.readHreg(SLAVE_ID, REG_READ, readBuf, 1, cbRead)) {
      Serial.printf("%lu us (sent)\n", micros() - readStartTime);
    }

  } else if (writeSuccessCount < TRY_COUNT) {
    // WRITE TEST
    uint16_t val = 1000 + writeSuccessCount + (currentBaudIdx * 100);
    for (int i = 0; i < DATASIZE; i++) writeBuf[i] = val + i;

    writeStartTime = micros();
    Serial.printf("[Baud %ld] WRITE #%d (val=%d) -> ", BAUDRATES[currentBaudIdx], writeSuccessCount + 1, val);
    if (mb.writeHreg(SLAVE_ID, REG_WRITE, writeBuf, DATASIZE, cbWrite)) {
      Serial.printf("%lu us (sent)\n", micros() - writeStartTime);
    }

  } else {
    // All 3 reads + 3 writes done
    Serial.printf("BAUD %ld: ALL TESTS PASSED\n", BAUDRATES[currentBaudIdx]);

    // WITH THIS:
    if (currentBaudIdx < NUM_BAUDS - 1) {
      uint16_t nextIdx = currentBaudIdx + 1;
      Serial.printf("REQUESTING BAUD SWITCH TO INDEX %d\n", nextIdx);
      mb.writeHreg(SLAVE_ID, REG_BAUD_CTRL, &nextIdx, 1, cbBaudSwitch);
      delay(500);  // Give slave time to process
      switchToBaud(nextIdx);
      delay(BAUD_SWITCH_DELAY);
    } else {
      Serial.println("\nALL 5 BAUD RATES TESTED SUCCESSFULLY!");
      Serial.println("Test complete. Reset to rerun.");
      testInProgress = false;
      while (true) delay(1000);
    }
  }
}