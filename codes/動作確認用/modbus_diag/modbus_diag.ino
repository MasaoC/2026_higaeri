// =============================================================================
// Modbus RTU 診断スケッチ（loggerマスター側で実行）
// =============================================================================
// 【使い方】
//   TEST_MODE を 1〜5 に変えてコンパイル・書き込み、シリアルモニタで結果を確認する。
//   各テストの判定基準はコメントを参照。
//
// 【ピン配置はlogger.inoと同一】
//   DE  = D2, RX = D7(GPIO20), TX = D6(GPIO21)
//   Baud = 9600, UART0
// =============================================================================

#include <Arduino.h>
#include <HardwareSerial.h>

constexpr uint8_t  kDePin  = D2;
constexpr uint8_t  kRxPin  = D7;   // GPIO20
constexpr uint8_t  kTxPin  = D6;   // GPIO21
constexpr uint32_t kBaud   = 9600;
constexpr uint8_t  kSlaveId = 1;   // air_data

// ── テストを選択（1〜5）─────────────────────────────────────────────────────
constexpr int TEST_MODE = 1;
// 1 = UART ループバックテスト（TX─RX 直結が必要）
// 2 = DE ピン制御テスト（テスタ or LEDで確認）
// 3 = 生 Modbus RTU フレーム送信テスト（スレーブ側を起動した状態で）
// 4 = タイムアウト段階テスト（スレーブ側を起動した状態で）
// 5 = スレーブ応答解析テスト（受信バイトを全てダンプ）
// =============================================================================

HardwareSerial ModbusSerial(0);

// ─── CRC16 (Modbus RTU) ───────────────────────────────────────────────────────
static uint16_t crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 0x0001) { crc = (crc >> 1) ^ 0xA001; }
      else               { crc >>= 1; }
    }
  }
  return crc;
}

// ─── 送信ヘルパー（DEをHIGHにして送信、完了後LOW）────────────────────────────
static void sendRaw(const uint8_t* buf, size_t len) {
  digitalWrite(kDePin, HIGH);
  delayMicroseconds(100);  // DE立ち上がり安定待ち
  ModbusSerial.write(buf, len);
  ModbusSerial.flush();
  delayMicroseconds(100);
  digitalWrite(kDePin, LOW);
}

// =============================================================================
// TEST 1: UART ループバックテスト
// 【準備】TX(D6) と RX(D7) をジャンパで直結する（RS-485 ICは経由しない）
// 【判定】PASS → ピン割り当てとUARTは正常
//         FAIL → ピン番号ミスか UART の設定不備
// =============================================================================
static void test1_uartLoopback() {
  Serial.println("=== TEST 1: UART Loopback ===");
  Serial.println("【準備】D6(TX)とD7(RX)をジャンパ直結してください");
  Serial.println("5秒待ちます...");
  delay(5000);

  ModbusSerial.begin(kBaud, SERIAL_8N1, kRxPin, kTxPin);

  const char* testStr = "MODBUS_LOOPBACK_TEST";
  const size_t len = strlen(testStr);

  // 受信バッファをクリア
  while (ModbusSerial.available()) ModbusSerial.read();

  // DEをLOWにしてから送信（自己受信テスト）
  digitalWrite(kDePin, LOW);
  ModbusSerial.print(testStr);
  ModbusSerial.flush();

  delay(50);

  char rxBuf[32] = {};
  int rxLen = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 500 && rxLen < (int)len) {
    if (ModbusSerial.available()) {
      rxBuf[rxLen++] = (char)ModbusSerial.read();
    }
  }

  Serial.printf("送信: \"%s\" (%d bytes)\n", testStr, len);
  Serial.printf("受信: \"%s\" (%d bytes)\n", rxBuf, rxLen);

  if (rxLen == (int)len && memcmp(testStr, rxBuf, len) == 0) {
    Serial.println(">> PASS: UART ピン配置・設定は正常");
  } else {
    Serial.println(">> FAIL: ピン番号の確認が必要（D6=GPIO21=TX, D7=GPIO20=RX）");
    Serial.println("         または UART0 が他で使用されている");
  }
}

// =============================================================================
// TEST 2: DE ピン制御テスト
// 【準備】D2ピンにテスタを当てるか、LEDを接続する
// 【判定】HIGH/LOW が 1秒ごとに切り替わることを確認
// =============================================================================
static void test2_dePin() {
  Serial.println("=== TEST 2: DE Pin Control ===");
  Serial.println("D2ピンを測定してください。1秒ごとにHIGH/LOWが切り替わります");
  Serial.println("Ctrl+C（リセット）で終了");

  for (int i = 0; i < 10; i++) {
    digitalWrite(kDePin, HIGH);
    Serial.printf("[%d] DE = HIGH (測定値は3.3V)\n", i);
    delay(1000);
    digitalWrite(kDePin, LOW);
    Serial.printf("[%d] DE = LOW  (測定値は0V)\n", i);
    delay(1000);
  }
  Serial.println(">> テスタで確認した値が一致すれば DE ピン正常");
}

// =============================================================================
// TEST 3: 生 Modbus RTU フレーム送信テスト
// 【準備】air_data スレーブを起動した状態でバスを接続
// 【判定】応答が 7 バイト受信できれば通信成立
//         0バイト → スレーブ未起動 or バス配線ミス or A/B逆接
// =============================================================================
static void test3_rawFrame() {
  Serial.println("=== TEST 3: Raw Modbus RTU Frame ===");
  Serial.println("air_dataスレーブを起動した状態で実行してください");
  delay(2000);

  ModbusSerial.begin(kBaud, SERIAL_8N1, kRxPin, kTxPin);
  while (ModbusSerial.available()) ModbusSerial.read();

  // FC03: slave=1, start=0x0000, count=4
  uint8_t req[6] = { kSlaveId, 0x03, 0x00, 0x00, 0x00, 0x04 };
  uint16_t crc = crc16(req, 6);
  uint8_t frame[8];
  memcpy(frame, req, 6);
  frame[6] = (uint8_t)(crc & 0xFF);        // CRC low byte
  frame[7] = (uint8_t)((crc >> 8) & 0xFF); // CRC high byte

  Serial.print("送信フレーム: ");
  for (int i = 0; i < 8; i++) Serial.printf("%02X ", frame[i]);
  Serial.println();

  sendRaw(frame, 8);

  // 応答待ち（最大500ms）
  uint8_t rxBuf[32] = {};
  int rxLen = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 500 && rxLen < 32) {
    if (ModbusSerial.available()) {
      rxBuf[rxLen++] = ModbusSerial.read();
    }
  }

  Serial.printf("受信バイト数: %d\n", rxLen);
  Serial.print("受信データ: ");
  for (int i = 0; i < rxLen; i++) Serial.printf("%02X ", rxBuf[i]);
  Serial.println();

  if (rxLen == 0) {
    Serial.println(">> FAIL: 応答なし");
    Serial.println("   確認1: air_dataスレーブが起動しているか");
    Serial.println("   確認2: RS-485バスのA/B線が逆接されていないか");
    Serial.println("   確認3: バス終端抵抗（120Ω）の有無");
    Serial.println("   確認4: 双方の GND が接続されているか");
  } else if (rxLen == 8 && rxBuf[0] == kSlaveId && rxBuf[1] == 0x03) {
    // エコー（送信した自分のフレームを受信している）
    Serial.println(">> WARN: 受信データが送信フレームと一致 → エコーの可能性");
    Serial.println("         RS-485の送受信が正しく切り替わっているか確認");
  } else if (rxLen >= 7 && rxBuf[0] == kSlaveId && rxBuf[1] == 0x03) {
    uint16_t rxCrc = crc16(rxBuf, rxLen - 2);
    uint16_t expectedCrc = (uint16_t)rxBuf[rxLen-2] | ((uint16_t)rxBuf[rxLen-1] << 8);
    if (rxCrc == expectedCrc) {
      Serial.println(">> PASS: 正常な Modbus 応答を受信、CRC 一致");
      // データ値を表示
      for (int r = 0; r < 4; r++) {
        uint16_t val = ((uint16_t)rxBuf[3 + r*2] << 8) | rxBuf[4 + r*2];
        Serial.printf("   Hreg[%d] = %u\n", r, val);
      }
    } else {
      Serial.printf(">> FAIL: CRC不一致 (計算値=%04X 受信値=%04X)\n", rxCrc, expectedCrc);
      Serial.println("         ノイズ or ボーレートの不一致");
    }
  } else if (rxLen > 0 && rxBuf[1] == 0x83) {
    Serial.printf(">> FAIL: スレーブがエラー応答 (例外コード=0x%02X)\n", rxBuf[2]);
    Serial.println("         0x01=ILLEGAL_FUNCTION, 0x02=ILLEGAL_ADDRESS, 0x03=ILLEGAL_VALUE");
  } else {
    Serial.println(">> FAIL: 不完全な応答 (ノイズ or ボーレート不一致の可能性)");
  }
}

// =============================================================================
// TEST 4: タイムアウト段階テスト
// 【準備】スレーブ起動・バス接続済み
// 【判定】どのタイムアウト値で応答するかを確認
// =============================================================================
static void test4_timeoutSweep() {
  Serial.println("=== TEST 4: Timeout Sweep ===");
  delay(2000);

  ModbusSerial.begin(kBaud, SERIAL_8N1, kRxPin, kTxPin);

  const uint32_t timeouts[] = {50, 100, 200, 300, 500, 1000, 2000};
  for (uint32_t timeout : timeouts) {
    while (ModbusSerial.available()) ModbusSerial.read();

    uint8_t req[6] = { kSlaveId, 0x03, 0x00, 0x00, 0x00, 0x01 };
    uint16_t crc = crc16(req, 6);
    uint8_t frame[8];
    memcpy(frame, req, 6);
    frame[6] = (uint8_t)(crc & 0xFF);
    frame[7] = (uint8_t)((crc >> 8) & 0xFF);

    sendRaw(frame, 8);

    int rxLen = 0;
    uint32_t t0 = millis();
    while (millis() - t0 < timeout) {
      if (ModbusSerial.available()) rxLen++;
      ModbusSerial.read();
    }

    Serial.printf("timeout=%4ums → 受信=%d bytes %s\n",
                  timeout, rxLen, rxLen >= 6 ? "PASS" : "---");
    delay(100);
  }
  Serial.println();
  Serial.println(">> logger/ModbusConfig.h の kModbusTimeoutMs (現在300ms) が");
  Serial.println("   PASSになる最小値以上に設定されているか確認してください");
}

// =============================================================================
// TEST 5: 全バイトダンプテスト
// 【準備】スレーブ起動・バス接続済み。1000msウィンドウで全受信データを表示
// =============================================================================
static void test5_fullDump() {
  Serial.println("=== TEST 5: Full Byte Dump ===");
  Serial.println("500ms間の受信バイトを全てダンプします（毎秒実行）");
  Serial.println("リセットで終了");
  delay(2000);

  ModbusSerial.begin(kBaud, SERIAL_8N1, kRxPin, kTxPin);

  int loopCount = 0;
  while (true) {
    while (ModbusSerial.available()) ModbusSerial.read();

    // FC03 req: slave=1, addr=0, count=4
    uint8_t req[6] = { kSlaveId, 0x03, 0x00, 0x00, 0x00, 0x04 };
    uint16_t crc = crc16(req, 6);
    uint8_t frame[8];
    memcpy(frame, req, 6);
    frame[6] = (uint8_t)(crc & 0xFF);
    frame[7] = (uint8_t)((crc >> 8) & 0xFF);

    sendRaw(frame, 8);

    uint8_t rxBuf[32] = {};
    int rxLen = 0;
    uint32_t t0 = millis();
    while (millis() - t0 < 1000 && rxLen < 32) {
      if (ModbusSerial.available()) {
        rxBuf[rxLen++] = ModbusSerial.read();
      }
    }

    Serial.printf("[%d] req: ", loopCount++);
    for (int i = 0; i < 8; i++) Serial.printf("%02X ", frame[i]);
    Serial.printf("  rx(%d): ", rxLen);
    for (int i = 0; i < rxLen; i++) Serial.printf("%02X ", rxBuf[i]);
    if (rxLen == 0) Serial.print("(なし)");
    Serial.println();

    delay(500);
  }
}

// =============================================================================

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n=== Modbus RTU 診断ツール ===");
  Serial.printf("TEST_MODE = %d\n\n", TEST_MODE);

  pinMode(kDePin, OUTPUT);
  digitalWrite(kDePin, LOW);

  switch (TEST_MODE) {
    case 1: test1_uartLoopback(); break;
    case 2: test2_dePin();        break;
    case 3: test3_rawFrame();     break;
    case 4: test4_timeoutSweep(); break;
    case 5: test5_fullDump();     break;
    default:
      Serial.println("TEST_MODE を 1〜5 に設定してください");
      break;
  }

  Serial.println("\n=== 診断完了 ===");
}

void loop() {
  // TEST 5 は setup() 内でループするため、ここは何もしない
}
