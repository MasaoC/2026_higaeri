/*
 * pot_calib_ble.ino
 *
 * Display基板 (XIAO ESP32-C3) のポテンショメータ生値を
 * 本番オーバーサンプリングと同じサンプリングレート (100 Hz / 10 ms) で
 * BLE (Nordic UART Service) 送信する校正用スケッチ。
 * オーバーサンプリング（移動平均）は行わない。
 *
 * ハードウェア構成 (display_d2 と同一基板):
 *   ポテンショメータ 1 : A0
 *   ポテンショメータ 2 : A1
 *   ステータス LED      : D10
 *
 * BLE 送信フォーマット (CSV, LF 終端):
 *   timestamp_ms,pot1_raw,pot2_raw
 *
 * 接続方法:
 *   NUS (Nordic UART Service) 対応の BLE ターミナルまたは
 *   receive_ble.py で "PotCalib-BLE" に接続し、
 *   TX Characteristic (6E400003-...) の Notify を登録する。
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <esp_timer.h>

// ── ピン定義 (display_d2 と同一) ─────────────────────────────────────────
constexpr uint8_t kPot1Pin      = A0;
constexpr uint8_t kPot2Pin      = A1;
constexpr uint8_t kStatusLedPin = D10;

// ── サンプリング設定 ──────────────────────────────────────────────────────
// display_d2 の kAnalogUpdateIntervalMs (10 ms) と同一の本番サンプリングレート。
// オーバーサンプリング（SmoothingFilter<10>）は行わず、生値をそのまま送信。
constexpr uint32_t kSampleIntervalUs = 10000UL; // 10 ms = 100 Hz

// ── BLE Nordic UART Service (NUS) UUID ───────────────────────────────────
static const char* kNusServiceUuid = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
static const char* kNusTxUuid      = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";
static const char* kNusRxUuid      = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";

// ── タイマー → ループ間 共有フラグ ─────────────────────────────────────
static volatile bool g_sampleDue = false;

// ── BLE ──────────────────────────────────────────────────────────────────
static BLECharacteristic* g_txChar       = nullptr;
static bool               g_bleConnected = false;

// ── タイマーコールバック (ESP_TIMER_TASK ディスパッチ) ────────────────────
// フラグを立てるだけ。analogRead はメインループで実施。
static void onSampleTimer(void* /*arg*/) {
  g_sampleDue = true;
}

// ── BLE 接続コールバック ──────────────────────────────────────────────────
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    g_bleConnected = true;
    digitalWrite(kStatusLedPin, HIGH);
    Serial.println("[BLE] connected");
  }
  void onDisconnect(BLEServer*) override {
    g_bleConnected = false;
    digitalWrite(kStatusLedPin, LOW);
    Serial.println("[BLE] disconnected – restart advertising");
    BLEDevice::startAdvertising();
  }
};

// ── setup ─────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(100);

  pinMode(kStatusLedPin, OUTPUT);
  digitalWrite(kStatusLedPin, LOW);
  pinMode(kPot1Pin, INPUT);
  pinMode(kPot2Pin, INPUT);

  // ── BLE 初期化 ────────────────────────────────────────────────────────
  BLEDevice::init("PotCalib-BLE");
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService* svc = server->createService(kNusServiceUuid);

  // TX Characteristic: ESP32 → PC (Notify)
  g_txChar = svc->createCharacteristic(
    kNusTxUuid, BLECharacteristic::PROPERTY_NOTIFY);
  g_txChar->addDescriptor(new BLE2902());

  // RX Characteristic: NUS 互換のため宣言（受信処理なし）
  svc->createCharacteristic(kNusRxUuid, BLECharacteristic::PROPERTY_WRITE);

  svc->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(kNusServiceUuid);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  BLEDevice::startAdvertising();

  Serial.println("[BLE] advertising as PotCalib-BLE");

  // ── 周期タイマー開始 ──────────────────────────────────────────────────
  esp_timer_handle_t timerHandle;
  esp_timer_create_args_t timerArgs = {};
  timerArgs.callback        = onSampleTimer;
  timerArgs.dispatch_method = ESP_TIMER_TASK;
  timerArgs.name            = "sample_timer";
  esp_timer_create(&timerArgs, &timerHandle);
  esp_timer_start_periodic(timerHandle, kSampleIntervalUs);

  Serial.println("[timer] 100 Hz sampling started (10 ms interval, no oversampling)");
}

// ── loop ──────────────────────────────────────────────────────────────────
void loop() {
  if (!g_sampleDue) return;
  g_sampleDue = false;

  const uint32_t tsMs = (uint32_t)(esp_timer_get_time() / 1000ULL);
  const uint16_t pot1 = (uint16_t)analogRead(kPot1Pin);
  const uint16_t pot2 = (uint16_t)analogRead(kPot2Pin);

  // シリアルデバッグ出力（USB 接続時）
  Serial.printf("%lu,%u,%u\n", (unsigned long)tsMs, pot1, pot2);

  // BLE 送信（未接続時はスキップ）
  if (g_bleConnected && g_txChar) {
    char buf[32];
    const int len = snprintf(buf, sizeof(buf), "%lu,%u,%u\n",
                             (unsigned long)tsMs, pot1, pot2);
    g_txChar->setValue((uint8_t*)buf, (size_t)len);
    g_txChar->notify();
  }
}
