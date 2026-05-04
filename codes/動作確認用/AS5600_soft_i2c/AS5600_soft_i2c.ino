#include <Arduino.h>
#include <SoftwareI2C.h>

// air_dataのAoA/AoSと同じピン設定（ソフトウェアI2C）
constexpr uint8_t kAosSdaPin = D2;
constexpr uint8_t kAosSclPin = D3;

constexpr uint8_t kAs5600Address  = 0x36;
constexpr uint8_t kAs5600RegAngle = 0x0C;

SoftwareI2C softWire;

void setup() {
  Serial.begin(115200);
  // シリアルポートの初期化待ち
  delay(1000);
  Serial.println("AS5600 Soft I2C Test Started");

  // SoftwareI2Cライブラリで初期化
  softWire.begin(kAosSdaPin, kAosSclPin);
}

void loop() {
  softWire.beginTransmission(kAs5600Address);
  softWire.write(kAs5600RegAngle);
  
  // I2C通信のエラーチェック (SoftwareI2Cでは引数なし)
  softWire.endTransmission();
  
  // 2バイト（12ビット分）の角度データを要求
  // 戻り値は 1: ACK(成功), 0: NAK(失敗) なので 1以外ならエラー
  if (softWire.requestFrom(kAs5600Address, (uint8_t)2) != 1) {
    Serial.println("AS5600 Read Error");
    delay(1000);
    return;
  }
  
  // 上位バイトの下位4ビットと下位バイトを結合して12ビット値を取得
  uint16_t rawAngle = ((uint16_t)softWire.read() << 8) & 0x0F00;
  rawAngle |= (uint16_t)softWire.read();
  
  // 0~4095 の値が出力される
  Serial.printf("Raw Angle: %u\n", rawAngle);
  
  delay(100);
}
