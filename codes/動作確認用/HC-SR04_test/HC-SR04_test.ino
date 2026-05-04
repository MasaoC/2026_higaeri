#include <Arduino.h>

void setup() {
  Serial.begin(115200); // PCシリアルモニタ用 (USB CDC)
  while (!Serial);
  
  // XIAO ESP32C3のデフォルトハードウェアUART (D6/D7ピン) を使用
  // ピン指定は不要
  Serial0.begin(9600); 
}

void loop() {
  while(Serial0.available()) Serial0.read();
  Serial0.write(0xA0);
  
  unsigned long startTime = millis();
  while (Serial0.available() < 3 && millis() - startTime < 300) {
    delay(1);
  }

  if (Serial0.available() >= 3) {
    uint8_t h = Serial0.read();
    uint8_t m = Serial0.read();
    uint8_t l = Serial0.read();

    uint32_t raw = ((uint32_t)h << 16) | ((uint32_t)m << 8) | l;
    Serial.print("Distance: ");
    Serial.print(raw / 1000.0);
    Serial.println(" mm");
  }
  delay(500); 
}