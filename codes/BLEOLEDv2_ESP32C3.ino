// UARTを受信し、OLED表示　及び　BLE送信　します。
// For ESP32C3 XIAO
// Author: Masao.C
// Last update: 2025.5.26
// Visit https://github.com/MasaoC/uart2ble_oled for detail.

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <Wire.h> 
#include <Adafruit_SSD1306.h>

//OLED
#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 32 
#define OLED_RESET -1 
#define BAUD_RATE 38400


bool display_working = true;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//Define two Serial devices mapped to the two internal UARTs
HardwareSerial MySerial1(1);
HardwareSerial MySerial0(0);//TX=D9(dummy), RX=D10

//定数
const int BUFFER_SIZE = 90;//受信バッファー.  86文字+\r+\n と少し余裕持たせてみた。
const int TIMEOUT_UART = 3000;// If no signal for TIMEOUT_UART ms, then NO SIGNAL... will be displayed.
const char NEXT_LINE_CHAR = '~';//改行とする文字
const char ENDOFDATA = '\n';//データ終了(区切り)文字
const char BLE_BEGIN_TOKEN = '#';
const char BIGFONT_TOKEN = '!';
const char BIGGEST_FONT_TOKEN = '$';
const unsigned long UPDATE_BLE_TIME = 3000;


//変数
uint8_t buf[BUFFER_SIZE];//受信バッファー(char型でもいいのだが、BLEライブラリとの親和性のため uint8_tとする。)
bool ledMode = true;//LED ON/OFF
bool RXisUsed = false;// D7ハードシリアル受信中。2ポートのうち、今どっちが使われいるのか判定するために使用する。
bool D10isUsed = false;// ソフトシリアル受信中。

unsigned long last_ble_notifytime = 0;//最後のled点灯・消灯時刻 ms
unsigned long last_message_time = 0;//最後にUART受信した時刻 ms

//Bluetooth機能を有効にする場合、#define BLE する。無効にする場合コメントアウトする。
#define BLE

#ifdef BLE
  #define BLE_NAME "ALBA TAIYO OLED v2"
  uint8_t connection_count = 0;
    
  BLEServer *pServer = NULL;
  BLECharacteristic * pTxCharacteristic;
  bool deviceConnected = false;
  bool oldDeviceConnected = false;
  uint8_t txValue = 0;

  #define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
  #define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E" // NORDIC RX Characteristic UUID
  #define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E" // NORDIC TX Characteristic UUID


  class MyServerCallbacks: public BLEServerCallbacks {
      void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        BLEDevice::startAdvertising();
        connection_count++;
        Serial.print("connected to ");
        /*
        map<short unsigned int, conn_status_t> peers = pServer->getPeerDevices(true);
        map<short unsigned int, conn_status_t>::iterator it;
        for (it = peers.begin(); it != peers.end(); it++)
        {
            Serial.print(it->first);
            Serial.println(it->second.mtu);
        }*/
        Serial.println(connection_count);
      };

      void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("disconnected");
        connection_count--;
      }
  };

  class MyCallbacks: public BLECharacteristicCallbacks {
      void onWrite(BLECharacteristic *pCharacteristic) {
        String rxValue = pCharacteristic->getValue();

        if (rxValue.length() > 0) {
          Serial.println("*********");
          Serial.print("Received Value: ");
          for (int i = 0; i < rxValue.length(); i++)
            Serial.print(rxValue[i]);

          Serial.println();
          Serial.println("*********");

          //電圧readして返信する。
          double val = analogRead(A0)*3.14/4095*10+0.13;//calibrated
          //uint8_t valchar[10];
          //char valcharc[10];
          //dtostrf(val, 5, 2, valcharc);
          //valchar[5] = ENDOFDATA;
          pTxCharacteristic->setValue(String(val));
          pTxCharacteristic->notify();
          last_ble_notifytime = millis();
        }
      }
  };
#endif

void printOLED(const char* chars){
  if(display_working){
    display.clearDisplay();
    display.setCursor(0,0);
    display.setTextSize(1);
    display.println(chars);
    display.display();
  }
}


//使用状況確認
void checkUsage(){
  
  if(MySerial0.available() > 0){
    RXisUsed = true;
    printOLED("RX UART RECEIVED");
    last_message_time = millis();
    return;
  }
  if(MySerial1.available() > 0){
    D10isUsed = true;
    printOLED("D10 UART RECEIVED");
    last_message_time = millis();
  }
}


void setup() {
  Serial.begin(BAUD_RATE);
  #ifdef BLE
    // Create the BLE Device
    BLEDevice::init(BLE_NAME);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0, ESP_PWR_LVL_P9);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL1, ESP_PWR_LVL_P9);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL2, ESP_PWR_LVL_P9);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL3, ESP_PWR_LVL_P9);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL4, ESP_PWR_LVL_P9);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL5, ESP_PWR_LVL_P9);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL6, ESP_PWR_LVL_P9);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL8, ESP_PWR_LVL_P9);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    // Create the BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);
    // Create a BLE Characteristic
    pTxCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_READ   |
                        BLECharacteristic::PROPERTY_NOTIFY |
                        BLECharacteristic::PROPERTY_INDICATE
                    );
    pTxCharacteristic->addDescriptor(new BLE2902());
    BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_RX,
                        BLECharacteristic::PROPERTY_WRITE
                      );
    pRxCharacteristic->setCallbacks(new MyCallbacks());
    // Start the service
    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x6);//iPhone おまじない
    pAdvertising->setMinPreferred(0x12);//iPhone おまじない
    Serial.println("ble advertising");
    BLEDevice::startAdvertising();
  #endif


  Wire.begin();
  Wire.beginTransmission(0x3C);
  int statusI2C = Wire.endTransmission();
  if( statusI2C != 0 ){
    Serial.print( 0x3C );
    Serial.println("I2C 0x3C not found");
    display_working = false;
  }
  if(display_working)
    display_working = display.begin(SSD1306_SWITCHCAPVCC,0x3C);
  
  if(display_working){
    display.setTextColor(WHITE);  
    display.clearDisplay();
    display.setCursor(0,0);
    display.setTextSize(2);
    display.println("HELLO!");
    display.setCursor(0,16);
    display.println("ALBATROSS!");
    display.display(); 

    Serial.println("display SD1306 success");
  }else{
    Serial.println("SD1306 OLED not found.");
  }

  MySerial1.begin(BAUD_RATE, SERIAL_8N1, D10, D9);
  MySerial0.begin(BAUD_RATE, SERIAL_8N1, -1, -1);//D10=RX, D9=TX dummy 余りpin。
  Serial.println("hserial");

  //readBytesUntil のタイムアウト時間を設定する。最初のbitから、データを最後まで受信しきるまでの時間は少なくとも待ちたい。
  //なので最小値は、約90bit/38400bps*1000=2.34 [ms] なので、最大100msも待てば最後まで受信できるはず。 
  Serial.setTimeout(100);
  MySerial0.setTimeout(100);
  MySerial1.setTimeout(100);

  Serial.println("Activated Waiting connection...");
  pinMode(A0, INPUT);
  delay(2000);
}

//buff に読み込んだ、シリアル受信char文字列をデコードしOLEDに表示する。
//NEXT_LINE_CHAR=[~]文字を使用することで強制改行 ができる。ただしOLED側都合で、4行まで。
//一文字目が[!]であるとフォントサイズが大きくなる。
//1文字目が[#]であると、BLEに転送する。
void decode_print_buff(int rlen){
  if(rlen <= 0 || rlen >= BUFFER_SIZE){
    return;
  }
  last_message_time = millis();

  #ifdef BLE
    //BLE開始文字だったら、
    if(buf[0] == BLE_BEGIN_TOKEN){
        buf[rlen] = ENDOFDATA;//plotter用
        //BLE_BEGIN_TOKENの一文字を飛ばして、送信文字列にENDOFDATAまで含める。（あえて+1-1は残した）
        pTxCharacteristic->setValue(buf+1,rlen-1+1);
        pTxCharacteristic->notify();
        last_ble_notifytime = millis();
        return;
    }
  #else
    //BLE　文字列はdecode せず無視する。
    if(buf[0] == BLE_BEGIN_TOKEN){
      return;
    }
  #endif
  if(!display_working){
    return;
  }

  char firstline[rlen+1];//終了文字('\0')のために+1
  char secondline[rlen+1];//まだ2行目の長さは不明のため、同じ長さ確保。
  char thirdline[rlen+1];//まだ3行目の長さは不明のため、同じ長さ確保。
  char fourthline[rlen+1];//まだ4行目の長さは不明のため、同じ長さ確保。
  
  int index_first_line = 0;
  int index_second_line = -1;
  int index_third_line = -1;
  int index_fourth_line = -1;

  int font_size = 1;
  int cursor_height = 8;
  int buf_index = 0;
  if(buf[0]==BIGFONT_TOKEN){
    font_size = 2;
    cursor_height = 16;
    buf_index = 1;
  }
  else if(buf[0] == BIGGEST_FONT_TOKEN){
    font_size = 3;
    cursor_height = 32;
    buf_index = 1;
  }

  //1文字づつ処理
  for(; buf_index < rlen; buf_index++){
    //改行文字であるならば、
    if(buf[buf_index] == NEXT_LINE_CHAR){
      if(index_second_line == -1){
        //１行目終了。
        firstline[index_first_line] = '\0';//終了文字・いれないと文字化けする。
        index_second_line = 0;
      }else if(index_third_line == -1){
        //２行目終了
        secondline[index_second_line] =  '\0';
        index_third_line = 0;
      }else if(index_fourth_line == -1){
        //3行目終了
        thirdline[index_third_line] =  '\0';
        index_fourth_line = 0;
      }
      continue;
    }
    if(index_second_line < 0){//1行目
      firstline[index_first_line++] = buf[buf_index];
    }else if(index_third_line < 0){//2行目
      secondline[index_second_line++] = buf[buf_index];
    }else if(index_fourth_line < 0){//3行目
      thirdline[index_third_line++] = buf[buf_index];
    }else{//4行目
      fourthline[index_fourth_line++] = buf[buf_index];
    }
  }
  firstline[index_first_line] = '\0';//改行なかった時のため。終了文字・いれないと文字化けする。
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(font_size);
  display.print(firstline);

  //font_size 3では、２行以上の表示不可能。
  if(font_size <= 2){
    //2行目
    if(index_second_line >= 0){
      secondline[index_second_line] ='\0';
      display.setCursor(0,cursor_height);
      display.print(secondline);
    }
    // font size 2 では３行以上、表示不可。
    if(font_size == 1){
      //３行目
      if(index_third_line >= 0){
        thirdline[index_third_line] ='\0';
        display.setCursor(0,cursor_height*2);
        display.print(thirdline);
      }
      //4行目
      if(index_fourth_line >= 0){
        fourthline[index_fourth_line] ='\0';
        display.setCursor(0,cursor_height*3);
        display.print(fourthline);
      }
    }
  }
  display.display();
}

//BLE定期送信
void update_ble(){
  if(millis() - last_ble_notifytime > UPDATE_BLE_TIME){
    ledMode = !ledMode;
    if(!RXisUsed && !D10isUsed){
      Serial.println("not connected");
    }else if(RXisUsed){
      Serial.println("RX is used");
    }else if(D10isUsed){
      Serial.println("D10 is used");
    }
    last_ble_notifytime = millis();
  }
}


void loop() {
  update_ble();

  //NO SIGNAL ...
  if(!RXisUsed && !D10isUsed && display_working){
    display.clearDisplay();
    display.setCursor(0,0);
    display.setTextSize(2);
    display.println("NO SIGNAL");
    //アニメーション
    display.setCursor((millis()/70)%110,16);
    if(ledMode){
      display.println(".");
    }else{
      display.println("..");
    }
    display.display();
    //RXかD10にUARTが来ていないか確認する。
    checkUsage();
  }
  //RX(D9)受信モードであるなら
  
  else if(RXisUsed){
    //受信していれば
    if (MySerial0.available() > 0) {
      int bytes = MySerial0.available();
      int rlen = MySerial0.readBytesUntil(ENDOFDATA, buf, BUFFER_SIZE);
      decode_print_buff(rlen);
      Serial.println("RX received");
    }
  // D10受信モードであるなら
  }
  else if(D10isUsed){
    //データがあれば受信
    if (MySerial1.available() > 0) {
      int bytes = MySerial1.available();
      int rlen = MySerial1.readBytesUntil(ENDOFDATA, buf, BUFFER_SIZE);
      decode_print_buff(rlen);
      Serial.println("D10 received");
    }
  }

  //Debug用 USB-C UARTを受信した場合。
  if (Serial.available() > 0) {
    RXisUsed = true;//内部処理としては、USB-CもD7ピンのRXも同じことにしておく。
    printOLED("USB-C UART RECEIVED");
    last_message_time = millis();
    int rlen = Serial.readBytesUntil(ENDOFDATA, buf, BUFFER_SIZE);
    decode_print_buff(rlen);
    Serial.println("ACK");//受信応答。
  }

  //RX,D10いずれか使用中で、タイムアウト時間経過していれば isUsed を falseにする。
  if(millis()-last_message_time > TIMEOUT_UART){
    D10isUsed = false;
    RXisUsed = false;
  }

  #ifdef BLE
    if (deviceConnected) {
      //定期的にBLEを送信したい場合はここに処理を追加。
    }

    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }

    // connecting
    if (deviceConnected && !oldDeviceConnected) {
		  // do stuff here on connecting
      oldDeviceConnected = deviceConnected;
    }
  #endif
}
