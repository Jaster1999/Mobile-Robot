// Techno-weed ESP32 PC bluetooth serial interface
// 
#include <Arduino.h>
#include "BluetoothSerial.h"
#include "esp_bt_device.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

String MACadd = "AA:BB:CC:11:22:33";
uint8_t address[6]  = {0x94, 0xB9, 0x7E, 0x7B, 0xA7, 0xE2};
String name = "TechnoWeed_ESP32test";
const char *pin = "1234"; //<- standard pin would be provided by default
bool connected;

//-------------- definitions -------------//
BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  //SerialBT.setPin(pin);
  SerialBT.begin("TW_ESP32PC", true); 
  //SerialBT.setPin(pin);
  Serial.println("The device started in master mode, make sure remote BT device is on!");
  //connected = SerialBT.connect(name);
  connected = SerialBT.connect(address);
  
  if(connected) {
    Serial.println("Connected Succesfully!");
  } 
  else {
    while(!SerialBT.connected(10000)) {
      Serial.println("Failed to connect. Make sure remote device is available and in range, then restart app."); 
      connected = SerialBT.connect(address);
    }
  }
  // if (SerialBT.disconnect()) {
  //   Serial.println("Disconnected Succesfully!");
  // }
  SerialBT.connect();
}

void loop() {


  
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }

  if (SerialBT.available()){
    Serial.write(SerialBT.read());
  }
  delay(20);
}