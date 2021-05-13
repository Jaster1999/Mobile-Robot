#include <Arduino.h>
#include "BluetoothSerial.h"
#include "esp_bt_device.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

//-------------- definitions -------------//
BluetoothSerial SerialBT;

void printMAC(){
  const uint8_t* point = esp_bt_dev_get_address();
    for (int i = 0; i < 6; i++) {
      char str[3];
  
      sprintf(str, "%02X", (int)point[i]);
      Serial.print(str);
  
      if (i < 5){
        Serial.print(":");
      }
 
  }
}

void setup() {
  Serial.begin(9600);
  SerialBT.begin("TechnoWeed_ESP32PC");
  Serial.println("The device started, now you can pair it with bluetooth!");
  Serial.println("Device name: TechnoWeed_ESP32PC");
  Serial.print("MAC: ");
  printMAC();
  Serial.println();
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