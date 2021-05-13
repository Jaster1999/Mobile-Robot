// Techno-weed ESP32 PC bluetooth serial interface
// 
#include <Arduino.h>
#include "BluetoothSerial.h"
#include "esp_bt_device.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

String MACadd = "AA:BB:CC:11:22:33";
uint8_t address[6]  = {0xAA, 0xBB, 0xCC, 0x11, 0x22, 0x33};
//uint8_t address[6]  = {0x00, 0x1D, 0xA5, 0x02, 0xC3, 0x22};
String name = "TechnoWeed_ESP32";
const char *pin = "1234"; //<- standard pin would be provided by default
bool connected;

//-------------- definitions -------------//
BluetoothSerial SerialBT;

// void printMAC(){
//   const uint8_t* point = esp_bt_dev_get_address();
//     for (int i = 0; i < 6; i++) {
//       char str[3];
  
//       sprintf(str, "%02X", (int)point[i]);
//       Serial.print(str);
  
//       if (i < 5){
//         Serial.print(":");
//       }
 
//   }
// }

void setup() {
  // Serial.begin(9600);
  // SerialBT.begin("TechnoWeed_ESP32PC");
  // Serial.println("The device started, now you can pair it with bluetooth!");
  // Serial.println("Device name: TechnoWeed_ESP32PC");
  // Serial.print("MAC: ");
  // printMAC();
  // Serial.println();
  Serial.begin(115200);
  //SerialBT.setPin(pin);
  SerialBT.begin("TechnoWeed_ESP32PC", true); 
  //SerialBT.setPin(pin);
  Serial.println("The device started in master mode, make sure remote BT device is on!");
  
  // connect(address) is fast (upto 10 secs max), connect(name) is slow (upto 30 secs max) as it needs
  // to resolve name to address first, but it allows to connect to different devices with the same name.
  // Set CoreDebugLevel to Info to view devices bluetooth address and device names
  connected = SerialBT.connect(name);
  //connected = SerialBT.connect(address);
  
  if(connected) {
    Serial.println("Connected Succesfully!");
  } else {
    while(!SerialBT.connected(10000)) {
      Serial.println("Failed to connect. Make sure remote device is available and in range, then restart app."); 
    }
  }
  // disconnect() may take upto 10 secs max
  if (SerialBT.disconnect()) {
    Serial.println("Disconnected Succesfully!");
  }
  // this would reconnect to the name(will use address, if resolved) or address used with connect(name/address).
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