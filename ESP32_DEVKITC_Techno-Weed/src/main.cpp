// ---- Bluetooth control of ESP32 DEVKITC V4
// ---- written by Knife for Techno weed teams mobile robot 21/04/21
//To do:    
        //  make motors controllable with bluetooth :: done to an extent
        //  Add direction control with vectors
        //  Add Ultrasonic sensor interfacing
        //  Add Part loaded switch
        
#include "BluetoothSerial.h"
#include "esp_bt_device.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

//-------------- definitions -------------//
#define FrontLeft_En 35
#define FrontLeft_Dir 34
#define FrontLeft_VCC 23
#define FrontRight_En 33
#define FrontRight_Dir 32
//#define Motor2_VCC 45
#define RearLeft_En 26
#define RearLeft_Dir 25
//#define Motor3_VCC 45
#define RearRight_En 14
#define RearRight_Dir 27
//#define Motor4_VCC 45

#define USonicTrig 23
#define UsonicEcho 22

#define LED 12
//#define duty 100

#define STOP '0'
#define FWD 'W' 
#define RVS 'S' 
#define LFT 'A'
#define RGHT 'D'
#define dutyUp '8'
#define dutyDown '2'

// --- objects ---//
BluetoothSerial SerialBT;

char Control_sig = STOP;
volatile int duty = 100;

void FrontLeft_CW() {
  digitalWrite(FrontLeft_Dir,LOW);
  ledcSetup(0,250,8);
  ledcAttachPin(FrontLeft_En,0);
  ledcWrite(0,duty);
} 

void FrontLeft_CCW() {
  digitalWrite(FrontLeft_Dir,HIGH);
  ledcSetup(0,250,8);
  ledcAttachPin(FrontLeft_En,0);
  ledcWrite(0,duty);
}

void FrontLeft_stop() {
  digitalWrite(FrontLeft_Dir,LOW);
  ledcSetup(0,250,8);
  ledcAttachPin(FrontLeft_En,0);
  ledcWrite(0,duty);
}

void FrontRight_CW() {
  digitalWrite(FrontRight_Dir,LOW);
  ledcSetup(0,250,8);
  ledcAttachPin(FrontRight_En,0);
  ledcWrite(0,duty);
} 

void FrontRight_CCW() {
  digitalWrite(FrontRight_Dir,HIGH);
  ledcSetup(0,250,8);
  ledcAttachPin(FrontRight_En,0);
  ledcWrite(0,duty);
}

void FrontRight_stop() {
  digitalWrite(FrontRight_Dir,LOW);
  ledcSetup(0,250,8);
  ledcAttachPin(FrontRight_En,0);
  ledcWrite(0,duty);
}

void RearLeft_CW() {
  digitalWrite(RearLeft_Dir,LOW);
  ledcSetup(0,250,8);
  ledcAttachPin(RearLeft_En,0);
  ledcWrite(0,duty);
} 

void RearLeft_CCW() {
  digitalWrite(RearLeft_Dir,HIGH);
  ledcSetup(0,250,8);
  ledcAttachPin(RearLeft_En,0);
  ledcWrite(0,duty);
}

void RearLeft_stop() {
  digitalWrite(RearLeft_Dir,LOW);
  ledcSetup(0,250,8);
  ledcAttachPin(RearLeft_En,0);
  ledcWrite(0,duty);
}

void RearRight_CW() {
  digitalWrite(RearRight_Dir,LOW);
  ledcSetup(0,250,8);
  ledcAttachPin(RearRight_En,0);
  ledcWrite(0,duty);
} 

void RearRight_CCW() {
  digitalWrite(RearRight_Dir,HIGH);
  ledcSetup(0,250,8);
  ledcAttachPin(RearRight_En,0);
  ledcWrite(0,duty);
}

void RearRight_stop() {
  digitalWrite(RearRight_Dir,LOW);
  ledcSetup(0,250,8);
  ledcAttachPin(RearRight_En,0);
  ledcWrite(0,duty);
}

//----------- Driving ------------//
void Left() {
  FrontLeft_CCW();
  FrontRight_CCW();
  RearLeft_CW();
  RearRight_CW();
}

void Right() {
  FrontLeft_CCW();
  FrontRight_CCW();
  RearLeft_CW();
  RearRight_CW();  
}

void Forward() {
  FrontLeft_CCW();
  FrontRight_CW();
  RearLeft_CCW();
  RearRight_CW();
}

void Reverse() {
  FrontLeft_CW();
  FrontRight_CCW();
  RearLeft_CW();
  RearRight_CCW();
}
void Stop() {
  FrontLeft_stop();
  FrontRight_stop();
  RearLeft_stop();
  RearRight_stop();
  }
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

void setup()
{
  //- -------- Setup Serial comms -------//
 
  Serial.begin(9600);
  SerialBT.begin("TechnoWeed ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  Serial.println("Device name: TechnoWeed ESP32test");
  Serial.print("MAC: ");
  printMAC();
  Serial.println();

  pinMode(LED, OUTPUT); // LED identifier

  pinMode(FrontLeft_En, OUTPUT); // MOtor 1 PWM enable 
  pinMode(FrontLeft_Dir, OUTPUT); // MOtor 1 direction control
  pinMode(FrontLeft_VCC,OUTPUT);  // Mode select

  pinMode(FrontRight_En, OUTPUT); // MOtor 1 PWM enable 
  pinMode(FrontRight_Dir, OUTPUT);

  pinMode(RearLeft_En, OUTPUT); // MOtor 1 PWM enable 
  pinMode(RearLeft_Dir, OUTPUT);
  
  pinMode(RearRight_En, OUTPUT); // MOtor 1 PWM enable 
  pinMode(RearRight_Dir, OUTPUT);

  digitalWrite(LED, LOW);
  digitalWrite(FrontLeft_VCC, HIGH);
}
 
void loop()
{
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  
  if (SerialBT.available()) {
   char Control_sig = SerialBT.read();
    Serial.write(Control_sig);
    switch (Control_sig) {
      case STOP:
        digitalWrite(LED, LOW);
        Serial.println("Stop");
        Stop();
        break;
        
      case FWD:
        Forward();
        Serial.println("FWD");
        digitalWrite(LED, HIGH);
        break;
        
      case RVS:
        Reverse();
        Serial.println("RVS");
        digitalWrite(LED, LOW);
        break;
        
      case LFT:
        Left();
        Serial.println("LFT");
        digitalWrite(LED, HIGH);
        break;

      case RGHT:
        Right();
        Serial.println("RIGHT");
        digitalWrite(LED, LOW);
        break;

      case dutyUp:
        duty = duty + 10;
        if (duty >= 255){ // cap the duty value
          duty = 255;
        }
        Serial.print(" Duty up: ");
        Serial.println(duty);
        digitalWrite(LED, HIGH);
        break;

      case dutyDown:
        duty = duty - 10;
        if (duty <= 50){ // motors dont seem to handle low duty 
          duty = 50;
        }
        Serial.print(" Duty down: ");
        Serial.println(duty);
        digitalWrite(LED, LOW);
        break;
        
      default:
        break;
    }
  }
}