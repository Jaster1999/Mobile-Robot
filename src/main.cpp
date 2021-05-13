// ---- Bluetooth control of ESP32 S2
// ---- written by Knife for Techno weed teams mobile robot 20/04/21
//To do:    
//          Add bluetooth initialisation   Done 
        //  make motors controllable with bluetooth Done
        //  Add direction control with vectors
        //  Add Ultrasonic sensor interfacing
        //  Add Part loaded switch
        
#include "BluetoothSerial.h"
#include "esp_bt_device.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

//-------------- definitions -------------//

#define FrontLeft_En 13
#define FrontLeft_Dir 12 
#define Motor_EN 14

#define FrontRight_En 2
#define FrontRight_Dir 0

#define RearLeft_En 26
#define RearLeft_Dir 25

#define RearRight_En 22
#define RearRight_Dir 23

#define Trig_US 27
#define Left_US 39
#define Back_US 36
#define Front_US 15
#define Right_US 21

#define Load_SW1 3
#define Load_SW2 1

#define Encoder_1_A 18
#define Encoder_1_B 19
#define Encoder_2_A 26
#define Encoder_2_B 5
#define Encoder_3_A 32
#define Encoder_3_B 33
#define Encoder_4_A 35
#define Encoder_4_B 34

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
int stopduty = 0;
int freq = 5000;

void FrontLeft_CW() {
  digitalWrite(FrontLeft_Dir,LOW);
  ledcSetup(0,freq,8);
  ledcAttachPin(FrontLeft_En, 0);
  ledcWrite(0,duty);
} 

void FrontLeft_CCW() {
  digitalWrite(FrontLeft_Dir,HIGH);
  ledcSetup(0,freq,8);
  ledcAttachPin(FrontLeft_En, 0);
  ledcWrite(0,duty);
}

void FrontLeft_stop() {
  digitalWrite(FrontLeft_Dir,LOW);
  ledcSetup(0,freq,8);
  ledcAttachPin(FrontLeft_En,0);
  ledcWrite(0,stopduty);
}

void FrontRight_CW() {
  digitalWrite(FrontRight_Dir,LOW);
  ledcSetup(0,freq,8);
  ledcAttachPin(FrontRight_En,0);
  ledcWrite(0,duty);
} 

void FrontRight_CCW() {
  digitalWrite(FrontRight_Dir,HIGH);
  ledcSetup(0,freq,8);
  ledcAttachPin(FrontRight_En,0);
  ledcWrite(0,duty);
}

void FrontRight_stop() {
  digitalWrite(FrontRight_Dir,LOW);
  ledcSetup(0,freq,8);
  ledcAttachPin(FrontRight_En,0);
  ledcWrite(0,stopduty);
}

void RearLeft_CW() {
  digitalWrite(RearLeft_Dir,LOW);
  ledcSetup(0,freq,8);
  ledcAttachPin(RearLeft_En,0);
  ledcWrite(0,duty);
} 

void RearLeft_CCW() {
  digitalWrite(RearLeft_Dir,HIGH);
  ledcSetup(0,freq,8);
  ledcAttachPin(RearLeft_En,0);
  ledcWrite(0,duty);
}

void RearLeft_stop() {
  digitalWrite(RearLeft_Dir,LOW);
  // ledcSetup(0,250,8);
  //ledcAttachPin(RearLeft_En,0);
  ledcWrite(0,stopduty);
}

void RearRight_CW() {
  digitalWrite(RearRight_Dir,LOW);
  ledcSetup(0,freq,8);
  ledcAttachPin(RearRight_En,0);
  ledcWrite(0,duty);
} 

void RearRight_CCW() {
  digitalWrite(RearRight_Dir,HIGH);
  ledcSetup(0,freq,8);
  ledcAttachPin(RearRight_En,0);
  ledcWrite(0,duty);
}

void RearRight_stop() {
  digitalWrite(RearRight_Dir,LOW);
  ledcSetup(0,freq,8);
  ledcAttachPin(RearRight_En,0);
  ledcWrite(0,stopduty);
}
void Motor_Controller() {
  ledcSetup(0,freq,8);
  ledcWrite(0,duty);
}

//----------- Driving ------------//
void Left() {
  //digitalWrite(Motor_EN, HIGH);
  FrontLeft_CW();
  FrontRight_CW();
  RearLeft_CCW();
  RearRight_CCW();
}

void Right() {
  //digitalWrite(Motor_EN, HIGH);
  FrontLeft_CCW();
  FrontRight_CCW();
  RearLeft_CW();
  RearRight_CW();  
}

void Forward() {
  //digitalWrite(Motor_EN, HIGH);
  FrontLeft_CCW();
  FrontRight_CW();
  RearLeft_CCW();
  RearRight_CW();
}

void Reverse() {
  //digitalWrite(Motor_EN, HIGH);
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
  //digitalWrite(Motor_EN, LOW);
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

  // pinMode(LED, OUTPUT); // LED identifier

  pinMode(FrontLeft_En, OUTPUT); // MOtor 1 PWM enable 
  pinMode(FrontLeft_Dir, OUTPUT); // MOtor 1 direction control
  pinMode(Motor_EN,OUTPUT);  // Mode select

  pinMode(FrontRight_En, OUTPUT); // MOtor 1 PWM enable 
  pinMode(FrontRight_Dir, OUTPUT);

  pinMode(RearLeft_En, OUTPUT); // MOtor 1 PWM enable 
  pinMode(RearLeft_Dir, OUTPUT);
  
  pinMode(RearRight_En, OUTPUT); // MOtor 1 PWM enable 
  pinMode(RearRight_Dir, OUTPUT);

  digitalWrite(Motor_EN, HIGH);
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
        Serial.println("Stop");
        Stop();
        break;
        
      case FWD:
        Forward();
        Serial.println("FWD");
        break;
        
      case RVS:
        Reverse();
        Serial.println("RVS");
        break;
        
      case LFT:
        Left();
        Serial.println("LFT");
        break;

      case RGHT:
        Right();
        Serial.println("RIGHT");
        break;

      case dutyUp:
        duty = duty + 10;
        if (duty >= 256){ // cap the duty value
          duty = 256;
        }
        Motor_Controller();
        Serial.print(" Duty up: ");
        Serial.println(duty);
        break;

      case dutyDown:
        duty = duty - 10;
        if (duty <= 50){ // motors dont seem to handle low duty 
          duty = 50;
        }
        Motor_Controller();
        Serial.print(" Duty down: ");
        Serial.println(duty);
        break;
        
      default:
        break;
    }
  }
}