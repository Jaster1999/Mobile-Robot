// ---- Bluetooth control of ESP32 S2
// ---- written by Knife for Techno weed teams mobile robot 20/04/21
//To do:    Add bluetooth initialisation
        //  make motors controllable with bluetooth
        //  Add direction control with vectors
        //  Add Ultrasonic sensor interfacing
        //  Add Part loaded switch
        
#include <Adafruit_NeoPixel.h>
#define PIN 18
#define NUMPIXELS 1
#define FrontLeft_En 1
#define FrontLeft_Dir 0
#define FrontLeft_VCC 45

#define FrontRight_En 3
#define FrontRight_Dir 2
//#define Motor2_VCC 45

#define RearLeft_En 5
#define RearLeft_Dir 4
//#define Motor3_VCC 45

#define RearRight_En 7
#define RearRight_Dir 6
//#define Motor4_VCC 45

#define LED 38
#define duty 100

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
 
enum {NONE, RED, GREEN, BLUE};
int ledColor = NONE;


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

void setup()
{
  //Serial.begin(9600);
  pixels.begin();
  pinMode(FrontLeft_En, OUTPUT); // MOtor 1 PWM enable 
  pinMode(FrontLeft_Dir, OUTPUT); // MOtor 1 direction control
  pinMode(FrontLeft_VCC,OUTPUT);  // Mode select
  pinMode(LED, OUTPUT); // LED identifier
  digitalWrite(FrontLeft_VCC, HIGH);

  pinMode(FrontRight_En, OUTPUT); // MOtor 1 PWM enable 
  pinMode(FrontRight_Dir, OUTPUT);

  pinMode(RearLeft_En, OUTPUT); // MOtor 1 PWM enable 
  pinMode(RearLeft_Dir, OUTPUT);
  
  pinMode(RearRight_En, OUTPUT); // MOtor 1 PWM enable 
  pinMode(RearRight_Dir, OUTPUT);
}
 
void loop()
{
  switch (ledColor) {
    case NONE:
      pixels.setPixelColor(0, pixels.Color(0, 0, 0));
      pixels.show();
      Right();
      break;
      
    case RED:
      pixels.setPixelColor(0, pixels.Color(20, 0, 0));
      pixels.show();
      Forward();
      //Serial.print("RED");
      break;
      
    case GREEN:
      pixels.setPixelColor(0, pixels.Color(0, 20, 0));
      pixels.show();
      //Serial.print("Green");
      Reverse();
      break;
      
    case BLUE:
      pixels.setPixelColor(0, pixels.Color(0, 0, 20));
      pixels.show();
      //Serial.println("BLUE");
      Left();
      break;
    default:
      break;
  }
 
  ledColor++;
  if (ledColor == 4) {
    ledColor = NONE;
  }
 
  delay(1500);
}
