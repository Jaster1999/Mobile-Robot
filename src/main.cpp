// Team Robot Manipulator Project
// SCARA arm design
// 1 x Nema 23 drives the Z axis
// 3 x Servos to drive Joint 2, 3 and gripper

// To do:  - Confirm pin selection with Stan
//         - Add interrupt capability

#include <Arduino.h>
#include <Servo.h> 
#include <AccelStepper.h>

#define LED         13 
#define joint2_pin  2
#define joint3_pin  3
#define Gripper_pin 4
#define Pressure    A0
#define reference   A1

#define Z_dir       10    // -- these need to be confirmed with Stan
#define Z_stp       11    // -- 
#define Z_en        12    // -- 
#define Z_homeSw    9     // -- 

#define BUF_LEN     20

#define STOP        '0'
#define HOME        'h' 

//------------------ Object Definitions --------------------- // 
// create servo object to control a servo joints
Servo joint2;  
Servo joint3;
Servo Gripper;
// creat stepper object to control Z axis
AccelStepper stepperZ(AccelStepper::FULL2WIRE, Z_dir, Z_stp);

//------------------ Function prototypes --------------------- // 
void handleSerial (void);
void homeStepper(void);

// ----------------- Globals -----------------------------=---//
long int homeSpeed  = 550; 
long int maxSpeed   = 1000; 
long int homeAccel  = 300; 
long int MaxAccel   = 500; 
long int init_homing = -1;
bool Homed = false;     // -- make a homed variable to ensure unit is homed before opperation
String sdata = "";
double Time = 0;
double previousTime = 0;

int i = 0;
int k = 0;

bool SafeToRun = false;

void setup() { 
  // --------------- Pin setup ------------------ //
  pinMode(LED, OUTPUT);
  pinMode(joint2_pin, OUTPUT);
  pinMode(joint3_pin, OUTPUT);
  pinMode(Gripper_pin, OUTPUT);

  Serial.begin(115200);

  joint2.attach(joint2_pin, 790, 2150);  // attaches the servo on pin 9
  joint3.attach(joint3_pin, 790, 2150);  // attaches the servo on pin 6
  Gripper.attach(Gripper_pin, 790, 2150);  // attaches the servo on pin 5



  // ----------------- set servos to go to initial angle
  joint2.write(0);  
  joint3.write(0);
  Gripper.write(0);
  Time = millis();
  unsigned long currentTime = millis();
  while(Homed == false){
    currentTime = millis();
    if ((currentTime - Time) >= 1000){
      Serial.println("home me ya bastards");
      Time = currentTime;
    }
    handleSerial();
  }
} 
 
 
void loop() { 
  // unsigned long currentTime = millis();
  digitalWrite(LED, LOW);
  handleSerial();
  }

void handleSerial(){
  static char sdata[BUF_LEN], *pSdata=sdata;
  byte ch;
  int Angle;
  if (Serial.available()){
    digitalWrite(LED, HIGH);
    // delay(100);

    ch = Serial.read();
    if (Homed == false){
      if (ch == HOME){
        homeStepper();
        Homed = true;
      }
    }
    if ((pSdata - sdata) >= BUF_LEN-1) {
      pSdata--; 
      Serial.print("BUFFER OVERRUN\n"); 
    }
    *pSdata++ = (char)ch;
    if (ch=='\n'){
      pSdata--;
      *pSdata = '\0';
    
      switch (sdata[0]) {
        case '1':
          // servo 1
          if (strlen(sdata)>1){
            Serial.println("Servo 1 bruddah");
            Angle = atoi(&sdata[1]);
            joint2.write(Angle);
            }
          break;

        case '2':
          // servo 2
          if (strlen(sdata)>1){
            Serial.println("Servo 2 bruddah");
            Angle = atoi(&sdata[1]);
            joint3.write(Angle);
            }
          break;

        case '3':
          // servo 3
          if (strlen(sdata)>1){
            Serial.println("Servo 3 bruddah");
            Angle = atoi(&sdata[1]);
            Gripper.write(Angle);
            }
          break;

        case HOME:
          // Home Z axis
          homeStepper();
          Serial.println("Stepper homed bruddah");
          break;

        case STOP:
          // Home Z axis
          stepperZ.stop();
          Serial.println("Stepper stopped bruddah");
          break;
        
        
        default: Serial.println(sdata);
      }

      pSdata = sdata;
    }
  }
}

void homeStepper(void){
  // --------------------- setup up homing speeds ---------- //
  stepperZ.setMaxSpeed(homeSpeed);
  stepperZ.setAcceleration(homeAccel);
  stepperZ.moveTo(-30000);

  Serial.print("Stepper Z is Homing . . . . . . . . . . . ");

  while ((digitalRead(Z_homeSw) == LOW) && (SafeToRun == true)) {  // Make the Stepper move CCW until the switch is activated
    if (Serial.available() > 0){
      char c = Serial.read();
      if (c == STOP){
        Serial.println("Stop");
        stepperZ.stop();
        SafeToRun = false;
        return;
      }  
    }    
    stepperZ.moveTo(init_homing);  // Set the position to move to
    init_homing--;  // Decrease by 1 for next move if needed
    stepperZ.run();  // Start moving the stepper
    delay(5);
  }
  init_homing = -1;
  stepperZ.setCurrentPosition(0); 
  Serial.println("Stepper Z is Homed \n");

}