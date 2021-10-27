// Team Robot Manipulator Project
// SCARA arm design
// 1 x Nema 23 drives the Z axis
// 3 x Servos to drive Joint 2, 3 and gripper

// To do:  - Confirm pin selection with Stan

#include <Arduino.h>
#include <Servo.h> 
#include <AccelStepper.h>
#include <EEPROM.h>

// #define LED         13 // No need to do this, arduino already have LEDBUILTIN defined
#define joint2_pin  2
#define joint3_pin  3
#define joint4_pin  4
#define Gripper_pin 5

#define Z_en        6    
#define Z_homeSw    A0
#define Z_dir       A4    
#define Z_stp       A5 
#define StepsPermm  625   

// #define BUF_LEN     20

#define STOP        '#'
#define HOME        'h' 

//UART CONSTs
#define USART_BAUDRATE 115200
#define MYUBRR (((F_CPU / (USART_BAUDRATE * 16UL))))
#define TXBUFFSIZE 64
#define RXbufSize 64 //Rx buffer size in bytes

//-------EEPROM Constants---------
#define Joint2LocationAddr 0
#define Joint3LocationAddr 1
#define Joint4LocationAddr 2

//---------Joint Angle/distance Limits--------------
#define StepperMax  250 //mm
#define StepperMin  0
#define Joint2Max   180
#define Joint2Min   0
#define Joint3Max   180
#define Joint3Min   0
#define Joint4Max   180
#define Joint4Min   0

//------------------ Object Definitions --------------------- // 
// create servo object to control a servo joints
Servo joint2;  
Servo joint3;
Servo joint4;
Servo Gripper;
// creat stepper object to control Z axis
AccelStepper stepperZ(AccelStepper::FULL2WIRE, Z_dir, Z_stp);

//------------------ Function prototypes --------------------- // 
void handleSerial (void);
void homeStepper(void); // Home stepper and set Servos to the default value from EEPROM
void moveStepper(long int steps);
void SerialWrite(const char* array);
bool Validate(long int angle, int jointNum);

// ----------------- Globals -----------------------------=---//
#define homeSpeed     1550
#define maxSpeed      2500
#define homeAccel     400
#define MaxAccel      800 
long int init_homing = -1;
bool Homed = false;     // -- make a homed variable to ensure unit is homed before opperation
char TXBUFF[TXBUFFSIZE];  //TX buffer Char array of size TXBUFFSIZE
int TXINDEX = 0;  // Index for TX buffer.  To know what character to send next
char RXbuff[RXbufSize];  //RX buffer Char array of size RXBUFFSIZE.  Received characters will be stored here
int rxbuffIndex = 0;  // RX buffer index.  To know where the most recent character was stored in the array
double Time = 0;
double currentTime = 0;
double previousTime = 0;
volatile long int steps; //Made steps a uint32 to allow it to go over 65535 steps, just in case of microstepping or something
//------Variables used for speed control of servos.  Volatile as they are to be used in an ISR
volatile int Joint2Setpoint = 0;
volatile int Joint3Setpoint = 0;
volatile int Joint4Setpoint = 0;
volatile int Joint2CurrentPos = 90;
volatile int Joint3CurrentPos = 90;
volatile int Joint4CurrentPos = 90;
volatile byte count = 0;
#define speed 4 // increase this to increase the delay between servo steps
bool ServosRun = false;

int i = 0;
int k = 0;

bool SafeToRun = false;
// int LEDstate = 0;
void setup() { 
  // --------------- Pin setup ------------------ //
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(joint2_pin, OUTPUT);
  pinMode(joint3_pin, OUTPUT);
  pinMode(joint4_pin, OUTPUT);
  pinMode(Gripper_pin, OUTPUT);
  pinMode(Z_en, OUTPUT);
  pinMode(Z_homeSw, INPUT);

  cli();  //Disable interrupts while we setup stuff
  //-----------Serial Interrupt setup-------------//
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);   // Turn on the transmission and reception circuitry
  UCSR0C = (1 << UCSZ00) | (1 << UCSZ01); // Use 8-bit character sizes

  UBRR0H = (MYUBRR >> 8); // Load upper 8-bits of the baud rate value into the high byte of the UBRR register
  UBRR0L = MYUBRR; // Load lower 8-bits of the baud rate value into the low byte of the UBRR register

  UCSR0B |= (1 << RXCIE0); // Enable the USART Recieve Complete interrupt (USART_RXC)
  UCSR0B |= (1 << TXCIE0); // Enable the USART Transmit Complete interrupt (USART_TXC)

  
  
  //-------Timer 2 setup for interrupt---------//
  // interrupt time = 1/(16Mhz/1024) * 127 =  8.128ms;
  TCCR2A = 0;                 // Reset entire TCCR1A to 0 
  TCCR2B = 0;                 // Reset entire TCCR1B to 0
  TCCR2B |= (1<<CS20|1<<CS21|1<<CS22);  // set CS20, CS21 and CS22 to 1 to get a prescalar of 1024
  TIMSK2 |= (1<<OCIE2A);  // Set OCIE2A to 1 to enable compare match A
  TCCR2A |= (1<<WGM21); // Set WGM21 to 1 to enable CTC mode (Clear Timer on Compare match)
  TCNT2 = 0; //Clear the timer 2 counter
  OCR2A = 255;  // set compare register A to this value 
  //decreasing the above value will decrease the time to interrupt
  //This will increase the speed of the servos
  
  //--------Servo setup-------------//
  joint2.attach(joint2_pin, 790, 2150);  // attaches the servo on pin 2
  joint3.attach(joint3_pin, 790, 2150);  // attaches the servo on pin 3
  joint4.attach(joint4_pin, 790, 2150);  // attaches the servo on pin 4
  Gripper.attach(Gripper_pin, 790, 2150);  // attaches the servo on pin 5

  sei(); // Enable the Global Interrupt Enable flag so that interrupts can be processed 

  int joint2Mem = EEPROM.read(Joint2LocationAddr);
  int joint3Mem = EEPROM.read(Joint3LocationAddr);
  int joint4Mem = EEPROM.read(Joint4LocationAddr);
  char msg[10];
  SerialWrite("EEPROM Values read: ");
  SerialWrite(itoa(joint2Mem, msg, 10));
  SerialWrite(", ");
  SerialWrite(itoa(joint3Mem, msg, 10));
  SerialWrite(", ");
  SerialWrite(itoa(joint4Mem, msg, 10));
  SerialWrite("\n");
  if(Validate(joint2Mem, 2)){Joint2Setpoint = joint2Mem;}
  else{Joint2Setpoint = 0;}
  if(Validate(joint3Mem, 3)){Joint3Setpoint = joint3Mem;}
  else{Joint3Setpoint = 0;}
  if(Validate(joint4Mem, 4)){Joint4Setpoint = joint4Mem;}
  else{Joint4Setpoint = 0;}

  Time = millis();
  currentTime = millis();
  while(Homed == false){
    currentTime = millis();
    if ((currentTime - Time) >= 1000){
      SerialWrite("home me please good sirs\n");
      // Serial.println("home me please good sirs");
      Time = currentTime;
    }
    handleSerial();
  }
} 

//intrerupt on receive
ISR (USART_RX_vect) //this is the Received Byte ISR
{
   char ReceivedByte;
   ReceivedByte = UDR0; // Fetch the received byte value into the variable "ByteReceived"
   if(ReceivedByte == '#')
   {
    // stop Z axis
    stepperZ.stop();
    SafeToRun = false;
    ServosRun = false;
    SerialWrite("Stepper stopped\n");
    // digitalWrite(LED_BUILTIN, HIGH);
   }
   else{
     if(rxbuffIndex < RXbufSize){
     RXbuff[rxbuffIndex] = ReceivedByte; //store byte in global array to allow code to read it else where
     }
     rxbuffIndex++;
   }
   
}

ISR (USART_TX_vect) //This is the TX complete ISR
{
  // digitalWrite(LED_BUILTIN, LOW);
  if(TXBUFF[TXINDEX]) //if it is not the end of the string
  {
    UCSR0B |= (1 << TXB80); //clear the flag
    UDR0 = TXBUFF[TXINDEX]; //load the next byte in
    TXINDEX++;  //increment the counter
  }
}

// Timer 2 ISR
ISR(TIMER2_COMPA_vect){                            
  // PORTB ^= 0x20; //toggle the LED
  if(count % speed == 0)
  {
    count = 0;
    if(ServosRun){
      if(Joint2CurrentPos>Joint2Setpoint)
      {
        Joint2CurrentPos--;
        joint2.write(Joint2CurrentPos);
        EEPROM.write(Joint2LocationAddr, Joint2CurrentPos);
      }
      else if (Joint2CurrentPos<Joint2Setpoint)
      {
        Joint2CurrentPos++;
        joint2.write(Joint2CurrentPos);
        EEPROM.write(Joint2LocationAddr, Joint2CurrentPos);
      }
      if(Joint3CurrentPos>Joint3Setpoint)
      {
        Joint3CurrentPos--;
        joint3.write(Joint3CurrentPos);
        EEPROM.write(Joint3LocationAddr, Joint3CurrentPos);
      }
      else if (Joint3CurrentPos<Joint3Setpoint)
      {
        Joint3CurrentPos++;
        joint3.write(Joint3CurrentPos);
        EEPROM.write(Joint3LocationAddr, Joint3CurrentPos);
      }
      if(Joint4CurrentPos>Joint4Setpoint)
      {
        Joint4CurrentPos--;
        joint4.write(Joint4CurrentPos);
        EEPROM.write(Joint4LocationAddr, Joint4CurrentPos);
      }
      else if (Joint4CurrentPos<Joint4Setpoint)
      {
        Joint4CurrentPos++;
        joint4.write(Joint4CurrentPos);
        EEPROM.write(Joint4LocationAddr, Joint4CurrentPos);
      }
    }
  }
  count++;
}
 
 
void loop() { 
  handleSerial();
  currentTime = millis();
  if ((currentTime - Time) >= 1000){
    // LEDstate ^= 1;
    // digitalWrite(LED_BUILTIN, LEDstate); // -- flash led just cos
    // The code below is a cleaner way of toggling the LED
    PORTB ^= 0x20; //toggle the LED 
    Time = currentTime;
  }
  }

void handleSerial(){
  int Angle;
  long int Dist_mm;
  // long int steps; //Made steps a uint32 to allow it to go over 65535 steps, just in case of microstepping or something
  if (rxbuffIndex>=RXbufSize) {
      rxbuffIndex = 0;
      SerialWrite("BUFFER OVERRUN\n");
      // Serial.print("BUFFER OVERRUN\n"); 
    }
  
  if (rxbuffIndex > 0 && (RXbuff[rxbuffIndex-1]=='\n' || RXbuff[rxbuffIndex-1] == '\r')){  //if there is something in the buffer an the last character is a newline or carrige return
    // digitalWrite(LED_BUILTIN, HIGH);
    // delay(100);
    ServosRun = true;
    SafeToRun = true;
    if (Homed == false){
      if (RXbuff[0] == HOME){
        rxbuffIndex = 0;
        homeStepper();
        Homed = true;
        SerialWrite("... Ready ...\n");
        // Serial.println("... Ready ...");
      }
    }
    
    else if ((Homed == true)){
      RXbuff[rxbuffIndex-1]='\0'; //add a null character to the end of the char array
      switch (RXbuff[0]) {
        case '2':
          // servo 1 on joint 2
          if (rxbuffIndex>1){

            Angle = atoi(&RXbuff[1]);
            if(Validate(Angle, 2))
            {
              Joint2Setpoint = Angle;
              // joint2.write(Angle);
              SerialWrite("Joint 2 moved to:");
              char intstr [10];
              itoa(Angle, intstr, 10);
              SerialWrite(intstr);
              SerialWrite("\n");
            }
            else{SerialWrite("Invalid Angle Received\n");}
            // Serial.print("Joint 2 moved to:"); 
            // Serial.println(Angle);
            }
          break;

        case '3':
          // servo 2 on joint 3
          if (rxbuffIndex>1){
            Angle = atoi(&RXbuff[1]);
            if(Validate(Angle, 3))
            {
              Joint3Setpoint = Angle;
              // joint3.write(Angle);
              SerialWrite("Joint 3 moved to:");
              char intstr [10];
              itoa(Angle, intstr, 10);
              SerialWrite(intstr);
              SerialWrite("\n");
            }
            else{SerialWrite("Invalid Angle Received\n");}
            // Serial.print("Joint 3 moved to:"); Serial.println(Angle);
            }
          break;

          case '4':
          // servo 3 on joint 4
          if (rxbuffIndex>1){
            Angle = atoi(&RXbuff[1]);
            if(Validate(Angle, 4))
            {
              Joint4Setpoint = Angle;
              // joint4.write(Angle);
              SerialWrite("Joint 4 moved to:");
              char intstr [10];
              itoa(Angle, intstr, 10);
              SerialWrite(intstr);
              SerialWrite("\n");
            }
            else{SerialWrite("Invalid Angle Received\n");}
            // Serial.print("Joint 4 moved to:"); Serial.println(Angle);
            }
          break;

        case 'G':
          // servo Gripper
          if (rxbuffIndex>1){ 
            Angle = atoi(&RXbuff[1]);
            Gripper.write(Angle);
            SerialWrite("Gripper moved to:");
            char intstr [10];
            itoa(Angle, intstr, 10);
            SerialWrite(intstr);
            SerialWrite("\n");
            // Serial.print("Gripper moved to:"); Serial.println(Angle);
            }
          break;

        case 'Z':
          // Z stepper call
          if (rxbuffIndex>1){

            Dist_mm = atol(&RXbuff[1]);
            //------------------------------------------------------------------------
            steps = Dist_mm*StepsPermm;
            SerialWrite("Debugging the steps:");
            char intstr45[10];
            ltoa(steps, intstr45, 10);
            SerialWrite(intstr45);
            SerialWrite("\n");
            //------------------------------------------------------------------------
            //Note at this point steps is actually the distance in mm
            if(Validate(Dist_mm, 1))
            {
              SerialWrite("Z-Axis moved to:");
              char intstr [10];
              ltoa(Dist_mm, intstr, 10);
              SerialWrite(intstr);
              SerialWrite("\n");
              // steps = Dist_mm*200; //StepsPermm; //Now steps is actually steps
              
              

              moveStepper(steps);
            }
            else{SerialWrite("Invalid Distance Received\n");}
            // Serial.print("Z axis moved to:"); Serial.println(steps);
            
            }
          break;

        case HOME:
          // Home Z axis
          homeStepper();
          SerialWrite("Stepper homed\n");
          // Serial.println("Stepper homed");
          break;
        
        
        default: SerialWrite(RXbuff); 
                // Serial.println(sdata);
      }
      rxbuffIndex = 0;
      SerialWrite("... Ready ...\n");
      // Serial.println("... Ready ...");
    }
  }
  SafeToRun = false;
}
// Home stepper and set Servos to the default value from EEPROM
void homeStepper(void){ 
  // ----------------- set servos to go to initial angle from angles
  Joint2Setpoint = 90;
  Joint3Setpoint = 90;
  Joint4Setpoint = 90;
  Gripper.write(0);

  // --------------------- setup up homing speeds ---------- //
  stepperZ.setMaxSpeed(homeSpeed);
  stepperZ.setAcceleration(homeAccel);
  stepperZ.moveTo(-30000);

  // Serial.print("Stepper Z is Homing . . . . . . . . . . . ");
  SerialWrite("Stepper Z is Homing . . . . . . . . . . . \n");
  // limit switch is pulled down so goes high when pressed
  while ((digitalRead(Z_homeSw) == LOW) && (SafeToRun == true)) {  // Make the Stepper move CCW until the switch is activated   
    stepperZ.moveTo(init_homing);  // Set the position to move to
    init_homing--;  // Decrease by 1 for next move if needed
    stepperZ.run();  // Start moving the stepper
    delayMicroseconds(50);
  }
  init_homing = 1;
  stepperZ.setCurrentPosition(0); 
  delay(5);
  // Serial.println("Stepper Z is Homed \n");
  SerialWrite("Stepper Z is Homed \n");
  stepperZ.setMaxSpeed(maxSpeed);
  stepperZ.setAcceleration(MaxAccel);
}

void moveStepper(long int steps){
  stepperZ.moveTo(steps);
  // delay(5);
  while((stepperZ.distanceToGo() != 0) && (SafeToRun == true)){
    stepperZ.run(); 
    // delay(5);
  }
  SerialWrite("Z stopped at: ");
  char intstr [10];
  ltoa(stepperZ.currentPosition(), intstr, 10);
  SerialWrite(intstr);
  SerialWrite("\n");
  // Serial.print("Z stopped at: "); Serial.println(stepperZ.currentPosition());
}


void SerialWrite(const char* array) //to write with the Interrupt driven, I created this function
{
  TXINDEX = 1;  //Set the index to be the second character in the string/array
  strcpy(TXBUFF, array);
  UDR0 = TXBUFF[0]; //load the first character into the UART0 TX/RX buffer
  delay(5);
}

bool Validate(long int Value, int jointNum)
{
  switch (jointNum)
  {
    case 1:
      if(Value >= StepperMin && Value <= StepperMax){return true;}
      else{return false;}
      break;
    case 2:
      if(Value >= Joint2Min && Value <= Joint2Max){return true;}
      else{return false;}
      break;
    case 3:
      if(Value >= Joint3Min && Value <= Joint3Max){return true;}
      else{return false;}
      break;
    case 4:
      if(Value >= Joint4Min && Value <= Joint4Max){return true;}
      else{return false;}
      break;
    default:
      return false;
      break;
  }
}