// Team Robot Manipulator Project
// SCARA arm design
// 1 x Nema 23 drives the Z axis
// 3 x Servos to drive Joint 2, 3 and gripper

// To do:  - Confirm pin selection with Stan
//         - Add interrupt capability
//         - Add Speed control to servos

#include <Arduino.h>
#include <Servo.h> 
#include <AccelStepper.h>

// #define LED         13 
#define joint2_pin  2
#define joint3_pin  3
#define joint4_pin  4
#define Gripper_pin 5

#define Z_en        6    
#define Z_homeSw    9
#define Z_dir       A4    
#define Z_stp       A5    

#define BUF_LEN     20

#define STOP        '#'
#define HOME        'h' 

#define USART_BAUDRATE 115200
#define MYUBRR (((F_CPU / (USART_BAUDRATE * 16UL))))
#define TXBUFFSIZE 64
#define RXbufSize 64 //Rx buffer size in bytes

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
void homeStepper(void);
void moveStepper(int steps);
void SerialWrite(const char* array);

// ----------------- Globals -----------------------------=---//
long int homeSpeed  = 550; 
long int maxSpeed   = 1000; 
long int homeAccel  = 300; 
long int MaxAccel   = 500; 
long int init_homing = -1;
bool Homed = false;     // -- make a homed variable to ensure unit is homed before opperation
char TXBUFF[TXBUFFSIZE];
int TXINDEX = 0;
char RXbuff[RXbufSize];
int rxbuffIndex = 0;
double Time = 0;
double currentTime = 0;
double previousTime = 0;

int i = 0;
int k = 0;

bool SafeToRun = false;
int LEDstate = 0;
void setup() { 
  // --------------- Pin setup ------------------ //
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(joint2_pin, OUTPUT);
  pinMode(joint3_pin, OUTPUT);
  pinMode(joint4_pin, OUTPUT);
  pinMode(Gripper_pin, OUTPUT);
  pinMode(Z_en, OUTPUT);
  pinMode(Z_homeSw, INPUT);
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);   // Turn on the transmission and reception circuitry
  UCSR0C = (1 << UCSZ00) | (1 << UCSZ01); // Use 8-bit character sizes

  UBRR0H = (MYUBRR >> 8); // Load upper 8-bits of the baud rate value into the high byte of the UBRR register
  UBRR0L = MYUBRR; // Load lower 8-bits of the baud rate value into the low byte of the UBRR register

  UCSR0B |= (1 << RXCIE0); // Enable the USART Recieve Complete interrupt (USART_RXC)
  UCSR0B |= (1 << TXCIE0); // Enable the USART Transmit Complete interrupt (USART_TXC)

  sei(); // Enable the Global Interrupt Enable flag so that interrupts can be processed 
  
  // Serial.begin(115200);

  joint2.attach(joint2_pin, 790, 2150);  // attaches the servo on pin 2
  joint3.attach(joint3_pin, 790, 2150);  // attaches the servo on pin 3
  joint4.attach(joint3_pin, 790, 2150);  // attaches the servo on pin 4
  Gripper.attach(Gripper_pin, 790, 2150);  // attaches the servo on pin 5


  // ----------------- set servos to go to initial angle
  joint2.write(0);  
  joint3.write(0);
  joint4.write(0);
  Gripper.write(0);
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
    SerialWrite("Stepper stopped\n");
    digitalWrite(LED_BUILTIN, HIGH);
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
  digitalWrite(LED_BUILTIN, LOW);
  if(TXBUFF[TXINDEX]) //if it is not the end of the string
  {
    UCSR0B |= (1 << TXB80); //clear the flag
    UDR0 = TXBUFF[TXINDEX]; //load the next byte in
    TXINDEX++;  //increment the counter
  }
}
 
 
void loop() { 
  handleSerial();
  currentTime = millis();
  if ((currentTime - Time) >= 1000){
    LEDstate ^= 1;
    digitalWrite(LED_BUILTIN, LEDstate); // -- flash led just cos
    Time = currentTime;
  }
  }

void handleSerial(){
  int Angle;
  int steps;
  if (rxbuffIndex>=RXbufSize) {
      rxbuffIndex = 0;
      SerialWrite("BUFFER OVERRUN\n");
      // Serial.print("BUFFER OVERRUN\n"); 
    }
  
  if (rxbuffIndex > 0 && RXbuff[rxbuffIndex-1]=='\n'){  //if there is something in the buffer an the last character is a newline
    digitalWrite(LED_BUILTIN, HIGH);
    // delay(100);
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
            joint2.write(Angle);
            SerialWrite("Joint 2 moved to:");
            char intstr [10];
            itoa(Angle, intstr, 10);
            SerialWrite(intstr);
            SerialWrite("\n");
            // Serial.print("Joint 2 moved to:"); 
            // Serial.println(Angle);
            }
          break;

        case '3':
          // servo 2 on joint 3
          if (rxbuffIndex>1){
            Angle = atoi(&RXbuff[1]);
            joint3.write(Angle);
            SerialWrite("Joint 3 moved to:");
            char intstr [10];
            itoa(Angle, intstr, 10);
            SerialWrite(intstr);
            SerialWrite("\n");
            // Serial.print("Joint 3 moved to:"); Serial.println(Angle);
            }
          break;

          case '4':
          // servo 3 on joint 4
          if (rxbuffIndex>1){
            Angle = atoi(&RXbuff[1]);
            joint3.write(Angle);
            SerialWrite("Joint 4 moved to:");
            char intstr [10];
            itoa(Angle, intstr, 10);
            SerialWrite(intstr);
            SerialWrite("\n");
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
            steps = atoi(&RXbuff[1]);
            SerialWrite("Z-Axis moved to:");
            char intstr [10];
            itoa(steps, intstr, 10);
            SerialWrite(intstr);
            SerialWrite("\n");
            // Serial.print("Z axis moved to:"); Serial.println(steps);
            moveStepper(steps);
            }
          break;

        case HOME:
          // Home Z axis
          homeStepper();
          SerialWrite("Stepper homed\n");
          // Serial.println("Stepper homed");
          break;

        case STOP:
          // stop Z axis
          stepperZ.stop();
          SerialWrite("Stepper stopped\n");
          // Serial.println("Stepper stopped");
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

void homeStepper(void){
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
    // delay(5);
  }
  init_homing = -1;
  stepperZ.setCurrentPosition(0); 
  delay(5);
  // Serial.println("Stepper Z is Homed \n");
  SerialWrite("Stepper Z is Homed \n");

}

void moveStepper(int steps){
  stepperZ.moveTo(steps);
  // delay(5);
  while((stepperZ.distanceToGo() != 0) && (SafeToRun == true)){
    stepperZ.run(); 
    // delay(5);
  }
  SerialWrite("Z stopped at: ");
  char intstr [10];
  itoa(stepperZ.currentPosition(), intstr, 10);
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