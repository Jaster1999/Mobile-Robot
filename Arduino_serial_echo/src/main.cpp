#include <Arduino.h>
#include <SoftwareSerial.h>

SoftwareSerial echo(3,2); //RX, TX

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  echo.begin(9600);
  echo.write("hello\n");

}

void loop() {
  // put your main code here, to run repeatedly:
  if (echo.available())
    Serial.write(echo.read());
  if (Serial.available())
    echo.write(Serial.read());
}