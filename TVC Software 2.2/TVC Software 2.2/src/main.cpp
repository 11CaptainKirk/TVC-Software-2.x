#include <Arduino.h>
#include <PWMServo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>



const int servoYpin = 4;
PWMServo servoY;
const int servoZpin = 5;
PWMServo servoZ;

const int servoYHome = 83;
const int servoZHome = 84;

void setup() {
  // put your setup code here, to run once:
  servoY.attach(servoYpin);
  servoZ.attach(servoZpin);
  delay(1000);
  servoY.write(servoYHome);
  servoZ.write(servoZHome);
  delay(500);
  servoY.write(servoYHome+10);
  delay(500);
  servoY.write(servoYHome+-10);
  delay(500);
  servoY.write(servoYHome);
  delay(500);
  servoZ.write(servoZHome+10);
  delay(500);
  servoZ.write(servoZHome+-10);
  delay(500);
  servoZ.write(servoZHome);
  
  for ( int i = 0; i <= 20; i++){
      servoZ.write(servoZHome+i-10);
      servoY.write(servoYHome+sqrt(400-(i*i))-10);
      delay(500);
  }
  for ( int i = 20; i >= 0; i--){
      servoZ.write(servoZHome+i-10);
      servoY.write(servoYHome-sqrt(400-(i*i))+10);
      delay(500);
  }

}

void loop() {
  // put your main code here, to run repeatedly:
}