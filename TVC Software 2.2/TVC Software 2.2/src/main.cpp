#include <Arduino.h>
#include <PWMServo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include "csv.hpp"
#include "beep.hpp"
#include <Adafruit_BMP085.h>
#include <SimpleKalmanFilter.h>
#include <TinyGPS++.h>
#include <SD.h>
#include "SDwrite.hpp"
#include <PID_v1.h>
#include "MPU9250.h"
#include "quaternion.hpp"


const int Proportional = 1;
const int Integral = 1;
const int Derivative = 1;

const int servoHomeY = 84;
const int servoHomeZ = 83;

// * Setup Servos
int servoYPin = 4;  // Set Servo Pin
PWMServo ServoY;      // Create Servo Object
int servoZPin = 5;  
PWMServo ServoZ;      
//

double Setpoint;
double InputY, InputZ;
double OutputY, OutputZ;

PID PIDy(&InputY, &OutputY, &Setpoint, 1, 0, 0, DIRECT); // PID Y //TODO: Add variables to PID values.
PID PIDz(&InputZ, &OutputZ, &Setpoint, 1, 0, 0, DIRECT); // PID Z

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); // Create IMU Object
Adafruit_BNO055 bno2 = Adafruit_BNO055(55, 0x29); // Create IMU Object

double seaPressure = 1013.25;

Adafruit_BMP085 bmp;

SimpleKalmanFilter simpleKalmanFilter(1, 1, 0.1);

TinyGPSPlus gps;


const int piezoPin = 23;

const int buttonPin = 39;
int buttonState;
int prevButtonState;
bool systemState = false;

float lat= -1;
float lng = -1;
float gpsAltitude = -1;
float numSats = -1;

const int sdWriteInterval = 30;   // How many ms between SD writes?
int prevSDWriteTime = 0;

// GPS ~~~~~~~~~~~~~~~~~~~~~~~~~

void writeInfo(){
 // Serial.print(F("Location: ")); 
    if (gps.location.isValid())
    {
     // Serial.print(gps.location.lat(), 6);
     // Serial.print(F(","));
     // Serial.print(gps.location.lng(), 6);
     // Serial.print(gps.altitude.meters(), 6);
      lat = gps.location.lat();
      lng = gps.location.lng();
      gpsAltitude = gps.altitude.meters();
    }
    else
    {
     // Serial.print(F("INVALID"));
    }

   // Serial.print(F("  Date/Time: "));
    if (gps.date.isValid())
    {
    //  Serial.print(gps.date.month());
    //  Serial.print(("/"));
    //  Serial.print(gps.date.day());
    //  Serial.print(("/"));
    //  Serial.print(gps.date.year());
    }
    else
    {
    //  Serial.print(F("INVALID"));
    }

   // Serial.print(F(" "));
    if (gps.time.isValid())
    {
    //  if (gps.time.hour() < 10) Serial.print(F("0"));
    //  Serial.print(gps.time.hour());
    //  Serial.print((":"));
    //  if (gps.time.minute() < 10) Serial.print(F("0"));
    //  Serial.print(gps.time.minute());
     // Serial.print((":"));
    //  if (gps.time.second() < 10) Serial.print(F("0"));
     // Serial.print(gps.time.second());
     // Serial.print(("."));
    //  if (gps.time.centisecond() < 10) Serial.print(F("0"));
     // Serial.print(gps.time.centisecond());
    }
    else
    {
    //  Serial.print(F("INVALID"));
    }
   // Serial.println(gps.satellites.value());
    numSats = gps.satellites.value();

   // Serial.println();
}

// ! PULSE FUNCTION ~~~~~~~~~~

int pulseState = LOW;
unsigned long prevTime;

void pulse(int pin, unsigned int interval){
    if (millis()-prevTime >= interval){  
        prevTime = millis();
        if (pulseState == LOW){
          pulseState = HIGH;
        } else {
          pulseState = LOW;
        }
    }
    digitalWrite(pin, pulseState);
}

// ! ~~~~~~~~~~~~~~~~~~~~~~~~~

void stateChangePulse(int onTime, int offTime, int numPulses = 2){
      for ( int i = 0; i < numPulses; i++){
      digitalWrite(piezoPin, HIGH);
      delay(onTime);
      digitalWrite(piezoPin, LOW);
      delay(offTime);
      }
}


// ? BUTTON READ FUNCTION ~~~~
void buttonRead () {
  buttonState = digitalRead(buttonPin);
  if((buttonState != prevButtonState) && (buttonState == HIGH)) {
    systemState = !systemState;
    if (systemState == true){
      stateChangePulse(70, 100, 3);
    } else {
      stateChangePulse(300, 200);
    }
  }
  prevButtonState = buttonState;
}
// ? ~~~~~~~~~~~~~~~~~~~~~~~







void setup() {
  Serial.begin(115200);
  Serial1.begin(9600);

ServoY.write(servoHomeY+(5));
delay(100);
ServoY.write(servoHomeY);

  Wire.begin();
  delay(2000);

  ServoY.attach(servoYPin);
  ServoZ.attach(servoZPin);
  pinMode(35, OUTPUT);
  digitalWrite(35, HIGH);
  pinMode(23, OUTPUT);
  digitalWrite(23, LOW);
  pinMode(39, INPUT_PULLUP);


  Serial.println("Orientation Sensor Test");
  Serial.println("");
  if(!bno.begin()){
    Serial.print("** BNO055 not detected **\t\t<<");
    while(1);
  }
  delay(1000); //TODO: Why this delay?
  bno.setExtCrystalUse(true);
  //

  Serial.println("Orientation Sensor 2 Test");
  Serial.println("");
  if(!bno2.begin()){
    Serial.print("** BNO055 2 not detected **\t\t<<");
    while(1);
  }
  delay(1000); //TODO: Why this delay?
  bno2.setExtCrystalUse(true);
  //





  if (!bmp.begin()) {
	Serial.println("Could not find a valid BMP085 sensor, check wiring!");
	while (1) {}
  }

  prevTime = 0;


  InputY = 0;
  InputZ = 0;
  Setpoint = 0;

  PIDy.SetMode(AUTOMATIC); // Turn PID on
  PIDz.SetMode(AUTOMATIC); 
  PIDy.SetOutputLimits(-255,255);
  PIDz.SetOutputLimits(-255,255); // Allow Negative Outputs
  PIDy.SetSampleTime(25);
  PIDz.SetSampleTime(25); // Increase update frequency (default 200)

// * SD INIT ~~~~~~~~~~~~~~~

if(!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("1. is a card inserted?");
    Serial.println("2. is your wiring correct?");
    Serial.println("3. did you change the chipSelect pin to match your shield or module?");
    Serial.println("Note: press reset or reopen this serial monitor after fixing your issue!");
    while (true);
  }
  Serial.println("SD initialization success.");


//* ~~~~~~~~~~~~~~~~~~~~~~~~~
while (Serial1.available() > 0)
    if (gps.encode(Serial1.read())){
       while (!gps.location.isValid()){
         delay(100);
       }
       if (gps.location.isValid()){
         stateChangePulse(50, 50, 5);
       }
      }

}

void loop() {



if (systemState) {



sensors_event_t event;
bno.getEvent(&event);

sensors_event_t event2;
bno2.getEvent(&event2);

float EulerX = event.orientation.x;
float EulerY = event.orientation.y;
float EulerZ = event.orientation.z;

float EZ2 = event2.orientation.z;

float bmpTemp = bmp.readTemperature();
float bmpPressure = bmp.readPressure();
float bmpAltitude = bmp.readAltitude();


InputY = EulerY;
InputZ = EulerZ;



PIDy.Compute();
PIDz.Compute();

// ? ~~~SERVO MOVE~~~~~~~~~~~~~~~~~~~~

if(OutputY >= 10){
  ServoY.write(servoHomeY+10);
} else if(OutputY <= -10){
  ServoY.write(servoHomeY-10);
} else {
  ServoY.write(servoHomeY+(OutputY));
}

if(OutputZ >= 10){
  ServoZ.write(servoHomeZ+10);
} else if(OutputZ <= -10){
  ServoZ.write(servoHomeZ-10);
} else {
  ServoZ.write(servoHomeZ+(OutputZ));
}
// ? ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// ! ~~~    GPS    ~~~~~~~~~~~~~~~~~~~~

while (Serial1.available() > 0)
    if (gps.encode(Serial1.read())){
      writeInfo();
      }
if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while(true);
  }

// ! ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


buttonRead();

float kalmanAltitude = simpleKalmanFilter.updateEstimate(bmpAltitude);

pulse(35, 500);

csv(EulerX, EulerY, EulerZ, bmpTemp, kalmanAltitude, bmpAltitude, lat, lng, gpsAltitude, EZ2);
// ! LOG TO SD CARD ~~~~~~~~~~~~~~~~~~~~~~~
if (millis() - prevSDWriteTime >= sdWriteInterval){    // Check to see if enough time has passed to write to the sd card again.

    writeSD(EulerX, EulerY, EulerZ, bmpTemp, bmpPressure, kalmanAltitude, bmpAltitude, lat, lng, gpsAltitude, numSats);
    prevSDWriteTime = millis();
}
// ! ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 delay(10);  // Temporary
  
}

else {
  pulse(35, 5000);
  buttonRead();
}

}