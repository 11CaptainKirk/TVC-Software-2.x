#include <Arduino.h>
#include <SD.h>

void writeSD (float EulerX, float EulerY, float EulerZ, float bmpTemp, float bmpPressure, float kalmanAltitude, float bmpAltitude, double lat, double lng, float gpsAltitude, int numSats){
    File logFile = SD.open("flightData1.txt", FILE_WRITE);
  if (logFile) {
    logFile.print(millis());
    logFile.print(F(","));
    logFile.print(EulerX,3);
    logFile.print(F(","));
    logFile.print(EulerY,3);
    logFile.print(F(","));
    logFile.print(EulerZ,3);
    logFile.print(F(","));
    logFile.print(bmpTemp,3);
    logFile.print(F(","));
    logFile.print(bmpPressure/100,3);
    logFile.print(F(","));
    logFile.print(bmpAltitude,3);
    logFile.print(F(","));
    logFile.print(lat, 4);
    logFile.print(F(","));
    logFile.print(lng, 4);
    logFile.print(F(","));
    logFile.print(gpsAltitude, 3);
    logFile.print(F(","));
    logFile.print(numSats);
    logFile.println(F(""));
    logFile.close();
  }
}
