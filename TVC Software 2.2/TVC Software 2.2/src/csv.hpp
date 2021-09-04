#include <Arduino.h>

void csv(float param1=0, float param2=0, float param3=0, float param4=0, float param5=0, float param6=0, float param7=0, float param8=0, float param9=0, float param10=0, float param11=0, float param12=0){
    Serial.print(param1, 2);
    Serial.print(",");
    Serial.print(param2, 2);
    Serial.print(",");
    Serial.print(param3, 2);
    Serial.print(",");
    Serial.print(param4, 2);
    Serial.print(",");
    Serial.print(param5, 2);
    Serial.print(",");
    Serial.print(param6, 2);
    Serial.print(",");
    Serial.print(param7, 2);
    Serial.print(",");
    Serial.print(param8, 2);
    Serial.print(",");
    Serial.print(param9, 2);
    Serial.print(",");
    Serial.print(param10, 2);
    Serial.print(",");
    Serial.print(param11, 2);
    Serial.print(",");
    Serial.print(param12, 2);
    Serial.println("");
}