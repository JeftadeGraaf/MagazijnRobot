#include <Arduino.h> 
#include "JavaSerial.h"


JavaSerial::JavaSerial(){
    Serial.begin(9600);    
}

void JavaSerial::readSerial(){
    if (Serial.available() > 0) {
        String inputString = Serial.readString();

        Serial.println(inputString + "\n");
    }
};