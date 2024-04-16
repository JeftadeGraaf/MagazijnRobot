#include <Arduino.h> 
#include "Motor.h"



Motor::Motor(int pwmPin, int directionPin){
    this-> pwmPin = pwmPin;
    this-> directionPin = directionPin;
}
void Motor::registerPins(){
    pinMode(this->pwmPin, OUTPUT);
    pinMode(this->directionPin, OUTPUT);
}
void Motor:: setManualPower(int axisValue){
    if(axisValue < 127){
        axisValue = 127 - axisValue;
        digitalWrite(directionPin, LOW);
    } else {
        axisValue = axisValue - 127;
        digitalWrite(directionPin, HIGH);
    }

    digitalWrite(pwmPin, axisValue);
}