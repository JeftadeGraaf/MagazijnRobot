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
    if(axisValue < 0){
        digitalWrite(directionPin, LOW);
        digitalWrite(pwmPin, (axisValue * -1));

    } else {
        digitalWrite(directionPin, HIGH);
        digitalWrite(pwmPin, axisValue);
    }
}