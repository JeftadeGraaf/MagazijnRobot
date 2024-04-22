#include <Arduino.h> 
#include "Motor.h"



Motor::Motor(int pwmPin, int directionPin, int brakePin, int currentSensingPin){
    this-> pwmPin = pwmPin;
    this-> directionPin = directionPin;
    this-> brakePin = brakePin;
    this-> currentSensingPin = currentSensingPin;
}
void Motor::registerPins(){
    pinMode(this->pwmPin, OUTPUT);
    pinMode(this->directionPin, OUTPUT);
}
void Motor:: setManualPower(int axisValue){
    if(axisValue == 0){
        digitalWrite(brakePin, HIGH);
        digitalWrite(pwmPin, 0);
    } else if (axisValue < 0){
        digitalWrite(brakePin, LOW);
        digitalWrite(directionPin, LOW);
        digitalWrite(pwmPin, (axisValue * -1));

    } else {
        digitalWrite(brakePin, LOW);
        digitalWrite(directionPin, HIGH);
        digitalWrite(pwmPin, axisValue);
    }
}