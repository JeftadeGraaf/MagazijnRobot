#include <Arduino.h>
#include "Joystick.h"


Joystick::Joystick(int xPin, int yPin){
    this -> xPin = xPin;
    this -> yPin = yPin;
}

void Joystick::registerPins(){
    pinMode(this->xPin, INPUT);
    pinMode(this->yPin, INPUT);
}

int Joystick::readXAxis(){
    return map(analogRead(this->xPin), 0, 1023, 0, 255);
}

int Joystick::readYAxis(){
    return map(analogRead(this->yPin), 0, 1023, 0, 255);
}