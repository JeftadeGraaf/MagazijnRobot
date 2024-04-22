#include <Arduino.h>
#include "Joystick.h"


Joystick::Joystick(int xPin, int yPin, int deadzone){
    this -> xPin = xPin;
    this -> yPin = yPin;
    this -> deadzone = deadzone;
}

void Joystick::registerPins(){
    pinMode(this->xPin, INPUT);
    pinMode(this->yPin, INPUT);
}

int Joystick::readXAxis(){
    int xValue = map(analogRead(this->xPin), 0, 1023, 0, 255);
    if(xValue >= 127 - this -> deadzone && xValue <= 127 + this-> deadzone){
        return 0;
    } else if(xValue < 127){
        xValue = (127 - xValue) * -1;
    } else {
        xValue = map(xValue, 127, 255, 0, 127);
    }
    return xValue * -1;
}

int Joystick::readYAxis(){
    int yValue = map(analogRead(this->yPin), 0, 1023, 0, 255);
    if(yValue >= 127 - this -> deadzone && yValue <= 127 + this-> deadzone){
        return 0;
    } else if(yValue < 127){
        yValue = (127 - yValue) * -1;
    } else {
        yValue = map(yValue, 127, 255, 0, 127);
    }
    return yValue;
}