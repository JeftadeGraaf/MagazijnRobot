#line 1 "C:\\Users\\mikes\\Desktop\\Windesheim\\Semester 2\\KBS\\MagazijnRobot\\Src\\JoystickModule\\Joystick.cpp"
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
    return analogRead(this->xPin);
}

int Joystick::readYAxis(){
    return analogRead(this->yPin);
}