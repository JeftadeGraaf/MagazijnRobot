#include <Arduino.h>
#include "Src/JoystickModule/Joystick.h"
#include "Src/MotorModule/Motor.h"
#include <Wire.h>

Joystick joystick = Joystick(A2, A3, 30);
Motor x_axisMotor = Motor(3, 12);
Motor y_axisMotor = Motor(11, 13);
int emergencyButtonPin = 2;


enum RobotState{
    automatic,
    manual,
    off
};

RobotState currentState = manual;
RobotState previousState = off;

void setup()
{
    pinMode(emergencyButtonPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(emergencyButtonPin), turnOffRobot, FALLING);
    Wire.begin();

    Serial.begin(9600);
    joystick.registerPins();
    x_axisMotor.registerPins();
    y_axisMotor.registerPins();
}

void loop()
{
    switch (currentState){
        case automatic:
            if(previousState != automatic){
                previousState = automatic;
                sendMessage(9, "aut");
            }
            break;
        case manual:
            if(previousState != manual){
                previousState = manual;
                sendMessage(9, "man");
            }
            handleManualInput();
            break;
        case off:
            if(previousState != off){
                previousState = off;
                sendMessage(9, "off");
            }
            break;
        default:
            currentState = off;
            break;
    }
}

void turnOffRobot(){
    currentState = off;
}

void handleManualInput(){
    int xValue = joystick.readXAxis();
    x_axisMotor.setManualPower(xValue);
}

void sendMessage(int address, String msg){
    Wire.beginTransmission(address);
    Wire.write(msg.c_str());
    Wire.endTransmission();
}