#include <Arduino.h>
#include "Src/JoystickModule/Joystick.h"
#include "Src/MotorModule/Motor.h"

Joystick joystick = Joystick(A2, A3, 30);
Motor x_axisMotor = Motor(3, 12);
Motor y_axisMotor = Motor(11, 13);
int emergencyButtonPin = 2;


enum RobotState{
    automatic,
    manual,
    off
};

RobotState currentState = off;

void setup()
{
    pinMode(emergencyButtonPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(emergencyButtonPin), turnOffRobot, FALLING);
    Serial.begin(9600);
    joystick.registerPins();
    x_axisMotor.registerPins();
    y_axisMotor.registerPins();
}

void loop()
{
    switch (currentState){
        case automatic:
            break;
        case manual:
            handleManualInput();
            break;
        case off:
            return;
        default:
            currentState = off;
            break;
    }
}

void turnOffRobot(){
    currentState = off;
}

void handleManualInput(){
}