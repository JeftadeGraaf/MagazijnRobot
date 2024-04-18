#include <Arduino.h>
#include "Src/JoystickModule/Joystick.h"
#include "Src/MotorModule/Motor.h"
#include <Wire.h>

#define emergencyButtonPin 2
#define resetButtonPin 10

Joystick joystick = Joystick(A2, A3, 30);
Motor x_axisMotor = Motor(3, 12);
Motor y_axisMotor = Motor(11, 13);

bool resetButtonWasPressed = false;
int debounceTime = 200;
unsigned long resetButtonTimer = 0;


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
            if(isResetButtonPressed()){
                switchToManualState();
            }
            break;
        case manual:
            handleManualInput();
            if(isResetButtonPressed()){
                switchToAutomaticState();
            }
            break;
        case off:
            if(isResetButtonPressed()){
                switchToManualState();
            }
            break;
        default:
            currentState = off;
            break;
    }
}

void turnOffRobot(){
    currentState = off;
    x_axisMotor.setManualPower(0);
    y_axisMotor.setManualPower(0);
    sendMessage(9, "off");
}

void switchToManualState(){
    currentState = manual;
    sendMessage(9, "man");
}

void switchToAutomaticState(){
    currentState = automatic;
    sendMessage(9, "aut");
}

void handleManualInput(){
    int xValue = joystick.readXAxis();
    x_axisMotor.setManualPower(xValue);
    int yValue = joystick.readYAxis();
    y_axisMotor.setManualPower(yValue);
}

void sendMessage(int address, String msg){
    Wire.beginTransmission(address);
    Wire.write(msg.c_str());
    Wire.endTransmission();
}


bool isResetButtonPressed(){
  if(digitalRead()){
    if(!resetButtonWasPressed){
    	if(millis() - resetButtonTimer >= debounceTime){
    	  resetButtonWasPressed = true;	
          return true;
    	}
    }  
  } else {
    resetButtonTimer = millis();
    resetButtonWasPressed = false;
  }
  return false;
}