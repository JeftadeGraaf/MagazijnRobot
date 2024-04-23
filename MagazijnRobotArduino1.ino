#include <Arduino.h>
#include "Src/JoystickModule/Joystick.h"
#include "Src/MotorModule/Motor.h"
#include "Src/JavaSerialModule/JavaSerial.h"
#include <Wire.h>

#define emergencyButtonPin 4
#define resetButtonPin 10
const int lInductiveSensor = 6;
const int rInductiveSensor = 7;
bool lInduction = true;
bool rInduction = true;


Joystick joystick = Joystick(A3, A2, 30);
Motor x_axisMotor = Motor(3, 12, 8, A0);
Motor y_axisMotor = Motor(11, 13, 9, A1);
JavaSerial serialModule = JavaSerial();

bool resetButtonWasPressed = false;
bool emergencyButtonWasPressed = false;

int debounceTime = 200;
unsigned long resetButtonTimer = 0;
unsigned long emergencyButtonTimer = 0;


enum RobotState{
    automatic,
    manual,
    off
};

RobotState currentState = manual;

void setup()
{
    pinMode(emergencyButtonPin, INPUT_PULLUP);
    pinMode(resetButtonPin, INPUT_PULLUP);
    pinMode(6, INPUT);
    pinMode(7, INPUT);
    Wire.begin();
    Serial.begin(9600);
    joystick.registerPins();
    x_axisMotor.registerPins();
    y_axisMotor.registerPins();
}

void loop()
{
    lInduction = digitalRead(lInductiveSensor);
    rInduction = digitalRead(rInductiveSensor);
    if(isEmergencyButtonPressed()){
        turnOffRobot();
    }
    
    serialModule.readSerial();

    switch (currentState){
        case automatic:
            if(isResetButtonPressed()){
                switchToManualState();
            }
            break;
        case manual:
            handleManualInput();
            if(isResetButtonPressed()){
                turnOffRobot();
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
    

    
    if((xValue < 0 && lInduction) || (xValue > 0 && rInduction) || xValue == 0){
        x_axisMotor.setManualPower(xValue);
    } else {
        x_axisMotor.setManualPower(0);
    }
    int yValue = joystick.readYAxis();
    y_axisMotor.setManualPower(yValue);
}

void sendMessage(int address, String msg){
    Wire.beginTransmission(address);
    Wire.write(msg.c_str());
    Wire.endTransmission();
}


bool isResetButtonPressed(){
  if(digitalRead(resetButtonPin) == LOW){
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

bool isEmergencyButtonPressed(){
  if(digitalRead(emergencyButtonPin) == LOW){
    if(!emergencyButtonWasPressed){
    	if(millis() - emergencyButtonTimer >= debounceTime){
    	  emergencyButtonWasPressed = true;	
          return true;
    	}
    }  
  } else {
    emergencyButtonTimer = millis();
    emergencyButtonWasPressed = false;
  }
  return false;
}