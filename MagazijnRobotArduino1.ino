#include <Arduino.h>
#include "Src/JoystickModule/Joystick.h"
#include "Src/MotorModule/Motor.h"
#include "Src/WireCommModule/WireComm.h"

#define emergencyButtonPin 4
#define resetButtonPin 10
#define arduinoAddress 8
#define secondArduinoAddress 9
const int lInductiveSensor = 6;
const int rInductiveSensor = 7;
bool lInduction = true;
bool rInduction = true;
bool isZAxisOut;
String msg = "";

WireComm wireComm = WireComm(arduinoAddress, secondArduinoAddress);


Joystick joystick = Joystick(A3, A2, 30); //parameters: xAxis, yAxis, deadZone
Motor x_axisMotor = Motor(3, 12, 8, A0); //parameters: pwmPin, directionPin, brakePin, currentSensingPin
Motor y_axisMotor = Motor(11, 13, 9, A1); //parameters: pwmPin, directionPin, brakePin, currentSensingPin

bool resetButtonWasPressed = false;
bool emergencyButtonWasPressed = false;

int debounceTime = 200;
unsigned long resetButtonTimer = 0;
unsigned long lastComCheckTime = 0;


enum RobotState{
    automatic,
    manual,
    off
};

RobotState currentState;

void setup()
{
    pinMode(emergencyButtonPin, INPUT_PULLUP);
    pinMode(resetButtonPin, INPUT_PULLUP);
    pinMode(lInductiveSensor, INPUT);
    pinMode(rInductiveSensor, INPUT);
    Serial.begin(9600);
    joystick.registerPins();
    x_axisMotor.registerPins();
    y_axisMotor.registerPins();
    lastComCheckTime = millis();
    initialiseState();
}

void loop()
{
    if(isEmergencyButtonPressed()){
        turnOffRobot();
    } else {
        handleRobotState();
    }
    if(wireComm.hasReceivedData()){
        String msg = wireComm.getReceivedData();
        Serial.println(msg);
        if(msg == "off"){
            turnOffRobot();
        } else if (msg == "man"){
            switchToManualState();
        } else if (msg == "aut"){
            switchToAutomaticState();
        } else if (msg == "mz0"){
            isZAxisOut = false;
        } else if (msg == "mz1"){
            isZAxisOut = true;
        }
        wireComm.setHasReceivedData(false);
    }
    if (millis() - lastComCheckTime >= 1000 && millis() - lastComCheckTime <= 2000) {
        Wire.requestFrom(9, 1);
        i2cCheck();
    } else if (millis() - lastComCheckTime > 2000 ) {
        Serial.println("No communication");
        turnOffRobot();
    }
}

void handleRobotState(){
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

void initialiseState(){
    currentState = off;
    wireComm.sendData("off");
}

void turnOffRobot(){
    currentState = off;
    x_axisMotor.setManualPower(0);
    y_axisMotor.setManualPower(0);
    wireComm.sendData("off");
}

void switchToManualState(){
    currentState = manual;
    wireComm.sendData("man");
}

void switchToAutomaticState(){
    currentState = automatic;
    wireComm.sendData("aut");
}

void handleManualInput(){
    lInduction = digitalRead(lInductiveSensor);
    rInduction = digitalRead(rInductiveSensor);
    if(!isZAxisOut){
        int xValue = joystick.readXAxis();
        if((xValue < 0 && lInduction) || (xValue > 0 && rInduction) || xValue == 0){
            x_axisMotor.setManualPower(xValue);
        } else {
            x_axisMotor.setManualPower(0);
        }
        int yValue = joystick.readYAxis();
        y_axisMotor.setManualPower(yValue);
    } else {
        x_axisMotor.setManualPower(0);
        y_axisMotor.setManualPower(0);
    }
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
    return digitalRead(emergencyButtonPin) == LOW;
}

void i2cCheck() {
    if (Wire.available() > 0) {
        char c = Wire.read();
        if (c == 'u') {
            lastComCheckTime = millis();
        }
    }
}