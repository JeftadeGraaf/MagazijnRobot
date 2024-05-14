#include <Arduino.h>
#include "Src/JoystickModule/Joystick.h"
#include "Src/MotorModule/Motor.h"
#include "Src/WireCommModule/WireComm.h"

#define emergencyButtonPin 4
#define resetButtonPin 10
#define arduinoAddress 8
#define secondArduinoAddress 9
#define rotatyPinXa 2
#define rotatyPinXb 5
const int lInductiveSensor = 6;
const int rInductiveSensor = 7;
bool lInduction = true;
bool rInduction = true;
bool tYSwitch = false;
bool bYSwitch = false;
bool isZAxisOut = true;
bool callibrate = true;
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

int possitionX = 0;
int possitionY = 0;

enum RobotState{
    automatic,
    manual,
    off,
    callibrating
};

RobotState currentState;

void setup()
{
    pinMode(emergencyButtonPin, INPUT_PULLUP);
    pinMode(resetButtonPin, INPUT_PULLUP);
    pinMode(lInductiveSensor, INPUT);
    pinMode(rInductiveSensor, INPUT);
    pinMode(rotatyPinXa, INPUT);
    pinMode(rotatyPinXb, INPUT);
    attachInterrupt(digitalPinToInterrupt(rotatyPinXa), readRotarty, RISING);
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
        // Serial.println(msg);
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
        } else if(msg == "my0h") {
            tYSwitch = false;
        } else if(msg == "my1h") {
            tYSwitch = true;
        } else if(msg == "my0l") {
            Serial.println(msg);
            bYSwitch = false;
        } else if(msg == "my1l") {
            Serial.println(msg);
            bYSwitch = true;
        } else if(msg.startsWith("py")) {
            // Serial.println(msg.substring(2));
            possitionY = msg.substring(2).toInt();
        }
        wireComm.setHasReceivedData(false);
    }
    if (millis() - lastComCheckTime >= 500 && millis() - lastComCheckTime <= 1000) {
        Wire.requestFrom(9, 1);
        i2cCheck();
    } else if (millis() - lastComCheckTime > 1000 ) {
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
                lastComCheckTime = millis();
            }
            break;
        case callibrating:
            callibrateMotor();
            break;
        default:
            currentState = off;
            break;
    }
}

void initialiseState(){
    currentState = callibrating;
    wireComm.sendData("cal");
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
        if((yValue < 0 && !tYSwitch) || (yValue > 0 && !bYSwitch) || yValue == 0){
            y_axisMotor.setManualPower(yValue);
        } else {
            y_axisMotor.setManualPower(0);
        }
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

void readRotarty(){
  if (digitalRead(rotatyPinXb)) {
    possitionX++;
  } else {
    possitionX--;
  }
}

void callibrateMotor() {
    if (!isZAxisOut) {
        if (digitalRead(rInductiveSensor)) {
            x_axisMotor.setManualPower(128);
        } else {
            x_axisMotor.setManualPower(0);
            possitionX = 0;
        }
        if (!bYSwitch) {
            y_axisMotor.setManualPower(128);
        } else {
            y_axisMotor.setManualPower(0);
        }

        if (!digitalRead(rInductiveSensor) && bYSwitch)
        {
            turnOffRobot();
        }
    }
}