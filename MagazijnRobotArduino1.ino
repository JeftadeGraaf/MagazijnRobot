#include <Arduino.h>
#include "Src/JoystickModule/Joystick.h"
#include "Src/MotorModule/Motor.h"
#include "Src/WireCommModule/WireComm.h"
#include "Src/JavaSerialModule/JavaSerial.h"

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
JavaSerial javaSerial = JavaSerial();

Joystick joystick = Joystick(A3, A2, 30); //parameters: xAxis, yAxis, deadZone
Motor x_axisMotor = Motor(3, 12, 8, A0); //parameters: pwmPin, directionPin, brakePin, currentSensingPin
Motor y_axisMotor = Motor(11, 13, 9, A1); //parameters: pwmPin, directionPin, brakePin, currentSensingPin

bool resetButtonWasPressed = false;
bool emergencyButtonWasPressed = false;

const int xPositions[6] = {0, 1595, 2295, 3000, 3690, 4395};
const int yPositions[6] = {0, 60, 560, 1060, 1560, 2060};

int debounceTime = 200;
unsigned long resetButtonTimer = 0;
unsigned long lastComCheckTime = 0;
unsigned long lastPossitionUpdate = 0;

int possitionX = 0;
int possitionY = 0;

int coordinates[3][2];
int nextX = 0;
int nextY = 0;


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
    pinMode(rotatyPinXa, INPUT_PULLUP);
    pinMode(rotatyPinXb, INPUT_PULLUP);
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

    if(millis() - lastPossitionUpdate >= 100 && currentState != callibrating) {
        // javaSerial.writeSerial("l"+String(possitionX)+","+String(possitionY));
        lastPossitionUpdate = millis();
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
            // Serial.println(msg);
            bYSwitch = false;
        } else if(msg == "my1l") {
            // Serial.println(msg);
            bYSwitch = true;
        } else if(msg.startsWith("py")) {
            // Serial.println(msg.substring(2));
            possitionY = msg.substring(2).toInt();
        }
        wireComm.setHasReceivedData(false);
    }
    if (millis() - lastComCheckTime >= 500 && millis() - lastComCheckTime <= 1300) {
        Wire.requestFrom(9, 1);
        i2cCheck();
    } else if (millis() - lastComCheckTime > 1300 ) {
        turnOffRobot();
    }
    checkSerial();
}

void handleRobotState(){
    switch (currentState){
        case automatic:
            if(isResetButtonPressed()){
                switchToManualState();
            }
            handleAutomaticInput();
            break;
        case manual:
            handleManualInput();
            if(isResetButtonPressed()){
                switchToAutomaticState();
            }
            break;
        case off:
            if(isResetButtonPressed()){
                switchToCallibration();
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

void switchToCallibration(){
    currentState = callibrating;
    wireComm.sendData("cal");
}

void handleManualInput(){
    lInduction = digitalRead(lInductiveSensor);
    rInduction = digitalRead(rInductiveSensor);
    if (!rInduction) {
        possitionX = 0;
    }
    
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

void handleAutomaticInput() {
    lInduction = digitalRead(lInductiveSensor);
    rInduction = digitalRead(rInductiveSensor);
    if (!rInduction) {
        possitionX = 0;
    }

    if (!isZAxisOut) {
        if (possitionX < xPositions[nextX] - 5 && lInduction) {
            x_axisMotor.setManualPower(-255);
        } else if (possitionX > xPositions[nextX] + 5 && rInduction)
        {
            x_axisMotor.setManualPower(255);
        } else {
            x_axisMotor.setManualPower(0);
        }

        if (possitionY < yPositions[nextY] - 15 && !tYSwitch) {
            y_axisMotor.setManualPower(-255);
        } else if (possitionY > yPositions[nextY] + 20 && !bYSwitch)
        {
            y_axisMotor.setManualPower(255);
        } else {
            y_axisMotor.setManualPower(0);
        }
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
    possitionX--;
  } else {
    possitionX++;
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
            possitionX = 0;
            switchToManualState();
            wireComm.sendData("dcal");
        }
    }
}

void checkSerial()
{
    if (javaSerial.messageAvailable())
    {
        wireComm.sendData("rs");
        unsigned long currentTime = millis();  
        String msg = javaSerial.readSerial();

        if (msg.startsWith("o"))
        {

            String coordinate = "";
            int coordinateNumber = 0;
            coordinates[0][0] = msg[1] - '0';
            coordinates[0][1] = msg[3] - '0';
            coordinates[1][0] = msg[5] - '0';
            coordinates[1][1] = msg[7] - '0';
            coordinates[2][0] = msg[9] - '0';
            coordinates[2][1] = msg[11] - '0';

            nextX = msg[1] - '0';
            nextY = msg[3] - '0';
            Serial.println(String(nextX) + " " + String(nextY));
            wireComm.sendData("sr");
            lastComCheckTime = millis();
        }
        // lastComCheckTime = millis();
        // if (currentState == automatic) {
        //     switchToAutomaticState();
        // }
    }
}