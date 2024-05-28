#include <Arduino.h>
#include "Src/JoystickModule/Joystick.h"
#include "Src/MotorModule/Motor.h"
#include "Src/WireCommModule/WireComm.h"
#include "Src/JavaSerialModule/JavaSerial.h"

#define emergencyButtonPin 4
#define resetButtonPin 10
#define arduinoAddress 8
#define secondArduinoAddress 9
#define rotaryPinXa 2
#define rotaryPinXb 5
#define lInductiveSensor 6
#define rInductiveSensor 7
bool lInduction = true;
bool rInduction = true;
bool tYSwitch = true;
bool bYSwitch = true;
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
bool yAxisBraking = false;
bool packetPickedUp = false;
bool moveYUp = false;
bool zPositionSent = false;
bool xOnPosition = false;
bool yOnPosition = false;

const int xPositions[6] = {0, 1580, 2295, 3000, 3690, 4395};
const int yPositions[6] = {0, 80, 580, 1080, 1580, 2080};
const int zPositions[3] = {900, 720, 500};

#define debounceTime 200
unsigned long resetButtonTimer = 0;
unsigned long lastComCheckTime = 0;
unsigned long lastPositionUpdate = 0;
unsigned long yAxisBrakingTime = 0;

volatile int positionX = 0;
int positionY = 0;
int positionYPickup = 0;
#define allowedYMovementUp 95
#define allowedYMovementDown -70

int coordinates[3][2];
int nextX = 0;
int nextY = 0;
int item = 0;


enum RobotState{
    automatic,
    manual,
    off,
    callibrating
};

RobotState currentState;

void setup()
{
    // TCCR2B = TCCR2B & B11111000 | B00000111;  // for PWM frequency of 30.64 Hz

    pinMode(emergencyButtonPin, INPUT_PULLUP);
    pinMode(resetButtonPin, INPUT_PULLUP);
    pinMode(lInductiveSensor, INPUT);
    pinMode(rInductiveSensor, INPUT);
    pinMode(rotaryPinXa, INPUT_PULLUP);
    pinMode(rotaryPinXb, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(rotaryPinXa), readRotarty, RISING);
    lInduction = digitalRead(lInductiveSensor);
    rInduction = digitalRead(rInductiveSensor);
    Serial.begin(115200);
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
    if (millis() - lastComCheckTime >= 200 && millis() - lastComCheckTime <= 1300) {
        Wire.requestFrom(9, 1);
        i2cCheck();
    } else if (millis() - lastComCheckTime > 1300 ) {
        turnOffRobot();
    }    

    if(millis() - lastPositionUpdate >= 75  && currentState != callibrating) {
        javaSerial.writeSerial("l"+String(positionX)+","+String(positionY));
        lastPositionUpdate = millis();
    }


    if(wireComm.hasReceivedData()){
        String msg = wireComm.getReceivedData();
        if(msg == "off"){
            turnOffRobot();
        } else if (msg == "man"){
            switchToManualState();
        } else if (msg == "aut"){
            switchToAutomaticState();
        } else if (msg == "mz0"){
            isZAxisOut = false;
            if (currentState == automatic && yOnPosition && xOnPosition)
            {
                javaSerial.writeSerial("p" + String(nextX) + "," + String(nextY));
                if (item < 2)
                {
                    item++;
                    nextX = coordinates[item][0];
                    nextY = coordinates[item][1];
                } else {
                    nextX = 0;
                    nextY = 0;
                    javaSerial.writeSerial("oc");
                }
                xOnPosition = false;
                yOnPosition = false;
                zPositionSent = false;
            }
            
        } else if (msg == "mz1"){
            isZAxisOut = true;
            positionYPickup = positionY;
        } else if(msg == "my0h") {
            tYSwitch = false;
        } else if(msg == "my1h") {
            tYSwitch = true;
        } else if(msg == "my0l") {
            bYSwitch = false;
        } else if(msg == "my1l") {
            bYSwitch = true; 
        } else if(msg == "yu") {
            moveYUp = true;
        } else if(msg.startsWith("py")) {
            positionY = msg.substring(2).toInt();
        }
        wireComm.setHasReceivedData(false);
    }
    
    checkSerial();
}

void handleRobotState(){
    switch (currentState){
        case automatic:
            if(isResetButtonPressed()){
                switchToManualState();
            }
            handleAutomaticMode();
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
    javaSerial.writeSerial("sr");
}

void turnOffRobot(){
    if (currentState != off) {
        currentState = off;
        x_axisMotor.setManualPower(0);
        y_axisMotor.setManualPower(0);
        wireComm.sendData("off");
        javaSerial.writeSerial("sr");
    }
}

void switchToManualState(){
    currentState = manual;
    wireComm.sendData("man");
    javaSerial.writeSerial("so");
}

void switchToAutomaticState(){
    currentState = automatic;
    zPositionSent = false;
    xOnPosition = false;
    yOnPosition = false;
    wireComm.sendData("aut");
    javaSerial.writeSerial("sg");
}

void switchToCallibration(){
    currentState = callibrating;
    wireComm.sendData("cal");
    javaSerial.writeSerial("sb");
}

void handleManualInput(){
    lInduction = digitalRead(lInductiveSensor);
    rInduction = digitalRead(rInductiveSensor);
    if (!rInduction) {
        positionX = 0;
    }
    
    if(!isZAxisOut){
        int xValue = joystick.readXAxis();
        if((xValue < 0 && lInduction) || (xValue > 0 && rInduction) || xValue == 0){
            x_axisMotor.setManualPower(xValue);
        } else {
            x_axisMotor.setManualPower(0);
        }
        int yValue = joystick.readYAxis();
        if((yValue < 0 && tYSwitch)){
            y_axisMotor.setManualPower(yValue);
        } else if (yValue > 0 && bYSwitch) {
            y_axisMotor.setManualPower(yValue);
            yAxisBraking = true;
            yAxisBrakingTime = millis();
        } else {
            if (yAxisBraking)
            {
                y_axisMotor.setManualPower(-255);
                if (millis() - yAxisBrakingTime > 40)
                {
                    yAxisBraking = false;
                    y_axisMotor.setManualPower(0);
                }
                
            } else {
                y_axisMotor.setManualPower(0);
            }
        }
    } else {
        int yValue = joystick.readYAxis();
        if ((yValue < 0 && tYSwitch) && (positionY - positionYPickup < allowedYMovementUp))
        {
            y_axisMotor.setManualPower(yValue);
        } else if ((yValue > 0 && bYSwitch) && (positionY - positionYPickup > allowedYMovementDown))
        {
            y_axisMotor.setManualPower(yValue);
            yAxisBraking = true;
            yAxisBrakingTime = millis();
        } else {
            if (yAxisBraking)
            {
                y_axisMotor.setManualPower(-255);
                if (millis() - yAxisBrakingTime > 40)
                {
                    yAxisBraking = false;
                    y_axisMotor.setManualPower(0);
                }
                
            } else {
                y_axisMotor.setManualPower(0);
            }
        }
        x_axisMotor.setManualPower(0);
    }
}

void handleAutomaticMode() {
    lInduction = digitalRead(lInductiveSensor);
    rInduction = digitalRead(rInductiveSensor);
    if (!rInduction) {
        positionX = 0;
    }   
        
    if (xOnPosition && yOnPosition) {
        x_axisMotor.setManualPower(0);

        if (moveYUp && nextY != 0)
        {
            y_axisMotor.setManualPower(-255);
            if (positionY - positionYPickup > allowedYMovementUp) {
                moveYUp = false;
                y_axisMotor.setManualPower(0);
                wireComm.sendData("z0");
            }
            
        } else {
        y_axisMotor.setManualPower(0);
        }
        if (!zPositionSent && (nextX != 0 && nextY != 0))
        {
            wireComm.sendData("z" + String(zPositions[item]));
            zPositionSent = true;
        }
        
    } else if (!isZAxisOut) {
        if ((positionX < xPositions[nextX] - 5 && lInduction) && nextX != 0) {
            x_axisMotor.setManualPower(-255);
        } else if ((positionX > xPositions[nextX] + 10 && rInduction) && nextX != 0) {
            x_axisMotor.setManualPower(255);
        } else if (nextX == 0 && rInduction){
            x_axisMotor.setManualPower(255);
        } else {
            xOnPosition = true;
            x_axisMotor.setManualPower(0);
        }

        if ((positionY < yPositions[nextY] - 10 && tYSwitch) && nextY != 0) {
            y_axisMotor.setManualPower(-255);
        } else if ((positionY > yPositions[nextY] + 5 && bYSwitch) && nextY != 0) {
            y_axisMotor.setManualPower(255);
            yAxisBraking = true;
            yAxisBrakingTime = millis();
        } else if (nextY == 0 && bYSwitch) {
            y_axisMotor.setManualPower(255);
        } else {
            if (yAxisBraking)
            {
                y_axisMotor.setManualPower(-255);
                if (millis() - yAxisBrakingTime > 40)
                {
                    yAxisBraking = false;
                    y_axisMotor.setManualPower(0);
                    yOnPosition = true;
                }
                
            } else {
                y_axisMotor.setManualPower(0);
                yOnPosition = true;
            }
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
  if (digitalRead(rotaryPinXb)) {
    positionX--;
  } else {
    positionX++;
  }
}

void callibrateMotor() {
    if (!isZAxisOut) {
        if (digitalRead(rInductiveSensor)) {
            x_axisMotor.setManualPower(255);
        } else {
            x_axisMotor.setManualPower(0);
            positionX = 0;
        }
        if (bYSwitch) {
            y_axisMotor.setManualPower(255);
        } else {
            y_axisMotor.setManualPower(0);
        }

        if (!digitalRead(rInductiveSensor) && !bYSwitch)
        {
            positionX = 0;
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
            coordinates[0][0] = msg[5] - '0';
            coordinates[0][1] = msg[7] - '0';
            coordinates[1][0] = msg[9] - '0';
            coordinates[1][1] = msg[11] - '0';
            coordinates[2][0] = msg[13] - '0';
            coordinates[2][1] = msg[15] - '0';

            nextX = msg[5] - '0';
            nextY = msg[7] - '0';
            lastComCheckTime = millis();
            xOnPosition = false;
            yOnPosition = false;
            item = 0;
            javaSerial.writeSerial("bo");
            wireComm.sendData("sr");
        }
    }
}