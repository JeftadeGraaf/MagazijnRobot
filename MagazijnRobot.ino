#include <Arduino.h>
#include "Src/JoystickModule/Joystick.h"
#include "Src/MotorModule/Motor.h"

Joystick joystick = Joystick(A0, A1, 30);
Motor x_axisMotor = Motor(3, 12);
Motor y_axisMotor = Motor(11, 13);

void setup()
{
    Serial.begin(9600);
    joystick.registerPins();
    x_axisMotor.registerPins();
    y_axisMotor.registerPins();
}

void loop()
{
}