#include <Arduino.h>
#include "Src/JoystickModule/Joystick.h"

Joystick joystick = Joystick(A0, A1);

void setup()
{
    Serial.begin(9600);
    joystick.registerPins();
}

void loop()
{
}