#line 1 "C:\\Users\\mikes\\Desktop\\Windesheim\\Semester 2\\KBS\\MagazijnRobot\\MagazijnRobot.ino"
#include <Arduino.h>
#include "Src/JoystickModule/Joystick.h"

Joystick joystick = Joystick(A0, A1);

#line 6 "C:\\Users\\mikes\\Desktop\\Windesheim\\Semester 2\\KBS\\MagazijnRobot\\MagazijnRobot.ino"
void setup();
#line 12 "C:\\Users\\mikes\\Desktop\\Windesheim\\Semester 2\\KBS\\MagazijnRobot\\MagazijnRobot.ino"
void loop();
#line 6 "C:\\Users\\mikes\\Desktop\\Windesheim\\Semester 2\\KBS\\MagazijnRobot\\MagazijnRobot.ino"
void setup()
{
    Serial.begin(9600);
    joystick.registerPins();
}

void loop()
{
    Serial.print("X: ");
    Serial.println(joystick.readXAxis());
    Serial.print("Y: ");
    Serial.println(joystick.readYAxis());
    delay(200);
}
