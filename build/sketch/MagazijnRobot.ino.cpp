#include <Arduino.h>
#line 1 "C:\\Users\\mikes\\Desktop\\Windesheim\\Semester 2\\KBS\\MagazijnRobot\\MagazijnRobot.ino"
#line 1 "C:\\Users\\mikes\\Desktop\\Windesheim\\Semester 2\\KBS\\MagazijnRobot\\MagazijnRobot.ino"
void setup();
#line 6 "C:\\Users\\mikes\\Desktop\\Windesheim\\Semester 2\\KBS\\MagazijnRobot\\MagazijnRobot.ino"
void loop();
#line 1 "C:\\Users\\mikes\\Desktop\\Windesheim\\Semester 2\\KBS\\MagazijnRobot\\MagazijnRobot.ino"
void setup()
{
	Serial.begin(9600);
}

void loop()
{
	Serial.write("Hello");
}
