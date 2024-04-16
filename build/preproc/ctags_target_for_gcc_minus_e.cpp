# 1 "C:\\Users\\mikes\\Desktop\\Windesheim\\Semester 2\\KBS\\MagazijnRobot\\MagazijnRobot.ino"
# 2 "C:\\Users\\mikes\\Desktop\\Windesheim\\Semester 2\\KBS\\MagazijnRobot\\MagazijnRobot.ino" 2
# 3 "C:\\Users\\mikes\\Desktop\\Windesheim\\Semester 2\\KBS\\MagazijnRobot\\MagazijnRobot.ino" 2

Joystick joystick = Joystick(A0, A1);

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
