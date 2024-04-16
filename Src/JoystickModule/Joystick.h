#ifndef JOYSTICK_H
#define JOYSTICK_H

class Joystick
{

public:

    Joystick(int xPin, int yPin);
    void registerPins();
    int readXAxis();
    int readYAxis();


private:
    int xPin;
    int yPin;
};

#endif // JOYSTICK_H