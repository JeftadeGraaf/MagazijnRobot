#ifndef JOYSTICK_H
#define JOYSTICK_H

class Joystick
{

public:

    Joystick(int xPin, int yPin, int deadzone);
    void registerPins();
    int readXAxis();
    int readYAxis();


private:
    int xPin;
    int yPin;
    int deadzone;
};

#endif // JOYSTICK_H