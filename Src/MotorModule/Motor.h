#ifndef MOTOR_H
#define MOTOR_H

class Motor
{

public:
    Motor(int pwmPin, int directionPin, int brakePin);
    void registerPins();
    void setManualPower(int axisValue);
    
private:
    int pwmPin;
    int directionPin;
    int brakePin;
};


#endif // MOTOR_H