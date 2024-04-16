#ifndef MOTOR_H
#define MOTOR_H

class Motor
{

public:
    Motor(int pwmPin, int directionPin);
    void registerPins();
    void setManualPower(int axisValue);
    
private:
    int pwmPin;
    int directionPin;
};


#endif // MOTOR_H