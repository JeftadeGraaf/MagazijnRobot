#ifndef JAVASERIAL_H
#define JAVASERIAL_H

class JavaSerial
{

public:

    JavaSerial();
    String readSerial();
    void writeSerial(String message);
    bool messageAvailable();
};
#endif