#ifndef WireComm_h
#define WireComm_h

#include <Wire.h>

class WireComm {
  private:  
    int ownAddress;
    int slaveAddress;
    static String data;
    static bool dataReceived;

  public:
    WireComm(int ownAddres, int slaveAddress);
    void sendData(String data);
    static void receiveEvent(int bytes);
    bool hasReceivedData();
    String getReceivedData();
    void setHasReceivedData(bool value);
};

#endif