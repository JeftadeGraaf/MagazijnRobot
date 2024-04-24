#include "WireComm.h"
#include <Wire.h>

// to use these variables as static they cannot be empty
String WireComm::data = "";
bool WireComm::dataReceived = false;

WireComm::WireComm(int ownAddres, int slaveAddress) {
  this-> ownAddress = ownAddres;
  this-> slaveAddress = slaveAddress;
  this-> dataReceived = false;
  Wire.begin(ownAddres);
  Wire.onReceive(WireComm::receiveEvent);
}

void WireComm::sendData(String data) {
  Wire.beginTransmission(slaveAddress);
  Wire.write(data.c_str());
  Wire.endTransmission();
}

void WireComm::receiveEvent(int bytes) {
  String msg = "";
  while (Wire.available() > 0) {
    msg = msg + char(Wire.read());
  }
  data = msg;
  dataReceived = true;
}

bool WireComm::hasReceivedData(){
  return dataReceived;
}

String WireComm::getReceivedData(){
  return data;
}