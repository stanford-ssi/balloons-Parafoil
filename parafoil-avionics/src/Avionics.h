#ifndef AVIONICS_H
#define AVIONICS_H

#include "Sensors.h"
#include "Log.h"
#include "Motor.h"
#include "Constants.h"

class Avionics {

public:
  void initialize();
  void record();
  void cutdown();
  void fly();
  void smartSleep(unsigned long ms);

  void forwardFlight();
  void bankLeft();
  void bankRight();
  

private:
  int inited = 0;

  bool release = false;
  long applyheat = 0;
  /********************OBJECTS*************************/
  Sensors sensors;
  Log sdcard;

  Motor motorA;
  Motor motorB;
};

#endif
