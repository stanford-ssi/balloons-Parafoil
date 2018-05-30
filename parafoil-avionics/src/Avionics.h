#ifndef AVIONICS_H
#define AVIONICS_H

#include "Sensors.h"
#include "Log.h"
#include "Motor.h"
#include "Constants.h"

class Avionics {

public:
  void initialize(Encoder& EncA, Encoder& EncB);
  void record();
  void cutdown();
  void fly(Encoder& EncA, Encoder& EncB);
  void smartSleep(unsigned long ms);

private:

  bool release = false;
  long applyheat = 0;
  /********************OBJECTS*************************/
  Sensors sensors;
  Log sdcard;
  Motor motors;
};

#endif
