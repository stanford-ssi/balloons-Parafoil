#ifndef AVIONICS_H
#define AVIONICS_H

#include "Sensors.h"
#include "Log.h"
#include "Receiver.h"
#include "Constants.h"

class Avionics {

public:
  void initialize();
  void record();
  void actuate(volatile int aileronPWM, long startTime, volatile boolean newSignal);
  void cutdown();
  void smartSleep(unsigned long ms);

private:

  bool release = false;
  long applyheat = 0;
  /********************OBJECTS*************************/
  Sensors sensors;
  Receiver receiver;
  Log sdcard;
};

#endif
