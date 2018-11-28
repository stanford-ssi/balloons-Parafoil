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
  void forwardFlight();
  void bankLeft();
  void bankRight();


  void smartSleep(unsigned long ms);


private:

  /******************************CUTDOWN***************************************/
  bool release = false;
  long applyheat = 0;
  bool trig = false;

  /******************************OBJECTS***************************************/
  Sensors sensors;
  Log sdcard;

  Motor motorA;
  Motor motorB;

};

#endif
