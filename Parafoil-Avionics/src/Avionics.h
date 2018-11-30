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


  void fly(long start);
  void forwardFlight();
  void bankLeft();
  void bankRight();

  bool getTrigState();
  void setTrigState(bool state);


  void smartSleep(unsigned long ms);


private:

  /******************************CUTDOWN***************************************/
  bool release = false;
  long applyheat = 0;
  bool trig = false;
  bool iffly = false;

  int state = -1;

  /******************************OBJECTS***************************************/
  Sensors sensors;
  Log sdcard;

  Motor motor1;
  Motor motor2;

};

#endif
