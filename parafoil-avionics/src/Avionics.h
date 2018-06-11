#ifndef AVIONICS_H
#define AVIONICS_H

#include "Sensors.h"
#include "Log.h"
#include "Motor.h"
#include "Constants.h"

class Avionics {

public:
  Avionics(): motorA(ENCODER_A_1,ENCODER_A_2), motorB(ENCODER_B_1,ENCODER_B_2) {} 
  void initialize();
  void record();
  void cutdown();
  void fly();
  void smartSleep(unsigned long ms);


private:

  bool release = false;
  long applyheat = 0;
  /********************OBJECTS*************************/
  Sensors sensors;
  Log sdcard;

  Motor motorA;
  Motor motorB;
};

#endif
