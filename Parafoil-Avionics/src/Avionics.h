#ifndef AVIONICS_H
#define AVIONICS_H

#include "Sensors.h"
#include "Log.h"
#include "Motor.h"
#include "Constants.h"
#include "Utils.h"

class Avionics {

public:
  void initialize();
  void record();
  void cutdown();
//  void fly();
  void smartSleep(unsigned long ms);


private:

  bool release = false;
  long applyheat = 0;
  bool trig = false;

  /******************************OBJECTS***************************************/
  Sensors sensors;
  Log sdcard;
  // Motor motors;
  // Encoder EncA(ENCODER_A_1, ENCODER_A_2);
  // Encoder EncB(ENCODER_B_1, ENCODER_B_2);

};

#endif
