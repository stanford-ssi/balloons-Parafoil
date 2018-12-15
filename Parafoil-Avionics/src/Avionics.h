/*
  Stanford Student Space Initiative
  Balloons | BALLOONERANG | DECEMBER 2018
  File: Avionics.h
  --------------------------
  Interface for physical actuators, SD card logging, and sensors
*/

#ifndef AVIONICS_H
#define AVIONICS_H

#include "Sensors.h"
#include "Logger.h"
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

  long getStart();
  void setStart(long set);

  long getPos1();
  long getPos2();

  void smartSleep(unsigned long ms);

private:
  /******************************CUTDOWN***************************************/
  long applyheat = 0; //Apply heat until cutdown
  bool trig = false; //Payload cutdown state
  int state = -1; //Current flight maneuver
  long start = -1; //Used to coordinate flight maneuvers
  long timer = -1; //Timer for cutdown

  /******************************OBJECTS***************************************/
  Sensors sensors;
  Logger log;
  Motor motor1;
  Motor motor2;
};

#endif
