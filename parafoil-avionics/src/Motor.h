#ifndef MOTOR_H
#define MOTOR_H

#include "Constants.h"
#include "Sensors.h"

class Motor{

public:
  //Motor Constructor (1 motor consists of direction pins,speed pin, and encoders)
  Motor(int dir1_pin, int dir2_pin, int speed_pin, int enc1_pin, int enc2_pin);
  enum Direction { CW, NEUTRAL, CCW };

  Encoder Enc; //Encoder object
  int speed;

  void set_position(int pos);
  void setDirection(Direction dir);
  int update();

private:

  Direction dir;
  int target = 0;

  int dir1_pin;
  int dir2_pin;
  int speed_pin;

  int counter = 1;

};

#endif
