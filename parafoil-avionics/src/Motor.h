
#ifndef MOTOR_H
#define MOTOR_H

#include "Constants.h"
#include "Sensors.h"

class Motor{

public:

  enum Direction { CW, NEUTRAL, CCW };
  Motor(int dir1_pin, int dir2_pin, int speed_pin, int enc1_pin, int enc2_pin);

  void set_position(int pos);
  int update();

  Encoder Enc;
  int speed;
private:

  Direction dir;
  int target = 0;

  int dir1_pin;
  int dir2_pin;
  int speed_pin;


  int counter = 1;

};

#endif
