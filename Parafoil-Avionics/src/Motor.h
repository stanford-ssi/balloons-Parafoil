
#ifndef MOTOR_H
#define MOTOR_H

#include "Constants.h"

class Motor{

public:
  enum Direction { CW, NEUTRAL, CCW };

  void initialize(Encoder* Enc, int dir1_pin, int dir2_pin, int speed_pin);
  void set_position(int pos);
  int update();

  int getEncPosition();

  void setDirection(Direction dir);
  void performScriptedFlight(Encoder& EncA, Encoder& EncB);
  int comparePositions(long currentPos, long setPoint) ;
  void forwardFlight(long loopTime, long currentPosA, long currentPosB);
  void bankLeft(long currentPosA);
  void bankRight(long currentPosB);

  Encoder* Enc;
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
