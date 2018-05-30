
#ifndef MOTOR_H
#define MOTOR_H

#include "Constants.h"

class Motor{

public:
  enum Direction { CW, NEUTRAL, CCW };
  Direction dir;

  void initializeMotors();
  void setDirection(Direction dir);
  void performScriptedFlight(Encoder& EncA, Encoder& EncB);
  int comparePositions(long currentPos, long setPoint) ;
  void forwardFlight(long loopTime, long currentPosA, long currentPosB);
  void bankLeft(long loopTime, long currentPosA);
  void bankRight(long loopTime, long currentPosB);

private:



  int counter = 0;

};

#endif
