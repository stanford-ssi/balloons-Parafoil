
#ifndef MOTOR_H
#define MOTOR_H

#include "Constants.h"

class Motor{

public:
  enum Direction { CW, NEUTRAL, CCW };
  Direction dir;

  void initializeMotors( Encoder* EncA, Encoder* EncB);
  void set_A_position(int pos);
  void set_B_position(int pos);
  void update();

  void setDirection(Direction dir);
  void performScriptedFlight(Encoder& EncA, Encoder& EncB);
  int comparePositions(long currentPos, long setPoint) ;
  void forwardFlight(long loopTime, long currentPosA, long currentPosB);
  void bankLeft(long currentPosA);
  void bankRight(long currentPosB);

  Direction A_dir;
  Direction B_dir;


  Encoder* EncA;
  Encoder* EncB;

private:


  int A_target = 0;
  int B_target = 0;



  int counter = 1;

};

#endif
