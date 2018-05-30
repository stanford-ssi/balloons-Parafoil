
#ifndef MOTOR_H
#define MOTOR_H

#include "Constants.h"
#include <PWMServo.h>

class Motor{

public:

  void initializeMotors();
  void steerLeft();
  void steerRight();


private:
  PWMServo servo1;
  PWMServo servo2;

};

#endif
