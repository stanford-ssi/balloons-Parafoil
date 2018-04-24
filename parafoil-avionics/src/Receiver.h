
#ifndef RECEIVER_H
#define RECEIVER_H

#include "Constants.h"
#include <PWMServo.h>

class Receiver{

public:

  void initializeReceiver();
  void calcPWM();
  void moveServo(volatile int aileronPWM, long startTime, volatile boolean newSignal);


private:
  PWMServo servo1;
  PWMServo servo2;

};

#endif
