#include "Receiver.h"

void Receiver::initializeReceiver(){
  pinMode(AILERON_IN, INPUT);

  servo1.attach(3);
  servo2.attach(4);
}



void Receiver::moveServo(volatile int aileronPWM, long startTime, volatile boolean newSignal){
  //FOR RC SIGNAL
  // if a new AILERON signal has been measured, lets print the value to serial, if not our code could carry on with some other processing
  if (newSignal) {
     int newPos = map(aileronPWM, MIN_AILERON, MAX_AILERON, -90, 90);
    // set this back to false when we have finished
    // with aileronPWM, while true, calcInput will not update
    // aileronPWM
    servo1.write(newPos);
    servo2.write(newPos);

    newSignal = false;
  }
}
