#include "Avionics.h"
#include "Log.h"
#include "Receiver.h"
#include "Sensors.h"
#include "Constants.h"

volatile int aileronPWM = NEUTRAL_AILERON;
volatile unsigned long startTime = 0;
volatile boolean newSignal = false;

void calcPWM();

int main(void){


  pinMode(AILERON_IN,INPUT);

  attachInterrupt(AILERON_IN,calcPWM ,CHANGE);
  Avionics avionics;

  avionics.initialize();

  while(true){
    avionics.record();
    avionics.actuate(aileronPWM,startTime,newSignal);
  }
  return 0;
}

void calcPWM(){
  // if the pin is high, its the start of an interrupt
  if (digitalRead(AILERON_IN) == HIGH){
    // get the time using micros - when our code gets really busy this will become inaccurate, but for the current application its
    // easy to understand and works very well
    startTime = micros();

  } else {

    // if the pin is low, its the falling edge of the pulse so now we can calculate the pulse duration by subtracting the
    // start time startTime from the current time returned by micros()
    if (startTime && !newSignal) {
      aileronPWM = (int)(micros() - startTime);
      startTime = 0;

      // tell loop we have a new signal on the AILERON channel
      // we will not update aileronPWM until loop sets
      // newSignal back to false
      newSignal = true;
    }
}
}
