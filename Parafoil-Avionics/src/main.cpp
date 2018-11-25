#include "Avionics.h"
#include "Log.h"
#include "Motor.h"
#include "Sensors.h"
#include "Constants.h"

AdjustableLowpass lowpassfilter;

void filterstuff(){
  float filteredvalue = lowpassfilter.update(0.5);
  float hello = filteredvalue+2;
}

int main(void){

/*************************************BOOT*************************************/
  Avionics avionics;

  IntervalTimer timer;
  float F0 = 20;
  float Q = 0.5;
  float Fs = 15;

  lowpassfilter.setQ(Q);
  lowpassfilter.setCorner(F0);
  lowpassfilter.setSampleRate(Fs);

  int FILTER_FREQ = 100;

  timer.begin(filterstuff, 1000000/FILTER_FREQ);


  avionics.initialize(); //Initialize all sensors, SD card, motors, etc.

/*************************************MAIN*************************************/
  while(true){
    avionics.record();
    // avionics.cutdown();
    // avionics.fly(EncA, EncB);
    avionics.smartSleep(50);
  }

  return 0;
}
