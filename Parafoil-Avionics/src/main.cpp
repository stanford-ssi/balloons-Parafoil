#include "Avionics.h"
#include "Log.h"
#include "Motor.h"
#include "Sensors.h"
#include "Constants.h"



int main(void){

/*************************************BOOT*************************************/
  Avionics avionics;





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
