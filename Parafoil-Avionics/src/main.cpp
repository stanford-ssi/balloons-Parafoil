/*
  Stanford Student Space Initiative
  Balloons | BALLOONERANG | DECEMBER 2018
  File: main.cpp
  --------------------------
  Flight code for main BALLOONERANG avionics
*/

#include "Avionics.h"
#include "Logger.h"
#include "Motor.h"
#include "Sensors.h"
#include "Constants.h"

int main(void){
/*************************************BOOT*************************************/
  Avionics avionics;
  avionics.initialize(); //Initialize all sensors, SD card, motors, etc.
//  avionics.setTrigState(true); //COMMENT OUT FOR ACTUAL LAUNCH

/*************************************MAIN*************************************/
  while(true){
    avionics.record();
    avionics.cutdown();
    if (avionics.getTrigState()){
      if(avionics.getStart() == -1){
        avionics.setStart( millis() );
      }
      else{
        avionics.fly(avionics.getStart());
      }
    }
    avionics.smartSleep(50);
  }
  return 0;
}
