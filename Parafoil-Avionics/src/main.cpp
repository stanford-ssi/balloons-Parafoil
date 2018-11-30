#include "Avionics.h"
#include "Log.h"
#include "Motor.h"
#include "Sensors.h"
#include "Constants.h"



int main(void){
/*************************************BOOT*************************************/
  Avionics avionics;
  avionics.initialize(); //Initialize all sensors, SD card, motors, etc.
  avionics.setTrigState(true);
  long start = 0;
/*************************************MAIN*************************************/
  while(true){
    Serial.println("hello");
    avionics.record();
  //  avionics.cutdown();
    if (avionics.getTrigState()){
      if(start == -1){
        start = millis();
      }
      else{
        avionics.fly(start);
      }
    }
    avionics.smartSleep(50);
  }

  return 0;
}
