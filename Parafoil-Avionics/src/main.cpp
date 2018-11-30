#include "Avionics.h"
#include "Log.h"
#include "Motor.h"
#include "Sensors.h"
#include "Constants.h"



int main(void){
/*************************************BOOT*************************************/
  Avionics avionics;
  avionics.initialize(); //Initialize all sensors, SD card, motors, etc.

  avionics.setTrigState(true); //COMMENT OUT FOR ACTUAL LAUNCH

//  long start = 0;
/*************************************MAIN*************************************/
  while(true){
//    Serial.println("hello");
    avionics.record();
  //  avionics.cutdown();
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
