#include "Avionics.h"
#include "Log.h"
#include "Motor.h"
#include "Sensors.h"
#include "Constants.h"



int main(void){
  // pinMode(18,OUTPUT);
  // pinMode(19,OUTPUT);
  // pinMode(20,OUTPUT);

/*************************************BOOT*************************************/
  Avionics avionics;

  avionics.initialize(); //Initialize all sensors, SD card, motors, etc.
  avionics.setTrigState(true);
  long start = 0;

/*************************************MAIN*************************************/
  while(true){
    // digitalWrite(18,HIGH);
    // digitalWrite(20,LOW);
    // analogWrite(19,200);

   // avionics.record();
    //avionics.cutdown();
    if (avionics.getTrigState()){
      if(start == -1){
        start = millis();
      }
      else{
        avionics.fly(start);
      }

      //Serial.println("HELLO");
    }
    //avionics.fly();
    avionics.smartSleep(50);
  }

  return 0;
}
