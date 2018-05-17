#include "Avionics.h"
#include "Log.h"
#include "Receiver.h"
#include "Sensors.h"
#include "Constants.h"



int main(void){

  Avionics avionics;

  avionics.initialize();

  while(true){
    Serial.println("loop start");
    avionics.record();
    avionics.cutdown();

  }
  return 0;
}
