#include "Avionics.h"
#include "Log.h"
#include "Receiver.h"
#include "Sensors.h"
#include "Constants.h"



int main(void){

  Avionics avionics;

  avionics.initialize();
  while(true){
    avionics.record();
    avionics.cutdown();
    avionics.smartSleep(50);
  }

  return 0;
}
