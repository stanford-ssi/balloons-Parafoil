#include "Avionics.h"
#include "Log.h"
#include "Motor.h"
#include "Sensors.h"
#include "Constants.h"



int main(void){

  Avionics avionics = Avionics();

  

  while(true){
    /* avionics.record(); */
    /* avionics.cutdown(); */
    Serial.println("break before fly");
    avionics.fly();
    avionics.smartSleep(50);
  }

  return 0;
}
