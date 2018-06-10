#include "Avionics.h"
#include "Log.h"
#include "Motor.h"
#include "Sensors.h"
#include "Constants.h"



int main(void){

  Avionics avionics;
  Encoder EncA(ENCODER_A_1, ENCODER_A_2);
  Encoder EncB(ENCODER_B_1, ENCODER_B_2);

  avionics.initialize();

  

  while(true){
    /* avionics.record(); */
    /* avionics.cutdown(); */
    Serial.println("break before fly");
    avionics.fly();
    avionics.smartSleep(50);
  }

  return 0;
}
