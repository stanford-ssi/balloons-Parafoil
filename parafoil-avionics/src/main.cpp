#include "Avionics.h"
#include "Log.h"
#include "Motor.h"
#include "Sensors.h"
#include "Constants.h"



int main(void){

  Avionics avionics = Avionics();

  while(true){
    avionics.fly();
  }

  return 0;
}
