#include "Log.h"

int main(void){
  Serial.begin(9600);
  Sensors sensors;
  Log sdcard;

  sensors.initializeSensors();
  sdcard.initializeSD();

  while(true){
    sensors.readAllSensors();
    sdcard.writeSD(sensors);
  }
  return 0;
}
