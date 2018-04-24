#include "Avionics.h"


void Avionics::initialize(){
  Serial.begin(9600);
  sensors.initializeSensors();
  sdcard.initializeSD(sensors);
  receiver.initializeReceiver();



}
void Avionics::record(){
  sensors.readAllSensors();
  sdcard.writeSD(sensors);

}


void Avionics::actuate(volatile int aileronPWM, long startTime, volatile boolean newSignal){
  receiver.moveServo(aileronPWM, startTime, newSignal);
}
