#include "Avionics.h"


void Avionics::initialize(){
  Serial.begin(9600);
  delay(5000);
  sensors.initializeSensors();
  sdcard.initializeSD(sensors);

  pinMode(WIRE, OUTPUT);
  digitalWrite(WIRE,LOW); //NICHROME WIRE SHOULD BE OFF FIRST
  //receiver.initializeReceiver();

}

void Avionics::record(){

  //sensors.readAllSensors();
  sdcard.writeSD(sensors);
}

void Avionics::cutdown(){

  if(sensors.getAlt() > CUTDOWN_ALT ){
    release = true;
    applyheat = millis();
    digitalWrite(WIRE,HIGH); //turn on nichrome
  }

  if(release){
    if(millis() - applyheat > RELEASE_TIME * 1000){
      digitalWrite(WIRE,LOW); //turn off nichrome wire
      release = false;
    }
  }
}


void Avionics::actuate(volatile int aileronPWM, long startTime, volatile boolean newSignal){
  receiver.moveServo(aileronPWM, startTime, newSignal);
}
