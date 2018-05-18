#include "Avionics.h"

bool trig;

void Avionics::initialize(){
  Serial.begin(9600);
  delay(5000);
  sensors.initializeSensors();
  sdcard.initializeSD(sensors);

  pinMode(WIRE, OUTPUT);
  digitalWrite(WIRE,LOW); //NICHROME WIRE SHOULD BE OFF FIRST
  //receiver.initializeReceiver();
  trig = false;

}

void Avionics::record(){

  //sensors.readAllSensors();
  sdcard.writeSD(sensors);
}

void Avionics::cutdown(){

  if(!trig && (sensors.getAlt() > CUTDOWN_ALT)
   && (release == false)){
  //if(sensors.getAlt() > CUTDOWN_ALT ){
    release = true;
    applyheat = millis();
    Serial.println("START CUTDOWN");
    digitalWrite(WIRE,HIGH); //turn on nichrome
    trig = true;
  }

  if(release){
    if(millis() - applyheat > RELEASE_TIME * 1000){
      digitalWrite(WIRE,LOW); //turn off nichrome wire
      Serial.println("END CUTDOWN");

      release = false;
    }
  }
}


void Avionics::actuate(volatile int aileronPWM, long startTime, volatile boolean newSignal){
  receiver.moveServo(aileronPWM, startTime, newSignal);
}
