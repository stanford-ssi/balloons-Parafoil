#include "Avionics.h"


void Avionics::initialize(){
  Serial.begin(9600);
  delay(5000);
  Serial.println("Hello world");
  sensors.initializeSensors();
  Serial.println("sensors done");
  sdcard.initializeSD(sensors);
  Serial.println("sd done");

  pinMode(WIRE, OUTPUT);
  digitalWrite(WIRE,LOW); //NICHROME WIRE SHOULD BE OFF FIRST
  //receiver.initializeReceiver();



}
void Avionics::record(){
  Serial.println("pre read");
  sensors.readAllSensors();
  Serial.println("post read");
  sdcard.writeSD(sensors);
  Serial.println("write sd)");

}

void Avionics::cutdown(){

  if(sensors.getAlt() > CUTDOWN_ALT ){
    release = true;
    applyheat = millis();
    digitalWrite(WIRE,HIGH); //turn on nichrome
  }

  if(release){
    if(millis() - applyheat > RELEASE_TIME * 1000){
      digitalWrite(WIRE,LOW);
      release = false;
    }
  }
}


void Avionics::actuate(volatile int aileronPWM, long startTime, volatile boolean newSignal){
  receiver.moveServo(aileronPWM, startTime, newSignal);
}
