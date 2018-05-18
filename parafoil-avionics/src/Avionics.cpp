#include "Avionics.h"

bool trig;

void Avionics::initialize(){
  Serial.begin(9600);
  delay(5000);
  sensors.initializeSensors();
  sdcard.initializeSD(sensors);

  pinMode(WIRE,OUTPUT);
  digitalWrite(WIRE,LOW);
  //receiver.initializeReceiver();
  trig = false;

}

void Avionics::record(){

  //sensors.readAllSensors();
  sdcard.writeSD(sensors);
}

void Avionics::cutdown(){

  if(!trig && ( sensors.getALt() > CUTDOWN_ALT) && (release == false)){
    pinMode(WIRE, INPUT_PULLUP); //turn on nichrome
    release = true;
    applyheat = millis();
    Serial.println("START CUTDOWN");
    trig = true;
  }

  if(release){
    if(millis() - applyheat > RELEASE_TIME * 1000){

      pinMode(WIRE,OUTPUT); //turn off nichrome wire
      digitalWrite(WIRE,LOW);

      Serial.println("END CUTDOWN");

      release = false;
    }
  }
}


void Avionics::actuate(volatile int aileronPWM, long startTime, volatile boolean newSignal){
  receiver.moveServo(aileronPWM, startTime, newSignal);
}
