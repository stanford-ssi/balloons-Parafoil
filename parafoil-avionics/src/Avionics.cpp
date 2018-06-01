#include "Avionics.h"

bool trig;

void Avionics::initialize(Encoder& EncA, Encoder& EncB){
  Serial.begin(9600);
  delay(5000);
  sensors.initializeSensors();
  sdcard.initializeSD(sensors);
  motors.initializeMotors();

  EncA.write(NEUTRAL);
  EncB.write(NEUTRAL);

  digitalWrite(LED_PIN,LOW);
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

  if(!trig && ( sensors.getAlt() > CUTDOWN_ALT ) && (release == false) ){
    digitalWrite(WIRE,HIGH); //turn on nichrome
    release = true;
    applyheat = millis();
    Serial.println("START CUTDOWN");
    digitalWrite(LED_PIN, HIGH);
    trig = true;
  }

  if(release){
    if(millis() - applyheat > RELEASE_TIME * 1000){
      // pinMode(WIRE,OUTPUT); //turn off nichrome wire
      digitalWrite(WIRE,LOW);
      Serial.println("END CUTDOWN");
      digitalWrite(LED_PIN, LOW);
      release = false;
    }
  }
}

void Avionics::fly(Encoder& EncA, Encoder& EncB){
   motors.performScriptedFlight(EncA,EncB);
}

void Avionics::smartSleep(unsigned long ms) {
  sensors.smartDelay(ms);
}
