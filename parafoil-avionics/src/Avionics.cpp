#include "Avionics.h"

bool trig;

void Avionics::initialize(){
  Serial.begin(9600);
  delay(5000);
  sensors.initializeSensors();
  sdcard.initializeSD(sensors);
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

  if(!trig && ( millis() > (30000) ) && (release == false) ){
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


void Avionics::actuate(volatile int aileronPWM, long startTime, volatile boolean newSignal){
  receiver.moveServo(aileronPWM, startTime, newSignal);
}
