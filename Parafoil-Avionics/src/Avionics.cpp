#include "Avionics.h"


void Avionics::initialize(){

  Serial.begin(9600); //Begin serial connection for serial port
  delay(5000); //Delay for serial monitor

  sensors.initializeSensors(); //Initialize sensors
  sdcard.initializeSD(sensors); //Initialize SD Card
  // motors.initializeMotors(); //Initialize motors
  //
  // EncA.write(NEUTRAL);
  // EncB.write(NEUTRAL);

  digitalWrite(LED_PIN,LOW); //Initialize LED
  pinMode(WIRE,OUTPUT); //Initialize nichrome wire cutdown
  digitalWrite(WIRE,LOW);

  trig = false; //Wire has not been turned on yet

}

void Avionics::record(){
  //sensors.readAllSensors();
  sdcard.writeSD(sensors);
}

void Avionics::cutdown(){

  if(!trig && ( sensors.getAlt() > CUTDOWN_ALT ) && (release == false) ){
    digitalWrite(WIRE,HIGH); //Turn on nichrome wire
    release = true; //Payload has been released
    applyheat = millis(); //Start time of hot wire
    Serial.println("START CUTDOWN");
    digitalWrite(LED_PIN, HIGH);
    trig = true;
  }

  if(release){
    if(millis() - applyheat > RELEASE_TIME * 1000){
      digitalWrite(WIRE,LOW); //Turn off nichrome wire
      Serial.println("END CUTDOWN");
      digitalWrite(LED_PIN, LOW);
      release = false;
    }
  }
}

// void Avionics::fly(Encoder& EncA, Encoder& EncB){
//     Serial.println("break in fly");
//    motors.performScriptedFlight(EncA,EncB);
// }

void Avionics::smartSleep(unsigned long ms) {
  sensors.smartDelay(ms);
}
