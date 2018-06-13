#include "Avionics.h"

bool trig;


Avionics::Avionics(): motorA(MOTOR_A_DIR_1, MOTOR_A_DIR_2, MOTOR_A_SPEED, ENCODER_A_1, ENCODER_A_2), 
		      motorB(MOTOR_B_DIR_1, MOTOR_B_DIR_2, MOTOR_B_SPEED, ENCODER_B_1, ENCODER_B_2) { 

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

void Avionics::fly(){

	motorA.set_position(10000);
	smartSleep(3000);

	motorB.set_position(10000);
	smartSleep(3000);

	motorA.set_position(0);
	smartSleep(3000);

	motorB.set_position(0);
	smartSleep(3000);
}

void Avionics::smartSleep(unsigned long ms) {
  unsigned long start = millis();
  unsigned long lap = millis();
  do {
	  sensors.gpsUpdate();
	  motorA.update();
	  motorB.update();

	  if( (millis() - lap) > 50){
		  lap = millis();
		  Serial.println("new 20Hz loop");
		  record();
		  /* cutdown(); */

		  Serial.print("A: ");
		  Serial.print(motorA.Enc.read() );
		  Serial.print("B: ");
		  Serial.println(motorB.Enc.read() );

	  }
  } while ( (millis() - start) < ms);

}
