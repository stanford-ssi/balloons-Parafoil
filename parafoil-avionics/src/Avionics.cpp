#include "Avionics.h"

bool trig;

void Avionics::initialize(Encoder* _EncA, Encoder* _EncB){


  EncA = _EncA;
  EncB = _EncB;

  Serial.begin(9600);
  delay(5000);
  sensors.initializeSensors();
  sdcard.initializeSD(sensors);
  motors.initializeMotors(EncA, EncB);

  EncA->write(NEUTRAL);
  EncB->write(NEUTRAL);

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
    /* Serial.println("break in fly"); */
	


	Serial.print("A: ");
	Serial.print(EncA->read() );
	Serial.print(" B: ");
	Serial.println(EncB->read() );
	

	motors.set_A_position(10000);
	while ( (motors.A_dir != NEUTRAL) && (motors.B_dir != NEUTRAL) ){
	Serial.print("A: ");
	Serial.print(EncA->read() );
	Serial.print(" B: ");
	Serial.println(EncB->read() );
	}

	motors.set_B_position(10000);
	while ( (motors.A_dir != NEUTRAL) && (motors.B_dir != NEUTRAL) ){
	Serial.print("A: ");
	Serial.print(EncA->read() );
	Serial.print(" B: ");
	Serial.println(EncB->read() );
	}


	motors.set_A_position(0);
	while ( (motors.A_dir != NEUTRAL) && (motors.B_dir != NEUTRAL) ){
	Serial.print("A: ");
	Serial.print(EncA->read() );
	Serial.print(" B: ");
	Serial.println(EncB->read() );
	}

	motors.set_B_position(0);
	while ( (motors.A_dir != NEUTRAL) && (motors.B_dir != NEUTRAL) ){
	Serial.print("A: ");
	Serial.print(EncA->read() );
	Serial.print(" B: ");
	Serial.println(EncB->read() );
	}



   /* motors.performScriptedFlight(EncA,EncB); */
}

void Avionics::smartSleep(unsigned long ms) {
  sensors.smartDelay(ms);
}
