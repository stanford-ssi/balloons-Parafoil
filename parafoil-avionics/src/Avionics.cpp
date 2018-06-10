#include "Avionics.h"

bool trig;

Encoder EncA(ENCODER_A_1, ENCODER_A_2);
Encoder EncB(ENCODER_B_1, ENCODER_B_2);

void Avionics::initialize(){


  Serial.begin(9600);
  delay(5000);
  sensors.initializeSensors();
  sdcard.initializeSD(sensors);

  motorA.initialize(&EncA, MOTOR_A_DIR_1, MOTOR_A_DIR_2, MOTOR_A_SPEED);
  motorB.initialize(&EncB, MOTOR_B_DIR_1, MOTOR_B_DIR_2, MOTOR_B_SPEED);


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
	Serial.print(motorA.Enc->read() );
	Serial.print(" B: ");
	Serial.println(motorB.Enc->read() );
	

	motorA.set_position(10000);
	while (! motorA.update() ){
	Serial.print("A: ");
	Serial.print(motorA.Enc->read() );
	Serial.print(" B: ");
	Serial.println(motorB.Enc->read() );
	}

	motorB.set_position(10000);
	while (! motorB.update() ){
	Serial.print("A: ");
	Serial.print(motorA.Enc->read() );
	Serial.print(" B: ");
	Serial.println(motorB.Enc->read() );
	}


	motorA.set_position(0);
	while (! motorA.update() ){
	Serial.print("A: ");
	Serial.print(motorA.Enc->read() );
	Serial.print(" B: ");
	Serial.println(motorB.Enc->read() );
	}

	motorB.set_position(0);
	while (! motorB.update() ){
	Serial.print("A: ");
	Serial.print(motorA.Enc->read() );
	Serial.print(" B: ");
	Serial.println(motorB.Enc->read() );
	}



   /* motors.performScriptedFlight(motorA.Enc,motorB.Enc); */
}

void Avionics::smartSleep(unsigned long ms) {
  sensors.smartDelay(ms);
}
