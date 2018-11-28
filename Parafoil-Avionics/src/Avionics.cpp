#include "Avionics.h"


void Avionics::initialize(){
  //SENSOR SHIT
  // Serial.begin(9600); //Begin serial connection for serial port
  // delay(5000); //Delay for serial monitor
  // sensors.initializeSensors(); //Initialize sensors
  // sdcard.initializeSD(sensors); //Initialize SD Card

  //CUTDOWN SHIT
  // digitalWrite(LED_PIN,LOW); //Initialize LED
  // pinMode(WIRE,OUTPUT); //Initialize nichrome wire cutdown
  // digitalWrite(WIRE,LOW);
  // trig = false;

  //FLIGHT SHIT
  Encoder *EncA = new Encoder(13,15);
  Encoder *EncB = new Encoder(15, 12);
  motorA.initialize(EncA, MOTOR_A_DIR_1, MOTOR_A_DIR_2, MOTOR_A_SPEED);
  motorB.initialize(EncB, MOTOR_B_DIR_1, MOTOR_B_DIR_2, MOTOR_B_SPEED);
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

void Avionics::forwardFlight(){
	motorA.set_position(0);
	motorB.set_position(0);
}

void Avionics::bankLeft(){
	motorA.set_position(10000);
	motorB.set_position(0);
}

void Avionics::bankRight(){
	motorA.set_position(0);
	motorB.set_position(10000);
}

void Avionics::fly(){
    /* Serial.println("break in fly"); */


	motorA.set_position(10000);
	while (! motorA.update() ){
	Serial.print("1A: ");
	Serial.print(motorA.Enc->read() );
	Serial.print(" 1B: ");
	Serial.println(motorB.Enc->read() );
	}

	motorB.set_position(10000);
	while (! motorB.update() ){
	Serial.print("2A: ");
	Serial.print(motorA.Enc->read() );
	Serial.print(" 2B: ");
	Serial.println(motorB.Enc->read() );
	}


	motorA.set_position(0);
	while (! motorA.update() ){
	Serial.print("3A: ");
	Serial.print(motorA.Enc->read() );
	Serial.print(" 3B: ");
	Serial.println(motorB.Enc->read() );
	}

	motorB.set_position(0);
	while (! motorB.update() ){
	Serial.print("4A: ");
	Serial.print(motorA.Enc->read() );
	Serial.print(" 4B: ");
	Serial.println(motorB.Enc->read() );
	}



   /* motors.performScriptedFlight(motorA.Enc,motorB.Enc); */
}

void Avionics::smartSleep(unsigned long ms) {
  sensors.smartDelay(ms);
}
