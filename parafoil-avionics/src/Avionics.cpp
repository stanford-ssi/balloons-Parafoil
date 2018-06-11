#include "Avionics.h"

bool trig;


void Avionics::initialize(){

  Serial.begin(9600);
  delay(5000);
  sensors.initializeSensors();
  sdcard.initializeSD(sensors);

  motorA.initialize(MOTOR_A_DIR_1, MOTOR_A_DIR_2, MOTOR_A_SPEED, ENCODER_A_1, ENCODER_A_2);
  motorB.initialize(MOTOR_B_DIR_1, MOTOR_B_DIR_2, MOTOR_B_SPEED, ENCODER_B_1, ENCODER_B_2);


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
	while (! motorA.update() ){
	Serial.print("1A: ");
	Serial.print(motorA.Enc.read() );
	Serial.print(" 1B: ");
	Serial.println(motorB.Enc.read() );
	}

	motorB.set_position(10000);
	while (! motorB.update() ){
	Serial.print("2A: ");
	Serial.print(motorA.Enc.read() );
	Serial.print(" 2B: ");
	Serial.println(motorB.Enc.read() );
	}


	motorA.set_position(0);
	while (! motorA.update() ){
	Serial.print("3A: ");
	Serial.print(motorA.Enc.read() );
	Serial.print(" 3B: ");
	Serial.println(motorB.Enc.read() );
	}

	motorB.set_position(0);
	while (! motorB.update() ){
	Serial.print("4A: ");
	Serial.print(motorA.Enc.read() );
	Serial.print(" 4B: ");
	Serial.println(motorB.Enc.read() );
	}



}

void Avionics::smartSleep(unsigned long ms) {
  sensors.smartDelay(ms);
}
