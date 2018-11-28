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
  Encoder *Enc1 = new Encoder(ENCODER_1_A,ENCODER_1_B);
  Encoder *Enc2 = new Encoder(ENCODER_2_A, ENCODER_2_B);
  motor1.initialize(Enc1, MOTOR_1_DIR_1, MOTOR_1_DIR_2, MOTOR_1_SPEED);
  motor2.initialize(Enc2, MOTOR_2_DIR_1, MOTOR_2_DIR_2, MOTOR_2_SPEED);


}


void Avionics::record(){
  sensors.readAllSensors();
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
  motor1.set_position(0);
	motor1.update();
	motor2.set_position(0);
  motor2.update();
}

void Avionics::bankLeft(){
//  Serial.println("BANKINGLEFT");
	motor1.set_position(TURN_RADIUS);
  motor1.update();
	motor2.set_position(0);
  motor2.update();
}

void Avionics::bankRight(){
  motor1.set_position(0);
  motor1.update();
	motor2.set_position(TURN_RADIUS);
  motor2.update();
}

void Avionics::fly(){


//  bankLeft();
  //bankRight();
  if( millis() < 4000) bankLeft();
  else if( millis() > 4000 && millis() < 8000){
    bankRight();
  }
  else if (millis() > 8000){
    forwardFlight();
  }

}

void Avionics::smartSleep(unsigned long ms) {
  sensors.smartDelay(ms);
}
