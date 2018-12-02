#include "Avionics.h"


void Avionics::initialize(){
  //SENSOR SHIT
  pinMode(WIRE, OUTPUT);
  digitalWrite(WIRE, LOW);
  Serial.begin(9600); //Begin serial connection for serial port
  delay(5000); //Delay for serial monitor
  sensors.initializeSensors(); //Initialize sensors
  sdcard.initializeSD(sensors); //Initialize SD Card

  //CUTDOWN SHIT
//  digitalWrite(LED_PIN,LOW); //Initialize LED
  pinMode(WIRE,OUTPUT); //Initialize nichrome wire cutdown
  digitalWrite(WIRE,LOW);
  trig = false;

  //FLIGHT SHIT
  Encoder *Enc1 = new Encoder(ENCODER_1_A,ENCODER_1_B);
  Encoder *Enc2 = new Encoder(ENCODER_2_A, ENCODER_2_B);
  motor1.initialize(Enc1, MOTOR_1_DIR_1, MOTOR_1_DIR_2, MOTOR_1_SPEED);
  motor2.initialize(Enc2, MOTOR_2_DIR_1, MOTOR_2_DIR_2, MOTOR_2_SPEED);


}


void Avionics::record(){
  sensors.readAllSensors();
//  state = ( (millis() - start)/TIME_STEP ) % 4;
  sdcard.writeSD(sensors, getStart(), getPos1(), getPos2());
}


void Avionics::cutdown(){

  if(!trig && ( sensors.getAlt() > CUTDOWN_ALT )) {
    if(timer == -1){
      timer = millis();
    }
  //  timer = millis();
    if(millis() - timer > TIMER_DELAY && timer != -1){
      digitalWrite(WIRE,HIGH);
      applyheat = millis(); //Start time of hot wire
      Serial.println("START CUTDOWN");
    //  digitalWrite(LED_PIN, HIGH);
      trig = true;
    }
  //  digitalWrite(WIRE,HIGH); //Turn on nichrome wire

//    release = true; //Payload has been released

  }

  if(trig){
    if(millis() - applyheat > RELEASE_TIME * 1000){
      digitalWrite(WIRE,LOW); //Turn off nichrome wire
      Serial.println("END CUTDOWN");
    //  digitalWrite(LED_PIN, LOW);

    //  release = false;
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
	motor2.set_position(TURN_RADIUS*4);
  motor2.update();
}

void Avionics::fly(long start){

  state = ( (millis() - start)/TIME_STEP ) % 4;
//  Serial.println(state);
  if ( state == 0){
//    Serial.println("FLY FORWARD");
    forwardFlight();
  }
  else if( state == 1){
  //  Serial.println("BANK LEFT");
    bankLeft();
  }

  else if( state == 2){
//    Serial.println("FLY FORWARD");
    forwardFlight();
  }

  else if( state == 3){
//      Serial.println("BANK RIGHT");
    bankRight();
  }
}

void Avionics::smartSleep(unsigned long ms) {
  sensors.smartDelay(ms);
}

bool Avionics::getTrigState(){
  return trig;
}

void Avionics::setTrigState(bool state){
  trig =state;
}

long Avionics::getStart(){
  return start;
}

void Avionics::setStart(long set){
  start =set;
}

long Avionics::getPos1(){
  return motor1.getEncPosition();

}
long Avionics::getPos2(){
  return motor2.getEncPosition();

}
// bool Avionics::getFly(){
//   return iffly;
// }
