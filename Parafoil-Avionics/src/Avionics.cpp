/*
  Stanford Student Space Initiative
  Balloons | BALLOONERANG | DECEMBER 2018
  File: Avionics.cpp
  --------------------------
  Implements "Avionics.h"
*/
#include "Avionics.h"

/*
 * Function: initialize
 * -------------------
 * This function initializes sensors, cutdown, and actuators
 */
void Avionics::initialize(){
  //Initialize sensors
  Serial.begin(9600); //Begin serial connection for serial port
  delay(5000); //Delay for serial monitor
  sensors.initializeSensors(); //Initialize sensors
  log.initializeSD(sensors); //Initialize SD Card

  //Initialize nichrome wire cutdown
  pinMode(NICHROME_WIRE,OUTPUT);
  digitalWrite(NICHROME_WIRE,LOW);
  trig = false;

  //Initialize DC motors & encoders
  Encoder *Enc1 = new Encoder(ENCODER_1_A,ENCODER_1_B);
  Encoder *Enc2 = new Encoder(ENCODER_2_A, ENCODER_2_B);
  motor1.initialize(Enc1, MOTOR_1_DIR_1, MOTOR_1_DIR_2, MOTOR_1_SPEED);
  motor2.initialize(Enc2, MOTOR_2_DIR_1, MOTOR_2_DIR_2, MOTOR_2_SPEED);
}

/*
 * Function: record
 * -------------------
 * This function writes sensor data and current flight maneuver to SD card
 */
void Avionics::record(){
  sensors.readAllSensors(); //Reads all sensors and prints to Serial
  log.writeSD(sensors, getStart(), getPos1(), getPos2());
}

/*
 * Function: cutdown
 * -------------------
 * This function cutdowns from some time period after passing altitude threshold
 */
void Avionics::cutdown(){
  if(!trig && ( sensors.getAlt() > CUTDOWN_ALT )) {
    if(timer == -1){
      timer = millis();
    }
    if(millis() - timer > TIMER_DELAY && timer != -1){
      digitalWrite(NICHROME_WIRE,HIGH);
      applyheat = millis(); //Start time of hot wire
      Serial.println("START CUTDOWN");
      trig = true;
    }
  }

  if(trig){
    if(millis() - applyheat > RELEASE_TIME * 1000){
      digitalWrite(NICHROME_WIRE,LOW); //Turn off nichrome wire
      Serial.println("END CUTDOWN");
    }
  }
}

/*
 * Function: forwardFlight
 * -------------------
 * This function sets both motors to their neutral positions
 */
void Avionics::forwardFlight(){
  motor1.set_position(0);
	motor1.update();
	motor2.set_position(0);
  motor2.update();
}

/*
 * Function: bankLeft
 * -------------------
 * This function rotates one motor to steer paylaod
 */
void Avionics::bankLeft(){
	motor1.set_position(TURN_RADIUS);
  motor1.update();
	motor2.set_position(0);
  motor2.update();
}

/*
 * Function: bankRight
 * -------------------
 * This function rotates one motor to steer paylaod
 */
void Avionics::bankRight(){
  motor1.set_position(0);
  motor1.update();
	motor2.set_position(TURN_RADIUS*4);
  motor2.update();
}

/*
 * Function: fly
 * -------------------
 * This function performs bankLeft, flyForward, and bankRight on a time step
 */
void Avionics::fly(long start){

  state = ( (millis() - start)/TIME_STEP ) % 4;
  if ( state == 0){
    forwardFlight();
  }
  else if( state == 1){
    bankLeft();
  }

  else if( state == 2){
    forwardFlight();
  }

  else if( state == 3){
    bankRight();
  }
}

/*
 * Function: smartSleep
 * -------------------
 * This function pauses main loop to read from Serail1 data
 */

void Avionics::smartSleep(unsigned long ms) {
  sensors.smartDelay(ms);
}

bool Avionics::getTrigState(){
  return trig;
}

void Avionics::setTrigState(bool state){
  trig = state;
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
