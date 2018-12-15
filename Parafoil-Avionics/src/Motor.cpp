/*
  Stanford Student Space Initiative
  Balloons | BALLOONERANG | DECEMBER 2018
  File: Motors.cpp
  --------------------------
  Implements "Motor.h"
*/
#include "Motor.h"
#include "Motor.h"

void Motor::initialize(Encoder* Enc, int dir1_pin, int dir2_pin, int speed_pin){

  this->Enc = Enc;
  this->dir1_pin = dir1_pin;
  this->dir2_pin = dir2_pin;
  this->speed_pin = speed_pin;

  Serial.println("Initliatizing motor");

  pinMode(dir1_pin, OUTPUT);
  pinMode(dir2_pin, OUTPUT);
  pinMode(speed_pin, OUTPUT);

  pinMode(18,OUTPUT);
  pinMode(19,OUTPUT);
  pinMode(20,OUTPUT);

  this->speed = MOTOR_SPEED;
  this->target = 0;
  this->dir = NEUTRAL;
  this->Enc->write(0);
}

int Motor::getEncPosition(){
  return this->Enc->read();
}

/*
 * Function: set_position
 * -------------------
 * This function sets the position we want the motor to spin and which direction
 * spin
 */
void Motor::set_position(int pos){
  int enc = 0;
  if( this->dir1_pin == 23 && this->dir2_pin == 21 ){
    enc = 1;
  }
  else{
    enc = 2;
  }
  this->target= pos;

	if( pos -  this->Enc->read() > 0 ){
		this->dir = CW;

    pinMode(18,OUTPUT);
    pinMode(19,OUTPUT);
    pinMode(20,OUTPUT);
		digitalWrite(this->dir1_pin, LOW);
		digitalWrite(this->dir2_pin, HIGH);
	}else{
		this->dir = CCW;
		digitalWrite(this->dir1_pin, HIGH);
		digitalWrite(this->dir2_pin, LOW);
	}
	analogWrite(this->speed_pin, this->speed);
}

/*
 * Function: update
 * -------------------
 * This function will stop the motor if it has arrived at its desired position
 * within some threshold
 *
 */
int Motor::update(){
//  Serial.println(this->dir);
	if( this->dir != NEUTRAL){
		if( (abs(this->Enc->read()  - this->target) < 1000 ) ){

			analogWrite(this->speed_pin, 0);
			this->dir = NEUTRAL;
      //Serial.println("POSITION FOUND AND NOT MOVING");
    //  Serial.println("FOUND");
			digitalWrite(this->dir1_pin, LOW);
			digitalWrite(this->dir2_pin, LOW);
		}
	}
	return this->dir == NEUTRAL;
}
