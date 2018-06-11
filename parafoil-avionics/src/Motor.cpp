#include "Motor.h"

Motor::Motor(int dir1_pin, int dir2_pin, int speed_pin, int enc1_pin, int enc2_pin):Enc(enc1_pin,enc2_pin){

  this->dir1_pin = dir1_pin;
  this->dir2_pin = dir2_pin;
  this->speed_pin = speed_pin;
  
  Serial.println("Initliatizing motor");

  pinMode(dir1_pin, OUTPUT);
  pinMode(dir2_pin, OUTPUT);
  pinMode(speed_pin, OUTPUT);

  this->speed = 100;
  this->target = 0;
  this->dir = NEUTRAL;
  this->Enc.write(0);
}


void Motor::set_position(int pos){
	this->target = pos;

	if( pos -  this->Enc.read() > 0){
		this->dir = CW;
		digitalWrite(this->dir1_pin, LOW);
		digitalWrite(this->dir2_pin, HIGH);
	}else{
		this->dir = CCW;
		digitalWrite(this->dir1_pin, HIGH);
		digitalWrite(this->dir2_pin, LOW);
	}
	analogWrite(this->speed_pin, this->speed);
}



int Motor::update(){
	if( this->dir != NEUTRAL){
		if( ( (this->dir ==  CW) && (this->Enc.read() > this->target)) ||
		    ( (this->dir == CCW) && (this->Enc.read() < this->target)) ){
			analogWrite(this->speed_pin, 0);
			this->dir = NEUTRAL;
			digitalWrite(this->dir1_pin, LOW);
			digitalWrite(this->dir2_pin, LOW);
		}
	}
	return this->dir == NEUTRAL;
}



