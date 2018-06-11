#include "Motor.h"

void Motor::initialize(int dir1_pin, int dir2_pin, int speed_pin, int enc1_pin, int enc2_pin){

  /* this->Enc = Encoder((uint8_t) enc1_pin, (uint8_t) enc2_pin); */

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



void Motor::setDirection(Direction dir) { //tells method to go cw or ccw.  cw and ccw will never both be false
  switch (dir) {
    case CW:
      Serial.println("Clockwise");
      digitalWrite(MOTOR_A_DIR_1, HIGH); //CHECK IF THIS SS THE CORRECT DIRECTION, OTHERWISE, CHANGE!
      digitalWrite(MOTOR_A_DIR_2, LOW);

      digitalWrite(MOTOR_B_DIR_1, LOW);
      digitalWrite(MOTOR_B_DIR_2, LOW);
      break;
    case NEUTRAL:
      Serial.println("Neutral");
      digitalWrite(MOTOR_A_DIR_1, LOW);
      digitalWrite(MOTOR_A_DIR_2, LOW);

      digitalWrite(MOTOR_B_DIR_1, LOW);
      digitalWrite(MOTOR_B_DIR_2, LOW);
      break;
    case CCW:
      Serial.println("Counter-Clockwise");
      digitalWrite(MOTOR_A_DIR_1, LOW);
      digitalWrite(MOTOR_A_DIR_2, HIGH);

      digitalWrite(MOTOR_B_DIR_1, LOW);
      digitalWrite(MOTOR_B_DIR_2, LOW);
      break;
    default:
      Serial.println("Illegal enum value");
      digitalWrite(MOTOR_A_DIR_1, LOW);
      digitalWrite(MOTOR_A_DIR_2, LOW);

      digitalWrite(MOTOR_B_DIR_1, LOW);
      digitalWrite(MOTOR_B_DIR_2, LOW);
      break;
  }
}




//compares current position of either motor A and B and specifies direction
int Motor::comparePositions(long currentPos, long setPoint) {
  analogWrite(MOTOR_A_SPEED, 100);
  analogWrite(MOTOR_B_SPEED, 100);
  int diff = setPoint - currentPos;
  Serial.print("Difference: ");
  Serial.println(diff);
  if (abs(diff) < SETPOINT_MARGIN_RADIUS) {
    Serial.println("Found position and hold");
  //  setDirection(NEUTRAL);
    return 0;
  } else if (diff > 0) {
    Serial.println("Keep going");
    //setDirection(CW);
    return -1;
  } else { // diff < 0
    Serial.println("Keep going");
    //setDirection(CCW);
    return 1;
  }
}

void Motor::forwardFlight(long loopTime, long currentPosA, long currentPosB){
    comparePositions(currentPosA, NEUTRAL);
    comparePositions(currentPosB, NEUTRAL);
}

void Motor::bankLeft(long currentPosA){
    comparePositions(currentPosA, MAX_MOTOR);
}

void Motor::bankRight(long currentPosB){
    comparePositions(currentPosB, MAX_MOTOR);
}
