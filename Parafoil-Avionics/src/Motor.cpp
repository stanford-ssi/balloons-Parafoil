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

//All this should do is determine where we want to go and what direction to spin
void Motor::set_position(int pos){
	this->target = pos;

  int enc = 0;
  if( this->dir1_pin == 23 && this->dir2_pin == 21 ){
    enc = 1;
  }
  else{
    enc = 2;
  }
  // if(enc == 2){
  // //  Serial.println(this->target);
  // //  Serial.println(this->Enc->read());
  // }
  // Serial.println("ENCODER: " + String(enc));
  // Serial.print("WHERE I WANT TO GO: ");
  // Serial.println(pos);

	if( pos -  this->Enc->read() > 0 ){
		this->dir = CW;
    // Serial.println("CW");
//     Serial.println(pos);
//     Serial.println( this->Enc->read());
//
    pinMode(18,OUTPUT);
    pinMode(19,OUTPUT);
    pinMode(20,OUTPUT);
		digitalWrite(this->dir1_pin, LOW);
		digitalWrite(this->dir2_pin, HIGH);
	}else{
		this->dir = CCW;
  //  Serial.println("CCW");
  //  Serial.println(pos -  this->Enc->read());
  //  Serial.println("HELLO");
		digitalWrite(this->dir1_pin, HIGH);
		digitalWrite(this->dir2_pin, LOW);
	}
	analogWrite(this->speed_pin, this->speed);
}


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

// int Motor::update(){
// //  Serial.println(this->dir);
// 	if( this->dir != NEUTRAL){
// 		if( ( (this->dir ==  CW) && (abs(this->Enc->read()  - this->target) > 0 )) ||
// 		    ( (this->dir == CCW) && (abs(this->Enc->read()  - this->target) > 0 )) ){
// 			analogWrite(this->speed_pin, 0);
// 			this->dir = NEUTRAL;
//       Serial.println("POSITION FOUND AND NOT MOVING");
//     //  Serial.println("FOUND");
// 			digitalWrite(this->dir1_pin, LOW);
// 			digitalWrite(this->dir2_pin, LOW);
// 		}
// 	}
// 	return this->dir == NEUTRAL;
// }



void Motor::setDirection(Direction dir) { //tells method to go cw or ccw.  cw and ccw will never both be false
  switch (dir) {
    case CW:
      Serial.println("Clockwise");
      digitalWrite(MOTOR_1_DIR_1, HIGH); //CHECK IF THIS SS THE CORRECT DIRECTION, OTHERWISE, CHANGE!
      digitalWrite(MOTOR_1_DIR_2, LOW);

      digitalWrite(MOTOR_2_DIR_1, LOW);
      digitalWrite(MOTOR_2_DIR_2, LOW);
      break;
    case NEUTRAL:
      Serial.println("Neutral");
      digitalWrite(MOTOR_1_DIR_1, LOW);
      digitalWrite(MOTOR_1_DIR_2, LOW);

      digitalWrite(MOTOR_2_DIR_1, LOW);
      digitalWrite(MOTOR_2_DIR_2, LOW);
      break;
    case CCW:
      Serial.println("Counter-Clockwise");
      digitalWrite(MOTOR_1_DIR_1, LOW);
      digitalWrite(MOTOR_1_DIR_2, HIGH);

      digitalWrite(MOTOR_2_DIR_1, LOW);
      digitalWrite(MOTOR_2_DIR_2, LOW);
      break;
    default:
      Serial.println("Illegal enum value");
      digitalWrite(MOTOR_1_DIR_1, LOW);
      digitalWrite(MOTOR_1_DIR_2, LOW);

      digitalWrite(MOTOR_2_DIR_1, LOW);
      digitalWrite(MOTOR_2_DIR_2, LOW);
      break;
  }
}




//compares current position of either motor A and B and specifies direction
int Motor::comparePositions(long currentPos, long setPoint) {
  analogWrite(MOTOR_1_SPEED, 100);
  analogWrite(MOTOR_2_SPEED, 100);
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

// void Motor::bankLeft(long currentPosA){
//     comparePositions(currentPosA, MAX_MOTOR);
// }
//
// void Motor::bankRight(long currentPosB){
//     comparePositions(currentPosB, MAX_MOTOR);
// }
// //CONSTRUCTOR
// Motor::Motor(int dir1_pin, int dir2_pin, int speed_pin, int enc1_pin, int enc2_pin){
//
//   //Initializing pins
//   this->dir1_pin = dir1_pin;
//   this->dir2_pin = dir2_pin;
//   this->speed_pin = speed_pin;
//
//   Serial.println("Initliatizing motor");
//
//   pinMode(dir1_pin, OUTPUT);
//   pinMode(dir2_pin, OUTPUT);
//   pinMode(speed_pin, OUTPUT);
//
//   this->speed = 100;
//   this->target = 0;
//   this->dir = NEUTRAL;
//   this->Enc.write(0);
// }

// 'void Motor::setDirection(Direction dir) { //tells method to go cw or ccw.  cw and ccw will never both be false
//   switch (dir) {
//     case CW:
//       Serial.println("Clockwise");
//       digitalWrite(MOTOR_A_DIR_1, HIGH); //CHECK IF THIS SS THE CORRECT DIRECTION, OTHERWISE, CHANGE!
//       digitalWrite(MOTOR_A_DIR_2, LOW);
//
//     case CCW:
//       Serial.println("Counter-Clockwise");
//       digitalWrite(MOTOR_B_DIR_1, HIGH);
//       digitalWrite(MOTOR_B_DIR_2, LOW);
//       break;
//
//     case NEUTRAL:
//       Serial.println("Neutral");
//       digitalWrite(MOTOR_A_DIR_1, LOW);
//       digitalWrite(MOTOR_A_DIR_2, LOW);
//
//
// //READ POSITION OF motors
// //FIND HOW MANY STEPS TO TURN EACH MOTOR TO DESIRED POSITION
// void Motor::set_position(int pos){
// 	this->target = pos;
//
//       digitalWrite(MOTOR_B_DIR_1, HIGH);
//       digitalWrite(MOTOR_B_DIR_2, LOW);
//       break;
//     default:
//       Serial.println("Illegal enum value");
//       digitalWrite(MOTOR_A_DIR_1, LOW);
//       digitalWrite(MOTOR_A_DIR_2, LOW);
//
//       digitalWrite(MOTOR_B_DIR_1, LOW);
//       digitalWrite(MOTOR_B_DIR_2, LOW);
//       break;
//   }
// }
//
// //ANOTHER method
// //TELLS MOTOR TO SPIN X NUMBER OF STEPS LEFT OR RIGHT (POSITIVE OR NEGATIVE)
//
//
// int loop_count = 0;
// void Motor::performScriptedFlight(Encoder& Enc1, Encoder& Enc2){
//   Serial.println("break in scripted flight");
//   Serial.print("loop counter: ");
//   Serial.println(loop_count);
//   loop_count ++;
//   //Print current positions
//   long currentPosA = Enc1.read();
//   long currentPosB = Enc2.read();
//   Serial.print("Enc1 position: ");
//   Serial.println(currentPosA);
//   Serial.print("Enc2 position: ");
//   Serial.println(currentPosB);
//     long loopTime = millis();
//   if( loopTime < 30000){
//     setDirection(CW);
//   }
//
//   if( loopTime > 30000){
//     Serial.println("LOOP");
//     Serial.println(currentPosA);
//       Serial.println("break in correction");
//     int cmp = comparePositions(currentPosA, 0);
//     if (cmp == 0) { // position within margins => dont move
//         setDirection(NEUTRAL);
//         return;
//       } else { // need to move
//         if (cmp > 0) { // forward
//           setDirection(CCW);
//         } else { // cmp < 0 => backward
//           setDirection(CW);
//         }
//       }
//
//   }
//
//
// int Motor::update(){
// 	if( this->dir != NEUTRAL){
// 		if( ( (this->dir ==  CW) && (this->Enc.read() > this->target)) ||
// 		    ( (this->dir == CCW) && (this->Enc.read() < this->target)) ){
// 			analogWrite(this->speed_pin, 0);
// 			this->dir = NEUTRAL;
// 			digitalWrite(this->dir1_pin, LOW);
// 			digitalWrite(this->dir2_pin, LOW);
// 		}
// 	}
// 	return this->dir == NEUTRAL;
// }
