// #include "Motor.h"
//
// //CONSTRUCTOR
// Motor::Motor(int dir1_pin, int dir2_pin, int speed_pin, int enc1_pin, int enc2_pin):Enc(enc1_pin,enc2_pin){
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
//
// void Motor::setDirection(Direction dir) { //tells method to go cw or ccw.  cw and ccw will never both be false
//   switch (dir) {
//     case CW:
//       Serial.println("Clockwise");
//       digitalWrite(MOTOR_A_DIR_1, HIGH); //CHECK IF THIS SS THE CORRECT DIRECTION, OTHERWISE, CHANGE!
//       digitalWrite(MOTOR_A_DIR_2, LOW);
//
//     case CCW:
//       Serial.println("Counter-Clockwise")
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
// void Motor::performScriptedFlight(Encoder& EncA, Encoder& EncB){
//   Serial.println("break in scripted flight");
//   Serial.print("loop counter: ");
//   Serial.println(loop_count);
//   loop_count ++;
//   //Print current positions
//   long currentPosA = EncA.read();
//   long currentPosB = EncB.read();
//   Serial.print("EncA position: ");
//   Serial.println(currentPosA);
//   Serial.print("EncB position: ");
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
