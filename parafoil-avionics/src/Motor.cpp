#include "Motor.h"

void Motor::initializeMotors(){
  Serial.println("Initliatizing motors pins and setDirection");
  pinMode(MOTOR_A_DIR_1, OUTPUT);
  pinMode(MOTOR_A_DIR_2, OUTPUT);
  pinMode(MOTOR_A_SPEED, OUTPUT);

  pinMode(MOTOR_B_DIR_1, OUTPUT);
  pinMode(MOTOR_B_DIR_2, OUTPUT);
  pinMode(MOTOR_B_SPEED, OUTPUT);

  setDirection(NEUTRAL);
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


void Motor::performScriptedFlight(Encoder& EncA, Encoder& EncB){
  Serial.println("break in scripted flight");
  //Print current positions
  long currentPosA = EncA.read();
  long currentPosB = EncB.read();
  Serial.print("EncA position: ");
  Serial.println(currentPosA);
  Serial.print("EncB position: ");
  Serial.println(currentPosB);
    long loopTime = millis();
  if( loopTime < 30000){
    setDirection(CW);
  }

  if( loopTime > 30000){
    Serial.println("LOOP");
    Serial.println(currentPosA);
      Serial.println("break in correction");
    int cmp = comparePositions(currentPosA, 0);
    if (cmp == 0) { // position within margins => dont move
        setDirection(NEUTRAL);
        return;
      } else { // need to move
        if (cmp > 0) { // forward
          setDirection(CCW);
        } else { // cmp < 0 => backward
          setDirection(CW);
        }
      }

  }
  // else if( loopTime < (1 * 60000))
  //   setDirection(NEUTRAL);
  // }
  // else{
  //
  // }

//   bankLeft(currentPosA);
  //
  // if(loopTime - (counter * FORWARD_FLIGHT_TIME) < 0){
  //   Serial.println("FLY FORWARD");
  //   forwardFlight(loopTime, currentPosA, currentPosB);
  //   return;
  // }
  //
  // else if(loopTime - (counter * BANK_LEFT_TIME) < 0){
  //   Serial.println("BANK LEFT");
  //   bankLeft(loopTime, currentPosA);
  //   return;
  // }
  //
  // else if(loopTime - (counter * FORWARD_FLIGHT_TIME) < 0){
  //   Serial.println("FLY FORWARD");
  //   forwardFlight(loopTime,currentPosA, currentPosB);
  //   return;
  // }
  //
  // else if(loopTime - (counter * BANK_RIGHT_TIME) < 0){
  //   Serial.println("BANK RIGHT");
  //   bankRight(loopTime, currentPosB);
  //   return;
  // }

  return;
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
