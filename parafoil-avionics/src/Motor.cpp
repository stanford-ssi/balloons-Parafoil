#include "Motor.h"

void Motor::initializeMotors(){
  Serial.println("Initliatizing motors and encoders");
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
      digitalWrite(MOTOR_A_DIR_2, LOW);

      digitalWrite(MOTOR_B_DIR_1, HIGH);
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

  //Print current positions
  long currentPosA = EncA.read();
  long currentPosB = EncB.read();
  Serial.print("EncA position: ");
  Serial.println(currentPosA);
  Serial.print("EncB position: ");
  Serial.println(currentPosB);

  long loopTime = millis();

  if(loopTime - (counter * FORWARD_FLIGHT_TIME) < 0){
    Serial.println("FLY FORWARD");
    forwardFlight(loopTime, currentPosA, currentPosB);
    return;
  }

  else if(loopTime - (counter * BANK_LEFT_TIME) < 0){
    Serial.println("BANK LEFT");
    bankLeft(loopTime, currentPosA);
    return;
  }

  else if(loopTime - (counter * FORWARD_FLIGHT_TIME) < 0){
    Serial.println("FLY FORWARD");
    forwardFlight(loopTime,currentPosA, currentPosB);
    return;
  }

  else if(loopTime - (counter * BANK_RIGHT_TIME) < 0){
    Serial.println("BANK RIGHT");
    bankRight(loopTime, currentPosB);
    return;
  }
  counter++;
  return;
}

//compares current position of either motor A and B and specifies direction
int Motor::comparePositions(long currentPos, long setPoint) {
  analogWrite(MOTOR_A_SPEED, 255);
  analogWrite(MOTOR_B_SPEED, 255);
  int diff = setPoint - currentPos;
  if (abs(diff) < SETPOINT_MARGIN_RADIUS) {
    setDirection(NEUTRAL);
    return 0;
  } else if (diff < 0) {
    setDirection(CCW);
    return -1;
  } else { // diff > 0
    setDirection(CW);
    return 1;
  }
}

void Motor::forwardFlight(long loopTime, long currentPosA, long currentPosB){
    comparePositions(currentPosA, NEUTRAL);
    comparePositions(currentPosB, NEUTRAL);
}

void Motor::bankLeft(long loopTime, long currentPosA){
    comparePositions(currentPosA, MAX_MOTOR);
}

void Motor::bankRight(long loopTime, long currentPosB){
    comparePositions(currentPosB, MAX_MOTOR);
}
