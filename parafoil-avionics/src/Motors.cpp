// #include "Motor.h"
//
// bool Motors::initializeMotors(){
//   // initialize pins
//   pinMode(MOTOR_A_DIR_1, OUTPUT);
//   pinMode(MOTOR_A_DIR_2, OUTPUT);
//   pinMode(MOTOR_A_SPEED, OUTPUT);
//
//   pinMode(MOTOR_B_DIR_1, OUTPUT);
//   pinMode(MOTOR_B_DIR_2, OUTPUT);
//   pinMode(MOTOR_B_SPEED, OUTPUT);
//
//   pinMode(AILERON_SIGNAL, INPUT);
//
//   EncA.write(NEUTRAL_MOTOR); // start off in neutral motor position
//   EncB.write(NEUTRAL_MOTOR); // start off in neutral motor position
//
//   setDirection(NEUTRAL); // encoder does not know direciton that it spins
//
//   //turn the PID on
//   myPID.SetMode(AUTOMATIC);
//   attachInterrupt(AILERON_SIGNAL, calcInput, CHANGE);
// }
//
// int readPWM(){
//
// }
