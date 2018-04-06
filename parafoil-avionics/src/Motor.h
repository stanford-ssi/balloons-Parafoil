#include <Encoder.h>
#include <PID_v1.h>

#include "Constants.h"

class Motors{

public:
  // define variables we'll be connecting to
  double Setpoint, Input, Output;
  // specify the links and initial tuning parameters
    double Kp=2;
    double Ki=5;
  double Kd=1; // This is for the encoder
  Motors():
   EncA(ENCODER_A_1, ENCODER_A_2);
   EncB(ENCODER_B_1, ENCODER_B_2);
   myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT){
  }


  bool initializeMotors();
  int readPWM();
  void calcInput();
  void setDirection();
  int comparePositions();
  long aileronToMotor(volatile int nAILERONIn);







  // Encoder objects


private:
  volatile int nAILERONIn = NEUTRAL_AILERON;
  volatile unsigned long ulStartPeriod = 0;
  volatile boolean bNewAILERONSignal = false;

  enum Direction { CW, NEUTRAL, CCW };
  Direction dir;

   Encoder EncA;
   Encoder EncB;
  PID myPID;

  long currentPos = 0;
  long currentPos2 = 0;
  long newPos = 0;
};
