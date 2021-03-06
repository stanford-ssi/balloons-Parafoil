// this code uses remote control receiver to spin motors w/ encoder to desired position using pwm


// libraries
#include <Encoder.h>
#include <PID_v1.h>

#define AILERON_SIGNAL 4 // INTERRUPT 2 = DIGITAL PIN 2 - use the interrupt number in attachInterrupt

#define MOTOR_A_DIR_1 2
#define MOTOR_A_DIR_2 3
#define MOTOR_A_SPEED 14

#define MOTOR_B_DIR_1 9
#define MOTOR_B_DIR_2 8
#define MOTOR_B_SPEED 15

#define ENCODER_A_1 0
#define ENCODER_A_2 1

#define ENCODER_B_1 11
#define ENCODER_B_2 12

#define MIN_AILERON 1000
#define NEUTRAL_AILERON 1500 // this is the duration middle PWM on the aileron stick
#define MAX_AILERON 2000

#define MIN_MOTOR -4000
#define NEUTRAL_MOTOR 0
#define MAX_MOTOR 4000

#define SETPOINT_MARGIN_RADIUS 10

enum Direction { CW, NEUTRAL, CCW };

// define variables we'll be connecting to
double Setpoint, Input, Output;

// specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1; // This is for the encoder
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Encoder objects
Encoder EncA(ENCODER_A_1, ENCODER_A_2);
Encoder EncB(ENCODER_B_1, ENCODER_B_2);

long currentPos = -999;
long newPos = -999;

/*
  Interrupt Service Routine (ISR) CONSTANTS:
    nAileron: volatile. set in the interrupt and read in the loop: it must be declared volatile
    ulStartPeriod: set in the interrupt, start time of the PWM signal
    bNewAILERONSignal: set in interrupt and read in loop. indiciates when we have a new signal
 */
volatile int nAILERONIn = NEUTRAL_AILERON;
volatile unsigned long ulStartPeriod = 0;
volatile boolean bNewAILERONSignal = false;

Direction dir;

void setup() {
  Serial.begin(9600);
  Serial.println("Basic Encoder Test:");

  // initialize pins
  pinMode(MOTOR_A_DIR_1, OUTPUT);
  pinMode(MOTOR_A_DIR_2, OUTPUT);
  pinMode(MOTOR_A_SPEED, OUTPUT);

  pinMode(MOTOR_B_DIR_1, OUTPUT);
  pinMode(MOTOR_B_DIR_2, OUTPUT);
  pinMode(MOTOR_B_SPEED, OUTPUT);

  pinMode(AILERON_SIGNAL, INPUT);

  EncA.write(NEUTRAL_MOTOR); // start off in neutral motor position
  EncB.write(NEUTRAL_MOTOR); // start off in neutral motor position

  setDirection(NEUTRAL); // encoder does not know direciton that it spins

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  attachInterrupt(AILERON_SIGNAL, calcInput, CHANGE);
}

void loop() {
  receivePWM();
  Serial.print("Raw AILERON PWM: ");
  Serial.println(nAILERONIn);

  Input = currentPos = EncA.read();
  newPos = aileronToMotor(nAILERONIn);
  Setpoint = abs(newPos);
  Serial.print("NEW SETPT: ");
  Serial.println(Setpoint);

  int cmp = comparePositions(newPos, Setpoint);
  if (cmp == 0) { // position within margins => dont move
    setDirection(NEUTRAL); 
    return;
  } else { // need to move
    if (cmp > 0) { // forward
      setDirection(CCW);
    } else { // cmp < 0 => backward
      setDirection(CW);
    } 

    // compute speed
    myPID.Compute();
    Serial.print("PWM OUTPUT: ");
    Serial.println(Output);
    analogWrite(MOTOR_A_SPEED, 255);
    analogWrite(MOTOR_B_SPEED, 255);
  }
}


void calcInput()
{
  // if the pin is high, its the start of an interrupt
  if (digitalRead(AILERON_SIGNAL) == HIGH) {
    // get the time using micros - when our code gets really busy this will become inaccurate, but for the current application its
    // easy to understand and works very well
    ulStartPeriod = micros();
  } else {
    // if the pin is low, its the falling edge of the pulse so now we can calculate the pulse duration by subtracting the
    // start time ulStartPeriod from the current time returned by micros()
    if (ulStartPeriod && !bNewAILERONSignal) {
      nAILERONIn = (int)(micros() - ulStartPeriod);
      ulStartPeriod = 0;

      // tell loop we have a new signal on the AILERON channel
      // we will not update nAILERONIn until loop sets
      // bNewAILERONSignal back to false
      bNewAILERONSignal = true;
    }
  }
}

void receivePWM() {
  //FOR RC SIGNAL
  // if a new AILERON signal has been measured, lets print the value to serial, if not our code could carry on with some other processing
  if (bNewAILERONSignal) {
    // set this back to false when we have finished
    // with nAILERONIn, while true, calcInput will not update
    // nAILERONIn
    bNewAILERONSignal = false;
  }
}

long aileronToMotor(volatile int nAILERONIn) {
  return map(nAILERONIn, MIN_AILERON, MAX_AILERON, MIN_MOTOR, MAX_MOTOR);
}

int comparePositions(long currentPos, long setPoint) {
  int diff = setPoint - currentPos;
  if (abs(diff) < SETPOINT_MARGIN_RADIUS) {
    return 0;
  } else if (diff < 0) {
    return -1;
  } else { // diff > 0
    return 1;
  }
}

void setDirection(Direction dir) { //tells method to go cw or ccw.  cw and ccw will never both be false
  switch (dir) {
    case CW:
      Serial.println("Clockwise");
      digitalWrite(MOTOR_A_DIR_1, HIGH); //CHECK IF THIS SS THE CORRECT DIRECTION, OTHERWISE, CHANGE!
      digitalWrite(MOTOR_A_DIR_2, LOW);

      digitalWrite(MOTOR_B_DIR_1, LOW);
      digitalWrite(MOTOR_B_DIR_2, HIGH);
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


