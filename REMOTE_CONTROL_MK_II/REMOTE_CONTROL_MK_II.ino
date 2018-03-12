//THIS CODE USES REMOTE CONTROL RECEIVER TO SPIN MOTORS W/ ENCODER TO DESIRED POSITION USING PWM




//LIBRARIES
#include <Encoder.h>
#include <PID_v1.h>

#define AILERON_SIGNAL_IN 4 // INTERRUPT 2 = DIGITAL PIN 2 - use the interrupt number in attachInterrupt
#define AILERON_SIGNAL_IN_PIN 4 // INTERRUPT 0 = DIGITAL PIN 2 - use the PIN number in digitalRead
#define NEUTRAL_AILERON 1500 // this is the duration middle PWM on the aileron stick
#define MIN_AILERON 1000
#define MAX_AILERON 2000
#define MAX_MOTOR 4000
#define MIN_MOTOR -4000

enum Direction {
  CW = -1,
  NEUTRAL = 0,
  CCW = 1
};

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1; // This is for the encoder
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

Encoder myEnc(0, 1); //Encoder object

long currentPos = -999;
long newPos = -999;

/*
   Interrupt Service Routine (ISR) CONSTANTS
   nAileron: Volatile, we set this in the interrupt and read it in the loop, therefore it must be declared volatile.
   ulStartPeriod: Set in the interrupt, start time of the PWM signal
   bNewAILERONSignal: Set in interrupt and read in loop. This indiciates when we have a new signal
*/
volatile int nAILERONIn = NEUTRAL_AILERON;
volatile unsigned long ulStartPeriod = 0;
volatile boolean bNewAILERONSignal = false;

Direction dir;





void setup() {
  Serial.begin(9600);
  Serial.println("Basic Encoder Test:");

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(14, OUTPUT);
  
  digitalWrite(2, LOW); //STARTS LOW, IN THIS TEST, ENCODER DOES NOT KNOW DIRECITON THAT IT SPINS
  digitalWrite(3, LOW);

  pinMode(4,INPUT);

    //initialize the variables we're linked to
  Input = analogRead(2);
  Setpoint = 10000;

  //turn the PID on
  myPID.SetMode(AUTOMATIC);

  attachInterrupt(AILERON_SIGNAL_IN, calcInput, CHANGE);
}

long oldPosition  = -999;

void loop() {


  receivePWM();
  Serial.print("Raw AILERON PWM: ");
  Serial.println(nAILERONIn);

  if (nAILERONIn > 1480 && nAILERONIn < 1500) {
    directionChange(0);
  } else if(nAILERONIn > NEUTRAL_AILERON){
    newPos = map(nAILERONIn, NEUTRAL_AILERON, MAX_AILERON, 0, MAX_MOTOR);
    currentPos = myEnc.read();
    Input = currentPos;
    Setpoint = abs(newPos);
    Serial.print("NEW SETPOINT: ");
    Serial.println(Setpoint);
    myPID.Compute();
    Serial.print("PWM OUTPUT: ");
    Serial.println(Output);

    //DIRECTION SHIT
    directionChange(1);
    
    analogWrite(14, Output);
  } else if(nAILERONIn < NEUTRAL_AILERON){
    newPos = map(nAILERONIn, MIN_AILERON, NEUTRAL_AILERON, -4000, 0);
    currentPos = myEnc.read();
    Input = currentPos;
    Setpoint = abs(newPos);
    Serial.print("NEW SETPOINT: ");
    Serial.println(Setpoint);
    myPID.Compute();
    Serial.print("PWM OUTPUT: ");
    Serial.println(Output);

    //DIRECTION SHIT
    directionChange(-1);
    analogWrite(14, Output);
  }
}


void calcInput()
{
  // if the pin is high, its the start of an interrupt
  if (digitalRead(AILERON_SIGNAL_IN_PIN) == HIGH)
  {
    // get the time using micros - when our code gets really busy this will become inaccurate, but for the current application its
    // easy to understand and works very well
    ulStartPeriod = micros();
  }
  else
  {
    // if the pin is low, its the falling edge of the pulse so now we can calculate the pulse duration by subtracting the
    // start time ulStartPeriod from the current time returned by micros()
    if (ulStartPeriod && (bNewAILERONSignal == false))
    {
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
  if (bNewAILERONSignal)
  {
    // set this back to false when we have finished
    // with nAILERONIn, while true, calcInput will not update
    // nAILERONIn
    bNewAILERONSignal = false;
  }
}

void directionChange(int dir) { //tells method to go cw or ccw.  cw and ccw will never both be false
  if (dir == -1) {
    Serial.println("Clockwise");
    digitalWrite(2,HIGH); //CHECK IF THIS SPINS IN THE CORRECT DIRECTION, OTHERWISE, CHANGE!
    digitalWrite(3,LOW);
  } else if (dir == 0) {
    Serial.println("Neutral");
    digitalWrite(2, LOW);
    digitalWrite(3, LOW);
  } else if (dir == 1) {
    Serial.println("Counter-Clockwise");
    digitalWrite(2, LOW);
    digitalWrite(3, HIGH);
  } else {
    Serial.println("Illegal enum value");
    digitalWrite(2, LOW);
    digitalWrite(3, LOW);
  }  
}


