
// First Example in a series of posts illustrating reading an RC Receiver with
// micro controller interrupts.
//
// Subsequent posts will provide enhancements required for real world operation
// in high speed applications with multiple inputs.
//
// http://rcarduino.blogspot.com/
//
// Posts in the series will be titled - How To Read an RC Receiver With A Microcontroller

// See also http://rcarduino.blogspot.co.uk/2012/04/how-to-read-multiple-rc-channels-draft.html 

#define AILERON_SIGNAL_IN 2 // INTERRUPT 2 = DIGITAL PIN 2 - use the interrupt number in attachInterrupt
#define AILERON_SIGNAL_IN_PIN 2 // INTERRUPT 0 = DIGITAL PIN 2 - use the PIN number in digitalRead

#define NEUTRAL_AILERON 1500 // this is the duration in microseconds of neutral AILERON on an electric RC Car

volatile int nAILERONIn = NEUTRAL_AILERON; // volatile, we set this in the Interrupt and read it in loop so it must be declared volatile
volatile unsigned long ulStartPeriod = 0; // set in the interrupt
volatile boolean bNewAILERONSignal = false; // set in the interrupt and read in the loop
// we could use nAILERONIn = 0 in loop instead of a separate variable, but using bNewAILERONSignal to indicate we have a new signal
// is clearer for this first example

void setup()
{
  // tell the Arduino we want the function calcInput to be called whenever INT0 (digital pin 2) changes from HIGH to LOW or LOW to HIGH
  // catching these changes will allow us to calculate how long the input pulse is
  attachInterrupt(AILERON_SIGNAL_IN,calcInput,CHANGE);

  Serial.begin(9600);
}

void loop()
{
 // if a new AILERON signal has been measured, lets print the value to serial, if not our code could carry on with some other processing
 if(bNewAILERONSignal)
 {

   Serial.println(nAILERONIn); 

   // set this back to false when we have finished
   // with nAILERONIn, while true, calcInput will not update
   // nAILERONIn
   bNewAILERONSignal = false;
 }

 Serial.print("AILERON PWM: ");
   Serial.println(nAILERONIn); 
 // other processing ...
}

void calcInput()
{
  // if the pin is high, its the start of an interrupt
  if(digitalRead(AILERON_SIGNAL_IN_PIN) == HIGH)
  {
    // get the time using micros - when our code gets really busy this will become inaccurate, but for the current application its
    // easy to understand and works very well
    ulStartPeriod = micros();
  }
  else
  {
    // if the pin is low, its the falling edge of the pulse so now we can calculate the pulse duration by subtracting the
    // start time ulStartPeriod from the current time returned by micros()
    if(ulStartPeriod && (bNewAILERONSignal == false))
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
