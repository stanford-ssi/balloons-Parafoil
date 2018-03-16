/*
   Avionics for parafoil.
   This code logs data from sensors, reads PWM values from the RC receiver, and
   sends a PWM signal to control speed and direction of the motors

*/



// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability


//   avoid using pins with LEDs attached
#include <Arduino.h>
#include <Encoder.h>
#include <PID_v1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_MAX31855.h>
#include <TinyGPS++.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>

#define AILERON_SIGNAL 6 // INTERRUPT 2 = DIGITAL PIN 2 - use the interrupt number in attachInterrupt


/*
   Preprocessor directive to debug code using print statements
   LED_PIN is a flashes if avionics components are not initialized
*/

#define DEBUG // comment out to turn off debugging
#ifdef DEBUG
#define DEBUG_PRINT(x)  Serial.print (String(x))
#define DEBUG_PRINTLN(x)  Serial.println (String(x))
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

#define LED_PIN 22

#define MOTOR_A_DIR_1 0
#define MOTOR_A_DIR_2 1
#define MOTOR_A_SPEED 14

#define MOTOR_B_DIR_1 2
#define MOTOR_B_DIR_2 3
#define MOTOR_B_SPEED 15

#define ENCODER_A_1 4
#define ENCODER_A_2 5

#define ENCODER_B_1 9
#define ENCODER_B_2 10

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

long currentPos = 0;
long currentPos2 = 0;
long newPos = 0;

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


bool firstSend = true;
String dataStringBuffer = "";


/*
   TIMING (INTERNAL)
   This initializes timing variables used to monitor SD card flush time, RB transmission time,
   and barometer measurement interval
*/
long startTime;
#define SD_CARD_FLUSH_TIME 10000 // Flushes SD card every 10 Seconds
#define ROCKBLOCK_TRANSMIT_TIME 300000 // Transmits to RB every 5 minutes
#define BAROMETER_MEASURMENT_INTERVAL 10000 // 10 Seconds
#define SERIAL_TIMEOUT 60000 //Serial timeout

/*
   SPI BUS
   This defines constants for the SPI line.  The thermocouple and SD card are on the SPI line
*/
#define SCK_PIN 13
#define SD_READER_CS 20
#define THERMOCOUPLE_CS 21

// SD Card Reader (SPI)
File dataFile;
long lastFlush;

// Thermocouple (SPI) | Exterior Temperature
Adafruit_MAX31855 thermocouple(13, THERMOCOUPLE_CS, 12);

// Environemental Parameters
#define LAUNCH_SITE_PRESSURE 1014.562

// BMP280 (I2C) | Pressure, Internal Temperature
Adafruit_BMP280 bmp;
long lastAscentTime;             // Time of last ascent rate calculation
double lastAlt;                 // Last altitude, for calculation
double ascentRate;              // Last calculated rate, to fill forward in logging

//BNO055 (I2C) IMU
Adafruit_BNO055 bno;

// GPS (Hardware Serial) GPS
TinyGPSPlus gps;
float f_lat = 0, f_long = 0;

int sats = -1;
long unsigned f_age = 0;


bool hasSD = false;


// Prototypes
String readSensors();
void flashLED();
void calcInput();
void receivePWM();
long aileronToMotor(volatile int nAILERONIn);
int comparePositions(long currentPos, long newPos);
void setDirection(Direction dir);

void setup() {
  startTime = millis();
  lastFlush = 0;



  pinMode(LED_PIN, OUTPUT); //Flashes LED when SD, BMP, or BNO not intialized
  digitalWrite(LED_PIN, LOW);



  //#ifdef DEBUG
  //  Serial.begin(9600);
  //  for (int i = 0; i < SERIAL_TIMEOUT && !Serial; i++) {
  //    continue;
  //    delay(1000);
  //  }
  //#endif

  // SD Card Reader
  SPI.setSCK(SCK_PIN);
  pinMode(SD_READER_CS, OUTPUT);
  DEBUG_PRINTLN("Initializing SD card...");
  if (!SD.begin(SD_READER_CS)) {
    DEBUG_PRINTLN("Card failed, or not present");
    // flashLED(); // BLOCKS CODE
  } else {
    dataFile = SD.open("datalog.txt", FILE_WRITE);
    DEBUG_PRINTLN("Card initialized.");
  }

  // BMP280
  if (!bmp.begin()) {
    DEBUG_PRINTLN("Could not find a valid BMP280 sensor, check wiring!");
    //flashLED(); //WILL STOP CODE IF BMP NOT DETECTED
  }
  lastAlt = bmp.readAltitude(LAUNCH_SITE_PRESSURE); // initialize ascent rate variables
  ascentRate = 0;
  lastAscentTime = millis();

  // BNO055
  if (!bno.begin()) {
    DEBUG_PRINTLN("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    //flashLED();  //WILL STOP CODE IF BNO NOT DETECTED
  }
  bno.setExtCrystalUse(true); // TODO figure this out

  // GPS
  Serial1.begin(9600);

  pinMode(THERMOCOUPLE_CS, OUTPUT);

  // Data column headers.
  if (dataFile) {
    dataFile.print("Time(ms), Pressure(Pa), Alt(m), AscentRate(m/s), TempIn(C), OrientationX(deg), y(deg) , z(deg), ");
    dataFile.println("TempOut(C), GPSLat, GPSLong, GPSAge, GPSSats, RBSignalQuality");
  }

  // RockBlock
  //IridiumSerial.begin(19200);
  //  DEBUG_PRINTLN("Starting rockblock serial");
  //  int err = modem.begin();
  //  int signalQuality = -1;
  //  DEBUG_PRINTLN(modem.getSignalQuality(signalQuality));
  //  if (err != ISBD_SUCCESS) {
  //    DEBUG_PRINT("Begin failed: error ");
  //    DEBUG_PRINTLN(err);
  //    if (err == ISBD_NO_MODEM_DETECTED) DEBUG_PRINTLN("No modem detected: check wiring.");
  //    //flashLED();
  //  }

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

  long loopTime = millis();
  readSensors();

  receivePWM();
  Serial.print("Raw AILERON PWM: ");
  Serial.println(nAILERONIn);

  currentPos = EncB.read();
  currentPos2 = EncB.read();
  Serial.print("Current pos: ");
  Serial.println(currentPos);
  Serial.print("Current pos 2: ");
  Serial.println(currentPos2);
  Input = currentPos;
  newPos = aileronToMotor(nAILERONIn);
  Setpoint = abs(newPos);
  Serial.print("NEW SETPT: ");
  Serial.println(Setpoint);

  int cmp = comparePositions(currentPos, newPos);
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

  //  if(firstSend) {
  //    firstSend = false;
  //    DEBUG_PRINTLN("Transmiting to ROCKBlock");
  //    char buf [200];
  //    modem.sendSBDText(buf);
  //    lastTransmit = loopTime;
  //  }
  //   Receive RockBlock Command as two bytes: command (char), and data (byte)
  //  uint8_t buffer[2];
  //  size_t bufferSize = sizeof(buffer);
  //  DEBUG_PRINTLN("Receiving RockBlock Commands.");
  //  modem.sendReceiveSBDText(NULL, buffer, bufferSize);
  //  DEBUG_PRINTLN("Turns it it doesn't block.");
  //
  //  if (loopTime - lastTransmit > ROCKBLOCK_TRANSMIT_TIME) {
  //    DEBUG_PRINTLN("Transmiting to ROCKBlock");
  //    char buf [200];
  //    dataStringBuffer.toCharArray(buf, sizeof(buf));
  //    modem.sendSBDText(buf);
  //    lastTransmit = loopTime;
  //  }

}




// Reads from all of the sensors and outputs the data string
String readSensors() {
  String dataString = "";

  // Timing
  long loopTime = millis();
  DEBUG_PRINT("Loop Time ");
  DEBUG_PRINTLN(loopTime);
  dataString += String(loopTime) + ", ";

  // BMP280 (Barometer + Thermometer) Input
  DEBUG_PRINTLN("BMP280 DATA");
  double tempIn = bmp.readTemperature();
  double pressure = bmp.readPressure();
  double alt = bmp.readAltitude(LAUNCH_SITE_PRESSURE); // avg sea level pressure for hollister for past month
  if (loopTime - lastAscentTime > BAROMETER_MEASURMENT_INTERVAL) { // calculate ascent rate in m/s every 10s
    ascentRate = (alt - lastAlt) * 1000 / (loopTime - lastAscentTime);
    lastAlt = alt;
    lastAscentTime = loopTime;
  }

  dataString += String(pressure) + ", " + String(alt) + ", ";
  dataString += String(ascentRate) + ", " + String(tempIn) + ", ";

  DEBUG_PRINT("Pressure: ");
  DEBUG_PRINTLN(pressure);
  DEBUG_PRINT("Altitude: ");
  DEBUG_PRINTLN(alt);
  DEBUG_PRINT("Ascent Rate: ");
  DEBUG_PRINTLN(ascentRate);
  DEBUG_PRINT("Inside Temperature: ");
  DEBUG_PRINTLN(tempIn);

  // BNO055 (IMU) Input
  DEBUG_PRINTLN("BNO055 Data");
  sensors_event_t event;
  bno.getEvent(&event);
  dataString += (String)event.orientation.x + ", " + (String)event.orientation.y + ", ";
  dataString += (String)event.orientation.z + ", ";
  DEBUG_PRINT("X-Direction: ");
  DEBUG_PRINTLN(event.orientation.x);
  DEBUG_PRINT("Y-Direction: ");
  DEBUG_PRINTLN(event.orientation.y);
  DEBUG_PRINT("Z-Direction: ");
  DEBUG_PRINTLN(event.orientation.z);

  // MAX31855 (Thermocouple) Input
  DEBUG_PRINTLN("MAX31855 Stuff");
  double temperature = thermocouple.readCelsius(); // celsius
  dataString += String(temperature) + ", ";
  DEBUG_PRINTLN(temperature);

  // GPS Input
  DEBUG_PRINTLN("GPS Stuff");

  while (Serial1.available()) {
    char c = Serial1.read();
    if (gps.encode(c)) {
      DEBUG_PRINTLN("Printing the encoded data!");
    }
  }
  f_lat = gps.location.lat();
  f_long = gps.location.lng();
  float f_alt = gps.altitude.meters();
  sats = gps.satellites.value();

  dataString += String(f_lat, 5) + ", " + String(f_long, 5) + ", ";
  dataString += String(f_alt) + ", " + String(sats) + ", ";
  DEBUG_PRINT("GPS Lat: ");
  DEBUG_PRINTLN(f_lat);
  DEBUG_PRINT("GPS Long: ");
  DEBUG_PRINTLN(f_long);
  DEBUG_PRINT("GPS Alt: ");
  DEBUG_PRINTLN(f_alt);
  DEBUG_PRINT("GPS Sats: ");
  DEBUG_PRINTLN(sats);
  DEBUG_PRINTLN("");

  dataStringBuffer = dataString;

  return dataString;
}

void flashLED() {
  while (false) {
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
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

int comparePositions(long currentPos, long newPos) {
  int diff = newPos - currentPos;
  if (abs(diff) <= SETPOINT_MARGIN_RADIUS) {
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
      digitalWrite(MOTOR_A_DIR_1, LOW); //CHECK IF THIS SS THE CORRECT DIRECTION, OTHERWISE, CHANGE!
      digitalWrite(MOTOR_A_DIR_2, HIGH);

      digitalWrite(MOTOR_B_DIR_1, HIGH);
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
      digitalWrite(MOTOR_A_DIR_1, HIGH);
      digitalWrite(MOTOR_A_DIR_2, LOW);

      digitalWrite(MOTOR_B_DIR_1, LOW);
      digitalWrite(MOTOR_B_DIR_2, HIGH);
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
