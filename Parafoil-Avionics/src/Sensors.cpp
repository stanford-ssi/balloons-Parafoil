/*
  Stanford Student Space Initiative
  Balloons | PARAFOIL | MARCH 2017
  File: Sensors.cpp
  --------------------------
  Implements "Sensors.h"
*/

#include "Sensors.h"

/*
 *Function: initializeSensors()
 *This function initializes the BMP280, BNO, and GPS.
 */
bool Sensors::initializeSensors(){

  //Setting pins
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(5000);

  Wire.begin();
  CORE_PIN18_CONFIG = 0;  // turn off primary pins before enable alternates
  CORE_PIN19_CONFIG = 0;
  CORE_PIN16_CONFIG = PORT_PCR_MUX(2)|PORT_PCR_ODE|PORT_PCR_SRE|PORT_PCR_DSE;
  CORE_PIN17_CONFIG = PORT_PCR_MUX(2)|PORT_PCR_ODE|PORT_PCR_SRE|PORT_PCR_DSE;
  Wire.setSDA(17); Wire.setSCL(16);
  if(!bmp.begin()){ //If unable to begin, flash light
    while(true){
      Serial.println("BMP280 could not be initialized.  Check wiring!");
      flashLED();
    }
    return false;
  }

  if(!bno.begin()){ //If unable to begin, flash light
    while(true){
      Serial.println("BNO could not be initialized.  Check wiring!");
      flashLED();
    }
    return false;
  }

  //bno.setExtCrystalaUse(true); /* Use external crystal for better accuracy */

  Serial1.begin(9600); //Begins serial communication with GPS
  delay(1000);

  if(!Serial1.available()){ //If unable to begin, flash light
    while(true){
      Serial.println("GPS could not be initliazed. Check wiring!");
      flashLED();
    }
    return false;
  }

 lastAscentTime = 0;             // Time of last ascent rate calculation
 lastAlt = 0.0;                 // Last altitude, for calculation
 ascentRate = 0.0;              // Last calculated rate, to fill forward in logging
 return true;
}

double Sensors::getTemp(){
  return bmp.readTemperature();
}

double Sensors::getPressure(){
  return bmp.readPressure();
}

double Sensors::getAlt(){
  return bmp.readAltitude(LAUNCH_SITE_PRESSURE);
}

double Sensors::getAscentRate(){
  long looptime = millis();
  double ascentRate = 0.0;
  if( looptime - lastAscentTime > BAROMETER_MEASURMENT_INTERVAL){
    double alt = getAlt();
    ascentRate = (alt - lastAlt) * 1000 / (looptime - lastAscentTime);
    lastAlt = alt;
    lastAscentTime = looptime;
  }
  return ascentRate;
}

double Sensors::getOrientationX(){
  sensors_event_t event;
  bno.getEvent(&event);
  double orientationX = (double)event.orientation.x;
  return orientationX;
}

double Sensors::getOrientationY(){
  sensors_event_t event;
  bno.getEvent(&event);
  double orientationY = (double)event.orientation.y;
  return orientationY;
}

double Sensors::getOrientationZ(){
  sensors_event_t event;
  bno.getEvent(&event);
  double orientationZ = (double)event.orientation.z;
  return orientationZ;
}

void Sensors::flashLED(){
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
}

double Sensors::getLat(){
  return gps.location.lat();
}

double Sensors::getLon(){
  return gps.location.lng();
}

uint8_t Sensors::getSats(){
  uint8_t sats = -1;
  return sats;
}

double Sensors::getSpeed(){
  double mps = gps.speed.mps();
  return mps;
}

double Sensors::getGPSAlt(){
  double altitude = gps.altitude.meters();
  return altitude;
}

void Sensors::smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do{ //Continues to read Serial1 data for ms seconds
    while (Serial1.available()){
      gps.encode(Serial1.read());
    }
  } while (millis() - start < ms);
}

String Sensors::readAllSensors(){
  dataString = "";
  dataString += String(millis());
//
  bmp.begin(); //BEGIN EVERY LOOP
//  bno.begin();
  dataString += " " + String(bmp.readTemperature());
  dataString += " " + String(getAlt());
  dataString += " " + String(getPressure());
  dataString += " " + String(getOrientationX());
  dataString += " " + String(getOrientationY());
  dataString += " " + String(getOrientationZ());
  dataString += " " + String(getLat(),6);
  dataString += " " + String(getLon(),6);
  dataString += " " + String(getSpeed());
  dataString += " " + String(getGPSAlt());
  dataString += " " + String(getSats());

  Serial.println(dataString);
  return dataString;
}
