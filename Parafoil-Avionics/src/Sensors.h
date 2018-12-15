/*
  Stanford Student Space Initiative
  Balloons | BALLOONERANG | DECEMBER 2018
  File: Sensors.h
  --------------------------
  Interface to sensors on avionics hardware.
*/
#ifndef SENSORS_H
#define SENSORS_H

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BNO055.h>
#include "TinyGPS++.h"

#include "Constants.h"

class Sensors{
public:
  /********************************CONSTRUCTOR*********************************/

  /********************************FUNCTIONS***********************************/
  bool initializeSensors();
  void flashLED();
  String readAllSensors();
  void smartDelay(unsigned long ms);
  String dataString;

  //BMP280
  double getTemp();
  double getPressure();
  double getAlt();
  double getAscentRate();

  //BNO
  double getOrientationX();
  double getOrientationY();
  double getOrientationZ();

  //GPS
  double getLat();
  double getLon();
  double getSpeed();
  double getGPSAlt();
  uint8_t getSats();
  void gpsUpdate();

private:
 long lastAscentTime; // Time of last ascent rate calculation
 double lastAlt; // Last altitude, for calculation
 double ascentRate; // Last calculated rate, to fill forward in logging

  /***********************************OBJECTS**********************************/
  Adafruit_BMP280 bmp;
  Adafruit_BNO055 bno;
  TinyGPSPlus gps;

};

#endif
