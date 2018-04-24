/*
  Stanford Student Space Initiative
  Balloons | PARAFOIL | MARCH 2017
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
  /***********CONSTRUCTOR*********/
  //Sensors();

  /*************FUNCTIONS*********/
  bool initializeSensors();
  void flashLED();
  String readAllSensors();
  String dataString;

  //BMP280
  double getTemp();
  double getPressure();
  double getAlt();

  //BNO
  double getOrientationX();
  double getOrientationY();
  double getOrientationZ();

  //GPS
  double getLat();
  double getLon();
  double getSpeed();
  uint8_t getSats();

private:
  /*************OBJECTS***********/
  Adafruit_BMP280 bmp;
  Adafruit_BNO055 bno;
  TinyGPSPlus gps;

};

#endif
