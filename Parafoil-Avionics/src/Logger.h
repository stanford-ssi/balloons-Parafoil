/*
  Stanford Student Space Initiative
  Balloons | PARAFOIL | MARCH 2017
  File: Log.h
  --------------------------
  Logs current state to SD card
*/
#ifndef LOG_H
#define LOG_H
#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include "Sensors.h"

class Logger{
public:
  /***********************************FUNCTIONS********************************/
  bool initializeSD(Sensors& sensors);
  void writeSD(Sensors& sensors, long start, long pos1, long pos2);

private:
  /************************************OBJECTS*********************************/
  File dataFile;
};

#endif
