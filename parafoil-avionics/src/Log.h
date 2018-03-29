/*
  Stanford Student Space Initiative
  Balloons | PARAFOIL | MARCH 2017
  File: Log.h
  --------------------------
  Logs current state to SD card
*/

#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include "Sensors.h"

class Log{
public:
  bool initializeSD();
  void writeSD(Sensors& sensors);




private:

  File dataFile;
};
