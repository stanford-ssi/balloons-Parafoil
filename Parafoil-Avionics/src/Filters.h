

#ifndef FILTERS_H
#define FILTERS_H

#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include "Sensors.h"
#include "Utils.h"

class Filters{
public:
  /***********************************FUNCTIONS********************************/
  bool initialize();
   void filter();
AdjustableLowpass lowpassfilter;
IntervalTimer timer;


private:
  /************************************OBJECTS*********************************/



};

#endif
