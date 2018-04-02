/*
  Stanford Student Space Initiative
  Balloons | PARAFOIL | MARCH 2017
  File: Log.cpp
  --------------------------
  Implements "Log.h"
*/
#include "Log.h"



bool Log::initializeSD(Sensors& sensors){
  bool success = true;
  pinMode(SD_READER_CS, OUTPUT);
  SPI.setSCK(SCK_PIN);
  Serial.println("Initiliazing SD card");

  if (!SD.begin(SD_READER_CS)) {
    while(true){
      Serial.println("Card failed, or not present");
      sensors.flashLED();
    }


    success = false;
  }
  else{
    dataFile = SD.open("datalog.txt", FILE_WRITE);
    Serial.println("SD card intialized.");
    dataFile.print("Time(ms), Pressure(Pa), Alt(m), AscentRate(m/s), TempIn(C), OrientationX(deg), y(deg) , z(deg), ");
    dataFile.println("GPSLat, GPSLong, GPSSats, RBSigalQuality");
  }

  return success;
}

void Log::writeSD(Sensors& sensors){
  dataFile.println(sensors.readAllSensors());

}
