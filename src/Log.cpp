/*
  Stanford Student Space Initiative
  Balloons | PARAFOIL | MARCH 2017
  File: Log.cpp
  --------------------------
  Implements "Log.h"
*/
#include "Log.h"



bool Log::initializeSD(Sensors& sensors){

  //Setting pins
  pinMode(SD_READER_CS, OUTPUT);
  SPI.setSCK(SCK_PIN);

  Serial.println("Initiliazing SD card");

  if (!SD.begin(SD_READER_CS)) { //If unable to begin, flash light
    while(true){
      Serial.println("Card failed, or not present");
      sensors.flashLED();
    }
    return false;
  }

  else{ //If able to communicate, print header on datalog file
    dataFile = SD.open("datalog.txt", FILE_WRITE);
    Serial.println("SD card intialized.");
    dataFile.print("Time(ms), TempIn(C), Alt(m), Pressure(Pa), OrientationX(deg), y(deg) , z(deg), ");
    dataFile.println("GPSLat, GPSLong, GPSSats, Speed(mps), Satellites");
    dataFile.close();
    return true;
  }
}

void Log::writeSD(Sensors& sensors){
  dataFile = SD.open("datalog.txt", FILE_WRITE);
  dataFile.println(sensors.readAllSensors());
  dataFile.close();
}
