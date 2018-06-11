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
    dataFile.print("Time(ms), TempIn(C), Alt(m), Pressure(Pa), OrientationX(deg), y(deg) , z(deg), ");
    dataFile.println("GPSLat, GPSLong, GPSSats, Speed(mps), Satellites");
    dataFile.flush();
    /* dataFile.close(); */
  }

  return success;
}


int pass = 0;

void Log::writeSD(Sensors& sensors){

	/* if(pass) return; */
  /* dataFile = SD.open("datalog.txt", FILE_WRITE); */

  if (dataFile) {
    dataFile.println(sensors.readAllSensors());
    dataFile.flush();
    /* dataFile.close(); */
  }
  else{
	  Serial.println("SD CARD FAILURE");
	  pass = 1;
  }
  /* dataFile.println(sensors.readAllSensors()); */
  /* dataFile.close(); */

}
