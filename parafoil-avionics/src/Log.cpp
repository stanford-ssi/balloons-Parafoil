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

<<<<<<< HEAD
  else{ //If able to communicate, print header on datalog file
    dataFile = SD.open("datalog.txt", FILE_WRITE);
    Serial.println("SD card intialized.");
    dataFile.print("Time(ms), TempIn(C), Alt(m), Pressure(Pa), OrientationX(deg), y(deg) , z(deg), ");
    dataFile.println("GPSLat, GPSLong, GPSSats, Speed(mps), Satellites");
    dataFile.close();
    return true;
=======
  else{
    /* dataFile = SD.open("datalog.txt", FILE_WRITE); */
    /* Serial.println("SD card intialized."); */
    /* dataFile.print("Time(ms), TempIn(C), Alt(m), Pressure(Pa), OrientationX(deg), y(deg) , z(deg), "); */
    /* dataFile.println("GPSLat, GPSLong, GPSSats, Speed(mps), Satellites"); */
    /* dataFile.flush(); */
    /* dataFile.close(); */
>>>>>>> 15d9e6a0f2137215b9e0d90c596e49b3f861a608
  }
}


int pass = 0;

void Log::writeSD(Sensors& sensors){
<<<<<<< HEAD
  dataFile = SD.open("datalog.txt", FILE_WRITE);
  dataFile.println(sensors.readAllSensors());
  dataFile.close();
=======

	/* if(pass) return; */
  File dataFilea = SD.open("datalog.txt", FILE_WRITE);

  if (dataFilea) {
    /* dataFilea.println(sensors.readAllSensors()); */
    dataFilea.println("Testing Strign!");
    dataFilea.flush();
    dataFilea.close();
  }
  else{
	  Serial.println("SD CARD FAILURE");
	  pass = 1;
  }
  /* dataFile.println(sensors.readAllSensors()); */
  /* dataFile.close(); */

>>>>>>> 15d9e6a0f2137215b9e0d90c596e49b3f861a608
}
