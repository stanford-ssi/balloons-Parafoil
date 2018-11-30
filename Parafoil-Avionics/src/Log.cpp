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

  else{
    dataFile = SD.open("datalog.txt", FILE_WRITE);
    Serial.println("SD card intialized.");
    dataFile.print("Time(ms), TempIn(C), Alt(m), Pressure(Pa), OrientationX(deg), y(deg) , z(deg), ");
    dataFile.println("GPSLat, GPSLong, GPSSats, Speed(mps), Satellites, Enc1 Pos, Enc2 Pos, Manuever");

    dataFile.flush();
    dataFile.close();

  }
}


int pass = 0;

void Log::writeSD(Sensors& sensors, long start, long pos1, long pos2){
  dataFile = SD.open("datalog.txt", FILE_WRITE);
  int state = ( (millis() - start)/TIME_STEP ) % 4;
  dataFile.print(sensors.readAllSensors());

  dataFile.print(" ");
  dataFile.print(String(pos1));
  dataFile.print(" ");
  dataFile.print(String(pos2));

  Serial.print(" ");
  Serial.print(String(pos1));
  Serial.print(" ");
  Serial.print(String(pos2));

  if ( state == 0){
    Serial.println(" FLY FORWARD");
    dataFile.print(" ");
    dataFile.println("FLY FORWARD");
    //forwardFlight();
  }
  else if( state == 1){
    Serial.println(" BANK LEFT");
    dataFile.print(" ");
    dataFile.println("BANK LEFT");
  //  bankLeft();
  }

  else if( state == 2){
    Serial.println(" FLY FORWARD");
    dataFile.print(" ");
    dataFile.println("FLY FORWARD");
  //  forwardFlight();
  }

  else if( state == 3){
    Serial.println(" BANK RIGHT");
    dataFile.print(" ");
    dataFile.println(" BANK RIGHT");
//    bankRight();
  }
  dataFile.close();

}
