/*
  Stanford Student Space Initiative
  Balloons | PARAFOIL | MARCH 2017
  File: Sensors.cpp
  --------------------------
  Implements "Sensors.h"
*/

#include "Sensors.h"

bool Sensors::initializeSensors(){
  bool success = true;

  pinMode(LED_PIN, OUTPUT);

  if(!bmp.begin()){
    Serial.println("BMP280 could not be initialized.  Check wiring!");
    success = false;
  }

  else if(!bno.begin()){
    Serial.println("BNO could not be initialized. Check wiring!");
    success = false;
  }
  bno.setExtCrystalUse(true); /* Use external crystal for better accuracy */

  Serial1.begin(9600);
  if(!Serial1.available()){
    Serial.println("GPS could not be initliazed. Check wiring!");
    success = false;
  }
  return success;
}

double Sensors::getTemp(){
  double tempIn = bmp.readTemperature(); //create new variable each time method is called?
  return tempIn;
}

double Sensors::getPressure(){
  double pressure = bmp.readPressure();//create new variable each time method is called?
  return pressure;
}

double Sensors::getAlt(){
  double alt = bmp.readAltitude(LAUNCH_SITE_PRESSURE);//create new variable each loop?
  return alt;
}

double Sensors::getOrientationX(){
  sensors_event_t event;
  bno.getEvent(&event);
  double orientationX = (double)event.orientation.x;
  return orientationX;
}

double Sensors::getOrientationY(){
  sensors_event_t event;
  bno.getEvent(&event);
  double orientationY = (double)event.orientation.y;
  return orientationY;
}

double Sensors::getOrientationZ(){
  sensors_event_t event;
  bno.getEvent(&event);
  double orientationZ = (double)event.orientation.z;
  return orientationZ;
}

void Sensors::flashLED(){
  while (true) {
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
  }
}

double Sensors::getLat(){
  double latitude = 0.0;
  if(Serial1.available()){
    char data = Serial1.read();
    if(gps.encode(data)){
      latitude = gps.location.lat();
    }
  }
  return latitude;
}

double Sensors::getLon(){
  double longitude = 0.0;
  if(Serial1.available()){
    char data = Serial1.read();
    if(gps.encode(data)){
      longitude = gps.location.lng();
    }
  }
  return longitude;
}

uint8_t Sensors::getSats(){
  uint8_t sats = -1;
  if(Serial1.available()){
    char data = Serial1.read();
    if(gps.encode(data)){
      sats = gps.satellites.value();
    }
  }
  return sats;
}

String Sensors::readAllSensors(){
  dataString = "";
  dataString += String(getTemp());
  dataString += String(getAlt());
  dataString += String(getPressure());
  dataString += String(getOrientationX());
  dataString += String(getOrientationY());
  dataString += String(getOrientationZ());
  dataString += String(getLat());
  dataString += String(getLon());
  dataString += String(getSats());
  Serial.println(dataString);
  return dataString;
}
