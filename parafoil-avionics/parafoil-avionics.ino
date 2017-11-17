#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <TinyGPS++.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>


#define DEBUG // comment out to turn off debugging
#ifdef DEBUG
  #define DEBUG_PRINT(x)  Serial.print (String(x))
  #define DEBUG_PRINTLN(x)  Serial.println (String(x))
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

//Teensy LED
#define LED_PIN 13

//SPI Ports
#define SCK_PIN 13
#define SD_READER_CS 10

//SD CARD
#define SD_CARD_FLUSH_TIME 10000
File dataFile;
long lastFlush;

//BMP280
Adafruit_BMP280 bmp;
long lastAscentTime;
double lastAlt;
double ascentRate;
#define BAROMETER_MEASURMENT_INTERVAL 10000 // 10 Seconds
#define LAUNCH_SITE_PRESSURE 1014.562

//GPS 
TinyGPSPlus gps;
float f_lat = 0, f_long = 0;
int sats = -1;
long unsigned f_age = 0;

void setup() {
  lastFlush = 0;

  #ifdef DEBUG
    Serial.begin(9600);
    while(Serial){
      continue;
    }
    #endif
 
  //GPS SERIAL 1
  Serial1.begin(9600);

  //SD CARD READER SET-UP
  SPI.setSCK(SCK_PIN);
  pinMode(SD_READER_CS, OUTPUT);
  if(!SD.begin(SD_READER_CS)){
    DEBUG_PRINTLN("Card failed, or not present");
  }
  //dataFile= SD.open("datalog.txt", FILE_WRITE);
  DEBUG_PRINTLN("Card initialized");


  //BMP280
  if(!bmp.begin()){
    DEBUG_PRINTLN("Could not find a BMP280 sensor, check wiring");
  }
  lastAlt = bmp.readAltitude(LAUNCH_SITE_PRESSURE);
  ascentRate = 0;
  lastAscentTime = millis();

  dataFile.print("Time(ms), Pressure(Pa), Alt(m), AscentRate(m/s), TempIn(C), OrientationX(deg), y(deg) , z(deg), ");
  dataFile.println("TempOut(C), GPSLat, GPSLong, GPSAge, GPSSats, RBSigalQuality");

}

void loop() {
  dataFile= SD.open("datalog.txt", FILE_WRITE);
 
  String dataString = readSensors();
  DEBUG_PRINTLN(dataString);
  dataFile.println(dataString);
  dataFile.close();
 // flashLED();
}

String readSensors() {
  String dataString = "";

  //Timing
  long loopTime = millis();
  DEBUG_PRINTLN("Loop time ");
  DEBUG_PRINTLN(loopTime);
  dataString += "Loop Time: " + String(loopTime)+ ", ";

  double temp = bmp.readTemperature();
  double pressure = bmp.readPressure();
  double alt = bmp.readAltitude(LAUNCH_SITE_PRESSURE);
  
  if (loopTime - lastAscentTime > BAROMETER_MEASURMENT_INTERVAL) { // calculate ascent rate in m/s every 10s
    ascentRate = (alt - lastAlt) * 1000 / (loopTime - lastAscentTime);
    lastAlt = alt;
    lastAscentTime = loopTime;
  }
  
  dataString += "Pressure: " + String(pressure) + ", " + " Altitutde: " + String(alt) + ", ";
  dataString += "Ascent Rate: " + String(ascentRate) + ", " + "Temperature: " + String(temp) + ", ";

  DEBUG_PRINTLN("GPS DATA: ");

  while (Serial1.available()){
    char c = Serial1.read();
    if(gps.encode(c)){
      DEBUG_PRINTLN("Printing the encoded data!");
    }
  }
  
  f_lat = gps.location.lat();
  f_long = gps.location.lng();
  f_age = gps.location.age();
  sats = gps.satellites.value();

  dataString += "LATITUDE: " + String(f_lat) + ", LONGITUDE: " + String(f_long) + ", ";
  dataString += "AGE: " + String(f_age) + ", " + "NUMBER OF SATS: " + String(sats) + ", ";

  DEBUG_PRINTLN("LATITUDE: " + String(f_lat));
  DEBUG_PRINT("LONGITUDE: " + String(f_long));
  Serial.println(f_long,4);
  DEBUG_PRINTLN("AGE: " + String(f_age));
  DEBUG_PRINTLN("NUMBER OF SATELLITES: " + String(sats));
  
  return dataString;
}





