/*
  Stanford Student Space Initiative
  Balloons | PARAFOIL | MARCH 2017
  File: Constants.h
  --------------------------
  This file defines all global constants
*/

#include <Arduino.h>
#include <stdint.h>
#include <string.h>

/***********************SENSORS***************************/
//BNO


//BMP
const static float LAUNCH_SITE_PRESSURE = 1014.562;

//LED
const static uint8_t LED_PIN = 22;

/********************SD CARD READER**********************/
const static uint8_t DIN_PIN = 12; //SPI DIN (MISO). (Thermocouple + SD card reader)
const static uint8_t DOUT_PIN = 11;
const static uint8_t SD_READER_CS = 20;

/************************GPS*****************************/



/******************MOTORS + ENCODERS*********************/

//EDITABLE CONSTANTS
