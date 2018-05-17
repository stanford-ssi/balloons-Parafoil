/*
  Stanford Student Space Initiative
  Balloons | PARAFOIL | MARCH 2017
  File: Constants.h
  --------------------------
  This file defines all global constants
*/

#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <Arduino.h>
#include <stdint.h>
#include <string.h>
#include <Encoder.h>

/***********************SENSORS***************************/
//BNO


//BMP
const static float LAUNCH_SITE_PRESSURE = 1014.562;
const static int BAROMETER_MEASURMENT_INTERVAL = 10000;


//LED
const static uint8_t LED_PIN = 22;

/********************SD CARD READER**********************/
const static uint8_t DIN_PIN = 12; //SPI DIN (MISO). (Thermocouple + SD card reader)
const static uint8_t DOUT_PIN = 11;
const static uint8_t SD_READER_CS = 20;

/************************GPS*****************************/

/***********************CUTDOWN***************************/
const static int CUTDOWN_ALT = 152; //equivalent ot 500ft
const static uint8_t WIRE = 19;
const static uint8_t RELEASE_TIME = 10; //10 sec release time

const static uint8_t AILERON_IN = 6; // INTERRUPT 2 = DIGITAL PIN 2 - use the interrupt number in attachInterrupt


const static uint16_t  MIN_AILERON = 1000; //minimum PWM on aileron stick
const static uint16_t NEUTRAL_AILERON = 1500; // neutral PWM on the aileron stick
const static uint16_t  MAX_AILERON = 2000;  //maximum PWM on aileron stick


#endif
