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
const static int CUTDOWN_ALT = 300; //equivalent ot 500ft
const static uint8_t WIRE = 14;
const static uint8_t RELEASE_TIME = 3; //3 sec release time




/*********************MOTORS*****************************/
const static int SETPOINT_MARGIN_RADIUS = 10;

const static uint8_t MOTOR_A_DIR_1 = 1;
const static uint8_t MOTOR_A_DIR_2 = 2;
const static uint8_t MOTOR_B_DIR_1 = 1;
const static uint8_t MOTOR_B_DIR_2 = 2;

const static uint8_t MOTOR_A_SPEED = 2;
const static uint8_t MOTOR_B_SPEED = 1;

const static int ENCODER_A_1 = 5;
const static int ENCODER_A_2 = 6;
const static int ENCODER_B_1 = 7;
const static int ENCODER_B_2 = 8;

const static int MIN_MOTOR = -4000;
const static int MAX_MOTOR = 4000;
const static int NEUTRAL = 0;

/************FLIGHT PATH********************/

const static int BANK_LEFT_TIME = 30000;
const static int BANK_RIGHT_TIME = 30000;
const static int FORWARD_FLIGHT_TIME = 10000;

#endif
