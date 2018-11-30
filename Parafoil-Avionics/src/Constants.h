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

/********************************SENSORS***************************************/
//BNO


//BMP
const static float LAUNCH_SITE_PRESSURE = 100744.95;
const static int BAROMETER_MEASURMENT_INTERVAL = 10000;


//LED
const static uint8_t LED_PIN = 22;
const static uint8_t GPS_LED = 4;
const static uint8_t SD_LED = 5;
const static uint8_t BNO_LED = 6;

/*******************************SD CARD READER*********************************/
const static uint8_t DIN_PIN = 12;
const static uint8_t DOUT_PIN = 11;
const static uint8_t SD_READER_CS = 10;

/************************************GPS***************************************/

/*********************************CUTDOWN**************************************/
const static int CUTDOWN_ALT = 300; //equivalent ot 500ft
const static uint8_t WIRE = 9;
const static uint8_t RELEASE_TIME = 3; //3 sec release time



/*******************************MOTORS*****************************************/
const static int SETPOINT_MARGIN_RADIUS = 3000;

const static uint8_t MOTOR_1_DIR_1 = 23;
const static uint8_t MOTOR_1_DIR_2 = 21;
const static uint8_t MOTOR_1_SPEED = 22;
const static int ENCODER_1_A = 2;
const static int ENCODER_1_B = 3;



const static uint8_t MOTOR_2_DIR_1 = 20;
const static uint8_t MOTOR_2_DIR_2 = 18;
const static uint8_t MOTOR_2_SPEED = 19;
const static int ENCODER_2_A = 14;
const static int ENCODER_2_B = 15;


const static int TIME_STEP = 10000;
const static int TURN_RADIUS = 5000;
const static int MOTOR_SPEED = 150;
const static int NEUTRAL = 0;

/*******************************FLIGHT PATH************************************/

const static int BANK_LEFT_TIME = 30000;
const static int BANK_RIGHT_TIME = 30000;
const static int FORWARD_FLIGHT_TIME = 30000;

#endif
