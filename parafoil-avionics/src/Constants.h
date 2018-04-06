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
#include <Encoder.h>

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


const static uint8_t AILERON_SIGNAL = 6; // INTERRUPT 2 = DIGITAL PIN 2 - use the interrupt number in attachInterrupt

const static uint8_t MOTOR_A_DIR_1 = 0;
const static uint8_t MOTOR_A_DIR_2 = 1;
const static uint8_t MOTOR_A_SPEED = 14;  //Analog pin for motor speed

const static uint8_t MOTOR_B_DIR_1 = 2;
const static uint8_t MOTOR_B_DIR_2 = 3;
const static uint8_t MOTOR_B_SPEED = 15; //Analog pin for motor speed

const static uint8_t  ENCODER_A_1 = 4;
const static uint8_t  ENCODER_A_2 = 5;

const static uint8_t  ENCODER_B_1 = 9;
const static uint8_t ENCODER_B_2 = 10;

const static uint16_t  MIN_AILERON = 1000; //minimum PWM on aileron stick
const static uint16_t NEUTRAL_AILERON = 1500; // neutral PWM on the aileron stick
const static uint16_t  MAX_AILERON = 2000;  //maximum PWM on aileron stick

const static int16_t MIN_MOTOR = -4000;  //minimum number of steps on motor from neutral
const static uint8_t  NEUTRAL_MOTOR = 0;
const static uint16_t  MAX_MOTOR = 4000; //maximum number of steps on motor from neutral

const static uint8_t SETPOINT_MARGIN_RADIUS = 10; //threshold for noise

//EDITABLE CONSTANTS
