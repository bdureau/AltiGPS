#ifndef _GLOBAL_H
#define _GLOBAL_H
#define LED_PIN PC13 //pin 13 for the arduino Uno and PC13 for the stm32 
//used for writing in the microcontroler internal eeprom
#include <EEPROM.h>

#include <I2Cdev.h>

#include <Wire.h>

#include "Bear_BMP085.h"

BMP085 bmp;
//bool blinkState = true;
//bool telemetryEnable = false;
bool mainLoopEnable = true;

//long last_telem_time=0;
//#include <nmea.h>
//NMEA gps(ALL);    // GPS data connection to all sentence types
#include <TinyGPS++.h>
TinyGPSPlus gps;
#endif
