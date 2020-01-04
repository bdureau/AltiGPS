#ifndef _CONFIG_H
#define _CONFIG_H

#define MAJOR_VERSION 1
#define MINOR_VERSION 0
#define CONFIG_START 32
#define BOARD_FIRMWARE "AltiGPS"
#include "Arduino.h"
//used for writing in the microcontroler internal eeprom
#include <EEPROM.h>

struct ConfigStruct {
  int unit;             //0 = meter 1 = feet
  int beepingMode;      // decide which way you want to report the altitude
  int outPut1;          // assign a function to each pyro
  int outPut2;
  int outPut3;
  int mainAltitude;     //deployment altitude for the main chute
  int superSonicYesNo;  // if set to yes do not do any altitude measurement when altimeter starts
  int outPut1Delay;      // delay output by x ms
  int outPut2Delay;
  int outPut3Delay;
  int beepingFrequency;  // this beeping frequency can be changed
  int nbrOfMeasuresForApogee; //how many measure to decide that apogee has been reached
  int endRecordAltitude;  // stop recording when landing define under which altitude we are not recording
  int recordTemperature;  //decide if we want to record temperature
  int superSonicDelay;   //nbr of ms during when we ignore any altitude measurements
  long connectionSpeed;   //altimeter connection baudrate
  int altimeterResolution; // BMP sensor resolution
  int eepromSize;
  int noContinuity;
  
  int outPut4;
  int outPut4Delay;
 
  int cksum;  
//  int cksum;  
};
extern ConfigStruct config;
extern void defaultConfig();
extern boolean readAltiConfig();
extern void writeConfigStruc();
extern unsigned int CheckSumConf( ConfigStruct );
extern bool writeAltiConfig( char *p );
#endif
