#ifndef _CONFIG_H
#define _CONFIG_H

#define MAJOR_VERSION 1
#define MINOR_VERSION 0
#define CONFIG_START 32
#define BOARD_FIRMWARE "AltiGPS"
#define BAT_MIN_VOLTAGE 7.0
//Voltage divider
#define R1 4.7
#define R2 10

#define VOLT_DIVIDER 10*(R1/(R1+R2))
#include "Arduino.h"
//used for writing in the microcontroler internal eeprom
#include <EEPROM.h>
#include <itoa.h>

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
  int liftOffAltitude; //Lift off Altitude in meters
  int batteryType; // 0= Unknown, 1= "2S (7.4 Volts)", 2 = "9 Volts",3 = "3S (11.1 Volts)
  int cksum;   
};

extern ConfigStruct config;
extern void defaultConfig();
extern bool readAltiConfig();
extern void writeConfigStruc();
extern unsigned int CheckSumConf( ConfigStruct );
extern unsigned int msgChk( char * buffer, long length );
extern bool writeAltiConfig( char *p );
#define SerialCom Serial1
#define SerialGPS Serial3
//#include "avdweb_VirtualDelay.h"

//pyro out 1
extern const int pyroOut1;
extern int pinApogee;
//pyro out 2
extern const int pyroOut2;
extern int pinMain;
//pyro out 3
extern const int pyroOut3;
extern int pinOut3;
//pyro out 4
extern const int pyroOut4;
extern int pinOut4;


extern int pinOut2;
extern int pinOut1;

extern int continuityPins[4];
#endif
