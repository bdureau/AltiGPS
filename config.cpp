#include "config.h"
ConfigStruct config;
//================================================================
// read and write in the microcontroler eeprom
//================================================================
const int pyroOut1 = PA1;
const int pyroOut2 = PA3;
//int pinMain = PA3;
const int pyroOut3 = PA5; 
const int pyroOut4 = PA7;
int continuityPins[4];

void defaultConfig()
{
  config.unit = 0;
  config.beepingMode = 0;
  config.outPut1 = 0;
  config.outPut2 = 1;
  config.outPut3 = 3;
  config.outPut1Delay = 0;
  config.outPut2Delay = 0;
  config.outPut3Delay = 0;
  config.mainAltitude = 50;
  config.superSonicYesNo = 0;
  config.beepingFrequency = 440;
  //config.separationVelocity = 10;
  config.nbrOfMeasuresForApogee = 5;
  config.endRecordAltitude = 3; // stop recording when landing define under which altitude we are not recording
  config.recordTemperature = 0; //decide if we want to record temperature
  config.superSonicDelay = 0;
  config.connectionSpeed = 38400;
  config.altimeterResolution = 0; //0 to 4 ie: from low resolution to high
  config.eepromSize = 512;
  config.noContinuity = 0;
  config.outPut4 = 3;
  config.outPut4Delay = 0;
  config.liftOffAltitude = 10;
  config.batteryType = 0; // 0= Unknown, 1= "2S (7.4 Volts)", 2 = "9 Volts",3 = "3S (11.1 Volts)
  config.recordingTimeout = 120;
  config.altiID = 0;
  config.useTelemetryPort =0;
  config.cksum = CheckSumConf(config);
}

bool readAltiConfig() {
  //set the config to default values so that if any have not been configured we can use the default ones
  defaultConfig();
  int i;
  for ( i = 0; i < sizeof(config); i++ ) {
    *((char*)&config + i) = EEPROM.read(CONFIG_START + i);
  }

  if ( config.cksum != CheckSumConf(config) ) {
    return false;
  }

  return true;

}

/*
  write the config received by the console

*/

bool writeAltiConfigV2( char *p ) {

  char *str;
  int i = 0;
  int command =0;
  long commandVal =0;
  int strChk = 0;
  char msg[100] = "";

  while ((str = strtok_r(p, ",", &p)) != NULL) // delimiter is the comma
  {
    //SerialCom.println(str);
    if (i == 1) {
      command = atoi(str);
      strcat(msg, str);
    }
    if (i == 2) {
      commandVal =  atol(str);
      strcat(msg, str);
    }
    if (i == 3) {
      strChk  =  atoi(str);  
    }
    i++;

  }
    //we have a partial config
  if (i < 4)
    return false;
  //checksum is ivalid ? 
  if (msgChk(msg, sizeof(msg)) != strChk)
    return false;  
    
  switch (command)
    {
      case 1:
        config.unit = (int) commandVal;
        break;
      case 2:
        config.beepingMode = (int) commandVal;
        break;
      case 3:
        config.outPut1 = (int) commandVal;
        break;
      case 4:
        config.outPut2 = (int) commandVal;
        break;
      case 5:
        config.outPut3 = (int) commandVal;
        break;
      case 6:
        config.mainAltitude = (int) commandVal;
        break;
      case 7:
        config.superSonicYesNo = (int) commandVal;
        break;
      case 8:
        config.outPut1Delay = (int) commandVal;
        break;
      case 9:
        config.outPut2Delay = (int) commandVal;
        break;
      case 10:
        config.outPut3Delay = (int) commandVal;
        break;
      case 11:
        config.beepingFrequency = (int) commandVal;
        break;
      case 12:
        config.nbrOfMeasuresForApogee = (int) commandVal;
        break;
      case 13:
        config.endRecordAltitude = (int) commandVal;
        break;
      case 14:
        config.recordTemperature = (int) commandVal;
        break;
      case 15:
        config.superSonicDelay = (int) commandVal;
        break;
      case 16:
        config.connectionSpeed = commandVal;
        break;
      case 17:
        config.altimeterResolution = (int) commandVal;
        break;
      case 18:
        config.eepromSize = (int) commandVal;
        break;
      case 19:
        config.noContinuity = (int) commandVal;
        break;
      case 20:
        config.outPut4 = (int) commandVal;
        break;
      case 21:
        config.outPut4Delay = (int) commandVal;
        break;
      case 22:
        config.liftOffAltitude = (int) commandVal;
        break;
      case 23:
        config.batteryType = (int) commandVal;
        break;
      case 24:
        config.recordingTimeout = (int) commandVal;
        break;
      case 25:
        config.altiID = (int)commandVal;
        break;
      case 26:
        config.useTelemetryPort = (int)commandVal;  
        break;     
    }

  // add checksum
  config.cksum = CheckSumConf(config);

  return true;
}
/*

   Write config structure to the EEPROM

*/
void writeConfigStruc()
{
  int i;
  for ( i = 0; i < sizeof(config); i++ ) {
    EEPROM.write(CONFIG_START + i, *((char*)&config + i));
  }
}

/*
   Calculate Checksum for the config
*/
unsigned int CheckSumConf( ConfigStruct cnf)
{
  int i;
  unsigned int chk = 0;

  for (i = 0; i < (sizeof(cnf) - sizeof(int)); i++)
    chk += *((char*)&cnf + i);

  return chk;
}

unsigned int msgChk( char * buffer, long length ) {

  long index;
  unsigned int checksum;

  for ( index = 0L, checksum = 0; index < length; checksum += (unsigned int) buffer[index++] );
  return (unsigned int) ( checksum % 256 );

}
