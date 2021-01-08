#include "config.h"
ConfigStruct config;
//================================================================
// read and write in the microcontroler eeprom
//================================================================
const int pyroOut1 = PA1;//9;
int pinApogee = PA1;//9;
const int pyroOut2 = PA3;
int pinMain = PA3;
const int pyroOut3 = PA5; //17;
int pinOut3 = PA5;//17;
const int pyroOut4 = PA7;
int pinOut4 = PA7;
int pinOut2 = -1;
int pinOut1 = -1;
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
  config.cksum = CheckSumConf(config);
}

unsigned int CheckSumConf( ConfigStruct cnf)
{
  int i;
  unsigned int chk = 0;

  for (i = 0; i < (sizeof(cnf) - sizeof(int)); i++)
    chk += *((char*)&cnf + i);

  return chk;
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
bool writeAltiConfig( char *p ) {

  char *str;
  int i = 0;
  int strChk = 0;
  char msg[120] = "";

  while ((str = strtok_r(p, ",", &p)) != NULL) // delimiter is the comma
  {
    //SerialCom.println(str);
    switch (i)
    {
      case 1:
        config.unit = atoi(str);
        strcat(msg, str);
        break;
      case 2:
        config.beepingMode = atoi(str);
        strcat(msg, str);
        break;
      case 3:
        config.outPut1 = atoi(str);
        strcat(msg, str);
        break;
      case 4:
        config.outPut2 = atoi(str);
        strcat(msg, str);
        break;
      case 5:
        config.outPut3 = atoi(str);
        strcat(msg, str);
        break;
      case 6:
        config.mainAltitude = atoi(str);
        strcat(msg, str);
        break;
      case 7:
        config.superSonicYesNo = atoi(str);
        strcat(msg, str);
        break;
      case 8:
        config.outPut1Delay = atol(str);
        strcat(msg, str);
        break;
      case 9:
        config.outPut2Delay = atol(str);
        strcat(msg, str);
        break;
      case 10:
        config.outPut3Delay = atol(str);
        strcat(msg, str);
        break;
      case 11:
        config.beepingFrequency = atoi(str);
        strcat(msg, str);
        break;
      case 12:
        config.nbrOfMeasuresForApogee = atoi(str);
        strcat(msg, str);
        break;
      case 13:
        config.endRecordAltitude = atol(str);
        strcat(msg, str);
        break;
      case 14:
        config.recordTemperature = atoi(str);
        strcat(msg, str);
        break;
      case 15:
        config.superSonicDelay = atoi(str);
        strcat(msg, str);
        break;
      case 16:
        config.connectionSpeed = atol(str);
        strcat(msg, str);
        break;
      case 17:
        config.altimeterResolution = atoi(str);
        strcat(msg, str);
        break;
      case 18:
        config.eepromSize = atoi(str);
        strcat(msg, str);
        break;
      case 19:
        config.noContinuity = atoi(str);
        strcat(msg, str);
        break;
      case 20:
        config.outPut4 = atoi(str);
        strcat(msg, str);
        break;
      case 21:
        config.outPut4Delay = atol(str);
        strcat(msg, str);
        break;
      case 22:
        config.liftOffAltitude = atoi(str);
        strcat(msg, str);
        break;
      case 23:
        config.batteryType = atoi(str);
        strcat(msg, str);
        break;
      case 24:
        //our checksum
        strChk = atoi(str);
        break;
    }
    i++;

  }
  //we have a partial config
  if (i < 23)
    return false;
  if (msgChk(msg, sizeof(msg)) != strChk)
    return false;
  // add checksum
  config.cksum = CheckSumConf(config);

  writeConfigStruc();
  return true;
}

void writeConfigStruc()
{
  int i;
  for ( i = 0; i < sizeof(config); i++ ) {
    EEPROM.write(CONFIG_START + i, *((char*)&config + i));
  }
}

unsigned int msgChk( char * buffer, long length ) {

  long index;
  unsigned int checksum;

  for ( index = 0L, checksum = 0; index < length; checksum += (unsigned int) buffer[index++] );
  return (unsigned int) ( checksum % 256 );

}
