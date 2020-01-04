#include "config.h"
ConfigStruct config;
//================================================================
// read and write in the microcontroler eeprom
//================================================================
void defaultConfig()
{
  config.unit = 0;
  config.beepingMode=0;
  config.outPut1=0;
  config.outPut2=1;
  config.outPut3=3;
  config.outPut1Delay=0;
  config.outPut2Delay=0;
  config.outPut3Delay=0;
  config.mainAltitude=50;
  config.superSonicYesNo=0;
  config.beepingFrequency = 440;
  //config.separationVelocity = 10; 
  config.nbrOfMeasuresForApogee = 5;
  config.endRecordAltitude=3;  // stop recording when landing define under which altitude we are not recording
  config.recordTemperature =0;  //decide if we want to record temperature
  config.superSonicDelay =0;
  config.connectionSpeed =38400;
  config.altimeterResolution = 0; //0 to 4 ie: from low resolution to high
  config.eepromSize=512;
  config.noContinuity = 0;
  config.outPut4=3;
  config.outPut4Delay=0;
 
  config.cksum=CheckSumConf(config); 
}

unsigned int CheckSumConf( ConfigStruct cnf)
 {
     int i;
     unsigned int chk=0;
    
     //for (i=0; i < (sizeof(cnf)-2); i++) 
     for (i=0; i < (sizeof(cnf)-sizeof(int)); i++) 
     chk += *((char*)&cnf + i);
    
     return chk;
 }

boolean readAltiConfig() {
  //set the config to default values so that if any have not been configured we can use the default ones
  defaultConfig();
  int i;
  for( i=0; i< sizeof(config); i++ ) {
    *((char*)&config + i) = EEPROM.read(CONFIG_START + i);
  }

  if ( config.cksum != CheckSumConf(config) ) {
  //if ( config.cksum != 0xBA ) {
    return false;
  }

  return true;

}

/*
* write the config received by the console
*
*/
bool writeAltiConfig( char *p ) {

  char *str;
  int i=0;
  while ((str = strtok_r(p, ",", &p)) != NULL) // delimiter is the comma
  {
    Serial1.println(str);
    switch (i)
    {
    case 1:
      config.unit =atoi(str);
      break;
    case 2:
      config.beepingMode=atoi(str);
      break;
    case 3:
      config.outPut1=atoi(str);
      break;   
    case 4:
      config.outPut2=atoi(str);
      break;
    case 5:
      config.outPut3=atoi(str);
      break;
    case 6:
      config.mainAltitude=atoi(str);
      break;
    case 7:
      config.superSonicYesNo=atoi(str);
      break;
    case 8:
      config.outPut1Delay=atol(str);
      break;
    case 9:
      config.outPut2Delay=atol(str);
      break;
    case 10:
      config.outPut3Delay=atol(str);
      break;
    case 11:
      config.beepingFrequency =atoi(str);
      break;
    case 12:
      config.nbrOfMeasuresForApogee=atoi(str);
      break;
    case 13:
      config.endRecordAltitude=atol(str);
      break;
    case 14:
      config.recordTemperature=atoi(str);
      break;
    case 15:
      config.superSonicDelay=atoi(str);
      break;
    case 16:
      config.connectionSpeed=atol(str);
      break;
    case 17:
      config.altimeterResolution=atoi(str);
      break;
    case 18:
      config.eepromSize =atoi(str);
      break;
    case 19:
      config.noContinuity=atoi(str);
      break;
    
    case 20:
      config.outPut4=atoi(str);
      break;  
    case 21:
      config.outPut4Delay=atol(str);
      //SerialCom.print(F("WTF "));
      break;
  
    }
    i++;

  }

  config.cksum = CheckSumConf(config);

  /*for( i=0; i<sizeof(config); i++ ) {
    EEPROM.write(CONFIG_START+i, *((char*)&config + i));
  }*/
  writeConfigStruc();
  return true;
}

void writeConfigStruc()
{
    int i;
    for( i=0; i<sizeof(config); i++ ) {
      EEPROM.write(CONFIG_START+i, *((char*)&config + i));
    }
}
