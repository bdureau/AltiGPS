#include "logger_i2c_eeprom.h"
#include "IC2extEEPROM.h"
extEEPROM eep(kbits_512, 1, 64);
logger_I2C_eeprom::logger_I2C_eeprom(uint8_t deviceAddress)
{

}


void logger_I2C_eeprom::begin()
{
  Wire.begin();
  //initialize Flight structure
}

void logger_I2C_eeprom::clearFlightList()
{
  int i;
  for (i = 0; i < 25; i++)
  {
    _FlightConfig[i].flight_start = 0;
    _FlightConfig[i].flight_stop = 0;
  }
}



int logger_I2C_eeprom::readFlightList() {
  eep.read(0, ((byte*)&_FlightConfig), sizeof(_FlightConfig));
  return FLIGHT_LIST_START + sizeof(_FlightConfig) ;
}

unsigned long logger_I2C_eeprom::readFlight(unsigned long eeaddress) {
  eep.read(eeaddress, ((byte*)&_FlightData), sizeof(_FlightData));
  return eeaddress + sizeof(_FlightData);
}

int logger_I2C_eeprom::writeFlightList()
{
  eep.write(FLIGHT_LIST_START, ((byte*)&_FlightConfig), sizeof(_FlightConfig));
  return FLIGHT_LIST_START + sizeof(_FlightConfig);
}

/*
   writeFastFlight(int eeaddress)

*/
unsigned long logger_I2C_eeprom::writeFastFlight(unsigned long eeaddress) {
  eep.write(eeaddress, ((byte*)&_FlightData), sizeof(_FlightData));
  return eeaddress + sizeof(_FlightData);
}

/*

   getLastFlightNbr()
   Parse the flight index end check if the flight_start address is > 0
   return -1 if no flight have been recorded else return the flight number

*/
int logger_I2C_eeprom::getLastFlightNbr()
{
  int i;
  for (i = 0; i < 25; i++)
  {
    if (_FlightConfig[i].flight_start == 0)
    {
      break;
    }
  }
  i--;
  return i;
}
/*

 eraseLastFlight()
 
 */
bool logger_I2C_eeprom::eraseLastFlight(){
int i;
  for (i = 0; i < 25; i++)
  {
    if (_FlightConfig[i].flight_start == 0)
    {
      if(i>0) {
        _FlightConfig[i-1].flight_start = 0;
        _FlightConfig[i-1].flight_stop = 0;
        writeFlightList();
        return true;
      }
    }
  }
  return false;
}
/*

   getLastFlightEndAddress()
   Parse the flight index end check if the flight_start address is > 0
   return -1 if no flight have been recorded else return the flight number

*/
long logger_I2C_eeprom::getLastFlightEndAddress()
{
  int i;
  for (i = 0; i < 25; i++)
  {
    if (_FlightConfig[i].flight_start == 0)
    {
      break;
    }
  }
  i--;
  return _FlightConfig[i].flight_stop;
}
int logger_I2C_eeprom::printFlightList()
{
  //retrieve from the eeprom
  int v_ret =  readFlightList();

  //Read the stucture
  int i;
  for (i = 0; i < 25; i++)
  {
    if (_FlightConfig[i].flight_start == 0)
      break;
    Serial1.print("Flight Nbr: ");
    Serial1.println(i);
    Serial1.print("Start: ");
    Serial1.println(_FlightConfig[i].flight_start);
    Serial1.print("End: ");
    Serial1.println(_FlightConfig[i].flight_stop);
  }
  return i;
}

void logger_I2C_eeprom::setFlightStartAddress(int flightNbr, long startAddress)
{
  _FlightConfig[flightNbr].flight_start = startAddress;
}

void logger_I2C_eeprom::setFlightEndAddress(int flightNbr, long endAddress)
{
  _FlightConfig[flightNbr].flight_stop = endAddress;
}

void logger_I2C_eeprom::setFlightTimeData( long difftime)
{
  _FlightData.diffTime = difftime;
}
void logger_I2C_eeprom::setFlightAltitudeData( long altitude)
{
  _FlightData.altitude = altitude;
}
void logger_I2C_eeprom::setFlightTemperatureData( long temperature) {
  _FlightData.temperature = temperature;
}
void logger_I2C_eeprom::setFlightPressureData( long pressure) {
  _FlightData.pressure = pressure;
}
void logger_I2C_eeprom::setFlightVoltageData( long voltage) {
  _FlightData.voltage = voltage;
}
void logger_I2C_eeprom::setFlightLatitudeData( long latitude) {
  _FlightData.latitude = latitude;
}
void logger_I2C_eeprom::setFlightLongitudeData( long longitude) {
  _FlightData.longitude = longitude;
}
void logger_I2C_eeprom::setFlightGPSAltitudeData( long GPSaltitude) {
  _FlightData.GPSaltitude = GPSaltitude;
}
void logger_I2C_eeprom::setFlightNbrOfSatData( long nbrOfSat) {
  _FlightData.nbrOfSat = nbrOfSat;
}
void logger_I2C_eeprom::setFlightGPSSpeedData( long GPSSpeed) {
  _FlightData.GPSSpeed = GPSSpeed;
}
void logger_I2C_eeprom::setFlightSeaAltitudeData( long SeaAltitude) {
  _FlightData.SeaAltitude = SeaAltitude;
}

long logger_I2C_eeprom::getFlightStart(int flightNbr)
{
  return  _FlightConfig[flightNbr].flight_start;
}
long logger_I2C_eeprom::getFlightStop(int flightNbr)
{
  return  _FlightConfig[flightNbr].flight_stop;
}

long logger_I2C_eeprom::getFlightTimeData()
{
  return _FlightData.diffTime;
}
long logger_I2C_eeprom::getFlightAltitudeData()
{
  return _FlightData.altitude;
}
long logger_I2C_eeprom::getSizeOfFlightData()
{
  return sizeof(_FlightData);
}

void logger_I2C_eeprom::printFlightData(int flightNbr)
{
  int startaddress;
  int endaddress;
  long flight_type;
  startaddress = getFlightStart(flightNbr);
  endaddress = getFlightStop(flightNbr);

  if (startaddress > 200)
  {
    int i = startaddress;
    unsigned long currentTime = 0;

    while (i < (endaddress + 1))
    {
      i = readFlight(i) + 1;
      char flightData[150] = "";
      char temp[20] = "";
      currentTime = currentTime + getFlightTimeData();
      strcat(flightData, "data,");
      sprintf(temp, "%i,", flightNbr );
      strcat(flightData, temp);
      sprintf(temp, "%i,", currentTime );
      strcat(flightData, temp);
      sprintf(temp, "%i,", getFlightAltitudeData() );
      strcat(flightData, temp);
      sprintf(temp, "%i,", _FlightData.temperature );
      strcat(flightData, temp);
      sprintf(temp, "%i,", _FlightData.pressure );
      strcat(flightData, temp);
      sprintf(temp, "%i,", _FlightData.voltage );
      strcat(flightData, temp);
      sprintf(temp, "%i,", _FlightData.latitude );
      strcat(flightData, temp);
      sprintf(temp, "%i,", _FlightData.longitude );
      strcat(flightData, temp);
      sprintf(temp, "%i,", _FlightData.GPSaltitude );
      strcat(flightData, temp);
      sprintf(temp, "%i,", _FlightData.nbrOfSat );
      strcat(flightData, temp);
      sprintf(temp, "%i,", _FlightData.GPSSpeed );
      strcat(flightData, temp);
      sprintf(temp, "%i,", _FlightData.SeaAltitude );
      strcat(flightData, temp);
      unsigned int chk = msgChk(flightData, sizeof(flightData));
      sprintf(temp, "%i", chk);
      strcat(flightData, temp);
      strcat(flightData, ";\n");
      SerialCom.print("$");
      SerialCom.print(flightData);

      //This will slow down the data
      // this is for telemetry modules without enought buffer
      if (config.telemetryType == 0) 
        delay(0);
      else if (config.telemetryType == 1)  
        delay(20); 
      else if (config.telemetryType == 2)
        delay(50);
      else if (config.telemetryType == 3)
        delay(100);
    }
  }
}
boolean logger_I2C_eeprom::CanRecord()
{
  long lastFlight;
  lastFlight = getLastFlightNbr();
  if (lastFlight == -1)
    return true;
  // Serial.println(lastFlight);
  if (lastFlight == 24)
  {
    //#ifdef SERIAL_DEBUG
    //Serial.println("25 flights");
    //#endif
    return false;
  }
  if (getFlightStop(lastFlight) > 65500 )
  {
    //#ifdef SERIAL_DEBUG
    //Serial.println("memory is full");
    //#endif
    return false;
  }
  return true;
}
