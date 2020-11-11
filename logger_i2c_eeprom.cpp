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

int logger_I2C_eeprom::readFlight(int eeaddress) {
  eep.read(eeaddress, ((byte*)&_FlightData), sizeof(_FlightData));
  return eeaddress + sizeof(_FlightData);
}

int logger_I2C_eeprom::writeFlightList()
{
  eep.write(FLIGHT_LIST_START, ((byte*)&_FlightConfig), sizeof(_FlightConfig));
  return FLIGHT_LIST_START + sizeof(_FlightConfig);
}

/*
 * writeFastFlight(int eeaddress)
 * 
 */
int logger_I2C_eeprom::writeFastFlight(int eeaddress){
  eep.write(eeaddress, ((byte*)&_FlightData), sizeof(_FlightData));
  return eeaddress + sizeof(_FlightData);
}

/*
 * 
 * getLastFlightNbr()
 * Parse the flight index end check if the flight_start address is > 0
 * return -1 if no flight have been recorded else return the flight number
 * 
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
 * 
 * getLastFlightNbr()
 * Parse the flight index end check if the flight_start address is > 0
 * return -1 if no flight have been recorded else return the flight number
 * 
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
void logger_I2C_eeprom::setFlightTemperatureData( long temperature){
  _FlightData.temperature = temperature;
}
void logger_I2C_eeprom::setFlightPressureData( long pressure){
  _FlightData.pressure = pressure;
}

void logger_I2C_eeprom::setFlightLatitudeData( long latitude){
  _FlightData.latitude = latitude;
}
void logger_I2C_eeprom::setFlightLongitudeData( long longitude){
  _FlightData.longitude = longitude;
}
//void logger_I2C_eeprom::setFlightRocketPos(char *w, char *x, char *y, char *z )


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
void logger_I2C_eeprom::PrintFlight(int flightNbr)
{
  long startaddress;
  long endaddress;
  long flight_type;
  startaddress = getFlightStart(flightNbr);
  endaddress = getFlightStop(flightNbr);
  //flight_type = getFlightType(flightNbr);

  if (startaddress > 200)
  {
    int i = startaddress;
    unsigned long currentTime = 0;
    Serial1.println("StartFlight;" );
    while (i < (endaddress + 1))
    {
      i = readFlight(i) + 1;

      currentTime = currentTime + getFlightTimeData();
      Serial1.print("$" + String("data,") + String(flightNbr) + "," + String(currentTime) + "," + String(getFlightAltitudeData()) + ",");
      Serial1.print(_FlightData.temperature);
      Serial1.print(",");
      Serial1.print(_FlightData.pressure);
      Serial1.print(",");
      Serial1.print(_FlightData.latitude);
      Serial1.print(",");
      Serial1.print(_FlightData.longitude);
    }
    Serial1.println("EndFlight;" );
  }
  else
    Serial1.println(F("No such flight\n"));
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

      currentTime = currentTime + getFlightTimeData();
      //long pos[4];
      //getFlightRocketPos(pos);
      Serial1.print("$" + String("data,") + String(flightNbr) + "," + String(currentTime) + "," + String(getFlightAltitudeData()) + ",");
      Serial1.print(_FlightData.temperature);
      Serial1.print(",");
      Serial1.print(_FlightData.pressure);
      Serial1.print(",");
      Serial1.print(_FlightData.latitude);
      Serial1.print(",");
      Serial1.print(_FlightData.longitude);
      Serial1.println(";");
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
