/*

   Description: Model Rocket GPS flight recorder. This will allow you to record your flight
   Author: Boris du Reau
   Date: January 2020
   Sensor used is an a BMP180 board

   You can use a stm32F103C board

   GPS is on Serial3

  TODO:
 Add it to the console
*/


#include "config.h"
#include "global.h"
#include "utils.h"
#include "kalman.h"
#include "logger_i2c_eeprom.h"
logger_I2C_eeprom logger(0x50) ;
long endAddress = 65536;
// current file number that you are recording
int currentFileNbr = 0;

// EEPROM start adress for the flights. Anything before that is the flight index
long currentMemaddress = 200;
boolean liftOff = false;
boolean landed = true;
//ground level altitude
long initialAltitude;
long liftoffAltitude = 20;
long lastAltitude;
//current altitude
long currAltitude;
bool canRecord;
bool recording = false;
bool rec = false;
unsigned long initialTime = 0;
unsigned long prevTime = 0;
unsigned long diffTime;
unsigned long currentTime = 0;
double ReadAltitude()
{
  return KalmanCalc(bmp.readAltitude());
}
/*
   Initial setup
    do the board calibration
    if you use the calibration function do not move the board until the calibration is complete

*/
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F\r\n"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C\r\n"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F\r\n"
void setup()
{
  

  Wire.begin();
  //Init bluetooth
  Serial1.begin(38400);
  //Init GPS serial port
  Serial3.begin(9600);
 
Serial3.print(PMTK_SET_NMEA_UPDATE_5HZ);

  while (!Serial1);      // wait for Leonardo enumeration, others continue immediately
  bmp.begin( config.altimeterResolution);
  // init Kalman filter
  KalmanInit();
  // let's do some dummy altitude reading
  // to initialise the Kalman filter
  for (int i = 0; i < 50; i++) {
    ReadAltitude();
  }

  long sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += ReadAltitude(); //bmp.readAltitude();
    delay(50);
  }
  initialAltitude = (sum / 10.0);

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  boolean softConfigValid = false;
  // Read altimeter softcoded configuration
  softConfigValid = readAltiConfig();

  // check if configuration is valid
  if (!softConfigValid)
  {
    //default values
    defaultConfig();
    writeConfigStruc();
  }
 

  // Get flight
  int v_ret;
  v_ret = logger.readFlightList();
  //int epromsize = logger.determineSize();
  //Serial1.println(epromsize);
  long lastFlightNbr = logger.getLastFlightNbr();

  if (lastFlightNbr < 0)
  {
    currentFileNbr = 0;
    currentMemaddress = 201;
  }
  else
  {
    currentMemaddress = logger.getFlightStop(lastFlightNbr) + 1;
    currentFileNbr = lastFlightNbr + 1;
  }
  canRecord = logger.CanRecord();
  //canRecord = true;
}


/*

   MAIN PROGRAM LOOP

*/
void loop(void)
{
  MainMenu();
}

void Mainloop(void)
{
  long startTime = millis();
  long lastWriteTime = millis();
  /*Serial1.print("Start main loop: ");
    Serial1.println(startTime);*/
  //read current altitude
  currAltitude = (ReadAltitude() - initialAltitude);
  bool lift = false;

  if ( currAltitude > liftoffAltitude)
      lift = true;
  

  if ((lift && !liftOff) || (recording && !liftOff))
  {
    liftOff = true;
    if (recording)
      rec = true;
    // save the time
    initialTime = millis();
    prevTime = 0;
    if (canRecord)
    {
      long lastFlightNbr = logger.getLastFlightNbr();

      if (lastFlightNbr < 0)
      {
        currentFileNbr = 0;
        currentMemaddress = 201;
      }
      else
      {
        currentMemaddress = logger.getFlightStop(lastFlightNbr) + 1;
        currentFileNbr = lastFlightNbr + 1;
      }
      //Save start address
      logger.setFlightStartAddress (currentFileNbr, currentMemaddress);
    }
    //Serial1.println("We have a liftoff");
  }
  if (canRecord && liftOff)
  {
    currentTime = millis() - initialTime;
    diffTime = currentTime - prevTime;
    prevTime = currentTime;
    logger.setFlightTimeData( diffTime);
    logger.setFlightAltitudeData(currAltitude);
    logger.setFlightTemperatureData((long) bmp.readTemperature());
    logger.setFlightPressureData((long) bmp.readPressure());
    boolean dataReady = false;
    
    //wait for 1/2 second
    while ((millis() - lastWriteTime) <500) {
      if(Serial3.available() > 0)
        if (gps.encode(Serial3.read())) 
          dataReady =true;
    }
    
    if(dataReady) {
      /*if (gps.gprmc_status() == 'A') {
        logger.setFlightLatitudeData((long) (gps.gprmc_latitude()*1000));
        logger.setFlightLongitudeData((long) (gps.gprmc_longitude() *1000));
      }*/
      if (gps.location.isValid()) {
        logger.setFlightLatitudeData((long) (gps.location.lat()*100000));
        logger.setFlightLongitudeData((long) (gps.location.lng() *100000));
      }
      
    }

    if(dataReady) {
      //Serial1.println("writting");
      currentMemaddress = logger.writeFastFlight(currentMemaddress);
      currentMemaddress++;
      lastWriteTime = millis();
    }
  }

  if (((canRecord && currAltitude < 10) && liftOff && !recording && !rec) || (!recording && rec))
  {
    liftOff = false;
    rec = false;
    //end loging
    //store start and end address
    logger.setFlightEndAddress (currentFileNbr, currentMemaddress - 1);
    logger.writeFlightList();
    // Serial1.println("We have landed");
  }

  

  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);

  
 /* float q1[4];
  
  q1[0] = q.w;
  q1[1] = q.x;
  q1[2] = q.y;
  q1[3] = q.z;
  
  SendTelemetry(q1, 500);*/
  
  if (!liftOff) // && !canRecord)
    delay(10);
}




//================================================================
// Main menu to interpret all the commands sent by the altimeter console
//================================================================
void MainMenu()
{
  char readVal = ' ';
  int i = 0;

  char commandbuffer[300];


  while ( readVal != ';') {
    Mainloop();
    while (Serial1.available())
    {
      readVal = Serial1.read();
      if (readVal != ';' )
      {
        if (readVal != '\n')
          commandbuffer[i++] = readVal;
      }
      else
      {
        commandbuffer[i++] = '\0';
        break;
      }
    }
  }

  interpretCommandBuffer(commandbuffer);
}

void interpretCommandBuffer(char *commandbuffer) {
  // calibrate 
  if (commandbuffer[0] == 'c')
  {
    
   // state = 0;
    
    config.cksum = CheckSumConf(config);
    writeConfigStruc();
    Serial1.print(F("$OK;\n"));
  }
  //get altimeter config
  else if (commandbuffer[0] == 'b')
  {
    Serial1.print(F("$start;\n"));

    SendAltiConfig();

    Serial1.print(F("$end;\n"));
  }
  //write altimeter config
  else if (commandbuffer[0] == 's')
  {
    if (writeAltiConfig(commandbuffer))
      Serial1.print(F("$OK;\n"));
    else
      Serial1.print(F("$KO;\n"));
  }
  //reset alti config
  else if (commandbuffer[0] == 'd')
  {
    defaultConfig();
    writeConfigStruc();
  }
  //hello
  else if (commandbuffer[0] == 'h')
  {
    //FastReading = false;
    Serial1.print(F("$OK;\n"));
  }
  //this will erase all flight
  else if (commandbuffer[0] == 'e')
  {
    Serial1.println(F("Erase\n"));
    logger.clearFlightList();
    logger.writeFlightList();
  }
  //start or stop recording
  else if (commandbuffer[0] == 'w')
  {
    if (commandbuffer[1] == '1') {
      Serial1.print(F("Start Recording\n"));
      recording = true;
    }
    else {
      Serial1.print(F("Stop Recording\n"));
      recording = false;
    }
    Serial1.print(F("$OK;\n"));
  }
  //this will read one flight
  else if (commandbuffer[0] == 'r')
  {
    char temp[3];
    /*Serial1.println(F("Read flight: "));
    Serial1.println( commandbuffer[1]);
    Serial1.println( "\n");*/
    temp[0] = commandbuffer[1];
    if (commandbuffer[2] != '\0')
    {
      temp[1] = commandbuffer[2];
      temp[2] = '\0';
    }
    else
      temp[1] = '\0';

    if (atol(temp) > -1)
    {
      //logger.PrintFlight(atoi(temp));
      Serial1.print(F("$start;\n"));
      logger.printFlightData(atoi(temp));
      Serial1.print(F("$end;\n"));
    }
    else
      Serial1.println(F("not a valid flight"));
  }
  //Number of flight
  else if (commandbuffer[0] == 'n')
  {
    Serial1.print(F("$start;\n"));
    Serial1.print(F("$nbrOfFlight,"));
    //Serial1.print(F("n;"));
    //logger.printFlightList();
    logger.readFlightList();
    Serial1.print(logger.getLastFlightNbr());
    Serial1.print(";\n");
    Serial1.print(F("$end;\n"));
  }
  //list all flights
  else if (commandbuffer[0] == 'l')
  {
    Serial1.println(F("Flight List: \n"));
    logger.printFlightList();
  }  //get all flight data
  else if (commandbuffer[0] == 'a')
  {
    Serial1.print(F("$start;\n"));
    //getFlightList()
    int i;
    ///todo
    for (i = 0; i < logger.getLastFlightNbr() + 1; i++)
    {
      logger.printFlightData(i);
    }

    Serial1.print(F("$end;\n"));
  }
  //telemetry on/off
  else if (commandbuffer[0] == 'y')
  {
    if (commandbuffer[1] == '1') {
      Serial1.print(F("Telemetry enabled\n"));
      telemetryEnable = true;
    }
    else {
      Serial1.print(F("Telemetry disabled\n"));
      telemetryEnable = false;
    }
    Serial1.print(F("$OK;\n"));
  }
  else
  {
    Serial1.println(F("Unknown command" ));
    Serial1.println(commandbuffer[0]);
  }
}

/*

   Send telemetry to the Android device

*/
void SendTelemetry(float * arr, int freq) {

  float currAltitude;
  float temperature;
  int pressure;
  float batVoltage;
  if (last_telem_time - millis() > freq)
    if (telemetryEnable) {
      currAltitude = ReadAltitude();
      pressure = bmp.readPressure();
      temperature = bmp.readTemperature();
      last_telem_time = millis();
      Serial1.print(F("$telemetry,"));
      Serial1.print("RocketMotorGimbal");
      Serial1.print(F(","));
      //tab 1
      
      //tab 2
      //Altitude
      Serial1.print(currAltitude);
      Serial1.print(F(","));
      //temperature
      Serial1.print(temperature);
      Serial1.print(F(","));
      //Pressure
      Serial1.print(pressure);
      Serial1.print(F(","));
      //Batt voltage
      pinMode(PB1, INPUT_ANALOG);
      batVoltage = analogRead(PB1);
      Serial1.print(batVoltage);
      Serial1.print(F(","));
      //tab3
      serialPrintFloatArr(arr, 4);
      Serial1.println(F(";"));
    }
}

/*

   Send the Gimbal configuration to the Android device

*/
void SendAltiConfig() {
  /*bool ret = readAltiConfig();
  //if (!ret)
  //  Serial1.print(F("invalid conf"));

  Serial1.print(F("$alticonfig"));
  Serial1.print(F(","));
  //AltimeterName
  Serial1.print(BOARD_FIRMWARE);
  Serial1.print(F(","));
  Serial1.print(config.connectionSpeed);
  Serial1.print(F(","));
  Serial1.print(config.altimeterResolution);
  Serial1.print(F(","));
  Serial1.print(config.eepromSize);
  Serial1.print(F(","));
  //alti major version
  Serial1.print(MAJOR_VERSION);
  //alti minor version
  Serial1.print(F(","));
  Serial1.print(MINOR_VERSION);
  Serial1.print(F(","));
  Serial1.print(config.unit);
  Serial1.print(F(","));
  Serial1.print(config.endRecordAltitude);
  Serial1.print(F(","));
  Serial1.print(config.beepingFrequency);
  Serial1.print(F(";\n"));*/
  bool ret= readAltiConfig();
  if(!ret)
    Serial1.print(F("invalid conf"));
  Serial1.print(F("$alticonfig"));
  Serial1.print(F(","));
  //Unit
  Serial1.print(config.unit);
  Serial1.print(F(","));
  //beepingMode
  Serial1.print(config.beepingMode);
  Serial1.print(F(","));
  //output1
  Serial1.print(config.outPut1);
  Serial1.print(F(","));
  //output2
  Serial1.print(config.outPut2);
  Serial1.print(F(","));
  //output3
  Serial1.print(config.outPut3);
  Serial1.print(F(","));
  //supersonicYesNo
  Serial1.print(config.superSonicYesNo);
  Serial1.print(F(","));
  //mainAltitude
  Serial1.print(config.mainAltitude);
  Serial1.print(F(","));
  //AltimeterName
  Serial1.print(F(BOARD_FIRMWARE));
  Serial1.print(F(","));
  //alti major version
  Serial1.print(MAJOR_VERSION);
  //alti minor version
  Serial1.print(F(","));
  Serial1.print(MINOR_VERSION);
  Serial1.print(F(","));
  //output1 delay
  Serial1.print(config.outPut1Delay);
  Serial1.print(F(","));
  //output2 delay
  Serial1.print(config.outPut2Delay);
  Serial1.print(F(","));
  //output3 delay
  Serial1.print(config.outPut3Delay);
  Serial1.print(F(","));
  //Beeping frequency
  Serial1.print(config.beepingFrequency);
  Serial1.print(F(","));
  Serial1.print(config.nbrOfMeasuresForApogee);
  Serial1.print(F(","));
  Serial1.print(config.endRecordAltitude);
  Serial1.print(F(","));
  Serial1.print(config.recordTemperature);
  Serial1.print(F(","));
  Serial1.print(config.superSonicDelay);
  Serial1.print(F(","));
  Serial1.print(config.connectionSpeed);
  Serial1.print(F(","));
  Serial1.print(config.altimeterResolution);
  Serial1.print(F(","));
  Serial1.print(config.eepromSize);
  Serial1.print(F(","));
  Serial1.print(config.noContinuity);
  
  Serial1.print(F(","));
  //output4
  Serial1.print(config.outPut4);
  Serial1.print(F(","));
   //output4 delay
  Serial1.print(config.outPut4Delay);
  //Serial1.print(F(","));

  Serial1.print(F(";\n"));
}
