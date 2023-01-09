/*
  Rocket GPS Logger ver 1.3
  Copyright Boris du Reau 2012-2022
  Description: Model Rocket GPS flight recorder. This will allow you to record your flight
  Author: Boris du Reau
  Date: January 2020
  Sensor used is an a BMP180 board

  You can use a stm32F103C board. I am using my STM32 altimeter board that has 2 serial ports.

  GPS is on SerialGPS

  Major changes on version 1.0
  Initial version
  Major changes on version 1.1
  adding checksum
  Major changes on version 1.2
  Code review so that is compatible with the other altimeters
  TODO:
  Major changes on version 1.3
  Code review for compatibility
  Major changes on version 1.4
  Adding voltage recording
  Major changes on version 1.5
  Compatibility with the latest console app
*/

//Alti GPS config lib
#include "config.h"
#include "global.h"
#include "utils.h"
#include "beepfunc.h"
#include "kalman.h"
#include "logger_i2c_eeprom.h"

//////////////////////////////////////////////////////////////////////
// Global variables
//////////////////////////////////////////////////////////////////////
//EEProm address
logger_I2C_eeprom logger(0x50) ;
// End address of the 512 eeprom
//long endAddress = 65536;
// current file number that you are recording
int currentFileNbr = 0;
// EEPROM start adress for the flights.
// Anything before that are the flights indexes
long currentMemaddress = 200;
//stop recording a maximum of 20 seconds after main has fired
//long recordingTimeOut = 20000;
bool canRecord = true;
//boolean exitRecording = false;
bool recording = false;
//bool rec = false;
//bool GpsRawEnable = false;

//ground level altitude
long initialAltitude;
// any altitude above that and we consider that it is a liftoff
//long liftoffAltitude = 20;
long lastAltitude;
//current altitude
long currAltitude;
//Apogee altitude
long apogeeAltitude;
long mainAltitude;
long drogueFiredAltitude;

boolean liftOff = false;
unsigned long initialTime = 0;
boolean FastReading = false;

//nbr of measures to do so that we are sure that apogee has been reached
//unsigned long measures = 5;
//unsigned long mainDeployAltitude;
long startupTime = 0;
long timeToSat = 0;


//by default apogee pin
const int pinChannel1Continuity = PA2;
// by default continuity for the main
const int pinChannel2Continuity = PA4;
// third output
const int pinChannel3Continuity = PA6;
// fourth output
const int pinChannel4Continuity = PB0;

//float FEET_IN_METER = 1;

boolean Output1Fired = false;
boolean Output2Fired = false;
boolean Output3Fired = false;
boolean Output4Fired = false;

boolean allApogeeFiredComplete = false;
boolean allMainFiredComplete = false;
boolean allTimerFiredComplete = false;
boolean allLiftOffFiredComplete = false;
boolean allLandingFiredComplete = false;
boolean allAltitudeFiredComplete = false;


//telemetry
boolean telemetryEnable = false;
long lastTelemetry = 0;
long lastBattWarning = 0;

void MainMenu();


/*
   Just return the current altitude filtered of any noise
*/
double ReadAltitude()
{
  return KalmanCalc(bmp.readAltitude());
}
/*
   Initial setup
   Initialise the GPS

*/
//#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F\r\n"
//#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C\r\n"
//#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F\r\n"

/*

  ResetGlobalVar()

*/
void ResetGlobalVar() {

  //exitRecording = false;

  allApogeeFiredComplete = false;
  allMainFiredComplete = false;
  allTimerFiredComplete = false;
  allLiftOffFiredComplete = false;
  allLandingFiredComplete = false;
  allAltitudeFiredComplete = false;

  liftOff = false;
  apogeeAltitude = 0;
  mainAltitude = 0;

  Output1Fired = false;
  Output2Fired = false;
  Output3Fired = false;
  Output4Fired = false;


  lastAltitude = 0;//initialAltitude;

}

void initAlti() {

  ResetGlobalVar();

  // set main altitude (if in feet convert to metrics)
  /* if (config.unit == 0)
      FEET_IN_METER = 1;
    else
      FEET_IN_METER = 3.28084 ;*/
  //mainDeployAltitude = int(config.mainAltitude / FEET_IN_METER);
  // beepFrequency
  beepingFrequency = config.beepingFrequency;


  //number of measures to do to detect Apogee
  // measures = config.nbrOfMeasuresForApogee;

  //check which pyro are enabled
  pos = -1;
  // The altimeter board has 4 pyro so we might as well use them
  if (config.outPut1 != 3) {
    pos++;
    continuityPins[pos] = pinChannel1Continuity;
  }
  if (config.outPut2 != 3) {
    pos++;
    continuityPins[pos] = pinChannel2Continuity;
  }
  if (config.outPut3 != 3) {
    pos++;
    continuityPins[pos] = pinChannel3Continuity;
  }
  if (config.outPut4 != 3)  {
    pos++;
    continuityPins[pos] = pinChannel4Continuity;
  }
}

/*
    Setup and start program
*/
void setup()
{
  //startupTime = millis();
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

  initAlti();
  // init Kalman filter
  KalmanInit();
  // initialise the connection
  Wire.begin();

  //Init bluetooth (which is on Serial 1)
  SerialCom.begin(38400);
  //software pull up so that all bluetooth modules work!!!
  pinMode(PB11, INPUT_PULLUP);
  

  //Init GPS serial port (Which is on Serial 3)
  SerialGPS.begin(9600);

  //SerialGPS.print(PMTK_SET_NMEA_UPDATE_5HZ);
  SerialGPS.print("$PMTK220,200*2C\r\n");

  while (!SerialCom);      // wait for Leonardo enumeration, others continue immediately
  bmp.begin( config.altimeterResolution);

  //Initialise the output pin
  pinMode(pyroOut1, OUTPUT);
  pinMode(pyroOut2, OUTPUT);
  pinMode(pyroOut3, OUTPUT);
  pinMode(pyroOut4, OUTPUT);

  pinMode(pinSpeaker, OUTPUT);

  // pinMode(pinAltitude1, INPUT);
  // pinMode(pinAltitude2, INPUT);

  pinMode(pinChannel1Continuity , INPUT);
  pinMode(pinChannel2Continuity , INPUT);
  pinMode(pinChannel3Continuity , INPUT);
  pinMode(pinChannel4Continuity , INPUT);

  //Make sure that the output are turned off
  digitalWrite(pyroOut1, LOW);
  digitalWrite(pyroOut2, LOW);
  digitalWrite(pyroOut3, LOW);
  digitalWrite(pyroOut4, LOW);
  digitalWrite(pinSpeaker, LOW);

  //enable or disable continuity check
  if (config.noContinuity == 1)
    noContinuity = true;
  else
    noContinuity = false;

  //initialisation give the version of the altimeter
  //One long beep per major number and One short beep per minor revision
  //For example version 1.2 would be one long beep and 2 short beep
  beepAltiVersion(MAJOR_VERSION, MINOR_VERSION);

  // let's do some dummy altitude reading
  // to initialise the Kalman filter
  for (int i = 0; i < 50; i++) {
    ReadAltitude();
  }

  //let's read the launch site altitude
  long sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += ReadAltitude();
    delay(50);
  }
  initialAltitude = (sum / 10.0);
  lastAltitude = 0;
  //liftoffAltitude = config.liftOffAltitude;//20;

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  // Get flight
  int v_ret;
  v_ret = logger.readFlightList();
  //int epromsize = logger.determineSize();
  //SerialCom.println(epromsize);
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
  if (!canRecord)
    SerialCom.println("Cannot record");
}

/*
   setEventState(int pyroOut, boolean state)
    Set the state of the output
*/
void setEventState(int pyroOut, boolean state)
{
  if (pyroOut == pyroOut1)
  {
    Output1Fired = state;
#ifdef SERIAL_DEBUG
    SerialCom.println(F("Output1Fired"));
#endif
  }

  if (pyroOut == pyroOut2)
  {
    Output2Fired = state;
#ifdef SERIAL_DEBUG
    SerialCom.println(F("Output2Fired"));
#endif
  }

  if (pyroOut == pyroOut3)
  {
    Output3Fired = state;
#ifdef SERIAL_DEBUG
    SerialCom.println(F("Output3Fired"));
#endif
  }
  if (pyroOut == pyroOut4)
  {
    Output4Fired = state;
#ifdef SERIAL_DEBUG
    SerialCom.println(F("Output4Fired"));
#endif
  }
}

/*

    Send telemetry to the Android device

*/
void SendTelemetry(long sampleTime, int freq) {
  char altiTelem[150] = "";
  char temp[10] = "";

  if (telemetryEnable && (millis() - lastTelemetry) > freq) {
    lastTelemetry = millis();
    int val = 0;
    //check liftoff
    int li = 0;
    if (liftOff)
      li = 1;

    //check apogee
    int ap = 0;
    if (allApogeeFiredComplete)
      ap = 1;

    //check main
    int ma = 0;
    if (allMainFiredComplete)
      ma = 1;
    int landed = 0;
    if ( allMainFiredComplete && currAltitude < 10)
      landed = 1;
    strcat(altiTelem, "telemetry," );
    sprintf(temp, "%i,", currAltitude);
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", li);
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", ap);
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", apogeeAltitude);
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", ma);
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", mainAltitude);
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", landed);
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", sampleTime);
    strcat(altiTelem, temp);
    if (config.outPut1 != 3) {
      //check continuity
      val = digitalRead(pinChannel1Continuity);
      if (val == 0)
        strcat(altiTelem, "0,");
      else
        strcat(altiTelem, "1,");
    }
    else {
      strcat(altiTelem, "-1,");
    }
    if (config.outPut2 != 3) {
      //check continuity
      val = digitalRead(pinChannel2Continuity);
      //delay(20);
      if (val == 0)
        strcat(altiTelem, "0,");
      else
        strcat(altiTelem, "1,");
    }
    else {
      strcat(altiTelem, "-1,");
    }
    if (config.outPut3 != 3) {
      //check continuity
      val = digitalRead(pinChannel3Continuity);
      if (val == 0)
        strcat(altiTelem, "0,");
      else
        strcat(altiTelem, "1,");
    }
    else {
      strcat(altiTelem, "-1,");
    }
    if (config.outPut4 != 3) {
      //check continuity
      val = digitalRead(pinChannel4Continuity);
      if (val == 0)
        strcat(altiTelem, "0,");
      else
        strcat(altiTelem, "1,");
    }
    else {
      strcat(altiTelem, "-1,");
    }
    pinMode(PB1, INPUT_ANALOG);
    int batVoltage = analogRead(PB1);
    float bat = VOLT_DIVIDER * ((float)(batVoltage * 3300) / (float)4096000);
    //sprintf(temp, "%f,", bat);
    dtostrf(bat, 4, 2, temp);
    strcat(altiTelem, temp);
    strcat(altiTelem, ",");

    // temperature
    float temperature;
    temperature = bmp.readTemperature();
    sprintf(temp, "%i,", (int)temperature );
    strcat(altiTelem, temp);

    sprintf(temp, "%i,", (int)(100 * ((float)logger.getLastFlightEndAddress() / /*endAddress*/ 65536)) );
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", logger.getLastFlightNbr() + 1 );
    strcat(altiTelem, temp);

    //drogueFiredAltitude
    sprintf(temp, "%i,", drogueFiredAltitude);
    strcat(altiTelem, temp);

    //gps latitude
    sprintf(temp, "%i,", (int)(gps.location.lat() * 100000) );
    strcat(altiTelem, temp);

    //gps longitude
    sprintf(temp, "%i,", (int)(gps.location.lng() * 100000) );
    strcat(altiTelem, temp);

    //number of satellites
    sprintf(temp, "%i,", (int)gps.satellites.value());
    strcat(altiTelem, temp);

    //hdop
    sprintf(temp, "%i,", (int)gps.hdop.value());
    strcat(altiTelem, temp);

    //location age
    sprintf(temp, "%i,", (int)gps.location.age());
    strcat(altiTelem, temp);

    //altitude
    sprintf(temp, "%i,", (int)gps.altitude.meters());
    strcat(altiTelem, temp);

    //speed
    sprintf(temp, "%i,", (int)gps.speed.kmph());
    strcat(altiTelem, temp);

    //time for satelitte acquisition
    sprintf(temp, "%i,", timeToSat);
    strcat(altiTelem, temp);

    unsigned int chk;
    chk = msgChk(altiTelem, sizeof(altiTelem));
    sprintf(temp, "%i", chk);
    strcat(altiTelem, temp);
    strcat(altiTelem, ";\n");
    SerialCom.print("$");
    SerialCom.print(altiTelem);
  }
}

/*

   MAIN PROGRAM LOOP

*/
void loop(void)
{
  MainMenu();
}
/*
   Calculate the current velocity
*/
int currentVelocity(long prevTime, long curTime, int prevAltitude, int curAltitude)
{
  int curSpeed = int (float(curAltitude - prevAltitude) / (float( curTime - prevTime) / 1000));
  return curSpeed;
}

//================================================================
// Function:  recordAltitude()
// called for manual recording
//================================================================
void record() {
  unsigned long currentTime;
  unsigned long diffTime;
  unsigned long prevTime = millis();
  long lastWriteTime = millis();
  char readVal = ' ';
  int i = 0;

  char commandbuffer[100];
  
  // save the time
  initialTime = millis();
  
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
  while (recording) {
    boolean dataReady = false;
    currAltitude = (ReadAltitude() - initialAltitude);

    while ((millis() - lastWriteTime) < 100) {
      if (SerialGPS.available() > 0)
        if (gps.encode(SerialGPS.read()))
          dataReady = true;
    }
    currentTime = millis() - initialTime;
    diffTime = currentTime - prevTime;
    prevTime = currentTime;

    // check if we have anything on the serial port
    // this will exit the loop if we decided to stop recording
    if (SerialCom.available())
    {
      readVal = SerialCom.read();
      if (readVal != ';' )
      {
        if (readVal != '\n')
          commandbuffer[i++] = readVal;
      }
      else
      {
        commandbuffer[i++] = '\0';
        interpretCommandBuffer(commandbuffer);
      }
    }
    if (canRecord)
    {
      logger.setFlightTimeData( diffTime);
      logger.setFlightAltitudeData(currAltitude);
      logger.setFlightTemperatureData((long) bmp.readTemperature());
      logger.setFlightPressureData((long) bmp.readPressure());
      float bat = VOLT_DIVIDER * ((float)(analogRead(PB1) * 3300) / (float)4096000);
      logger.setFlightVoltageData((long) 100 * bat); 
      if (gps.location.isValid()) {
        logger.setFlightLatitudeData((long) (gps.location.lat() * 100000));
        logger.setFlightLongitudeData((long) (gps.location.lng() * 100000));
      } else {
        logger.setFlightLatitudeData((long) (0));
        logger.setFlightLatitudeData((long) (0));
      }
      
      if ( (currentMemaddress + logger.getSizeOfFlightData())  > /*endAddress*/ 65536) {
        //flight is full let's save it
        //save end address
        logger.setFlightEndAddress (currentFileNbr, currentMemaddress - 1);
        canRecord = false;
      } else {
        currentMemaddress = logger.writeFastFlight(currentMemaddress);
        currentMemaddress++;
        lastWriteTime = millis();
      }
      // wait for 5 second
      delay(5000);
    }
  }

  // we are exiting recording
  if (canRecord )
  {
    //end loging
    //store start and end address
    //save end address
    logger.setFlightEndAddress (currentFileNbr, currentMemaddress - 1);
    logger.writeFlightList();
  }
}

//================================================================
// Function:  recordAltitude()
// called for normal recording
//================================================================
void recordAltitude()
{
  float FEET_IN_METER = 1;

  if (config.unit == 0)
    FEET_IN_METER = 1;
  else
    FEET_IN_METER = 3.28084 ;
  ResetGlobalVar();
  unsigned long measures = config.nbrOfMeasuresForApogee;

  long lastWriteTime = millis();

  boolean OutputFiredComplete[4] = {false, false, false, false};
  int OutputDelay[4] = {0, 0, 0, 0};
  OutputDelay[0] = config.outPut1Delay;
  OutputDelay[1] = config.outPut2Delay;
  OutputDelay[2] = config.outPut3Delay;
  OutputDelay[3] = config.outPut4Delay;

  // 0 = main 1 = drogue 2 = timer 4 = landing 5 = liftoff 3 = disable 6 = altitude
  int OutputType[4] = {3, 3, 3, 3};
  OutputType[0] = config.outPut1;
  OutputType[1] = config.outPut2;
  OutputType[2] = config.outPut3;
  OutputType[3] = config.outPut4;
  int OutputPins[4] = { -1, -1, -1, -1};
  if (config.outPut1 != 3)
    OutputPins[0] = pyroOut1;
  if (config.outPut2 != 3)
    OutputPins[1] = pyroOut2;
  if (config.outPut3 != 3)
    OutputPins[2] = pyroOut3;
  //#ifdef ALTIMULTISTM32
  if (config.outPut4 != 3)
    OutputPins[3] = pyroOut4;
  //#endif


  boolean exitRecording = false;
  boolean apogeeReadyToFire = false;
  boolean mainReadyToFire = false;
  boolean landingReadyToFire = false;
  boolean liftOffReadyToFire = false;
  unsigned long apogeeStartTime = 0;
  unsigned long mainStartTime = 0;
  unsigned long landingStartTime = 0;
  unsigned long liftOffStartTime = 0;
  boolean ignoreAltiMeasure = false;
  unsigned long altitudeStartTime[] = {0, 0, 0, 0};

  boolean liftOffHasFired = false;
  //hold the state of all our outputs
  boolean outputHasFired[4] = {false, false, false, false};

  if (config.outPut1 == 3) Output1Fired = true;
  if (config.outPut2 == 3) Output2Fired = true;
  if (config.outPut3 == 3) Output3Fired = true;
  //#ifdef NBR_PYRO_OUT4
  if (config.outPut4 == 3) Output4Fired = true;
  //#endif

#ifdef SERIAL_DEBUG
  SerialCom.println(F("Config delay:"));
  SerialCom.println(config.outPut1Delay);
  SerialCom.println(config.outPut2Delay);
  SerialCom.println(config.outPut3Delay);
  //#ifdef NBR_PYRO_OUT4
  SerialCom.println(config.outPut4Delay);
  //#endif

#endif

  while (!exitRecording)
  {
    //read current altitude
    currAltitude = (ReadAltitude() - initialAltitude);
    //if (liftOff)
    //  SendTelemetry(millis() - initialTime, 200);
    if (( currAltitude > config.liftOffAltitude) && !liftOff && !allMainFiredComplete)
    {
      liftOff = true;
      SendTelemetry(0, 200);
      // save the time
      initialTime = millis();
      if (config.superSonicYesNo == 1)
        ignoreAltiMeasure = true;

#ifdef SERIAL_DEBUG
      SerialCom.println(F("we have lift off\n"));
#endif
      if (canRecord)
      {
        //Save start address
        logger.setFlightStartAddress (currentFileNbr, currentMemaddress);
#ifdef SERIAL_DEBUG
        SerialCom.println(F("Save start address\n"));
#endif
      }

    }
    unsigned long prevTime = 0;
    long prevAltitude = 0;
    // loop until we have reach an altitude of 3 meter
    while (liftOff)
    {
      unsigned long currentTime;
      unsigned long diffTime;

      currAltitude = (ReadAltitude() - initialAltitude);

      currentTime = millis() - initialTime;
      if (allMainFiredComplete && !allLandingFiredComplete && !landingReadyToFire && currAltitude < 10) {

        if (abs(currentVelocity(prevTime, currentTime, prevAltitude, currAltitude)) < 1  ) {
          //we have landed
          landingReadyToFire = true;
          landingStartTime = millis();
        }
      }
      prevAltitude = currAltitude;
      SendTelemetry(currentTime, 200);
      diffTime = currentTime - prevTime;
      prevTime = currentTime;

      if (!liftOffHasFired && !liftOffReadyToFire) {
        liftOffReadyToFire = true;
        liftOffStartTime = millis();
      }

      if (!allLiftOffFiredComplete) {
        //fire all liftoff that are ready
        for (int li = 0; li < 4; li++ ) {
          if (!outputHasFired[li] && ((millis() - liftOffStartTime) >= OutputDelay[li] ) && OutputType[li] == 5) {
            digitalWrite(OutputPins[li], HIGH);
            outputHasFired[li] = true;
          }
        }
        for (int li = 0; li < 4; li++ ) {
          if ((millis() - liftOffStartTime ) >= (1000 + OutputDelay[li])  && !OutputFiredComplete[li] && OutputType[li] == 5)
          {
            digitalWrite(OutputPins[li], LOW);
            setEventState(OutputPins[li], true);
            OutputFiredComplete[li] = true;
          }
        }

        allLiftOffFiredComplete = true;

        for (int li = 0; li < 4; li++ ) {
          if (!OutputFiredComplete[li] && OutputType[li] == 5)
          {
            allLiftOffFiredComplete = false;
          }
        }
        SendTelemetry(millis() - initialTime, 200);
      }

      //altitude events
      if (!allAltitudeFiredComplete) {
        //fire all altitude that are ready
        for (int al = 0; al < 4; al++ ) {
          if (!outputHasFired[al] && ((currAltitude >= OutputDelay[al]) ) && OutputType[al] == 6) {
            digitalWrite(OutputPins[al], HIGH);
            outputHasFired[al] = true;
            altitudeStartTime[al] = millis();
          }
        }
        for (int al = 0; al < 4; al++ ) {
          if (( millis()  >= (1000 + altitudeStartTime[al]))  && !OutputFiredComplete[al] && OutputType[al] == 6 && outputHasFired[al])
          {
            digitalWrite(OutputPins[al], LOW);
            setEventState(OutputPins[al], true);
            OutputFiredComplete[al] = true;
          }
        }

        allAltitudeFiredComplete = true;

        for (int al = 0; al < 4; al++ ) {
          if (!OutputFiredComplete[al] && OutputType[al] == 6)
          {
            allAltitudeFiredComplete = false;
          }
        }
        SendTelemetry(millis() - initialTime, 200);
      }
      // timer events
      if (!allTimerFiredComplete) {
        //fire all timers that are ready
        for (int ti = 0; ti < 4; ti++ ) {
          if (!outputHasFired[ti] && ((currentTime >= OutputDelay[ti]) ) && OutputType[ti] == 2) {
            digitalWrite(OutputPins[ti], HIGH);
            outputHasFired[ti] = true;
          }
        }
        for (int ti = 0; ti < 4; ti++ ) {
          if ((currentTime  >= (1000 + OutputDelay[ti]))  && !OutputFiredComplete[ti] && OutputType[ti] == 2)
          {
            digitalWrite(OutputPins[ti], LOW);
            setEventState(OutputPins[ti], true);
            OutputFiredComplete[ti] = true;
          }
        }

        allTimerFiredComplete = true;

        for (int ti = 0; ti < 4; ti++ ) {
          if (!OutputFiredComplete[ti] && OutputType[ti] == 2)
          {
            allTimerFiredComplete = false;
          }
        }
        SendTelemetry(millis() - initialTime, 200);
      }
      boolean dataReady = false;
      //wait for 1/2 second
      while ((millis() - lastWriteTime) < 200) {
        if (SerialGPS.available() > 0)
          if (gps.encode(SerialGPS.read()))
            dataReady = true;
      }

      if (dataReady) {
        if (gps.location.isValid()) {
          logger.setFlightLatitudeData((long) (gps.location.lat() * 100000));
          logger.setFlightLongitudeData((long) (gps.location.lng() * 100000));
        } else {
          logger.setFlightLatitudeData((long) (0));
          logger.setFlightLatitudeData((long) (0));
        }
      }
      if (canRecord)
      {
        logger.setFlightTimeData( diffTime);
        logger.setFlightAltitudeData(currAltitude);
        logger.setFlightTemperatureData((long) bmp.readTemperature());
        logger.setFlightPressureData((long) bmp.readPressure());
        float bat = VOLT_DIVIDER * ((float)(analogRead(PB1) * 3300) / (float)4096000);
        logger.setFlightVoltageData((long) 100 * bat); 
        if ( (currentMemaddress + logger.getSizeOfFlightData())  > /*endAddress*/ 65536) {
          //flight is full let's save it
          //save end address
          logger.setFlightEndAddress (currentFileNbr, currentMemaddress - 1);
          canRecord = false;
        } else {
          currentMemaddress = logger.writeFastFlight(currentMemaddress);
          currentMemaddress++;
          lastWriteTime = millis();
        }
        delay(50);
      }

      if (config.superSonicYesNo == 1)
      {
        //are we still in superSonic mode?
        if (currentTime > 3000)
          ignoreAltiMeasure = false;
      }
      if ((currAltitude < lastAltitude) && !apogeeReadyToFire  && !ignoreAltiMeasure )
      {
        measures = measures - 1;
        if (measures == 0)
        {
          //fire drogue
          apogeeReadyToFire = true;
          apogeeStartTime = millis();
          drogueFiredAltitude = currAltitude;
          apogeeAltitude = lastAltitude;
        }
      }
      else
      {
        lastAltitude = currAltitude;
        measures = config.nbrOfMeasuresForApogee;
      }
      if (apogeeReadyToFire && !allApogeeFiredComplete)
      {
        //fire all drogues if delay ok
        for (int ap = 0; ap < 4; ap++ ) {
          if (!outputHasFired[ap] && ((millis() - apogeeStartTime) >= OutputDelay[ap]) && OutputType[ap] == 1) {
            digitalWrite(OutputPins[ap], HIGH);
            outputHasFired[ap] = true;
          }
        }

        for (int ap = 0; ap < 4; ap++ ) {
          if ((millis() - apogeeStartTime ) >= (1000 + OutputDelay[ap]) && !OutputFiredComplete[ap] && OutputType[ap] == 1)
          {
            digitalWrite(OutputPins[ap], LOW);
            setEventState(OutputPins[ap], true);
            OutputFiredComplete[ap] = true;
          }
        }

        allApogeeFiredComplete = true;

        for (int ap = 0; ap < 4; ap++ ) {
          if (!OutputFiredComplete[ap] && OutputType[ap] == 1)
          {
            allApogeeFiredComplete = false;
          }
        }
        SendTelemetry(millis() - initialTime, 200);
      }
      if ((currAltitude  < /*mainDeployAltitude*/ int(config.mainAltitude / FEET_IN_METER)) && allApogeeFiredComplete && !mainReadyToFire && !allMainFiredComplete)
      {
        // Deploy main chute  X meters or feet  before landing...
        mainReadyToFire = true;
#ifdef SERIAL_DEBUG
        SerialCom.println(F("preparing main"));
#endif
        mainStartTime = millis();

        mainAltitude = currAltitude;
#ifdef SERIAL_DEBUG
        SerialCom.println(F("main altitude"));
        SerialCom.println(mainAltitude);
#endif
      }
      if (mainReadyToFire && !allMainFiredComplete)
      {
        //fire main
#ifdef SERIAL_DEBUG
        SerialCom.println(F("firing main"));
#endif
        for (int ma = 0; ma < 4; ma++ ) {
          if (!outputHasFired[ma] && ((millis() - mainStartTime) >= OutputDelay[ma]) && OutputType[ma] == 0) {
            digitalWrite(OutputPins[ma], HIGH);
            outputHasFired[ma] = true;
          }
        }


        for (int ma = 0; ma < 4; ma++ ) {
          if ((millis() - mainStartTime ) >= (1000 + OutputDelay[ma]) && !OutputFiredComplete[ma] && OutputType[ma] == 0)
          {
            digitalWrite(OutputPins[ma], LOW);
            setEventState(OutputPins[ma], true);
            OutputFiredComplete[ma] = true;
          }
        }
        allMainFiredComplete = true;

        for (int ma = 0; ma < 4; ma++ ) {
          if (!OutputFiredComplete[ma] && OutputType[ma] == 0)
          {
            allMainFiredComplete = false;
          }
        }
        SendTelemetry(millis() - initialTime, 200);
      }



      if (landingReadyToFire && !allLandingFiredComplete) {
        //fire all landing that are ready
        for (int la = 0; la < 4; la++ ) {
          if (!outputHasFired[la] && ((millis() - landingStartTime) >= OutputDelay[la] ) && OutputType[la] == 4) {
            digitalWrite(OutputPins[la], HIGH);
            outputHasFired[la] = true;
          }
        }
        for (int la = 0; la < 4; la++ ) {
          if ((millis() - landingStartTime ) >= (1000 + OutputDelay[la])  && !OutputFiredComplete[la] && OutputType[la] == 4)
          {
            digitalWrite(OutputPins[la], LOW);
            setEventState(OutputPins[la], true);
            OutputFiredComplete[la] = true;
          }
        }

        allLandingFiredComplete = true;

        for (int la = 0; la < 4; la++ ) {
          if (!OutputFiredComplete[la] && OutputType[la] == 4)
          {
            allLandingFiredComplete = false;
          }
        }
        SendTelemetry(millis() - initialTime, 200);
      }
      //if ((canRecord && MainFiredComplete && (currAltitude < 10)) || (canRecord && MainFiredComplete && (millis() - mainStartTime) > recordingTimeOut))
      if ((canRecord && allMainFiredComplete && (currAltitude < 10) && allLandingFiredComplete) || (canRecord && allMainFiredComplete && (millis() - mainStartTime) > /*recordingTimeOut*/ (config.recordingTimeout * 1000)))
      {
        //liftOff =false;
        //end loging
        //store start and end address

        //save end address
        logger.setFlightEndAddress (currentFileNbr, currentMemaddress - 1);
#ifdef SERIAL_DEBUG
        SerialCom.println(F("stop recording\n "));
        SerialCom.println(currentMemaddress);
#endif
        logger.writeFlightList();
      }

      if ((allMainFiredComplete && (currAltitude < 10) && allLandingFiredComplete) || (allMainFiredComplete && (millis() - mainStartTime) > /*recordingTimeOut*/ (config.recordingTimeout * 1000)))
      {
#ifdef SERIAL_DEBUG
        SerialCom.println(F("main fired complete"));
#endif
        liftOff = false;
        SendTelemetry(millis() - initialTime, 200);
        //exitRecording = true;
        // we have landed telemetry is not required anymore
        telemetryEnable = false;
      }
      /*#ifdef NBR_PYRO_OUT4
            if (Output1Fired == true && Output2Fired == true && Output3Fired == true && Output4Fired == true && LandingFiredComplete)
        #else
            if (Output1Fired == true && Output2Fired == true && Output3Fired == true && LandingFiredComplete)
        #endif*/
      if (allLandingFiredComplete)
      {
#ifdef SERIAL_DEBUG
        SerialCom.println(F("all event have fired"));
#endif
        exitRecording = true;
        SendTelemetry(millis() - initialTime, 200);
      }
    } // end while (liftoff)
  } //end while(recording)
}


//================================================================
// Function:  Mainloop()
// called for normal recording
//================================================================
/*void Mainloop(void)
  {
  ResetGlobalVar();

  boolean OutputFiredComplete[4] = {false, false, false, false};
  int OutputDelay[4] = {0, 0, 0, 0};
  OutputDelay[0] = config.outPut1Delay;
  OutputDelay[1] = config.outPut2Delay;
  OutputDelay[2] = config.outPut3Delay;
  OutputDelay[3] = config.outPut4Delay;

  // 0 = main 1 = drogue 2 = timer 4 = landing 5 = liftoff 3 = disable
  int OutputType[4] = {3, 3, 3, 3};
  OutputType[0] = config.outPut1;
  OutputType[1] = config.outPut2;
  OutputType[2] = config.outPut3;
  OutputType[3] = config.outPut4;

  int OutputPins[4] = { -1, -1, -1, -1};
  if (config.outPut1 != 3)
    OutputPins[0] = pyroOut1;
  if (config.outPut2 != 3)
    OutputPins[1] = pyroOut2;
  if (config.outPut3 != 3)
    OutputPins[2] = pyroOut3;
  if (config.outPut4 != 3)
    OutputPins[3] = pyroOut4;

  boolean apogeeReadyToFire = false;
  boolean mainReadyToFire = false;
  boolean landingReadyToFire = false;
  boolean liftOffReadyToFire = false;
  unsigned long apogeeStartTime = 0;
  unsigned long mainStartTime = 0;
  unsigned long landingStartTime = 0;
  unsigned long liftOffStartTime = 0;
  boolean ignoreAltiMeasure = false;

  boolean liftOffHasFired = false;
  //hold the state of all our outputs
  boolean outputHasFired[4] = {false, false, false, false};

  if (config.outPut1 == 3) Output1Fired = true;
  if (config.outPut2 == 3) Output2Fired = true;
  if (config.outPut3 == 3) Output3Fired = true;
  if (config.outPut4 == 3) Output4Fired = true;

  long startTime = millis();
  long lastWriteTime = millis();
  unsigned long prevTime = 0;
  long prevAltitude = 0;
  unsigned long diffTime;
  unsigned long currentTime = 0;

  //read current altitude
  currAltitude = (ReadAltitude() - initialAltitude);
  bool lift = false;

  if ( currAltitude > config.liftOffAltitude)
    lift = true;


  if ((lift && !liftOff) || (recording && !liftOff))
  {
    liftOff = true;
    //rocketLanded = false;
    //rocketApogee = false;
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
  }

  if (canRecord && liftOff)
  {
    currentTime = millis() - initialTime;
    if (allMainFiredComplete && !allLandingFiredComplete && !landingReadyToFire) {

      if (abs(currentVelocity(prevTime, currentTime, prevAltitude, currAltitude)) < 1  ) {
        //we have landed
        landingReadyToFire = true;
        landingStartTime = millis();
      }
    }
    prevAltitude = currAltitude;
    SendTelemetry(currentTime, 200);
    diffTime = currentTime - prevTime;
    prevTime = currentTime;

    logger.setFlightTimeData( diffTime);
    logger.setFlightAltitudeData(currAltitude);
    logger.setFlightTemperatureData((long) bmp.readTemperature());
    logger.setFlightPressureData((long) bmp.readPressure());
    boolean dataReady = false;


    //wait for 1/2 second
    while ((millis() - lastWriteTime) < 200) {
      if (SerialGPS.available() > 0)
        if (gps.encode(SerialGPS.read()))
          dataReady = true;
    }

    if (dataReady) {

      if (gps.location.isValid()) {
        logger.setFlightLatitudeData((long) (gps.location.lat() * 100000));
        logger.setFlightLongitudeData((long) (gps.location.lng() * 100000));
      }

    }



    if (dataReady) {
      if ( (currentMemaddress + logger.getSizeOfFlightData())  > endAddress) {
        //flight is full let save it
        //save end address
        logger.setFlightEndAddress (currentFileNbr, currentMemaddress - 1);
        canRecord = false;
      } else {
        currentMemaddress = logger.writeFastFlight(currentMemaddress);
        currentMemaddress++;
      }
    }
    if (!liftOffHasFired && !liftOffReadyToFire) {
      liftOffReadyToFire = true;
      liftOffStartTime = millis();
    }

    if (!allLiftOffFiredComplete) {
      //fire all liftoff that are ready
      for (int li = 0; li < 4; li++ ) {
        if (!outputHasFired[li] && ((millis() - liftOffStartTime) >= OutputDelay[li] ) && OutputType[li] == 5) {
          digitalWrite(OutputPins[li], HIGH);
          outputHasFired[li] = true;
        }
      }
      for (int li = 0; li < 4; li++ ) {
        if ((millis() - liftOffStartTime ) >= (1000 + OutputDelay[li])  && !OutputFiredComplete[li] && OutputType[li] == 5)
        {
          digitalWrite(OutputPins[li], LOW);
          setEventState(OutputPins[li], true);
          OutputFiredComplete[li] = true;
        }
      }

      allLiftOffFiredComplete = true;

      for (int li = 0; li < 4; li++ ) {
        if (!OutputFiredComplete[li] && OutputType[li] == 5)
        {
          allLiftOffFiredComplete = false;
        }
      }
      SendTelemetry(millis() - initialTime, 200);
    }
    // timer events
    if (!allTimerFiredComplete) {
      //fire all timers that are ready
      for (int ti = 0; ti < 4; ti++ ) {
        if (!outputHasFired[ti] && ((currentTime >= OutputDelay[ti]) ) && OutputType[ti] == 2) {
          digitalWrite(OutputPins[ti], HIGH);
          outputHasFired[ti] = true;
        }
      }
      for (int ti = 0; ti < 4; ti++ ) {
        if ((currentTime  >= (1000 + OutputDelay[ti]))  && !OutputFiredComplete[ti] && OutputType[ti] == 2)
        {
          digitalWrite(OutputPins[ti], LOW);
          setEventState(OutputPins[ti], true);
          OutputFiredComplete[ti] = true;
        }
      }

      allTimerFiredComplete = true;

      for (int ti = 0; ti < 4; ti++ ) {
        if (!OutputFiredComplete[ti] && OutputType[ti] == 2)
        {
          allTimerFiredComplete = false;
        }
      }
      SendTelemetry(millis() - initialTime, 200);
    }

    if (config.superSonicYesNo == 1)
    {
      //are we still in superSonic mode?
      if (currentTime > 3000)
        ignoreAltiMeasure = false;
    }
    if ((currAltitude < lastAltitude) && !apogeeReadyToFire  && !ignoreAltiMeasure )
    {
      measures = measures - 1;
      if (measures == 0)
      {
        //fire drogue
        apogeeReadyToFire = true;
        apogeeStartTime = millis();
        //drogueFiredAltitude = currAltitude;
        apogeeAltitude = currAltitude;
      }
    }
    else
    {
      lastAltitude = currAltitude;
      measures = config.nbrOfMeasuresForApogee;
    }

    if (apogeeReadyToFire && !allApogeeFiredComplete)
    {
      //fire all drogues if delay ok
      for (int ap = 0; ap < 4; ap++ ) {
        if (!outputHasFired[ap] && ((millis() - apogeeStartTime) >= OutputDelay[ap]) && OutputType[ap] == 1) {
          digitalWrite(OutputPins[ap], HIGH);
          outputHasFired[ap] = true;
        }
      }

      for (int ap = 0; ap < 4; ap++ ) {
        if ((millis() - apogeeStartTime ) >= (1000 + OutputDelay[ap]) && !OutputFiredComplete[ap] && OutputType[ap] == 1)
        {
          digitalWrite(OutputPins[ap], LOW);
          setEventState(OutputPins[ap], true);
          OutputFiredComplete[ap] = true;
        }
      }

      allApogeeFiredComplete = true;

      for (int ap = 0; ap < 4; ap++ ) {
        if (!OutputFiredComplete[ap] && OutputType[ap] == 1)
        {
          allApogeeFiredComplete = false;
        }
      }
      SendTelemetry(millis() - initialTime, 200);
    }
    if ((currAltitude  < mainDeployAltitude) && allApogeeFiredComplete && !mainReadyToFire && !allMainFiredComplete)
    {
      // Deploy main chute  X meters or feet  before landing...
      mainReadyToFire = true;

      mainStartTime = millis();

      mainAltitude = currAltitude;

    }
    if (mainReadyToFire && !allMainFiredComplete)
    {
      //fire main
      for (int ma = 0; ma < 4; ma++ ) {
        if (!outputHasFired[ma] && ((millis() - mainStartTime) >= OutputDelay[ma]) && OutputType[ma] == 0) {
          digitalWrite(OutputPins[ma], HIGH);
          outputHasFired[ma] = true;
        }
      }


      for (int ma = 0; ma < 4; ma++ ) {
        if ((millis() - mainStartTime ) >= (1000 + OutputDelay[ma]) && !OutputFiredComplete[ma] && OutputType[ma] == 0)
        {
          digitalWrite(OutputPins[ma], LOW);
          setEventState(OutputPins[ma], true);
          OutputFiredComplete[ma] = true;
        }
      }
      allMainFiredComplete = true;

      for (int ma = 0; ma < 4; ma++ ) {
        if (!OutputFiredComplete[ma] && OutputType[ma] == 0)
        {
          allMainFiredComplete = false;
        }
      }
      SendTelemetry(millis() - initialTime, 200);
    }
  }
  if (landingReadyToFire && !allLandingFiredComplete) {
    //fire all landing that are ready
    for (int la = 0; la < 4; la++ ) {
      if (!outputHasFired[la] && ((millis() - landingStartTime) >= OutputDelay[la] ) && OutputType[la] == 4) {
        digitalWrite(OutputPins[la], HIGH);
        outputHasFired[la] = true;
      }
    }
    for (int la = 0; la < 4; la++ ) {
      if ((millis() - landingStartTime ) >= (1000 + OutputDelay[la])  && !OutputFiredComplete[la] && OutputType[la] == 4)
      {
        digitalWrite(OutputPins[la], LOW);
        setEventState(OutputPins[la], true);
        OutputFiredComplete[la] = true;
      }
    }

    allLandingFiredComplete = true;

    for (int la = 0; la < 4; la++ ) {
      if (!OutputFiredComplete[la] && OutputType[la] == 4)
      {
        allLandingFiredComplete = false;
      }
    }
    SendTelemetry(millis() - initialTime, 200);
  }
  if (((canRecord && currAltitude < 10) && liftOff && !recording && !rec) || (!recording && rec))
  {
    liftOff = false;
    rec = false;
    //end loging
    //store start and end address
    logger.setFlightEndAddress (currentFileNbr, currentMemaddress - 1);
    logger.writeFlightList();
    // SerialCom.println("We have landed");
  }



  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
  checkBatVoltage(BAT_MIN_VOLTAGE);
  SendTelemetry(0, 500);

  if (!liftOff) // && !canRecord)
    delay(10);
  }

*/


//================================================================
// Main menu to interpret all the commands sent by the altimeter console
//================================================================
void MainMenu()
{
  char readVal = ' ';
  int i = 0;

  char commandbuffer[100]; // = "";
  float FEET_IN_METER = 1;
  if (config.unit == 0)
    FEET_IN_METER = 1;
  else
    FEET_IN_METER = 3.28084 ;


  /* while ( readVal != ';') {
     continuityCheckAsync();
     if (mainLoopEnable)
       Mainloop();
     while (SerialCom.available())
     {
       readVal = SerialCom.read();
       if (readVal != ';' )
       {
         if (readVal != '\n')
           commandbuffer[i++] = readVal;
       }
       else
       {
         commandbuffer[i++] = '\0';
         resetFlight();
         break;
       }
     }
    }

    interpretCommandBuffer(commandbuffer);
    for (int i = 0; i < sizeof(commandbuffer); i++)
     commandbuffer[i] = '\0';*/

  while ( readVal != ';')
  {
    if (!FastReading)
    {
      currAltitude = (ReadAltitude() - initialAltitude);
      if (liftOff)
        SendTelemetry(millis() - initialTime, 200);
      if (!( currAltitude > config.liftOffAltitude) && !recording)
      {
        //continuityCheckNew();
        continuityCheckAsync();
        long savedTime = millis();
        boolean dataReady = false;
        while ((millis() - savedTime) < 100) {
          if (SerialGPS.available() > 0)
            if (gps.encode(SerialGPS.read()))
              dataReady = true;
        }
        SendTelemetry(0, 500);
        checkBatVoltage(BAT_MIN_VOLTAGE);
         if (timeToSat == 0) {
           if ((int)gps.satellites.value() > 3)
             timeToSat = (millis() - startupTime);
         }

        //SendGPSTram();
      }
      else if ( currAltitude > config.liftOffAltitude)
      {
        recordAltitude();
      }
      else if (recording) {
        record();
      }
      long savedTime = millis();
      while (allApogeeFiredComplete  && allMainFiredComplete )
      {
        // check if we have anything on the serial port
        if (SerialCom.available())
        {
          readVal = SerialCom.read();
          if (readVal != ';' )
          {
            if (readVal != '\n')
              commandbuffer[i++] = readVal;
          }
          else
          {
            commandbuffer[i++] = '\0';
            resetFlight();
            interpretCommandBuffer(commandbuffer);
          }
        }


        //beep last altitude every 10 second
        while ((millis() - savedTime) > 10000) {

          beginBeepSeq();

          if (config.beepingMode == 0)
            beepAltitude(apogeeAltitude * FEET_IN_METER);
          else
            beepAltitudeNew(apogeeAltitude * FEET_IN_METER);
          beginBeepSeq();

          if (config.beepingMode == 0)
            beepAltitude(mainAltitude * FEET_IN_METER);
          else
            beepAltitudeNew(mainAltitude * FEET_IN_METER);

          savedTime = millis();
        }
      }
    }

    while (SerialCom.available())
    {
      readVal = SerialCom.read();
      if (readVal != ';' )
      {
        if (readVal != '\n') {
          commandbuffer[i++] = readVal;
          //SerialCom.print(readVal);
        }
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

/*

   This interprets menu commands. This can be used in the commend line or
   this is used by the Android console

   Commands are as folow:
   a  get all flight data
   b  get altimeter config
   c  toggle continuity on and off
   d  reset alti config
   e  erase all saved flights
   h  hello. Does not do much
   i  unused
   k  folowed by a number turn on or off the selected output
   l  list all flights
   m  followed by a number turn main loop on/off. if number is 1 then
      main loop in on else turn it off
   n  Return the number of recorded flights in the EEprom
   o  send test tram
   r  followed by a number which is the flight number.
      This will retrieve all data for the specified flight
   s  write altimeter config
   t  reset alti config (why?)
   w  Start or stop recording
   x  delete last curve
   y  followed by a number turn telemetry on/off. if number is 1 then
      telemetry in on else turn it off
   z  send gps raw data

*/
void interpretCommandBuffer(char *commandbuffer) {

  //SerialCom.print((char*)commandbuffer);

  //get all flight data
  if (commandbuffer[0] == 'a')
  {
    SerialCom.print(F("$start;\n"));
    //getFlightList()
    int i;
    ///todo
    for (i = 0; i < logger.getLastFlightNbr() + 1; i++)
    {
      logger.printFlightData(i);
    }
    SerialCom.print(F("$end;\n"));
  }
  //get altimeter config
  else if (commandbuffer[0] == 'b')
  {
    SerialCom.print(F("$start;\n"));
    SendAltiConfig();
    SerialCom.print(F("$end;\n"));
  }
  //toggle continuity on and off
  else if (commandbuffer[0] == 'c')
  {
    if (noContinuity == false)
    {
      noContinuity = true;
      SerialCom.println(F("Continuity off \n"));
    }
    else
    {
      noContinuity = false;
      SerialCom.println(F("Continuity on \n"));
    }
  }
  //reset alti config
  else if (commandbuffer[0] == 'd')
  {
    defaultConfig();
    writeConfigStruc();
    initAlti(); //???
  }
  //this will erase all flight
  else if (commandbuffer[0] == 'e')
  {
    SerialCom.println(F("Erase\n"));
    logger.clearFlightList();
    logger.writeFlightList();
    currentFileNbr = 0;
    currentMemaddress = 201;
  }
  //hello
  else if (commandbuffer[0] == 'h')
  {
    //FastReading = false;
    SerialCom.print(F("$OK;\n"));
  }
  //turn on or off the selected output
  else if (commandbuffer[0] == 'k')
  {
    char temp[2];
    boolean fire = true;

    temp[0] = commandbuffer[1];
    temp[1] = '\0';
    if (commandbuffer[2] == 'F')
      fire = false;

    if (atol(temp) > -1)
    {
      switch (atoi(temp))
      {
        case 1:
          fireOutput(pyroOut1, fire);
          break;
        case 2:
          fireOutput(pyroOut2, fire);
          break;
        case 3:
          fireOutput(pyroOut3, fire);
          break;
        case 4:
          fireOutput(pyroOut4, fire);
          break;
      }
    }
  }
  //list all flights
  else if (commandbuffer[0] == 'l')
  {
    SerialCom.println(F("Flight List: \n"));
    logger.printFlightList();
  }
  //mainloop on/off
  else if (commandbuffer[0] == 'm')
  {
    if (commandbuffer[1] == '1') {
#ifdef SERIAL_DEBUG
      SerialCom.print(F("main Loop enabled\n"));
#endif
      mainLoopEnable = true;
      //FastReading = true;
    }
    else {
#ifdef SERIAL_DEBUG
      SerialCom.print(F("main loop disabled\n"));
#endif
      mainLoopEnable = false;
      //FastReading = false;
    }
    SerialCom.print(F("$OK;\n"));
  }
  //Number of flight
  else if (commandbuffer[0] == 'n')
  {
    char flightData[30] = "";
    char temp[9] = "";
    SerialCom.print(F("$start;\n"));
    strcat(flightData, "nbrOfFlight,");
    sprintf(temp, "%i,", logger.getLastFlightNbr() + 1 );
    strcat(flightData, temp);
    unsigned int chk = msgChk(flightData, sizeof(flightData));
    sprintf(temp, "%i", chk);
    strcat(flightData, temp);
    strcat(flightData, ";\n");
    SerialCom.print("$");
    SerialCom.print(flightData);
    SerialCom.print(F("$end;\n"));
  }
  // send test tram
  else if (commandbuffer[0] == 'o')
  {
    SerialCom.print(F("$start;\n"));
    sendTestTram();
    SerialCom.print(F("$end;\n"));
  }
  //altimeter config param
  //write  config
  else if (commandbuffer[0] == 'p')
  {
    if (writeAltiConfigV2(commandbuffer)) {
      SerialCom.print(F("$OK;\n"));
    }
    else
      SerialCom.print(F("$KO;\n"));
  }
  else if (commandbuffer[0] == 'q')
  {
    writeConfigStruc();
    readAltiConfig();
    initAlti();
    SerialCom.print(F("$OK;\n"));
  }

  //this will read one flight
  else if (commandbuffer[0] == 'r')
  {
    char temp[3];
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
      SerialCom.print(F("$start;\n"));
      logger.printFlightData(atoi(temp));
      SerialCom.print(F("$end;\n"));
    }
    else
      SerialCom.println(F("not a valid flight"));
  }
  //write altimeter config
  else if (commandbuffer[0] == 's')
  {
    /*if (writeAltiConfig(commandbuffer)) {
      SerialCom.print(F("$OK;\n"));
      readAltiConfig();
      initAlti();
    }
    else
      SerialCom.print(F("$KO;\n"));*/
  }
  //start or stop recording
  else if (commandbuffer[0] == 'w')
  {
    if (commandbuffer[1] == '1') {
      SerialCom.print(F("Start Recording\n"));
      recording = true;
    }
    else {
      SerialCom.print(F("Stop Recording\n"));
      recording = false;
    }
    SerialCom.print(F("$OK;\n"));
  }
  //telemetry on/off
  else if (commandbuffer[0] == 'y')
  {
    if (commandbuffer[1] == '1') {
      SerialCom.print(F("Telemetry enabled\n"));
      telemetryEnable = true;
    }
    else {
      SerialCom.print(F("Telemetry disabled\n"));
      telemetryEnable = false;
    }
    SerialCom.print(F("$OK;\n"));
  }
  //delete last curve
  else if (commandbuffer[0] == 'x')
  {
    logger.eraseLastFlight();
  }

  //gps rw data on off
  /*else if (commandbuffer[0] == 'z')
    {
    if (commandbuffer[1] == '1') {
      SerialCom.print(F("GPS raw enabled\n"));
    //  GpsRawEnable = true;
    }
    else {
      SerialCom.print(F("GPS raw disabled\n"));
    //      GpsRawEnable = false;
    }
    SerialCom.print(F("$OK;\n"));
    }*/
  // empty command
  else if (commandbuffer[0] == ' ')
  {
    SerialCom.print(F("$K0;\n"));
  }
  else
  {
    SerialCom.println(F("Unknown command" ));
    SerialCom.println(commandbuffer[0]);
  }
}




/*

    Send the GPS configuration to the Android device

*/
void SendAltiConfig() {

  char altiConfig[120] = "";
  char temp[10] = "";

  bool ret = readAltiConfig();
  if (!ret)
    SerialCom.print(F("invalid conf"));
  strcat(altiConfig, "alticonfig,");
  //Unit
  sprintf(temp, "%i,", config.unit);
  strcat(altiConfig, temp);
  //beepingMode
  sprintf(temp, "%i,", config.beepingMode);
  strcat(altiConfig, temp);
  //output1
  sprintf(temp, "%i,", config.outPut1);
  strcat(altiConfig, temp);
  //output2
  sprintf(temp, "%i,", config.outPut2);
  strcat(altiConfig, temp);
  //output3
  sprintf(temp, "%i,", config.outPut3);
  strcat(altiConfig, temp);
  //supersonicYesNo
  sprintf(temp, "%i,", config.superSonicYesNo);
  strcat(altiConfig, temp);
  //mainAltitude
  sprintf(temp, "%i,", config.mainAltitude);
  strcat(altiConfig, temp);
  //AltimeterName
  strcat(altiConfig, BOARD_FIRMWARE);
  strcat(altiConfig, ",");
  //alti major version
  sprintf(temp, "%i,", MAJOR_VERSION);
  strcat(altiConfig, temp);
  //alti minor version
  sprintf(temp, "%i,", MINOR_VERSION);
  strcat(altiConfig, temp);
  //output1 delay
  sprintf(temp, "%i,", config.outPut1Delay);
  strcat(altiConfig, temp);
  //output2 delay
  sprintf(temp, "%i,", config.outPut2Delay);
  strcat(altiConfig, temp);
  //output3 delay
  sprintf(temp, "%i,", config.outPut3Delay);
  strcat(altiConfig, temp);
  //Beeping frequency
  sprintf(temp, "%i,", config.beepingFrequency);
  strcat(altiConfig, temp);
  sprintf(temp, "%i,", config.nbrOfMeasuresForApogee);
  strcat(altiConfig, temp);
  sprintf(temp, "%i,", config.endRecordAltitude);
  strcat(altiConfig, temp);
  sprintf(temp, "%i,", config.recordTemperature);
  strcat(altiConfig, temp);
  sprintf(temp, "%i,", config.superSonicDelay);
  strcat(altiConfig, temp);
  sprintf(temp, "%lu,", config.connectionSpeed);
  strcat(altiConfig, temp);
  sprintf(temp, "%i,", config.altimeterResolution);
  strcat(altiConfig, temp);
  sprintf(temp, "%i,", config.eepromSize);
  strcat(altiConfig, temp);
  sprintf(temp, "%i,", config.noContinuity);
  strcat(altiConfig, temp);
  //output4
  sprintf(temp, "%i,", config.outPut4);
  strcat(altiConfig, temp);
  //output4 delay
  sprintf(temp, "%i,", config.outPut4Delay);
  strcat(altiConfig, temp);
  //Lift off altitude
  sprintf(temp, "%i,", config.liftOffAltitude);
  strcat(altiConfig, temp);
  //Battery type
  sprintf(temp, "%i,", config.batteryType);
  strcat(altiConfig, temp);
  // recording timeout
  sprintf(temp, "%i,", config.recordingTimeout);
  strcat(altiConfig, temp);
  unsigned int chk = 0;
  chk = msgChk( altiConfig, sizeof(altiConfig) );
  sprintf(temp, "%i;\n", chk);
  strcat(altiConfig, temp);

  SerialCom.print("$");
  SerialCom.print(altiConfig);
}

/*

   Check if the battery voltage is OK.
   If not warn the user so that the battery does not get
   damaged by over discharging
*/
void checkBatVoltage(float minVolt) {

  pinMode(PB1, INPUT_ANALOG);
  int batVoltage = analogRead(PB1);

  float bat = VOLT_DIVIDER * ((float)(batVoltage * 3300) / (float)4096000);

    if (bat < minVolt) {
      for (int i = 0; i < 10; i++)
      {
        tone(pinSpeaker, 1600, 1000);
        delay(50);
        noTone(pinSpeaker);
      }
      delay(1000);
    }
}
/*
   Turn on or off one altimeter output
   This is used to test them
*/
void fireOutput(int pin, boolean fire) {
  if (fire)
    digitalWrite(pin, HIGH);
  else
    digitalWrite(pin, LOW);
}

/*
    Test tram
*/
void sendTestTram() {

  char altiTest[100] = "";
  char temp[10] = "";

  strcat(altiTest, "testTrame," );
  strcat(altiTest, "Bear altimeters are the best!!!!,");
  unsigned int chk;
  chk = msgChk(altiTest, sizeof(altiTest));
  sprintf(temp, "%i", chk);
  strcat(altiTest, temp);
  strcat(altiTest, ";\n");
  SerialCom.print("$");
  SerialCom.print(altiTest);

}

/*

   re-nitialise all flight related global variables

*/
void resetFlight() {
  //  recordingTimeOut = config.recordingTimeout * 1000;
  allApogeeFiredComplete  = false;
  allMainFiredComplete = false;
  allTimerFiredComplete = false;
  allLiftOffFiredComplete = false;
  allLandingFiredComplete = false;
  liftOff = false;
  Output1Fired = false;
  Output2Fired = false;
  Output3Fired = false;
#ifdef NBR_PYRO_OUT4
  Output4Fired = false;
#endif

  apogeeAltitude = 0;
  logger.readFlightList();
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
}
/*void SendGPSTram() {
  unsigned long ms = 100;
  unsigned long start = millis();
  if (GpsRawEnable) {
    do
    {
      while (SerialGPS.available())
        SerialCom.print(SerialGPS.read());
    } while (millis() - start < ms);
  }
  }*/
