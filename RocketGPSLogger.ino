/*
  Rocket GPS Logger ver 1.0
  Copyright Boris du Reau 2012-2020
  Description: Model Rocket GPS flight recorder. This will allow you to record your flight
  Author: Boris du Reau
  Date: January 2020
  Sensor used is an a BMP180 board

  You can use a stm32F103C board. I am using my STM32 altimeter board that has 2 serial ports.

  GPS is on SerialGPS

  Major changes on version 1.0
  Initial version
  TODO:

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
long endAddress = 65536;
// current file number that you are recording
int currentFileNbr = 0;

// EEPROM start adress for the flights.
// Anything before that are the flights indexes
long currentMemaddress = 200;
boolean liftOff = false;
boolean landed = true;
//ground level altitude
long initialAltitude;
// any altitude above that and we consider that it is a liftoff
long liftoffAltitude = 20;
long lastAltitude;
//current altitude
long currAltitude;
//Apogee altitude
long apogeeAltitude;
long mainAltitude;
bool canRecord;
bool recording = false;
bool rec = false;
unsigned long initialTime = 0;
unsigned long prevTime = 0;
unsigned long diffTime;
unsigned long currentTime = 0;
long lastTelemetry = 0;

boolean mainHasFired = false;
//nbr of measures to do so that we are sure that apogee has been reached
unsigned long measures = 5;
unsigned long mainDeployAltitude;
float FEET_IN_METER = 1;
// to store all event
boolean timerEvent1_enable = false;
boolean timerEvent2_enable = false;
boolean timerEvent3_enable = false;
boolean timerEvent4_enable = false;

boolean apogeeEvent_Enable = false;
boolean mainEvent_Enable = false;
// enable/disable output
boolean out1Enable = true;
boolean out2Enable = true;
boolean out3Enable = true;
boolean out4Enable = true;

// main loop
//boolean mainLoopEnable =true;

int apogeeDelay = 0;
int mainDelay = 0;
int out1Delay = 0;
int out2Delay = 0;
int out3Delay = 0;
int out4Delay = 0;

boolean Output1Fired = false;
boolean Output2Fired = false;
boolean Output3Fired = false;
boolean Output4Fired = false;
//by default apogee pin
const int pinChannel1Continuity = PA2;
// by default continuity for the main
const int pinChannel2Continuity = PA4;
// third output
const int pinChannel3Continuity = PA6;
// fourth output
const int pinChannel4Continuity = PB0;
void assignPyroOutputs();



// var for the main loop
boolean apogeeReadyToFire = false;
boolean mainReadyToFire = false;
unsigned long apogeeStartTime = 0;
unsigned long mainStartTime = 0;
//unsigned long liftoffStartTime=0;
boolean ignoreAltiMeasure = false;
// boolean finishedEvent = false;
boolean Event1Fired = false;
boolean Event2Fired = false;
boolean Event3Fired = false;
boolean Event4Fired = false;
boolean MainFiredComplete = false;

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
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F\r\n"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C\r\n"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F\r\n"

void initAlti() {
  out1Enable = true;
  out2Enable = true;
  out3Enable = true;
  out4Enable = true;

  // set main altitude (if in feet convert to metrics)
  if (config.unit == 0)
    FEET_IN_METER = 1;
  else
    FEET_IN_METER = 3.28084 ;

  mainDeployAltitude = int(config.mainAltitude / FEET_IN_METER);
  // beepFrequency
  beepingFrequency = config.beepingFrequency;

  assignPyroOutputs();

  //number of measures to do to detect Apogee
  measures = config.nbrOfMeasuresForApogee;

  //check which pyro are enabled
  pos = -1;
  // The altimeter board has 4 pyro so we might as well use them
  if (out1Enable) {
    pos++;
    continuityPins[pos] = pinChannel1Continuity;
  }
  if (out2Enable) {
    pos++;
    continuityPins[pos] = pinChannel2Continuity;
  }
  if (out3Enable) {
    pos++;
    continuityPins[pos] = pinChannel3Continuity;
  }
  if (out4Enable)  {
    pos++;
    continuityPins[pos] = pinChannel4Continuity;
  }
}

/*
    Setup and start program
*/
void setup()
{
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

  Wire.begin();
  //Init bluetooth (which is on Serial 1)
  SerialCom.begin(38400);
  //Init GPS serial port (Which is on Serial 3)
  SerialGPS.begin(9600);

  SerialGPS.print(PMTK_SET_NMEA_UPDATE_5HZ);

  while (!SerialCom);      // wait for Leonardo enumeration, others continue immediately
  bmp.begin( config.altimeterResolution);
  initAlti();
  if (out1Enable == false) Output1Fired = true;
  if (out2Enable == false) Output2Fired = true;
  if (out3Enable == false) Output3Fired = true;
  if (out4Enable == false) Output4Fired = true;



  // init Kalman filter
  KalmanInit();
  // let's do some dummy altitude reading
  // to initialise the Kalman filter
  for (int i = 0; i < 50; i++) {
    ReadAltitude();
  }

  long sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += ReadAltitude();
    delay(50);
  }
  initialAltitude = (sum / 10.0);
  lastAltitude = 0;
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
  liftoffAltitude = config.liftOffAltitude;


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
      //SerialCom.println(F("hello"));
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
    //SerialCom.println("We have a liftoff");
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
    while ((millis() - lastWriteTime) < 500) {
      if (SerialGPS.available() > 0)
        if (gps.encode(SerialGPS.read()))
          dataReady = true;
    }

    if (dataReady) {
      /*if (gps.gprmc_status() == 'A') {
        logger.setFlightLatitudeData((long) (gps.gprmc_latitude()*1000));
        logger.setFlightLongitudeData((long) (gps.gprmc_longitude() *1000));
        }*/
      if (gps.location.isValid()) {
        logger.setFlightLatitudeData((long) (gps.location.lat() * 100000));
        logger.setFlightLongitudeData((long) (gps.location.lng() * 100000));
      }

    }

    if (dataReady) {
      //SerialCom.println("writting");
      currentMemaddress = logger.writeFastFlight(currentMemaddress);
      currentMemaddress++;
      lastWriteTime = millis();
    }

    //////
    SerialCom.println(timerEvent1_enable);
    if (timerEvent1_enable && Event1Fired == false)
    {
      //SerialCom.println("hello1");
      //SerialCom.println("before Event1Fired");
      if (currentTime >= config.outPut1Delay)
      {
        //fire output pyroOut1
        digitalWrite(pyroOut1, HIGH);
        Event1Fired = true;
        SerialCom.println("Event1Fired");
      }
    }
    if (timerEvent1_enable && Event1Fired == true)
    {

      if ((currentTime - config.outPut1Delay) >= 1000 && Output1Fired == false)
      {
        //switch off output pyroOut1
        digitalWrite(pyroOut1, LOW);
        Output1Fired = true;
      }
    }
    if (timerEvent2_enable && Event2Fired == false)
    {
      //SerialCom.println("hello2");
      if (currentTime >= config.outPut2Delay)
      {
        //fire output pyroOut2
        digitalWrite(pyroOut2, HIGH);
        Event2Fired = true;
      }
    }
    if (timerEvent2_enable && Event2Fired == true )
    {
      if ((currentTime - config.outPut2Delay) >= 1000 && Output2Fired == false)
      {
        //switch off output pyroOut2
        digitalWrite(pyroOut2, LOW);
        Output2Fired = true;
      }
    }
    if (timerEvent3_enable && Event3Fired == false)
    {
      //SerialCom.println("hello3");
      if (currentTime >= config.outPut3Delay)
      {
        //fire output pyroOut3
        digitalWrite(pyroOut3, HIGH);
        Event3Fired = true;
      }
    }
    if (timerEvent3_enable && Event3Fired == true)
    {
      if ((currentTime - config.outPut3Delay) >= 1000 && Output3Fired == false)
      {
        //switch off output pyroOut3
        digitalWrite(pyroOut3, LOW);
        Output3Fired = true;
      }
    }

    if (timerEvent4_enable && Event4Fired == false)
    {
      //SerialCom.println("hello4");
      if (currentTime >= config.outPut4Delay)
      {
        //fire output pyroOut4
        digitalWrite(pyroOut4, HIGH);
        Event4Fired = true;

      }
    }
    if (timerEvent4_enable && Event4Fired == true)
    {
      if ((currentTime - config.outPut4Delay) >= 1000 && Output4Fired == false)
      {
        //switch off output pyroOut4
        digitalWrite(pyroOut4, LOW);
        Output4Fired = true;

      }
    }
    if (config.superSonicYesNo == 1)
    {
      //are we still in superSonic mode?
      if (currentTime > 3000)
        ignoreAltiMeasure = false;
    }
    if ((currAltitude < lastAltitude) && apogeeHasFired == false && ignoreAltiMeasure == false)
    {
      measures = measures - 1;
      if (measures == 0)
      {
        //fire drogue
        apogeeReadyToFire = true;
        apogeeStartTime = millis();
        apogeeAltitude = currAltitude;
      }
    }
    else
    {
      lastAltitude = currAltitude;
      measures = config.nbrOfMeasuresForApogee;
    }
    if (apogeeReadyToFire)
    {
      if ((millis() - apogeeStartTime) >= apogeeDelay)
      {
        //fire drogue
        digitalWrite(pinApogee, HIGH);
        setEventState(pinApogee, true);
#ifdef SERIAL_DEBUG
        SerialCom.println(F("Apogee has fired"));
#endif
        apogeeReadyToFire = false;
        apogeeHasFired = true;
        SendTelemetry(millis() - initialTime, 200);
      }
    }
    if ((currAltitude  < mainDeployAltitude) && apogeeHasFired == true && mainHasFired == false)
    {
      // Deploy main chute  X meters or feet  before landing...
      digitalWrite(pinApogee, LOW);
#ifdef SERIAL_DEBUG
      SerialCom.println(F("Apogee firing complete"));
#endif
      mainReadyToFire = true;
#ifdef SERIAL_DEBUG
      SerialCom.println(F("preparing main"));
#endif
      mainStartTime = millis();
      //digitalWrite(pinMain, HIGH);
      //mainHasFired=true;
      mainAltitude = currAltitude;
#ifdef SERIAL_DEBUG
      SerialCom.println(F("main altitude"));

      SerialCom.println(mainAltitude);
#endif
    }
    if (mainReadyToFire)
    {
      if ((millis() - mainStartTime) >= mainDelay)
      {
        //fire main
#ifdef SERIAL_DEBUG
        SerialCom.println(F("firing main"));
#endif
        digitalWrite(pinMain, HIGH);
        mainReadyToFire = false;
        //setEventState(pinMain, true);
        mainHasFired = true;
        SendTelemetry(millis() - initialTime, 200);
      }
    }

    if (mainHasFired)
    {

      if ((millis() - (mainStartTime + mainDelay)) >= 1000 && MainFiredComplete == false)
      {
        digitalWrite(pinMain, LOW);
        setEventState(pinMain, true);
        //liftOff =false;
#ifdef SERIAL_DEBUG
        SerialCom.println("Main fired");
#endif
        MainFiredComplete = true;
      }
    }
    /////
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




//================================================================
// Main menu to interpret all the commands sent by the altimeter console
//================================================================
void MainMenu()
{
  char readVal = ' ';
  int i = 0;

  char commandbuffer[300];


  while ( readVal != ';') {
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
   e  erase all saved flights
   r  followed by a number which is the flight number.
      This will retrieve all data for the specified flight
   w  Start or stop recording
   n  Return the number of recorded flights in the EEprom
   l  list all flights
   c  toggle continuity on and off
   a  get all flight data
   b  get altimeter config
   s  write altimeter config
   d  reset alti config
   h  hello. Does not do much
   k  folowed by a number turn on or off the selected output
   y  followed by a number turn telemetry on/off. if number is 1 then
      telemetry in on else turn it off
   m  followed by a number turn main loop on/off. if number is 1 then
      main loop in on else turn it off
*/
void interpretCommandBuffer(char *commandbuffer) {

  //this will erase all flight
  if (commandbuffer[0] == 'e')
  {
    SerialCom.println(F("Erase\n"));
    logger.clearFlightList();
    logger.writeFlightList();
    currentFileNbr = 0;
    currentMemaddress = 201;
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
  //Number of flight
  else if (commandbuffer[0] == 'n')
  {
    SerialCom.print(F("$start;\n"));
    SerialCom.print(F("$nbrOfFlight,"));
    logger.readFlightList();
    SerialCom.print(logger.getLastFlightNbr());
    SerialCom.print(";\n");
    SerialCom.print(F("$end;\n"));
  }
  //list all flights
  else if (commandbuffer[0] == 'l')
  {
    SerialCom.println(F("Flight List: \n"));
    logger.printFlightList();
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
  //get all flight data
  else if (commandbuffer[0] == 'a')
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
  //write altimeter config
  else if (commandbuffer[0] == 's')
  {
    if (writeAltiConfig(commandbuffer))
      SerialCom.print(F("$OK;\n"));
    else
      SerialCom.print(F("$KO;\n"));
  }
  //reset alti config
  else if (commandbuffer[0] == 'd')
  {
    defaultConfig();
    writeConfigStruc();
    //initAlti(); ???
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
  //mainloop on/off
  else if (commandbuffer[0] == 'm')
  {
    if (commandbuffer[1] == '1') {
#ifdef SERIAL_DEBUG
      SerialCom.print(F("main Loop enabled\n"));
#endif
      mainLoopEnable = true;
    }
    else {
#ifdef SERIAL_DEBUG
      SerialCom.print(F("main loop disabled\n"));
#endif
      mainLoopEnable = false;
    }
    SerialCom.print(F("$OK;\n"));
  }
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
   Just to assign pyro output fonctionalities
*/
void assignPyroOutputs()
{
  pinMain = -1;
  pinApogee = -1;
  pinOut1 = -1;
  pinOut2 = -1;
  pinOut3 = -1;
  pinOut4 = -1;


  switch (config.outPut1)
  {
    case 0:
      mainEvent_Enable = true;
      mainDelay = config.outPut1Delay;
      pinMain = pyroOut1;
      break;
    case 1:
      apogeeEvent_Enable = true;
      apogeeDelay = config.outPut1Delay;
      pinApogee = pyroOut1;
      break;
    case 2:
      timerEvent1_enable = true;
      out1Delay = config.outPut1Delay;
      pinOut1 = pyroOut1;
      break;
    default:
      out1Enable = false;
      break;
  }

  switch (config.outPut2)
  {
    case 0:
      mainEvent_Enable = true;
      pinMain = pyroOut2;
      mainDelay = config.outPut2Delay;
      break;
    case 1:
      apogeeEvent_Enable = true;
      pinApogee = pyroOut2;
      apogeeDelay = config.outPut2Delay;
      break;
    case 2:
      timerEvent2_enable = true;
      out2Delay = config.outPut2Delay;
      pinOut2 = pyroOut2;
      break;
    default:
      out2Enable = false;
      break;
  }
  switch (config.outPut3)
  {
    case 0:
      mainEvent_Enable = true;
      mainDelay = config.outPut3Delay;
      pinMain = pyroOut3;
      break;
    case 1:
      apogeeEvent_Enable = true;
      apogeeDelay = config.outPut3Delay;
      pinApogee = pyroOut3;
      break;
    case 2:
      timerEvent3_enable = true;
      out3Delay = config.outPut3Delay;
      pinOut3 = pyroOut3;
      break;
    default:
      out3Enable = false;
      break;
  }
  //output 4
  switch (config.outPut4)
  {
    case 0:
      mainEvent_Enable = true;
      mainDelay = config.outPut4Delay;
      pinMain = pyroOut4;
      break;
    case 1:
      apogeeEvent_Enable = true;
      apogeeDelay = config.outPut4Delay;
      pinApogee = pyroOut4;
      break;
    case 2:
      timerEvent4_enable = true;
      out4Delay = config.outPut4Delay;
      pinOut4 = pyroOut4;
      break;
    default:
      out4Enable = false;
      break;
  }
}

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
    if (apogeeHasFired)
      ap = 1;

    //check main
    int ma = 0;
    if (mainHasFired)
      ma = 1;
    int landed = 0;
    if ( mainHasFired && currAltitude < 10)
      landed = 1;
    //SerialCom.print(F("$telemetry,"));
    strcat(altiTelem, "telemetry," );
    //SerialCom.print(currAltitude);
    //SerialCom.print(F(","));
    sprintf(temp, "%i,", currAltitude);
    strcat(altiTelem, temp);
    //SerialCom.print(li);
    //SerialCom.print(F(","));
    sprintf(temp, "%i,", li);
    strcat(altiTelem, temp);
    //SerialCom.print(ap);
    //SerialCom.print(F(","));
    sprintf(temp, "%i,", ap);
    strcat(altiTelem, temp);
    //SerialCom.print(apogeeAltitude);
    //SerialCom.print(F(","));
    sprintf(temp, "%i,", apogeeAltitude);
    strcat(altiTelem, temp);
    //SerialCom.print(ma);
    //SerialCom.print(F(","));
    sprintf(temp, "%i,", ma);
    strcat(altiTelem, temp);
    //SerialCom.print(mainAltitude);
    //SerialCom.print(F(","));
    sprintf(temp, "%i,", mainAltitude);
    strcat(altiTelem, temp);
    //SerialCom.print(landed);
    //SerialCom.print(F(","));
    sprintf(temp, "%i,", landed);
    strcat(altiTelem, temp);
    //SerialCom.print(sampleTime);
    //SerialCom.print(F(","));
    sprintf(temp, "%i,", sampleTime);
    strcat(altiTelem, temp);
    if (out1Enable) {
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
    if (out2Enable) {
      //check continuity
      val = digitalRead(pinChannel2Continuity);
      delay(20);
      if (val == 0)
        strcat(altiTelem, "0,");
      else
        strcat(altiTelem, "1,");
    }
    else {
      strcat(altiTelem, "-1,");
    }
    if (out3Enable) {
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
    if (out4Enable) {
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
    sprintf(temp, "%f,", bat);
    strcat(altiTelem, temp);

    // temperature
    float temperature;
    temperature = bmp.readTemperature();
    sprintf(temp, "%i,", (int)temperature );
    strcat(altiTelem, temp);

    sprintf(temp, "%i,", (int)(100 * ((float)logger.getLastFlightEndAddress() / endAddress)) );
    strcat(altiTelem, temp);
    sprintf(temp, "%i,", logger.getLastFlightNbr() + 1 );
    strcat(altiTelem, temp);
    //SerialCom.print((long)gps.location.lat() * 100000);
    // SerialCom.print(F(","));
    sprintf(temp, "%l,", (long)gps.location.lat() * 100000 );
    strcat(altiTelem, temp);
    //SerialCom.print((long) (gps.location.lng() * 100000));
    //SerialCom.println(F(";"));
    sprintf(temp, "%l,", (long)gps.location.lng() * 100000 );
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

    Send the GPS configuration to the Android device

*/
void SendAltiConfig() {

  char altiConfig[150] = "";
  char temp[10] = "";
  
  bool ret = readAltiConfig();
  if (!ret)
    SerialCom.print(F("invalid conf"));
  //SerialCom.print(F("$alticonfig"));
 // SerialCom.print(F(","));
   strcat(altiConfig, "alticonfig,");
  //Unit
  //SerialCom.print(config.unit);
  //SerialCom.print(F(","));
  sprintf(temp, "%i,", config.unit);
  strcat(altiConfig, temp);
  //beepingMode
  //SerialCom.print(config.beepingMode);
  //SerialCom.print(F(","));
  sprintf(temp, "%i,", config.beepingMode);
  strcat(altiConfig, temp);
  //output1
  //SerialCom.print(config.outPut1);
  //SerialCom.print(F(","));
  sprintf(temp, "%i,", config.outPut1);
  strcat(altiConfig, temp);
  //output2
  //SerialCom.print(config.outPut2);
  //SerialCom.print(F(","));
  sprintf(temp, "%i,", config.outPut2);
  strcat(altiConfig, temp);
  //output3
  //SerialCom.print(config.outPut3);
  //SerialCom.print(F(","));
  sprintf(temp, "%i,", config.outPut3);
  strcat(altiConfig, temp);
  //supersonicYesNo
  //SerialCom.print(config.superSonicYesNo);
  //SerialCom.print(F(","));
   sprintf(temp, "%i,", config.superSonicYesNo);
  strcat(altiConfig, temp);
//mainAltitude
  //SerialCom.print(config.mainAltitude);
  //SerialCom.print(F(","));
  sprintf(temp, "%i,", config.mainAltitude);
  strcat(altiConfig, temp);
  //AltimeterName
  //SerialCom.print(F(BOARD_FIRMWARE));
  //SerialCom.print(F(","));
  //sprintf(temp, "%i,", BOARD_FIRMWARE);
  strcat(altiConfig, BOARD_FIRMWARE);
  strcat(altiConfig,",");
  //alti major version
  // SerialCom.print(MAJOR_VERSION);
  sprintf(temp, "%i,", MAJOR_VERSION);
  strcat(altiConfig, temp);
  //alti minor version
  //SerialCom.print(F(","));
  //SerialCom.print(MINOR_VERSION);
  //SerialCom.print(F(","));
  sprintf(temp, "%i,", MINOR_VERSION);
  strcat(altiConfig, temp);
    //output1 delay
  //SerialCom.print(config.outPut1Delay);
  //SerialCom.print(F(","));
  sprintf(temp, "%i,", config.outPut1Delay);
  strcat(altiConfig, temp);
  //output2 delay
  //SerialCom.print(config.outPut2Delay);
  //SerialCom.print(F(","));
  sprintf(temp, "%i,", config.outPut2Delay);
  strcat(altiConfig, temp);
  //output3 delay
  //SerialCom.print(config.outPut3Delay);
  //SerialCom.print(F(","));
  sprintf(temp, "%i,", config.outPut3Delay);
  strcat(altiConfig, temp);
  //Beeping frequency
  //SerialCom.print(config.beepingFrequency);
  //SerialCom.print(F(","));
  sprintf(temp, "%i,", config.beepingFrequency);
  strcat(altiConfig, temp);
  //SerialCom.print(config.nbrOfMeasuresForApogee);
  //SerialCom.print(F(","));
  sprintf(temp, "%i,", config.nbrOfMeasuresForApogee);
  strcat(altiConfig, temp);
   //SerialCom.print(config.endRecordAltitude);
  //SerialCom.print(F(","));
  sprintf(temp, "%i,", config.endRecordAltitude);
  strcat(altiConfig, temp);
  //SerialCom.print(config.recordTemperature);
  //SerialCom.print(F(","));
  sprintf(temp, "%i,", config.recordTemperature);
  strcat(altiConfig, temp);
  //SerialCom.print(config.superSonicDelay);
  //SerialCom.print(F(","));
  sprintf(temp, "%i,", config.superSonicDelay);
  strcat(altiConfig, temp);
  //SerialCom.print(config.connectionSpeed);
  //SerialCom.print(F(","));
  sprintf(temp, "%lu,", config.connectionSpeed);
  strcat(altiConfig, temp);
   //SerialCom.print(config.altimeterResolution);
  //SerialCom.print(F(","));
  sprintf(temp, "%i,", config.altimeterResolution);
  strcat(altiConfig, temp);
  //SerialCom.print(config.eepromSize);
  //SerialCom.print(F(","));
  sprintf(temp, "%i,", config.eepromSize);
  strcat(altiConfig, temp);
  //SerialCom.print(config.noContinuity);
  //SerialCom.print(F(","));
  sprintf(temp, "%i,", config.noContinuity);
  strcat(altiConfig, temp);
  //output4
  //SerialCom.print(config.outPut4);
  //SerialCom.print(F(","));
  sprintf(temp, "%i,", config.outPut4);
  strcat(altiConfig, temp);
  //output4 delay
  //SerialCom.print(config.outPut4Delay);
  //SerialCom.print(F(";\n"));
  sprintf(temp, "%i,", config.outPut4Delay);
  strcat(altiConfig, temp);
  //Lift off altitude
  //SerialCom.print(config.liftOffAltitude);
  //SerialCom.print(F(","));
  sprintf(temp, "%i,", config.liftOffAltitude);
  strcat(altiConfig, temp);
  //Battery type
  //SerialCom.print(config.batteryType);
  sprintf(temp, "%i,", config.batteryType);
  //SerialCom.print(F(";\n"));
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
