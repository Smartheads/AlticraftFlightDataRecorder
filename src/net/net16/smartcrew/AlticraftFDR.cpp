/*
* MIT License
*
* Copyright (c) 2019 Robert Hutter
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* AlticraftFDR.cpp - Main file of the Alticraft Flight Data Recorder software.
*   Execution start here. Program features specified in the SRS.
*
*/
#include <RGBLED.h>
#include <SPI.h>
#include <SD.h>
#include <avr/pgmspace.h>

/* BEGINNING OF PROGRAM PREFERENCES */

#define DEBUG ON                      // Comment this line out if you don't want debug
#define NEWFILE_INTERVAL_TIME 3000000   // Change the interval between savin into a new file (us)
#define LAUNCH_ARM_TIME 3000          // Change the launch arm time (ms)
#define LAUNCH_COUNTDOWN 10000          // Change the launch countdown time (ms)
#define SHUTDOWN_TIME 3000            // Change the shutdown timer (ms)
#define BAUD_RATE 115200              // Change baud rate here

#define LAUNCH_LOG_STAGE ACTIVE            // Change operation mode here

/*  OPERATION MODES: (use double quotes)
*     - "PASSIVE_LOG": Only log flight data. Disables launch and staging features.
*     - "ONLY_LAUNCH_AND_LOG": Enables only launch control and flight data logging.
*     - "LAUNCH_LOG_STAGE": Enables all features including launch, staging control
*         and flight data logging.
*/

#define ALTITUDE ACTIVE               // Change trigger mode here
#define TRIGGER_ALTITUDE 200          // Change trigger altitude here
#define TRIGGER_PRESSURE 100.0        // Change trigger pressure here

/*  STAGE TRIGGER MODES: (use double quotes)
*     - "ALTITUDE": Activate staging at a specific altitude. (m)
*     - "PRESSURE": Activate staging at a specific pressure. (kPa)
*/

#define SD_CHIP_SELECT 10             // Change this to match your SD shield or module

/* SD_CHIP_SELECT options:
*    - 4: Arduino Ethernet shield
*    - 10: Adafruit SD shields and modules, Catalex SD card adapter
*    - 8 Sparkfun SD shield
*    - SDCARD_SS_PIN: MKRZero SD
*/

/* END OF PREFERENCES*/

#if defined LAUNCH_LOG_STAGE || defined ONLY_LAUNCH_AND_LOG
  #include <Buzzer.h>
#endif
#ifdef LAUNCH_LOG_STAGE
  #include <Servo.h>
#endif

/* 
* Preprocessing settings
*
*/
#if defined PASSIVE_LOG
  #define OPERATION_MODE "PASSIVE_LOG"
#endif

#if defined ONLY_LAUNCH_AND_LOG
  #define OPERATION_MODE "ONLY_LAUNCH_AND_LOG"
#endif

#if defined LAUNCH_LOG_STAGE
  #define OPERATION_MODE "LAUNCH_LOG_STAGE"
#endif

#if defined ALTITUDE
  #define TRIGGER_MODE "ALTITUDE"
#endif

#if defined PRESSURE
  #define TRIGGER_MODE "PRESSURE"
#endif

/* Pins */
#define RED_PIN 2
#define GREEN_PIN 4
#define BLUE_PIN 5
#define BUZZER_PIN 9
#define BUTTON_PIN 6
#define LAUNCH_PIN 8
#define SERVO_PIN 7
/* END of pins */

/* Device names */
#define BMP280 F("BMP280")
#define MPU9250 F("MPU9250")
/* END of device names */

/* Messages that cant be saved to PROGMEM */
#define DATAHEADER1 "[+"
#define DATAHEADER2 "us/"
#define DATAHEADER3 "]:\t"

#ifdef DEBUG
  /* All other messages */
  const char init2[] PROGMEM = {"--------------------------------------------\nINITIALIZATION PHASE BEGINNING_\n"};
  const char buzzer_init[] PROGMEM = {"Initializing buzzer...\n"};
  const char rgb_init[] PROGMEM = {"Initializing RGB LED...\n"};
  const char sd_init[] PROGMEM = {"Initializing SD module...\n"};
  const char sd_type1[] PROGMEM = {"CARD TYPE: "};
  const char sd_clusters[] PROGMEM = {"CLUSTERS: "};
  const char sd_blockspersecond[] PROGMEM = {"BLOCKS x CLUSTER: "};
  const char sd_totalblocks[] PROGMEM = {"TOTAL BLOCKS: "};
  const char volume_type[] PROGMEM = {"VOLUME TYPE: FAT"};
  const char volume_size1[] PROGMEM = {"VOLUME SIZE: "};
  const char volume_size2[] PROGMEM = {" kb_\n"};
  const char end_line[] PROGMEM = {"_\n"};
  const char test[] PROGMEM = {"INITIALIZATION PHASE ENDED_\n--------------------------------------------\nTESTING PHASE BEGINNING_\n"};
  const char test_err[] PROGMEM = {"--------------------------------------------\nAN ERROR HAS OCCURED DURING TESTING PHASE_\nREVIEW LOG FOR MORE INFORMATION_\nABORTING SETUP_\nSETTING LED TO WHITE_\nRESTART TO TRY AGAIN_\n"};
  const char test_ok[] PROGMEM = {"TESTING PHASE ENDED_\nALL SYSTEMS NOMINAL_\n"};
  const char sd_init_err[] PROGMEM = {"Error initializing SD card...\n"};
  const char volume_init_err[] PROGMEM = {"Error initializing SD volume...\n"};
  const char mk_worksp_err[] PROGMEM = {"Error creating workspace...\n"};
  const char op_worksp_err[] PROGMEM = {"Error opening workspace...\n"};
  const char wait[] PROGMEM = {"ARMING IGNITION SYSTEM_\n--------------------------------------------\nWAITING FOR LAUNCH SIGNAL_\n"};
  const char warn[] PROGMEM = {"LAUNCH SIGNAL RECIVED_\n--------------------------------------------\nWARNING! IGNITION IN 10 SECONDS_\nPRESS ANY BUTTON TO CANCEL_\n"};
  const char launch1[] PROGMEM = {"--------------------------------------------\nLIFTOFF_\nIGNITION AT: "};
  const char launch2[] PROGMEM = {" microseconds_\n"};
  const char startdatalog[] PROGMEM = {"--------------------------------------------\n"};
  const char init1[] PROGMEM = {"--------------------------------------------\nALTICRAFT FLIGHT DATA RECORDER_\n\nCOPYRIGHT (C) ROBERT HUTTER 2019\n\nVERSION: 1.0\nBUILD DATE: "};
  const char trig1[] PROGMEM = {"STAGING TRIGGER MODE: "};
  const char init_dig[] PROGMEM = {"Initializing digital output pins...\n"};
  const char shutoff1[] PROGMEM = {"--------------------------------------------\nSHUTDOWN COMMAND RECIVED_\nSHUTDOWN TIME: "};
  const char shutoff2[] PROGMEM = {"_\nSETTING RGB LED OFF_\nSHUTTING DOWN_\n"};
  const char init_servo[] PROGMEM = {"Initializing servo motor...\n"};
  const char init3[] PROGMEM = {"\nOPERATION MODE: "};
  const char trig2[] PROGMEM = {"\nTRIGGER VALUE: "};
  
  const char* const messages[] PROGMEM =
  {
    init2, buzzer_init, rgb_init, sd_init, sd_type1, sd_clusters, sd_blockspersecond,
    sd_totalblocks, volume_type, volume_size1, volume_size2, end_line, test, test_err,
    test_ok, sd_init_err, volume_init_err, mk_worksp_err, op_worksp_err , wait, warn,
    launch1, launch2, startdatalog, init1, trig1, init_dig, shutoff1, shutoff2,
    init_servo, init3, trig2
  };
  /* END of messages */
#endif

/* Program variables */
#if defined LAUNCH_LOG_STAGE || defined ONLY_LAUNCH_AND_LOG
  SRL::Buzzer* buzzer;
#endif
#ifdef LAUNCH_LOG_STAGE
  Servo stageservo;
#endif

SRL::rgbled rgb(RED_PIN, GREEN_PIN, BLUE_PIN);
Sd2Card sdcard;
SdVolume sdvolume;
SdFile* logfile;
SdFile sdfilemanager;

bool rw_active = false;
/* END of program variables */

/* Function prototypes */
void writeOut(char const* message);
void logData(char const* message, char const* devicename);
void createNewLogFile(SdFile* logfile);

/* END of function prototypes */

/**
* Execution starts here.
*
*/
void setup()
{
  // Begin initialization. Write out welcome message.
  #ifdef DEBUG
    Serial.begin(BAUD_RATE);
    Serial.flush();
    writeOut((char*)pgm_read_word(&messages[24]));
    writeOut(String(__DATE__).c_str());
    writeOut((char*)pgm_read_word(&messages[30]));
    writeOut(String(OPERATION_MODE).c_str());
    writeOut((char*)pgm_read_word(&messages[11]));
    
    #if defined LAUNCH_LOG_STAGE
      writeOut((char*)pgm_read_word(&messages[25]));
      writeOut(String(TRIGGER_MODE).c_str());
      writeOut((char*)pgm_read_word(&messages[31]));
      #if defined PRESSURE
        writeOut((String(TRIGGER_PRESSURE) + F(" kPa\n")).c_str());
      #else
        #if defined ALTITUDE
          writeOut((String(TRIGGER_ALTITUDE) + F(" m\n")).c_str());
        #endif
      #endif
    #endif

    writeOut((char*)pgm_read_word(&messages[0]));
  #endif
  
  // Initialize sensors
  #ifdef DEBUG
    writeOut((char*)pgm_read_word(&messages[2]));
  #endif
  rgb.setColor(BLUE);
  
  // Initialize components needed for launch
  #if defined LAUNCH_LOG_STAGE || defined ONLY_LAUNCH_AND_LOG
    #ifdef DEBUG
      writeOut((char*)pgm_read_word(&messages[1]));
    #endif
    buzzer = new SRL::Buzzer(BUZZER_PIN);
    buzzer->turnOn();
    delay(100);
    buzzer->turnOff();
    #ifdef DEBUG
      writeOut((char*)pgm_read_word(&messages[26]));
    #endif
    pinMode(BUTTON_PIN, INPUT);
    pinMode(LAUNCH_PIN, OUTPUT);
  #endif

  // Initilize components only needed for staging
  #ifdef LAUNCH_LOG_STAGE
    #ifdef DEBUG
      writeOut((char*)pgm_read_word(&messages[29]));
    #endif
    stageservo.attach(SERVO_PIN);
  #endif

  #ifdef DEBUG
    writeOut((char*)pgm_read_word(&messages[3]));
  #endif

  /* byte sdtest
  * 0000 - four bits
  * 000X - SD card init
  * 00X0 - SD volume init
  * 0X00 - create new dir
  * X000 - open workspace dir
  * 
  * OK only if sdtest == 15
  */
  byte sdtest = sdcard.init(SPI_HALF_SPEED, SD_CHIP_SELECT);

  // Initialize volume if card checks out
  if (sdtest)
  {
    sdtest = (sdtest << 1) | sdvolume.init(&sdcard);

    #ifdef DEBUG
      if(sdtest == 3)
      {
        writeOut((char*)pgm_read_word(&messages[4]));
        writeOut(String(sdcard.type()).c_str());
        writeOut((char*)pgm_read_word(&messages[11]));
        
        writeOut((char*)pgm_read_word(&messages[5]));
        writeOut(String(sdvolume.clusterCount()).c_str());
        writeOut((char*)pgm_read_word(&messages[11]));
  
        writeOut((char*)pgm_read_word(&messages[6]));
        writeOut(String(sdvolume.blocksPerCluster()).c_str());
        writeOut((char*)pgm_read_word(&messages[11]));
  
        writeOut((char*)pgm_read_word(&messages[7]));
        writeOut(String(sdvolume.blocksPerCluster() * sdvolume.clusterCount()).c_str());
        writeOut((char*)pgm_read_word(&messages[11]));
  
        writeOut((char*)pgm_read_word(&messages[8]));
        writeOut(String(sdvolume.fatType()).c_str());
        writeOut((char*)pgm_read_word(&messages[11]));
  
        writeOut((char*)pgm_read_word(&messages[9]));
        writeOut(String(sdvolume.blocksPerCluster() * sdvolume.clusterCount() / 2).c_str());
        writeOut((char*)pgm_read_word(&messages[10]));
      }
    #endif

    // Create a new directory to log data into and open it
    sdtest = (sdtest << 1) | sdfilemanager.openRoot(&sdvolume);

    if(sdtest == 7)
    {
      String dirname = __DATE__;
      while (!sdfilemanager.makeDir(new SdFile(), dirname.c_str()))
      {
        dirname += "a";
      }
  
      sdtest = (sdtest << 1) | sdfilemanager.open(&sdfilemanager, dirname.c_str(), O_RDWR);
      rw_active = true;
    }
  }
  
  // Test all components, report if there is an error.
  bool ok = true;
  
  #ifdef DEBUG
    // Debug the error
    writeOut((char*)pgm_read_word(&messages[12]));

    // Debug sd card
    ok = false;
    switch (sdtest)
    {
      case 0:
        writeOut((char*)pgm_read_word(&messages[15]));
        break;

      case 1:
        writeOut((char*)pgm_read_word(&messages[16]));
        break;

      case 3:
        writeOut((char*)pgm_read_word(&messages[17]));
        break;

      case 7:
        writeOut((char*)pgm_read_word(&messages[18]));
        break;

      case 15:
        ok = true;
        break;

      default:
        break;
    }

    // If not ok, print error to screen
    if (!ok)
    {
      writeOut((char*)pgm_read_word(&messages[13]));
    }
    else
    {
      writeOut((char*)pgm_read_word(&messages[14]));
    }

  #else
    // True if any component fails
    ok = (sdtest == 15);
  #endif

  if (!ok)
  {
    rgb.setColor(WHITE);
      
    // Wait forever (until the user reboots the system)
    for(;;){}
  }
  
  // If launch mode enabled, follow launch prcedure
  #if defined LAUNCH_LOG_STAGE || defined ONLY_LAUNCH_AND_LOG
    #ifdef DEBUG
      writeOut((char*)pgm_read_word(&messages[19]));
    #endif
    rgb.setColor(GREEN);

    bool* cont;
    cont = new bool(true);
    do
    { 
      // Wait for ignition sequence to be started.
      do
      {
        while(!digitalRead(BUTTON_PIN));
        
        for(unsigned int i = 0; i < LAUNCH_ARM_TIME/10; i++)
        {
          if(!digitalRead(BUTTON_PIN))
          {
            *cont = false;
            break;
          }
          delay(10);
        }
      } while(!*cont);
      
      // Give feedback to the user
      rgb.setColor(RED);
      delay(500);
      rgb.setColor(GREEN);
  
      // Arm rocket, turn on launch warnings
      buzzer->turnOn();
  
      for (unsigned int i = 0; i < LAUNCH_COUNTDOWN/10; i++)
      {
        // Listen for cancel
        if (digitalRead(BUTTON_PIN))
        {
          *cont = false;
          rgb.setColor(GREEN);
          buzzer->turnOff();
          break;
        }
  
        // Color warning
        if (i % 50 == 0)
        {
          if (i % 100 == 0)
          {
            rgb.setColor(GREEN);
          }
          else
          {
            rgb.setColor(RED);
          }
        }
        
        delay(10);
      }
    } while (!*cont);
    delete cont;

    // Ignite rocket (first stage)
    digitalWrite(LAUNCH_PIN, HIGH);

    // Buzzer no longer needed, destruct
    buzzer->turnOff();
    buzzer->~Buzzer();
  
    #ifdef DEBUG
      writeOut((char*)pgm_read_word(&messages[20]));
    
      // If all went well, LAUNCH
      writeOut((char*)pgm_read_word(&messages[21]));
      writeOut(String(micros()).c_str());
      writeOut((char*)pgm_read_word(&messages[22]));
    #endif
  #endif

  #ifdef DEBUG
    writeOut((char*)pgm_read_word(&messages[23]));
  #endif
  rgb.setColor(RED);

  logfile = new SdFile();
  createNewLogFile(logfile);
}

/**
* Main loop. Runs repeatadly after the setup is done.
*
*/
void loop()
{
  // Take measurements
  
  // If staging mode enabled, compare readings with trigger
  #if defined LAUNCH_LOG_STAGE
    #if defined PRESSURE
    
    #else
      #if defined ALTITUDE
      
      #endif
    #endif
  #endif
  
  // Save measurements to file
  logData("DATA HERE", "BMP180"); // TEMPORARY
  
  // Create new file every few intervals
  static unsigned long lasttime = micros();

  if (lasttime + NEWFILE_INTERVAL_TIME >= micros())
  {
    // Create a new file
    createNewLogFile(logfile);
  }
  
  // Check to see if it's time to stop the recording
  static long lastpressed = -SHUTDOWN_TIME;
  if(digitalRead(BUTTON_PIN))
  {
    if(lastpressed + SHUTDOWN_TIME >= millis())
    {
      // Shut down
      #ifdef DEBUG
        rw_active = false;
        writeOut((char*)pgm_read_word(&messages[27]));
        writeOut(String(micros()).c_str());
        writeOut((char*)pgm_read_word(&messages[28]));
      #endif
      
      logfile->close();
      sdfilemanager.close();
      
      rgb.turnOff();
      digitalWrite(LAUNCH_PIN, LOW);
      exit(0);
    }
    else if(lastpressed == -SHUTDOWN_TIME)
    {
      lastpressed = millis();
    }
  }
  else
  {
    lastpressed = -SHUTDOWN_TIME;
  }
}

/**
* Log data recorded by instruments.
*
*/
void logData(char const* message, char const* devicename)
{
  writeOut(DATAHEADER1);
  writeOut(String(micros()).c_str());
  writeOut(DATAHEADER2);
  writeOut(devicename);
  writeOut(DATAHEADER3);
  writeOut(message);
  writeOut("\n");
}

/**
* Writes out a message to the SD card, and Serial. (Serial is a mirror)
*
* @param message
*/
void writeOut(char const* message)
{
  Serial.print(message);
  
  // Write to SD card
  if (rw_active)
  {
    logfile->write(message);
  }
}

/**
* Creates a new logfile. 
* 
* @param logfile Reference to SdFile.
*/
void createNewLogFile(SdFile* logfile)
{
  logfile->close();
  bool ok = logfile->createContiguous(sdfilemanager, ("log_" + String(micros()) + ".d").c_str(), 1) == 1;
}