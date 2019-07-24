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
#include <Buzzer.h>
#include <SPI.h>
#include <SD.h>

/* BEGINNING OF PROGRAM PREFERENCES */

#define DEBUG ON                      // Comment this line out if you don't want debug

#define BAUD_RATE 115200              // Change baud rate here
#define ONLY_LAUNCH_AND_LOG ACTIVE            // Change operation mode here

/*  OPERATION MODES: (use double quotes)
*     - "PASSIVE_LOG": Only log flight data. Disables launch and staging features.
*     - "ONLY_LAUNCH_AND_LOG": Enables only launch control and flight data logging.
*     - "LAUNCH_LOG_STAGE": Enables all features including launch, staging control
*         and flight data logging.
*/

#define PRESSURE ACTIVE               // Change trigger mode here
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

namespace std
{
  typedef String string;
}

typedef struct
{
  enum : uint8_t
  {
    red = 2,
    green = 4,
    blue = 5,
    buzzer = 9
  };
} pins;

/* Messages */
#define INIT1 "--------------------------------------------\nALTICRAFT FLIGHT DATA RECORDER_\n\nCOPYRIGHT (C) ROBERT HUTTER 2019\n\nVERSION: 1.0\nBUILD DATE: " + std::string(__DATE__) + "\nOPERATION MODE: " + std::string(OPERATION_MODE) + "\n"
#define TRIG1 "STAGING TRIGGER MODE: " + std::string(TRIGGER_MODE) + "\nTRIGGER VALUE: "
#define INIT2 "--------------------------------------------\nINITIALIZATION PHASE BEGINNING_\n"
#define BUZZER_INIT "Initializing buzzer...\n"
#define RGB_INIT "Initializing RGB LED...\n"
#define SD_INIT "Initializing SD module...\n"
#define SD_TYPE "CARD TYPE: "
#define SD_CLUSTERS "CLUSTERS: "
#define SD_BLOCKSPERCLUSTER "BLOCKS x CLUSTER: "
#define SD_TOTALBLOCKS "TOTAL BLOCKS: "
#define VOLUME_TYPE "VOLUME TYPE: FAT"
#define VOLUME_SIZE1 "VOLUME SIZE: "
#define VOLUME_SIZE2 " kb_\n"
#define END_LINE "_\n"
#define TEST "INITIALIZATION PHASE ENDED_\n--------------------------------------------\nTESTING PHASE BEGINNING_\n"
#define TEST_ERR "--------------------------------------------\nAN ERROR HAS OCCURED DURING TESTING PHASE_\nREVIEW LOG FOR MORE INFORMATION_\nABORTING SETUP_\nSETTING LED TO WHITE_\nRESTART TO TRY AGAIN_\n"
#define TEST_OK "TESTING PHASE ENDED_\nALL SYSTEMS NOMINAL_\n"
#define WAIT "ARMING IGNITION SYSTEM_\n--------------------------------------------\nWAITING FOR LAUNCH SIGNAL_\n"
#define WARN "LAUNCH SIGNAL RECIVED_\n--------------------------------------------\nWARNING! IGNITION IN 10 SECONDS_\nPRESS ANY BUTTON TO CANCEL_\n"
#define LAUNCH1 "--------------------------------------------\nLIFTOFF_\nIGNITION AT: "
#define LAUNCH2 " microseconds_\n"
#define STARTDATALOG "--------------------------------------------\n"
#define DATAHEADER1 "[+"
#define DATAHEADER2 "us/"
#define DATAHEADER3 "]:\t"
/* END of messages */

template <class T>
inline std::string to_string (const T& t);

SRL::rgbled rgb(pins::red, pins::green, pins::blue);
SRL::Buzzer buzzer(pins::buzzer);
Sd2Card sdcard;
SdVolume sdvolume;
SdFile sdfile;

void writeOut(std::string message);
void logData(std::string message, std::string devicename);

/**
* Execution starts here.
*
*/
void setup()
{
  // Begin initialization. Write out welcome message.
  #ifdef DEBUG
    Serial.begin(BAUD_RATE);
    writeOut(INIT1);
    
    #if defined LAUNCH_LOG_STAGE
      #if defined PRESSURE
        writeOut(to_string(TRIGGER_PRESSURE) + " kPa\n");
      #else
        #if defined ALTITUDE
          writeOut(to_string(TRIGGER_ALTITUDE) + " m\n");
        #endif
      #endif
    #endif

    writeOut(INIT2);
  #endif
  
  // Initialize sensors
  #ifdef DEBUG
    writeOut(RGB_INIT);
  #endif
  rgb.setColor(BLUE);
  
  // Initialize components needed for launch
  #if defined LAUNCH_LOG_STAGE || defined ONLY_LAUNCH_AND_LOG
    #ifdef DEBUG
      writeOut(BUZZER_INIT);
    #endif
    buzzer.turnOn();
    delay(100);
    buzzer.turnOff();
  #endif

  #ifdef DEBUG
    writeOut(SD_INIT);
  #endif
  byte sdtest = sdcard.init(SPI_HALF_SPEED, SD_CHIP_SELECT);
  #ifdef DEBUG
    writeOut(SD_TYPE);
    writeOut(sdcard.type());
    writeOut(END_LINE);
  #endif

  // Initialize volume if card checks out
  if (sdtest)
  {
    sdtest = (sdtest << 1) | sdvolume.init(sdcard);

    #ifdef DEBUG
      if(sdtest == 3)
      {
        writeOut(SD_CLUSTERS);
        writeOut(sdvolume.clusterCount());
        writeOut(END_LINE);
  
        writeOut(SD_BLOCKSPERCLUSTER);
        writeOut(sdvolume.blocksPerCluster());
        writeOut(END_LINE);
  
        writeOut(SD_TOTALBLOCKS);
        writeOut(sdvolume.blocksPerCluster() * sdvolume.clusterCount());
        writeOut(END_LINE);
  
        writeOut(VOLUME_TYPE);
        writeOut(sdvolume.fatType());
        writeOut(END_LINE);
  
        writeOut(VOLUME_SIZE1);
        writeOut(sdvolume.blocksPerCluster() * sdvolume.clusterCount() / 2);
        writeOut(VOLUME_SIZE2);
      }
    #endif
  }
  
  // Test all components, report if there is an error.
  #ifdef DEBUG
    writeOut(TEST);
  #endif
  
  if (false) //TEMPORARY, true if an error occures
  {
    #ifdef DEBUG
      writeOut(TEST_ERR);
    #endif
    rgb.setColor(WHITE);
    
    // Wait forever (until the user reboots the system)
    for(;;){}
  }

  #ifdef DEBUG
    writeOut(TEST_OK);
  #endif
  
  // If launch mode enabled, follow launch prcedure
  #if defined LAUNCH_LOG_STAGE || defined ONLY_LAUNCH_AND_LOG
    // Wait for ignition sequence to be started.
    #ifdef DEBUG
      writeOut(WAIT);
    #endif
    rgb.setColor(GREEN);
    
    for (int i = 0; i < 10; i++)
    {
      delay(1000);
    }
  
  
    // Arm rocket, turn on launch warnings
    #ifdef DEBUG
      writeOut(WARN);
    
      // If all went well, LAUNCH
      writeOut(LAUNCH1);
      writeOut(String(micros()));
      writeOut(LAUNCH2);
    #endif
  #endif

  #ifdef DEBUG
    writeOut(STARTDATALOG);
  #endif
  rgb.setColor(RED);

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
  
  
  // Check to see if it's time to stop the recording
  
}

/**
* Log data recorded by instruments.
*
*/
void logData(std::string message, std::string devicename)
{
  writeOut(DATAHEADER1);
  writeOut(String(micros()));
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
void writeOut(std::string message)
{
  Serial.print(message);
  
  // Write to SD card
  
}
