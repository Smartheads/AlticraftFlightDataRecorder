/*
* MIT License
*
* Copyright (c) 2020 Robert Hutter
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
#include <I2C.h>
#include <avr/pgmspace.h>

/* BEGINNING OF PROGRAM PREFERENCES */
// Debug
#define DEBUG ACTIVE                      // Comment this line out if you don't want debug
#define BAUD_RATE 115200              // Change baud rate here

// Measurement preferences
#define BASE_PRESSURE 100325        // Base pressure to use for altitude calculation
#define MEASUREMENT_RATE 20         // Measurement rate: set the timeout between measurements (Hz).
#define ACCEL_SENSITIVITY 0             // Change the accelerometer sensitivity
#define GYRO_SENSITIVITY 0              // Change the gyroscope sensitivity
#define PRESSURE_OVERSAMPLING 3      // Change the pressure oversampling setting
#define TEMPERATURE_OVERSAMPLING 1   // Change the temperature oversampling setting

// Calibration data
#define ACCEL_X_OFFSET 144 
#define ACCEL_Y_OFFSET 885 
#define ACCEL_Z_OFFSET -345              
#define GYRO_X_OFFSET -402               
#define GYRO_Y_OFFSET 109            
#define GYRO_Z_OFFSET -29              
#define BMP280_DIG_T1 27317
#define BMP280_DIG_T2 25771
#define BMP280_DIG_T3 50
#define BMP280_DIG_P1 38529
#define BMP280_DIG_P2 -10512
#define BMP280_DIG_P3 3024
#define BMP280_DIG_P4 6002
#define BMP280_DIG_P5 -132
#define BMP280_DIG_P6 -7
#define BMP280_DIG_P7 15500
#define BMP280_DIG_P8 -14600
#define BMP280_DIG_P9 6000

// Timing
#define LAUNCH_ARM_TIME 3000          // Change the launch arm time (ms)
#define LAUNCH_TIMEOUT 2000           // Time between launch sequence activated and countdown begins (ms)
#define LAUNCH_COUNTDOWN 10000          // Change the launch countdown time (ms)
#define NEWFILE_INTERVAL_TIME 3000000   // Change the interval between saving into a new file (us)
#define SHUTDOWN_TIME 3000            // Change the shutdown timer (ms)

// Advanced
#define BMP280_POWER_MODE 3          // BMP280 power mode (3 = normal mode)
#define EXPECTED_FILE_SIZE 17408    // Expected size of log files (b)

/* SELECT OPERATIONAL MODE */
#define LAUNCH_LOG_STAGE ACTIVE            // Change operation mode here

/*  OPERATION MODES:
*     - PASSIVE_LOG: Only log flight data. Disables launch and staging features.
*     - ONLY_LAUNCH_AND_LOG: Enables only launch control and flight data logging.
*     - LAUNCH_LOG_STAGE: Enables all features including launch, staging control
*         and flight data logging.
*/

/* SELECT STAGE TRIGGER IF REQUIRED */
#define ALTITUDE ACTIVE               // Change trigger mode here
#define TRIGGER_ALTITUDE 200          // Change trigger altitude here
#define TRIGGER_PRESSURE 100.0        // Change trigger pressure here

/*  STAGE TRIGGER MODES:
*     - ALTITUDE: Activates staging at a specific altitude. (m)
*     - PRESSURE: Activates staging at a specific pressure. (kPa)
*     - TEMPERATURE: Activates staging at a specific temperature. (deg C)
*     - APOAPSIS: Activates staging when altitude is at a constant drop.
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
#if ACCEL_SENSITIVITY == 0
  #define ACCEL_SCALE_FACTOR 16384.0f
#elif ACCEL_SENSITIVITY == 1
  #define ACCEL_SCALE_FACTOR 8192.0f
#elif ACCEL_SENSITIVITY == 2
  #define ACCEL_SCALE_FACTOR 4096.0f
#elif ACCEL_SENSITIVITY == 3
  #define ACCEL_SCALE_FACTOR 2048.0f
#endif

#if GYRO_SENSITIVITY == 0
  #define GYRO_SCALE_FACTOR 131.0f
#elif GYRO_SENSITIVITY = 1
  #define GYRO_SCALE_FACTOR 65.5f
#elif GYRO_SENSITIVITY = 2
  #define GYRO_SCALE_FACTOR 32.8f
#elif GYRO_SENSITIVITY = 3
  #define GYRO_SCALE_FACTOR 16.4f
#endif

#define MEASUREMENT_INTERVAL (1000 / MEASUREMENT_RATE)

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

/* Register addresses */
#define MPU9250_ADDRESS 0x68          // I2C address of MPU9250
#define BMP280_ADDRESS 0x76           // I2C address of BMP280

#define BMP280_CTRL_MEAS 0xF4   // Oversampling, power mode
#define BMP280_PRESS 0xF7       // Start of burst read
#define BMP280_CHIP_ID 0xD0

#define MPU9250_GYRO_CONFIG 0x1B
#define MPU9250_ACCEL_CONFIG 0x1C
#define MPU9250_WHO_AM_I 0x75
#define MPU9250_ACCEL_DATA 0x3B
/* END of register addresses */

#define BMP280_CHIP_ID_VALUE 0x58
#define MPU9250_WHO_AM_I_VALUE 0x71
#define WORKSPACE_DIR_NAME "FLT"
#define LOG_FILE_HEADER F("timestamp,pressure,altitude,temp,accelX,accelY,accelZ,GyroX,GyroY,GyroZ\n")

#ifdef DEBUG
  char outBuffer[135];

  /* All other messages */
  const char init2[] PROGMEM = {"INITIALIZATION PHASE BEGINNING_\n"};
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
  const char test_err1[] PROGMEM = {"AN ERROR HAS OCCURED DURING TESTING PHASE_\nREVIEW LOG FOR MORE INFORMATION_"};
  const char test_ok[] PROGMEM = {"TESTING PHASE ENDED_\nALL SYSTEMS NOMINAL_\n"}; // 14
  const char sd_init_err[] PROGMEM = {"Error initializing SD card...\n"};
  const char volume_init_err[] PROGMEM = {"Error initializing SD volume...\n"};
  const char mk_worksp_err[] PROGMEM = {"Error creating workspace...\n"};
  const char op_worksp_err[] PROGMEM = {"Error opening workspace...\n"}; // 18
  const char wait[] PROGMEM = {"ARMING IGNITION SYSTEM_\n--------------------------------------------\nWAITING FOR LAUNCH SIGNAL_\n"};
  const char warn[] PROGMEM = {"LAUNCH SIGNAL RECIVED_\n--------------------------------------------\nWARNING! IGNITION IN 10 SECONDS_\nPRESS ANY BUTTON TO CANCEL_\n"};
  const char launch1[] PROGMEM = {"--------------------------------------------\nLIFTOFF_\nIGNITION AT: "};
  const char launch2[] PROGMEM = {" microseconds_\n"};
  const char startdatalog[] PROGMEM = {"--------------------------------------------\n"};
  const char init1[] PROGMEM = {"ALTICRAFT FLIGHT DATA RECORDER_\n\nCOPYRIGHT (C) ROBERT HUTTER 2019\n\nVERSION: 1.0\nBUILD DATE: "}; // 24
  const char trig1[] PROGMEM = {"STAGING TRIGGER MODE: "};
  const char init_dig[] PROGMEM = {"Initializing digital output pins...\n"};
  const char shutoff1[] PROGMEM = {"SHUTDOWN COMMAND RECIVED_\nSHUTDOWN TIME: "}; // 27
  const char shutoff2[] PROGMEM = {"_\nSETTING RGB LED OFF_\nSHUTTING DOWN_\n"};
  const char init_servo[] PROGMEM = {"Initializing servo motor...\n"};
  const char init3[] PROGMEM = {"\nOPERATION MODE: "};
  const char trig2[] PROGMEM = {"\nTRIGGER VALUE: "};
  const char mpu_init[] PROGMEM = {"Initializing MPU9250...\n"};
  const char bmp_init[] PROGMEM = {"Initializing BMP280...\n"};
  const char test_err2[] PROGMEM = {"\nABORTING SETUP_\nSETTING LED TO WHITE_\nRESTART TO TRY AGAIN_\n"};
  const char divider[] PROGMEM = {"--------------------------------------------\n"}; // 35
  const char bmp_test_err[] PROGMEM = {"Error reading BMP280 chip id...\n"};
  const char sd_err_o_ws[] PROGMEM = {"Error opening workspace..."};
  const char mpu_test_err[] PROGMEM = {"Error reading MPU9250 device id...\n"}; // 38
  const char user_abort[] PROGMEM = {"USER ABORTED LAUNCH_\nSETTING LED TO WHITE_\nRESTART TO TRY AGAIN_\n"};
  
  const char* const messages[] PROGMEM =
  {
    init2, buzzer_init, rgb_init, sd_init, sd_type1, sd_clusters, sd_blockspersecond,
    sd_totalblocks, volume_type, volume_size1, volume_size2, end_line, test, test_err1,
    test_ok, sd_init_err, volume_init_err, mk_worksp_err, op_worksp_err , wait, warn,
    launch1, launch2, startdatalog, init1, trig1, init_dig, shutoff1, shutoff2,
    init_servo, init3, trig2, mpu_init, bmp_init, test_err2, divider, bmp_test_err,
    sd_err_o_ws, mpu_test_err, user_abort
  };
  /* END of messages */
#endif

/* Program variables */
#if defined(LAUNCH_LOG_STAGE) || defined(ONLY_LAUNCH_AND_LOG)
  #include <Buzzer.h>
  SRL::Buzzer* buzzer;
#endif
#ifdef LAUNCH_LOG_STAGE
  #include <Servo.h>
  Servo stageservo;
#endif

SRL::I2CDevice bmp(BMP280_ADDRESS);
SRL::I2CDevice mpu(MPU9250_ADDRESS);
SRL::rgbled rgb(RED_PIN, GREEN_PIN, BLUE_PIN);
Sd2Card sdcard;
SdVolume sdvolume;
SdFile* logfile;
SdFile debugFile;
SdFile sdfilemanager;
/* END of program variables */

/* Function prototypes */
void logData(int32_t, float, int32_t, float, float, float, float, float, float);
void createNewLogFile(SdFile*);

#ifdef DEBUG
  void writeOutDebugMessage(int);
  void writeOutDebugMessage(String);
#endif

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
    writeOutDebugMessage(35);
    writeOutDebugMessage(24);
    writeOutDebugMessage(String(__DATE__));
    writeOutDebugMessage(30);
    writeOutDebugMessage(String(OPERATION_MODE));
    writeOutDebugMessage(11);
    
    #ifdef LAUNCH_LOG_STAGE
      writeOutDebugMessage(25);
      writeOutDebugMessage(String(TRIGGER_MODE));
      writeOutDebugMessage(30);
      #ifdef PRESSURE
        writeOutDebugMessage(String(TRIGGER_PRESSURE) + F(" kPa\n"));
      #else
        #ifdef ALTITUDE
          writeOutDebugMessage(String(TRIGGER_ALTITUDE) + F(" m\n"));
        #endif
      #endif
    #endif

    writeOutDebugMessage(35);
    writeOutDebugMessage(0);
    Serial.flush();
  #endif
  
  // Initialize sensors
  #ifdef DEBUG
    writeOutDebugMessage(2);
  #endif
  rgb.setColor(BLUE);

  // Initialize MPU9250
  #ifdef DEBUG
    writeOutDebugMessage(32);
  #endif
  { // Local scope for gc and ac
    uint8_t gc = 0;
    mpu.readBytes(MPU9250_GYRO_CONFIG, &gc, 1);
    gc = ((uint8_t)0xE7) & gc; // 11100111 mask
    gc = gc | ((uint8_t)GYRO_SENSITIVITY);
    mpu.writeByte(MPU9250_GYRO_CONFIG, gc);
    
    uint8_t ac = 0;
    mpu.readBytes(MPU9250_ACCEL_CONFIG, &ac, 1);
    ac = ((uint8_t)0xE7) & ac; // 11100111 mask
    ac = ac | ((uint8_t)ACCEL_SENSITIVITY);
    mpu.writeByte(MPU9250_ACCEL_CONFIG, ac);
  }

  // Initialize BMP280
  #ifdef DEBUG
    writeOutDebugMessage(33);
  #endif
  { // Local scope for bmpSetting
    uint8_t bmpSetting = (((uint8_t)TEMPERATURE_OVERSAMPLING) << 5) | (((uint8_t)PRESSURE_OVERSAMPLING) << 2) | ((uint8_t)BMP280_POWER_MODE);
    bmp.writeByte(BMP280_CTRL_MEAS, bmpSetting);
  }
  
  // Initialize components needed for launch
  #if defined LAUNCH_LOG_STAGE || defined ONLY_LAUNCH_AND_LOG
    #ifdef DEBUG
      writeOutDebugMessage(1);
    #endif
    buzzer = new SRL::Buzzer(BUZZER_PIN);
    buzzer->turnOn();
    delay(100);
    buzzer->turnOff();
    #ifdef DEBUG
      writeOutDebugMessage(26);
    #endif
    pinMode(BUTTON_PIN, INPUT);
    pinMode(LAUNCH_PIN, OUTPUT);
  #endif

  // Initilize components only needed for staging
  #ifdef LAUNCH_LOG_STAGE
    #ifdef DEBUG
      writeOutDebugMessage(29);
    #endif
    stageservo.attach(SERVO_PIN);
  #endif

  #ifdef DEBUG
    writeOutDebugMessage(3);
  #endif

  /* uint8_t sdtest
  * 0000 - last four LSB
  * 000X - SD card init
  * 00X0 - SD volume init
  * 0X00 - create new dir
  * X000 - open workspace dir
  * 
  * OK only if sdtest == 0x0F (1111)
  */
  uint8_t sdtest = sdcard.init(SPI_HALF_SPEED, SD_CHIP_SELECT);

  // Initialize volume if card checks out
  if (sdtest == 0x01)
  {
    sdtest = (((uint8_t)sdvolume.init(&sdcard)) << 1) | sdtest;

    #ifdef DEBUG
      if(sdtest == 0x03) // Print information on sdvolume
      {
        writeOutDebugMessage(4);
        writeOutDebugMessage(String(sdcard.type()));
        writeOutDebugMessage(11);
        
        writeOutDebugMessage(5);
        writeOutDebugMessage(String(sdvolume.clusterCount()));
        writeOutDebugMessage(11);
  
        writeOutDebugMessage(6);
        writeOutDebugMessage(String(sdvolume.blocksPerCluster()));
        writeOutDebugMessage(11);
  
        writeOutDebugMessage(7);
        writeOutDebugMessage(String(sdvolume.blocksPerCluster() * sdvolume.clusterCount()));
        writeOutDebugMessage(11);
  
        writeOutDebugMessage(8);
        writeOutDebugMessage(String(sdvolume.fatType()));
        writeOutDebugMessage(11);
  
        writeOutDebugMessage(9);
        writeOutDebugMessage(String(sdvolume.blocksPerCluster() * sdvolume.clusterCount() / 2));
        writeOutDebugMessage(10);
      }
    #endif

    // Create a new directory to log data into and open it
    sdtest = (((uint8_t)sdfilemanager.openRoot(&sdvolume)) << 2) | sdtest;

    if(sdtest == 0x07) // 0111
    {
      uint8_t num = 0;
      const String dir = WORKSPACE_DIR_NAME;
      String dirname = dir + String(num);
      bool success = false;
      while (!success)
      {
        SdFile sdf;
        success = sdf.makeDir(&sdfilemanager, dirname.c_str());
        if (!success)
        {
          dirname = dir + String(++num); // Issue somewhere here
        }
      }

      SdFile sdf;
      sdtest = ((uint8_t)(sdf.open(&sdfilemanager, dirname.c_str(), O_READ)) << 3) | sdtest;
      sdfilemanager = sdf;
    }
  }
  
  /* Testing phase begins here */

  // Debug potenital SD card error (testing already happened)
  #ifdef DEBUG
    writeOutDebugMessage(12); // Testing phase beginning
    Serial.flush();

    // Debug sd card
    switch (sdtest)
    {
      case 0x00:
        writeOutDebugMessage(15);
        break;

      case 0x01:
        writeOutDebugMessage(16);
        break;

      case 0x03:
        writeOutDebugMessage(17);
        break;

      case 0x07:
        writeOutDebugMessage(37);
      break;

      case 0x0E:
        writeOutDebugMessage(18);
        break;
        
      default:
        break;
    }
  #endif

  // Test MPU9250, read device ID
  uint8_t mpuChipId = 0;
  mpu.readBytes(MPU9250_WHO_AM_I, &mpuChipId, 1);

  #ifdef DEBUG
    if (mpuChipId != MPU9250_WHO_AM_I_VALUE)
    {
      writeOutDebugMessage(38); // Error with MPU9250
    }
  #endif

  // Test BMP280, read device ID
  uint8_t bmpChipId = 0;
  bmp.readBytes(BMP280_CHIP_ID, &bmpChipId, 1);

  #ifdef DEBUG
    if (bmpChipId != BMP280_CHIP_ID_VALUE)
    {
      writeOutDebugMessage(36); // Error with BMP280
    }
  #endif
  
  // Check if any component failed
  if (sdtest != 0x0F || bmpChipId != BMP280_CHIP_ID_VALUE || mpuChipId != MPU9250_WHO_AM_I_VALUE)
  {
    #ifdef DEBUG
      writeOutDebugMessage(35); // Divider
      writeOutDebugMessage(13); // Testing has failed
      writeOutDebugMessage(34);
    #endif 
    
    rgb.setColor(WHITE);
      
    // Wait forever (until the user reboots the system)
    sdfilemanager.close();
    for(;;){}
  }

  #ifdef DEBUG
    writeOutDebugMessage(14);
  #endif
  
  // If launch mode enabled, follow launch prcedure
  #if defined(LAUNCH_LOG_STAGE) || defined(ONLY_LAUNCH_AND_LOG)
    #ifdef DEBUG
      writeOutDebugMessage(19);
    #endif
    rgb.setColor(GREEN);

    bool cont = true;
    
    // Wait for ignition sequence to be started.
    do
    {
      while(digitalRead(BUTTON_PIN) == LOW);
      cont = true;
      
      for(uint16_t i = 0; i < LAUNCH_ARM_TIME/10; i++)
      {
        if(digitalRead(BUTTON_PIN) == LOW)
        {
          cont = false;
          break;
        }
        delay(10);
      }
    } while(!cont);
    
    // Give feedback to the user (also gives time to release button)
    rgb.setColor(RED);
    delay(LAUNCH_TIMEOUT);
    rgb.setColor(GREEN);

    // Arm rocket, turn on launch warnings
    buzzer->turnOn();

    #ifdef DEBUG
      writeOutDebugMessage(20); // Launch signal recieved
    #endif

    // Continue with launch sequence
    for (uint16_t i = 0; i < LAUNCH_COUNTDOWN/10; i++)
    {
      // Listen for abort
      if (digitalRead(BUTTON_PIN) == HIGH)
      {
        cont = false;
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

    if (!cont) // Check if user aborted
    {
        #ifdef DEBUG
          writeOutDebugMessage(35); // Divider
          writeOutDebugMessage(39); // User abort
        #endif

        rgb.setColor(WHITE);
        
        sdfilemanager.close();
        for(;;) {} // Wait until reset
    }

    // Ignite rocket (first stage)
    digitalWrite(LAUNCH_PIN, HIGH);

    // Buzzer no longer needed, destruct
    buzzer->turnOff();
    delete buzzer;

    #ifdef DEBUG
    debugFile.createContiguous(sdfilemanager, "debug.txt", 1);
    #endif
  
    #ifdef DEBUG
      // If all went well, LAUNCH
      writeOutDebugMessage(21);
      writeOutDebugMessage(String(micros()));
      writeOutDebugMessage(22);
    #endif
  #endif

  #ifdef DEBUG
    writeOutDebugMessage(23);
  #endif
  rgb.setColor(RED);

  // Create log and debug files
  logfile = new SdFile();
  createNewLogFile(logfile);
  logfile->write(String(LOG_FILE_HEADER).c_str());
  logfile->sync();
}

/**
* Main loop. Runs repeatadly after the setup is done.
*
*/
void loop()
{
  // Take measurements
  int32_t pressure, temperature;
  float ax, ay, az, gx, gy, gz, altitude;
  { // BMP280
    uint8_t bmpBuffer[6] = { 0 };
    bmp.readBytes(BMP280_PRESS, bmpBuffer, 6);
    
    int32_t adc_P = (int32_t) ((((uint32_t) (bmpBuffer[0])) << 12) | (((uint32_t) (bmpBuffer[1])) << 4) | ((uint32_t) bmpBuffer[2] >> 4));
    int32_t adc_T = (int32_t) ((((int32_t) (bmpBuffer[3])) << 12) | (((int32_t) (bmpBuffer[4])) << 4) | (((int32_t) (bmpBuffer[5])) >> 4));
  
    {
      // Calculate compensated temperature. From BMP280_driver @BoschSensortech
      int32_t var1, var2;
      var1 = ((((adc_T / 8) - ((int32_t) BMP280_DIG_T1 << 1))) * ((int32_t) BMP280_DIG_T2)) / 2048;
      var2 = (((((adc_T / 16) - ((int32_t) BMP280_DIG_T1)) * ((adc_T / 16) - ((int32_t) BMP280_DIG_T1))) / 4096) * ((int32_t) BMP280_DIG_T3)) / 16384;
      int32_t t_fine = var1 + var2;
      temperature = (t_fine * 5 + 128) / 256;
      
      // Calculate compensated pressure. From BMP280_driver @BoschSensortech
      var1 = (((int32_t) t_fine) / 2) - (int32_t) 64000;
      var2 = (((var1 / 4) * (var1 / 4)) / 2048) * ((int32_t) BMP280_DIG_P6);
      var2 = var2 + ((var1 * ((int32_t) BMP280_DIG_P5)) * 2);
      var2 = (var2 / 4) + (((int32_t) BMP280_DIG_P4) * 65536);
      var1 =
          (((BMP280_DIG_P3 * (((var1 / 4) * (var1 / 4)) / 8192)) / 8) +
           ((((int32_t) BMP280_DIG_P2) * var1) / 2)) / 262144;
      var1 = ((((32768 + var1)) * ((int32_t) BMP280_DIG_P1)) / 32768);
      pressure = (uint32_t)(((int32_t)(1048576 - adc_P) - (var2 / 4096)) * 3125);

      if (var1 != 0)
      {
          /* Check for overflows against UINT32_MAX/2; if pres is left-shifted by 1 */
          if (pressure < 0x80000000)
          {
              pressure = (pressure << 1) / ((uint32_t) var1);
          }
          else
          {
              pressure = (pressure / (uint32_t) var1) * 2;
          }
          var1 = (((int32_t) BMP280_DIG_P9) * ((int32_t) (((pressure / 8) * (pressure / 8)) / 8192))) /
                 4096;
          var2 = (((int32_t) (pressure / 4)) * ((int32_t) BMP280_DIG_P8)) / 8192;
          pressure = (uint32_t) ((int32_t) pressure + ((var1 + var2 + BMP280_DIG_P7) / 16));
      }
      else
      {
          pressure = 0;
      }
    }
  }

  altitude = ((float) 44330.0 * (1 - pow(((double)pressure) / ((double)BASE_PRESSURE), 1 / 5.255)));

  { // MPU9250
    uint8_t mpuBuffer[14] = { 0 };
    mpu.readBytes(MPU9250_ACCEL_DATA, mpuBuffer, 14);
    
    int16_t rax, ray, raz, rgx, rgy, rgz;
    rax = (int16_t) (((int16_t)mpuBuffer[0] << 8) | ((int16_t)mpuBuffer[1]));
    ray = (int16_t) (((int16_t)mpuBuffer[2] << 8) | ((int16_t)mpuBuffer[3]));
    raz = (int16_t) (((int16_t)mpuBuffer[4] << 8) | ((int16_t)mpuBuffer[5]));
    rgx = (int16_t) (((int16_t)mpuBuffer[8] << 8) | ((int16_t)mpuBuffer[9]));
    rgy = (int16_t) (((int16_t)mpuBuffer[10] << 8) | ((int16_t)mpuBuffer[11]));
    rgz = (int16_t) (((int16_t)mpuBuffer[12] << 8) | ((int16_t)mpuBuffer[13]));

    ax = ((float)(rax - ((int16_t)ACCEL_X_OFFSET))) / ACCEL_SCALE_FACTOR;
    ay = ((float)(ray - ((int16_t)ACCEL_Y_OFFSET))) / ACCEL_SCALE_FACTOR;
    az = ((float)(raz - ((int16_t)ACCEL_Z_OFFSET))) / ACCEL_SCALE_FACTOR;
    gx = ((float)(rgx - ((int16_t)GYRO_X_OFFSET))) / GYRO_SCALE_FACTOR;
    gy = ((float)(rgy - ((int16_t)GYRO_Y_OFFSET))) / GYRO_SCALE_FACTOR;
    gz = ((float)(rgz - ((int16_t)GYRO_Z_OFFSET))) / GYRO_SCALE_FACTOR;
  }
  
  // If staging mode enabled, compare readings with trigger
  #ifdef LAUNCH_LOG_STAGE
    #ifdef PRESSURE
      //static int32_t pBuffer[10] ( 0 };

      
      
    #elif defined(ALTITUDE)
      
    #elif defined(APOAPSIS)

    #endif
  #endif
  
  // Save measurements to file
  //static unsigned long lastlog = millis();

  //if (lastlog + MEASUREMENT_INTERVAL <= millis())
  //{
    logData(pressure, altitude, temperature, ax, ay, az, gx, gy, gz);
  //  lastlog = millis();
  //}
  
  // Create new file every few intervals
  static unsigned long lasttime = micros();

  if (lasttime + NEWFILE_INTERVAL_TIME <= micros())
  {
    // Create a new file
    createNewLogFile(logfile);
    lasttime = micros();
  }
  
  // Check to see if it's time to stop the recording
  static long lastpressed = millis;
  if(digitalRead(BUTTON_PIN))
  {
    if(lastpressed + SHUTDOWN_TIME <= millis())
    {
      // Shut down
      #ifdef DEBUG
        writeOutDebugMessage(35);
        writeOutDebugMessage(27);
        writeOutDebugMessage(String(micros()));
        writeOutDebugMessage(28);
      #endif
      
      logfile->close();
      debugFile.close();
      sdfilemanager.close();
      
      rgb.turnOff();
      digitalWrite(LAUNCH_PIN, LOW);
      exit(0);
    }
    else
    {
      lastpressed = millis();
    }
  }
}

/**
* Log data recorded by instruments.
*
*/
void logData(int32_t p, float a, int32_t t, float ax, float ay, float az, float gx, float gy, float gz)
{
  logfile->write(String(micros(), DEC).c_str());
  logfile->write(',');
  logfile->write(String(p, DEC).c_str());
  logfile->write(',');
  logfile->write(String(a, DEC).c_str());
  logfile->write(',');
  logfile->write(String(t, DEC).c_str());
  logfile->write(',');
  logfile->write(String(ax, DEC).c_str());
  logfile->write(',');
  logfile->write(String(ay, DEC).c_str());
  logfile->write(',');
  logfile->write(String(az, DEC).c_str());
  logfile->write(',');
  logfile->write(String(gx, DEC).c_str());
  logfile->write(',');
  logfile->write(String(gy, DEC).c_str());
  logfile->write(',');
  logfile->write(String(gz, DEC).c_str());
  logfile->write('\n');
  logfile->sync();
}

/**
* Creates a new logfile. 
* 
* @param logfile Reference to SdFile.
*/
void createNewLogFile(SdFile* logfile)
{
  static uint16_t fileId = 0;
  logfile->close();
  logfile->createContiguous(sdfilemanager, ("log_" + String(fileId++) + ".csv").c_str(), EXPECTED_FILE_SIZE);
}

#ifdef DEBUG
  /**
   * Calls writeOut function with debug message indexed in arg.
   * 
   * @param i Index of debug message to print.
   */
  void writeOutDebugMessage(int i)
  {
    strcpy_P(outBuffer, (char*) pgm_read_word(&messages[i]));
    debugFile.write(outBuffer);
    Serial.print(outBuffer);
  }

  /**
   * Overloaded function of writeOut with
   * @param message String object
   */
  void writeOutDebugMessage(String message)
  {
    debugFile.write(message.c_str());
    Serial.print(message);
  }
#endif