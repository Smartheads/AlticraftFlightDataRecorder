/*
* MIT License
*
* Copyright (c) 2021 Robert Hutter
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
* AlticraftFDR.h - Header file of AlticraftFDR.cpp.
*
*/

#ifndef __ALTICRAFTFDR_H__
#define __ALTICRAFTFDR_H__

#ifndef Arduino_h
    #include <VirtualArduino.h>
    extern HardwareSerial Serial;
#endif

#include <RGBLED.h>
#include <SPI.h>
#include <SD.h>
#include <I2C.h>
#include <avr/pgmspace.h>

/* BEGINNING OF PROGRAM PREFERENCES */
// Debug
#define DEBUG ACTIVE                      // Comment this line out if you don't want debug
#define BAUD_RATE 9600              // Change baud rate here

// Measurement preferences
#define BASE_PRESSURE 100565        // Base pressure to use for altitude calculation
//#define MEASUREMENT_RATE 1000         // Measurement rate: samples in 1 second (Hz). Comment this line out to disable measurement limiting.
#define ACCEL_SENSITIVITY 0             // Change the accelerometer sensitivity
#define GYRO_SENSITIVITY 0              // Change the gyroscope sensitivity
#define PRESSURE_OVERSAMPLING 3      // Change the pressure oversampling setting
#define TEMPERATURE_OVERSAMPLING 1   // Change the temperature oversampling setting

// Staging servo preferences
#define SERVO_START_ANGLE 0           // Starting servo angle (0-180 deg)
#define SERVO_STAGE_ANGLE 180          // Servo angle at staging (0-180 deg)

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
#define NEWFILE_INTERVAL_TIME 3000   // Change the interval between saving into a new file (ms)
#define SHUTDOWN_TIME 3000            // Change the shutdown timer (ms)
#define STAGE_TRIGGER_TIME 1000       // Amount of time, that the staging condition must stay valid before actual staging operation. (ms)
#define TOUCHDOWN_DETECTION_TIME 10000   // Amount of time the FDR must remain still until touchdown detection activates. (ms)
#define TOUCHDOWN_DETECTION_FROM 20000   // Amount of time, after which touchdown detection is armed. (ms)

// Advanced
#define BMP280_POWER_MODE 3          // BMP280 power mode (3 = normal mode)

/* SELECT OPERATIONAL MODE */
#define LAUNCH_LOG_STAGE ACTIVE            // Change operation mode here
#define TOUCHDOWN_DETECTION ACTIVE          // Comment this line out to disable touchdown detection
#define SAR_HELPER ACTIVE                 // Comment this line out to disable search and rescue helper (buzzer upon touchdown).
#define LAUNCH_STABILITY_ABORT ACTIVE     // Comment this line out to disable the launch stability abort feature

/*  OPERATION MODES:
*     - PASSIVE_LOG: Only log flight data. Disables launch and staging features.
*     - ONLY_LAUNCH_AND_LOG: Enables only launch control and flight data logging.
*     - LAUNCH_LOG_STAGE: Enables all features including launch, staging control
*         and flight data logging.
*/

/* SELECT STAGE TRIGGER IF REQUIRED */
#define ALTITUDE ACTIVE               // Change trigger mode here
#define TRIGGER_ALTITUDE -200          // Change trigger altitude here (m)
#define TRIGGER_TEMPERATURE_MIN 2000   // Change trigger min temperature here (C * 100)
#define TRIGGER_TEMPERATURE_MAX 2800   // Change trigger max temperature here (C * 100)
#define TRIGGER_PRESSURE 101565        // Change trigger pressure here (Pa)
#define BAD_MEAS_ABORT_THRESHOLD 100    // Change bad measurement abort threshold value (samples)
#define BAD_WRITE_ABORT_THRESHOLD 100   // Change bad write to sd abort threshold value (occurances)
#define LAUNCH_ABORT_SENSITIVITY 0.2f // Change launch abort sensitivity threshold value (g)

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

#ifdef MEASUREMENT_RATE
#define MEASUREMENT_INTERVAL (1000 / MEASUREMENT_RATE)
#endif

#ifdef PASSIVE_LOG
#define OPERATION_MODE "PASSIVE_LOG"
#endif

#ifdef ONLY_LAUNCH_AND_LOG
#define OPERATION_MODE "ONLY_LAUNCH_AND_LOG"
#endif

#ifdef LAUNCH_LOG_STAGE
#define OPERATION_MODE "LAUNCH_LOG_STAGE"
#endif

#ifdef ALTITUDE
#define TRIGGER_MODE "ALTITUDE"
#endif

#ifdef PRESSURE
#define TRIGGER_MODE "PRESSURE"
#endif

#ifdef TEMPERATURE
#define TRIGGER_MODE "TEMPERATURE"
#endif

#ifdef APOAPSIS
#define TRIGGER_MODE "APOAPSIS"
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
const char init2[] PROGMEM = { "INITIALIZATION PHASE BEGINNING_\n" };
const char buzzer_init[] PROGMEM = { "Initializing buzzer...\n" };
const char rgb_init[] PROGMEM = { "Initializing RGB LED...\n" };
const char sd_init[] PROGMEM = { "Initializing SD module...\n" };
const char sd_type1[] PROGMEM = { "CARD TYPE: " };
const char sd_clusters[] PROGMEM = { "CLUSTERS: " };
const char sd_blockspersecond[] PROGMEM = { "BLOCKS x CLUSTER: " };
const char sd_totalblocks[] PROGMEM = { "TOTAL BLOCKS: " };
const char volume_type[] PROGMEM = { "VOLUME TYPE: FAT" };
const char volume_size1[] PROGMEM = { "VOLUME SIZE: " };
const char volume_size2[] PROGMEM = { " kb_\n" };
const char end_line[] PROGMEM = { "_\n" }; // 11
const char test[] PROGMEM = { "INITIALIZATION PHASE ENDED_\n--------------------------------------------\nTESTING PHASE BEGINNING_\n" };
const char test_err1[] PROGMEM = { "AN ERROR HAS OCCURED DURING TESTING PHASE_\nREVIEW LOG FOR MORE INFORMATION_" };
const char test_ok[] PROGMEM = { "TESTING PHASE ENDED_\nALL SYSTEMS NOMINAL_\n" }; // 14
const char sd_init_err[] PROGMEM = { "Error initializing SD card...\n" };
const char volume_init_err[] PROGMEM = { "Error initializing SD volume...\n" };
const char mk_worksp_err[] PROGMEM = { "Error creating workspace...\n" };
const char op_worksp_err[] PROGMEM = { "Error opening workspace...\n" }; // 18
const char wait[] PROGMEM = { "ARMING IGNITION SYSTEM_\n--------------------------------------------\nWAITING FOR LAUNCH SIGNAL_\n" };
const char warn[] PROGMEM = { "LAUNCH SIGNAL RECIVED_\n--------------------------------------------\nWARNING! IGNITION IN 10 SECONDS_\nPRESS ANY BUTTON TO CANCEL_\n" };
const char launch1[] PROGMEM = { "--------------------------------------------\nLIFTOFF_\nIGNITION AT: " };
const char launch2[] PROGMEM = { " microseconds_\n" };
const char startdatalog[] PROGMEM = { "--------------------------------------------\n" };
const char init1[] PROGMEM = { "ALTICRAFT FLIGHT DATA RECORDER_\n\nCOPYRIGHT (C) ROBERT HUTTER 2020\n\nVERSION: 1.0\nBUILD DATE: " }; // 24
const char trig1[] PROGMEM = { "STAGING TRIGGER MODE: " }; // 25
const char init_dig[] PROGMEM = { "Initializing digital output pins...\n" };
const char shutoff1[] PROGMEM = { "SHUTDOWN COMMAND RECIVED_\nSHUTDOWN TIME: " }; // 27
const char shutoff2[] PROGMEM = { "_\nSETTING RGB LED OFF_\nSHUTTING DOWN_\n" };
const char init_servo[] PROGMEM = { "Initializing servo motor...\n" };
const char init3[] PROGMEM = { "\nOPERATIONAL MODES:\n" }; // 30
const char trig2[] PROGMEM = { "\nTRIGGER VALUE: " };
const char mpu_init[] PROGMEM = { "Initializing MPU9250...\n" };
const char bmp_init[] PROGMEM = { "Initializing BMP280...\n" };
const char test_err2[] PROGMEM = { "\nABORTING SETUP_\nSETTING LED TO WHITE_\nRESTART TO TRY AGAIN_\n" };
const char divider[] PROGMEM = { "--------------------------------------------\n" }; // 35
const char bmp_test_err[] PROGMEM = { "Error reading BMP280 chip id...\n" };
const char sd_err_o_ws[] PROGMEM = { "Error opening workspace..." };
const char mpu_test_err[] PROGMEM = { "Error reading MPU9250 device id...\n" }; // 38
const char launch_abort[] PROGMEM = { "SETTING LED TO WHITE_\nRESTART TO TRY AGAIN_\n" }; // 39
const char staging1[] PROGMEM = { "STAGING CONDITION MET AT: " }; // 40
const char staging2[] PROGMEM = { " microseconds_\nACTIVATING STAGING SERVO_\n" };
const char ldetect1[] PROGMEM = { "TOUCHDOWN DETECTED AT: " }; // 42
const char ldetect2[] PROGMEM = { " microseconds_\nSTOPPING RECORDING_\nSETTING LED TO BLUE_\n" };
const char init4[] PROGMEM = { "TOUCHDOWN DETECTION ARMED " };
const char init5[] PROGMEM = { " milliseconds AFTER LAUNCH_\n" }; // 45
const char sar_helper[] PROGMEM = { "ALERTING SEARCH AND RESCUE_\n" }; // 46
const char ldetect3[] PROGMEM = { "PRESS AND HOLD BUTTON TO SHUTDOWN_\n" }; // 47
const char user_abort[] PROGMEM = { "USER ABORTED LAUNCH_\n" }; // 48
const char stab_abort[] PROGMEM = { "LAUNCH STABILITY ABORT_\n" }; // 49

const char* const messages[] PROGMEM =
{
  init2, buzzer_init, rgb_init, sd_init, sd_type1, sd_clusters, sd_blockspersecond,
  sd_totalblocks, volume_type, volume_size1, volume_size2, end_line, test, test_err1,
  test_ok, sd_init_err, volume_init_err, mk_worksp_err, op_worksp_err , wait, warn,
  launch1, launch2, startdatalog, init1, trig1, init_dig, shutoff1, shutoff2,
  init_servo, init3, trig2, mpu_init, bmp_init, test_err2, divider, bmp_test_err,
  sd_err_o_ws, mpu_test_err, launch_abort, staging1, staging2, ldetect1, ldetect2,
  init4, init5, sar_helper, ldetect3, user_abort, stab_abort
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

unsigned long liftoffat = 0; // time of liftoff in miliseconds, for land detection system.
/* END of program variables */

/* Function prototypes */
uint8_t logData(int32_t, float, int32_t, float, float, float, float, float, float);
uint8_t createNewLogFile(SdFile*);
void activateStaging(void);
void abortLaunch(void);

#ifdef DEBUG
void writeOutDebugMessage(int);
void writeOutDebugMessage(String);
#endif

/* END of function prototypes */

void setup(void);
void loop(void);

#endif