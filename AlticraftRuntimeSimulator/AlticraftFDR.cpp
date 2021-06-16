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

#include "AlticraftFDR.h"

#ifdef DEBUG
    char outBuffer[135];
#endif

#if defined(LAUNCH_LOG_STAGE) || defined(ONLY_LAUNCH_AND_LOG)
#include <Buzzer.h>
    SRL::Buzzer* buzzer;
#endif
#ifdef LAUNCH_LOG_STAGE
#include <Servo.h>
    Servo stageservo;
#endif

SRL::I2CDevice* bmp;
SRL::I2CDevice* mpu;
SRL::rgbled* rgb;
Sd2Card* sdcard;
SdVolume* sdvolume;
SdFile* logfile;
SdFile* debugFile;
SdFile* sdfilemanager;

unsigned long liftoffat = 0;

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

    #ifdef LAUNCH_STABILITY_ABORT
      writeOutDebugMessage("LAUNCH_STABILITY_ABORT_\n");
    #endif

    #ifdef TOUCHDOWN_DETECTION
      writeOutDebugMessage("TOUCHDOWN_DETECTION_\n");
    #endif

    #ifdef SAR_HELPER
      writeOutDebugMessage("SAR_HELPER_\n");
    #endif
    
    #ifdef LAUNCH_LOG_STAGE
      #ifndef APOAPSIS
        writeOutDebugMessage(25);
        writeOutDebugMessage(String(TRIGGER_MODE));
        writeOutDebugMessage(31);
      #endif
      #ifdef PRESSURE
        writeOutDebugMessage(String(TRIGGER_PRESSURE) + F(" kPa\n"));
      #else
        #ifdef ALTITUDE
          writeOutDebugMessage(String(TRIGGER_ALTITUDE) + F(" m\n"));
        #endif
      #endif
    #endif

    #ifdef TOUCHDOWN_DETECTION
      writeOutDebugMessage(44);
      writeOutDebugMessage(String(TOUCHDOWN_DETECTION_FROM, DEC));
      writeOutDebugMessage(45);
    #endif

    writeOutDebugMessage(35);
    writeOutDebugMessage(0);
    Serial.flush();
  #endif
  
  // Initialize sensors
  #ifdef DEBUG
    writeOutDebugMessage(2);
  #endif
  rgb = new SRL::rgbled(RED_PIN, GREEN_PIN, BLUE_PIN);
  rgb->setColor(BLUE);

  // Initialize MPU9250
  #ifdef DEBUG
    writeOutDebugMessage(32);
  #endif
  mpu = new SRL::I2CDevice(MPU9250_ADDRESS);
  { // Local scope for gc and ac
    uint8_t gc = 0;
    mpu->readBytes(MPU9250_GYRO_CONFIG, &gc, 1);
    gc = ((uint8_t)0xE7) & gc; // 11100111 mask
    gc = gc | ((uint8_t)GYRO_SENSITIVITY);
    mpu->writeByte(MPU9250_GYRO_CONFIG, gc);
    
    uint8_t ac = 0;
    mpu->readBytes(MPU9250_ACCEL_CONFIG, &ac, 1);
    ac = ((uint8_t)0xE7) & ac; // 11100111 mask
    ac = ac | ((uint8_t)ACCEL_SENSITIVITY);
    mpu->writeByte(MPU9250_ACCEL_CONFIG, ac);
  }

  // Initialize BMP280
  #ifdef DEBUG
    writeOutDebugMessage(33);
  #endif
  bmp = new SRL::I2CDevice(BMP280_ADDRESS);
  { // Local scope for bmpSetting
    uint8_t bmpSetting = (((uint8_t)TEMPERATURE_OVERSAMPLING) << 5) | (((uint8_t)PRESSURE_OVERSAMPLING) << 2) | ((uint8_t)BMP280_POWER_MODE);
    bmp->writeByte(BMP280_CTRL_MEAS, bmpSetting);
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
    stageservo.write(SERVO_START_ANGLE);
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
  sdcard = new Sd2Card;
  uint8_t sdtest = sdcard->init(SPI_FULL_SPEED, SD_CHIP_SELECT);

  // Initialize volume if card checks out
  if (sdtest == 0x01)
  {
    sdvolume = new SdVolume;
    sdtest = (((uint8_t)sdvolume->init(sdcard)) << 1) | sdtest;

    #ifdef DEBUG
      if(sdtest == 0x03) // Print information on sdvolume
      {
        writeOutDebugMessage(4);
        writeOutDebugMessage(String(sdcard->type()));
        writeOutDebugMessage(11);
        
        writeOutDebugMessage(5);
        writeOutDebugMessage(String(sdvolume->clusterCount()));
        writeOutDebugMessage(11);
  
        writeOutDebugMessage(6);
        writeOutDebugMessage(String(sdvolume->blocksPerCluster()));
        writeOutDebugMessage(11);
  
        writeOutDebugMessage(7);
        writeOutDebugMessage(String(sdvolume->blocksPerCluster() * sdvolume->clusterCount()));
        writeOutDebugMessage(11);
  
        writeOutDebugMessage(8);
        writeOutDebugMessage(String(sdvolume->fatType()));
        writeOutDebugMessage(11);
  
        writeOutDebugMessage(9);
        writeOutDebugMessage(String(sdvolume->blocksPerCluster() * sdvolume->clusterCount() / 2));
        writeOutDebugMessage(10);
      }
    #endif

    // Create a new directory to log data into and open it
    sdfilemanager = new SdFile;
    sdtest = (((uint8_t)sdfilemanager->openRoot(sdvolume)) << 2) | sdtest;

    if(sdtest == 0x07) // 0111
    {
      uint8_t num = 0;
      String dir = WORKSPACE_DIR_NAME;
      String dirname = dir + String(num);
      bool success = false;
      while (!success)
      {
        SdFile sdf;
        success = sdf.makeDir(sdfilemanager, dirname.c_str());
        if (!success)
        {
          dirname = dir + String(++num); // Issue somewhere here
        }
      }

      SdFile* sdf = new SdFile;
      sdtest = ((uint8_t)(sdf->open(sdfilemanager, dirname.c_str(), O_READ)) << 3) | sdtest;
      delete sdfilemanager;
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
  mpu->readBytes(MPU9250_WHO_AM_I, &mpuChipId, 1);

  #ifdef DEBUG
    if (mpuChipId != MPU9250_WHO_AM_I_VALUE)
    {
      writeOutDebugMessage(38); // Error with MPU9250
    }
  #endif

  // Test BMP280, read device ID
  uint8_t bmpChipId = 0;
  bmp->readBytes(BMP280_CHIP_ID, &bmpChipId, 1);

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
    
    rgb->setColor(WHITE);
      
    // Wait forever (until the user reboots the system)
    sdfilemanager->close();
    for(;;){}
  }

  #ifdef DEBUG
    debugFile = new SdFile;
    debugFile->open(sdfilemanager, "debug.txt", O_CREAT | O_WRITE);
    writeOutDebugMessage(14);
  #endif
  
  // If launch mode enabled, follow launch prcedure
  #if defined(LAUNCH_LOG_STAGE) || defined(ONLY_LAUNCH_AND_LOG)
    #ifdef DEBUG
      writeOutDebugMessage(19);
    #endif
    rgb->setColor(GREEN);

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
    rgb->setColor(RED);
    delay(LAUNCH_TIMEOUT);
    rgb->setColor(GREEN);

    // Arm rocket, turn on launch warnings
    buzzer->turnOn();

    #ifdef DEBUG
      writeOutDebugMessage(20); // Launch signal recieved
    #endif
  #elif // PASSIVE_LOG mode
    rbg.setColor(RED);
  #endif

  #ifdef DEBUG
    writeOutDebugMessage(23);
  #endif

  // Create log file
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
  uint8_t logsuccess = 0;
  int32_t pressure = 0, temperature = 0;
  float ax = 0.0f, ay = 0.0f, az = 0.0f, gx = 0.0f, gy = 0.0f, gz = 0.0f, altitude = 0.0f;

  #ifdef MEASUREMENT_INTERVAL
    static unsigned long lastlog = millis();
  
    if (lastlog + MEASUREMENT_INTERVAL <= millis())
    {
  #endif
    { // BMP280
      uint8_t bmpBuffer[6] = { 0 };
      bmp->readBytes(BMP280_PRESS, bmpBuffer, 6);
      
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
      mpu->readBytes(MPU9250_ACCEL_DATA, mpuBuffer, 14);
      
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
  
      // Save measurements to file
      logsuccess = logData(pressure, altitude, temperature, ax, ay, az, gx, gy, gz);
  #ifdef MEASUREMENT_INTERVAL
      lastlog = millis();
    }
  #endif

  // Basic flight control operations (launch and staging)
  #if defined(LAUNCH_LOG_STAGE) || defined(ONLY_LAUNCH_AND_LOG)
    if (liftoffat == 0) // Use variable liftoffat as marker (0 if not launched yet)
    {
      static unsigned long cdownstarted = millis(), lcolorswitch = millis();
      static unsigned int badmeascount = 0, badwritecount = 0;
      
      // Listen for abort
      if (digitalRead(BUTTON_PIN) == HIGH)
      {
        #ifdef DEBUG
          writeOutDebugMessage(48);
        #endif
        abortLaunch();
      }

      // Make sure sensors are working
      if (ax != 0.0f && ay != 0.0f && az != 0.0f && gx != 0.0f
          && gy != 0.0f && gz != 0.0f && pressure != 0 && temperature != 0)
      {
        badmeascount = 0; // Reset counter if all sensors in check
      }
      else if (badmeascount > BAD_MEAS_ABORT_THRESHOLD)
      {
        // Abort on suspected hardware malfunction of sensors
        #ifdef DEBUG
        writeOutDebugMessage(48);
        #endif
        abortLaunch();
      }
      else
      {
        badmeascount++;
      }

      // Make sure sd card is working (saves are successfull)
      if (logsuccess) // Not zero
      {
          badwritecount = 0; // Reset counter if sd card works
      }
      else if (badwritecount > BAD_WRITE_ABORT_THRESHOLD)
      {
        // Abort on suspected hardware malfunction of sd card
        #ifdef DEBUG
          writeOutDebugMessage(48);
        #endif
          abortLaunch();
      }
      else
      {
          badwritecount++;
      }

      // Check launch stability if mode active
      #ifdef LAUNCH_STABILITY_ABORT
        static float lax = ax, lay = ay, laz = az;

        if (ax != 0.0f && ay != 0.0f && az != 0.0f) // Filter error or no measurement this cycle
        {
          if (
            (abs(ax-lax) > ((float)LAUNCH_ABORT_SENSITIVITY)
            || abs(ay-lay) > ((float)LAUNCH_ABORT_SENSITIVITY)
            || abs(az-laz) > ((float)LAUNCH_ABORT_SENSITIVITY))
            && lax != 0.0f) // Dont trigger if lax is empty, first update!
          {
            // Deviation out of threshold, abort launch
            #ifdef DEBUG
              writeOutDebugMessage(49);
            #endif
            abortLaunch();
          }
          
          lax = ax;
          lay = ay;
          laz = az;
        }
      #endif

      // Color warning
      if (lcolorswitch + 500 <= millis())
      {
        lcolorswitch = millis();
        if (compareColor(rgb->getColor(), GREEN))
        {
          rgb->setColor(RED);
        }
        else
        {
          rgb->setColor(GREEN);
        }
      }

      // Countdown done, time to launch
      if (cdownstarted + LAUNCH_COUNTDOWN < millis())
      {
        // Ignite rocket (first stage)
        digitalWrite(LAUNCH_PIN, HIGH);
        liftoffat = millis();
        rgb->setColor(RED);
    
        // Buzzer no longer needed, destruct
        buzzer->turnOff();
        #ifndef SAR_HELPER // Still needed for touchdown detection
          delete buzzer;
        #endif
      
        #ifdef DEBUG
          // If all went well, LAUNCH
          writeOutDebugMessage(21);
          writeOutDebugMessage(String(micros()));
          writeOutDebugMessage(22);
        #endif 
      }
    }
    #ifdef LAUNCH_LOG_STAGE
      else // If rocket already in flight. Only important if staging mode active
      {
        static bool stagingHappened = false;
      
        #ifdef PRESSURE
          static unsigned long lpmeas = millis();
    
          if (pressure <= TRIGGER_PRESSURE)
          {
            if (lpmeas + STAGE_TRIGGER_TIME <= millis())
            {
                // Staging command
                if (!stagingHappened)
                {
                  activateStaging();
                  stagingHappened = true;
                }
            }
          }
          // Reset timer if condition not met
          // Discard false readings, note measurements are not taken every iteration of the loop
          else if (pressure != 0)
          {
            lpmeas = millis();
          }
        #endif
        #ifdef ALTITUDE
          static unsigned long ameas = millis();
    
          if (altitude >= ((float)TRIGGER_ALTITUDE))
          {
            if (ameas + STAGE_TRIGGER_TIME <= millis())
            {
                // Staging command
                if (!stagingHappened)
                {
                  activateStaging();
                  stagingHappened = true;
                }
            }
          }
          else if (altitude != 0.0f) // reset timer if condition not met
          {
            ameas = millis();
          }
        #endif
        #ifdef TEMPERATURE
          static unsigned long tmeas = millis();
    
          if ((temperature < TRIGGER_TEMPERATURE_MIN || temperature > TRIGGER_TEMPERATURE_MAX))
          {
            if (tmeas + STAGE_TRIGGER_TIME <= millis())
            {
                // Staging command
                if (!stagingHappened)
                {
                  activateStaging();
                  stagingHappened = true;
                }
            }
          }
          else if (temperature != 0) // reset timer if condition not met
          {
            tmeas = millis();
          }
        #endif
        #ifdef APOAPSIS
          static unsigned long apmeas = millis();
          static float lalt = altitude;
          
          if (altitude != 0) // discard false readings
          {
            if (altitude < lalt)
            {
              if (apmeas + STAGE_TRIGGER_TIME <= millis())
              {
                // Staging command
                if (!stagingHappened)
                {
                  activateStaging();
                  stagingHappened = true;
                }
              }
            }
            else
            {
              apmeas = millis();
            }
    
            lalt = altitude;
          }
        #endif
      }
    #endif
  #endif
  
  // touchdown detection
  #if defined(TOUCHDOWN_DETECTION) || defined(SAR_HELPER)
    static unsigned long ldtime = millis();
    static float ldalt = altitude;
    static bool touchdownHappened = false;
  
    if (altitude != 0.0f && liftoffat != 0)
    {
      float absvalue = ldalt - altitude; // Must be calculated outside of abs(), due to function limitation. @see Arduino docs
      if (abs(absvalue) < 1.0f)
      {
        if (ldtime + TOUCHDOWN_DETECTION_TIME <= millis())
        {
          if ((liftoffat + TOUCHDOWN_DETECTION_FROM < millis()) && !touchdownHappened)
          {
            #ifdef DEBUG
              writeOutDebugMessage(35);
              writeOutDebugMessage(42);
              writeOutDebugMessage(String(micros(), DEC));
              writeOutDebugMessage(43);
              #ifdef SAR_HELPER
                writeOutDebugMessage(46);
              #endif
              writeOutDebugMessage(47);
            #endif
            
            #ifdef TOUCHDOWN_DETECTION
                logfile->close();
                sdfilemanager->close();
            #endif

            #ifdef SAR_HELPER
              buzzer->turnOn();
            #endif
            
            rgb->setColor(BLUE);
            touchdownHappened = true;
          }
        }
        else if (liftoffat + TOUCHDOWN_DETECTION_FROM >= millis())
        {
          ldtime = millis(); // Reset clock, so detection doesnt occur right after launch
        }
      }
      else
      {
        ldtime = millis();
      }
  
      ldalt = altitude;
    }
  #endif
  
  // Create new file every few intervals
  static unsigned long lasttime = millis();

  if (lasttime + NEWFILE_INTERVAL_TIME <= millis())
  {
    // Create a new file
    createNewLogFile(logfile);
    lasttime = millis();
  }
  
  // Check to see if it's time to stop the recording
  static unsigned long lastpressed = millis();
  if(digitalRead(BUTTON_PIN) == HIGH)
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
      debugFile->close();
      sdfilemanager->close();
      Serial.flush();

      #ifdef SAR_HELPER
        buzzer->turnOff();
      #endif
      
      rgb->turnOff();
      digitalWrite(LAUNCH_PIN, LOW);
      exit(0);
    }
  }
  else
  {
    lastpressed = millis();
  }
}

/**
* Log data recorded by instruments.
*
* @return Returns 1 (true) if all write operations were successfull and 0 (false) if not.
*/
uint8_t logData(int32_t p, float a, int32_t t, float ax, float ay, float az, float gx, float gy, float gz)
{
  uint8_t ret = logfile->write(String(micros(), DEC).c_str());
  ret = ret && logfile->write(',');
  ret = ret && logfile->write(String(p, DEC).c_str());
  ret = ret && logfile->write(',');
  ret = ret && logfile->write(String(a, DEC).c_str());
  ret = ret && logfile->write(',');
  ret = ret && logfile->write(String(t, DEC).c_str());
  ret = ret && logfile->write(',');
  ret = ret && logfile->write(String(ax, DEC).c_str());
  ret = ret && logfile->write(',');
  ret = ret && logfile->write(String(ay, DEC).c_str());
  ret = ret && logfile->write(',');
  ret = ret && logfile->write(String(az, DEC).c_str());
  ret = ret && logfile->write(',');
  ret = ret && logfile->write(String(gx, DEC).c_str());
  ret = ret && logfile->write(',');
  ret = ret && logfile->write(String(gy, DEC).c_str());
  ret = ret && logfile->write(',');
  ret = ret && logfile->write(String(gz, DEC).c_str());
  ret = ret && logfile->write('\n');

  return ret;
}

/**
* Creates a new logfile. 
* 
* @param logfile Reference to SdFile.
* @return Returns 1 (true) if successfully creates the new logfile and 0 (false) if not.
*/
uint8_t createNewLogFile(SdFile* logfile)
{
  static uint16_t fileId = 0;
  logfile->close();
  return logfile->open(sdfilemanager, (String("log_") + String(fileId++) + String(".csv")).c_str(), O_CREAT | O_WRITE);
}

/**
 * Activates staging.
 * 
 */
void activateStaging(void)
{
  #ifdef DEBUG
    writeOutDebugMessage(35);
    writeOutDebugMessage(40);
    writeOutDebugMessage(String(micros(), DEC));
    writeOutDebugMessage(41);
  #endif

  // Activate servo motor
  stageservo.write(SERVO_STAGE_ANGLE);
}

/**
 * Aborts the launch sequence and shuts down
 */
void abortLaunch(void)
{
  rgb->setColor(WHITE);
  buzzer->turnOff();

  #ifdef DEBUG
    writeOutDebugMessage(35); // Divider
    writeOutDebugMessage(39); // User abort
  #endif

  logfile->close();
  sdfilemanager->close();
  for(;;) {} // Wait until reset
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
    debugFile->write(outBuffer);
    Serial.print(outBuffer);
  }

  /**
   * Overloaded function of writeOut with
   * @param message String object
   */
  void writeOutDebugMessage(String message)
  {
    debugFile->write(message.c_str());
    Serial.print(message);
  }
#endif