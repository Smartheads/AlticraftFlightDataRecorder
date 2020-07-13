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
* BMP280CalibrationDataReader.cpp - A program designed to read calibration 
*   data from BMP280 environmental sensor used in Alticraft Flight Data Recorder.
*
*/

#include <I2C.h>

/* BEGINNING OF PROGRAM PREFERENCES */

#define BAUD_RATE 115200              // Change baud rate here
#define BMP280_ADDRESS 0x76           // I2C address of BMP280

/* END OF PREFERENCES*/

/* BMP280 Registers */
#define BMP280_DIG_T1 0x88
#define BMP280_DIG_T2 0x8A
#define BMP280_DIG_T3 0x8C
#define BMP280_DIG_P1 0x8E
#define BMP280_DIG_P2 0x90
#define BMP280_DIG_P3 0x92
#define BMP280_DIG_P4 0x94
#define BMP280_DIG_P5 0x96
#define BMP280_DIG_P6 0x98
#define BMP280_DIG_P7 0x9A
#define BMP280_DIG_P8 0x9C
#define BMP280_DIG_P9 0x9E

#define BMP280_CHIP_ID 0xD0
#define BMP280_CHIP_ID_VALUE 0x58

// Global variables
SRL::I2CDevice bmp(BMP280_ADDRESS);
uint16_t dig_T1, dig_P1;
int16_t dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("---- BMP280 Calibration Data Reader ----");
  Serial.println("Copyright (c) 2020 Robert Hutter\n");

  // Test BMP280
  uint8_t bmpChipId = 0;
  bmp.readBytes(BMP280_CHIP_ID, &bmpChipId, 1);
  
  if (bmpChipId != BMP280_CHIP_ID_VALUE)
  {
    Serial.println("Error connecting to BMP280...\n");
    Serial.println("Please try again later...\n");

    Serial.println("---- End of Program ----");
    Serial.flush();
    exit(1);
  }

  Serial.println("Reading calibration data...\n");

  bmp.readUShort(BMP280_DIG_T1, &dig_T1);
  bmp.readSShort(BMP280_DIG_T2, &dig_T2);
  bmp.readSShort(BMP280_DIG_T3, &dig_T3);
  bmp.readUShort(BMP280_DIG_P1, &dig_P1);
  bmp.readSShort(BMP280_DIG_P2, &dig_P2);
  bmp.readSShort(BMP280_DIG_P3, &dig_P3);
  bmp.readSShort(BMP280_DIG_P4, &dig_P4);
  bmp.readSShort(BMP280_DIG_P5, &dig_P5);
  bmp.readSShort(BMP280_DIG_P6, &dig_P6);
  bmp.readSShort(BMP280_DIG_P7, &dig_P7);
  bmp.readSShort(BMP280_DIG_P8, &dig_P8);
  bmp.readSShort(BMP280_DIG_P9, &dig_P9);

  Serial.println("Dig T1: " + String(dig_T1, DEC));
  Serial.println("Dig T2: " + String(dig_T2, DEC));
  Serial.println("Dig T3: " + String(dig_T3, DEC));
  Serial.println("Dig P1: " + String(dig_P1, DEC));
  Serial.println("Dig P2: " + String(dig_P2, DEC));
  Serial.println("Dig P3: " + String(dig_P3, DEC));
  Serial.println("Dig P4: " + String(dig_P4, DEC));
  Serial.println("Dig P5: " + String(dig_P5, DEC));
  Serial.println("Dig P6: " + String(dig_P6, DEC));
  Serial.println("Dig P7: " + String(dig_P7, DEC));
  Serial.println("Dig P8: " + String(dig_P8, DEC));
  Serial.println("Dig P9: " + String(dig_P9, DEC));

  Serial.println("\n---- End of Program ----");

  Serial.flush();
  exit(0);
}

void loop() {
  // put your main code here, to run repeatedly:

}