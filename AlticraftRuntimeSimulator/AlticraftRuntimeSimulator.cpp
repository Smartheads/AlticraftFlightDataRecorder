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
* AlticraftRuntimeSimulator.cpp - Main file of the Alticraft Runtime Simulator.
*   Execution starts here. This program creates a virual runtime enviroment for
*   Alticraft Flight Data Recorder software releases.
*
*   This program is designed to test new versions of the Alticraft Flight Data
*   Recorder software before deploying to hardware solutions.
*/
#include "AlticraftFDR.h"
//#include "AlticraftTest.h"
#include <VirtualArduino.h>
#include <stdio.h>

// Configure virtual i2c devices
void mpuEventHandler(uint8_t*, unsigned int);
vard::I2C_Device vmpu(0x68, &mpuEventHandler);

void bmpEventHandler(uint8_t*, unsigned int);
vard::I2C_Device vbmp(0x76, &bmpEventHandler);

// Configure virtual input pins
OutputLevel buttonReader(void);
vard::DigitalInputPin buttonPin(6, &buttonReader);

int wmain(void)
{
    printf("Alticraft FDR Runtime Simulator - Creates a virtual runtime enviroment for Alticraft Flight Data Recorder Software\n");
    printf("Version: 1.0.0.\n");
    printf("Build date: %s\n\n", __DATE__);

    // Setup virtual arduino
    SYSTEMTIME systime;
    GetLocalTime(&systime);
    vard::sysbasetime = systime.wHour * 3600000 + systime.wMinute * 60000 + systime.wSecond * 1000 + systime.wMilliseconds;
    vard::winconsole = GetStdHandle(STD_OUTPUT_HANDLE);
    vard::sdvolume_root = "C:\\Users\\Robert Hutter\\Documents\\GitHub\\AlticraftFlightDataRecorder\\AlticraftRuntimeSimulator\\v_sdvolume_root\\";

    // Attach i2c devices
    vard::attach_I2C_Device(&vmpu);
    vard::attach_I2C_Device(&vbmp);

    // Attach virtual input pins
    vard::attach_digital_input_pin(&buttonPin);

    // Start the program
    setup();

    for (;;)
    {
        loop();
    }
}

/* Emulate mpu9250 */
void mpuEventHandler(uint8_t* readbuffer, unsigned int size)
 {
    switch (readbuffer[0])
    {
    case 0x1b: // GYRO_CONFIG
        if (size == 1)
        {
            vmpu.push(0);
        }
        else
        {
            vard::logevent(vard::Level::INFO, "MPU9250 gyro config register set to %u.", readbuffer[1]);
        }
        break;

    case 0x1c: // ACCEL_CONFIG
        if (size == 1)
        {
            vmpu.push(0);
        }
        else
        {
            vard::logevent(vard::Level::INFO, "MPU9250 accel config register set to %u.", readbuffer[1]);
        }
        break;

    case 0x75:
        vmpu.push(0x71);
        break;
    }
}

/* emulate bmp280 */
void bmpEventHandler(uint8_t* readbuffer, unsigned int size)
{
    switch (readbuffer[0])
    {
    case 0xD0:
        vbmp.push(0x58);
        break;
    
    case 0xF4:
        vard::logevent(vard::Level::INFO, "BMP280 ctrl meas register set to %u.", readbuffer[1]);
        break;
    }
}

OutputLevel buttonReader(void)
{
    return OutputLevel::HIGH;
}