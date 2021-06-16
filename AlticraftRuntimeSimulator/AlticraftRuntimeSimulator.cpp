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

int wmain(void)
{
    printf("Alticraft FDR Runtime Simulator - Creates a virtual runtime enviroment for Alticraft Flight Data Recorder Software\n");
    printf("Version: 1.0.0.\n");
    printf("Build date: %s\n\n", __DATE__);

    // Setup virtual arduino
    SYSTEMTIME systime;
    GetLocalTime(&systime);
    sysbasetime = systime.wHour * 3600000 + systime.wMinute * 60000 + systime.wSecond * 1000 + systime.wMilliseconds;

    // Start the program
    setup();

    for (;;)
    {
        loop();
    }
}