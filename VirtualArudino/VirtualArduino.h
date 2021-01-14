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
* VArduino.h - Header file of virtual Arduino source file.
*
*/

#define WIN32_LEAN_AND_MEAN

#ifndef __VARDUINO_H__
#define __VARDUINO_H__

#include <stdint.h>
#include <string>

#define INPUT WININPUT
#include <windows.h>
#include <sysinfoapi.h>
#include <minwinbase.h>
#include <synchapi.h>
#undef INPUT

#define ARDUINO 100

#define String std::string

enum Level { FATAL, ERR, SEVERE, WARNING, INFO };
enum PinModes { INPUT, OUTPUT, INPUT_PULLUP };

void logevent(Level, const char*);

void delay(unsigned long);
void pinMode(uint8_t, uint8_t);

class HardwareSerial
{
public:
	HardwareSerial(void);
	~HardwareSerial(void);

	bool begin(unsigned long);
	void flush(void);
	
	void print(const char* text);
	void println(const char* text);

private:
	bool isavailable, isconnected;
};

extern HardwareSerial Serial;

#endif