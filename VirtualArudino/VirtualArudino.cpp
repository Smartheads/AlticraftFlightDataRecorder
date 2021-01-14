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
* VArduino.cpp - Source code for Virtual Arduino.
*
*/

#include "VirtualArduino.h"

/*
* Logs an event to the standard output.
*
* @param level Level of event.
* @param msg c String of message.
* @param enum level
*/
void logevent(Level level, const char* msg)
{
	SYSTEMTIME systime;
	GetLocalTime(&systime);

	switch (level)
	{
	case FATAL:
		printf("[FATAL][%02d:%02d.%02d] %s\n", systime.wHour, systime.wMinute, systime.wMilliseconds, msg);
		break;

	case ERR:
		printf("[ERROR][%02d:%02d.%02d] %s\n", systime.wHour, systime.wMinute, systime.wMilliseconds, msg);
		break;

	case SEVERE:
		printf("[SEVERE][%02d:%02d.%02d] %s\n", systime.wHour, systime.wMinute, systime.wMilliseconds, msg);
		break;

	case WARNING:
		printf("[WARNING][%02d:%02d.%02d] %s\n", systime.wHour, systime.wMinute, systime.wMilliseconds, msg);
		break;

	default:
		printf("[INFO][%02d:%02d.%02d] %s\n", systime.wHour, systime.wMinute, systime.wMilliseconds, msg);
		break;
	}
}

/**
* Halts execution for a specified duration.
*
* @param milis Amount of time to halt in ms.
*/
void delay(unsigned long milis)
{
	Sleep(milis);
}

/**
* Setup an GPIO pin for usage.
*
* @param pin Number of pin to configure.
* @param mode Configuration mode.
*/
void pinMode(uint8_t pin, uint8_t mode)
{
	switch (mode)
	{
		char buff[64];

	case INPUT:
		sprintf_s(buff, "Pin %u set for INPUT.", pin);
		logevent(INFO, buff);
		break;

	case OUTPUT:
		sprintf_s(buff, "Pin %u set for OUTPUT.", pin);
		logevent(INFO, buff);
		break;

	case INPUT_PULLUP:
		sprintf_s(buff, "Pin %u set for INPUT_PULLUP.", pin);
		logevent(INFO, buff);
		break;
	}
}

/*
* Constructor method for class HardwareSerial.
*
*/
HardwareSerial::HardwareSerial(void)
{
	this->isavailable = true;
	this->isconnected = false;
}

/**
* Destructor for class HardwareSerial.
*
*/
HardwareSerial::~HardwareSerial(void)
{

}

/*
* Begins serial communication.
*
* Returns 1 if successful and 0 if failed.
*
* @param baud - Baud rate of serial communication
* @return
*/
bool HardwareSerial::begin(unsigned long baud)
{
	if (this->isavailable)
	{
		char buff[64];
		sprintf_s(buff, "Serial.begin called: Baud set to %lu.", baud);
		logevent(INFO, buff);

		this->isconnected = true;
		return true;
	}
	else
	{
		logevent(WARNING, "Serial.begin called: No device available.");

		return false;
	}
}

/*
* Flushes outgoing serial buffer.
*
*/
void HardwareSerial::flush(void)
{

}

/**
* Print a message to Serial.
*
* @param text Text to print to Serial.
*/
void HardwareSerial::print(const char* text)
{
	uint8_t len = strlen(text) + 32;
	char* buff = (char*)malloc(len*sizeof(char));
	sprintf_s(buff, len, "Serial.print called: %s", text);
	logevent(INFO, buff);
	free(buff);
}

/**
* Print a message over serial ended by a newline character.
*
* @param text The message to print.
*/
void HardwareSerial::println(const char* text)
{
	char* newmsg;
	uint8_t len = strlen(text) + 2;
	newmsg = (char*) malloc(len * sizeof(char));
	strcpy_s(newmsg, len, text);
	newmsg[len-2] = '\n';
	newmsg[len-1] = '\0';
	this->print(newmsg);
	free(newmsg);
}
