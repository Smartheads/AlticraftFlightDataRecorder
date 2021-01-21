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
	case Level::FATAL:
		printf("[FATAL][%02d:%02d.%02d] %s\n", systime.wHour, systime.wMinute, systime.wMilliseconds, msg);
		break;

	case Level::ERR:
		printf("[ERROR][%02d:%02d.%02d] %s\n", systime.wHour, systime.wMinute, systime.wMilliseconds, msg);
		break;

	case Level::SEVERE:
		printf("[SEVERE][%02d:%02d.%02d] %s\n", systime.wHour, systime.wMinute, systime.wMilliseconds, msg);
		break;

	case Level::WARNING:
		printf("[WARNING][%02d:%02d.%02d] %s\n", systime.wHour, systime.wMinute, systime.wMilliseconds, msg);
		break;

	default:
		printf("[INFO][%02d:%02d.%02d] %s\n", systime.wHour, systime.wMinute, systime.wMilliseconds, msg);
		break;
	}
}

/**
* Returns Virtual Arduino system uptime in milliseconds.
*
* @return
*/
unsigned long millis(void)
{
	SYSTEMTIME systime;
	GetLocalTime(&systime);
	unsigned long current = systime.wHour * 3600000 + systime.wMinute * 60000 + systime.wSecond * 1000 + systime.wMilliseconds;
	return current - sysbasetime;
}

/**
* Returns Virtual Arduino system uptime in microseconds.
*
* @return
*/
unsigned long micros(void)
{
	return millis() * 1000;
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
* Halts execution for a specified duration.
* 
* @param micros Amount if time to halt in microseconds.
*/
void delayMicroseconds(unsigned long micros)
{
	if (micros > 1000) // Smaller then one millisecond
	{
		Sleep((DWORD)micros / 1000);
	}
	// else just skip, we cant sleep for less then a millisecond
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
		logevent(Level::INFO, buff);
		break;

	case OUTPUT:
		sprintf_s(buff, "Pin %u set for OUTPUT.", pin);
		logevent(Level::INFO, buff);
		break;

	case INPUT_PULLUP:
		sprintf_s(buff, "Pin %u set for INPUT_PULLUP.", pin);
		logevent(Level::INFO, buff);
		break;
	}
}

/**
* Toggles a GPIO pin as specified.
*
* @param uint8_t Pin to toggle
* @param uint8_t Logical level of state.
*/
void digitalWrite(uint8_t pin, OutputLevel level)
{
	char buff[64];
	switch (level)
	{
		case LOW:
			sprintf_s(buff, 64, "digitalWrite called. pin=%u value=LOW", pin);
			logevent(Level::INFO, buff);
		break;

		case HIGH:
			sprintf_s(buff, 64, "digitalWrite called. pin=%u value=HIGH", pin);
			logevent(Level::INFO, buff);
		break;

		default:
			sprintf_s(buff, 64, "digitalWrite called. pin=%u value=undefined", pin);
			logevent(Level::ERR, buff);
		break;
	}
}

/**
* Read the digital state of a pin.
*
* @param pin Pin to read.
*/
OutputLevel digitalRead(uint8_t pin)
{
	return LOW;
}

/**
* Outputs an analog PWM signal on a specified pin.
*
* @param pin Pin to toggle
* @param value PWM value
*/
void analogWrite(uint8_t pin, uint16_t value)
{
	char buff[64];
	sprintf_s(buff, 64, "analogWrite called. pin=%u value=%u", pin, value);
	logevent(Level::INFO, buff);
}

/**
* Reads a pulse (either HIGH or LOW) on a pin. For example,
* if level is HIGH, pulseIn() waits for the pin to go from
* LOW to HIGH, starts timing, then waits for the pin to go
* LOW and stops timing. Returns the length of the pulse in
* microseconds.
* 
* Description from arduino.cc
* 
* @param pin Pin number to read pulse from.
* @param level Type of pulse to read.
* @return Time in milliseconds.
*/
unsigned long pulseIn(uint8_t pin, OutputLevel level)
{
	while (digitalRead(pin) != level);
	unsigned long start = millis();
	while (digitalRead(pin) == level);
	return millis() - start;
}

/**
* Constructor for class String expansion of std::string.
*
*/
String::String(void)
{
	this->str = std::string();
}

/**
* Constructor for class String.
* 
* @param cstr c String.
*/
String::String(const char* cstr)
{
	this->str = std::string(cstr);
}

/**
* Constructor for class String.
* 
* @param num Number to create String out of.
*/
String::String(int num)
{
	char buff[32];
	sprintf_s(buff, 32, "%d", num);
	this->str = std::string(buff);
}

/**
* Returns the length of the String.
* 
* @return 
*/
unsigned int String::length(void)
{
	return str.length();
}

/**
* Returns a char array of specifed length.
*
* @param buff Buffer to write char length to
* @param size Size of buffer
*/
void String::toCharArray(char* buff, unsigned int size)
{
	std::string subs = this->str.substr(0, size-2); // Leave room for \0
	for (int i = 0; i < size; i++)
	{
		buff[i] = subs.at(i);
	}
	buff[size - 1] = '\0';
}

/**
* Returns a c string of the String.
* 
* @return pointer to the c string.
*/
const char* String::c_str(void)
{
	return this->str.c_str();
}

String String::operator+(String b)
{
	char* buff = (char*)malloc(this->str.length() + b.length());

}

/**
* Return std::string of String. 
* 
* @return
*/
std::string String::std_string(void)
{
	return this->str;
}

/*
* Constructor method for class HardwareSerial.
*
*/
HardwareSerial::HardwareSerial(void)
{
	this->isavailable = true;
	this->isconnected = false;
	this->readbuffer = NULL;
}

/**
* Destructor for class HardwareSerial.
*
*/
HardwareSerial::~HardwareSerial(void)
{
	if (this->readbuffer != NULL)
	{
		free(this->readbuffer);
	}
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
		logevent(Level::INFO, buff);

		this->isconnected = true;
		return true;
	}
	else
	{
		logevent(Level::ERR, "Serial.begin called: No device available.");

		return false;
	}
}

/*
* Flushes outgoing serial buffer.
*
*/
void HardwareSerial::flush(void)
{
	logevent(Level::INFO, "Serial.flush called.");
}

/**
* Print a message to Serial.
*
* @param text Text to print to Serial.
*/
void HardwareSerial::print(const char* text)
{
	if (!this->isconnected)
	{
		logevent(Level::ERR, "Serial.print called. Port not open.");
		return;
	}
	size_t len = strlen(text) + 32;
	char* buff = (char*)malloc(len*sizeof(char));
	sprintf_s(buff, len, "Serial.print called: %s", text);
	logevent(Level::INFO, buff);
	free(buff);
}

/**
* Print a message over serial ended by a newline character.
*
* @param text The message to print.
*/
void HardwareSerial::println(const char* text)
{
	if (!this->isconnected)
	{
		logevent(Level::ERR, "Serial.println called. Port not open.");
		return;
	}
	/*char* newmsg;
	uint8_t len = strlen(text) + 2;
	newmsg = (char*) malloc(len * sizeof(char));
	strcpy_s(newmsg, len, text);
	newmsg[len-2] = '\n';
	newmsg[len-1] = '\0';
	this->print(newmsg);
	free(newmsg);*/

	size_t len = strlen(text) + 32;
	char* buff = (char*)malloc(len * sizeof(char));
	sprintf_s(buff, len, "Serial.println called: %s", text);
	logevent(Level::INFO, buff);
	free(buff);
}

/**
* Prints a String to Serial.
*
* @param str String to print.
*/
void HardwareSerial::print(String str)
{
	if (!this->isconnected)
	{
		logevent(Level::ERR, "Serial.print called. Port not open.");
		return;
	}
	char* buff;
	buff = (char*)malloc(str.length() + 64);
	sprintf_s(buff, str.length() + 64, "Serial.print called: %s", str.c_str());
	logevent(Level::INFO, buff);
	free(buff);
}

/**
* Prints a String to Serial with a newline character.
*
* @param str String to print.
*/
void HardwareSerial::println(String str)
{
	if (!this->isconnected)
	{
		logevent(Level::ERR, "Serial.println called. Port not open.");
		return;
	}
	char* buff;
	buff = (char*)malloc(str.length() + 64);
	sprintf_s(buff, str.length() + 64, "Serial.println called: %s", str.c_str());
	logevent(Level::INFO, buff);
	free(buff);
}

/**
* Prints a number to Serial in given radix.
*
* @param num Number to print.
* @param radix Radix to print in i.e. DEC.
*/
void HardwareSerial::print(int16_t num, Radix radix)
{
	if (!this->isconnected)
	{
		logevent(Level::ERR, "Serial.print called. Port not open.");
		return;
	}
	char buff[32];
	switch (radix)
	{
		case OCT:
			sprintf_s(buff, 32, "%o", num);
		break;

		case HEX:
			sprintf_s(buff, 32, "%x", num);
		break;

		default:
			sprintf_s(buff, 32, "%d", num);
		break;
	}

	this->print(buff);
}

/**
* Prints a number to Serial in given radix.
*
* @param num Number to print.
* @param radix Radix to print in i.e. DEC.
*/
void HardwareSerial::println(int16_t num, Radix radix)
{
	if (!this->isconnected)
	{
		logevent(Level::ERR, "Serial.println called. Port not open.");
		return;
	}
	char buff[32];
	switch (radix)
	{
	case OCT:
		sprintf_s(buff, 32, "%o", num);
		break;

	case HEX:
		sprintf_s(buff, 32, "%x", num);
		break;

	default:
		sprintf_s(buff, 32, "%d", num);
		break;
	}

	this->println(buff);
}
/**
* Writes a byte to Serial.
* 
* @param val Value of byte to write.
*/
void HardwareSerial::write(uint8_t val)
{
	if (this->isconnected)
	{
		char buff[64];
		sprintf_s(buff, 64, "Wire.write called. value=%u", val);
		logevent(Level::INFO, buff);
	}
	else
	{
		logevent(Level::ERR, "Serial.write called. Port not open.");
	}
}

/**
* Write bytes from buffer to Serial.
*
* @param buff Byte buffer to write from.
* @param size Size of byte buffer
*/
void HardwareSerial::write(uint8_t* buff, unsigned int size)
{
	if (this->isconnected)
	{
		for (int i = 0; i < size; i++)
		{
			this->write(buff[i]);
		}
	}
	else
	{
		logevent(Level::ERR, "Serial.write called. Port not open.");
	}
}

/**
* Returns the next byte in the incomming byte buffer.
* 
* @return
*/
uint8_t HardwareSerial::read(void)
{
	if (this->isconnected)
	{
		if (this->pos > this->buffsize)
		{
			char buff[64];
			sprintf_s(buff, 64, "Serial.read called. value=%u", this->readbuffer[pos]);
			logevent(Level::INFO, buff);
			return this->readbuffer[pos++];
		}
		else
		{
			logevent(Level::ERR, "Serial.read called. readbuffer empty");
			return 0;
		}
	}
	
	logevent(Level::ERR, "Serial.read called. Port not open.");
	return 0;
}

/**
* Read bytes from incomming byte buffer.
* 
* @param buff Byte buffer to receive data.
* @param len Number of bytes to read. Must be less
*	then Serial.available().
*/
void HardwareSerial::readBytes(uint8_t* buff, unsigned int len)
{
	if (this->isconnected)
	{
		for (int i = 0; i < len; i++)
		{
			buff[i] = this->read();
		}
	}
	else
	{
		logevent(Level::ERR, "Serial.readBytes called. Port not open.");
	}
}

/**
* Reads a String from Serial.
* 
* @return String object.
*/
String HardwareSerial::readString(void)
{
	if (this->isconnected)
	{
		unsigned int len = this->buffsize - this->pos;
		char* word = (char*)malloc(static_cast<size_t>(len) + 1);
		for (int i = this->pos; i < this->buffsize; i++)
		{
			word[i - this->pos] = (char) this->read();
		}
		word[len] = '\0';
		String str = String(word);
		free(word);
		return str;
	}

	logevent(Level::ERR, "Serial.readString called. Port not open.");
	return String();
}

/**
* Returns size of incomming byte buffer.
* 
* @return
*/
unsigned int HardwareSerial::available(void)
{
	return this->buffsize - this->pos;
}

/**
* Set the timeout of the serial connection.
*
* @param delay Maximum timeout in milliseconds.
*/
void HardwareSerial::setTimeout(unsigned long delay)
{
	char buff[64];
	sprintf_s(buff, 64, "Serial.setTimeout called. value=%lu", delay);
	logevent(Level::INFO, buff);
}
