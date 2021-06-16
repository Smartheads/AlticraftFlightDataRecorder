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

std::list<vard::Interrupt> intregistry;
std::map<uint8_t, PinModes> pinregistry;
vard::HardwareSerial Serial;
unsigned long vard::sysbasetime;
bool allowinterrupts = true;

/**
* Disable interrupts. 
* 
*/
void noInterrupts(void)
{
	allowinterrupts = false;
}

/**
* Allow interrupts.
*
*/
void interrupts(void)
{
	allowinterrupts = true;
}

/**
* Attach a pin with specific mode to interrupt ISR
*	routine.
* 
* @param interrupt Number of interrupt
* @param ISR Pointer to function to execute on interrupt
* @param mode Interrupt mode, LOW, CHANGE, RISING, FALLING.
*/
void attachInterrupt(int interrupt, void (*ISR)(void), InterruptMode mode)
{
	vard::Interrupt i = vard::Interrupt();
	i.number = interrupt;
	i.ISR = ISR;
	i.mode = mode;
	intregistry.push_back(i);
}

/**
* Detach interrupt. Will effectively disable
*  the interrupt.
*
* @param interrupt Number of interrupt to detach.
*/
void detachInterrupt(int interrupt)
{
	for (std::list<vard::Interrupt>::iterator i = intregistry.begin(); i != intregistry.end(); ++i)
	{
		if ((*i).number == interrupt)
		{
			intregistry.erase(i);
			break;
		}
	}
}

/*
* Logs an event to the standard output.
*
* @param level Level of event.
* @param msg c String of message.
* @param enum level
*/
void vard::logevent(Level level, const char* msg)
{
	SYSTEMTIME systime;
	GetLocalTime(&systime);

	switch (level)
	{
	case vard::Level::FATAL:
		printf("[FATAL][%02d:%02d.%02d] %s\n", systime.wHour, systime.wMinute, systime.wMilliseconds, msg);
		break;

	case vard::Level::ERR:
		printf("[ERROR][%02d:%02d.%02d] %s\n", systime.wHour, systime.wMinute, systime.wMilliseconds, msg);
		break;

	case vard::Level::SEVERE:
		printf("[SEVERE][%02d:%02d.%02d] %s\n", systime.wHour, systime.wMinute, systime.wMilliseconds, msg);
		break;

	case vard::Level::WARNING:
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
	return current - vard::sysbasetime;
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
void pinMode(uint8_t pin, PinModes mode)
{
	// Register pin
	std::map<uint8_t, PinModes>::iterator it = pinregistry.find(pin);
	if (it == pinregistry.end())
	{
		pinregistry.emplace(pin, mode);
	}
	else
	{
		pinregistry.at(pin) = mode;
	}

	switch (mode)
	{
		char buff[64];

	case INPUT:
		sprintf_s(buff, "Pin %u set for INPUT.", pin);
		vard::logevent(vard::Level::INFO, buff);
		break;

	case OUTPUT:
		sprintf_s(buff, "Pin %u set for OUTPUT.", pin);
		vard::logevent(vard::Level::INFO, buff);
		break;

	case INPUT_PULLUP:
		sprintf_s(buff, "Pin %u set for INPUT_PULLUP.", pin);
		vard::logevent(vard::Level::INFO, buff);
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

	// Check registry for pin
	std::map<uint8_t, PinModes>::iterator it = pinregistry.find(pin);
	if (it == pinregistry.end())
	{
		sprintf_s(buff, 64, "digitalWrite called. Pin not registered. pin=%u", pin);
		vard::logevent(vard::Level::ERR, buff);
	}
	else if (pinregistry.at(pin) == PinModes::OUTPUT)
	{
		switch (level)
		{
		case LOW:
			sprintf_s(buff, 64, "digitalWrite called. pin=%u value=LOW", pin);
			vard::logevent(vard::Level::INFO, buff);
			break;

		case HIGH:
			sprintf_s(buff, 64, "digitalWrite called. pin=%u value=HIGH", pin);
			vard::logevent(vard::Level::INFO, buff);
			break;

		default:
			sprintf_s(buff, 64, "digitalWrite called. pin=%u value=undefined", pin);
			vard::logevent(vard::Level::ERR, buff);
			break;
		}
	}
	else
	{
		sprintf_s(buff, 64, "digitalWrite called. Pin not set for OUTPUT. pin=%u", pin);
		vard::logevent(vard::Level::ERR, buff);
	}
}

/**
* Read the digital state of a pin.
*
* @param pin Pin to read.
*/
OutputLevel digitalRead(uint8_t pin)
{
	char buff[64];

	std::map<uint8_t, PinModes>::iterator it = pinregistry.find(pin);
	if (it == pinregistry.end())
	{
		sprintf_s(buff, 64, "digitalRead called. Pin not registered. pin=%u", pin);
		vard::logevent(vard::Level::ERR, buff);
	}
	else if (pinregistry.at(pin) == PinModes::INPUT || pinregistry.at(pin) == PinModes::INPUT_PULLUP)
	{
		sprintf_s(buff, 64, "digitalRead called. pin=%u", pin);
		vard::logevent(vard::Level::INFO, buff);

		return OutputLevel::LOW;
	}
	else
	{
		sprintf_s(buff, 64, "digitalRead called. Pin not set for INPUT. pin=%u", pin);
		vard::logevent(vard::Level::ERR, buff);
	}
	return OutputLevel::LOW;
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

	// Check registry for pin
	std::map<uint8_t, PinModes>::iterator it = pinregistry.find(pin);
	if (it == pinregistry.end())
	{
		sprintf_s(buff, 64, "analogWrite called. Pin not registered. pin=%u", pin);
		vard::logevent(vard::Level::ERR, buff);
	}
	else if (pinregistry.at(pin) == PinModes::OUTPUT)
	{
		sprintf_s(buff, 64, "analogWrite called. pin=%u value=%u", pin, value);
		vard::logevent(vard::Level::INFO, buff);
	}
	else
	{
		sprintf_s(buff, 64, "analogWrite called. Pin not set for OUTPUT. pin=%u", pin);
		vard::logevent(vard::Level::ERR, buff);
	}
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
*	Reads a word from flash memory. 
* 
*	@param cstr Pointer to a cstring saved in flash memory.
*/
const char* pgm_read_word(const char* const* cstr)
{
	return *cstr;
}

/**
*	Copies the string from src to dest. 
* 
*	@param dest Const char array to write to.
*	@param src C string to copy.
*/
void strcpy_P(char* dest, const char* src)
{
	int i = 0;
	while (src[i] != '\0')
	{
		dest[i] = src[i++];
	}
	dest[i] = '\0';
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
* @param r Radix of number.
*/
String::String(int num, Radix r)
{
	switch (r)
	{
		case Radix::HEX:
		{ // local scope
			char buff[32];
			sprintf_s(buff, 32, "%x", num);
			this->str = std::string(buff);
		}
		break;

		case Radix::OCT:
		{ // local scope
			char buff[32];
			sprintf_s(buff, 32, "%o", num);
			this->str = std::string(buff);
		}
		break;

		default:
			char buff[32];
			sprintf_s(buff, 32, "%d", num);
			this->str = std::string(buff);
		break;
	}
}

/**
* Returns the length of the String.
* 
* @return 
*/
unsigned int String::length(void)
{
	return (unsigned int) str.length();
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
	for (unsigned int i = 0; i < size; i++)
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

/**
*	Returns a character at a given index in the string. 
* 
*	@param i Index of character to return.
*/
char String::charAt(unsigned int i)
{
	return str[i];
}

String String::operator+(String b)
{
	char* buff = new char[this->str.length() + b.length() + 1];
	for (unsigned int i = 0; i < str.length(); i++)
	{
		buff[i] = str[i];
	}

	for (unsigned int i = 0; i < b.length(); i++)
	{
		buff[i + str.length()] = b.charAt(i);
	}

	buff[str.length() + b.length()] = '\0';

	String result = String(buff);
	delete[] buff;

	return result;
}

String String::operator+(const char* b)
{
	return operator+(String(b));
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
vard::HardwareSerial::HardwareSerial(void)
{
	
}

/**
* Destructor for class HardwareSerial.
*
*/
vard::HardwareSerial::~HardwareSerial(void)
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
bool vard::HardwareSerial::begin(unsigned long baud)
{
	if (this->isavailable)
	{
		char buff[64];
		sprintf_s(buff, "Serial.begin called: Baud set to %lu.", baud);
		vard::logevent(vard::Level::INFO, buff);

		this->isconnected = true;
		return true;
	}
	else
	{
		vard::logevent(vard::Level::ERR, "Serial.begin called: No device available.");

		return false;
	}
}

/*
* Flushes outgoing serial buffer.
*
*/
void vard::HardwareSerial::flush(void)
{
	vard::logevent(vard::Level::INFO, "Serial.flush called.");
}

/**
* Print a message to Serial.
*
* @param text Text to print to Serial.
*/
void vard::HardwareSerial::print(const char* text)
{
	if (!this->isconnected)
	{
		vard::logevent(vard::Level::ERR, "Serial.print called. Port not open.");
		return;
	}
	size_t len = strlen(text) + 32;
	char* buff = (char*)malloc(len*sizeof(char));
	sprintf_s(buff, len, "Serial.print called: %s", text);
	vard::logevent(vard::Level::INFO, buff);
	free(buff);
}

/**
* Print a message over serial ended by a newline character.
*
* @param text The message to print.
*/
void vard::HardwareSerial::println(const char* text)
{
	if (!this->isconnected)
	{
		vard::logevent(vard::Level::ERR, "Serial.println called. Port not open.");
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
	vard::logevent(vard::Level::INFO, buff);
	free(buff);
}

/**
* Prints a String to Serial.
*
* @param str String to print.
*/
void vard::HardwareSerial::print(String str)
{
	if (!this->isconnected)
	{
		vard::logevent(vard::Level::ERR, "Serial.print called. Port not open.");
		return;
	}
	char* buff;
	buff = (char*)malloc(str.length() + 64);
	sprintf_s(buff, str.length() + 64, "Serial.print called: %s", str.c_str());
	vard::logevent(vard::Level::INFO, buff);
	free(buff);
}

/**
* Prints a String to Serial with a newline character.
*
* @param str String to print.
*/
void vard::HardwareSerial::println(String str)
{
	if (!this->isconnected)
	{
		vard::logevent(vard::Level::ERR, "Serial.println called. Port not open.");
		return;
	}
	char* buff;
	buff = (char*)malloc(str.length() + 64);
	sprintf_s(buff, str.length() + 64, "Serial.println called: %s", str.c_str());
	vard::logevent(vard::Level::INFO, buff);
	free(buff);
}

/**
* Prints a number to Serial in given radix.
*
* @param num Number to print.
* @param radix Radix to print in i.e. DEC.
*/
void vard::HardwareSerial::print(int16_t num, Radix radix)
{
	if (!this->isconnected)
	{
		vard::logevent(vard::Level::ERR, "Serial.print called. Port not open.");
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
void vard::HardwareSerial::println(int16_t num, Radix radix)
{
	if (!this->isconnected)
	{
		vard::logevent(vard::Level::ERR, "Serial.println called. Port not open.");
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
void vard::HardwareSerial::write(uint8_t val)
{
	if (this->isconnected)
	{
		char buff[64];
		sprintf_s(buff, 64, "Wire.write called. value=%u", val);
		vard::logevent(vard::Level::INFO, buff);
	}
	else
	{
		vard::logevent(vard::Level::ERR, "Serial.write called. Port not open.");
	}
}

/**
* Write bytes from buffer to Serial.
*
* @param buff Byte buffer to write from.
* @param size Size of byte buffer
*/
void vard::HardwareSerial::write(uint8_t* buff, unsigned int size)
{
	if (this->isconnected)
	{
		for (unsigned int i = 0; i < size; i++)
		{
			this->write(buff[i]);
		}
	}
	else
	{
		vard::logevent(vard::Level::ERR, "Serial.write called. Port not open.");
	}
}

/**
* Returns the next byte in the incomming byte buffer.
* 
* @return
*/
uint8_t vard::HardwareSerial::read(void)
{
	if (this->isconnected)
	{
		if (this->pos > this->buffsize)
		{
			char buff[64];
			sprintf_s(buff, 64, "Serial.read called. value=%u", this->readbuffer[pos]);
			vard::logevent(vard::Level::INFO, buff);
			return this->readbuffer[pos++];
		}
		else
		{
			vard::logevent(vard::Level::ERR, "Serial.read called. readbuffer empty");
			return 0;
		}
	}
	
	vard::logevent(vard::Level::ERR, "Serial.read called. Port not open.");
	return 0;
}

/**
* Read bytes from incomming byte buffer.
* 
* @param buff Byte buffer to receive data.
* @param len Number of bytes to read. Must be less
*	then Serial.available().
*/
void vard::HardwareSerial::readBytes(uint8_t* buff, unsigned int len)
{
	if (this->isconnected)
	{
		for (unsigned int i = 0; i < len; i++)
		{
			buff[i] = this->read();
		}
	}
	else
	{
		vard::logevent(vard::Level::ERR, "Serial.readBytes called. Port not open.");
	}
}

/**
* Reads a String from Serial.
* 
* @return String object.
*/
String vard::HardwareSerial::readString(void)
{
	if (this->isconnected)
	{
		unsigned int len = this->buffsize - this->pos;
		char* word = (char*)malloc(static_cast<size_t>(len) + 1);
		for (unsigned int i = this->pos; i < this->buffsize; i++)
		{
			word[i - this->pos] = (char) this->read();
		}
		word[len] = '\0';
		String str = String(word);
		free(word);
		return str;
	}

	vard::logevent(vard::Level::ERR, "Serial.readString called. Port not open.");
	return String();
}

/**
* Returns size of incomming byte buffer.
* 
* @return
*/
unsigned int vard::HardwareSerial::available(void)
{
	return this->buffsize - this->pos;
}

/**
* Set the timeout of the serial connection.
*
* @param delay Maximum timeout in milliseconds.
*/
void vard::HardwareSerial::setTimeout(unsigned long delay)
{
	char buff[64];
	sprintf_s(buff, 64, "Serial.setTimeout called. value=%lu", delay);
	vard::logevent(vard::Level::INFO, buff);
}
