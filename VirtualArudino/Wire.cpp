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
* Wire.cpp - Source code of Wire for Virtual Arduino runtime environment.
*
*/

#include "Wire.h"

HardwareWire Wire;

/**
* Contructor for the class HardwareWire.
*
*/
HardwareWire::HardwareWire(void)
{

}

/**
* Destructor for class HardwareWire.
*
*/
HardwareWire::~HardwareWire(void)
{
	if (readbuffer != NULL)
	{
		free(readbuffer);
	}
}

/**
* Begins a Wire connection. Returns 0 if fails.
*
* @return
*/
bool HardwareWire::begin(void)
{
	if (this->isavailable)
	{
		this->isconnected = true;
		logevent(Level::INFO, "Wire.begin called. Successful connection.");
		return true;
	}

	logevent(Level::ERR, "Wire.begin called. Connection failed.");
	return false;
}

/**
* Prepares for a transmission to the given address.
*
* @param addr Address to send data to.
*/
bool HardwareWire::beginTransmission(uint8_t addr)
{
	if (this->isconnected)
	{
		char buff[64];
		sprintf_s(buff, 64, "Wire.beginTransmission called. address=%u", addr);
		logevent(Level::INFO, buff);
		this->targetaddr = addr;
		return true;
	}

	logevent(Level::ERR, "Wire.beginTransmision called. Connection not open.");
	return false;
}

/**
* Writes a byte through Wire.
* Requires an open connection.
* 
* @param val Byte to send
*/
bool HardwareWire::write(uint8_t val)
{
	if (this->targetaddr)
	{
		char buff[64];
		sprintf_s(buff, 64, "Wire.write called. address=%u value=%u", this->targetaddr, val);
		logevent(Level::INFO, buff);
		return true;
	}

	logevent(Level::ERR, "Wire.write called. Connection not open.");
	return false;
}

/**
* Writes a byte buffer over Wire.
* 
* @param buff Pointer to byte buffer.
* @param size Size of buffer.
* @return True if successful.
*/
bool HardwareWire::write(uint8_t* buff, uint8_t size)
{
	if (this->targetaddr)
	{
		for (int i = 0; i < size; i++)
		{
			this->write(*(buff + i));
		}

		return true;
	}

	logevent(Level::ERR, "Wire.write called. Connection not open.");
	return false;
}

/**
* Closes a transmission sequence.
*
*/
bool HardwareWire::endTransmission(void)
{
	this->targetaddr = 0;
	return true;
}

/**
* Opens a transmission sequence to read.
* 
* @param addr Address to read from.
* @param size Number of bytes to read into buffer.
*/
bool HardwareWire::requestFrom(uint8_t addr, uint8_t size)
{
	if (this->isconnected)
	{
		char buff[64];
		sprintf_s(buff, 64, "Wire.requestFrom called. addr=%u size=%u", addr, size);

		if (this->readbuffer != NULL)
		{
			
		}

		this->readbuffer = (uint8_t*)malloc(size);
		for (int i = 0; i < size; i++)
		{
			this->readbuffer[i] = 0;
		}
		this->pos = 0;
		this->buffsize = size;
		return true;
	}

	logevent(Level::ERR, "Wire.requestFrom called. Connection not open.");
	return false;
}

/**
* Read a byte from the read buffer.
* 
* @return A byte from the buffer.
*/
uint8_t HardwareWire::read(void)
{
	if (this->readbuffer != NULL)
	{
		if (this->pos >= this->buffsize)
		{
			return readbuffer[pos++];
		}

		free(this->readbuffer);
	}
}
