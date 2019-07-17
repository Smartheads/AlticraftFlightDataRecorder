#ifndef ARDUINO_H
#define ARDUINO_H

#include <string>
#include <iostream>

#define OUTPUT 0
#define INPUT 1
#define HIGH 1
#define LOW 0

typedef unsigned int uint8_t;

void analogWrite(uint8_t pin, unsigned int value);
void digitalWrite(uint8_t pin, unsigned int value);
void pinMode(uint8_t pin, unsigned int value);
void delay(int ms);
unsigned long micros(void);

class xSerial
{
	public:
		xSerial();
		void begin(int baudrate);
		void print(std::string in);
};

#endif
