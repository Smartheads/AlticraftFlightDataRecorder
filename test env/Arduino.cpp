#include "Arduino.h"

void analogWrite(uint8_t pin, unsigned int value){}
void digitalWrite(uint8_t pin, unsigned int value){}
void pinMode(uint8_t pin, unsigned int value){}
void delay(int ms){}
unsigned long micros(void)
{
	return 134431324;
}

xSerial::xSerial(){}
void xSerial::begin(int baudrate){}
void xSerial::print(std::string in)
{
	std::cout << in;
}

void setup(void);
void loop(void);

int main(void)
{
	setup();
	for (int i = 0; i < 5; i++)
	{
		loop();
	}
	
	return 0;
}
