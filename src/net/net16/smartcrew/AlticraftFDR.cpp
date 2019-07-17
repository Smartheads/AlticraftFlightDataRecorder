/*
* MIT License
*
* Copyright (c) 2019 Robert Hutter
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
* AlticraftFDR.cpp - Main file of the Alticraft Flight Data Recorder software.
*   Execution start here. Program features specified in the SRS.
*
*/

#include <string>
#include <sstream>
#include "Arduino.h"
#include "RGBLED.h"
#include "Buzzer.h"


xSerial Serial; //TEMPORARY


/* BEGINNING OF PROGRAM PREFERANCES */

#define BAUD_RATE 115200							// Change baud rate here
#define ONLY_LAUNCH_AND_LOG ACTIVE						// Change operation mode here

/*	OPERATION MODES: (use double quotes)
*			- "PASSIVE_LOG": Only log flight data. Disables launch and staging features.
*			- "ONLY_LAUNCH_AND_LOG": Enables only launch control and flight data logging.
*			- "LAUNCH_LOG_STAGE": Enables all features including launch, staging control
*					and flight data logging.
*/

#define PRESSURE ACTIVE								// Change trigger mode here
#define TRIGGER_ALTITUDE 200					// Change trigger altitude here
#define TRIGGER_PRESSURE 100.0				// Change trigger pressure here

/*	STAGE TRIGGER MODES: (use double quotes)
*			- "ALTITUDE": Activate staging at a specific altitude. (m)
*			- "PRESSURE": Activate staging at a specific pressure. (kPa)
*/

/* END OF PREFERENCES*/

/* 
* Preprocessing settings
*
*/
#if defined PASSIVE_LOG
	#define OPERATION_MODE "PASSIVE_LOG"
#endif

#if defined ONLY_LAUNCH_AND_LOG
	#define OPERATION_MODE "ONLY_LAUNCH_AND_LOG"
#endif

#if defined LAUNCH_LOG_STAGE
	#define OPERATION_MODE "LAUNCH_LOG_STAGE"
#endif

#if defined ALTITUDE
	#define TRIGGER_MODE "ALTITUDE"
#endif

#if defined PRESSURE
	#define TRIGGER_MODE "PRESSURE"
#endif

typedef struct
{
	enum : uint8_t
	{
		red = 2,
		green = 4,
		blue = 5,
		buzzer = 9
	};
} pins;

const struct
{
	std::string init1 = "--------------------------------------------\nALTICRAFT FLIGHT DATA RECORDER_\n\nCOPYRIGHT (C) ROBERT HUTTER 2019\n\nVERSION: 1.0\nBUILD DATE: 2019.07.17\nOPERATION MODE: " + std::string(OPERATION_MODE) + "\n";
	std::string trig1 = "STAGING TRIGGER MODE: " + std::string(TRIGGER_MODE) + "\nTRIGGER VALUE: ";
	std::string init2 = "--------------------------------------------\nINITIALIZATION PHASE BEGINNING_\n";
	std::string buzzer_init = "Initializing buzzer...\n";
	std::string rgb_init = "Initializing RGB LED...\n";
	std::string test = "INITIALIZATION PHASE ENDED_\n--------------------------------------------\nTESTING PHASE BEGINNING_\n";
	std::string test_err = "--------------------------------------------\nAN ERROR HAS OCCURED DURING TESTING PHASE_\nREVIEW LOG FOR MORE INFORMATION_\nABORTING SETUP_\nSETTING LED TO WHITE_\nRESTART TO TRY AGAIN_\n";
	std::string testok = "TESTING PHASE ENDED_\nALL SYSTEMS NOMINAL_\n";
	std::string wait = "ARMING IGNITION SYSTEM_\n--------------------------------------------\nWAITING FOR LAUNCH SIGNAL_\n";
	std::string warn = "LAUNCH SIGNAL RECIVED_\n--------------------------------------------\nWARNING! IGNITION IN 10 SECONDS_\nPRESS ANY BUTTON TO CANCEL_\n";
	std::string launch1 = "--------------------------------------------\nLIFTOFF_\nIGNITION AT: ";
	std::string launch2 = " microseconds_\n";
	std::string startdatalog = "--------------------------------------------\n";
	std::string dataheader1 = "[+";
	std::string dataheader2 = "us/";
	std::string dataheader3 = "]:\t";
} messages;

template <class T>
inline std::string to_string (const T& t);

rgbled rgb(pins::red, pins::green, pins::blue);
Buzzer buzzer(pins::buzzer);

void writeOut(std::string message);
void logData(std::string message, std::string devicename);

/**
* Execution starts here.
*
*/
void setup()
{
	// Initialize all components, report if there is an error.
	Serial.begin(BAUD_RATE);
	writeOut(messages.init1);
	
	#if defined LAUNCH_LOG_STAGE
		writeOut(messages.trig1);
		#if defined PRESSURE
			writeOut(to_string(TRIGGER_PRESSURE) + " kPa\n");
		#else
			#if defined ALTITUDE
				writeOut(to_string(TRIGGER_ALTITUDE) + " m\n");
			#endif
		#endif
	#endif
	
	writeOut(messages.init2);
	
	#if defined LAUNCH_LOG_STAGE || defined ONLY_LAUNCH_AND_LOG
	  writeOut(messages.buzzer_init);
	  buzzer.turnOn();
	  delay(100);
	  buzzer.turnOff();
	#endif
	
	writeOut(messages.rgb_init);
	rgb.setColor(BLUE);
	
	// Test all components, report if there is an error.
	writeOut(messages.test);
	
	if (false) //TEMPORARY, true if an error occures
	{
		writeOut(messages.test_err);
		rgb.setColor(WHITE);
		
		// Wait forever (until the user reboots the system)
		for(;;){}
	}
	
	writeOut(messages.testok);
	
	#if defined LAUNCH_LOG_STAGE || defined ONLY_LAUNCH_AND_LOG
	  // Wait for ignition sequence to be started.
	  writeOut(messages.wait);
	  rgb.setColor(GREEN);
	  
	  for (int i = 0; i < 10; i++)
	  {
	  	delay(1000);
	  }
	
	
	  // Arm rocket, turn on launch warnings
	  writeOut(messages.warn);
	  
	  // If all went well, LAUNCH
	  writeOut(messages.launch1);
	  writeOut(to_string(micros()));
	  writeOut(messages.launch2);
	#endif
	
	writeOut(messages.startdatalog);
	rgb.setColor(RED);

}

/**
* Main loop. Runs repeatadly after the setup is done.
*
*/
void loop()
{
	// Take measurements
	
	// Save measurements to file
	logData("DATA HERE", "BMP180"); // TEMPORARY
	
	// Create new file every few intervals
}

/**
*	Log data recorded by instruments.
*
*/
void logData(std::string message, std::string devicename)
{
	writeOut(messages.dataheader1);
	writeOut(to_string(micros()));
	writeOut(messages.dataheader2);
	writeOut(devicename);
	writeOut(messages.dataheader3);
	writeOut(message);
	writeOut("\n");
}

/**
*	Writes out a message to the SD card, and Serial. (Serial is a mirror)
*
*	@param message
*/
void writeOut(std::string message)
{
	Serial.print(message);
	// Write to SD card
}

/**
*	Converts class T into an std::string object.
*
*/
template <class T>
inline std::string to_string (const T& t)
{
	std::stringstream ss;
	ss << t;
	return ss.str();
}
