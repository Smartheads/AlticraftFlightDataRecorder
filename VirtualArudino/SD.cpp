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
* SD.cpp - Source code for virtual SD card. Part of the Virtual Arduino runtime
*	environment.
*
*/

#include "SD.h"

/*
* Constructor method for class Sd2Card.
*
*/
Sd2Card::Sd2Card(void)
{

}

/** 
* Destructor for class Sd2Card.
*
*/
Sd2Card::~Sd2Card(void)
{

}

/**
*	Initializes Sd2Card. 
* 
*	@param speed W/R speed selection of chip.
*	@param cs Chip select number.
*/
uint8_t Sd2Card::init(int speed, int cs)
{
	char buff[64];
	sprintf_s(buff, 64, "SD2Card.init called. speed=%d cs=%d", speed, cs);
	vard::logevent(vard::Level::INFO, buff);
	return true;
}

/**
*	Returns Sd card type. 
* 
*/
uint8_t Sd2Card::type(void)
{
	return SD_TYPE;
}

/*
* Constructor method for class SdVolume.
*
*/
SdVolume::SdVolume(void)
{

}

/**
* Destructor for class SdVolume.
*
*/
SdVolume::~SdVolume(void)
{

}

/**
*	Initializes SdVolume. 
* 
*	@sd SD card volume is found on.
*/
uint8_t SdVolume::init(Sd2Card* sd)
{
	vard::logevent(vard::Level::INFO, "SdVolume.init called.");
	return true;
}

/**
*	Returns SD volume cluster count. 
* 
*/
unsigned int SdVolume::clusterCount(void)
{
	return SD_CLUSTER_COUNT;
}

/**
*	Returns SD volume blocks per cluster count. 
* 
*/
unsigned int SdVolume::blocksPerCluster(void)
{
	return SD_BLOCKS_PER_CLUSTER;
}

/**
*	Returns SD volume FAT type. 
*
*/
unsigned int SdVolume::fatType(void)
{
	return SD_FAT_TYPE;
}

/*
* Constructor method for class SdFile.
*
*/
SdFile::SdFile(void)
{

}

/**
* Destructor for class SdFile.
*
*/
SdFile::~SdFile(void)
{

}

/**
*	Opens root directory of Sd volume. 
* 
*	@param vol Sd Volume.
*	@return True if successfull.
*/
uint8_t SdFile::openRoot(SdVolume* vol)
{
	if (vol->rootPath != NULL)
	{
		vard::logevent(vard::Level::INFO, "SdFile.openRoot called.");
		path = vol->rootPath;
		return true;
	}
	
	vard::logevent(vard::Level::ERR, "SdFile.openRoot called. SdVolume rootPath = NULL.");
	return false;
}

/**
*	Creates a new directory with given name.
* 
*	@param dir Root directory to create new dir in.
*	@param dirname Name of new directory.
*	@return True if successfull.
*/
uint8_t SdFile::makeDir(SdFile* dir, const char* dirname)
{
	if (dir->path != NULL)
	{
		char* path = new char[strlen(dir->path) + strlen(dirname) + 1];
		strcpy_s(path, strlen(dir->path) + strlen(dirname) + 1, dir->path);
		strcpy_s(path + strlen(dir->path), strlen(dir->path) + strlen(dirname) + 1, dirname);
		path[strlen(dir->path) + strlen(dirname)] = '\0';
		
		size_t newsize = strlen(path) + 1;
		wchar_t* wcstring = new wchar_t[newsize];
		size_t convertedChars = 0;
		mbstowcs_s(&convertedChars, wcstring, newsize, path, _TRUNCATE);

		bool result = _wmkdir(wcstring) == 0;
		char buff[128];
		if (result)
		{
			sprintf_s(buff, 128, "SdFile.makeDir called. Dir created. dirname=%s", dirname);
			vard::logevent(vard::Level::INFO, buff);
		}
		else
		{
			sprintf_s(buff, 128, "SdFile.makeDir called. Failed to create new dir. dirname=%s", dirname);
			vard::logevent(vard::Level::ERR, buff);
		}
		return result;
	}

	vard::logevent(vard::Level::ERR, "SdFile.makeDir called. SdFile path = NULL.");
	return false;
}

/**
*	Opens/creates a file with given name.
* 
*	@param dir Root directory of file.
*	@param fname Name of file to open.
*	@param mode Open mode.
*/
uint8_t SdFile::open(SdFile* dir, const char* fname, uint8_t mode)
{
	char buff[64];
	sprintf_s(buff, "SdFile.open called. fname=%s mode=%u", fname, mode);
	vard::logevent(vard::Level::INFO, buff);
	return true;
}

uint8_t SdFile::open(SdFile dir, const char* fname, uint8_t mode)
{
	return open(&dir, fname, mode);
}

/**
*	Writes a single char to the file. 
* 
*	@param c Char to write.
*	@return True if successfull.
*/
uint8_t SdFile::write(char c)
{
	char buff[32];
	sprintf_s(buff, "SdFile.write called. c=%c", c);
	vard::logevent(vard::Level::INFO, buff);
	return true;
}

/**
*	Writes a c string to the file.
*
*	@param str String to write.
*	@return True if successfull.
*/
uint8_t SdFile::write(const char* str)
{
	char buff[128];
	sprintf_s(buff, "SdFile.write called: %s", str);
	vard::logevent(vard::Level::INFO, buff);
	return true;
}

/**
*	Flushes buffer before continuing.
*
*/
void SdFile::sync(void)
{
	vard::logevent(vard::Level::INFO, "SdFile.sync called.");
}

/**
*	Closes the Sd file. 
* 
*/
void SdFile::close(void)
{
	vard::logevent(vard::Level::INFO, "SdFile.close called.");
}