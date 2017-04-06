/******************************************************************************
SparkFunBME280.cpp
BME280 Arduino and Teensy Driver
Marshall Taylor @ SparkFun Electronics
May 20, 2015

Edited by Corey Crisp @ University of Washington for use with Beaglebone Linux

Resources:
Development environment specifics:
Beaglebone Linux Environment

Need to install i2c-tools to use:
sudo apt-get install i2c-tools

This code is released under the [MIT License](http://opensource.org/licenses/MIT).
Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact techsupport@sparkfun.com.
Distributed as-is; no warranty is given.
******************************************************************************/
//See SparkFunBME280.h for additional topology notes.

#include "BME280.h"
#include <sys/time.h>
#include <string.h>
#include <iostream>
#include <time.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <limits.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <vector>

//****************************************************************************//
//
//  Settings and configuration
//
//****************************************************************************//



//Constructor -- Specifies default configuration
BME280::BME280( void )
{
	//Construct with these default settings if nothing is specified

	//Select interface mode
	settings.commInterface = I2C_MODE; //Can be I2C_MODE
	//Select address for I2C.  Does nothing for SPI
	settings.I2CAddress = 0x77; //Ignored for SPI_MODE

	settings.runMode = 0;
	settings.tempOverSample = 0;
	settings.pressOverSample = 0;
	settings.humidOverSample = 0;

	adapter_nr = 2;  //Set to correspond with Beaglebone Black i2c bus (bus 2 is P9.19-SCL and P9.20-SDA)

}

/**************************************************************************/
/*!
    @brief  Reads Device
*/
/**************************************************************************/
void BME280::readlog(void)
{
	std::vector<double> data(3);
	gettimeofday(&tval_before_atmo, NULL);

	data = readAll();

	char tmbuf[64];
	nowtime = tval_before_atmo.tv_sec;
	nowtm = localtime(&nowtime);
	strftime(tmbuf, sizeof(tmbuf), "%Y-%m-%d,%H:%M:%S", nowtm);

	char buffer[50];
	memset(buffer,'\0',sizeof(buffer));
	char* bufptr = buffer;
	int txo = snprintf(buffer,150,"%s.%06ld,%.2f,%.2f,%.1f\n"
		,tmbuf,tval_before_atmo.tv_usec,data[0],data[1],data[2]);

	//printf("%s",buffer);
	fwrite(bufptr,sizeof(char),txo,pATMOFile);
	//fflush(pIMUFile);

}



//****************************************************************************//
//
//  Configuration section
//
//  This uses the stored SensorSettings to start the IMU
//  Use statements such as "mySensor.settings.commInterface = SPI_MODE;" to 
//  configure before calling .begin();
//
//****************************************************************************//
uint8_t BME280::begin()
{
	//Check the settings structure values to determine how to setup the device
	uint8_t dataToWrite = 0;  //Temporary variable


	snprintf(filename, 19, "/dev/i2c-%d", adapter_nr);
	file = open(filename, O_RDWR, O_NDELAY);
	if (file < 0) 
	{
		/* ERROR HANDLING; you can check errno to see what went wrong */
		exit(1);
	}

	int addr = settings.I2CAddress; /* The I2C address */
	if (ioctl(file, I2C_SLAVE, addr) < 0) {
		/* ERROR HANDLING; you can check errno to see what went wrong */
		exit(1);
	}

  	time_t t = time(NULL);
  	struct tm tm = *localtime(&t);

  	int filenamess   = snprintf(atmofile,50,"ATMO_Log_%d-%02d-%02d_%02d%02d%02d.txt", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
  	int filenamesize = snprintf(atmofilename,100,"/root/uFloatTests/i2cFullTest/logs/ATMO_Log_%d-%02d-%02d_%02d%02d%02d.txt", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

  	pATMOFile = fopen(atmofilename,"a");
  	if(pATMOFile == NULL)
  	{
	  	freopen(atmofilename,"a",pATMOFile);
  	}

	//Reading all compensation data, range 0x88:A1, 0xE1:E7
	
	calibration.dig_T1 = ((uint16_t)((readRegister(BME280_DIG_T1_MSB_REG) << 8) + readRegister(BME280_DIG_T1_LSB_REG)));
	calibration.dig_T2 = ((int16_t)((readRegister(BME280_DIG_T2_MSB_REG) << 8) + readRegister(BME280_DIG_T2_LSB_REG)));
	calibration.dig_T3 = ((int16_t)((readRegister(BME280_DIG_T3_MSB_REG) << 8) + readRegister(BME280_DIG_T3_LSB_REG)));

	calibration.dig_P1 = ((uint16_t)((readRegister(BME280_DIG_P1_MSB_REG) << 8) + readRegister(BME280_DIG_P1_LSB_REG)));
	calibration.dig_P2 = ((int16_t)((readRegister(BME280_DIG_P2_MSB_REG) << 8) + readRegister(BME280_DIG_P2_LSB_REG)));
	calibration.dig_P3 = ((int16_t)((readRegister(BME280_DIG_P3_MSB_REG) << 8) + readRegister(BME280_DIG_P3_LSB_REG)));
	calibration.dig_P4 = ((int16_t)((readRegister(BME280_DIG_P4_MSB_REG) << 8) + readRegister(BME280_DIG_P4_LSB_REG)));
	calibration.dig_P5 = ((int16_t)((readRegister(BME280_DIG_P5_MSB_REG) << 8) + readRegister(BME280_DIG_P5_LSB_REG)));
	calibration.dig_P6 = ((int16_t)((readRegister(BME280_DIG_P6_MSB_REG) << 8) + readRegister(BME280_DIG_P6_LSB_REG)));
	calibration.dig_P7 = ((int16_t)((readRegister(BME280_DIG_P7_MSB_REG) << 8) + readRegister(BME280_DIG_P7_LSB_REG)));
	calibration.dig_P8 = ((int16_t)((readRegister(BME280_DIG_P8_MSB_REG) << 8) + readRegister(BME280_DIG_P8_LSB_REG)));
	calibration.dig_P9 = ((int16_t)((readRegister(BME280_DIG_P9_MSB_REG) << 8) + readRegister(BME280_DIG_P9_LSB_REG)));

	calibration.dig_H1 = ((uint8_t)(readRegister(BME280_DIG_H1_REG)));
	calibration.dig_H2 = ((int16_t)((readRegister(BME280_DIG_H2_MSB_REG) << 8) + readRegister(BME280_DIG_H2_LSB_REG)));
	calibration.dig_H3 = ((uint8_t)(readRegister(BME280_DIG_H3_REG)));
	calibration.dig_H4 = ((int16_t)((readRegister(BME280_DIG_H4_MSB_REG) << 4) + (readRegister(BME280_DIG_H4_LSB_REG) & 0x0F)));
	calibration.dig_H5 = ((int16_t)((readRegister(BME280_DIG_H5_MSB_REG) << 4) + ((readRegister(BME280_DIG_H4_LSB_REG) >> 4) & 0x0F)));
	calibration.dig_H6 = ((uint8_t)readRegister(BME280_DIG_H6_REG));

	//Set the oversampling control words.
	//config will only be writeable in sleep mode, so first insure that.
	writeRegister(BME280_CTRL_MEAS_REG, 0x00);
	
	//Set the config word
	dataToWrite = (settings.tStandby << 0x5) & 0xE0;
	dataToWrite |= (settings.filter << 0x02) & 0x1C;
	writeRegister(BME280_CONFIG_REG, dataToWrite);
	
	//Set ctrl_hum first, then ctrl_meas to activate ctrl_hum
	dataToWrite = settings.humidOverSample & 0x07; //all other bits can be ignored
	writeRegister(BME280_CTRL_HUMIDITY_REG, dataToWrite);
	
	//set ctrl_meas
	//First, set temp oversampling
	dataToWrite = (settings.tempOverSample << 0x5) & 0xE0;
	//Next, pressure oversampling
	dataToWrite |= (settings.pressOverSample << 0x02) & 0x1C;
	//Last, set mode
	dataToWrite |= (settings.runMode) & 0x03;
	//Load the byte
	writeRegister(BME280_CTRL_MEAS_REG, dataToWrite);
	
	return readRegister(0xD0);
}

//Strictly resets.  Run .begin() afterwards
void BME280::reset( void )
{
	writeRegister(BME280_RST_REG, 0xB6);
	
}

//****************************************************************************//
//
//  Conversions Section
//
//****************************************************************************//
float BME280::TempF( float tempC )
{
	float output = tempC;
	output = (output * 9) / 5 + 32;

	return output;
}

//****************************************************************************//
//
//  One Shot read ALL
//
//****************************************************************************//

std::vector<double> BME280::readAll(void)
{

	std::vector<double> v(3);

	
	uint8_t buffer[8];
	i2c_smbus_read_i2c_block_data(file,BME280_PRESSURE_MSB_REG,8, buffer);

	//GET TEMP

	// Returns temperature in DegC, resolution is 0.01 DegC. Output value of 5123 equals 51.23 DegC.
	// t_fine carries fine temperature as global value
	//get the reading (adc_T);
	int32_t adc_T = ((uint32_t)buffer[3] << 12) | ((uint32_t)buffer[4] << 4) | ((buffer[5] >> 4) & 0x0F);

	//By datasheet, calibrate
	int64_t var1, var2;

	var1 = ((((adc_T>>3) - ((int32_t)calibration.dig_T1<<1))) * ((int32_t)calibration.dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)calibration.dig_T1)) * ((adc_T>>4) - ((int32_t)calibration.dig_T1))) >> 12) *	((int32_t)calibration.dig_T3)) >> 14;
	t_fine = var1 + var2;
	double output = (t_fine * 5 + 128) >> 8;

	v[0] = output / 100;

	//GET HUMIDITY

	// Returns humidity in %RH as unsigned 32 bit integer in Q22. 10 format (22 integer and 10 fractional bits).
	// Output value of 47445 represents 47445/1024 = 46. 333 %RH
	int32_t adc_H = ((uint32_t)buffer[6] << 8) | ((uint32_t)buffer[7]);
	
	int32_t var3;
	var3 = (t_fine - ((int32_t)76800));
	var3 = (((((adc_H << 14) - (((int32_t)calibration.dig_H4) << 20) - (((int32_t)calibration.dig_H5) * var3)) +
	((int32_t)16384)) >> 15) * (((((((var3 * ((int32_t)calibration.dig_H6)) >> 10) * (((var3 * ((int32_t)calibration.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
	((int32_t)calibration.dig_H2) + 8192) >> 14));
	var3 = (var3 - (((((var3 >> 15) * (var3 >> 15)) >> 7) * ((int32_t)calibration.dig_H1)) >> 4));
	var3 = (var3 < 0 ? 0 : var3);
	var3 = (var3 > 419430400 ? 419430400 : var3);

	v[1] = (double)(var3>>12) / 1024.0;

	//GET PRESSURE

	// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
	// Output value of 24674867ùrepresents 24674867/256 = 96386.2 Pa = 963.862 hPa
	int32_t adc_P = ((uint32_t)buffer[0] << 12) | ((uint32_t)buffer[1] << 4) | ((buffer[2] >> 4) & 0x0F);
	
	int64_t var4, var5, p_acc;
	var4 = ((int64_t)t_fine) - 128000;
	var5 = var4 * var4 * (int64_t)calibration.dig_P6;
	var5 = var5 + ((var4 * (int64_t)calibration.dig_P5)<<17);
	var5 = var5 + (((int64_t)calibration.dig_P4)<<35);
	var4 = ((var4 * var4 * (int64_t)calibration.dig_P3)>>8) + ((var4 * (int64_t)calibration.dig_P2)<<12);
	var4 = (((((int64_t)1)<<47)+var4))*((int64_t)calibration.dig_P1)>>33;
	if (var4 == 0)
	{
		v[2] = 0; // avoid exception caused by division by zero
		return v;
	}
	p_acc = 1048576 - adc_P;
	p_acc = (((p_acc<<31) - var5)*3125)/var4;
	var4 = (((int64_t)calibration.dig_P9) * (p_acc>>13) * (p_acc>>13)) >> 25;
	var5 = (((int64_t)calibration.dig_P8) * p_acc) >> 19;
	p_acc = ((p_acc + var4 + var5) >> 8) + (((int64_t)calibration.dig_P7)<<4);
	
	v[2] = (double)p_acc / 256.0;
	
	return v;
}



//****************************************************************************//
//
//  Utility
//
//****************************************************************************//
uint8_t BME280::readRegister(uint8_t offset)
{
	//Return value
	uint8_t result;

	result = i2c_smbus_read_byte_data(file,offset); // receive a byte as a proper uint8_t

	return result;
}


void BME280::writeRegister(uint8_t offset, uint8_t dataToWrite)
{

	i2c_smbus_write_byte_data(file,offset,dataToWrite);

}