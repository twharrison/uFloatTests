/**************************************************************************/
/*!
    @file     Adafruit_ADS1015.cpp
    @author   K.Townsend (Adafruit Industries)
    @license  BSD (see license.txt)
    Driver for the ADS1015/ADS1115 ADC
    This is a library for the Adafruit MPL115A2 breakout
    ----> https://www.adafruit.com/products/???
    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!
    @section  HISTORY
    v1.0 - First release
*/
/**************************************************************************/
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
#include "ADS1115.h"

int file;
int adapter_nr = 2; /* probably dynamically determined */
char filename[20];

/**************************************************************************/
/*!
    @brief  Instantiates a new ADS1115 class w/appropriate properties
*/
/**************************************************************************/
Adafruit_ADS1115::Adafruit_ADS1115(uint8_t i2cAddress)
{
   m_i2cAddress = i2cAddress;
   m_conversionDelay = ADS1115_CONVERSIONDELAY;
   m_bitShift = 0;
   m_gain = GAIN_ONE; /* +/- 6.144V range (limited to VDD +0.3V max!) */
}

/**************************************************************************/
/*!
    @brief  Sets up the HW (reads coefficients values, etc.)
*/
/**************************************************************************/
void Adafruit_ADS1115::begin() {
  //time_t t = time(NULL);
  //struct tm tm = *localtime(&t);
  //char imufilename[100];


  //int filenamess   = snprintf(imufile,50,"IMU_Log_%d-%02d-%02d_%02d%02d%02d.txt", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
  //int filenamesize = snprintf(imufilename,100,"/root/SWIFT/logs/IMU_Log_%d-%02d-%02d_%02d%02d%02d.txt", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);



  //pIMUFile = fopen(imufilename,"a");
  //if(pIMUFile == NULL)
  //{
	//  freopen(imufilename,"a",pIMUFile);
  //}
  
  
  /* Enable I2C */
  

  snprintf(filename, 19, "/dev/i2c-%d", adapter_nr);
  file = open(filename, O_RDWR, O_NDELAY);
  if (file < 0) 
  {
    /* ERROR HANDLING; you can check errno to see what went wrong */
    exit(1);
  }

  int addr = m_i2cAddress; /* The I2C address */

  if (ioctl(file, I2C_SLAVE, addr) < 0) {
    /* ERROR HANDLING; you can check errno to see what went wrong */
    exit(1);
  }
  
  setGain(GAIN_ONE);
  
}

/**************************************************************************/
/*!
    @brief  Sets the gain and input voltage range
*/
/**************************************************************************/
void Adafruit_ADS1115::setGain(adsGain_t gain)
{
  m_gain = gain;
}

/**************************************************************************/
/*!
    @brief  Gets a gain and input voltage range
*/
/**************************************************************************/
adsGain_t Adafruit_ADS1115::getGain()
{
  return m_gain;
}

/**************************************************************************/
/*!
    @brief  Gets a single-ended ADC reading from the specified channel
*/
/**************************************************************************/
uint16_t Adafruit_ADS1115::readADC_SingleEnded(uint8_t channel) {
  if (channel > 3)
  {
    return 0;
  }
  
  // Start with default values
  uint16_t config = ADS1015_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)
                    ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
                    ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
                    ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
                    ADS1015_REG_CONFIG_DR_MASK      | // 1600 samples per second (default)
                    ADS1015_REG_CONFIG_MODE_SINGLE;   // Single-shot mode (default)

  // Set PGA/voltage range
  config |= m_gain;

  // Set single-ended input channel
  switch (channel)
  {
    case (0):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_0;
      break;
    case (1):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_1;
      break;
    case (2):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_2;
      break;
    case (3):
      config |= ADS1015_REG_CONFIG_MUX_SINGLE_3;
      break;
  }

  // Set 'start single-conversion' bit
  config |= ADS1015_REG_CONFIG_OS_SINGLE;

  // Write config register to the ADC
  i2c_smbus_write_word_data(file, ADS1015_REG_POINTER_CONFIG, config);

  // Wait for the conversion to complete
  usleep(8000);

  // Read the conversion results
  uint8_t buffer[2];
  uint16_t res;
  i2c_smbus_read_i2c_block_data(file, ADS1015_REG_POINTER_CONVERT,2,buffer);

  res = (((uint16_t)buffer[0]) << 8) | ((uint16_t)buffer[1]);

  return res;
}


/**************************************************************************/
/*!
    @brief  In order to clear the comparator, we need to read the
            conversion results.  This function reads the last conversion
            results without changing the config value.
*/
/**************************************************************************/
int16_t Adafruit_ADS1115::getLastConversionResults()
{
  // Wait for the conversion to complete
  usleep(m_conversionDelay*1000);

  // Read the conversion results
  uint8_t buffer[2];
  int16_t res;
  i2c_smbus_read_i2c_block_data(file, ADS1015_REG_POINTER_CONVERT,2,buffer);

  res = (((uint16_t)buffer[0]) << 8) | ((uint16_t)buffer[1]);

  return res;
  
}
