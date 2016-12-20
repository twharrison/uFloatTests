#ifndef _fona_control_H
#define _fona_control_H

#include <stdint.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#define FONA_DEVICE "/dev/ttyS1" //Beaglebone Black serial port
//#define _POSIX_SOURCE 1 /* POSIX compliant source */

#define BAUDRATE_Fona B115200   // Change as needed, keep B

/* Define FONA AT Commands */
#define FONA_AT			"AT\r"
#define FONA_ECHO_OFF		"ATE0\r"
#define FONA_CMD_REPEAT 	"A/\r"
#define FONA_NO_ECHO      	"ATE0\r"
#define FONA_PIN_CHECK    	"AT+CPIN?\r"
#define FONA_PIN_SEND      	"AT+CPIN=1234\r"
#define FONA_SMS_TYPE      	"AT+CMGF=1\r"

/* Define FONA GPS AT Commands */
#define GPS_POWER_ON   	"AT+CGNSPWR=1\r"
#define GPS_POWER_OFF   "AT+CGNSPWR=0\r"
#define GPS_GET_DATA   	"AT+CGNSINF\r"

/* Define FONA GPS NMEA Commands */
#define PMTK_CMD_HOT_START	"AT+CGNSCMD=0,\"$PMTK101*32\"\r"
#define PMTK_CMD_WARM_START	"AT+CGNSCMD=0,\"$PMTK102*31\"\r"
#define PMTK_CMD_COLD_START 	"AT+CGNSCMD=0,\"$PMTK103*30\"\r"
#define PMTK_SET_NMEA_5HZ   	"AT+CGNSCMD=0,\"$PMTK220,200*2C\"\r"
#define PMTK_SET_BAUD_38400 	"AT+CGNSCMD=0,\"$PMTK251,38400*27\"\r"
#define PMTK_SET_WAAS      	"AT+CGNSCMD=0,\"$PMTK301,2*2E\"\r"
#define PMTK_SET_SBAS      	"AT+CGNSCMD=0,\"$PMTK313,1*2E\"\r"
#define PMTK_NMEA_TYPES      	"AT+CGNSCMD=0,\"$PMTK314,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*29\"\r"
#define PMTK_STANDY_MODE   	"AT+CGNSCMD=0,\"$PMTK161,0*28\"\r"    //Send any byte to exit standby mode


class Fona_control {
   public:
   
	  void begin(void);
	  void get_gps(void);
	  Fona_control(void); // Constructor when using HardwareSerial
	  void gps_parse(char* data);
	  void sms_location_send(void);
	  uint8_t fix_status, fix_mode, sats, sats_used, glo_sats, cn0;
	  uint32_t month, day, minute, hour;
	  uint32_t year;
	  
	  double gpstime, seconds, latitude, longitude, speed, course, hdop, vdop, pdop, hpa, vpa;
	  
	  int fona_fd;
   
   private:


};


#endif