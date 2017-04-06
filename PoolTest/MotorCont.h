#ifndef _SMC_control_H
#define _SMC_control_H
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "serialPort.h"

#define SMC_DEVICE "/dev/ttyS2" //Beaglebone Black serial port
//#define _POSIX_SOURCE 1 /* POSIX compliant source */


#define BAUDRATE_SMC B115200   // Change as needed, keep B
#define SERIAL_ERROR -9999

class Motor_Control 
{
	public:

		unsigned char	ERROR_STATUS 			= 0;
		unsigned char	ERRORS_OCCURRED			= 1;
		unsigned char	SERIAL_ERRORS_OCCURRED		= 2;
		unsigned char	LIMIT_STATUS			= 3;
		unsigned char	RESET_FLAGS			= 127;
		unsigned char	AN1_RAW_UNLIMITED		= 12;
		unsigned char	AN1_RAW				= 13;
		unsigned char	AN1_SCALED			= 14;
		unsigned char	AN2_RAW_UNLIMITED		= 16;
		unsigned char	AN2_RAW				= 17;
		unsigned char	AN2_SCALED			= 18;
		unsigned char	TARGET_SPEED			= 20;
		unsigned char	SPEED				= 21;
		unsigned char	BRAKE_AMT			= 22;
		unsigned char	INPUT_VOLTAGE			= 23;
		unsigned char	TEMPERATURE			= 24;
		unsigned char	RC_PERIOD			= 26;
		unsigned char	BAUDRATE_REGISTER		= 27;
		unsigned char	SYSTEM_TIME_LOW			= 28;
		unsigned char	SYSTEM_TIME_HIGH		= 29;
		unsigned char	MAX_FORWARD_SPEED		= 30;
		unsigned char	MAX_FORWARD_ACCEL		= 31;
		unsigned char	MAX_FORWARD_DECEL		= 32;
		unsigned char	BRAKE_FORWARD_DURATION		= 33;
		unsigned char	MAX_REVERSE_SPEED		= 36;
		unsigned char	MAX_REVERSE_ACCEL		= 37;
		unsigned char	MAX_REVERSE_DECEL		= 38;
		unsigned char	BRAKE_REVERSE_DURATION		= 39;

   
		Motor_Control(void); // Constructor when using HardwareSerial
		int smcBegin(void);
		int smcGetVariable(unsigned char variableId);
		signed short smcGetTargetSpeed(void);
		int smcGetErrorStatus(void);
		int smcExitSafeStart(void);		
		int smcSetTargetSpeed(int speed);
		int smcGetLimitStatus(void);
		int smcMoveToLimit(int dir, uint32_t speed = 3200);

		//Calculates CRC of message to be sent
		unsigned char getCRC(unsigned char message[], unsigned char length);
	    const unsigned char CRC7_POLY = 0x91;

		int smc_fd;		
		double conv_factor;
		
   
   	private:

	protected:

		serialPort port;
	

};
#endif