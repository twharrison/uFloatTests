#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include "serialPort.h"
#include "MotorCont.h"


Motor_Control::Motor_Control(void):port() {
	conv_factor = 1;
	smcBegin();
}

int Motor_Control::smcBegin(void)
{
	printf("\nMotor Controller Initializing...\n\n");
	int out = port.openPort((char *)SMC_DEVICE);
	if (out == -1) {

		printf("Error connecting to serial port...is it active? Check 'dmesg | grep tty'\n\n");
		return(-1);
	}
	else {	
		port.configure(BAUDRATE_SMC,(char *)SMC_DEVICE);
	}
	
	char cmd = 0xAA;
	int n = port.writePort(&cmd,1);
	usleep(50000);
	n = port.writePort(&cmd,1);
	usleep(50000);
	int baudrate = 72000000/((uint16_t)smcGetVariable(BAUDRATE_REGISTER));
	printf("\nSMC Baudrate = %i\n\n",baudrate);	

	return 0;	
}

// Reads a variable from the SMC and returns it as number between 0 and 65535.
// Returns SERIAL_ERROR if there was an error.
// The 'variableId' argument must be one of IDs listed in the
// "Controller Variables" section of the user's guide.
// For variables that are actually signed, additional processing is required
// (see smcGetTargetSpeed for an example).
int Motor_Control::smcGetVariable(unsigned char variableId)
{
	char command[] = {0xA1, variableId};
	if(port.writePort(command, sizeof(command)) == -1)
	{
		perror("error writing");
		return SERIAL_ERROR;
	}
	char response[2];
	if(port.readPort(response,2) != 2)
	{
		perror("error reading");
		return SERIAL_ERROR;
	}
	return (response[0] | (response[1] << 8));
}

// Returns the target speed (-3200 to 3200).
// Returns SERIAL_ERROR if there is an error.
signed short Motor_Control::smcGetTargetSpeed(void)
{
	int val = smcGetVariable(TARGET_SPEED);
	return val == SERIAL_ERROR ? SERIAL_ERROR : (signed short)val;
}

// Returns a number where each bit represents a different error, and the
// bit is 1 if the error is currently active.
// See the user's guide for definitions of the different error bits.
// Returns SERIAL_ERROR if there is an error.
int Motor_Control::smcGetErrorStatus(void)
{
	return smcGetVariable(ERROR_STATUS);
}

// Sends the Exit Safe Start command, which is required to drive the motor.
// Returns 0 if successful, SERIAL_ERROR if there was an error sending.
int Motor_Control::smcExitSafeStart(void)
{
	char command = 0x83;

	if (port.writePort(&command, 1) == -1)
	{
		perror("error writing");
		return SERIAL_ERROR;
	}
	return 0;
}

// Sets the SMC's target speed (-3200 to 3200).
// Returns 0 if successful, SERIAL_ERROR if there was an error sending.
int Motor_Control::smcSetTargetSpeed(int speed)
{
	char command[3];
	if (speed < 0)
	{
		command[0] = 0x86; // Motor Reverse
		speed = -speed;
	}
	else
	{
		command[0] = 0x85; // Motor Forward
	}
	command[1] = speed & 0x1F;
	command[2] = speed >> 5;
	if (port.writePort(command, sizeof(command)) == -1)
	{
		perror("error writing");
		return SERIAL_ERROR;
	}
	return 0;
}


//Calculates CRC of message to be sent
unsigned char Motor_Control::getCRC(unsigned char message[], unsigned char length)
{
	unsigned char i, j, crc = 0;
	for (i = 0; i < length; i++)
	{
		crc ^= message[i];
		for (j = 0; j < 8; j++)
		{
			if (crc & 1)
			crc ^= CRC7_POLY;
			crc >>= 1;
		}
	}
	return crc;
}

int Motor_Control::smcMoveToLimit(int dir, uint32_t speed) {

	int limFlag = 0;
	
	int targspd = dir*((int)speed);

	smcSetTargetSpeed(targspd);

	while(1) {
		limFlag = smcGetLimitStatus();
		switch(limFlag) {
			case 1:
			{
                                if(dir == limFlag) {
                                    //printf("motor stopping \n");
                                    //printf("Lim Flag: %i \n", limFlag);
					smcSetTargetSpeed(0);
					return(1);
				}
				break;
			}
			case -1:
			{
				if(dir == limFlag) {
                                    //printf("motor stopping \n");
                                    //printf("Lim Flag: %i \n", limFlag);
					smcSetTargetSpeed(0);
					return(-1);
				}
				break;
			}
			case -2:
			{
				return(-2);
			}
		}
		usleep(50000);
	}
}

int Motor_Control::smcGetLimitStatus(void) {

	uint16_t limFlag = 0;
	limFlag = smcGetVariable(LIMIT_STATUS);
        //printf("Raw Int: %u  ||  Raw Hex: %x \n", limFlag, limFlag);
        //uint16_t tmp = limFlag & 0x80;
        //printf("Lim Flag Condition: %x \n", tmp);
        
	if((limFlag & 0x80)  ==  0x80) {
		//Reverse limit
		return(-1);
	}
	else if((limFlag & 0x100) == 0x100) {
		//Forward limit
		return(+1);
	}
	else if((limFlag & 0x01) == 0x01) {
		//Safe start violation
		return(-2);
	}
	else if((limFlag & 0x02) == 0x02) {
		//Temp reducing target speed
		return(-3);
	}
	else if((limFlag & 0x04) == 0x04) {
		//max speed reducing target speed
		return(-4);
	}	
	else if((limFlag & 0x08) == 0x08) {
		//Motor speed not equal to target because of accel limits
		return(-5);
	}
	else if((limFlag & 0x10) == 0x10) {
		//Motor speed not equal to target because of accel limits
		return(-6);
	}
	else {
		return(0);
	}
}

