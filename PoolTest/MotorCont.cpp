#include <fcntl.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include "MotorCont.h"


Motor_Control::Motor_Control(void)
{
	conv_factor = 1;
	smcBegin();
}

int Motor_Control::smcBegin(void)
{
	printf("\nMotor Controller Initializing...\n\n");
	
	struct termios oldtio, newtio;
	// Load the pin configuration
	/* Open modem device for reading and writing and not as controlling tty
	because we don't want to get killed if linenoise sends CTRL-C. */
	smc_fd = open(SMC_DEVICE, O_RDWR | O_NOCTTY);
	//fcntl(fona_fd, F_SETFL, 0);
	if (smc_fd < 0) { perror(SMC_DEVICE); exit(-1); }

	bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */

	/*  BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
	CRTSCTS : output hardware flow control (only used if the cable has
			  all necessary lines. See sect. 7 of Serial-HOWTO)
	CS8     : 8n1 (8bit,no parity,1 stopbit)
	CLOCAL  : local connection, no modem contol
	CREAD   : enable receiving characters */
	newtio.c_cflag = 0;
	newtio.c_cflag |= (BAUDRATE_SMC | CLOCAL | CREAD );
	newtio.c_cflag &= ~CSIZE;
	newtio.c_cflag |= CS8;
	newtio.c_cflag &= ~PARENB;   
	newtio.c_cflag &= ~CRTSCTS;
	newtio.c_cflag &= ~CSTOPB;
	newtio.c_cc[VMIN]  = 1;
	newtio.c_cc[VTIME] = 1;

	/* setup for non-canonical mode */
	//newtio.c_iflag &= ~(IGNBRK | BRKINT | IGNPAR | ISTRIP | INLCR | INPCK | ICRNL | IXON | IGNCR | IXOFF | IXANY);  
	newtio.c_iflag = 0;	

	/* Set line flags */ 
	//newtio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	//newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);	
	newtio.c_lflag = 0;

	/*  Raw output */
	//newtio.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
	//newtio.c_oflag &= ~OPOST;
	newtio.c_oflag = 0;

	/* now clean the modem line and activate the settings for the port */
	//tcflush(smc_fd, TCIOFLUSH);
	if(tcsetattr(smc_fd,TCSANOW,&newtio) < 0)
	{
		printf("Error Setting SMC Serial Port Attributes!");
		exit(0);
	};
	
	/* terminal settings done, now send SMC initialization commands */
	
	char cmd = 0xAA;
	int n = write(smc_fd,&cmd,1);
	usleep(50000);
	n = write(smc_fd,&cmd,1);
	usleep(50000);
	int baudrate = 72000000/smcGetVariable(smc_fd,BAUDRATE_REGISTER);
	printf("\nSMC Baudrate = %i\n\n",baudrate);	

	return 0;
	
}

// Reads a variable from the SMC and returns it as number between 0 and 65535.
// Returns SERIAL_ERROR if there was an error.
// The 'variableId' argument must be one of IDs listed in the
// "Controller Variables" section of the user's guide.
// For variables that are actually signed, additional processing is required
// (see smcGetTargetSpeed for an example).
int Motor_Control::smcGetVariable(int fd, unsigned char variableId)
{
	unsigned char command[] = {0xA1, variableId};
	if(write(fd, command, sizeof(command)) == -1)
	{
		perror("error writing");
		return SERIAL_ERROR;
	}
	unsigned char response[2];
	if(read(fd,response,2) != 2)
	{
		perror("error reading");
		return SERIAL_ERROR;
	}
	return (response[0] | (response[1] << 8));
}

// Returns the target speed (-3200 to 3200).
// Returns SERIAL_ERROR if there is an error.
signed short Motor_Control::smcGetTargetSpeed(int fd)
{
	int val = smcGetVariable(fd, TARGET_SPEED);
	return val == SERIAL_ERROR ? SERIAL_ERROR : (signed short)val;
}

// Returns a number where each bit represents a different error, and the
// bit is 1 if the error is currently active.
// See the user's guide for definitions of the different error bits.
// Returns SERIAL_ERROR if there is an error.
int Motor_Control::smcGetErrorStatus(int fd)
{
	return smcGetVariable(fd,ERROR_STATUS);
}

// Sends the Exit Safe Start command, which is required to drive the motor.
// Returns 0 if successful, SERIAL_ERROR if there was an error sending.
int Motor_Control::smcExitSafeStart(int fd)
{
	const unsigned char command = 0x83;

	if (write(fd, &command, 1) == -1)
	{
		perror("error writing");
		return SERIAL_ERROR;
	}
	return 0;
}

// Sets the SMC's target speed (-3200 to 3200).
// Returns 0 if successful, SERIAL_ERROR if there was an error sending.
int Motor_Control::smcSetTargetSpeed(int fd, int speed)
{
	unsigned char command[3];
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
	if (write(fd, command, sizeof(command)) == -1)
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
