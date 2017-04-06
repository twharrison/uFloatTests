//SERIAL PORT CPP

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h> 
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <sys/poll.h>
#include "serialPort.h"

serialPort::serialPort(void) {
	//Constructor
}


///***************************************************************************
// Function:open
//
/// Opens serial port
/// 
///
/// @param [in] pointer to character array holding device port name
/// (e.g. S1,S2,..,S5,USB0,USB1,...) -> /dev/tty[S1] only include part in brackets 
/// (S3 on beaglebone is tx only, so don't use)
/// @return 0 if failed, 1 if connected
///***************************************************************************
uint8_t serialPort::openPort(char* port) {

	portName = port;

	fd = open(portName, O_RDWR | O_NOCTTY);
	if (fd > 0)
	{
		return(0);
	}
	else
	{
		printf("Invalid Serial Port");
		perror(portName);
		return(-1);
	}
}

///***************************************************************************
// Function:close
//
/// Closes serial port
/// 
/// @return 0 if failed, 1 if connected
///***************************************************************************
void serialPort::closePort(void) {
	close(fd);
}

///***************************************************************************
// Function:configure
//
/// Configures serial port
/// 
///
/// @param [in] baudrate
/// 
/// 
/// @return 0 if failed, 1 if connected
///***************************************************************************
uint8_t serialPort::configure(int32_t baudrate,char* ser_port) {

	//Serial port setting structure
	struct termios oldtio, newtio;

	bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */

	/*  BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
	CRTSCTS : output hardware flow control (only used if the cable has
			  all necessary lines. See sect. 7 of Serial-HOWTO)
	CS8     : 8n1 (8bit,no parity,1 stopbit)
	CLOCAL  : local connection, no modem contol
	CREAD   : enable receiving characters */
	newtio.c_cflag = 0;
	newtio.c_cflag |=  (baudrate | CLOCAL | CREAD);
	newtio.c_cflag &= ~CSIZE;
	newtio.c_cflag |= CS8;
	newtio.c_cflag &= ~PARENB;   
	newtio.c_cflag &= ~CRTSCTS;
	newtio.c_cflag &= ~CSTOPB;
	newtio.c_cc[VMIN]  = 1;
	newtio.c_cc[VTIME] = 0;

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
	tcflush(fd, TCIOFLUSH);
	if(tcsetattr(fd,TCSANOW,&newtio) < 0)
	{
		printf("Error Setting %s Serial Port Attributes!\n",ser_port);
		exit(0);
	};
	tcflush(fd, TCIOFLUSH); // clear buffer

	//Set serial port to low-latency (Higher CPU usage)
	char lowlat_cmd[100];
	int tx0 = snprintf(lowlat_cmd,100,"sudo setserial -a %s low_latency",ser_port);
	tx0 = system(lowlat_cmd);
	
	printf("Port settings for %s complete...",ser_port);
	return(0);
}


///***************************************************************************
// Function:read
//
/// Configures serial port
/// 
///
/// @param [in] pointer to buffer, min and max bytes to read(usually equal)
/// 
/// 
/// @return number of bytes read
///***************************************************************************
int serialPort::readPort(char *buffer, uint32_t num_bytes) {

	size_t total_read = 0;
	size_t total_left = num_bytes;
	char* bufptr = buffer;

	// Loop until all bytes read
	while( total_left > 0)
	{
		ssize_t current = read(fd,bufptr,total_left);
		if(current <= 0)
		{
			printf("Port Read Error");
			return(-1);
		}
		else
		{
			total_read += current;
			total_left -= current;
			bufptr += current;
		}
	}
		
	return ((int)total_read);
}

///***************************************************************************
// Function:write
//
/// Configures serial port
/// 
///
/// @param [in] pointer to buffer, number of bytes to write
/// 
/// 
/// @return number of bytes written (0 on failure)
///***************************************************************************
int serialPort::writePort(char *buffer, uint32_t bytes) {

	size_t total_writ = 0;
	size_t total_left = bytes;
	char* bufptr = buffer;

	// Loop until all bytes read
	while( total_left > 0)
	{
		ssize_t current = write(fd,bufptr,total_left);
		if(current < 0)
		{
			perror("Port Write Error");
			return(0);
		}
		else
		{
			total_writ += current;
			total_left -= current;
			bufptr += current;
		}
	}	
	return ((int)total_writ);
}



///***************************************************************************
// Function:flushIn
//
/// Flush serial port 
///
///***************************************************************************
void serialPort::flushIn(void) {

	tcdrain(fd); // make sure all output is sent buffer
	tcflush(fd, TCIFLUSH); // clear buffer

}

///***************************************************************************
// Function:drainOut
//
/// Flush serial port 
///
///***************************************************************************
void serialPort::drainOut(void) {

	tcdrain(fd); // make sure all output is sent buffer
}