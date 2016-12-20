#include <string.h>
#include <math.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <stdio.h>
#include "Xbee_control.h"


Xbee_control::Xbee_control(void)
{
	begin();
}

uint8_t Xbee_control::crc_calc(uint8_t header[],uint8_t msg[], int h_size, int m_size)
{
	uint8_t _crc = 0;
	for(int i=3;i<h_size;++i)
	{
		_crc += header[i];
	}
	for(int i=0;i<m_size;++i)
	{
		_crc += msg[i];
	}
	_crc = (0xFF) - _crc;
	return(_crc);
}

int8_t Xbee_control::send_msg_to_base(int port, uint8_t msg[], uint8_t m_size)
{
	uint8_t header[17] = {0x7E,0x00,0x00,0x10,0x00,0x00,0x13,0xA2,0x00,0x41,0x46,0x76,0x9B,0xFF,0xFE,0x00,0x00};
	int n_write = 0;
	uint8_t crc = 0;
	int size_packet = (sizeof(header)-3)+m_size;
	header[1] = size_packet >> 8;
	header[2] = size_packet;
	
	crc = crc_calc(header,msg,17,m_size);

	n_write = write(port,header,17);
	n_write += write(port,msg,m_size);
	n_write += write(port,&crc,1);
	
	if(n_write < (17+m_size+1))
	{
		return(-1);  //Return value of -1 means error sending data
	}
	return(0);  //Return value of 1 means all bytes sent correctly	
}

void Xbee_control::begin(void)
{
	printf("XBEE Setup Beginning\n");
	
	struct termios oldtio, newtio;
	// Load the pin configuration
	/* Open modem device for reading and writing and not as controlling tty
	because we don't want to get killed if linenoise sends CTRL-C. */
	xbee_fd = open(XBEE_DEVICE, O_RDWR | O_NOCTTY); // | O_NDELAY | O_NONBLOCK);
	fcntl(xbee_fd, F_SETFL, 0);
	if (xbee_fd < 0) { perror(XBEE_DEVICE); exit(-1); }

	bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */

	/*  BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
	CRTSCTS : output hardware flow control (only used if the cable has
			  all necessary lines. See sect. 7 of Serial-HOWTO)
	CS8     : 8n1 (8bit,no parity,1 stopbit)
	CLOCAL  : local connection, no modem contol
	CREAD   : enable receiving characters */
	newtio.c_cflag = 0;
	newtio.c_cflag |= (BAUDRATE_XBEE | CLOCAL | CREAD );
	newtio.c_cflag &= ~CSIZE;
	newtio.c_cflag |= CS8;
	newtio.c_cflag &= ~PARENB;   
	newtio.c_cflag &= ~CRTSCTS;
	newtio.c_cflag &= ~CSTOPB;
	newtio.c_cc[VMIN]  = 0;
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
	//tcflush(fona_fd, TCIOFLUSH);
	if(tcsetattr(xbee_fd,TCSANOW,&newtio) < 0)
	{
		printf("Error Setting Xbee Serial Port Attributes!");
		exit(0);
	};
	
	/* terminal settings done, now send Xbee initialization commands if needed*/
	
}


