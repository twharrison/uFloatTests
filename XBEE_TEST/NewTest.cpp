#include <stdint.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <iostream>
#include <stdio.h>

#define FONA_DEVICE "/dev/ttyO5"
#define BAUDRATE_Xbee B115200
#define RF_PACK_START 0x7E
#define RF_TX_PACKET  0x10

/* CRC Calculator Function:
	Takes in header array, message string or array, and sizes of both.
	The sizes must be calculated and passed to the function, since array
	size cannot be calculated inside function due to only passing a
	pointer to the array and not the entire array itself. */

uint8_t crc_calc(uint8_t header[],uint8_t msg[], int h_size, int m_size)
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


int main(void){

printf("XBee Test Beginning\n");
	
	struct termios oldtio, newtio;
	// Load the pin configuration
	/* Open modem device for reading and writing and not as controlling tty
	because we don't want to get killed if linenoise sends CTRL-C. */
	int fona_fd = open(FONA_DEVICE, O_RDWR | O_NOCTTY); // | O_NDELAY | O_NONBLOCK);
	fcntl(fona_fd, F_SETFL, 0);
	if (fona_fd < 0) { perror(FONA_DEVICE); exit(-1); }

	bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */

	/*  BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
	CRTSCTS : output hardware flow control (only used if the cable has
			  all necessary lines. See sect. 7 of Serial-HOWTO)
	CS8     : 8n1 (8bit,no parity,1 stopbit)
	CLOCAL  : local connection, no modem contol
	CREAD   : enable receiving characters */
	//cfsetspeed(&newtio,BAUDRATE_Xbee);
	newtio.c_cflag = 0;
	newtio.c_cflag |= (BAUDRATE_Xbee | CLOCAL | CREAD );
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
	//newtio.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OLCUC | OPOST);
	//newtio.c_oflag &= ~OPOST;
	newtio.c_oflag = 0;


	/* now clean the modem line and activate the settings for the port */
	//tcflush(fona_fd, TCIOFLUSH);
	if(tcsetattr(fona_fd,TCSANOW,&newtio) < 0)
	{
		printf("Error Setting FONA Serial Port Attributes!");
		exit(0);
	};
	
	/* terminal settings done, now send FONA initialization commands*/
	
	//sleep(1);
	uint8_t header[17];
	uint8_t crc = 0;
	int bytes_avail = 0;
	int n_write 	= 0;
	int n_read 		= 0;
	memset(header,'\0',17);
	uint8_t msg[] = "Hey there. How's it going PC? This is just a test.";

	header[0] = RF_PACK_START;
	header[1] = 0x00;
	header[2] = 0x00;
	header[3] = RF_TX_PACKET;
	header[4] = 0x00;
	header[5] = 0x00;
	header[6] = 0x13;
	header[7] = 0xA2;
	header[8] = 0x00;
	header[9] = 0x41;
	header[10]= 0x46;
	header[11]= 0x76;
	header[12]= 0x9B;
	header[13]= 0xFF;
	header[14]= 0xFE;
	header[15]= 0x00;
	header[16]= 0x00;

	printf("TEESSST\n");
	//buffer[17]= 0x48;
	//buffer[18]= 0x49;
	//buffer[19]= 0x14;

	int size_pack = (sizeof(header)-3)+sizeof(msg);

	header[1] = size_pack >> 8;
	header[2] = size_pack;

	int header_size = sizeof(header);
	int msg_size = sizeof(msg);

	crc = crc_calc(header,msg,header_size,msg_size);

	n_write = write(fona_fd,header,sizeof(header));
	n_write += write(fona_fd,msg,sizeof(msg));
	n_write += write(fona_fd,&crc,1);

	return 0;
}