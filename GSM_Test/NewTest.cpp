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

#define FONA_DEVICE "/dev/ttyO1"
#define BAUDRATE_Fona B115200
#define FONA_AT			"AT\r"
#define FONA_ECHO_OFF		"ATE0\r"

int main(void){

printf("FONA Beginning\n");
	
	struct termios oldtio, newtio;
	// Load the pin configuration
	/* Open modem device for reading and writing and not as controlling tty
	because we don't want to get killed if linenoise sends CTRL-C. */
	int fona_fd = open(FONA_DEVICE, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
	fcntl(fona_fd, F_SETFL, 0);
	if (fona_fd < 0) { perror(FONA_DEVICE); exit(-1); }

	bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */

	/*  BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
	CRTSCTS : output hardware flow control (only used if the cable has
			  all necessary lines. See sect. 7 of Serial-HOWTO)
	CS8     : 8n1 (8bit,no parity,1 stopbit)
	CLOCAL  : local connection, no modem contol
	CREAD   : enable receiving characters */
	//cfsetspeed(&newtio,BAUDRATE_Fona);
	newtio.c_cflag = 0;
	newtio.c_cflag |= (BAUDRATE_Fona | CLOCAL | CREAD );
	newtio.c_cflag &= ~CSIZE;
	newtio.c_cflag |= CS8;
	newtio.c_cflag &= ~PARENB;   
	newtio.c_cflag &= ~CRTSCTS;
	newtio.c_cflag &= ~CSTOPB;
	newtio.c_cc[VMIN]  = 0;
	newtio.c_cc[VTIME] = 10;

	/* setup for non-canonical mode */
	newtio.c_iflag &= ~(IGNBRK | BRKINT | IGNPAR | ISTRIP | INLCR | INPCK | ICRNL | IXON | IGNCR | IXOFF | IXANY);  
	//newtio.c_iflag = 0;	

	/* Set line flags */ 
	//newtio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);	

	/*  Raw output */
	//newtio.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OLCUC | OPOST);
	newtio.c_oflag &= ~OPOST;
	

	/* now clean the modem line and activate the settings for the port */
	//tcflush(fona_fd, TCIOFLUSH);
	if(tcsetattr(fona_fd,TCSANOW,&newtio) < 0)
	{
		printf("Error Setting FONA Serial Port Attributes!");
		exit(0);
	};
	
	/* terminal settings done, now send FONA initialization commands*/
	
	//sleep(1);
	unsigned char buffer[50];
	int bytes_avail = 0;
	int n_write 	= 0;
	int n_read 		= 0;
	int cnt 		= 0;
	memset(buffer,'\0',50);
	//tcflush(fona_fd, TCIOFLUSH);
	while(strstr((char *)buffer,"OK") == NULL && cnt < 5)
	{
		memset(buffer,'\0',50);
		n_write = write(fona_fd,FONA_AT,sizeof(FONA_AT)-1);
		//sleep(1);
		//ioctl(fona_fd,FIONREAD,&bytes_avail);
		//printf("BA: %i\n",bytes_avail);
		n_read = read(fona_fd,buffer,6);
		printf("%s\n",buffer);
		
		//sleep(1);
		cnt++;
	}

	memset(buffer,'\0',50);
	n_write = write(fona_fd,FONA_AT,sizeof(FONA_AT)-1);
	//sleep(0.1);
	//ioctl(fona_fd,FIONREAD,&bytes_avail);
	//printf("BA: %i\n",bytes_avail);
	n_read = read(fona_fd,buffer,6);
	printf("%s\n",buffer);
	


	return 0;
}