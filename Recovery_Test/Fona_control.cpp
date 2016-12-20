#include <string.h>
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
#include "Fona_control.h"


Fona_control::Fona_control(void)
{
	begin();
}

void Fona_control::get_gps(void)
{
	printf("Reading GPS\n");
	unsigned char gpsbuff[250];
	memset(gpsbuff,'\0',250);
	//tcflush(fona_fd, TCIOFLUSH);

	int bytes_a = 0;
	int n_write = write(fona_fd,GPS_GET_DATA,sizeof(GPS_GET_DATA)-1);
	int n_read = read(fona_fd,gpsbuff,3);
	int cnt = 0;
	//while(gpsbuff[cnt] != '\r')
	while(strstr((char *)gpsbuff,"OK\r\n") == NULL)
	{
		n_read += read(fona_fd,&gpsbuff[cnt],1);
		cnt++;
	}
	if (n_read >= 70)
	{
		gps_parse((char *)gpsbuff);
	}
	printf("Status: %i\n", fix_status);
	printf("Fix: 	%i\n", fix_mode);
	printf("Year:	%i\n", year);
	printf("Month:	%i\n", month);
	printf("Day:	%i\n", day);
	printf("Hour:	%i\n", hour);
	printf("Minute:	%i\n", minute);
	printf("Secs:	%06.3f\n", seconds);		
	printf("Lat:	%f\n", latitude);
	printf("Long: 	%f\n", longitude);
	

	//printf("Buffer: %s\n",gpsbuff);
	//printf("Bytes read: %i\n",n_read);


	return;
}

void Fona_control::sms_location_send(void)
{
	char readbuff[100];
	memset(readbuff,'\0',100);
	tcflush(fona_fd, TCIOFLUSH);

	char cmd1[] = "AT+CMGS=\"17742121063\"\r";
	char msg[150];
	int n = snprintf(msg,150,"uFloat#1: %i/%02i/%02i-%02i:%02i:%06.3f at %09.6f,%010.6f",year,month,day,hour,minute,seconds,latitude,longitude);
	char ctrl_z = 0x1A;

	//printf("CMD: %s\n",cmd1);
	int n_write = write(fona_fd,cmd1,sizeof(cmd1)-1);
	int n_read = 0;
	int i = 0;
	while(strstr((char *)readbuff,">") == NULL)
	{
		n_read += read(fona_fd,&readbuff[i],1);
		readbuff[i+1] ='\0';
		i++;
	}
	//printf("%s\n",readbuff);

	n_write = write(fona_fd,msg,sizeof(msg)-1);
	n_write = write(fona_fd,&ctrl_z,1);

	/* Sleep for n (10 seems to work well) seconds to allow device 
	to send SMS message. Interrupting the SMS send (or any other command) 
	by sending another command before previous finishes executing
	can cause weird issues, as noted in reference manual */

	usleep(1000);

	i = 0;
	n_read = 0;
	while(strstr((char *)readbuff,"OK\r\n") == NULL && i < 25)
	{
		//printf("i = %i\n",i);
		n_read += read(fona_fd,&readbuff[i],1);
		//printf("N: %i\n",n_read);
		//printf("Buffer: %s\n",readbuff);
		//readbuff[i+1] ='\0';
		i++;
	}
	//printf("Buff: %s\n",readbuff);


	tcflush(fona_fd, TCIOFLUSH);

}

void Fona_control::gps_parse(char* data)
{
	char *p = data;

	//Get Run Status
	p = strchr(p, ':')+2;
	fix_status = atoi(p);

	//Get Fix Mode
	p = strchr(p, ',')+1;
	fix_mode = atoi(p);

	//Get Time
	p = strchr(p, ',')+1;
	unsigned char tempyear[5] = {p[0],p[1],p[2],p[3],'\0'};
	year = atoi((char *)tempyear);
	unsigned char tempmonth[3] = {p[4],p[5],'\0'};
	month = atoi((char *)tempmonth);
	unsigned char tempday[3] = {p[6],p[7],'\0'};
	day = atoi((char *)tempday);
	unsigned char temphour[3] = {p[8],p[9],'\0'};
	hour = atoi((char *)temphour);
	unsigned char tempmin[3] = {p[10],p[11],'\0'};
	minute = atoi((char *)tempmin);
	unsigned char tempsecs[7] = {p[12],p[13],p[14],p[15],p[16],p[17],'\0'};
	seconds = atof((char *)tempsecs);
	
	//Get Lat
	p = strchr(p, ',')+1;
	latitude = atof(p);

	//Get Long
	p = strchr(p, ',')+1;
	longitude = atof(p);

	//Get Sats

	
}


void Fona_control::begin(void)
{
	printf("FONA Beginning\n");
	
	struct termios oldtio, newtio;
	// Load the pin configuration
	/* Open modem device for reading and writing and not as controlling tty
	because we don't want to get killed if linenoise sends CTRL-C. */
	fona_fd = 0;
	while(fona_fd < 1)
	{
		//sleep(1);
		fona_fd = open(FONA_DEVICE, O_RDWR | O_NOCTTY); // | O_NDELAY | O_NONBLOCK);
		if (fona_fd < 0) { perror(FONA_DEVICE);}
		std::cout << fona_fd << std::endl;
	}

	printf("FONA Serial Port Opened!");
	//fcntl(fona_fd, F_SETFL, 0);

	if (fona_fd < 0) { perror(FONA_DEVICE);}

	printf("Setting FONA Serial Port Attributes!");
	bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */

	/*  BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
	CRTSCTS : output hardware flow control (only used if the cable has
			  all necessary lines. See sect. 7 of Serial-HOWTO)
	CS8     : 8n1 (8bit,no parity,1 stopbit)
	CLOCAL  : local connection, no modem contol
	CREAD   : enable receiving characters */
	newtio.c_cflag = 0;
	newtio.c_cflag |= (BAUDRATE_Fona | CLOCAL | CREAD );
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
	//tcflush(fona_fd, TCIOFLUSH);
	if(tcsetattr(fona_fd,TCSANOW,&newtio) < 0)
	{
		printf("Error Setting FONA Serial Port Attributes!");
		exit(0);
	};
	
	/* terminal settings done, now send FONA initialization commands*/
	
	sleep(1);
	unsigned char buffer[50];
	int bytes_avail = 0;
	int n_write 	= 0;
	int n_read 		= 0;
	int cnt 		= 0;
	memset(buffer,'\0',50);
	tcflush(fona_fd, TCIOFLUSH);

	while(strstr((char *)buffer,"OK") == NULL && cnt < 5)
	{
		//std::cout << "Count = " << cnt << std::endl;
		memset(buffer,'\0',50);
		n_write = write(fona_fd,FONA_AT,sizeof(FONA_AT)-1);
		n_read = read(fona_fd,buffer,9);
		cnt++;
	}


	printf("AT Accepted\n");
	tcflush(fona_fd, TCIOFLUSH);
	unsigned char buffer1[50];
	memset(buffer1,'\0',50);

	int n = write(fona_fd,FONA_ECHO_OFF,sizeof(FONA_ECHO_OFF)-1);
	n = 0;
	int i = 0;
	while(strstr((char *)buffer1,"OK\r\n") == NULL)
	{
		n = read(fona_fd,&buffer1[i],1);
		i++;
	}
	memset(buffer1,'\0',50);
	tcflush(fona_fd, TCIOFLUSH);
	
	n = write(fona_fd,GPS_POWER_ON,sizeof(GPS_POWER_ON)-1);
	n = 0;
	i = 0;
	while(strstr((char *)buffer1,"OK\r\n") == NULL)
	{
		n = read(fona_fd,&buffer1[i],1);
		i++;
	}
	memset(buffer1,'\0',50);

	n = write(fona_fd,FONA_SMS_TYPE,sizeof(FONA_SMS_TYPE)-1);
	n = 0;
	i = 0;
	while(strstr((char *)buffer1,"OK\r\n") == NULL)
	{
		n = read(fona_fd,&buffer1[i],1);
		i++;
	}
	memset(buffer1,'\0',50);

	n = write(fona_fd,PMTK_SET_NMEA_5HZ,sizeof(PMTK_SET_NMEA_5HZ)-1);
	n = 0;
	i = 0;
	while(strstr((char *)buffer1,"OK\r\n") == NULL)
	{
		n = read(fona_fd,&buffer1[i],1);
		i++;
	}
	memset(buffer1,'\0',50);

	n = write(fona_fd,PMTK_NMEA_TYPES,sizeof(PMTK_NMEA_TYPES)-1);
	n = 0;
	i = 0;
	while(strstr((char *)buffer1,"OK\r\n") == NULL)
	{
		n = read(fona_fd,&buffer1[i],1);
		i++;
	}
	memset(buffer1,'\0',50);

	n = write(fona_fd,PMTK_SET_WAAS,sizeof(PMTK_SET_WAAS)-1);
	n = 0;
	i = 0;
	while(strstr((char *)buffer1,"OK\r\n") == NULL)
	{
		n = read(fona_fd,&buffer1[i],1);
		i++;
	}
	memset(buffer1,'\0',50);

	n = write(fona_fd,PMTK_SET_SBAS,sizeof(PMTK_SET_SBAS)-1);
	n = 0;
	i = 0;
	while(strstr((char *)buffer1,"OK\r\n") == NULL)
	{
		n = read(fona_fd,&buffer1[i],1);
		i++;
	}
	memset(buffer1,'\0',50);

	printf("FONA Attributes Set!");
	sleep(1);

}


