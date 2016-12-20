#include <stdlib.h>
#include <signal.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <termios.h>
#include <iomanip>
#include <time.h>
#include <sys/time.h>
#include <sys/timerfd.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <cstdlib>
#include <poll.h>
#include "Fona_control.h"
#include "Xbee_control.h"
#include "GPIO.h"

//Class Instantiations
Fona_control fona;
Xbee_control xbee;

#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define MAX_BUF 64

using namespace std;
using namespace exploringBB;

GPIO *outGPIO, *inGPIO;
static int threadCheck = 0;

//Button Press Callback Function
int recoveryCallback(int var) {
	outGPIO->streamWrite(HIGH);
	cout << "Button Pressed!" << endl;
	uint8_t msg[100];
	int n = snprintf((char *)msg,100,"Hi. The GPS Date/Time is: %i/%02i/%02i-%02i:%02i:%.3f",fona.year,fona.month,fona.day,fona.hour,fona.minute,fona.seconds);
	int msg_size = sizeof(msg);
	int check = xbee.send_msg_to_base(xbee.xbee_fd,msg,msg_size);
	fona.sms_location_send();
	outGPIO->streamWrite(LOW);
	return 0;
}

//Periodic Thread handling struct
struct periodic_info
{
	int timer_fd;
	unsigned long long wakeups_missed;
};

//Periodic Thread handling function
static int make_periodic (unsigned int period, struct periodic_info *info)
{
	int ret;
	unsigned int ns;
	unsigned int sec;
	int fd;
	struct itimerspec itval;

	/* Create the timer */
	fd = timerfd_create (CLOCK_MONOTONIC, 0);
	info->wakeups_missed = 0;
	info->timer_fd = fd;
	if (fd == -1)
		return fd;

	/* Make the timer periodic */
	sec = period/1000000;
	ns = (period - (sec * 1000000)) * 1000;
	itval.it_interval.tv_sec = sec;
	itval.it_interval.tv_nsec = ns;
	itval.it_value.tv_sec = sec;
	itval.it_value.tv_nsec = ns;
	ret = timerfd_settime (fd, 0, &itval, NULL);
	return ret;
}

//Periodic Thread waiting function
static void wait_period (struct periodic_info *info)
{
	unsigned long long missed;
	int ret;

	/* Wait for the next timer event. If we have missed any the
	   number is written to "missed" */
	ret = read (info->timer_fd, &missed, sizeof (missed));
	if (ret == -1)
	{
		perror ("read timer");
		return;
	}

	info->wakeups_missed += missed;
}

void *GPS_ReadLog(void*)
{
	//Initialize IMU Thread
	cout << "Beginning GPS Read Thread..." << endl;
	
	//Create 10Hz timing for IMU thread
	struct periodic_info info_gps;
	make_periodic(1000000, &info_gps);	

	while(1)
	{
		if(threadCheck == 0) 
		{
			//Read and Log IMU data at 1Hz
			fona.get_gps();
		}

		//Clock timer wait function for high accuracy function timing
		wait_period (&info_gps);
	}
}



int main()
{
	
	//Should redirect cout and printf statements to log file when all is working as expected
	cout << "UFloat Recovery Test Initialization Beginning" <<  endl;

	//Setup GPIO input and output for LED and Button Press
	inGPIO = new GPIO(74);
	outGPIO = new GPIO(75);
	inGPIO->setDebounceTime(200);
	inGPIO->setDirection(INPUT);
	outGPIO->setDirection(OUTPUT);
	outGPIO->streamOpen();
	outGPIO->streamWrite(LOW);
	inGPIO->setEdgeType(RISING); 

	time_t t = time(NULL);
	struct tm tm = *localtime(&t);
	char gpsfilename[100];
	FILE * pGPSFile;

	int filenamesize = snprintf(gpsfilename,100,"/root/Recovery_Test/Logs/GPS_Log_%d-%02d-%02d_%02d%02d%02d.txt", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
	pGPSFile = fopen(gpsfilename,"a");
	if(pGPSFile == NULL)
	{
		freopen(gpsfilename,"a",pGPSFile);
	}


	//Create thread objects (Only IMU is threaded since it is polled)
	pthread_t gpsThread;
	
	//Start thread objects
	pthread_create(&gpsThread,NULL,&GPS_ReadLog,NULL);

	cout << "THREADS CREATED" << endl;
		
	//One last flush of NS-RAW, GPS, and AIRMAR buffers to ensure garbage data is gone
	tcflush(fona.fona_fd, TCIOFLUSH); // clear buffer
	tcflush(xbee.xbee_fd, TCIOFLUSH); // clear buffer


	//Enter Main Program loop that will read in data from streaming devices
	cout << "Main Loop Entered...data logging is underway!" << endl;
	while(1)
	{   
		//cout << "Waiting for button press..." << endl;
		inGPIO->waitForEdge();
		threadCheck = 1;
		usleep(500000);
		outGPIO->streamWrite(HIGH);
		//cout << "Button Pressed!" << endl;
           	uint8_t msg[150];
		char* msgptr = (char *)msg;
		int n = snprintf((char *)msg,150,"uFloat#1: %i/%02i/%02i-%02i:%02i:%06.3f at %09.6f,%010.6f",fona.year,fona.month,fona.day,fona.hour,fona.minute,fona.seconds,fona.latitude,fona.longitude);
		int msg_size = sizeof(msg);
		int check = xbee.send_msg_to_base(xbee.xbee_fd,(uint8_t *)msg,n);
		fona.sms_location_send();
		msg[n] = '\n';
		fwrite(msgptr,sizeof(char),n+1,pGPSFile);
		
		fflush(pGPSFile);
		outGPIO->streamWrite(LOW);
		threadCheck = 0;
	}	

	return 0;
}