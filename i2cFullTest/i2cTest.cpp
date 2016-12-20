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
#include "Bar30.h"
#include "BME280.h"
#include "BNO055.h"
#include "Periodic.h"


using namespace std;

//Instatiate classes for all devices

BNO055 bno = BNO055();
BME280 atmoSensor;
MS5837 presSensor;

//Date buffer to store deployment date for data logging.
char _date_[50];


// This is the fucntion that will be put into its own thread for reading and logging the IMU data at 10Hz
void *IMU_ReadLog(void*)
{
	//Initialize IMU Thread
	cout << "Beginning IMU Thread..." << endl;
	bno.begin();
	
	//Create 10Hz timing for IMU thread
	struct periodic_info info_imu;
	make_periodic (100000, &info_imu);	

	//Get Temp from IMU and output to cout file...just because
	float temp = bno.getTemp();
	cout << "IMU Temp Sensor Reads " << temp << "deg C" << endl;
	
	//Activate IMU external crystal for best performance
	bno.setExtCrystalUse(true);
	sleep(1);

	while(1)
	{
		//Read and Log IMU data at 10Hz
		bno.readlog();

		//Clock timer wait function for high accuracy function timing
		wait_period (&info_imu);
	}
}

//Function used in main thread to read and log RTK data
void *BME280_ReadLog(void*)
{
	//Initialize IMU Thread
	cout << "Beginning BME280 Thread..." << endl;
	
	//Create 10Hz timing for IMU thread
	struct periodic_info info_atmo;
	make_periodic (1000000, &info_atmo);

	atmoSensor.settings.commInterface = I2C_MODE;
	atmoSensor.settings.I2CAddress = 0x77;
	atmoSensor.settings.runMode = 3; //Normal mode
	atmoSensor.settings.tStandby = 4;
	atmoSensor.settings.filter = 0;
	atmoSensor.settings.pressOverSample = 1;
	atmoSensor.settings.humidOverSample = 1;
	atmoSensor.settings.tempOverSample = 1;

	atmoSensor.begin();
	
	sleep(1);

	while(1)
	{
		//Read and Log IMU data at 10Hz
		atmoSensor.readlog();
		
		//Clock timer wait function for high accuracy function timing
		wait_period (&info_atmo);
	}
}

//Function used in main thread to read and log GPS data
void *BAR30_ReadLog(void*)
{
	
	//Initialize IMU Thread
	cout << "Beginning BAR30 Thread..." << endl;
	
	//Create 10Hz timing for IMU thread
	struct periodic_info info_pres;
	make_periodic (100000, &info_pres);
	presSensor.init();
	presSensor.setFluidDensity(1025); //Sea Water density kg/m^3
	sleep(1);

	while(1)
	{

		//Read and Log IMU data at 10Hz
		presSensor.readlog();
		
		//Clock timer wait function for high accuracy function timing
		wait_period (&info_pres);
	}
	
}


int main()
{
	//Create thread objects (Only IMU is threaded since it is polled)
	pthread_t IMU_Thread;
	pthread_t BAR30_Thread;	
	pthread_t ATMO_Thread;
	
	//Start thread objects
	pthread_create(&IMU_Thread,NULL,&IMU_ReadLog,NULL);
	pthread_create(&BAR30_Thread,NULL,&BAR30_ReadLog,NULL);
	pthread_create(&ATMO_Thread,NULL,&BME280_ReadLog,NULL);


	sleep(5);
	cout << "THREADS CREATED" << endl;

	//Enter Main Program loop that will read in data from streaming devices
	cout << "Main Loop Entered...data logging is underway!" << endl;
	while(1)
	{
		sleep(1);
	}	

	return 0;
}