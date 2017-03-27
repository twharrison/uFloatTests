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
#include <math.h>
#include <termios.h>
#include "Bar30.h"
#include "BME280.h"
#include "BNO055.h"
#include "Periodic.h"
#include "MotorCont.h"
#include "eqep.h"
#include "PoolTest.h" 

//Define used namespaces
using namespace std;

//Instantiate classes for all devices
BNO055 bno = BNO055();  // IMU
BME280 atmoSensor;		// Internal condition monitor: pressure, temp, relative humidity 
MS5837 presSensor;		// External pressure sensor
Motor_Control smc;      // Motor Controller
eQEP eqep0(eQEP0, eQEP::eQEP_Mode_Absolute);  // Quadrature decoder 

//Date buffer to store deployment date for data logging.
char _date_[50];

//File Pointers
FILE * pCycleFile;
int exitCheck = 0;

///////////      MAIN PROGRAM     ///////////
int main(void)
{
	// Intialize - Start recording data
	//Create sensor thread objects (Only IMU is threaded since it is polled)
	pthread_t IMU_Thread;
	pthread_t BAR30_Thread;
	pthread_t ATMO_Thread;

	//Set encoder period for polling (not 100% necessary, but put in just in case)
	eqep0.set_period(100000000L);

	//Set Motor Gear Ratio
	smc.conv_factor = 1.0/12.0*1.0 / pool_test.gearRatio*1.0 / 48.0;

	//Start thread objects
	pthread_create(&IMU_Thread, NULL, &IMU_ReadLog, NULL);
	pthread_create(&BAR30_Thread, NULL, &BAR30_ReadLog, NULL);
	pthread_create(&ATMO_Thread, NULL, &BME280_ReadLog, NULL);
	sleep(5);
	cout << "THREADS CREATED" << endl;

	// Command motor fully extend, fully retract: get



	// Delay to place in pool
	sleep(pool_test.delay0)

	// Start Yoyo test
	int ct = yoyoTest(pool_test.cycles, pool_test.delay1, pool_test.delay2)
		
	
	// Save Log (maybe everything is directly written and this is redundant)

	// End
}

/////////     MOTOR  FUNCTIONS     /////////////

// YoYo test function
int yoyoTest(int cycles, int delay1, int delay2, double speed)
{
	//Loop through once for each cycle
	for (int i = 0; i<cycles; i++)
	{
		// Command decent
		// Wait K1_seconds
		// Command rise
		// Wait K2_Seconds

		int a = absPosMove(dist, 1, delay1);
		//usleep(delay*1000);
		a = absPosMove(0, 1, delay1);
		//usleep(delay*1000);
	}
	//Make sure speed is set to zero before exiting cycle test
	int n = smc.smcSetTargetSpeed(smc.smc_fd, 0);
	return 0;
}

//Motor Move PID function
int absPosMove(double setpoint, int cycle, int delay)
{
	unsigned int periodTime = 20000;
	double tolPos = 0.0001;
	double kp = 5.0;
	double ki = 0.01;
	double kd = 0.001;
	double dt = periodTime / 1000000.0;

	double sp1 = setpoint;
	double cp = eqep0.get_position(false)*smc.conv_factor;
	double pinit = cp;


	double e0 = sp1 - cp;
	double e1 = 0;
	double P_err = 0;
	double I_err = 0;
	double D_err = 0;
	int32_t u = 0;
	int32_t n = 0;
	uint8_t msg[100];
	char* msgptr = (char*)msg;
	//uint16_t adc0, adc2;

	struct timeval tval_before;
	time_t nowtime;
	struct tm *nowtm;

	struct periodic_info info;
	make_periodic(periodTime, &info);

	int delaycycles = delay / 10;
	int numDelayCycles = 0;

	while (fabs(e0) >= tolPos)
	{
		gettimeofday(&tval_before, NULL);
		char tmbuf[64];
		nowtime = tval_before.tv_sec;
		nowtm = localtime(&nowtime);
		strftime(tmbuf, sizeof(tmbuf), "%Y-%m-%d,%H:%M:%S", nowtm);

		if (exitCheck == 1)
		{
			n = smc.smcSetTargetSpeed(smc.smc_fd, 0);
			return 0;
		}

		cp = eqep0.get_position(false)*smc.conv_factor;

		e1 = e0;
		e0 = sp1 - cp;

		I_err += e0*dt;
		D_err = (e0 - e1) / dt;

		if (fabs(e0) >= tolPos)
		{
			P_err = e0;
			//u = (kp*P_err + kd*D_err + ki*I_err)*30000;
			u = (kp*P_err) * 30000;

			if (u>3200) u = 3200;

			//CHANGE VALUES HERE
			else if (u>0 && u<1400) u = 1400;
			else if (u<0 && u>-1400) u = -1400;


			else if (u<-3200) u = -3200;
			cout << "U = " << u << endl;
			double errorS = fabs(cp - sp1) / fabs(pinit - setpoint) * 100;

			//cout << errorS << flush;

			//if(errorS > 98) u = u/8; 
			//else if(errorS > 96) u = u/4;
			//else if(errorS > 94) u = u/2;


		}
		else
		{
			u = 0;
		}
		n = smc.smcSetTargetSpeed(smc.smc_fd, u);
		if (cycle == 1)
		{
			//adc0 = ads.readADC_SingleEnded(0);
			//adc2 = ads.readADC_SingleEnded(2);
			//double a1, a2;
			//a1 = adc0*0.000125;
			//a2 = adc2*0.000125;
			//int nn = snprintf((char *)msg,100,"%f,%f,%f\n",cp,a1,a2);
			int nn = snprintf((char *)msg, 100, "%s.%06ld,%f\n", tmbuf, tval_before.tv_usec, cp);
			fwrite(msgptr, sizeof(char), nn, pCycleFile);
		}
		//usleep(10000);
		wait_period(&info);
	}

	while (numDelayCycles < delaycycles)
	{

		gettimeofday(&tval_before, NULL);
		char tmbuf[64];
		nowtime = tval_before.tv_sec;
		nowtm = localtime(&nowtime);
		strftime(tmbuf, sizeof(tmbuf), "%Y-%m-%d,%H:%M:%S", nowtm);
		cp = eqep0.get_position(false)*smc.conv_factor;

		//adc0 = ads.readADC_SingleEnded(0);
		//adc2 = ads.readADC_SingleEnded(2);
		//double a1, a2;
		//a1 = adc0*0.000125;
		//a2 = adc2*0.000125;
		//int nn = snprintf((char *)msg,100,"%f,%f,%f\n",cp,a1,a2);

		//fwrite(msgptr,sizeof(char),nn,pCycleFile);


		int nn = snprintf((char *)msg, 100, "%s.%06ld,%f\n", tmbuf, tval_before.tv_usec, cp);
		fwrite(msgptr, sizeof(char), nn, pCycleFile);

		numDelayCycles++;
		wait_period(&info);
	}
	n = smc.smcSetTargetSpeed(smc.smc_fd, 0);
	return 0;
}


/////////     SENSOR FUNCTIONS    ///////////   

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


