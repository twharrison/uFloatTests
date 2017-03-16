#include <string.h>
#include <math.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <signal.h>
#include "MotorCont.h"
#include "eqep.h"
#include "Periodic.h"
#include <sys/time.h>
#include <sys/timerfd.h>


//Create Class Instances
Motor_Control smc;
eQEP eqep0(eQEP0, eQEP::eQEP_Mode_Absolute);
//Adafruit_ADS1115 ads;

//Define used namespaces
using namespace std;

//File Pointers
FILE * pCycleFile;
int exitCheck = 0;


//Exit handler function for exiting program "gracefully"
void my_exit_handler(int s)
{
	//STOP MOTOR	
	smc.smcSetTargetSpeed(smc.smc_fd,0);

	exitCheck = 1;

	//Flush any log file data from buffers to files
	fflush(pCycleFile);
	
}


//Motor Move PID function
int absPosMove(double setpoint, int cycle, int delay)
{	
	unsigned int periodTime = 20000;
	double tolPos = 0.0001;
	double kp = 5.0;
	double ki = 0.01;
	double kd = 0.001;
	double dt = periodTime/1000000.0;

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
	char* msgptr = (char* )msg;
	//uint16_t adc0, adc2;

	struct timeval tval_before;
	time_t nowtime;
	struct tm *nowtm;

	struct periodic_info info;
	make_periodic(periodTime, &info);

	int delaycycles = delay/10;
	int numDelayCycles = 0;

	while(fabs(e0) >= tolPos)  
	{
		gettimeofday(&tval_before, NULL);
		char tmbuf[64];
		nowtime = tval_before.tv_sec;
		nowtm = localtime(&nowtime);
		strftime(tmbuf, sizeof(tmbuf), "%Y-%m-%d,%H:%M:%S", nowtm);

		if(exitCheck == 1)
		{
			n = smc.smcSetTargetSpeed(smc.smc_fd,0);
			return 0;
		}

		cp = eqep0.get_position(false)*smc.conv_factor;

		e1 = e0;
		e0 = sp1 - cp;

		I_err += e0*dt;
		D_err = (e0 - e1)/dt;
		
		if (fabs(e0) >= tolPos)
		{
			P_err = e0;
			//u = (kp*P_err + kd*D_err + ki*I_err)*30000;
			u = (kp*P_err)*30000;
			
			if(u>3200) u=3200;
			
			//CHANGE VALUES HERE
			else if(u>0 && u<1400) u = 1400;
			else if(u<0 && u>-1400) u = -1400;


			else if(u<-3200) u=-3200;
			cout << "U = " << u << endl;
			double errorS = fabs(cp-sp1)/fabs(pinit-setpoint)*100;

			//cout << errorS << flush;

			//if(errorS > 98) u = u/8; 
			//else if(errorS > 96) u = u/4;
			//else if(errorS > 94) u = u/2;


		}
		else
		{ u = 0; }			
		n = smc.smcSetTargetSpeed(smc.smc_fd,u);
		if(cycle == 1)
		{
			//adc0 = ads.readADC_SingleEnded(0);
			//adc2 = ads.readADC_SingleEnded(2);
			//double a1, a2;
			//a1 = adc0*0.000125;
			//a2 = adc2*0.000125;
			//int nn = snprintf((char *)msg,100,"%f,%f,%f\n",cp,a1,a2);
			int nn = snprintf((char *)msg,100,"%s.%06ld,%f\n",tmbuf,tval_before.tv_usec,cp);
			fwrite(msgptr,sizeof(char),nn,pCycleFile);
		}
		//usleep(10000);
		wait_period(&info);
	}

	while(numDelayCycles < delaycycles)
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


		int nn = snprintf((char *)msg,100,"%s.%06ld,%f\n",tmbuf,tval_before.tv_usec,cp);
		fwrite(msgptr,sizeof(char),nn,pCycleFile);

		numDelayCycles++;
		wait_period(&info);
	}
	n = smc.smcSetTargetSpeed(smc.smc_fd,0);
	return 0;
}

//Cycle test function
int cycleTest(double dist, int cycles, int delay, double speed)
{
	//Loop through once for each cycle
	for(int i=0;i<cycles;i++)
	{
		int a = absPosMove(dist,1,delay);
		//usleep(delay*1000);
		a = absPosMove(0,1,delay);
		//usleep(delay*1000);
	}
	//Make sure speed is set to zero before exiting cycle test
	int n = smc.smcSetTargetSpeed(smc.smc_fd,0);
	return 0;
}


//Main function
int main(void)
{
	// Set up exit handler function for emergency stop
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = my_exit_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);

	//Set encoder period for polling (not 100% necessary, but put in just in case)
	eqep0.set_period(100000000L);

	//Get Motor Gear Ratio for testing different motors
	system("clear");
	
	cout << "What is the Current Motor's gear ration X:1? (Only enter integer for X)..." << flush;
	double gearRatio;
	cin >> gearRatio;

	smc.conv_factor = 1.0/12.0*1.0/gearRatio*1.0/48.0;

	cout << "Conversion factor for current motor is " << smc.conv_factor << endl << endl;
	sleep(5);

		

	//Infinite while loop to run motor tests
	while(1)
	{
		exitCheck = 0;	  //Set exitCheck to 0
		system("clear");  //Clears screen after each "test" so screen doesn't clutter

		cout << endl << endl <<"uFloat Motor Test Program..."  << endl << endl;
		cout << "Test Options...enter number corresponding to desired" << endl;
		cout << "test option and then press the Enter key!" << endl << endl;

		cout << "[0]  Cycle Test           :: Run cycle test" << endl;
		cout << "[1]  Positive Small Jog   :: Move 0.1 inch up" << endl;
		cout << "[2]  Negative Small Jog   :: Move 0.1 inch down" << endl;
		cout << "[3]  Positive Medium Jog  :: Move 1.0 inch up" << endl;
		cout << "[4]  Negative Medium Jog  :: Move 1.0 inch down" << endl;
		cout << "[5]  Positive Large Jog   :: Move 2.0 inch up" << endl;
		cout << "[6]  Negative Large Jog   :: Move 2.0 inch down" << endl;
		cout << "[7]  Set Zero Position    :: Make current position the zero point" << endl;
		cout << "[8]  Move To Position     :: Move to absolute position" << endl;
		cout << "[9]  Get Current Position :: Absolute Position" << endl;
		cout << "[10] Exit Cycle Test      :: Close Program" << endl << endl;
		
		cout << "Enter Test Options: ";		

		int testcase;
		cin >> testcase;
		
		//Switch statement to execute desired test case
		switch(testcase)
		{
			case 0:
			{
				double dist, motspeed;
				int cycles, msdelay;
				
				//Get Time for file name
				time_t t = time(NULL);
				struct tm tm = *localtime(&t);

				//Create log file for cycle
				char cyclefilename[150];
				int filenamesize = snprintf(cyclefilename,150,"/root/uFloatTests/MotorTest/Logs/Cycle_Log_%d-%02d-%02d_%02d%02d%02d.txt", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
				pCycleFile = fopen(cyclefilename,"a");
				if(pCycleFile == NULL)
				{
					freopen(cyclefilename,"a",pCycleFile);
				}				

				//Collect test parameters from user
				cout << "Enter cylce test parameters..." << endl << endl;
				cout << "Enter desired Cycle Move distance in decimal inches: ";
				cin >> dist;
				cout << "Enter desired number of whole cycles: ";
				cin >> cycles;
				cout << "Enter desired delay time in integer milliseconds: ";
				cin >> msdelay;
				cout << "Enter desired motor speed in decimal RPMs: ";
				cin >> motspeed;
				cout << "Commencing cycle test..." << flush;

				//Run Cycle test
				int ct = cycleTest(dist,cycles,msdelay,motspeed);
				cout << "Cycle test complete!" << endl;
				fflush(pCycleFile);  //Flush log file to disk
				fclose(pCycleFile);  //Close current log file
				break;
			}
			case 1:
			{
				cout << "Jogging 0.1 inches forwards..." << flush;
				//sleep(1);
				double sp = eqep0.get_position(false)*smc.conv_factor + 0.1;
				int n = absPosMove(sp,0,0);
				cout << "Jogging complete!" << endl;
				break;
			}
			case 2:
			{
				cout << "Jogging 0.1 inches backwards..." << flush;
				//sleep(1);
				double sp = eqep0.get_position(false)*smc.conv_factor - 0.1;
				int n = absPosMove(sp,0,0);
				cout << "Jogging complete!" << endl;
				break;
			}
			case 3:
			{
				cout << "Jogging 1.0 inches forwards..." << flush;
				//sleep(1);
				double sp = eqep0.get_position(false)*smc.conv_factor + 1.0;
				int n = absPosMove(sp,0,0);
				cout << "Jogging complete!" << endl;
				break;
			}
			case 4:
			{
				cout << "Jogging 1.0 inches backwards..." << flush;
				//sleep(1);
				double sp = eqep0.get_position(false)*smc.conv_factor - 1.0;
				int n = absPosMove(sp,0,0);
				cout << "Jogging complete!" << endl;
				break;
			}
			case 5:
			{
				cout << "Jogging 2.0 inches forwards..." << flush;
				//sleep(1);
				double sp = eqep0.get_position(false)*smc.conv_factor + 2.0;
				int n = absPosMove(sp,0,0);
				cout << "Jogging complete!" << endl;
				break;
			}
			case 6:
			{
				cout << "Jogging 2.0 inches backwards..." << flush;
				//sleep(1);
				double sp = eqep0.get_position(false)*smc.conv_factor - 2.0;
				int n = absPosMove(sp,0,0);
				cout << "Jogging complete!" << endl;
				break;
			}
			case 7:
			{
				cout << "Setting current position to be zero point..." << flush;
				//sleep(1);
				eqep0.set_position(0);
				cout << "Position zeroed!" << endl;
				break;
			}
			case 8:
			{
				double sp;
				cout << "Enter desired absolute position in inches..." << flush;
				cin >> sp;
				cout << "Moving to " << sp << "inches ...";
				//sleep(1);
				int n = absPosMove(sp,0,0);
				cout << "Move complete!" << endl;
				break;
			}
			case 9:
			{
				double cp = eqep0.get_position(false)*smc.conv_factor;
				cout << "Current Absolute Position is: " << cp << " inches" << endl;
				sleep(2);
				break;
			}
			case 10:
			{
				smc.smcSetTargetSpeed(smc.smc_fd,0);
				exit(1);
				break;
			}			
		}
		smc.smcSetTargetSpeed(smc.smc_fd,0);  //Make sure motor speed is 0
		sleep(1); //Sleep for 2 seconds before going back to beginning of while loop
	}
	
	//Exit program and return 0
	return(0);
}




