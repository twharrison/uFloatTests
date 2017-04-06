/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   main.cpp
 * Author: Trevor
 *
 * Created on April 6, 2017, 9:10 AM
 */

#include <cstdlib>
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
#include "Periodic.h"
#include "MotorCont.h"
#include "eqep.h"
#include "serialPort.h"
using namespace std;

/*
 * 
 */
// Initialize Motor Controller and encoder
Motor_Control smc;              // Motor Controller
eQEP eqep0(eQEP0, eQEP::eQEP_Mode_Absolute);  // Quadrature decoder 

int32_t v = 0;  // Target speed. 
int32_t l = 0;  // Limit status
int lim_flag = 0;
double lim_x = 0;
double half_x = 0;
double x_now = 0;
int  gear_ratio = 264;
int n = 0; // Placeholder. 

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
			//cout << "U = " << u << endl;
			double errorS = fabs(cp-sp1)/fabs(pinit-setpoint)*100;

			//cout << errorS << flush;

			//if(errorS > 98) u = u/8; 
			//else if(errorS > 96) u = u/4;
			//else if(errorS > 94) u = u/2;


		}
		else
		{ u = 0; }			
		n = smc.smcSetTargetSpeed(u);
		if(cycle == 1)
		{
			//adc0 = ads.readADC_SingleEnded(0);
			//adc2 = ads.readADC_SingleEnded(2);
			//double a1, a2;
			//a1 = adc0*0.000125;
			//a2 = adc2*0.000125;
			//int nn = snprintf((char *)msg,100,"%f,%f,%f\n",cp,a1,a2);
			int nn = snprintf((char *)msg,100,"%s.%06ld,%f\n",tmbuf,tval_before.tv_usec,cp);
			//fwrite(msgptr,sizeof(char),nn,pCycleFile);
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
		//fwrite(msgptr,sizeof(char),nn,pCycleFile);

		numDelayCycles++;
		wait_period(&info);
	}
	n = smc.smcSetTargetSpeed(0);
	return 0;
}

int main() {
    
    //Set encoder period for polling (not 100% necessary, but put in just in case)
    eqep0.set_period(100000000L);
    //Set Motor Gear Ratio
    smc.conv_factor = 1.0/12.0*1.0 /gear_ratio*1.0 / 48.0;
    
    // Move Fully In
    x_now = eqep0.get_position(false);
    cout << "Initial position is " << x_now << endl;
    cout << "Moving piston in" << endl;
    
    lim_flag = smc.smcMoveToLimit(-1, 3200);
    if(lim_flag == -1){
        //Set encoder to 0
        eqep0.set_position(0);
        x_now = eqep0.get_position(false);
        cout << "Current position is: " << x_now << endl;
    }
    cout << "Limit flag is " << lim_flag << endl;
    
    // Move Fully Out
    cout << "Moving piston out" << endl;
    lim_flag = smc.smcMoveToLimit(1, 3200);
    if(lim_flag == 1){
        //Get encoder value of outer limit
        x_now = eqep0.get_position(false);
        cout << "Current position is: " << x_now << endl;
    }
    lim_x = x_now;
    cout << "Limit flag is " << lim_flag << endl;
            
    // Calc Center position
    double half_x = lim_x/2;
    cout << "Halfway mark is: " << half_x << endl;
    cout << "Moving piston to half-way" << endl;
    n = absPosMove(half_x*smc.conv_factor,0,0);
 
    
    return 0;
}

