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
#include "MotorCont.h"
#include "eqep.h"

Motor_Control smc;

int main(void)
{
eQEP eqep0(eQEP0, eQEP::eQEP_Mode_Absolute);

smc.conv_factor = 0.000006576;

eqep0.set_period(100000000L);

int n = 0;
signed short speednow = 0;

double kp = 1.0;
double ki = 0.0001;
double kd = 0.01;
double dt = 0.1;
double sp1 = 0.5;
double sp2 = 0;
double cp = eqep0.get_position(false)*smc.conv_factor;
double e0 = sp1 - cp;
double e1 = 0;
double P_err = 0;
double I_err = 0;
double D_err = 0;
int u = 0;

while(fabs(e0) >= 0.00005) 
{

	cp = eqep0.get_position(false)*smc.conv_factor;
	
	e1 = e0;
	e0 = sp1 - cp;
	printf("\ne0 = %f\n",e0);
	if (fabs(e0) >= 0.00005)
	{
		P_err = e0;

		printf("P = %f, I = %f, D = %f\n",P_err,I_err,D_err);

		u = (kp*P_err + kd*D_err + ki*I_err)*30000;
		printf("Input Raw = %i\n",u);
		if(u>3200) u=3200;
		if(u>=0 && u<200) u = 200;
		if(u<=0 && u>-200) u = -200;
		if(u<-3200) u=-3200;

		I_err += e0*dt;
		D_err = (e0 - e1)/dt;
	}
	else
	{
		u = 0;
	}
		


	n = smc.smcSetTargetSpeed(smc.smc_fd,u);
	printf("Input = %i\n",u);
	printf("Position = %f\n",cp);
	usleep(10000);
	//system("clear");


}

printf("SP1 Reached!\n\n");
smc.smcSetTargetSpeed(smc.smc_fd,0);
e0 = sp2 - cp;
P_err = 0;
I_err = 0;
D_err = 0;
sleep(5);


while( fabs(e0) >= 0.00005) 
{

	cp = eqep0.get_position(false)*smc.conv_factor;
	
	e1 = e0;
	e0 = sp2 - cp;
	printf("\ne0 = %f\n",e0);
	if (fabs(e0) >= 0.00005)
	{
		P_err = e0;

		printf("P = %f, I = %f, D = %f\n",P_err,I_err,D_err);

		u = (kp*P_err + kd*D_err + ki*I_err)*30000;

		if(u>3200) u=3200;
		if(u>=0 && u<200) u = 200;
		if(u<=0 && u>-200) u = -200;
		if(u<-3200) u=-3200;

		I_err += e0*dt;
		D_err = (e0 - e1)/dt;
	}
	else
	{
		u = 0;
	}

	n = smc.smcSetTargetSpeed(smc.smc_fd,u);
	printf("Input = %i\n",u);
	printf("Position = %f\n",cp);
	
	usleep(10000);
	//system("clear");

}

	smc.smcSetTargetSpeed(smc.smc_fd,0);

	return(0);

}
