#ifndef _pool_test_H
#define _pool_test_H

typedef struct pool_test
{
	int delay0;		//[msec] Delay before starting yoyo cycles
	int delay1;   	//[msec] Delay at bottom of yoyo cycles
	int delay2;       //[msec] Delay at top of yoyo cycles
	int cycles;       //[] Number of cycles to run
	double gear_ratio;  //[] Gear ratio for of the gearbox used 1/[N]
	double speed;			// Generic max speed for motor. 
	// Eventually add depth profiles here. 
};
	
