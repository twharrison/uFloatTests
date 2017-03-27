#ifndef _pool_test_H
#define _pool_test_H

typedef struct pool_test
{
	int delay0 60000;		//[msec] Delay before starting yoyo cycles
	int delay1 10000;   	//[msec] Delay at bottom of yoyo cycles
	int delay2 10000;       //[msec] Delay at top of yoyo cycles
	int cycles 10000;       //[] Number of cycles to run
	double gear_ratio 264;  //[] Gear ratio for of the gearbox used 1/[N]
	double speed;			// Generic max speed for motor. 
	// Eventually add depth profiles here. 
};
	
