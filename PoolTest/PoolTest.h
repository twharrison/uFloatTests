struct testConfig
{
	long start_delay;    //[sec] Delay before starting yoyo cycles
	long surface_delay;  //[sec] Delay at bottom of yoyo cycles
	long ground_delay;   //[sec] Delay at top of yoyo cycles
	int num_cycles;     //[] Number of cycles to run
	double gear_ratio;  //[] Gear ratio for of the gearbox used 1/[N]
	double speed;       // Generic max speed for motor. 
        int pct_pos;        // Percent of piston throw for positive buoyancy
        int pct_neg;        // Percent of piston throw for negative buoyancy
	// Eventually add depth profiles here. 
};
#ifndef PoolTest_h_included
// All function declarations
void loadConfig(testConfig&);
void *IMU_ReadLog(void*);
void *BAR30_ReadLog(void*);
void *BME280_ReadLog(void*);
//int absPosMove(double setpoint, int cycle, int delay);
//int yoyoTest(int cycles, int delay1, int delay2)

#endif


