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
using namespace std;

//Instantiate classes for all devices
testConfig config;              // Test Configuration
BNO055 bno = BNO055();          // IMU
BME280 atmoSensor;		// Internal condition monitor: pressure, temp, relative humidity 
MS5837 presSensor;		// External pressure sensor
Motor_Control smc;              // Motor Controller
eQEP eqep0(eQEP0, eQEP::eQEP_Mode_Absolute);  // Quadrature decoder 

//Date buffer to store deployment date for data logging.
char _date_[50];


//File Pointers
FILE *pCycleFile;
FILE *pMasterLog; 
int exitCheck = 0;
time_t t = time(NULL);



// Motor variable initializations
int32_t v = 0;  // Target speed. 
int32_t l = 0;  // Limit status
//ouble lim_x = 0;  // Limit Value






/////////     MOTOR  FUNCTIONS     /////////////



//Motor Move PID function
// Delay is in millisec
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
			n = smc.smcSetTargetSpeed(0);
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
			//cout << "U = " << u << endl;
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
		n = smc.smcSetTargetSpeed(u);
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
	n = smc.smcSetTargetSpeed(0);
	return 0;
}

// YoYo test function
int yoyoTest(int cycles, int delay1, double x_pos, double x_neg)
{
	//Loop through once for each cycle
	for (int i = 0; i<cycles; i++)
	{
                
                // Move Down
		int a = absPosMove(x_neg, 1, delay1);
		//usleep(delay*1000);
                
                // Move Up
		a = absPosMove(x_pos, 1, delay1);
		//usleep(delay*1000);
	}
	//Make sure speed is set to zero before exiting cycle test
	int h = smc.smcSetTargetSpeed(0);
	return 0;
}

// Initial limit find
int getMotorLimits(double* plim_x){
    int lim_flag = 0;
    
    // Move Fully In
    double x_now = eqep0.get_position(false);
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
    *plim_x = x_now;
    cout << "Limit flag is " << lim_flag << endl;
            
    /* Calc Center position
    double half_x = lim_x/2;
    cout << "Halfway mark is: " << half_x << endl;
    cout << "Moving piston to half-way" << endl;
    n = absPosMove(half_x*smc.conv_factor,0,0);
     */
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
	make_periodic (100000, &info_atmo);

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

///// CONFIGURATION /////
void loadConfig(testConfig& config) {
    ifstream fin("test_config.txt");
    string line;
    // Get each line
    while (getline(fin, line)) {
        // Find the first substring following delimeter = in each line
        istringstream sin (line.substr(line.find("=") + 1));
        // Find test indicating the setting
        if (line.find("gear_ratio") != -1)
            sin >> config.gear_ratio;
        else if (line.find("num_cycles") != -1)
            sin >> config.num_cycles;
        else if (line.find("start_delay") != -1)
            sin >> config.start_delay;
        else if (line.find("surface_delay") != -1)
            sin >> config.surface_delay;
        else if (line.find("ground_delay") != -1)
            sin >> config.ground_delay;
        else if (line.find("pct_pos") != -1)
            sin >> config.pct_pos;
        else if (line.find("pct_neg") != -1)
            sin >> config.pct_neg;
    }
}


///////////      MAIN PROGRAM     ///////////
int main(void)
{
    double lim_x = 0;
    //freopen("log_output.txt","w",stdout);
    // Parse  test configuration
    cout << "Loading test config " << endl;
    loadConfig(config);
    cout << "Test config loaded " << endl;
    
    // Initialize log file
    // Log File Initialization
    
    struct tm tm = *localtime(&t);
    char cyclefilename[150];
    int filenamesize = snprintf(cyclefilename,150,\
        "/root/uFloatTests/Pool_Test/Logs/Cycle_Log_%d-%02d-%02d_%02d%02d%02d.txt",\
        tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    pCycleFile = fopen(cyclefilename,"a");
    if(pCycleFile == NULL)
    {
          freopen(cyclefilename,"a",pCycleFile);
    }   
    
    char masterfilename[150];
    filenamesize = snprintf(masterfilename,150,
        "/root/uFloatTests/Pool_Test/log_output.txt");
    pMasterLog = fopen(masterfilename,"a");
    if(pMasterLog== NULL)
    {
          freopen(masterfilename,"a",pMasterLog);
          
          
    }   
   
    
    // Initialize - Start recording data
    //Create sensor thread objects (Only IMU is threaded since it is polled)
    pthread_t IMU_Thread;
    pthread_t BAR30_Thread;
    pthread_t ATMO_Thread;

    //Set encoder period for polling (not 100% necessary, but put in just in case)
    eqep0.set_period(100000000L);

    //Set Motor Gear Ratio
    smc.conv_factor = 1.0/12.0*1.0 / config.gear_ratio*1.0 / 48.0;

    //Start thread objects
    pthread_create(&IMU_Thread, NULL, &IMU_ReadLog, NULL);
    pthread_create(&BAR30_Thread, NULL, &BAR30_ReadLog, NULL);
    pthread_create(&ATMO_Thread, NULL, &BME280_ReadLog, NULL);
    usleep(900000);
    cout << "Sensor Threads Created" << endl;

    fprintf(pMasterLog,"Finding Motor Limits\n");
     // Command motor fully retract, fully extend
    int n = getMotorLimits(&lim_x);
    fprintf(pMasterLog, "Motor Limits Found: %f \n", lim_x);
        
    // Delay to place in pool
    cout << "Waiting for you to put me in water..." << endl;
    cout << "Delay is: " << config.start_delay << endl;
    
    // Add message to log file to check if service running correctly
    /*char msg_chk [100];
    uint8_t msg[100];
    char* msgptr = (char*)msg;
    int nn = snprintf((char *)msg, 100, "%s.%06ld,%f\n", tmbuf, tval_before.tv_usec, cp);
    fwrite(msgptr, sizeof(char), nn, masterLog);
    */
    
    // Start Yoyo test
    //double position_pos = (0.5 +(config.pct_pos/100.0))*lim_x*smc.conv_factor;
    //double position_neg = (0.5 -(config.pct_neg/100.0))*lim_x*smc.conv_factor;
    double position_pos = 3.2;
    double position_neg = 3.0;
    
    cout << "Config Pct Pos " << config.pct_pos << endl;
    cout << "Config Pct Neg " << config.pct_neg << endl;
    cout << "SMS Conv Factor " << smc.conv_factor << endl;
    cout << "Lim_x " << lim_x << endl;
    cout << "Position_neg " << position_neg << endl;
    cout << "Position_pos " << position_pos << endl;
    cout << "Starting Yoyo test..." << endl;
    
    
    fprintf(pMasterLog, "Starting Yoyo Test... \n");
    int ct = yoyoTest(config.num_cycles, config.surface_delay*500, position_pos, position_neg);

    cout << "Test complete. Nice work!!" << endl; 
    fprintf(pMasterLog, "Finished Test. %i \n", ct);
    // End
    return 0;
}