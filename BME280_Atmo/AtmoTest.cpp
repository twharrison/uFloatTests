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
#include <iomanip>
#include <signal.h>
#include <vector>
#include "BME280.h"
#include "Periodic.h"

#include <sys/time.h>
#include <sys/timerfd.h>

//Define used namespaces
using namespace std;

BME280 mySensor;




int main(void)
{
	struct periodic_info info;
	make_periodic(500000, &info);

	mySensor.settings.commInterface = I2C_MODE;
	mySensor.settings.I2CAddress = 0x77;
	mySensor.settings.runMode = 3; //Normal mode
	mySensor.settings.tStandby = 4;
	mySensor.settings.filter = 0;
	mySensor.settings.pressOverSample = 1;
	mySensor.settings.humidOverSample = 1;
	mySensor.settings.tempOverSample = 1;
	
	mySensor.begin();
	
	sleep(1);
	
	std::vector<double> data(3);

	while(1)
	{
		data = mySensor.readAll();
		
		cout << fixed << setprecision(2) << "Temperature is " << data[0] << " degrees C" << endl;
		cout << fixed << setprecision(2) << "Temperature is " << mySensor.TempF(data[0]) << " degrees F" << endl;
		cout << fixed << setprecision(2) << "Humidity is " << data[1] << " %" << endl;
		cout << fixed << setprecision(1) << "Pressure is " << data[2] << " Pa" << endl << endl;
		
		wait_period(&info);
	}
	return 0;
}

	


