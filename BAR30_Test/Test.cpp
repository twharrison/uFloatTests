#include <string.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include "Bar30.h"

MS5837 pres_sensor;

int main()
{
	pres_sensor.init();
	pres_sensor.setFluidDensity(1025); //Sea Water density kg/m^3
	int cnt = 0;
	int buffcnt = 0;
	float depth_buffer, depth_zero_avg;
	float zero_depth = 0.0;
	float pres, temp, alt, depth;
	
	while(1)
	{
		/* Read Pressure Sensor */
		pres_sensor.read_pressure();
		
		/* Calculate values of interest...Pressure (mbar), Temp (deg C), 
		  Altitude (m above mean sea level), Depth (m below surface) */

		pres = pres_sensor.pressure();
		temp = pres_sensor.temperature();
		alt  = pres_sensor.altitude();
		depth = pres_sensor.depth();

		if( buffcnt == 0) { depth_buffer = 0.0;}

		if( buffcnt < 10) { depth_buffer += depth; buffcnt++; }

		if( cnt < 500 )	{zero_depth += depth; cnt++; }
		
		if(cnt == 500) { depth_zero_avg = zero_depth/500; }

		
		if( buffcnt == 10)
		{
			buffcnt = 0;
			depth_buffer = depth_buffer/10;
			printf("Pressure:	%f mbar\n",pres);
			printf("Temperature:	%f degrees C\n",temp);
			//printf("Altitude:	%f m above mean sea level\n",alt);
			printf("Depth:		%f mm below surface\n\n",(depth_buffer-depth_zero_avg)*1000);
		}
		//usleep(10000);
	}


}